// hello_world_main.cpp
// ESP32-P4 (CrowPanel Advance 5") spot-welder DISPLAY/BRIDGE firmware.
//
// Ported from the original Sunton ESP32-S3 build:
//   - Full 5-tab LVGL UI lives in ui.cpp / ui.h (unchanged logic).
//   - This file is the ESP-IDF bootstrap: RGB LCD + STC8 backlight + GT911
//     touch + STM32 UART link, and it bridges between the UI callbacks and the
//     STM32 controller protocol.
//
// Threading model (LVGL is NOT thread-safe):
//   - lvgl_task  : owns ALL LVGL calls (lv_timer_handler + ui_update). Touch
//                  event callbacks (which send STM32 commands) also run here.
//   - stm32_task : polls the STM32 over UART, parses STATUS, and stores the
//                  result into g_state under a mutex. Never touches LVGL.

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_attr.h"      // IRAM_ATTR for the VSYNC ISR callback
#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "lvgl.h"

#include "ui.h"
#include "touch_gt911.h"
#include "wifi_bridge.h"

static const char *TAG = "WELDER_UI";

// ============================================================
//  PINS / CONSTANTS
// ============================================================
#define STM32_UART_NUM      UART_NUM_1
#define STM32_TX_PIN        29
#define STM32_RX_PIN        30
#define STM32_BOOT0_PIN     31
#define BUF_SIZE            8192  // WAVEFORM_DATA lines can be huge (OLD board used 8192)
#define UART_RX_BUF_SIZE    (128 * 1024)  // 128 KB buffer (still generous at 1 Mbaud)
#define STM32_BAUD          1000000  // 1 Mbaud — stable link without HW flow control
// WiFi credentials, captive portal and the Flask TCP bridge (port 8888) are all
// handled in wifi_bridge.cpp (provisioning is done at runtime, stored in NVS).

#define LCD_H_RES           800
#define LCD_V_RES           480
#define LCD_PIXEL_CLOCK_HZ  (15 * 1000 * 1000)  // 15 MHz for extra margin on config-tab scrolling (was 16 MHz — stable main screen but minor scroll glitch)

#define LCD_PCLK             3
#define LCD_DE               2
#define LCD_VSYNC           41
#define LCD_HSYNC           40
#define LCD_B0   8
#define LCD_B1   7
#define LCD_B2   6
#define LCD_B3   5
#define LCD_B4   4
#define LCD_G0  14
#define LCD_G1  13
#define LCD_G2  12
#define LCD_G3  11
#define LCD_G4  10
#define LCD_G5   9
#define LCD_R0  19
#define LCD_R1  18
#define LCD_R2  17
#define LCD_R3  16
#define LCD_R4  15

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_SDA_GPIO        45
#define I2C_SCL_GPIO        46
#define STC8_I2C_ADDR       0x2F

// ============================================================
//  GLOBALS
// ============================================================
static lv_display_t       *lvgl_disp     = NULL;
static esp_lcd_panel_handle_t g_panel_handle = NULL;
// Given by the RGB panel VSYNC ISR; taken by the flush callback so LVGL only
// hands a freshly-rendered framebuffer to the scan-out engine between frames
// (avoid-tearing, matches the Elecrow esp_lvgl_port reference).
static SemaphoreHandle_t  g_vsync_sem    = NULL;
static i2c_master_dev_handle_t stc8_dev_handle = NULL;
static i2c_master_bus_handle_t g_i2c_bus    = NULL;

// Shared welder state (written by stm32_task, read by lvgl_task).
static WelderDisplayState g_state;
static SemaphoreHandle_t  g_state_mtx = NULL;

// STM32 remote-flash coordination (see stm32_task pause point + the
// welder_prep_stm32_flash() hook called by stm32_flash.cpp).
static TaskHandle_t       s_stm32_task_handle = NULL;
static volatile bool      s_stm32_pause_req   = false;  // flasher -> stm32_task
static volatile bool      s_stm32_paused      = false;  // stm32_task -> flasher

// Set true once the first STATUS line from the STM32 has been parsed. Used to
// adopt the controller's flash-persisted settings into the Config UI exactly
// once at boot ("load last settings on boot"). The STM32 is now the source of
// truth for persisted settings (its flash save/load works), so the ESP32 must
// NOT push hard-coded defaults over them.
static volatile bool g_status_received = false;

// Phase-1B telemetry globals declared extern in ui.h. Defined here so the link
// resolves; populated by the STATUS parser.
float weld_v = 0, cap_v = 0;
float weld_v_b = 0, weld_v_a = 0;
float cap_v_b = 0, cap_v_a = 0;
float vcap_b = 0, vcap_a = 0;
float energy_cap_j = 0, energy_weld_j = 0, energy_loss_j = 0;

// Weld counter (persistent in NVS, incremented on WELD_DONE). Flask dashboard
// displays this as "Total Welds" in the System Info panel.
static uint32_t weld_count = 0;

// DISPLAY packet guard: only send after STATUS2 has populated voltage data.
// Matches OLD ESP32 behaviour (ESP32_8048S043C line 4463: "if (hasRawStatus2Data)").
static bool has_status2_data = false;

// ============================================================
//  VOLTAGE DISPLAY SMOOTHER (ported 1:1 from OLD ESP32)
// ============================================================
// Single source of smoothing: a per-channel deadband (5mV) that suppresses
// tiny INA226 noise jitter on the displayed voltage. The SAME smoothed values
// are mirrored to BOTH the local LVGL screen (g_state) AND the Flask web UI
// (DISPLAY packet) — so the device and the dashboard always agree. Raw STATUS2
// is still forwarded separately for graph/calc consumers that need raw data.
//
// Matches ESP32_8048S043C/src/main.cpp VoltageDisplaySmoother (lines 216-265).
class VoltageDisplaySmoother {
   private:
    struct Channel {
        float lastValue;
        bool  initialized;
    };
    static const int MAX_CHANNELS = 8;
    Channel channels[MAX_CHANNELS];
    float   threshold;

   public:
    explicit VoltageDisplaySmoother(float thresh = 0.02f) : threshold(thresh) {
        for (int i = 0; i < MAX_CHANNELS; i++) {
            channels[i].initialized = false;
            channels[i].lastValue   = 0.0f;
        }
    }

    float getDisplayValue(int channel, float rawValue) {
        if (channel < 0 || channel >= MAX_CHANNELS) return rawValue;
        if (!channels[channel].initialized) {
            channels[channel].lastValue   = rawValue;
            channels[channel].initialized = true;
            return rawValue;
        }
        float delta = fabsf(rawValue - channels[channel].lastValue);
        if (delta < threshold) return channels[channel].lastValue;
        channels[channel].lastValue = rawValue;
        return rawValue;
    }
};

enum VoltageChannel {
    CH_VPACK = 0,
    CH_VCAP  = 1,
    CH_CELL1 = 2,
    CH_CELL2 = 3,
    CH_CELL3 = 4,
};

// Single global smoother instance — the "one place" all displayed voltages
// pass through, so the local screen and Flask dashboard stay mirrored.
static VoltageDisplaySmoother g_volt_smoother(0.005f);

// ============================================================
//  STC8 BACKLIGHT / PANEL POWER (I2C @ 0x2F)
// ============================================================
static void stc8_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    i2c_master_transmit(stc8_dev_handle, buf, 2, 1000 / portTICK_PERIOD_MS);
}

static void i2c_and_backlight_init(void)
{
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = I2C_MASTER_NUM;
    bus_config.sda_io_num = (gpio_num_t)I2C_SDA_GPIO;
    bus_config.scl_io_num = (gpio_num_t)I2C_SCL_GPIO;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &g_i2c_bus));

    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = STC8_I2C_ADDR;
    dev_config.scl_speed_hz = 400000;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(g_i2c_bus, &dev_config, &stc8_dev_handle));

    stc8_write(0x18, 1);              // LCD panel power ON
    vTaskDelay(pdMS_TO_TICKS(30));
    stc8_write(0x1B, 1);              // backlight power rail ON
    vTaskDelay(pdMS_TO_TICKS(20));
    stc8_write(0x20, 0);              // PWM brightness 0 (enabled after panel init)
    ESP_LOGI(TAG, "Display power & backlight initialized via STC8 @ 0x%02X", STC8_I2C_ADDR);
}

static void backlight_set(uint8_t pct)
{
    if (!stc8_dev_handle) return;
    stc8_write(0x20, pct);
    ESP_LOGI(TAG, "Backlight PWM set to %d%%", pct);
}

// ============================================================
//  STM32 UART LINK
// ============================================================
static void uart_init(void)
{
    uart_config_t uart_config = {};
    uart_config.baud_rate = STM32_BAUD;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    ESP_ERROR_CHECK(uart_param_config(STM32_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(STM32_UART_NUM, STM32_TX_PIN, STM32_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(STM32_UART_NUM, UART_RX_BUF_SIZE, 0, 0, NULL, 0));
    // Set RX timeout: flush after 10 symbol times of idle (helps partial-packet delivery).
    ESP_ERROR_CHECK(uart_set_rx_timeout(STM32_UART_NUM, 10));
    // Bump UART ISR priority to preempt normal tasks (helps during 30s UI build).
    ESP_ERROR_CHECK(uart_set_rx_full_threshold(STM32_UART_NUM, 120));  // ISR fires at 120 bytes
    ESP_LOGI(TAG, "UART initialized: %d baud on TX=%d RX=%d", STM32_BAUD, STM32_TX_PIN, STM32_RX_PIN);
}

// Send a command line to the STM32 (appends newline).
static void stm_send(const char *cmd)
{
    uart_write_bytes(STM32_UART_NUM, cmd, strlen(cmd));
    uart_write_bytes(STM32_UART_NUM, "\n", 1);
    // Debug-level: the periodic STATUS poll (every ~350 ms) would otherwise
    // flood the console. Raise the log level to DEBUG to see it if needed.
    ESP_LOGD(TAG, "-> STM32: %s", cmd);
}

static void stm_sendf(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    stm_send(buf);
}

// ---- STATUS field extraction helpers ----
static bool extract_float(const char *s, const char *key, float *out)
{
    const char *p = strstr(s, key);
    if (!p) return false;
    p += strlen(key);
    char *end = NULL;
    float v = strtof(p, &end);
    if (end == p) return false;
    *out = v;
    return true;
}

static bool extract_int(const char *s, const char *key, int *out)
{
    const char *p = strstr(s, key);
    if (!p) return false;
    p += strlen(key);
    char *end = NULL;
    long v = strtol(p, &end, 10);
    if (end == p) return false;
    *out = (int)v;
    return true;
}

// ============================================================
//  WELD COUNTER (NVS persistence)
// ============================================================
static void load_weld_count_from_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open("weldcount", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, "count", &weld_count);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "Weld count from NVS: %lu", (unsigned long)weld_count);
}

static void save_weld_count_to_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open("weldcount", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, "count", weld_count);
        nvs_commit(h);
        nvs_close(h);
    }
}

// ============================================================
//  STATUS ENRICHMENT (WiFi + System + Energy + Weld Count)
// ============================================================
// The OLD ESP32 enriched every STATUS line with WiFi/system/energy/weld_count
// fields before sending to Flask. The NEW P4 must do the same — the STM32
// doesn't know about WiFi or the weld counter, so the ESP32 injects them.
//
// CRITICAL: Only enrich STATUS. Forward STATUS2 (battery voltages) RAW so
// Flask's _parse_status2() handler can parse it correctly.
static void enrich_and_broadcast(const char *line)
{
    // STATUS2 (battery voltages): forward raw, no enrichment. Flask has a
    // dedicated parser for this (_parse_status2).
    if (strncmp(line, "STATUS2,", 8) == 0) {
        wifi_bridge_broadcast(line);
        return;
    }

    // Only enrich STATUS packets. Forward everything else as-is (events, etc.).
    if (strncmp(line, "STATUS,", 7) != 0) {
        wifi_bridge_broadcast(line);
        return;
    }

    // Build enriched STATUS by appending WiFi/system/energy/weld_count fields.
    char enriched[2048];
    int len = snprintf(enriched, sizeof(enriched), "%s", line);
    if (len < 0 || len >= (int)sizeof(enriched) - 512) {
        wifi_bridge_broadcast(line);  // fallback if buffer too small
        return;
    }

    // Parse fields from STATUS to compute state/enabled (Flask needs these).
    int armed = 0, welding = 0, ready = 0, chg_en = 0;
    extract_int(line, "armed=", &armed);
    extract_int(line, "welding=", &welding);
    extract_int(line, "ready=", &ready);
    extract_int(line, "chg_en=", &chg_en);

    // Compute derived fields (Flask UI depends on these).
    bool enabled = (armed == 1);
    const char *state;
    if (!enabled) {
        state = "DISABLED";
    } else if (welding == 1) {
        state = "WELDING";
    } else if (chg_en == 1) {
        state = "CHARGING";
    } else {
        state = "IDLE";
    }

    // Add computed fields (Flask expects enabled for arm button, state for status label).
    len += snprintf(enriched + len, sizeof(enriched) - len,
                    ",enabled=%d,state=%s", enabled ? 1 : 0, state);

    // Energy (already parsed from STM32 into globals)
    len += snprintf(enriched + len, sizeof(enriched) - len,
                    ",energy_cap_j=%.3f,energy_weld_j=%.3f,energy_loss_j=%.3f",
                    energy_cap_j, energy_weld_j, energy_loss_j);

    // Weld counter
    len += snprintf(enriched + len, sizeof(enriched) - len,
                    ",weld_count=%lu", (unsigned long)weld_count);

    // WiFi info
    bool wifi_connected = false, wifi_ap_mode = false;
    char wifi_ssid[33] = {0}, wifi_ip[16] = {0};
    int wifi_rssi = 0;
    wifi_bridge_get_info(&wifi_connected, &wifi_ap_mode, wifi_ssid, wifi_ip, &wifi_rssi);
    
    // Sanitize SSID (commas/equals break CSV parser)
    for (char *p = wifi_ssid; *p; p++) {
        if (*p == ',' || *p == '=') *p = ' ';
    }

    len += snprintf(enriched + len, sizeof(enriched) - len,
                    ",wifi_connected=%d,wifi_ap_mode=%d,wifi_ssid=%s,wifi_ip=%s,wifi_rssi=%d",
                    wifi_connected ? 1 : 0, wifi_ap_mode ? 1 : 0,
                    wifi_ssid, wifi_ip, wifi_rssi);

    // System info
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    const char *chip_model = CONFIG_IDF_TARGET;  // "esp32p4"
    
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);  // NULL = default flash chip
    
    len += snprintf(enriched + len, sizeof(enriched) - len,
                    ",fw_version=1.0.0,chip_model=%s,flash_size=%lu,free_heap=%lu,uptime_s=%lu",
                    chip_model,
                    (unsigned long)flash_size,
                    (unsigned long)esp_get_free_heap_size(),
                    (unsigned long)(esp_timer_get_time() / 1000000ULL));

    wifi_bridge_broadcast(enriched);
}

// ============================================================
//  DISPLAY PACKET (smoothed voltages for Flask frontend)
// ============================================================
// Flask's frontend JavaScript updates battery voltage divs from 'display_update'
// events (line 2253 in control.html), NOT from 'status_update'. The OLD ESP32
// sent DISPLAY packets every 1s with smoothed voltages. The NEW P4 must send
// these too, or the Flask UI will never show battery voltages.
static void send_display_packet(void)
{
    char buf[256];
    xSemaphoreTake(g_state_mtx, portMAX_DELAY);

    // g_state already holds the SMOOTHED voltages (parse_status_line runs every
    // value through g_volt_smoother). So this DISPLAY packet mirrors exactly
    // what the local LVGL screen shows — one smoother, one source of truth.
    int len = snprintf(buf, sizeof(buf),
                       "DISPLAY,vpack=%.3f,vcap=%.3f,cell1=%.3f,cell2=%.3f,cell3=%.3f",
                       g_state.pack_voltage,
                       g_state.weld_v,  // vcap = contact voltage (weld_v), smoothed
                       g_state.cell1_v,
                       g_state.cell2_v,
                       g_state.cell3_v);

    xSemaphoreGive(g_state_mtx);

    if (len > 0 && len < (int)sizeof(buf)) {
        wifi_bridge_broadcast(buf);
    }
}

// Parse a STATUS line from the STM32 into g_state (under mutex).
static void parse_weld_done(const char *line)
{
    // Extract last-weld stats from EVENT,WELD_DONE for the Status dashboard.
    xSemaphoreTake(g_state_mtx, portMAX_DELAY);
    float fv; int iv;

    if (extract_int(line, "total_ms=", &iv))
        g_state.last_weld_duration_ms = (uint32_t)iv;
    if (extract_float(line, "peak_a=", &fv))
        g_state.last_weld_peak_a = fv;
    if (extract_float(line, "avg_a=", &fv))
        g_state.last_weld_avg_a = fv;

    // Prefer joule-mode workpiece energy; fall back to time-mode weld energy.
    float jw = NAN, ew = NAN;
    extract_float(line, "joule_workpiece_j=", &jw);
    extract_float(line, "energy_weld_j=", &ew);
    g_state.last_weld_energy_j = !isnan(jw) ? jw : ew;

    // Prefer joule-mode lead loss; fall back to time-mode lead loss.
    float jl = NAN, el = NAN;
    extract_float(line, "joule_loss_j=", &jl);
    extract_float(line, "energy_lead_j=", &el);
    g_state.last_weld_lead_loss_j = !isnan(jl) ? jl : el;

    // Snapshot the control mode and target at the time of the weld.
    g_state.last_weld_was_joule = (g_state.control_mode == 1);
    g_state.last_weld_target_j = g_state.joule_target_j;

    // Accuracy % (joule mode): 100 * delivered / target.
    g_state.last_weld_accuracy_pct =
        (g_state.last_weld_target_j > 0.0f)
            ? (100.0f * g_state.last_weld_energy_j / g_state.last_weld_target_j)
            : 0.0f;

    g_state.last_weld_valid = true;

    xSemaphoreGive(g_state_mtx);

    // Increment persistent weld counter for Flask dashboard "Total Welds".
    weld_count++;
    save_weld_count_to_nvs();
}

static void parse_status_line(const char *line)
{
    if (!strstr(line, "STATUS")) return;

    xSemaphoreTake(g_state_mtx, portMAX_DELAY);
    float fv; int iv;

    // Battery voltages come from STATUS2 (INA226 readings), not STATUS.
    // Apply smoothing so the local screen and Flask DISPLAY match.
    if (strncmp(line, "STATUS2,", 8) == 0) {
        if (extract_float(line, "vpack=", &fv))  g_state.pack_voltage = g_volt_smoother.getDisplayValue(CH_VPACK, fv);
        if (extract_float(line, "cell1=", &fv))  g_state.cell1_v      = g_volt_smoother.getDisplayValue(CH_CELL1, fv);
        if (extract_float(line, "cell2=", &fv))  g_state.cell2_v      = g_volt_smoother.getDisplayValue(CH_CELL2, fv);
        if (extract_float(line, "cell3=", &fv))  g_state.cell3_v      = g_volt_smoother.getDisplayValue(CH_CELL3, fv);
        
        // Charger state (STATUS2 has chg_en and ichg fields).
        if (extract_int(line, "chg_en=", &iv))   g_state.charging = (iv == 1);
        if (extract_float(line, "ichg=", &fv))   g_state.charger_current = g_state.charging ? fv : 0.0f;
        
        // Signal that voltage data is available for DISPLAY packet.
        has_status2_data = true;
        
        xSemaphoreGive(g_state_mtx);
        return;
    }

    // STATUS packet (main telemetry + settings).
    if (extract_float(line, "temp=", &fv))   g_state.temperature  = fv;

    // Contact / cap voltage (prefer canonical weld_v, fall back to vcap).
    if (extract_float(line, "weld_v=", &fv)) { g_state.weld_v = g_volt_smoother.getDisplayValue(CH_VCAP, fv); weld_v = fv; }
    else if (extract_float(line, "vcap=", &fv)) { g_state.weld_v = g_volt_smoother.getDisplayValue(CH_VCAP, fv); weld_v = fv; }
    if (extract_float(line, "cap_v=", &fv))  { g_state.cap_v = fv;  cap_v = fv; }

    if (extract_int(line, "armed=", &iv))    g_state.armed   = (iv == 1);
    if (extract_int(line, "ready=", &iv))    g_state.welding = (iv == 1);

    // Contact/probe hold time (STATUS packet). Each step = 0.5s; the UI shows
    // "Contact X.Xs" from this. Was previously unparsed -> stuck at 0.0s.
    if (extract_int(line, "contact_hold_steps=", &iv)) {
        g_state.contact_hold_steps = (uint8_t)iv;
        // A main STATUS packet carries the persisted controller settings; mark
        // that we've seen one so the UI can adopt them once at boot. (STATUS2
        // battery packets don't include this field, so they won't trip it.)
        g_status_received = true;
    }
    if (extract_int(line, "contact_with_pedal=", &iv))  g_state.contact_with_pedal = (iv == 1);

    // Charger telemetry (STATUS2 packet): chg_en = charger relay on/off,
    // ichg = charge current in A. Mirror the old ESP32 behaviour and only
    // report current while the charger is actually on (else show 0).
    // These were previously unparsed -> charging gauge stuck at 0.
    if (extract_int(line, "chg_en=", &iv)) g_state.charging = (iv == 1);
    if (extract_float(line, "ichg=", &fv)) g_state.charger_current = g_state.charging ? fv : 0.0f;

    if (extract_float(line, "energy_cap_j=", &fv))  { g_state.energy_cap_j = fv;  energy_cap_j = fv; }
    if (extract_float(line, "energy_weld_j=", &fv)) { g_state.energy_weld_j = fv; energy_weld_j = fv; }
    if (extract_float(line, "energy_loss_j=", &fv)) { g_state.energy_loss_j = fv; energy_loss_j = fv; }

    // Recipe sync (so Pulse/Joule tabs reflect the controller).
    if (extract_int(line, "mode=", &iv))            g_state.weld_mode   = (uint8_t)iv;
    if (extract_int(line, "d1=", &iv))              g_state.pulse_d1    = (uint16_t)iv;
    if (extract_int(line, "gap1=", &iv))            g_state.pulse_gap1  = (uint16_t)iv;
    if (extract_int(line, "d2=", &iv))              g_state.pulse_d2    = (uint16_t)iv;
    if (extract_int(line, "gap2=", &iv))            g_state.pulse_gap2  = (uint16_t)iv;
    if (extract_int(line, "d3=", &iv))              g_state.pulse_d3    = (uint16_t)iv;
    if (extract_int(line, "power=", &iv))           g_state.power_pct   = (uint8_t)iv;
    if (extract_int(line, "preheat_en=", &iv))      g_state.preheat_enabled = (iv == 1);
    if (extract_int(line, "preheat_ms=", &iv))      g_state.preheat_ms  = (uint16_t)iv;
    if (extract_int(line, "preheat_pct=", &iv))     g_state.preheat_pct = (uint8_t)iv;
    if (extract_int(line, "preheat_gap_ms=", &iv))  g_state.preheat_gap_ms = (uint16_t)iv;
    if (extract_int(line, "trigger_mode=", &iv))    g_state.trigger_mode = (uint8_t)iv;

    // Joule / dashboard mirror.
    if (extract_int(line, "control_mode=", &iv))    g_state.control_mode = (uint8_t)iv;
    if (extract_float(line, "joule_target_j=", &fv)) g_state.joule_target_j = fv;
    if (extract_int(line, "joule_max_ms=", &iv))    g_state.joule_max_ms = (uint16_t)iv;
    if (extract_float(line, "joule_actual=", &fv))  g_state.joule_actual_j = fv;
    if (extract_float(line, "lead_r_ohm=", &fv))    g_state.lead_resistance_mohm = fv * 1000.0f;

    xSemaphoreGive(g_state_mtx);
}

// ============================================================
//  UI -> STM32 CALLBACKS  (run in lvgl_task context)
// ============================================================
static void cb_arm_toggle(bool arm)              { stm_sendf("ARM,%d", arm ? 1 : 0); }

static void cb_recipe_apply(uint8_t mode, uint16_t d1, uint16_t gap1,
                            uint16_t d2, uint16_t gap2, uint16_t d3,
                            uint8_t power_pct, bool preheat_en,
                            uint16_t preheat_ms, uint8_t preheat_pct,
                            uint16_t preheat_gap_ms)
{
    // STM32 parses SET_PULSE,mode,d1,gap1,d2,gap2,d3 (mode FIRST). Match that
    // order exactly — sending mode last made the STM32 read d1 as the mode
    // (e.g. d1=10 -> mode clamped to 3/triple).
    stm_sendf("SET_PULSE,%u,%u,%u,%u,%u,%u", mode, d1, gap1, d2, gap2, d3);
    stm_sendf("SET_POWER,%u", power_pct);
    stm_sendf("SET_PREHEAT,%u,%u,%u,%u",
              preheat_en ? 1 : 0, preheat_ms, preheat_pct, preheat_gap_ms);
}

static void cb_config_change(const ConfigState &cfg)
{
    // Apply controller-relevant config; UI-only fields stay local. The STM32
    // persists these to its own flash, so a single send is enough — no need to
    // re-assert them (that used to fight the controller's restored values).
    stm_sendf("SET_CONTACT_HOLD,%u", cfg.contact_hold_steps);
    stm_sendf("SET_LEAD_R,%.3f", cfg.lead_resistance_mohm / 1000.0f);
    stm_sendf("SET_CONTACT_WITH_PEDAL,%u", cfg.contact_with_pedal ? 1 : 0);
    // Brightness 0=LOW,1=MED,2=HIGH -> backlight PWM.
    static const uint8_t bl[3] = { 25, 60, 100 };
    backlight_set(bl[cfg.brightness <= 2 ? cfg.brightness : 2]);
}

static void cb_trigger_source(uint8_t mode)      { stm_sendf("SET_TRIGGER_MODE,%u", mode); }
static void cb_weld_count_reset(void)
{
    weld_count = 0;
    save_weld_count_to_nvs();
    stm_send("RESET_WELD_COUNT");  // also tell STM32 (if it has its own counter)
}
static void cb_contact_with_pedal(bool en)       { stm_sendf("SET_CONTACT_WITH_PEDAL,%u", en ? 1 : 0); }
static void cb_calibrate(void)                   { stm_send("CAL_LEAD_START"); }
static void cb_contact_delay(uint8_t steps)      { stm_sendf("SET_CONTACT_HOLD,%u", steps); }

static void cb_joule_apply(bool mode_joule, float target_j, uint16_t max_ms)
{
    stm_sendf("SET_MODE,%d", mode_joule ? 1 : 0);
    stm_sendf("SET_JOULE_TARGET,%.1f", target_j);
    stm_sendf("SET_JOULE_MAX,%u", max_ms);
}

// Maintenance / firmware buttons.
//   - "Reconfigure WiFi": drop STA, raise the AP + captive portal (wifi_bridge).
//   - "Factory Reset": wipe saved WiFi creds from NVS, then reboot (comes back
//     up in portal mode because there are no creds).
//   - OTA / SD firmware update are not ported to the P4 yet (safe stubs).
static void cb_wifi_reconfigure(void) { wifi_bridge_reconfigure(); }
static void cb_restart(void)          { ESP_LOGI(TAG, "Restart requested"); esp_restart(); }
static void cb_factory_reset(void)    { ESP_LOGW(TAG, "Factory reset: wiping WiFi creds + reboot");
                                        wifi_bridge_factory_reset(); vTaskDelay(pdMS_TO_TICKS(200)); esp_restart(); }
static void cb_fw_update_esp32(void)  { ESP_LOGW(TAG, "ESP32 SD firmware update not implemented on P4 yet"); }
static void cb_fw_update_stm32(void)  { ESP_LOGW(TAG, "STM32 SD firmware update not implemented on P4 yet"); }

// ============================================================
//  LVGL PLUMBING
// ============================================================
// RGB panel VSYNC ISR. With no bounce buffer the scan-out engine reads the
// active PSRAM framebuffer directly; on_vsync fires at the start of each frame.
// Wake the flush callback so it only swaps framebuffers on a frame boundary
// (tear-free). Lives in IRAM and touches only IRAM-safe APIs because the
// flash/PSRAM cache may be disabled when it fires (e.g. during NVS/flash writes).
static IRAM_ATTR bool rgb_vsync_cb(esp_lcd_panel_handle_t panel,
                                   const esp_lcd_rgb_panel_event_data_t *edata,
                                   void *user_ctx)
{
    BaseType_t hp_task_woken = pdFALSE;
    if (g_vsync_sem) {
        xSemaphoreGiveFromISR(g_vsync_sem, &hp_task_woken);
    }
    return hp_task_woken == pdTRUE;
}

// Double-buffered DIRECT-mode flush (proven Elecrow esp_lvgl_port pattern).
// In DIRECT mode LVGL renders the invalidated areas straight into one of the two
// driver framebuffers and may call this several times per refresh. Only on the
// LAST area do we hand that framebuffer to the scan-out engine and block until a
// whole frame has streamed, so the engine never latches a buffer mid-draw. For
// non-final areas there is nothing to copy (LVGL already drew into the fb).
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    if (lv_display_flush_is_last(disp)) {
        // px_map is the base of the framebuffer LVGL just rendered. Draw the WHOLE
        // screen so the RGB driver recognises it as one of its own framebuffers
        // and switches the active scan-out buffer to it (zero-copy). Then wait for
        // the next frame-complete event so the swap lands tear-free.
        esp_lcd_panel_draw_bitmap(g_panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, px_map);
        if (g_vsync_sem) {
            xSemaphoreTake(g_vsync_sem, 0);
            xSemaphoreTake(g_vsync_sem, portMAX_DELAY);
        }
    }
    (void)area;
    lv_display_flush_ready(disp);
    // NOTE: never call ESP_LOGx here. This runs in the LVGL render hot path and
    // shares the stdout lock with stm32_task; logging here stalls rendering and
    // contributes to the periodic flicker.
}

static void lv_tick_task(void *arg) { lv_tick_inc(2); }

// ============================================================
//  TASKS
// ============================================================
static void stm32_task(void *arg)
{
    ESP_LOGI(TAG, "STM32 comm task started");
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Load the persistent weld counter from NVS HERE (on this task's internal-
    // RAM stack), NOT in app_main (whose stack is in PSRAM — see app_main note).
    // The 1s delay above gives wifi_bridge's prov_task time to run nvs_flash_init;
    // we also call it ourselves (idempotent: returns ESP_OK if already done) so
    // the read works even if prov_task hasn't reached it yet. nvs_flash_init and
    // nvs_get_* both disable the flash cache, so they MUST run from an internal-
    // RAM stack — which xTaskCreate gives this task.
    {
        esp_err_t r = nvs_flash_init();
        if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            nvs_flash_erase();
            nvs_flash_init();
        }
        load_weld_count_from_nvs();
    }

    // Console-echo throttle. The 115200 console is ~17x slower than the 2 Mbaud
    // STM32 link, so echoing every line floods the console TX buffer, blocks
    // this read loop, overflows the 2 Mbaud RX FIFO (-> dropped bytes /
    // "STATrS" corruption) and starves CPU0 idle (-> flicker + task watchdog).
    // Policy:
    //   - STATUS / STATUS2 telemetry: parsed always, but echoed at most 1 Hz.
    //   - STM32's own per-command "DBG,..." echo: never printed (pure noise).
    //   - Everything else (EVENT / WAVEFORM / RXHEALTH / CAL / errors): printed.
    uint32_t last_status_log_ms = 0;
    uint32_t last_ready_ms = 0;
    uint32_t last_display_ms = 0;

    while (1) {
        // Cooperative pause point for the STM32 firmware-flasher (stm32_flash.cpp).
        // When a remote STM32 update starts, welder_prep_stm32_flash() raises
        // s_stm32_pause_req and waits until we park HERE — a safe spot where this
        // task holds no UART driver call and no stdout/state mutex. That lets the
        // flasher take over UART_NUM_1 (delete+reinstall at 115200 8E1) without
        // racing this poll loop. We never resume (the flasher reboots the ESP32
        // when done), but the loop is written to resume cleanly anyway.
        if (s_stm32_pause_req) {
            s_stm32_paused = true;
            while (s_stm32_pause_req) vTaskDelay(pdMS_TO_TICKS(20));
            s_stm32_paused = false;
        }

        // READY heartbeat: STM32 requires periodic READY,1 to keep system_ready
        // alive (has a 10s timeout). Without this, it stays in ready=0 and
        // refuses all commands (ARM, WELD, CAL_LEAD_START) with NOT_READY/DENY.
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        if (now_ms - last_ready_ms >= 1000) {
            stm_send("READY,1");
            last_ready_ms = now_ms;
        }

        // DISPLAY packet: Flask frontend waits for 'display_update' events to
        // populate battery voltage divs. Send every 1s, but ONLY after STATUS2
        // has populated voltage data (matches OLD ESP32 guard at line 4463).
        if (now_ms - last_display_ms >= 1000) {
            if (has_status2_data) {
                send_display_packet();
                last_display_ms = now_ms;
            }
        }

        stm_send("STATUS");
        int len = uart_read_bytes(STM32_UART_NUM, data, BUF_SIZE - 1, pdMS_TO_TICKS(150));
        if (len > 0) {
            data[len] = '\0';
            // STM32 may send multiple newline-delimited lines per poll.
            char *save = NULL;
            char *line = strtok_r((char *)data, "\r\n", &save);
            while (line) {
                parse_status_line(line);

                if (line[0] != '\0') {
                    // Tolerant prefix match: "STAT" still catches corrupted
                    // "STATrS"/"STATre" variants so they don't slip through as
                    // "events" and re-trigger the flood.
                    bool is_status = (strncmp(line, "STAT", 4) == 0);
                    bool is_dbg    = (strncmp(line, "DBG", 3) == 0);
                    // Filter obvious garbage: boot-time dropped bytes can create
                    // fragments like ",joule_status=..." that don't start with an
                    // uppercase letter. Real messages start with STAT/EVENT/RXHEALTH/CAL.
                    bool looks_valid = isupper((unsigned char)line[0]);

                    // Forward every genuine STM32 line (STATUS + async events)
                    // to the Flask web client over the TCP bridge. STATUS packets
                    // are enriched with WiFi/system/energy/weld_count (Flask needs
                    // these but STM32 doesn't know them). Other packets forwarded
                    // as-is. Skips STM32's DBG echo and obvious garbage.
                    if (!is_dbg && looks_valid) {
                        enrich_and_broadcast(line);
                    }

                    // Calibration progress (CAL_STATUS / CAL_RESULT / CAL_ERROR).
                    // Update the Config tab status line and log for debugging.
                    bool is_cal = (strncmp(line, "CAL_", 4) == 0);
                    if (is_cal && looks_valid) {
                        ui_notify_cal_message(line);
                        ESP_LOGI(TAG, "STM32: %s", line);
                    }

                    // Weld completion: parse last-weld stats for Status dashboard.
                    bool is_weld_done = (strncmp(line, "EVENT,WELD_DONE", 15) == 0);
                    if (is_weld_done && looks_valid) {
                        parse_weld_done(line);
                        ESP_LOGI(TAG, "STM32: %s", line);
                    }

                    if (is_status) {
                        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
                        if (now - last_status_log_ms >= 1000) {
                            last_status_log_ms = now;
                            ESP_LOGI(TAG, "STM32: %s", line);  // 1 Hz heartbeat
                        }
                        // NOTE: no contact-hold "self-heal" here any more. The
                        // STM32 now persists settings in its own flash and is
                        // the source of truth; re-asserting an ESP32-side value
                        // would overwrite what the controller restored at boot.
                    } else if (!is_dbg && !is_cal && !is_weld_done && looks_valid) {
                        ESP_LOGI(TAG, "STM32: %s", line);  // genuine async events
                    }
                }
                line = strtok_r(NULL, "\r\n", &save);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void lvgl_task(void *arg)
{
    ESP_LOGI(TAG, "LVGL task started");
    lv_indev_t *indev = lv_indev_get_next(NULL);  // first (touch) indev
    uint32_t last_ui = 0;

    while (1) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);

        // Tell the UI when a hardware touch is active so it suppresses redraw
        // churn that competes with LVGL touch processing.
        if (indev) {
            lv_indev_state_t st = lv_indev_get_state(indev);
            ui_set_touch_active(st == LV_INDEV_STATE_PRESSED);
        }

        // Push fresh telemetry into the UI ~10 Hz.
        if (now - last_ui >= 100) {
            last_ui = now;
            WelderDisplayState snap;
            xSemaphoreTake(g_state_mtx, portMAX_DELAY);
            snap = g_state;
            xSemaphoreGive(g_state_mtx);
            ui_update(snap);

            // One-shot "load last settings on boot": once the STM32 has sent a
            // real STATUS, adopt its flash-restored controller settings into the
            // Config tab so the UI mirrors what the controller actually loaded.
            // Runs here (lvgl_task) so LVGL stays single-threaded. Honors the
            // Config-tab toggle; default is ON.
            static bool config_synced = false;
            if (!config_synced && g_status_received) {
                config_synced = true;
                if (ui_get_config().load_last_on_boot) {
                    ui_sync_persisted_from_status(snap);
                    ESP_LOGI(TAG, "Adopted STM32 flash settings: contact_hold=%u steps, cwp=%d, lead_r=%.3f mohm",
                             (unsigned)snap.contact_hold_steps,
                             (int)snap.contact_with_pedal,
                             (double)snap.lead_resistance_mohm);
                }
            }
        }

        // Apply any WiFi-UI requests queued by the WiFi provisioning task
        // (prov_task). Those requests CREATE/touch LVGL objects, so they must
        // run here on the LVGL task — never directly on prov_task (doing so
        // corrupted LVGL state and crashed in get_selector_style_prop when the
        // AP setup portal came up after an erase-flash).
        ui_poll_deferred();

        lv_timer_handler();

        // CRITICAL: this delay MUST block for at least 1 RTOS tick so the
        // IDLE0 task on CPU0 gets to run and feed the task watchdog.
        //
        // pdMS_TO_TICKS(5) evaluates to 0 ticks at the default 100 Hz tick rate
        // (5 * 100 / 1000 = 0). vTaskDelay(0) does NOT block — it only yields to
        // equal/higher priority tasks — so the lower-priority IDLE0 task never
        // runs, the watchdog is never fed, and it trips at the WDT timeout.
        // That was the root cause of the repeating "IDLE0 (CPU 0)" watchdog
        // crash. Clamp to a minimum of 1 tick so the task always truly sleeps,
        // regardless of the configured CONFIG_FREERTOS_HZ.
        TickType_t delay_ticks = pdMS_TO_TICKS(5);
        if (delay_ticks == 0) {
            delay_ticks = 1;
        }
        vTaskDelay(delay_ticks);
    }
}

// ============================================================
//  STM32 REMOTE-FLASH PLATFORM HOOK
// ============================================================
// Called by stm32_flash.cpp before it drives the STM32 into its ROM bootloader.
// It (1) cooperatively parks stm32_task at its safe pause point so nothing else
// touches UART_NUM_1, and (2) slows the RGB panel pixel clock to free PSRAM bus
// bandwidth — with XIP-from-PSRAM the continuous scan-out otherwise jitters the
// timing-sensitive 115200 8E1 bootloader UART and breaks the AN3155 handshake
// (this matches the OLD board's proven quietDisplayBus() approach). The flasher
// reboots the ESP32 when done, so there is no "un-prep" — a fresh boot restores
// the normal stm32_task + full-speed pclk + 1 Mbaud app link.
extern "C" void welder_prep_stm32_flash(void)
{
    // 1) Park stm32_task at its cooperative pause point (no UART/mutex held).
    s_stm32_pause_req = true;
    for (int i = 0; i < 100 && !s_stm32_paused; i++) {  // up to ~1 s
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "stm32_task %s for STM32 flash",
             s_stm32_paused ? "parked" : "did NOT park in time (continuing)");

    // 2) Slow the LCD pixel clock to quiet the PSRAM bus during the flash.
    if (g_panel_handle) {
        // 2 MHz matches the OLD board's proven quietDisplayBus() value — slow
        // enough that the RGB scan-out DMA stops starving the bootloader UART.
        esp_lcd_rgb_panel_set_pclk(g_panel_handle, 2 * 1000 * 1000);  // 2 MHz
        esp_lcd_rgb_panel_restart(g_panel_handle);  // re-sync scan-out at new clk
    }
}

// ============================================================
//  APP MAIN
// ============================================================
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "===== ESP32-P4 Welder Display (UI port) =====");

    // BOOT0 LOW first so the STM32 stays in application mode.
    gpio_config_t boot0 = {};
    boot0.pin_bit_mask = (1ULL << STM32_BOOT0_PIN);
    boot0.mode = GPIO_MODE_OUTPUT;
    boot0.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&boot0);
    gpio_set_level((gpio_num_t)STM32_BOOT0_PIN, 0);
    ESP_LOGI(TAG, "GPIO%d (BOOT0) set LOW", STM32_BOOT0_PIN);

    // Shared welder state defaults.
    g_state_mtx = xSemaphoreCreateMutex();
    memset(&g_state, 0, sizeof(g_state));
    g_state.weld_mode = 1;
    g_state.power_pct = 80;
    g_state.trigger_mode = 1;

    i2c_and_backlight_init();
    uart_init();

    lv_init();

    // ---- RGB panel: two driver-managed PSRAM framebuffers, no bounce buffer ----
    esp_lcd_rgb_panel_config_t panel_conf = {};
    panel_conf.clk_src = LCD_CLK_SRC_DEFAULT;
    panel_conf.data_width = 16;
    // DMA burst alignment for PSRAM framebuffer reads. Larger bursts (64 B)
    // mean fewer, more efficient PSRAM transactions, so scan-out reads the
    // framebuffer with less bus arbitration overhead. Without this the default
    // small bursts compete with WiFi/cache traffic and cause line drift.
    panel_conf.dma_burst_size = 64;
    panel_conf.de_gpio_num = (gpio_num_t)LCD_DE;
    panel_conf.pclk_gpio_num = (gpio_num_t)LCD_PCLK;
    panel_conf.vsync_gpio_num = (gpio_num_t)LCD_VSYNC;
    panel_conf.hsync_gpio_num = (gpio_num_t)LCD_HSYNC;
    panel_conf.disp_gpio_num = (gpio_num_t)-1;
    int dpins[16] = { LCD_B0, LCD_B1, LCD_B2, LCD_B3, LCD_B4,
                      LCD_G0, LCD_G1, LCD_G2, LCD_G3, LCD_G4, LCD_G5,
                      LCD_R0, LCD_R1, LCD_R2, LCD_R3, LCD_R4 };
    memcpy(panel_conf.data_gpio_nums, dpins, sizeof(dpins));
    panel_conf.timings.pclk_hz = LCD_PIXEL_CLOCK_HZ;
    panel_conf.timings.h_res = LCD_H_RES;
    panel_conf.timings.v_res = LCD_V_RES;
    // ---- RGB panel timing ----
    // These are the AUTHORITATIVE values from Elecrow's official board config
    // (esp_panel_board_custom_conf.h) for this exact 5" 800x480 CrowPanel:
    //   PCLK 16 MHz, HPW=10 HBP=10 HFP=20, VPW=10 VBP=10 VFP=10, rising-edge PCLK.
    // The previous values (PCLK 18 MHz, tiny HFP/HPW) left too little blanking to
    // refill the RGB FIFO from PSRAM under render load -> glitch lines on scroll
    // and button repaints. If the image is shifted horizontally, tune
    // hsync_back_porch (image moves RIGHT when increased); same idea vertically
    // with vsync_back_porch.
    panel_conf.timings.hsync_back_porch = 10;
    panel_conf.timings.hsync_front_porch = 20;
    panel_conf.timings.hsync_pulse_width = 10;
    panel_conf.timings.vsync_back_porch = 10;
    panel_conf.timings.vsync_front_porch = 10;
    panel_conf.timings.vsync_pulse_width = 10;
    // Official Elecrow config drives data on the rising PCLK edge (active_neg=0).
    panel_conf.timings.flags.pclk_active_neg = 0;
    panel_conf.flags.fb_in_psram = 1;
    // Two full framebuffers in PSRAM. LVGL renders into the back buffer while the
    // panel scans out the front buffer, then we swap on VSYNC (see flush_cb).
    // This is the proper cure for the drift/tearing: the scan-out engine always
    // reads a complete, stable frame.
    panel_conf.num_fbs = 2;
    // NO bounce buffer on this P4 board. Testing on real hardware showed the
    // bounce path (on_frame_buf_complete) actually re-introduced glitching on
    // the main screen here; the correct, fast PSRAM timings (above) plus double
    // framebuffers + VSYNC swap are what keep scan-out stable. With the proper
    // 16 MHz / wider-blanking timings the scan-out DMA refills directly from
    // PSRAM without starving, so the bounce buffer is unnecessary and harmful.
    panel_conf.bounce_buffer_size_px = 0;

    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_conf, &g_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(g_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(g_panel_handle));

    // Binary semaphore + VSYNC callback for tear-free buffer swaps (see
    // flush_cb). With NO bounce buffer the scan-out reads PSRAM directly, so the
    // correct event is on_vsync (fires at the start of each frame) — NOT
    // on_frame_buf_complete (that only fires in bounce-buffer mode).
    g_vsync_sem = xSemaphoreCreateBinary();
    assert(g_vsync_sem);
    esp_lcd_rgb_panel_event_callbacks_t rgb_cbs = {};
    rgb_cbs.on_vsync = rgb_vsync_cb;
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(g_panel_handle,
                                                              &rgb_cbs, NULL));
    // Note: esp_lcd_panel_disp_on_off() is NOT supported for RGB panels
    // (returns ESP_ERR_NOT_SUPPORTED). RGB panels are always on when receiving signals.
    ESP_LOGI(TAG, "RGB LCD initialized: %dx%d", LCD_H_RES, LCD_V_RES);

    vTaskDelay(pdMS_TO_TICKS(100));
    backlight_set(100);

    // ---- LVGL display (DIRECT mode, double framebuffer) ----
    // Use the RGB driver's two PSRAM framebuffers directly as LVGL's draw
    // buffers. In DIRECT mode LVGL draws only the invalidated areas into the back
    // framebuffer (cheap repaints for button presses / scrolling), and the flush
    // callback swaps it in on a frame boundary (see lvgl_flush_cb). Combined with
    // the correct 16 MHz / wide-blanking timings this gives both a stable
    // scan-out under WiFi load and smooth, low-traffic partial redraws (DIRECT
    // redraws far less than FULL → less PSRAM contention).
    void *fb0 = NULL, *fb1 = NULL;
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(g_panel_handle, 2, &fb0, &fb1));
    lvgl_disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_flush_cb(lvgl_disp, lvgl_flush_cb);
    lv_display_set_buffers(lvgl_disp, fb0, fb1,
                           LCD_H_RES * LCD_V_RES * sizeof(lv_color_t),
                           LV_DISPLAY_RENDER_MODE_DIRECT);
    ESP_LOGI(TAG, "LVGL display ready (DIRECT mode, 2 framebuffers, 16MHz timings)");

    // ---- GT911 touch (shares the STC8 I2C bus) ----
    if (!touch_gt911_init(g_i2c_bus, lvgl_disp)) {
        ESP_LOGW(TAG, "Touch init failed - UI will be display-only");
    }

    // ---- Register UI callbacks + build the ported 5-tab UI ----
    ui_set_config_cb(cb_config_change);
    ui_set_trigger_source_cb(cb_trigger_source);
    ui_set_weld_count_reset_cb(cb_weld_count_reset);
    ui_set_contact_with_pedal_cb(cb_contact_with_pedal);
    ui_set_calibrate_cb(cb_calibrate);
    ui_set_joule_apply_cb(cb_joule_apply);
    ui_set_contact_delay_cb(cb_contact_delay);
    ui_set_wifi_reconfigure_cb(cb_wifi_reconfigure);
    ui_set_restart_cb(cb_restart);
    ui_set_factory_reset_cb(cb_factory_reset);
    ui_set_fw_update_esp32_cb(cb_fw_update_esp32);
    ui_set_fw_update_stm32_cb(cb_fw_update_stm32);

    ui_init(cb_arm_toggle, cb_recipe_apply);
    ConfigState defaults = config_defaults();
    ui_load_config(defaults);
    // Set ONLY the local backlight from the UI default. We deliberately do NOT
    // push the controller settings (contact hold, lead R, contact-with-pedal)
    // to the STM32 here: the STM32 now persists those in its own flash and
    // restores them at boot, so pushing ESP32 defaults would clobber the
    // user's saved values. Instead, lvgl_task adopts the controller's restored
    // settings into the Config UI once the first STATUS arrives (see below).
    static const uint8_t bl[3] = { 25, 60, 100 };
    backlight_set(bl[defaults.brightness <= 2 ? defaults.brightness : 2]);
    ui_set_system_info("P4-1.0", "ESP32-P4", 16 * 1024 * 1024, 0);
    ESP_LOGI(TAG, "UI created");

    // ---- LVGL tick timer ----
    esp_timer_create_args_t tick_args = {};
    tick_args.callback = &lv_tick_task;
    tick_args.name = "lv_tick";
    esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, 2000));  // 2 ms

    // ---- Tasks ----
    xTaskCreatePinnedToCore(lvgl_task, "lvgl", 16384, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(stm32_task, "stm32", 6144, NULL, 4,
                            &s_stm32_task_handle, 1);

    // ---- WiFi provisioning + Flask TCP bridge ----
    // Mirrors the old Sunton S3 behaviour: load saved creds from NVS and
    // connect (STA); if none/failed, raise a soft-AP + captive portal + QR
    // code on the Setup tab. Commands arriving from the Flask client on the
    // TCP bridge (port 8888) are forwarded straight to the STM32 via stm_send.
    wifi_bridge_start(stm_send);

    // NOTE: the persistent weld counter is loaded from NVS inside stm32_task()
    // — NOT here. On this board the app_main (main task) stack lives in PSRAM,
    // and any SPI-flash op (nvs_get_*) disables the flash cache, which makes the
    // PSRAM stack unreachable -> assert in esp_task_stack_is_sane_cache_disabled().
    // stm32_task is created with xTaskCreate (internal-RAM stack), so the NVS
    // read is safe there. It is also the task that increments/saves the counter,
    // so there is no cross-task race. (Same reasoning as wifi_bridge's prov_task.)

    ESP_LOGI(TAG, "System initialized - welder UI running");
}
