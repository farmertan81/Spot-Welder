/**
 * Spot Welder – Merged Firmware (Milestone 1) – REFACTORED
 *
 * REFACTOR CHANGES:
 *   - All INA226 sensor code REMOVED (migrated to STM32)
 *   - All charger control code REMOVED (migrated to STM32)
 *   - STATUS2 UART parser added to receive INA226 data from STM32
 *   - UI now displays STM32-sourced VPACK, CELL1-3, ICHG, charger state
 *   - TCP/WiFi bridge KEPT intact
 *
 * I2C COEXISTENCE:
 *   smartdisplay_init() claims I2C_NUM_0 for GT911 touch on GPIO19/GPIO20.
 *   DO NOT call Wire.begin() – it conflicts with smartdisplay's I2C driver.
 *   Wire1 (INA226) bus has been REMOVED – no longer used by ESP32.
 *
 * TOUCH:
 *   The debounced_touchpad_read callback wraps the smartdisplay driver,
 *   adding a state-machine filter with coordinate scaling.  Copied verbatim
 *   from the proven Touch project.
 */

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp32_smartdisplay.h>
#include <esp_wifi.h>
#include <lv_conf.h>
#include <lvgl.h>
#include <math.h>

#include "ui.h"

// =========================
// Sunton hardware mapping
// =========================
#define STM32_TO_ESP32_PIN 17  // ESP RX  <- STM32 TX
#define ESP32_TO_STM32_PIN 18  // ESP TX  -> STM32 RX

// Touch I2C (I2C_NUM_0) – managed by smartdisplay, DO NOT use Wire
#define TOUCH_SDA 19
#define TOUCH_SCL 20

#define BUTTON_PIN 0  // GPIO0 (BOOT button on Sunton board)

HardwareSerial STM32Serial(2);

// =========================
// WiFi / TCP
// =========================
const char* ssid = "Jaime's Wi-Fi Network";
const char* password = "jackaustin";

WiFiServer server(8888);
WiFiClient client;

static uint32_t lastTcpRxMs = 0;
static const uint32_t TCP_IDLE_TIMEOUT_MS = 600000;

// STM32 UART line limits
static const size_t MAX_LINE_LENGTH = 2048;
static const size_t MAX_WAVEFORM_LINE_LENGTH = 8192;

// =========================
// STM32-sourced INA226 data (parsed from STATUS2 packets)
// =========================
static float stm_vpack = 0.0f;
static float stm_vlow = 0.0f;
static float stm_vmid = 0.0f;
static float stm_cell1 = 0.0f;
static float stm_cell2 = 0.0f;
static float stm_cell3 = 0.0f;
static float stm_ichg = 0.0f;

/**
 * DISPLAY-ONLY VOLTAGE SMOOTHER FOR ESP32 TFT/LVGL
 * Prevents screen label flicker while keeping raw telemetry for logic/bridge.
 * Threshold: 0.02V (20mV)
 */
class VoltageDisplaySmoother {
   private:
    struct Channel {
        float lastValue;
        bool initialized;
    };

    static const int MAX_CHANNELS = 8;
    Channel channels[MAX_CHANNELS];
    float threshold;

   public:
    explicit VoltageDisplaySmoother(float thresh = 0.02f) : threshold(thresh) {
        for (int i = 0; i < MAX_CHANNELS; i++) {
            channels[i].initialized = false;
            channels[i].lastValue = 0.0f;
        }
    }

    float getDisplayValue(int channel, float rawValue) {
        if (channel < 0 || channel >= MAX_CHANNELS) return rawValue;

        if (!channels[channel].initialized) {
            channels[channel].lastValue = rawValue;
            channels[channel].initialized = true;
            return rawValue;
        }

        float delta = fabsf(rawValue - channels[channel].lastValue);
        if (delta < threshold) {
            return channels[channel].lastValue;
        }

        channels[channel].lastValue = rawValue;
        return rawValue;
    }
};

enum VoltageChannel {
    CH_VPACK = 0,
    CH_VCAP = 1,
    CH_CELL1 = 2,
    CH_CELL2 = 3,
    CH_CELL3 = 4,
    CH_VLOW = 5,
    CH_VMID = 6
};

// Global instance used only when rendering values on local TFT/LVGL.
static VoltageDisplaySmoother tftSmoother(0.02f);

// Voltage/energy naming migration (Phase 1B):
// - Canonical names from STM32: weld_v/cap_v + *_b/*_a and energy_*_j.
// - Backward compatibility: keep old vcap_b/vcap_a fields alive in bridge
// output.
// - TODO(Phase 2): remove legacy vcap_* compatibility fields once Flask/UI are
// fully migrated.
float weld_v = 0.0f;
float cap_v = 0.0f;
float weld_v_b = 0.0f;
float weld_v_a = 0.0f;
float cap_v_b = 0.0f;
float cap_v_a = 0.0f;
float vcap_b = 0.0f;  // legacy alias for weld_v_b
float vcap_a = 0.0f;  // legacy alias for weld_v_a
float energy_cap_j = 0.0f;
float energy_weld_j = 0.0f;
float energy_loss_j = 0.0f;

static float stm_vcap = 0.0f;         // legacy STATUS alias for weld_v
static float stm_power = 0.0f;        // weld power setting from STATUS
static bool stm_ina_ok = false;       // INA226 health flag from STATUS2
static bool stm_charger_on = false;   // chg_en field from STATUS2
static uint32_t last_status2_ms = 0;  // timestamp of last STATUS2

// =========================
// UI / mirrored settings
// =========================
uint8_t weld_mode = 1;
uint16_t weld_d1 = 5;
uint16_t weld_gap1 = 0;
uint16_t weld_d2 = 0;
uint16_t weld_gap2 = 0;
uint16_t weld_d3 = 0;

uint8_t weld_power_pct = 100;

bool preheat_enabled = false;
uint16_t preheat_ms = 20;
uint8_t preheat_pct = 30;
uint16_t preheat_gap_ms = 3;

static float lead_resistance_ohms = 0.00200f;  // configurable, persisted
static const float LEAD_RESISTANCE_MIN_OHMS = 0.0001f;
static const float LEAD_RESISTANCE_MAX_OHMS = 0.0100f;

// Local ESP32 UI range (mΩ) for Config tab lead resistance control.
static const float UI_LEAD_R_MIN_MOHM = 0.5f;
static const float UI_LEAD_R_MAX_MOHM = 5.0f;
static const float UI_LEAD_R_DEFAULT_MOHM = 2.0f;

// Tracks host-initiated LEAD_R updates that are waiting on STM32 ACK/DENY.
// This makes it explicit that ESP32 must not synthesize ACKs for SET_LEAD_R.
static bool lead_r_update_inflight = false;
static float pending_lead_r_ohms = NAN;

static const uint8_t TRIGGER_MODE_PEDAL = 1;
static const uint8_t TRIGGER_MODE_CONTACT = 2;

uint8_t trigger_mode = TRIGGER_MODE_PEDAL;
uint8_t contact_hold_steps = 2;  // each step = 0.5s
bool contact_with_pedal =
    false;  // require contact detection when pedal trigger

uint32_t weld_count = 0;  // running weld counter (incremented on WELD_DONE)

bool welding_now = false;
unsigned long last_weld_time = 0;
const unsigned long WELD_COOLDOWN = 500;

// =========================
// STM32 mirrored truth
// =========================
static bool stm_armed = false;
static bool stm_ready = false;
static bool stm_synced = false;
static uint32_t stm_last_status_ms = 0;
static float stm_weld_current = 0.0f;
static float temperature_c = NAN;
static uint32_t last_temp_update_ms = 0;

// =========================
// Phase 1 – deferred boot config
// ====
static bool stm32_booted = false;   // set true on "BOOT," line from STM32
static bool config_sent = false;    // set true after sendBootConfig() runs
static uint32_t setup_done_ms = 0;  // millis() at end of setup()
static const uint32_t BOOT_TIMEOUT_MS = 3000;  // fallback if no BOOT msg
static uint32_t config_sent_ms = 0;  // millis() when sendBootConfig() completed
static uint32_t last_stm32_boot_sync_ms = 0;
static const uint32_t STM32_BOOT_RESYNC_DEBOUNCE_MS = 1500;

// Forward declarations
static void sendBootConfig();
static void syncSettingsAfterUiReconnect();
static void syncUiConfigFromRuntime();

// =========================
// READY heartbeat
// =========================
static bool uiConnected = false;
static uint32_t lastReadySentMs = 0;
static const uint32_t READY_PERIOD_MS = 1000;

// STATUS forwarding throttle state (shared by parser + command fast-path).
static uint32_t lastStatusForwardMs = 0;
static const uint32_t STATUS_FORWARD_MIN_INTERVAL_MS = 100;

/* ============================================================
 * TOUCH FILTER / DEBOUNCE  (verbatim from proven Touch project)
 * ============================================================ */

#define TOUCH_MIN_PRESS_MS 30
#define TOUCH_DEBOUNCE_MS 80
#define TOUCH_RELEASE_DEBOUNCE_MS 40
#define TOUCH_SMOOTH_FACTOR 0.3f

#define GT911_RAW_X_MAX 470
#define GT911_RAW_Y_MAX 265
#define DISPLAY_RES_X 800
#define DISPLAY_RES_Y 480

#define TOUCH_MAX_JUMP_PX 100

enum TouchState {
    TOUCH_IDLE,
    TOUCH_PENDING_PRESS,
    TOUCH_PRESSED,
    TOUCH_PENDING_RELEASE
};

static struct {
    TouchState state;
    uint32_t state_enter_ms;
    uint32_t last_press_ms;
    float smooth_x, smooth_y;
    int16_t last_raw_x, last_raw_y;
    bool has_prev_coords;
    uint32_t tap_count;
    uint32_t touch_reads;
    uint32_t rejected_noise;
} touch_filter = {};

static lv_indev_read_cb_t original_read_cb = nullptr;

static void debounced_touchpad_read(lv_indev_t* indev, lv_indev_data_t* data) {
    touch_filter.touch_reads++;

    lv_indev_data_t raw_data = {};
    raw_data.state = LV_INDEV_STATE_RELEASED;
    raw_data.point.x = 0;
    raw_data.point.y = 0;
    raw_data.continue_reading = false;

    if (original_read_cb) {
        original_read_cb(indev, &raw_data);
    }

    bool touched = (raw_data.state == LV_INDEV_STATE_PRESSED);
    int16_t raw_x = raw_data.point.x;
    int16_t raw_y = raw_data.point.y;

    if (touched) {
        touch_filter.last_raw_x = raw_x;
        touch_filter.last_raw_y = raw_y;
    }

    int16_t tx = (int16_t)((int32_t)raw_x * DISPLAY_RES_X / GT911_RAW_X_MAX);
    int16_t ty = (int16_t)((int32_t)raw_y * DISPLAY_RES_Y / GT911_RAW_Y_MAX);

    if (tx < 0) tx = 0;
    if (ty < 0) ty = 0;
    if (tx >= DISPLAY_RES_X) tx = DISPLAY_RES_X - 1;
    if (ty >= DISPLAY_RES_Y) ty = DISPLAY_RES_Y - 1;

    uint32_t now = millis();

    switch (touch_filter.state) {
        case TOUCH_IDLE:
            if (touched) {
                touch_filter.smooth_x = (float)tx;
                touch_filter.smooth_y = (float)ty;
                touch_filter.state = TOUCH_PENDING_PRESS;
                touch_filter.state_enter_ms = now;
            }
            data->state = LV_INDEV_STATE_RELEASED;
            data->point.x = (int16_t)touch_filter.smooth_x;
            data->point.y = (int16_t)touch_filter.smooth_y;
            break;

        case TOUCH_PENDING_PRESS:
            if (!touched) {
                touch_filter.state = TOUCH_IDLE;
                touch_filter.state_enter_ms = now;
                touch_filter.rejected_noise++;
                data->state = LV_INDEV_STATE_RELEASED;
                data->point.x = (int16_t)touch_filter.smooth_x;
                data->point.y = (int16_t)touch_filter.smooth_y;
            } else {
                float alpha = TOUCH_SMOOTH_FACTOR;
                touch_filter.smooth_x =
                    alpha * (float)tx + (1.0f - alpha) * touch_filter.smooth_x;
                touch_filter.smooth_y =
                    alpha * (float)ty + (1.0f - alpha) * touch_filter.smooth_y;

                if ((now - touch_filter.state_enter_ms) >= TOUCH_MIN_PRESS_MS) {
                    if ((now - touch_filter.last_press_ms) >=
                        TOUCH_DEBOUNCE_MS) {
                        touch_filter.state = TOUCH_PRESSED;
                        touch_filter.state_enter_ms = now;
                        touch_filter.last_press_ms = now;
                        touch_filter.tap_count++;

                        data->point.x = (int16_t)touch_filter.smooth_x;
                        data->point.y = (int16_t)touch_filter.smooth_y;
                        data->state = LV_INDEV_STATE_PRESSED;
                    } else {
                        touch_filter.state = TOUCH_IDLE;
                        touch_filter.rejected_noise++;
                        data->state = LV_INDEV_STATE_RELEASED;
                        data->point.x = (int16_t)touch_filter.smooth_x;
                        data->point.y = (int16_t)touch_filter.smooth_y;
                    }
                } else {
                    data->state = LV_INDEV_STATE_RELEASED;
                    data->point.x = (int16_t)touch_filter.smooth_x;
                    data->point.y = (int16_t)touch_filter.smooth_y;
                }
            }
            break;

        case TOUCH_PRESSED:
            if (!touched) {
                touch_filter.state = TOUCH_PENDING_RELEASE;
                touch_filter.state_enter_ms = now;
                data->point.x = (int16_t)touch_filter.smooth_x;
                data->point.y = (int16_t)touch_filter.smooth_y;
                data->state = LV_INDEV_STATE_PRESSED;
            } else {
                float alpha = TOUCH_SMOOTH_FACTOR;
                float new_x =
                    alpha * (float)tx + (1.0f - alpha) * touch_filter.smooth_x;
                float new_y =
                    alpha * (float)ty + (1.0f - alpha) * touch_filter.smooth_y;

                if (TOUCH_MAX_JUMP_PX > 0) {
                    float dx = new_x - touch_filter.smooth_x;
                    float dy = new_y - touch_filter.smooth_y;
                    float dist = sqrtf(dx * dx + dy * dy);
                    if (dist > TOUCH_MAX_JUMP_PX) {
                        new_x = (float)tx;
                        new_y = (float)ty;
                    }
                }

                touch_filter.smooth_x = new_x;
                touch_filter.smooth_y = new_y;

                if (touch_filter.smooth_x < 0) touch_filter.smooth_x = 0;
                if (touch_filter.smooth_y < 0) touch_filter.smooth_y = 0;
                if (touch_filter.smooth_x > 799) touch_filter.smooth_x = 799;
                if (touch_filter.smooth_y > 479) touch_filter.smooth_y = 479;

                data->point.x = (int16_t)touch_filter.smooth_x;
                data->point.y = (int16_t)touch_filter.smooth_y;
                data->state = LV_INDEV_STATE_PRESSED;
            }
            break;

        case TOUCH_PENDING_RELEASE:
            if (touched) {
                touch_filter.state = TOUCH_PRESSED;
                touch_filter.state_enter_ms = now;

                float alpha = TOUCH_SMOOTH_FACTOR;
                touch_filter.smooth_x =
                    alpha * (float)tx + (1.0f - alpha) * touch_filter.smooth_x;
                touch_filter.smooth_y =
                    alpha * (float)ty + (1.0f - alpha) * touch_filter.smooth_y;

                data->point.x = (int16_t)touch_filter.smooth_x;
                data->point.y = (int16_t)touch_filter.smooth_y;
                data->state = LV_INDEV_STATE_PRESSED;
            } else if ((now - touch_filter.state_enter_ms) >=
                       TOUCH_RELEASE_DEBOUNCE_MS) {
                touch_filter.state = TOUCH_IDLE;
                touch_filter.state_enter_ms = now;
                touch_filter.has_prev_coords = false;
                data->state = LV_INDEV_STATE_RELEASED;
                data->point.x = (int16_t)touch_filter.smooth_x;
                data->point.y = (int16_t)touch_filter.smooth_y;
            } else {
                data->point.x = (int16_t)touch_filter.smooth_x;
                data->point.y = (int16_t)touch_filter.smooth_y;
                data->state = LV_INDEV_STATE_PRESSED;
            }
            break;
    }

    data->continue_reading = false;
}

// =========================
// Forward declarations
// =========================
void sendToPi(const String& msg);
void forwardToStm32(const String& line);
void updateScreenDisplay();
void pollStm32Uart();
void processCommand(String cmd);
void ensureWiFiAndServer();
void setUiConnected(bool connected);
void serviceReadyHeartbeat();
void requestStm32Status();

void handleButton();

String buildStatus();

static void save_recipe_to_nvs();
static void save_lead_resistance_to_nvs();
static void load_lead_resistance_from_nvs();
static void sendLeadResistanceToStm32(float ohms);

// =========================
// Helpers
// =========================
static bool extractFieldValue(const String& line, const String& key,
                              String& out) {
    int idx = line.indexOf(key);
    if (idx < 0) return false;

    int start = idx + key.length();
    int end = line.indexOf(',', start);
    if (end < 0) end = line.length();

    out = line.substring(start, end);
    out.trim();
    return out.length() > 0;
}

static bool extractIntField(const String& line, const String& key, int& out) {
    String s;
    if (!extractFieldValue(line, key, s)) return false;
    out = s.toInt();
    return true;
}

static bool extractFloatField(const String& line, const String& key,
                              float& out) {
    String s;
    if (!extractFieldValue(line, key, s)) return false;
    if (s == "ERR" || s == "NaN") return false;
    out = s.toFloat();
    return true;
}

static inline bool isNoisyKeepaliveLine(const String& s) {
    return s.startsWith("ACK,READY");
}

// =========================
// Status builder
// =========================
/**
 * buildStatus() - System state packet
 *
 * NOTE: Voltage telemetry is NOT included here.
 * All voltages are sent via STATUS2 only to ensure
 * Flask web UI and ESP32 TFT display stay in sync.
 *
 * Including voltages in both STATUS and STATUS2 caused
 * display mismatch due to cached values confusing the
 * 20mV display smoother.
 */
String buildStatus() {
    bool enabled = stm_armed;

    String state;
    if (!enabled) {
        state = "DISABLED";
    } else if (welding_now) {
        state = "WELDING";
    } else if (stm_charger_on) {
        state = "CHARGING";
    } else {
        state = "IDLE";
    }

    bool temp_stale = (millis() - last_temp_update_ms) > 5000;
    String t_str = (!temp_stale && isfinite(temperature_c))
                       ? String(temperature_c, 1)
                       : "ERR";

    unsigned long now = millis();
    long cooldown_ms = (long)(WELD_COOLDOWN - (now - last_weld_time));
    if (cooldown_ms < 0) cooldown_ms = 0;

    float iweld = (fabs(stm_weld_current) < 0.002f) ? 0.0f : stm_weld_current;

    String status = "STATUS";
    status += ",enabled=" + String(enabled ? 1 : 0);
    status += ",armed=" + String(stm_armed ? 1 : 0);
    status += ",ready=" + String(stm_ready ? 1 : 0);
    status += ",welding=" + String(welding_now ? 1 : 0);
    status += ",state=" + state;
    status += ",iweld=" + String(iweld, 3);
    status += ",energy_cap_j=" + String(energy_cap_j, 3);
    status += ",energy_weld_j=" + String(energy_weld_j, 3);
    status += ",energy_loss_j=" + String(energy_loss_j, 3);
    status += ",temp=" + t_str;
    status += ",ina_ok=" + String(stm_ina_ok ? 1 : 0);
    status += ",charger_on=" + String(stm_charger_on ? 1 : 0);
    status += ",cooldown_ms=" + String(cooldown_ms);
    status += ",pulse_ms=" + String(weld_d1);
    status += ",power_pct=" + String(weld_power_pct);
    status += ",preheat_en=" + String(preheat_enabled ? 1 : 0);
    status += ",preheat_ms=" + String(preheat_ms);
    status += ",preheat_pct=" + String(preheat_pct);
    status += ",preheat_gap_ms=" + String(preheat_gap_ms);
    status += ",lead_r_ohm=" + String(lead_resistance_ohms, 6);
    status += ",lead_r_mohm=" + String(lead_resistance_ohms * 1000.0f, 3);
    status += ",mode=" + String(weld_mode);
    status += ",d1=" + String(weld_d1);
    status += ",gap1=" + String(weld_gap1);
    status += ",d2=" + String(weld_d2);
    status += ",gap2=" + String(weld_gap2);
    status += ",d3=" + String(weld_d3);
    status += ",trigger_mode=" + String(trigger_mode);
    status += ",contact_hold_steps=" + String(contact_hold_steps);
    status += ",contact_with_pedal=" + String(contact_with_pedal ? 1 : 0);
    status += ",weld_count=" + String(weld_count);

    return status;
}

void updateScreenDisplay() {
    // RAW telemetry path for logic/calculations/bridge packets.
    const float raw_vpack = stm_vpack;
    const float raw_cell1 = stm_cell1;
    const float raw_cell2 = stm_cell2;
    const float raw_cell3 = stm_cell3;
    const float raw_cap_v = cap_v;

    WelderDisplayState ds;

    // DISPLAY-ONLY smoothing path (local TFT/LVGL labels):
    ds.pack_voltage = tftSmoother.getDisplayValue(CH_VPACK, raw_vpack);
    ds.cell1_v = tftSmoother.getDisplayValue(CH_CELL1, raw_cell1);
    ds.cell2_v = tftSmoother.getDisplayValue(CH_CELL2, raw_cell2);
    ds.cell3_v = tftSmoother.getDisplayValue(CH_CELL3, raw_cell3);
    ds.cap_v = tftSmoother.getDisplayValue(CH_VCAP, raw_cap_v);

    ds.temperature = temperature_c;
    ds.charger_current = stm_charger_on ? stm_ichg : 0.0f;
    ds.weld_v = weld_v;
    ds.weld_v_b = weld_v_b;
    ds.weld_v_a = weld_v_a;
    ds.cap_v_b = cap_v_b;
    ds.cap_v_a = cap_v_a;

    // Keep derived math raw/authoritative.
    ds.weld_v_drop = weld_v_b - weld_v_a;
    ds.cap_v_drop = cap_v_b - cap_v_a;

    ds.energy_cap_j = energy_cap_j;
    ds.energy_weld_j = energy_weld_j;
    ds.energy_loss_j = energy_loss_j;
    ds.armed = stm_armed;
    ds.welding = welding_now;
    ds.charging = stm_charger_on;
    ds.weld_count = weld_count;
    ds.weld_mode = weld_mode;
    ds.pulse_d1 = weld_d1;
    ds.pulse_gap1 = weld_gap1;
    ds.pulse_d2 = weld_d2;
    ds.pulse_gap2 = weld_gap2;
    ds.pulse_d3 = weld_d3;
    ds.power_pct = weld_power_pct;
    ds.preheat_enabled = preheat_enabled;
    ds.preheat_ms = preheat_ms;
    ds.preheat_pct = preheat_pct;
    ds.preheat_gap_ms = preheat_gap_ms;
    ds.trigger_mode = trigger_mode;
    ds.contact_hold_steps = contact_hold_steps;
    ui_update(ds);
}

// =========================
// TCP / UART helpers
// =========================
void sendToPi(const String& msg) {
    if (client && client.connected()) {
        client.print(msg);
        client.print("\n");

        lastTcpRxMs = millis();  // ✅ FIX: treat TX as activity

        Serial.printf("[TCP] TX: %s\n", msg.c_str());
    }
}
void forwardToStm32(const String& line) {
    String out = line;
    out.trim();
    if (out.length() == 0) return;

    STM32Serial.print(out);
    STM32Serial.print("\r\n");
    STM32Serial.flush();

    if (!out.startsWith("READY,")) {
        Serial.printf("[UART->STM32] %s\n", out.c_str());
    }
}

void requestStm32Status() {
    if (!config_sent) return;  // gated until boot config sent
    forwardToStm32("STATUS");
}

// =========================
// STATUS2 parser (replaces old CHGSTAT parser)
// =========================
static void parseStatus2(const String& line) {
    int iv;
    float fv;

    // Extract INA226 health flag
    if (extractIntField(line, "ina_ok=", iv)) stm_ina_ok = (iv == 1);

    // Extract charger enable state
    if (extractIntField(line, "chg_en=", iv)) stm_charger_on = (iv == 1);

    // CHANGED: field name match for STM32 output
    if (extractFloatField(line, "vpack=", fv) ||
        extractFloatField(line, "vpack_ina=", fv)) {
        stm_vpack = fv;
    }

    if (extractFloatField(line, "vlow=", fv)) stm_vlow = fv;
    if (extractFloatField(line, "vmid=", fv)) stm_vmid = fv;

    // Extract individual cell voltages
    if (extractFloatField(line, "cell1=", fv)) stm_cell1 = fv;
    if (extractFloatField(line, "cell2=", fv)) stm_cell2 = fv;
    if (extractFloatField(line, "cell3=", fv)) stm_cell3 = fv;

    // Extract charge current
    if (extractFloatField(line, "ichg=", fv)) stm_ichg = fv;

    last_status2_ms = millis();
}

// =========================
// Front button
// =========================
void handleButton() {
    if (!config_sent) return;  // gated until boot config sent
    static unsigned long press_start = 0;
    static unsigned long last_release = 0;
    static bool was_pressed = false;
    static bool waiting_for_double = false;
    static unsigned long double_tap_window = 0;

    bool is_pressed = (digitalRead(BUTTON_PIN) == LOW);

    if (is_pressed && !was_pressed) {
        press_start = millis();
    } else if (!is_pressed && was_pressed) {
        unsigned long press_duration = millis() - press_start;
        unsigned long time_since_last = millis() - last_release;

        if (press_duration >= 2000) {
            Serial.println("Front button long press -> deep sleep");
            waiting_for_double = false;
            esp_deep_sleep_start();
        } else if (press_duration >= 50) {
            if (waiting_for_double && time_since_last <= 500) {
                Serial.println("Front button double tap -> ESP restart");
                delay(100);
                ESP.restart();
            } else {
                waiting_for_double = true;
                double_tap_window = millis();
            }
        }

        last_release = millis();
    }

    if (waiting_for_double && (millis() - double_tap_window > 500)) {
        bool target = !stm_armed;
        forwardToStm32(String("ARM,") + (target ? "1" : "0"));
        waiting_for_double = false;
    }

    was_pressed = is_pressed;
}

// =========================
// STM32 RX parser
// =========================
void pollStm32Uart() {
    static String stmLine;
    static bool stmLineOverflow = false;
    static uint32_t droppedUartLines = 0;

    while (STM32Serial.available()) {
        char ch = (char)STM32Serial.read();

        if (ch == '\r') continue;

        if (ch == '\n') {
            if (stmLineOverflow) {
                droppedUartLines++;
                Serial.printf(
                    "[UART] Dropped overlength STM32 line (%u chars buffered, "
                    "drops=%lu)\n",
                    (unsigned int)stmLine.length(),
                    (unsigned long)droppedUartLines);
                stmLine = "";
                stmLineOverflow = false;
                continue;
            }

            // Strip non-printable chars from UART line
            {
                String cleaned;
                cleaned.reserve(stmLine.length());
                for (unsigned int i = 0; i < stmLine.length(); i++) {
                    char c = stmLine[i];
                    if ((c >= 32 && c <= 126) || c == '\t') {
                        cleaned += c;
                    }
                }
                cleaned.trim();
                stmLine = cleaned;
            }

            if (stmLine.length() > 0) {
                // BOOT message handler – trigger safety re-sync
                if (stmLine.startsWith("BOOT,") || stmLine == "BOOT") {
                    Serial.printf("[Boot] STM32 boot detected: %s\n",
                                  stmLine.c_str());
                    stm32_booted = true;

                    uint32_t now = millis();
                    bool resync_allowed = (now - last_stm32_boot_sync_ms) >
                                          STM32_BOOT_RESYNC_DEBOUNCE_MS;

                    if (resync_allowed) {
                        if (!config_sent) {
                            sendBootConfig();
                        } else {
                            Serial.println(
                                "[Boot] STM32 reboot detected after initial "
                                "sync -> "
                                "re-syncing current live settings/state");
                            syncSettingsAfterUiReconnect();
                        }
                        last_stm32_boot_sync_ms = now;
                    } else {
                        Serial.println(
                            "[Boot] Ignoring duplicate BOOT message "
                            "(debounced)");
                    }

                    stmLine = "";
                    continue;
                }

                // STATUS2 packet from STM32 (INA226 data + charger state)
                if (stmLine.startsWith("STATUS2,")) {
                    parseStatus2(stmLine);
                    // Log to serial so STATUS2 is visible in monitor
                    Serial.printf("[STM32->UART] %s\n", stmLine.c_str());
                    // Forward to TCP client
                    sendToPi(stmLine);
                    stmLine = "";
                    continue;
                }

                if (!isNoisyKeepaliveLine(stmLine)) {
                    Serial.printf("[STM32->UART] %s\n", stmLine.c_str());
                }

                if (stmLine.startsWith("ACK,ARM,")) {
                    int v = stmLine.substring(8).toInt();
                    stm_armed = (v == 1);
                    stm_synced = true;
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("ACK,READY")) {
                    int v = stm_ready ? 1 : 0;
                    int comma = stmLine.lastIndexOf(',');
                    if (comma >= 0 && (comma + 1) < (int)stmLine.length()) {
                        v = stmLine.substring(comma + 1).toInt();
                    }
                    stm_ready = (v == 1);
                    sendToPi(stmLine);

                } else if (stmLine.startsWith("ACK,SET_PULSE")) {
                    sendToPi(stmLine);
                    sendToPi(String("ACK,SET_PULSE,mode=") + String(weld_mode) +
                             ",d1=" + String(weld_d1) + ",gap1=" +
                             String(weld_gap1) + ",d2=" + String(weld_d2) +
                             ",gap2=" + String(weld_gap2) +
                             ",d3=" + String(weld_d3));
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("ACK,SET_POWER")) {
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("ACK,SET_PREHEAT")) {
                    sendToPi(stmLine);
                    sendToPi(String("ACK,SET_PREHEAT,en=") +
                             String(preheat_enabled ? 1 : 0) +
                             ",ms=" + String(preheat_ms) +
                             ",pct=" + String(preheat_pct) +
                             ",gap=" + String(preheat_gap_ms));
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("ACK,SET_TRIGGER_MODE")) {
                    int iv = 0;
                    if (extractIntField(stmLine, "mode=", iv)) {
                        trigger_mode = (uint8_t)iv;
                    }
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("ACK,SET_CONTACT_HOLD")) {
                    int iv = 0;
                    if (extractIntField(stmLine, "steps=", iv)) {
                        contact_hold_steps = (uint8_t)iv;
                    }
                    syncUiConfigFromRuntime();
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("ACK,SET_CONTACT_WITH_PEDAL")) {
                    int v =
                        stmLine.substring(strlen("ACK,SET_CONTACT_WITH_PEDAL,"))
                            .toInt();
                    contact_with_pedal = (v == 1);
                    syncUiConfigFromRuntime();
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("ACK,LEAD_R")) {
                    float fv = NAN;
                    if (extractFloatField(stmLine, "ohm=", fv) &&
                        isfinite(fv)) {
                        lead_resistance_ohms = fv;
                        save_lead_resistance_to_nvs();
                    }
                    lead_r_update_inflight = false;
                    pending_lead_r_ohms = NAN;
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("DENY,LEAD_R")) {
                    // STM32 rejected LEAD_R update, so clear pending state and
                    // forward authoritative DENY upstream.
                    lead_r_update_inflight = false;
                    pending_lead_r_ohms = NAN;
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("STATUS,")) {
                    int iv = 0;
                    float fv = NAN;

                    if (extractIntField(stmLine, "armed=", iv)) {
                        stm_armed = (iv == 1);
                    }

                    if (extractFloatField(stmLine, "vpack=", fv)) {
                        stm_vpack = fv;
                    }

                    if (extractIntField(stmLine, "ready=", iv)) {
                        stm_ready = (iv == 1);
                    }

                    // Voltage field migration parser:
                    // Prefer new canonical fields when present.
                    // Legacy fields are still parsed as fallback for old STM32
                    // FW.
                    bool got_weld_v = false;
                    if (extractFloatField(stmLine, "weld_v=", fv)) {
                        weld_v = fv;
                        stm_vcap = fv;  // legacy STATUS vcap alias
                        got_weld_v = true;
                    }
                    if (!got_weld_v &&
                        extractFloatField(stmLine, "vcap=", fv)) {
                        weld_v = fv;
                        stm_vcap = fv;
                    }

                    if (extractFloatField(stmLine, "cap_v=", fv)) {
                        cap_v = fv;
                    }

                    // STATUS may carry before/after points in some firmware
                    // variants.
                    if (extractFloatField(stmLine, "weld_v_b=", fv)) {
                        weld_v_b = fv;
                        vcap_b = fv;
                    } else if (extractFloatField(stmLine, "vcap_b=", fv)) {
                        weld_v_b = fv;
                        vcap_b = fv;
                    }
                    if (extractFloatField(stmLine, "weld_v_a=", fv)) {
                        weld_v_a = fv;
                        vcap_a = fv;
                    } else if (extractFloatField(stmLine, "vcap_a=", fv)) {
                        weld_v_a = fv;
                        vcap_a = fv;
                    }
                    if (extractFloatField(stmLine, "cap_v_b=", fv))
                        cap_v_b = fv;
                    if (extractFloatField(stmLine, "cap_v_a=", fv))
                        cap_v_a = fv;

                    if (extractFloatField(stmLine, "energy_cap_j=", fv))
                        energy_cap_j = fv;
                    if (extractFloatField(stmLine, "energy_weld_j=", fv))
                        energy_weld_j = fv;
                    if (extractFloatField(stmLine, "energy_loss_j=", fv))
                        energy_loss_j = fv;

                    // ====
                    // RECIPE SYNC FROM STM32 STATUS
                    // ====
                    // ESP32 is the source of truth for recipe settings on boot.
                    // After boot config is sent, we suppress STM32 STATUS
                    // from overwriting recipe globals for a grace period
                    // (to let STM32 process our SET_* commands).
                    // After the grace period, we re-enable sync so the
                    // ESP32 stays in sync with STM32 for runtime changes.

                    {
                        // Grace period: don't let STATUS overwrite recipe
                        // for BOOT_GRACE_MS after boot config was sent.
                        // This prevents stale STM32 defaults from clobbering
                        // the NVS-loaded recipe we just pushed.
                        static const uint32_t BOOT_GRACE_MS = 2000;
                        bool allow_recipe_sync =
                            !config_sent ||
                            (config_sent_ms > 0 &&
                             (millis() - config_sent_ms) > BOOT_GRACE_MS);

                        if (allow_recipe_sync) {
                            // Pulse / timing
                            if (extractIntField(stmLine, "d1=", iv))
                                weld_d1 = (uint16_t)iv;
                            if (extractIntField(stmLine, "gap1=", iv))
                                weld_gap1 = (uint16_t)iv;
                            if (extractIntField(stmLine, "d2=", iv))
                                weld_d2 = (uint16_t)iv;
                            if (extractIntField(stmLine, "gap2=", iv))
                                weld_gap2 = (uint16_t)iv;
                            if (extractIntField(stmLine, "d3=", iv))
                                weld_d3 = (uint16_t)iv;

                            // Power
                            if (extractFloatField(stmLine, "power=", fv)) {
                                stm_power = fv;
                                weld_power_pct = (uint8_t)fv;
                            }

                            // Configurable lead resistance (runtime sync from
                            // STM32)
                            if (extractFloatField(stmLine, "lead_r_ohm=", fv))
                                lead_resistance_ohms = fv;

                            // Preheat
                            if (extractIntField(stmLine, "preheat_en=", iv))
                                preheat_enabled = (iv == 1);
                            if (extractIntField(stmLine, "preheat_ms=", iv))
                                preheat_ms = (uint16_t)iv;
                            if (extractIntField(stmLine, "preheat_pct=", iv))
                                preheat_pct = (uint8_t)iv;
                            if (extractIntField(stmLine, "preheat_gap_ms=", iv))
                                preheat_gap_ms = (uint16_t)iv;

                            // Trigger / behavior
                            if (extractIntField(stmLine, "trigger_mode=", iv))
                                trigger_mode = (uint8_t)iv;
                            if (extractIntField(stmLine,
                                                "contact_hold_steps=", iv))
                                contact_hold_steps = (uint8_t)iv;
                            if (extractIntField(stmLine,
                                                "contact_with_pedal=", iv))
                                contact_with_pedal = (iv == 1);
                        } else {
                            // During grace period, still parse power for
                            // stm_power (read-only display) but don't
                            // overwrite recipe globals
                            if (extractFloatField(stmLine, "power=", fv))
                                stm_power = fv;
                            if (extractFloatField(stmLine, "lead_r_ohm=", fv))
                                lead_resistance_ohms = fv;
                            Serial.println(
                                "[SYNC] Recipe sync suppressed (boot grace "
                                "period)");
                        }
                    }

                    // ====
                    // EXISTING CODE CONTINUES
                    // ====

                    if (extractFloatField(stmLine, "temp=", fv)) {
                        temperature_c = fv;
                        last_temp_update_ms = millis();
                    }

                    if (extractFloatField(stmLine, "iwfilt=", fv)) {
                        stm_weld_current = fv;
                    } else if (extractFloatField(stmLine, "iweld_v=", fv)) {
                        stm_weld_current = fv;
                    }

                    // Keep legacy vcap= parser path active for old STM32
                    // builds.
                    if (extractFloatField(stmLine, "vcap=", fv)) {
                        stm_vcap = fv;
                        if (!got_weld_v) {
                            weld_v = stm_vcap;
                        }
                    }

                    syncUiConfigFromRuntime();

                    stm_last_status_ms = millis();
                    stm_synced = true;

                    // ✅ DEBUG (optional but very useful)
                    Serial.printf("[SYNC] d1=%d power=%d preheat=%d\n", weld_d1,
                                  weld_power_pct, preheat_ms);

                    uint32_t now = millis();
                    if ((now - lastStatusForwardMs) >=
                        STATUS_FORWARD_MIN_INTERVAL_MS) {
                        sendToPi(buildStatus());
                        lastStatusForwardMs = now;
                    }

                } else if (stmLine.startsWith("EVENT,WELD_START")) {
                    welding_now = true;
                    last_weld_time = millis();
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("EVENT,WELD_DONE")) {
                    welding_now = false;
                    last_weld_time = millis();
                    weld_count++;

                    float fv_done = NAN;
                    // New energy fields (Phase 1A/1B)
                    if (extractFloatField(stmLine, "energy_cap_j=", fv_done))
                        energy_cap_j = fv_done;
                    if (extractFloatField(stmLine, "energy_weld_j=", fv_done))
                        energy_weld_j = fv_done;
                    if (extractFloatField(stmLine, "energy_loss_j=", fv_done))
                        energy_loss_j = fv_done;

                    // Prefer new before/after voltage names when present.
                    if (extractFloatField(stmLine, "weld_v_b=", fv_done)) {
                        weld_v_b = fv_done;
                        vcap_b = fv_done;
                    } else if (extractFloatField(stmLine, "vcap_b=", fv_done)) {
                        weld_v_b = fv_done;
                        vcap_b = fv_done;
                    }
                    if (extractFloatField(stmLine, "weld_v_a=", fv_done)) {
                        weld_v_a = fv_done;
                        vcap_a = fv_done;
                    } else if (extractFloatField(stmLine, "vcap_a=", fv_done)) {
                        weld_v_a = fv_done;
                        vcap_a = fv_done;
                    }
                    if (extractFloatField(stmLine, "cap_v_b=", fv_done))
                        cap_v_b = fv_done;
                    if (extractFloatField(stmLine, "cap_v_a=", fv_done))
                        cap_v_a = fv_done;

                    // Optional instantaneous fields may be included by some
                    // builds.
                    if (extractFloatField(stmLine, "weld_v=", fv_done)) {
                        weld_v = fv_done;
                        stm_vcap = fv_done;
                    }
                    if (extractFloatField(stmLine, "cap_v=", fv_done))
                        cap_v = fv_done;

                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("WAVEFORM,") ||
                           stmLine.startsWith("WAVEFORM_")) {
                    // Forward waveform packets transparently (legacy
                    // single-line format and chunked WAVEFORM_START/DATA/END
                    // format).
                    sendToPi(stmLine);
                    delayMicroseconds(200);  // gives WiFi stack breathing room
                } else if (stmLine.length() > 10 && isdigit(stmLine[0])) {
                    // Bare CSV waveform line (no prefix)
                    sendToPi(stmLine);
                    delayMicroseconds(200);

                } else if (stmLine.startsWith("EVENT,PEDAL_PRESS") ||
                           stmLine.startsWith("EVENT,ARM_TIMEOUT") ||
                           stmLine.startsWith("EVENT,READY_TIMEOUT") ||
                           stmLine.startsWith("DENY,") ||
                           stmLine.startsWith("ERR,")) {
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("IWDBG,")) {
                    sendToPi(stmLine);

                } else if (stmLine.startsWith("ACK,IWZERO,")) {
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("ACK,") ||
                           stmLine.startsWith("DENY,")) {
                    // Forward any ACK/DENY packet even if this firmware
                    // version doesn't have a dedicated handler for it yet.
                    sendToPi(stmLine);

                } else if (stmLine.startsWith("RXHEALTH,")) {
                    // keep local only
                }
            }

            stmLine = "";
        } else {
            if (stmLineOverflow) {
                continue;
            }

            size_t maxLineLength = MAX_LINE_LENGTH;
            if (stmLine.startsWith("WAVEFORM,") ||
                stmLine.startsWith("WAVEFORM_")) {
                maxLineLength = MAX_WAVEFORM_LINE_LENGTH;
            } else if (stmLine.length() < 9) {
                const char* wfPrefixes[] = {"WAVEFORM,", "WAVEFORM_"};
                bool possibleWaveform = false;
                for (size_t p = 0; p < 2 && !possibleWaveform; ++p) {
                    possibleWaveform = true;
                    for (size_t i = 0; i < stmLine.length(); ++i) {
                        if (stmLine[i] != wfPrefixes[p][i]) {
                            possibleWaveform = false;
                            break;
                        }
                    }
                }
                if (possibleWaveform) {
                    maxLineLength = MAX_WAVEFORM_LINE_LENGTH;
                }
            }

            if (stmLine.length() < maxLineLength) {
                stmLine += ch;
            } else {
                stmLineOverflow = true;
            }
        }
    }
}  // ← ADD THIS — closes pollStm32Uart()
// =========================
// Command parser
// =========================
void processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "PING") {
        sendToPi("ACK:PING");
        return;
    }

    if (cmd == "CELLS") {
        // CELLS now sourced from STM32 STATUS2 data
        char buf[160];
        snprintf(buf, sizeof(buf), "CELLS,C1=%.3f,C2=%.3f,C3=%.3f", stm_cell1,
                 stm_cell2, stm_cell3);
        sendToPi(String(buf));
        return;
    }

    if (cmd.startsWith("ACK,")) {
        Serial.printf("[TCP] Ignoring UI ACK: %s\n", cmd.c_str());
        return;
    }

    // Block STM32-forwarding commands until boot config sent
    if (!config_sent) {
        Serial.printf("[CMD] Dropped (pre-boot): %s\n", cmd.c_str());
        return;
    }

    Serial.printf("[CMD] Processing: %s\n", cmd.c_str());

    if (cmd.startsWith("SET_PULSE,")) {
        int values[6] = {0};
        int start = 10;

        for (int i = 0; i < 5; i++) {
            int commaPos = cmd.indexOf(',', start);
            if (commaPos < 0) {
                sendToPi("ERR:BAD_SET_PULSE");
                return;
            }
            values[i] = cmd.substring(start, commaPos).toInt();
            start = commaPos + 1;
        }
        values[5] = cmd.substring(start).toInt();

        weld_mode = values[0];
        weld_d1 = values[1];
        weld_gap1 = values[2];
        weld_d2 = values[3];
        weld_gap2 = values[4];
        weld_d3 = values[5];

        forwardToStm32(cmd);
        updateScreenDisplay();    // redraw touch UI with new values
        sendToPi(buildStatus());  // immediate STATUS to Flask (bypass throttle)
        lastStatusForwardMs =
            millis();  // reset throttle so next auto-STATUS doesn't double-fire
        return;
    }

    if (cmd.startsWith("SET_POWER,")) {
        weld_power_pct = cmd.substring(10).toInt();
        if (weld_power_pct < 50) weld_power_pct = 50;
        if (weld_power_pct > 100) weld_power_pct = 100;

        forwardToStm32(cmd);
        updateScreenDisplay();    // redraw touch UI with new values
        sendToPi(buildStatus());  // immediate STATUS to Flask (bypass throttle)
        lastStatusForwardMs =
            millis();  // reset throttle so next auto-STATUS doesn't double-fire
        return;
    }

    if (cmd.startsWith("SET_PREHEAT,")) {
        int values[4] = {0};
        int start = 12;

        for (int i = 0; i < 3; i++) {
            int commaPos = cmd.indexOf(',', start);
            if (commaPos < 0) {
                sendToPi("ERR:BAD_SET_PREHEAT");
                return;
            }
            values[i] = cmd.substring(start, commaPos).toInt();
            start = commaPos + 1;
        }
        values[3] = cmd.substring(start).toInt();

        preheat_enabled = (values[0] == 1);
        preheat_ms = values[1];
        preheat_pct = values[2];
        preheat_gap_ms = values[3];

        forwardToStm32(cmd);
        updateScreenDisplay();    // redraw touch UI with new values
        sendToPi(buildStatus());  // immediate STATUS to Flask (bypass throttle)
        lastStatusForwardMs =
            millis();  // reset throttle so next auto-STATUS doesn't double-fire
        return;
    }

    if (cmd.startsWith("SET_LEAD_R,")) {
        float mohm = cmd.substring(strlen("SET_LEAD_R,")).toFloat();
        float ohms = mohm / 1000.0f;

        if (!isfinite(ohms) || ohms < LEAD_RESISTANCE_MIN_OHMS ||
            ohms > LEAD_RESISTANCE_MAX_OHMS) {
            sendToPi("ERR:LEAD_R_RANGE");
            return;
        }

        // LEAD_R flow MUST be authoritative from STM32:
        // 1) forward to STM32,
        // 2) wait for STM32 ACK/DENY,
        // 3) only then mirror persistence locally.
        // Never send a synthetic ACK from ESP32 here.
        pending_lead_r_ohms = ohms;
        lead_r_update_inflight = true;
        sendLeadResistanceToStm32(ohms);
        return;
    }

    if (cmd.startsWith("SET_TRIGGER_MODE,")) {
        int mode = cmd.substring(strlen("SET_TRIGGER_MODE,")).toInt();
        if (mode < TRIGGER_MODE_PEDAL) mode = TRIGGER_MODE_PEDAL;
        if (mode > TRIGGER_MODE_CONTACT) mode = TRIGGER_MODE_CONTACT;
        trigger_mode = (uint8_t)mode;

        forwardToStm32(cmd);
        updateScreenDisplay();    // redraw touch UI with new values
        sendToPi(buildStatus());  // immediate STATUS to Flask (bypass throttle)
        lastStatusForwardMs =
            millis();  // reset throttle so next auto-STATUS doesn't double-fire
        return;
    }

    if (cmd.startsWith("SET_CONTACT_HOLD,")) {
        int steps = cmd.substring(strlen("SET_CONTACT_HOLD,")).toInt();
        if (steps < 1) steps = 1;
        if (steps > 10) steps = 10;
        contact_hold_steps = (uint8_t)steps;

        forwardToStm32(cmd);
        updateScreenDisplay();    // redraw touch UI with new values
        sendToPi(buildStatus());  // immediate STATUS to Flask (bypass throttle)
        lastStatusForwardMs =
            millis();  // reset throttle so next auto-STATUS doesn't double-fire
        return;
    }

    if (cmd.startsWith("SET_CONTACT_WITH_PEDAL,")) {
        int val = cmd.substring(strlen("SET_CONTACT_WITH_PEDAL,")).toInt();
        contact_with_pedal = (val == 1);

        forwardToStm32(cmd);
        updateScreenDisplay();    // redraw touch UI with new values
        sendToPi(buildStatus());  // immediate STATUS to Flask (bypass throttle)
        lastStatusForwardMs =
            millis();  // reset throttle so next auto-STATUS doesn't double-fire
        return;
    }

    if (cmd == "RESET_WELD_COUNT") {
        weld_count = 0;
        Serial.println("[Weld] Counter reset to 0");
        sendToPi(buildStatus());
        return;
    }

    if (cmd.startsWith("ARM,")) {
        forwardToStm32(cmd);
        updateScreenDisplay();    // redraw touch UI with new values
        sendToPi(buildStatus());  // immediate STATUS to Flask (bypass throttle)
        lastStatusForwardMs =
            millis();  // reset throttle so next auto-STATUS doesn't double-fire
        return;
    }

    if (cmd == "IWZERO" || cmd == "IWDBG") {
        forwardToStm32(cmd);
        updateScreenDisplay();    // redraw touch UI with new values
        sendToPi(buildStatus());  // immediate STATUS to Flask (bypass throttle)
        lastStatusForwardMs =
            millis();  // reset throttle so next auto-STATUS doesn't double-fire
        return;
    }

    if (cmd == "FIRE") {
        sendToPi("ACK:FIRE_IGNORED");
        return;
    }

    if (cmd == "STATUS") {
        sendToPi(buildStatus());
        requestStm32Status();
        return;
    }

    // Unknown command – forward raw to STM32 (passthrough for debug commands
    // like AMC_LIVE, DBG_SHUNT, etc.)
    Serial.printf("[PASSTHROUGH->STM32] %s\n", cmd.c_str());
    forwardToStm32(cmd);
    updateScreenDisplay();    // redraw touch UI with new values
    sendToPi(buildStatus());  // immediate STATUS to Flask (bypass throttle)
    lastStatusForwardMs =
        millis();  // reset throttle so next auto-STATUS doesn't double-fire
}

// =========================
// READY / UI connection
// =========================
void setUiConnected(bool connected) {
    if (connected == uiConnected) return;
    uiConnected = connected;

    if (uiConnected) {
        Serial.println("[READY] UI (TCP) connected");
        stm_synced = false;
        requestStm32Status();  // gated by config_sent
    } else {
        Serial.println("[READY] UI (TCP) disconnected -> ARM,0");
        if (config_sent) forwardToStm32("ARM,0");  // gated until boot config
        stm_synced = false;
    }
}

void serviceReadyHeartbeat() {
    if (!config_sent) return;  // gated until boot config sent
    uint32_t now = millis();
    if ((now - lastReadySentMs) >= READY_PERIOD_MS) {
        forwardToStm32("READY,1");
        lastReadySentMs = now;
    }
}

// =========================
// WiFi / server maintenance
// =========================
void ensureWiFiAndServer() {
    static unsigned long last_attempt = 0;
    static bool server_started = false;

    if (WiFi.status() == WL_CONNECTED) {
        if (!server_started) {
            Serial.println("✅ WiFi connected (re) - IP: " +
                           WiFi.localIP().toString());
            Serial.println("Starting TCP server on port 8888...");
            server.begin();
            server.setNoDelay(true);
            server_started = true;
        }
        return;
    }

    server_started = false;
    server.stop();

    if (client) {
        client.stop();
    }

    setUiConnected(false);

    unsigned long now = millis();
    if (now - last_attempt > 5000) {
        last_attempt = now;
        Serial.println("⚠️ WiFi disconnected; retrying...");
        WiFi.disconnect(true);
        delay(200);
        WiFi.begin(ssid, password);
    }
}

// =========================
// UI arm toggle callback
// =========================
static void onArmToggle(bool arm) {
    forwardToStm32(String("ARM,") + (arm ? "1" : "0"));
}

// =========================
// UI recipe apply callback (Phase 2B)
// =========================
static void onRecipeApply(uint8_t mode, uint16_t d1, uint16_t gap1, uint16_t d2,
                          uint16_t gap2, uint16_t d3, uint8_t power_pct_val,
                          bool preheat_en, uint16_t ph_ms, uint8_t ph_pct,
                          uint16_t ph_gap_ms) {
    char buf[80];

    // SET_PULSE – reuse existing processCommand path (updates locals + forwards
    // to STM32)
    snprintf(buf, sizeof(buf), "SET_PULSE,%d,%d,%d,%d,%d,%d", mode, d1, gap1,
             d2, gap2, d3);
    processCommand(String(buf));

    // SET_POWER
    snprintf(buf, sizeof(buf), "SET_POWER,%d", power_pct_val);
    processCommand(String(buf));

    // SET_PREHEAT
    snprintf(buf, sizeof(buf), "SET_PREHEAT,%d,%d,%d,%d", preheat_en ? 1 : 0,
             ph_ms, ph_pct, ph_gap_ms);
    processCommand(String(buf));

    Serial.println("[Recipe] Applied via touch UI");

    // Persist recipe to NVS so it can be restored on boot
    save_recipe_to_nvs();
}

// =========================
// Config Persistence (NVS via Preferences)
// =========================
static Preferences prefs;

static void apply_brightness(uint8_t level) {
    // Brightness delta guard – skip if unchanged
    static int8_t last_brightness = -1;  // -1 = never set
    if ((int8_t)level == last_brightness) return;
    last_brightness = (int8_t)level;

    float val;
    switch (level) {
        case 0:
            val = 0.3f;
            break;  // LOW
        case 1:
            val = 0.6f;
            break;  // MED
        case 2:
            val = 1.0f;
            break;  // HIGH
        default:
            val = 1.0f;
            break;
    }
    smartdisplay_lcd_set_backlight(val);
    Serial.printf("[Config] Brightness set to %s (%.1f)\n",
                  level == 0   ? "LOW"
                  : level == 1 ? "MED"
                               : "HIGH",
                  (double)val);
}

static float clamp_lead_r_mohm_ui(float value) {
    float clamped = value;
    if (!isfinite(clamped)) clamped = UI_LEAD_R_DEFAULT_MOHM;
    if (clamped < UI_LEAD_R_MIN_MOHM) clamped = UI_LEAD_R_MIN_MOHM;
    if (clamped > UI_LEAD_R_MAX_MOHM) clamped = UI_LEAD_R_MAX_MOHM;
    return clamped;
}

static void save_config_to_nvs(const ConfigState& cfg) {
    prefs.begin("weldcfg", false);
    prefs.putBool("holdRepeat", cfg.hold_to_repeat);
    prefs.putUChar("timeStep", cfg.time_step_ms);
    prefs.putUChar("powerStep", cfg.power_step_pct);
    prefs.putBool("loadLast", cfg.load_last_on_boot);
    prefs.putUChar("bright", cfg.brightness);
    prefs.putBool("cwPedal", cfg.contact_with_pedal);
    prefs.putUChar("trigMode", trigger_mode);
    prefs.putUChar("holdSteps", cfg.contact_hold_steps);
    prefs.putFloat("leadRmohm", clamp_lead_r_mohm_ui(cfg.lead_resistance_mohm));
    prefs.end();
    Serial.println("[Config] Saved to NVS");
}

static ConfigState load_config_from_nvs() {
    ConfigState cfg = config_defaults();
    prefs.begin("weldcfg", true);  // read-only
    cfg.hold_to_repeat = prefs.getBool("holdRepeat", cfg.hold_to_repeat);
    cfg.time_step_ms = prefs.getUChar("timeStep", cfg.time_step_ms);
    cfg.power_step_pct = prefs.getUChar("powerStep", cfg.power_step_pct);
    cfg.load_last_on_boot = prefs.getBool("loadLast", cfg.load_last_on_boot);
    cfg.brightness = prefs.getUChar("bright", cfg.brightness);
    cfg.contact_with_pedal = prefs.getBool("cwPedal", cfg.contact_with_pedal);

    trigger_mode = prefs.getUChar("trigMode", trigger_mode);
    cfg.contact_hold_steps =
        prefs.getUChar("holdSteps", cfg.contact_hold_steps);

    // Prefer consolidated weldcfg key; fallback to legacy namespace key.
    bool has_lead_r_in_weldcfg = prefs.isKey("leadRmohm");
    if (has_lead_r_in_weldcfg) {
        cfg.lead_resistance_mohm =
            prefs.getFloat("leadRmohm", cfg.lead_resistance_mohm);
    }
    contact_hold_steps = cfg.contact_hold_steps;  // sync global

    prefs.end();

    if (!has_lead_r_in_weldcfg) {
        prefs.begin("spotwelder", true);
        cfg.lead_resistance_mohm =
            prefs.getFloat("lead_r_mohm", UI_LEAD_R_DEFAULT_MOHM);
        prefs.end();
    }

    cfg.lead_resistance_mohm = clamp_lead_r_mohm_ui(cfg.lead_resistance_mohm);
    lead_resistance_ohms = cfg.lead_resistance_mohm / 1000.0f;

    Serial.println("[Config] Loaded from NVS");
    return cfg;
}

static void save_recipe_to_nvs() {
    prefs.begin("weldrecipe", false);
    prefs.putUChar("mode", weld_mode);
    prefs.putUShort("d1", weld_d1);
    prefs.putUShort("gap1", weld_gap1);
    prefs.putUShort("d2", weld_d2);
    prefs.putUShort("gap2", weld_gap2);
    prefs.putUShort("d3", weld_d3);
    prefs.putUChar("power", weld_power_pct);
    prefs.putBool("phEn", preheat_enabled);
    prefs.putUShort("phMs", preheat_ms);
    prefs.putUChar("phPct", preheat_pct);
    prefs.putUShort("phGap", preheat_gap_ms);
    prefs.end();
    Serial.println("[Config] Recipe saved to NVS");
}

static void save_lead_resistance_to_nvs() {
    float lead_r_mohm = clamp_lead_r_mohm_ui(lead_resistance_ohms * 1000.0f);

    // New consolidated location with other config values.
    prefs.begin("weldcfg", false);
    prefs.putFloat("leadRmohm", lead_r_mohm);
    prefs.end();

    // Legacy key kept for backward compatibility during migration.
    prefs.begin("spotwelder", false);
    prefs.putFloat("lead_r_mohm", lead_r_mohm);
    prefs.end();

    Serial.printf("[Config] Lead resistance saved: %.3f mOhm\n",
                  (double)lead_r_mohm);
}

static void load_lead_resistance_from_nvs() {
    // Prefer consolidated key in weldcfg.
    prefs.begin("weldcfg", true);
    bool has_weldcfg_key = prefs.isKey("leadRmohm");
    float lead_r_mohm = prefs.getFloat("leadRmohm", UI_LEAD_R_DEFAULT_MOHM);
    prefs.end();

    if (!has_weldcfg_key) {
        // Fallback to legacy namespace used before Session 4.
        prefs.begin("spotwelder", true);
        lead_r_mohm = prefs.getFloat("lead_r_mohm", UI_LEAD_R_DEFAULT_MOHM);
        prefs.end();
    }

    lead_r_mohm = clamp_lead_r_mohm_ui(lead_r_mohm);
    lead_resistance_ohms = lead_r_mohm / 1000.0f;

    Serial.printf("[Config] Lead resistance loaded: %.3f mOhm\n",
                  (double)(lead_resistance_ohms * 1000.0f));
}

static void sendLeadResistanceToStm32(float ohms) {
    float clamped = ohms;
    if (!isfinite(clamped) || clamped < LEAD_RESISTANCE_MIN_OHMS)
        clamped = LEAD_RESISTANCE_MIN_OHMS;
    if (clamped > LEAD_RESISTANCE_MAX_OHMS) clamped = LEAD_RESISTANCE_MAX_OHMS;

    char buf[48];
    snprintf(buf, sizeof(buf), "LEAD_R,%.6f", clamped);
    forwardToStm32(String(buf));
}

static void load_recipe_from_nvs() {
    prefs.begin("weldrecipe", true);
    weld_mode = prefs.getUChar("mode", weld_mode);
    weld_d1 = prefs.getUShort("d1", weld_d1);
    weld_gap1 = prefs.getUShort("gap1", weld_gap1);
    weld_d2 = prefs.getUShort("d2", weld_d2);
    weld_gap2 = prefs.getUShort("gap2", weld_gap2);
    weld_d3 = prefs.getUShort("d3", weld_d3);
    weld_power_pct = prefs.getUChar("power", weld_power_pct);
    preheat_enabled = prefs.getBool("phEn", preheat_enabled);
    preheat_ms = prefs.getUShort("phMs", preheat_ms);
    preheat_pct = prefs.getUChar("phPct", preheat_pct);
    preheat_gap_ms = prefs.getUShort("phGap", preheat_gap_ms);
    prefs.end();
    Serial.println("[Config] Recipe loaded from NVS");
}

// =========================
// Phase 1: Deferred boot config – sends all NVS-loaded settings to STM32
// Called when BOOT message received OR after fallback timeout.
// Pre-flushes STM32 rx_build[] then sends paced commands (20ms inter-gap).
// =========================
static void sendBootConfig() {
    if (config_sent) return;  // idempotent guard
    config_sent = true;

    Serial.println("[Boot] Sending deferred boot config to STM32...");

    // Flush stale bytes from both sides of the UART link.
    while (STM32Serial.available()) STM32Serial.read();  // flush ESP32 RX
    delay(50);
    STM32Serial.print("\r\n");  // flush STM32 rx_build[]
    STM32Serial.flush();
    delay(50);
    Serial.println("[Boot] Flushing STM32 RX buffer");

    char buf[80];

    // SAFETY DEFAULTS ON BOOT: always start disarmed + pedal mode.
    stm_armed = false;
    trigger_mode = TRIGGER_MODE_PEDAL;
    contact_with_pedal = true;

    // 1. Recipe / pulse settings
    snprintf(buf, sizeof(buf), "SET_PULSE,%d,%d,%d,%d,%d,%d", weld_mode,
             weld_d1, weld_gap1, weld_d2, weld_gap2, weld_d3);
    forwardToStm32(String(buf));
    delay(20);

    // 2. Power
    snprintf(buf, sizeof(buf), "SET_POWER,%d", weld_power_pct);
    forwardToStm32(String(buf));
    delay(20);

    // 3. Preheat
    snprintf(buf, sizeof(buf), "SET_PREHEAT,%d,%d,%d,%d",
             preheat_enabled ? 1 : 0, preheat_ms, preheat_pct, preheat_gap_ms);
    forwardToStm32(String(buf));
    delay(20);

    // 4. Trigger mode (forced to pedal)
    forwardToStm32("SET_TRIGGER_MODE,1");
    delay(20);

    // 5. Contact hold
    snprintf(buf, sizeof(buf), "SET_CONTACT_HOLD,%d", (int)contact_hold_steps);
    forwardToStm32(String(buf));
    delay(20);

    // 6. Contact with pedal (forced enabled)
    forwardToStm32("SET_CONTACT_WITH_PEDAL,1");
    delay(20);

    // 7. Lead resistance (persisted, in ohms)
    sendLeadResistanceToStm32(lead_resistance_ohms);
    delay(20);

    // 8. Force disarmed state for safety
    forwardToStm32("ARM,0");
    delay(20);

    // 9. Kick-start STM32 status broadcasting
    forwardToStm32("READY,1");
    lastReadySentMs = millis();
    delay(20);

    // 10. Request current STM32 status
    requestStm32Status();

    config_sent_ms = millis();
    Serial.println(
        "[Boot] Config sent with safety defaults (DISARMED + PEDAL)");
}

// =========================
// Reconnect sync: re-send current recipe/settings so host UI can
// immediately re-enable ARM without requiring manual Apply.
// =========================
static void syncSettingsAfterUiReconnect() {
    if (!config_sent) {
        Serial.println(
            "[TCP] UI reconnect sync skipped (boot config not sent yet)");
        return;
    }

    // Re-validate recipe before sending to STM32.
    if (weld_mode < 1 || weld_mode > 3 || weld_d1 == 0) {
        Serial.printf(
            "[TCP] Invalid recipe for reconnect sync (mode=%u d1=%u) - "
            "skipped\n",
            (unsigned)weld_mode, (unsigned)weld_d1);
        return;
    }

    Serial.println(
        "[TCP] === Re-syncing current live settings/state (no forced defaults) "
        "===");
    Serial.printf(
        "[TCP] Sync values: mode=%u d1=%u gap1=%u d2=%u gap2=%u d3=%u power=%u "
        "preheat_en=%u preheat_ms=%u preheat_pct=%u preheat_gap=%u trigger=%u "
        "contact_hold=%u contact_pedal=%u armed=%u\n",
        (unsigned)weld_mode, (unsigned)weld_d1, (unsigned)weld_gap1,
        (unsigned)weld_d2, (unsigned)weld_gap2, (unsigned)weld_d3,
        (unsigned)weld_power_pct, (unsigned)(preheat_enabled ? 1 : 0),
        (unsigned)preheat_ms, (unsigned)preheat_pct, (unsigned)preheat_gap_ms,
        (unsigned)trigger_mode, (unsigned)contact_hold_steps,
        (unsigned)(contact_with_pedal ? 1 : 0), (unsigned)(stm_armed ? 1 : 0));

    char buf[96];

    snprintf(buf, sizeof(buf), "SET_PULSE,%u,%u,%u,%u,%u,%u",
             (unsigned)weld_mode, (unsigned)weld_d1, (unsigned)weld_gap1,
             (unsigned)weld_d2, (unsigned)weld_gap2, (unsigned)weld_d3);
    forwardToStm32(String(buf));
    delay(20);

    snprintf(buf, sizeof(buf), "SET_POWER,%u", (unsigned)weld_power_pct);
    forwardToStm32(String(buf));
    delay(20);

    snprintf(buf, sizeof(buf), "SET_PREHEAT,%u,%u,%u,%u",
             (unsigned)(preheat_enabled ? 1 : 0), (unsigned)preheat_ms,
             (unsigned)preheat_pct, (unsigned)preheat_gap_ms);
    forwardToStm32(String(buf));
    delay(20);

    snprintf(buf, sizeof(buf), "SET_TRIGGER_MODE,%u", (unsigned)trigger_mode);
    forwardToStm32(String(buf));
    delay(20);

    snprintf(buf, sizeof(buf), "SET_CONTACT_HOLD,%u",
             (unsigned)contact_hold_steps);
    forwardToStm32(String(buf));
    delay(20);

    snprintf(buf, sizeof(buf), "SET_CONTACT_WITH_PEDAL,%u",
             (unsigned)(contact_with_pedal ? 1 : 0));
    forwardToStm32(String(buf));
    delay(20);

    sendLeadResistanceToStm32(lead_resistance_ohms);
    delay(20);

    snprintf(buf, sizeof(buf), "ARM,%u", (unsigned)(stm_armed ? 1 : 0));
    forwardToStm32(String(buf));
    delay(20);

    // Ask STM32 to immediately publish fresh status after re-sync.
    requestStm32Status();

    // Mark sync optimistic; definitive state is refreshed by incoming
    // STATUS/ACK.
    stm_synced = true;
    Serial.println("[TCP] Re-sync complete (current state preserved)");
}

static void syncUiConfigFromRuntime() {
    ConfigState cfg = ui_get_config();
    bool changed = false;

    uint8_t hold_steps = contact_hold_steps;
    if (hold_steps < 1) hold_steps = 1;
    if (hold_steps > 10) hold_steps = 10;

    if (cfg.contact_hold_steps != hold_steps) {
        cfg.contact_hold_steps = hold_steps;
        changed = true;
    }

    if (cfg.contact_with_pedal != contact_with_pedal) {
        cfg.contact_with_pedal = contact_with_pedal;
        changed = true;
    }

    float lead_r_mohm = clamp_lead_r_mohm_ui(lead_resistance_ohms * 1000.0f);
    if (fabsf(cfg.lead_resistance_mohm - lead_r_mohm) > 0.0001f) {
        cfg.lead_resistance_mohm = lead_r_mohm;
        changed = true;
    }

    if (changed) {
        ui_load_config(cfg);
    }
}

// Callback from UI when config changes
static void onConfigChange(const ConfigState& cfg) {
    ConfigState normalized_cfg = cfg;
    normalized_cfg.lead_resistance_mohm =
        clamp_lead_r_mohm_ui(normalized_cfg.lead_resistance_mohm);

    apply_brightness(normalized_cfg.brightness);

    // Sync contact_hold_steps from ConfigState to main.cpp global and STM32
    if (normalized_cfg.contact_hold_steps != contact_hold_steps) {
        contact_hold_steps = normalized_cfg.contact_hold_steps;
        char buf[40];
        snprintf(buf, sizeof(buf), "SET_CONTACT_HOLD,%d",
                 (int)contact_hold_steps);
        processCommand(String(buf));
    }

    float target_lead_ohm = normalized_cfg.lead_resistance_mohm / 1000.0f;
    if (fabsf(target_lead_ohm - lead_resistance_ohms) > 0.000001f) {
        char buf[48];
        snprintf(buf, sizeof(buf), "SET_LEAD_R,%.3f",
                 (double)normalized_cfg.lead_resistance_mohm);
        processCommand(String(buf));
    }

    save_config_to_nvs(normalized_cfg);
}

// Callback from UI when trigger source changes (Status tab buttons)
static void onTriggerSourceChange(uint8_t mode) {
    char buf[40];
    snprintf(buf, sizeof(buf), "SET_TRIGGER_MODE,%d", (int)mode);
    processCommand(String(buf));

    ConfigState cfg = load_config_from_nvs();
    save_config_to_nvs(cfg);

    Serial.printf("[Trigger] Mode changed to %s via touch UI\n",
                  mode == 1 ? "Pedal" : "Probe");
}

// Callback from UI when weld counter is tapped (reset)
static void onWeldCountReset() {
    processCommand("RESET_WELD_COUNT");
    Serial.println("[Weld] Counter reset via touch UI");
}

// Callback from UI when contact-with-pedal toggle changes
static void onContactWithPedalChange(bool enabled) {
    char buf[40];
    snprintf(buf, sizeof(buf), "SET_CONTACT_WITH_PEDAL,%d", enabled ? 1 : 0);

    processCommand(String(buf));

    ConfigState cfg = load_config_from_nvs();
    cfg.contact_with_pedal = enabled;
    save_config_to_nvs(cfg);

    Serial.printf("[Config] Contact-with-pedal set to %s via touch UI\n",
                  enabled ? "ON" : "OFF");
}

// =========================
// Setup
// =========================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("===========================================");
    Serial.println("ESP32 Spot Welder Controller Boot Sequence");
    Serial.println("===========================================");
    Serial.println("[Boot] 1/4 Initializing firmware...");

    // --- UART to STM32 ---
    STM32Serial.setRxBufferSize(
        8192);  // LAMBO buffer for large waveform bursts
    STM32Serial.begin(2000000, SERIAL_8N1, STM32_TO_ESP32_PIN,
                      ESP32_TO_STM32_PIN);
    Serial.println("✅ STM32 UART bridge ready (Serial2 @ 2000000)");

    // --- Front button ---
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // ==========================================================
    // DISPLAY + TOUCH INIT
    // smartdisplay_init() claims I2C_NUM_0 for GT911 at 400kHz
    // DO NOT call Wire.begin() after this – it conflicts!
    // ==========================================================
    Serial.println("Initializing display (smartdisplay)...");
    smartdisplay_init();
    smartdisplay_lcd_set_backlight(1.0f);
    Serial.println("✅ Display initialized");

    // ==========================================================
    // TOUCH INDEV OVERRIDE – wrap smartdisplay driver with filter
    // ==========================================================
    lv_indev_t* indev = lv_indev_get_next(nullptr);
    bool found_touch = false;

    while (indev != nullptr) {
        if (lv_indev_get_type(indev) == LV_INDEV_TYPE_POINTER) {
            Serial.println(
                "✅ Touch indev found – wrapping with debounce filter");
            original_read_cb = indev->read_cb;
            indev->read_cb = debounced_touchpad_read;
            found_touch = true;
            break;
        }
        indev = lv_indev_get_next(indev);
    }

    if (!found_touch) {
        Serial.println("⚠️ No touch indev found from smartdisplay!");
    }

    // ==========================================================
    // LVGL UI
    // ==========================================================
    ui_init(onArmToggle, onRecipeApply);
    Serial.println("✅ UI created (5-tab shell + Pulse tab)");

    // --- Config: Load settings from NVS into memory (Phase 1: NO STM32 sends
    // here) ---
    {
        ConfigState cfg = load_config_from_nvs();

        // Apply brightness immediately (local display only, not STM32)
        apply_brightness(cfg.brightness);

        // Restore recipe from NVS if enabled – into globals only, NOT sent to
        // STM32
        if (cfg.load_last_on_boot) {
            load_recipe_from_nvs();
            Serial.println(
                "[Config] Load-last-on-boot: recipe loaded to memory (deferred "
                "send)");
        }

        // Sync globals from loaded config (no STM32 commands)
        // Force safety-critical defaults regardless of NVS
        // (user can change after boot, but we always start safe)
        stm_armed = false;                  // DISARMED
        trigger_mode = TRIGGER_MODE_PEDAL;  // MANUAL/PEDAL
        contact_with_pedal = true;          // require pedal gating
        load_lead_resistance_from_nvs();

        // Push loaded config to UI widgets (display-side only)
        ui_set_config_cb(onConfigChange);
        ui_load_config(cfg);
        Serial.println(
            "✅ Config loaded into memory (STM32 send deferred to "
            "boot/timeout)");
    }

    // --- Safety defaults (always on ESP32 reboot) ---
    Serial.println("[Boot] 2/4 Applying safety defaults...");
    stm_armed = false;  // always boot disarmed
    stm_synced = false;
    trigger_mode = TRIGGER_MODE_PEDAL;  // always boot in pedal mode
    contact_with_pedal = true;          // pedal gating enabled

    Serial.println("[Boot] Safety defaults applied:");
    Serial.println("       - Armed: false");
    Serial.println("       - Trigger mode: PEDAL");
    Serial.println("       - Contact with pedal: enabled");
    Serial.printf(
        "[Boot] Persistent recipe retained: mode=%u d1=%u gap1=%u d2=%u "
        "gap2=%u d3=%u power=%u preheat_en=%u preheat_ms=%u preheat_pct=%u "
        "preheat_gap=%u\n",
        (unsigned)weld_mode, (unsigned)weld_d1, (unsigned)weld_gap1,
        (unsigned)weld_d2, (unsigned)weld_gap2, (unsigned)weld_d3,
        (unsigned)weld_power_pct, (unsigned)(preheat_enabled ? 1 : 0),
        (unsigned)preheat_ms, (unsigned)preheat_pct, (unsigned)preheat_gap_ms);

    // --- Register remaining UI callbacks ---
    ui_set_trigger_source_cb(onTriggerSourceChange);
    ui_set_weld_count_reset_cb(onWeldCountReset);
    ui_set_contact_with_pedal_cb(onContactWithPedalChange);
    Serial.println("[Boot] 3/4 UI callbacks registered");

    // ==========================================================
    // WiFi  (non-blocking – removed blocking wait)
    // ==========================================================
    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(
        WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    Serial.println("[Boot] 4/4 Starting WiFi + TCP services...");
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    setup_done_ms = millis();
    Serial.println(
        "[Boot] Boot complete - waiting for STM32 BOOT/fallback sync");
    Serial.println("===========================================");
}

// Loop
// =========================
static unsigned long lv_last_tick = 0;

void loop() {
    // --- LVGL tick + task handler ---
    unsigned long now_ms = millis();
    lv_tick_inc(now_ms - lv_last_tick);
    lv_last_tick = now_ms;
    lv_timer_handler();

    // --- Phase 1: fallback timeout – send config if no BOOT msg received ---
    if (!config_sent && !stm32_booted && setup_done_ms > 0 &&
        (now_ms - setup_done_ms) > BOOT_TIMEOUT_MS) {
        Serial.println("[Boot] No BOOT message – sending config via fallback");
        sendBootConfig();
    }

    // --- Existing logic ---
    ensureWiFiAndServer();
    pollStm32Uart();
    serviceReadyHeartbeat();

    static uint32_t lastStatusReqMs = 0;
    uint32_t now = millis();

    if ((now - lastStatusReqMs) >= 1000) {
        requestStm32Status();
        lastStatusReqMs = now;
    }

    if (WiFi.status() == WL_CONNECTED) {
        WiFiClient newClient = server.available();
        if (newClient) {
            if (client && client.connected()) {
                Serial.println(
                    "[TCP] New client arrived -> dropping old client");
                client.stop();
            }

            client = newClient;
            client.setNoDelay(true);
            Serial.println("[TCP] Client connected from " +
                           client.remoteIP().toString());

            lastTcpRxMs = millis();
            setUiConnected(true);
            sendToPi(buildStatus());
            syncSettingsAfterUiReconnect();
            requestStm32Status();
        }

        if (client && !client.connected()) {
            Serial.println("[TCP] Client disconnected");
            client.stop();
            setUiConnected(false);
        }

        if (client && client.connected() && TCP_IDLE_TIMEOUT_MS > 0) {
            if ((millis() - lastTcpRxMs) > TCP_IDLE_TIMEOUT_MS) {
                Serial.println("[TCP] Idle timeout -> dropping client");
                client.stop();
                setUiConnected(false);
            }
        }

        if (client && client.connected() && client.available()) {
            String line = client.readStringUntil('\n');
            line.trim();
            if (line.length() > 0) {
                lastTcpRxMs = millis();
                Serial.printf("[TCP] RX: %s\n", line.c_str());
                processCommand(line);
            }
        }
    }

    handleButton();

    // --- Drive UI touch-active flag from hardware touch driver state ---
    {
        bool hw_touch = (touch_filter.state == TOUCH_PRESSED ||
                         touch_filter.state == TOUCH_PENDING_PRESS ||
                         touch_filter.state == TOUCH_PENDING_RELEASE);
        ui_set_touch_active(hw_touch);
    }

    // --- Update on-screen UI (display-smoothing applied inside helper) ---
    updateScreenDisplay();

    delay(5);
}

