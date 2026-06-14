/**
 * OTA firmware for ESP32-based TFT display
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
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <SD.h>      // microSD card (bundled with arduino-esp32 – NOT a lib_deps entry)
#include <SPI.h>     // dedicated SPI bus for the SD card (bundled)
#include <Update.h>  // ESP32 self-flash from a firmware image (bundled)
#include <WebServer.h>
#include <WiFi.h>
#include <esp32_smartdisplay.h>
#include <driver/i2c.h>  // raw GT911 register access for touch-sensitivity
#if defined(DISPLAY_ST7262_PAR)
// RGB-panel runtime controls used for the display-drift / OTA-glitch fix:
//   esp_lcd_rgb_panel_set_pclk()  -> slow the pixel clock during OTA so the
//                                    panel reads PSRAM slower and leaves bus
//                                    bandwidth for the firmware flash writes.
//   esp_lcd_rgb_panel_restart()   -> re-align the scan-out DMA after a drift
//                                    ("permanent shift") without a power cycle.
#include <esp_lcd_panel_rgb.h>
// esp_lcd_panel_disp_on_off() halts/restarts the RGB panel's scan-out so the
// LCD stops streaming the (about-to-be-garbage) PSRAM framebuffer during the
// flash-cache-disabled windows of an OTA write. Used by the OTA hard-blank fix.
#include <esp_lcd_panel_ops.h>
// esp_cache_msync() writes the CPU data cache back to PSRAM. After we memset the
// framebuffer(s) the cleared bytes may still be sitting in cache; the RGB panel
// DMA reads PSRAM directly (bypassing cache), so we must flush cache->PSRAM
// before restarting the scan-out or the DMA can still fetch stale garbage.
#include <esp_cache.h>
#endif
#include <esp_wifi.h>
#include <lv_conf.h>
#include <lvgl.h>
#include <math.h>
#include <string.h>  // memset() for the boot framebuffer clear

#include "ui.h"

// Firmware version string shown on the Setup tab.
#define FW_VERSION "1.0.0"

// =========================
// OTA (over-the-air) firmware update configuration
// =========================
// These must match the [env:esp32-8048S043C-ota] upload settings in
// platformio.ini so `pio run -e esp32-8048S043C-ota -t upload` can reach the
// device over WiFi (espota). The mDNS hostname makes the device reachable at
// "spotwelder.local"; the password gates uploads.
#define OTA_HOSTNAME "spotwelder"
#define OTA_PASSWORD "spotwelder2024"
#define OTA_PORT 3232

// =========================
// Sunton hardware mapping
// =========================
// STM32 data link uses GPIO17/18 (the original, known-good wiring). The dual
// firmware updater drives the STM32 into its ROM bootloader purely in SOFTWARE
// (an ESP32 -> STM32 "BOOTLOADER" UART command that the STM32 application acts
// on), so NO BOOT0/RESET GPIO control lines are needed and these are the only
// pins on the STM32 link. The GT911 touch I2C bus (GPIO19/20) is untouched.
#define STM32_TO_ESP32_PIN 17  // ESP RX  <- STM32 TX
#define ESP32_TO_STM32_PIN 18  // ESP TX  -> STM32 RX

// Touch I2C (I2C_NUM_0) – managed by smartdisplay, DO NOT use Wire
#define TOUCH_SDA 19
#define TOUCH_SCL 20

#define BUTTON_PIN 0  // GPIO0 (BOOT button on Sunton board)

// =========================
// microSD card (SPI) – pins come from the board definition
// (boards/esp32-8048S043C.json: TF_CS=10, TF_SPI_MOSI=11, TF_SPI_SCLK=12,
//  TF_SPI_MISO=13). Guard with fallbacks so the file still compiles if the
// board macros are ever absent.
// =========================
#ifndef TF_CS
#define TF_CS 10
#endif
#ifndef TF_SPI_MOSI
#define TF_SPI_MOSI 11
#endif
#ifndef TF_SPI_SCLK
#define TF_SPI_SCLK 12
#endif
#ifndef TF_SPI_MISO
#define TF_SPI_MISO 13
#endif

// Firmware-image filenames expected at the SD card root.
#define SD_ESP32_FW_PATH "/esp32_firmware.bin"
#define SD_STM32_FW_PATH "/stm32_firmware.bin"

// =========================
// STM32 remote-flash control: SOFTWARE bootloader entry (no GPIO mod)
// -------------------------------------------------------------------------
// There are NO BOOT0/NRST control wires between the ESP32 and the STM32. The
// updater instead asks the running STM32 application to jump into its built-in
// ROM bootloader by sending a "BOOTLOADER" command over the normal UART link
// (sendBootloaderCommand(), below). The STM32 firmware must implement a handler
// that, on receiving this command, calls its jump-to-system-memory routine.
// After flashing, the AN3155 "Go" command (0x21) launches the freshly written
// application — again, with no hardware reset line.
//
// Trade-off: this is software-only, so if the STM32 application is bricked (no
// running firmware to receive the command) there is no GPIO fallback to force
// the bootloader; recovery then needs the STM32's own BOOT0 strap / SWD. The
// GT911 touch I2C bus (GPIO19/20) is never touched by any of this.
// =========================

HardwareSerial STM32Serial(2);

// =========================
// WiFi / TCP
// =========================
// WiFi credentials are NOT hardcoded. They are provisioned at runtime via the
// captive-portal Setup flow and stored in NVS ("wificfg"). See the
// "WiFi provisioning" section below.
String wifi_ssid = "";  // loaded from NVS at boot
String wifi_pass = "";  // loaded from NVS at boot

WiFiServer server(8888);  // STM32<->Flask bridge (STA mode)
WiFiClient client;

// ---- WiFi provisioning state machine ----
enum WifiProvState {
    WIFI_PROV_STA_CONNECTING,  // trying saved creds
    WIFI_PROV_STA_CONNECTED,   // associated, bridge running
    WIFI_PROV_AP_PORTAL        // AP + captive portal up (setup mode)
};
static WifiProvState wifi_prov_state = WIFI_PROV_STA_CONNECTING;
static uint32_t wifi_sta_attempt_start_ms = 0;
// How long to wait for a saved network before falling back to AP portal.
static const uint32_t WIFI_STA_CONNECT_TIMEOUT_MS = 20000;

static DNSServer dnsServer;
static WebServer portalServer(80);
static const byte DNS_PORT = 53;
static IPAddress apIP(192, 168, 4, 1);
static String ap_ssid_str = "";  // "SpotWelder-XXXX"
// Cached network-scan <option> list. We scan ONCE when entering AP mode (and
// on explicit user rescan) instead of on every HTTP request. A blocking
// WiFi.scanNetworks() inside the request path delays the phone's captive-portal
// probe by 1-2 s, which is the main reason the setup page does not pop up
// reliably. Serving a cached list makes every portal response instant.
static String g_scan_options = "";
// Set true by the portal handler once new creds are saved; loop() then
// tears down the portal and reconnects in STA mode.
static volatile bool wifi_creds_just_saved = false;
// Last WiFi info pushed to the UI (avoids redundant ui_set_wifi_info calls).
static bool ui_wifi_last_connected = false;
static bool ui_wifi_last_ap = false;

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
 * Threshold: 0.005V (5mV)
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
static VoltageDisplaySmoother tftSmoother(0.005f);

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

// Dual-path bridge timing:
// - STATUS2 is forwarded raw at 500ms cadence (graph/calc consumers)
// - DISPLAY is sent smoothed at 1000ms cadence (UI label consumers)
static String rawStatus2Packet;
static bool hasRawStatus2Data = false;
unsigned long lastStatus2Forward = 0;
unsigned long lastDisplayUpdate = 0;
const int STATUS2_INTERVAL = 500;
const int DISPLAY_INTERVAL = 1000;

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
// Lead resistance is synced from the STM32 STATUS stream exactly once at boot
// (first valid, non-zero value). After that it is updated ONLY by calibration
// (CAL_RESULT) or an explicit SET_LEAD_R round-trip (ACK,LEAD_R). This prevents
// a stray "lead_r_ohm=0" in a periodic STATUS message from making the on-screen
// reading flash to "00".
static bool lead_r_synced_from_status = false;
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

// ---- Dashboard / Joule-mode mirrored state (from STM32 STATUS) ----
static uint8_t  control_mode = 0;        // 0=TIME, 1=JOULE
static float    joule_target_j = 50.0f;  // configured target energy (J)
static uint16_t joule_max_ms = 40;       // configured max-duration safety (ms)
static float    joule_actual_j = 0.0f;   // live workpiece energy from STATUS (J)

// Session-based calibration age (ESP32 has no RTC/wall clock).
static bool     cal_done_this_session = false;
static uint32_t last_cal_ms = 0;

// ---- Last-weld latched results (from EVENT,WELD_DONE) ----
static bool     last_weld_valid = false;
static bool     last_weld_was_joule = false;
static float    last_weld_target_j = 0.0f;
static float    last_weld_energy_j = 0.0f;
static float    last_weld_accuracy_pct = 0.0f;
static uint32_t last_weld_duration_ms = 0;
static float    last_weld_peak_a = 0.0f;
static float    last_weld_avg_a = 0.0f;
static float    last_weld_lead_loss_j = 0.0f;

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

// =========================================================================
// Firmware-update completion popup — deferred to the NEXT boot
// -------------------------------------------------------------------------
// A firmware update that succeeds (and the STM32 update in all cases) ends by
// rebooting the ESP32. We must NOT paint the result popup right before the
// reboot: that render lands while the panel/scan-out is about to be torn down,
// and its leftover pixels in PSRAM produced a brief glitchy frame on the way
// back up. Instead we stash the result in RTC memory (which survives a warm
// ESP.restart() but not a power cycle), and the freshly-booted firmware shows
// the popup ~1 s later — only AFTER displayBootStabilize() + the boot drift
// re-syncs have fully settled the display. A magic sentinel guards against the
// uninitialised RTC garbage present on a true cold power-on.
#define FW_BOOT_POPUP_MAGIC 0xF09A57EDUL
RTC_NOINIT_ATTR static uint32_t rtc_fw_popup_magic;
RTC_NOINIT_ATTR static bool rtc_fw_popup_success;
RTC_NOINIT_ATTR static char rtc_fw_popup_device[8];
RTC_NOINIT_ATTR static char rtc_fw_popup_msg[96];

// Latched in setup() from the RTC flag, consumed exactly once in loop() after
// the display has settled. Plain RAM — only meaningful within one boot.
static bool boot_fw_popup_pending = false;
static bool boot_fw_popup_success = false;
static char boot_fw_popup_device[8] = {0};
static char boot_fw_popup_msg[96] = {0};
// How long after setup() completes before the deferred popup is shown. By this
// point the boot framebuffer clear + dual DMA restart have run, so the panel is
// stable. Reduced from 1000ms -> 300ms now that the display is solid from the
// first frame (XIP-from-PSRAM): the operator sees the result almost immediately.
static const uint32_t FW_BOOT_POPUP_DELAY_MS = 300;

// Stash a firmware-update result so the popup is shown on the next boot. Call
// immediately before ESP.restart().
static void armBootFirmwarePopup(bool success, const char* device,
                                 const char* msg) {
    rtc_fw_popup_success = success;
    snprintf(rtc_fw_popup_device, sizeof(rtc_fw_popup_device), "%s",
             device ? device : "");
    snprintf(rtc_fw_popup_msg, sizeof(rtc_fw_popup_msg), "%s", msg ? msg : "");
    rtc_fw_popup_magic = FW_BOOT_POPUP_MAGIC;  // set last: marks the data valid
}

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
 * GT911 TOUCH SENSITIVITY (hardware threshold adjust)
 *
 * smartdisplay_init() already installs the legacy I2C driver on
 * I2C_NUM_0 (SDA=GPIO19, SCL=GPIO20, 400 kHz) for the GT911 controller,
 * so we can issue raw register reads/writes here WITHOUT re-init and
 * WITHOUT touching Wire (which would conflict).
 *
 * GT911 7-bit address = 0x5D (board uses the default ADDR pin level).
 *
 * IMPORTANT (v2 – noise-immune profiles):
 *   The spot welder's high-current switching generates EMI that couples into
 *   the touch panel.  Simply lowering Touch_Level (v1) made the panel MORE
 *   sensitive but also dropped the threshold BELOW the EMI noise floor, so the
 *   controller reported a storm of phantom touches ("bouncing / mind of its
 *   own").  The fix is NOT a lower threshold – it is to keep a MODERATE
 *   threshold and add hardware noise filtering that scales with sensitivity.
 *
 * Each UI sensitivity step (1-10) now selects a full PROFILE that tunes
 * several config registers together:
 *   0x804C  Touch_Number   – forced to 1 (single-finger UI; rejects phantom
 *                            multi-point noise)
 *   0x804F  Shake_Count    – de-jitter confirm samples. We raise the
 *                            touch-DOWN confirm count (low nibble) at high
 *                            sensitivity so a press must persist for several
 *                            scan frames before it registers (kills transient
 *                            EMI spikes). The touch-UP nibble is preserved.
 *   0x8050  Filter         – Normal_Filter coordinate smoothing (low nibble),
 *                            raised at high sensitivity to stop the cursor
 *                            "bouncing around". First_Filter nibble preserved.
 *   0x8052  Noise_Reduction– 0-15, raised at high sensitivity for EMI immunity.
 *   0x8053  Touch_Level    – press threshold (kept in a usable 46-88 band, not
 *                            driven down to the noise floor).
 *   0x8054  Leave_Level    – release threshold, ~16 below Touch_Level for
 *                            solid hysteresis (less release chatter).
 *   0x8047..0x80FE  184 config bytes
 *   0x80FF  Config_Chksum = (~sum(0x8047..0x80FE) + 1) & 0xFF
 *   0x8100  Config_Fresh – write 0x01 to commit the new config
 *
 * We use the safe read-modify-write method: read the whole config block,
 * change ONLY the profile bytes (preserving the high nibbles of Shake_Count /
 * Filter that the factory set), recompute the checksum, then write the block
 * back and set Config_Fresh.  This is the documented, low-risk way to retune
 * the controller at runtime.
 * ============================================================ */

#define GT911_I2C_PORT        I2C_NUM_0
#define GT911_I2C_ADDR_7BIT   0x5D
#define GT911_REG_CONFIG_VER  0x8047  // first byte of config block
#define GT911_REG_TOUCH_NUMBER 0x804C // max touch points (1-5)
#define GT911_REG_SHAKE_COUNT 0x804F  // de-jitter: hi nibble=touch-up, lo=touch-down confirm
#define GT911_REG_FILTER      0x8050  // hi nibble=First_Filter, lo=Normal_Filter (coord smoothing)
#define GT911_REG_NOISE_REDUCE 0x8052 // 0-15: higher = more EMI/noise immunity
#define GT911_REG_TOUCH_LEVEL 0x8053  // press threshold (higher = firmer press needed)
#define GT911_REG_LEAVE_LEVEL 0x8054  // release threshold (must be < Touch_Level)
#define GT911_REG_CONFIG_CHK  0x80FF  // checksum byte
#define GT911_REG_CONFIG_FRESH 0x8100 // commit trigger
#define GT911_CONFIG_LEN      184      // 0x8047..0x80FE inclusive
#define GT911_I2C_TIMEOUT_MS  100

// Read `len` bytes from a 16-bit GT911 register into buf. Returns true on ok.
static bool gt911_read_reg(uint16_t reg, uint8_t* buf, size_t len) {
    uint8_t reg_be[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    esp_err_t err = i2c_master_write_read_device(
        GT911_I2C_PORT, GT911_I2C_ADDR_7BIT, reg_be, 2, buf, len,
        pdMS_TO_TICKS(GT911_I2C_TIMEOUT_MS));
    return err == ESP_OK;
}

// Write `len` bytes to a 16-bit GT911 register. Returns true on ok.
static bool gt911_write_reg(uint16_t reg, const uint8_t* data, size_t len) {
    uint8_t tx[2 + 256];
    if (len > 256) return false;
    tx[0] = (uint8_t)(reg >> 8);
    tx[1] = (uint8_t)(reg & 0xFF);
    memcpy(tx + 2, data, len);
    esp_err_t err = i2c_master_write_to_device(
        GT911_I2C_PORT, GT911_I2C_ADDR_7BIT, tx, len + 2,
        pdMS_TO_TICKS(GT911_I2C_TIMEOUT_MS));
    return err == ESP_OK;
}

// One tuning profile per UI sensitivity step.  These bundle the threshold with
// the noise-filtering registers so that high sensitivity stays usable under the
// welder's EMI.  All values are deliberately conservative (well within the
// GT911's documented ranges) since they cannot be verified on bench hardware
// from here – the operator can fine-tune after a quick on-device test.
struct Gt911TouchProfile {
    uint8_t touch_level;      // 0x8053 press threshold  (higher = firmer press)
    uint8_t leave_level;      // 0x8054 release threshold (< touch_level)
    uint8_t noise_reduction;  // 0x8052 value 0-15        (higher = more immune)
    uint8_t shake_down;       // 0x804F low nibble 0-15   (touch-down confirm)
    uint8_t filter_norm;      // 0x8050 low nibble 0-15   (coordinate smoothing)
};

// Index 0 unused; sensitivity 1..10.  As sensitivity rises the threshold drops
// (more responsive) BUT noise-reduction, confirm samples and coordinate
// smoothing all rise to keep it stable against EMI.  Threshold floor is kept at
// 46 (NOT the old 19) so it never sinks into the noise floor.
static const Gt911TouchProfile kTouchProfiles[11] = {
    /* [0] unused      */ {0, 0, 0, 0, 0},
    /* [1] least sens  */ {88, 72, 1, 1, 2},
    /* [2]             */ {82, 66, 1, 1, 2},
    /* [3]             */ {76, 60, 2, 1, 3},
    /* [4]             */ {70, 54, 2, 2, 3},
    /* [5] balanced    */ {64, 48, 3, 2, 4},
    /* [6]             */ {60, 44, 4, 2, 4},
    /* [7]             */ {56, 40, 6, 3, 5},
    /* [8]             */ {52, 36, 8, 3, 6},
    /* [9]             */ {49, 32, 10, 4, 7},
    /* [10] most sens  */ {46, 28, 12, 4, 8},
};

static inline uint8_t clamp_sens(uint8_t sens) {
    if (sens < 1) return 1;
    if (sens > 10) return 10;
    return sens;
}

// Apply a full touch-sensitivity profile to the GT911 by read-modify-writing
// its config block.  Only the profile bytes are changed (high nibbles of the
// Shake_Count / Filter bytes are preserved). Safe: bails out on any I2C error
// and sets Config_Fresh LAST so a half-written config is never committed.
static bool applyTouchSensitivity(uint8_t sens) {
    sens = clamp_sens(sens);
    const Gt911TouchProfile& p = kTouchProfiles[sens];

    // 1) Read the full config block.
    uint8_t cfg[GT911_CONFIG_LEN];
    if (!gt911_read_reg(GT911_REG_CONFIG_VER, cfg, GT911_CONFIG_LEN)) {
        Serial.println(
            "[Touch] GT911 config read FAILED – sensitivity not applied");
        return false;
    }

    // 2) Compute byte offsets into the config block (base = 0x8047).
    const size_t off_tnum  = GT911_REG_TOUCH_NUMBER - GT911_REG_CONFIG_VER;   // 5
    const size_t off_shake = GT911_REG_SHAKE_COUNT  - GT911_REG_CONFIG_VER;   // 8
    const size_t off_filt  = GT911_REG_FILTER       - GT911_REG_CONFIG_VER;   // 9
    const size_t off_noise = GT911_REG_NOISE_REDUCE - GT911_REG_CONFIG_VER;   // 11
    const size_t off_touch = GT911_REG_TOUCH_LEVEL  - GT911_REG_CONFIG_VER;   // 12
    const size_t off_leave = GT911_REG_LEAVE_LEVEL  - GT911_REG_CONFIG_VER;   // 13

    // Build the new bytes, preserving high nibbles where only the low nibble is
    // ours to tune (Touch_Number low 3 bits hold the finger count).
    uint8_t new_tnum  = (uint8_t)((cfg[off_tnum] & 0xF0) | 0x01);  // 1 finger
    uint8_t new_shake = (uint8_t)((cfg[off_shake] & 0xF0) | (p.shake_down & 0x0F));
    uint8_t new_filt  = (uint8_t)((cfg[off_filt]  & 0xF0) | (p.filter_norm & 0x0F));
    uint8_t new_noise = (uint8_t)(p.noise_reduction & 0x0F);
    uint8_t new_touch = p.touch_level;
    uint8_t new_leave = p.leave_level;

    // 3) Skip the (relatively expensive) write if nothing actually changed.
    if (cfg[off_tnum] == new_tnum && cfg[off_shake] == new_shake &&
        cfg[off_filt] == new_filt && cfg[off_noise] == new_noise &&
        cfg[off_touch] == new_touch && cfg[off_leave] == new_leave) {
        Serial.printf(
            "[Touch] GT911 sens %u already applied "
            "(touch=%u leave=%u noise=%u shake=0x%02X filt=0x%02X)\n",
            sens, new_touch, new_leave, new_noise, new_shake, new_filt);
        return true;
    }

    cfg[off_tnum]  = new_tnum;
    cfg[off_shake] = new_shake;
    cfg[off_filt]  = new_filt;
    cfg[off_noise] = new_noise;
    cfg[off_touch] = new_touch;
    cfg[off_leave] = new_leave;

    // 4) Recompute checksum over the 184 config bytes.
    uint8_t sum = 0;
    for (size_t i = 0; i < GT911_CONFIG_LEN; i++) sum += cfg[i];
    uint8_t checksum = (uint8_t)((~sum) + 1);

    // 5) Write the config block back, then checksum, then commit (fresh last).
    if (!gt911_write_reg(GT911_REG_CONFIG_VER, cfg, GT911_CONFIG_LEN)) {
        Serial.println("[Touch] GT911 config write FAILED");
        return false;
    }
    if (!gt911_write_reg(GT911_REG_CONFIG_CHK, &checksum, 1)) {
        Serial.println("[Touch] GT911 checksum write FAILED");
        return false;
    }
    uint8_t fresh = 0x01;
    if (!gt911_write_reg(GT911_REG_CONFIG_FRESH, &fresh, 1)) {
        Serial.println("[Touch] GT911 config-fresh write FAILED");
        return false;
    }
    Serial.printf(
        "[Touch] GT911 sens=%u applied: Touch_Level=%u Leave_Level=%u "
        "Noise=%u Shake=0x%02X Filter=0x%02X\n",
        sens, new_touch, new_leave, new_noise, new_shake, new_filt);
    return true;
}

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

// RAW physical touch state, updated on every indev read BEFORE the
// press/release debounce filter runs.  The UI hold-to-repeat logic reads this
// (via touch_is_physically_down()) so it can stop incrementing the instant the
// finger physically lifts, instead of waiting out TOUCH_RELEASE_DEBOUNCE_MS
// (during which LVGL still reports PRESSED).  That debounce is what caused the
// +/- overshoot after release.  Kept volatile: written in the indev read,
// read from the LVGL event handlers (same task, but be explicit).
volatile bool g_touch_phys_down = false;

// Exposed to ui.cpp (see extern declaration there).
bool touch_is_physically_down() { return g_touch_phys_down; }

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
    // Publish the RAW (un-debounced) physical state for the UI repeat logic.
    g_touch_phys_down = touched;
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
static void save_weld_count_to_nvs();
static void load_weld_count_from_nvs();
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

    // Control mode (0=TIME, 1=JOULE) + Joule targets.  The Flask dashboard
    // reads control_mode to show TIME vs JOULE; without this field it can never
    // learn the mode and stays out of sync with the on-device display.
    status += ",control_mode=" + String(control_mode);
    status += ",joule_target_j=" + String(joule_target_j, 1);
    status += ",joule_max_ms=" + String(joule_max_ms);

    // ---- WiFi info (mirrors the Setup tab; consumed by the Flask dashboard) ----
    // The Flask server has no other way to know the ESP32's SSID/IP/RSSI, so we
    // forward the same values shown on the on-device Setup tab.
    {
        bool wc_connected;
        bool wc_ap;
        String wc_ssid;
        String wc_ip;
        int    wc_rssi = 0;
        if (wifi_prov_state == WIFI_PROV_AP_PORTAL) {
            wc_connected = true;            // soft-AP is up (setup mode)
            wc_ap        = true;
            wc_ssid      = ap_ssid_str;
            wc_ip        = WiFi.softAPIP().toString();
        } else if (WiFi.status() == WL_CONNECTED) {
            wc_connected = true;
            wc_ap        = false;
            wc_ssid      = WiFi.SSID();
            wc_ip        = WiFi.localIP().toString();
            wc_rssi      = WiFi.RSSI();
        } else {
            wc_connected = false;
            wc_ap        = false;
            wc_ssid      = wifi_ssid;       // the network we are trying to join
            wc_ip        = "";
        }
        // Protect the CSV k=v parser: SSIDs may legally contain ',' or '='.
        wc_ssid.replace(",", " ");
        wc_ssid.replace("=", " ");
        status += ",wifi_connected=" + String(wc_connected ? 1 : 0);
        status += ",wifi_ap_mode="   + String(wc_ap ? 1 : 0);
        status += ",wifi_ssid="      + wc_ssid;
        status += ",wifi_ip="        + wc_ip;
        status += ",wifi_rssi="      + String(wc_rssi);
    }

    // ---- System info (mirrors the Setup tab System section) ----
    status += ",fw_version=" + String(FW_VERSION);
    {
        String chip = String(ESP.getChipModel());
        chip.replace(",", " ");
        chip.replace("=", " ");
        status += ",chip_model="  + chip;
    }
    status += ",flash_size=" + String((uint32_t)ESP.getFlashChipSize());
    status += ",free_heap="  + String((uint32_t)ESP.getFreeHeap());
    status += ",uptime_s="   + String((uint32_t)(millis() / 1000UL));

    return status;
}

/**
 * Build DISPLAY packet with smoothed voltages for UI labels.
 * Sent at 1Hz to prevent flicker and uses a 5mV deadband.
 */
String buildDisplayPacket() {
    char buf[256];

    const float raw_vpack = stm_vpack;
    const float raw_cell1 = stm_cell1;
    const float raw_cell2 = stm_cell2;
    const float raw_cell3 = stm_cell3;
    const float raw_vcap = cap_v;

    float disp_vpack = tftSmoother.getDisplayValue(CH_VPACK, raw_vpack);
    float disp_cell1 = tftSmoother.getDisplayValue(CH_CELL1, raw_cell1);
    float disp_cell2 = tftSmoother.getDisplayValue(CH_CELL2, raw_cell2);
    float disp_cell3 = tftSmoother.getDisplayValue(CH_CELL3, raw_cell3);
    float disp_vcap = tftSmoother.getDisplayValue(CH_VCAP, raw_vcap);

    snprintf(buf,
             sizeof(buf),
             "DISPLAY,vpack=%.3f,vcap=%.3f,cell1=%.3f,cell2=%.3f,cell3=%.3f",
             disp_vpack,
             disp_vcap,
             disp_cell1,
             disp_cell2,
             disp_cell3);

    return String(buf);
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

    // ---- Dashboard / Joule-mode mirrored state ----
    ds.control_mode        = control_mode;
    ds.joule_target_j      = joule_target_j;
    ds.joule_max_ms        = joule_max_ms;
    ds.joule_actual_j      = joule_actual_j;
    ds.lead_resistance_mohm = lead_resistance_ohms * 1000.0f;

    // Calibration freshness is session-based (ESP32 has no RTC).
    ds.cal_valid   = cal_done_this_session;
    ds.cal_age_sec = cal_done_this_session
                         ? (uint32_t)((millis() - last_cal_ms) / 1000UL)
                         : 0U;

    // ---- Last-weld latched results ----
    ds.last_weld_valid        = last_weld_valid;
    ds.last_weld_was_joule    = last_weld_was_joule;
    ds.last_weld_target_j     = last_weld_target_j;
    ds.last_weld_energy_j     = last_weld_energy_j;
    ds.last_weld_accuracy_pct = last_weld_accuracy_pct;
    ds.last_weld_duration_ms  = last_weld_duration_ms;
    ds.last_weld_peak_a       = last_weld_peak_a;
    ds.last_weld_avg_a        = last_weld_avg_a;
    ds.last_weld_lead_loss_j  = last_weld_lead_loss_j;

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
                    // Cache latest raw STATUS2 packet for timed forwarding.
                    rawStatus2Packet = stmLine;
                    hasRawStatus2Data = true;
                    // Log to serial so STATUS2 is visible in monitor
                    Serial.printf("[STM32->UART] %s\n", stmLine.c_str());
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
                        // Authoritative SET_LEAD_R round-trip (ESP32 UI or
                        // Flask): lock out later STATUS-driven overwrites.
                        lead_r_synced_from_status = true;
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

                    // ---- Dashboard / Joule-mode mirror (display state) ----
                    if (extractIntField(stmLine, "control_mode=", iv))
                        control_mode = (uint8_t)iv;
                    if (extractFloatField(stmLine, "joule_target_j=", fv))
                        joule_target_j = fv;
                    if (extractIntField(stmLine, "joule_max_ms=", iv))
                        joule_max_ms = (uint16_t)iv;
                    // Live workpiece energy (lead-compensated control variable).
                    // Present only in JOULE-mode STATUS frames; holds the last
                    // weld's final energy between welds.
                    if (extractFloatField(stmLine, "joule_actual=", fv))
                        joule_actual_j = fv;

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

                            // Configurable lead resistance: sync from STM32
                            // STATUS only once at boot, and only for a valid
                            // non-zero value. Afterwards lead_r changes solely
                            // via calibration / SET_LEAD_R, so a stray
                            // lead_r_ohm=0 can't flash the on-screen reading.
                            if (!lead_r_synced_from_status &&
                                extractFloatField(stmLine, "lead_r_ohm=", fv) &&
                                isfinite(fv) && fv > 0.0f) {
                                lead_resistance_ohms = fv;
                                lead_r_synced_from_status = true;
                            }

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
                            // Boot-only, non-zero lead_r sync (see note above).
                            if (!lead_r_synced_from_status &&
                                extractFloatField(stmLine, "lead_r_ohm=", fv) &&
                                isfinite(fv) && fv > 0.0f) {
                                lead_resistance_ohms = fv;
                                lead_r_synced_from_status = true;
                            }
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
                    save_weld_count_to_nvs();  // persist across power cycles

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

                    // ---- Latch last-weld results for the STATUS dashboard ----
                    {
                        int   iv_done = 0;
                        float jw = NAN;   // joule workpiece energy
                        float jl = NAN;   // joule lead loss
                        float ew = NAN;   // time-mode weld energy
                        float el = NAN;   // measured lead-loss energy

                        if (extractIntField(stmLine, "total_ms=", iv_done))
                            last_weld_duration_ms = (uint32_t)iv_done;
                        if (extractFloatField(stmLine, "peak_a=", fv_done))
                            last_weld_peak_a = fv_done;
                        if (extractFloatField(stmLine, "avg_a=", fv_done))
                            last_weld_avg_a = fv_done;

                        extractFloatField(stmLine, "joule_workpiece_j=", jw);
                        extractFloatField(stmLine, "joule_loss_j=", jl);
                        extractFloatField(stmLine, "energy_weld_j=", ew);
                        extractFloatField(stmLine, "energy_lead_j=", el);

                        last_weld_was_joule = (control_mode == 1U);
                        last_weld_target_j  = joule_target_j;

                        if (last_weld_was_joule && !isnan(jw)) {
                            last_weld_energy_j = jw;
                        } else if (!isnan(ew)) {
                            last_weld_energy_j = ew;
                        }
                        if (last_weld_was_joule && !isnan(jl)) {
                            last_weld_lead_loss_j = jl;
                        } else if (!isnan(el)) {
                            last_weld_lead_loss_j = el;
                        }

                        last_weld_accuracy_pct =
                            (last_weld_target_j > 0.0f)
                                ? (100.0f * last_weld_energy_j /
                                   last_weld_target_j)
                                : 0.0f;
                        last_weld_valid = true;
                    }

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

                } else if (stmLine.startsWith("CAL_")) {
                    // Lead-resistance auto-calibration responses
                    // (CAL_STATUS / CAL_RESULT / CAL_ERROR). Forward
                    // transparently so the Pi/Flask calibration waiter and the
                    // web UI receive progress and the final measurement.
                    //
                    // Debug trace: every CAL_ line is logged so we can see
                    // exactly where the routine is (waiting for trigger,
                    // measuring, result, error/timeout) when diagnosing a
                    // "calibration never completes" report.
                    Serial.print("[Cal] STM32 -> ");
                    Serial.println(stmLine);

                    if (stmLine.startsWith("CAL_RESULT")) {
                        // Successful calibration this session: latch freshness
                        // so the CONFIG tab can show a relative age (no RTC).
                        cal_done_this_session = true;
                        last_cal_ms = millis();
                        float cal_ohm = NAN;
                        if (extractFloatField(stmLine, "CAL_RESULT=", cal_ohm)) {
                            lead_resistance_ohms = cal_ohm;
                            // Authoritative: don't let a later STATUS overwrite.
                            lead_r_synced_from_status = true;
                        }
                    }

                    // Drive the on-device touch UI calibration status line so
                    // it reflects live progress and leaves the "Calibrating..."
                    // state on a terminal CAL_RESULT / CAL_ERROR (incl. the
                    // STM32's 8 s TIMEOUT_NO_TRIGGER) instead of hanging on
                    // "keep leads shorted" forever. (Same task as
                    // lv_timer_handler() -> safe to touch LVGL directly.)
                    ui_notify_cal_message(stmLine.c_str());

                    sendToPi(stmLine);

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
        save_weld_count_to_nvs();  // persist the reset across power cycles
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
// WiFi provisioning (captive portal + NVS credentials)
// =========================
// Credentials live in their own NVS namespace ("wificfg") so they are
// independent of recipe/config and survive power cycles. NVS on the ESP32 is
// stored in a dedicated flash partition; for at-rest encryption the project
// can enable NVS encryption (flash-encryption) at the build level — the keys
// here are plain string keys, the values are whatever the user provisions.

static void save_wifi_creds_to_nvs(const String& ssid, const String& pass) {
    Preferences wprefs;
    wprefs.begin("wificfg", false);
    wprefs.putString("ssid", ssid);
    wprefs.putString("pass", pass);
    wprefs.end();
    Serial.println("[WiFi] Credentials saved to NVS");
}

static void load_wifi_creds_from_nvs() {
    Preferences wprefs;
    wprefs.begin("wificfg", true);  // read-only
    wifi_ssid = wprefs.getString("ssid", "");
    wifi_pass = wprefs.getString("pass", "");
    wprefs.end();
    Serial.printf("[WiFi] Creds from NVS: ssid='%s' (%s)\n", wifi_ssid.c_str(),
                  wifi_ssid.length() ? "set" : "EMPTY");
}

// Build the per-device AP name from the last 2 bytes of the STA MAC.
static String makeApSsid() {
    uint8_t mac[6] = {0};
    WiFi.macAddress(mac);
    char buf[24];
    snprintf(buf, sizeof(buf), "SpotWelder-%02X%02X", mac[4], mac[5]);
    return String(buf);
}

// Push the current WiFi status to the UI (Setup tab + Status indicator),
// only when something actually changed to avoid redraw churn.
static void pushWifiInfoToUi(bool force) {
    bool connected;
    bool ap_mode;
    String ssid;
    String ip;
    int rssi = 0;

    if (wifi_prov_state == WIFI_PROV_AP_PORTAL) {
        connected = true;  // AP is up
        ap_mode = true;
        ssid = ap_ssid_str;
        ip = WiFi.softAPIP().toString();
    } else if (WiFi.status() == WL_CONNECTED) {
        connected = true;
        ap_mode = false;
        ssid = WiFi.SSID();
        ip = WiFi.localIP().toString();
        rssi = WiFi.RSSI();
    } else {
        connected = false;
        ap_mode = false;
        ssid = wifi_ssid;  // the network we are trying
        ip = "";
    }

    if (!force && connected == ui_wifi_last_connected &&
        ap_mode == ui_wifi_last_ap) {
        // Still refresh RSSI/IP occasionally even if connect-state is same.
        // (handled by periodic forced calls from loop)
    }
    ui_wifi_last_connected = connected;
    ui_wifi_last_ap = ap_mode;
    ui_set_wifi_info(connected, ap_mode, ssid.c_str(), ip.c_str(), rssi);
}

// ---- Captive portal HTTP handlers ----

// Rebuild the cached <option> list of nearby networks. Called ONCE on entering
// AP mode and on explicit user rescan — never from inside a normal page load,
// so the phone's captive-portal probe gets an instant response.
static void rescanNetworks() {
    int n = WiFi.scanNetworks();
    String opts = "";
    for (int i = 0; i < n && i < 30; i++) {
        String s = WiFi.SSID(i);
        if (s.length() == 0) continue;
        opts += "<option value=\"" + s + "\">" + s + " (" +
                String(WiFi.RSSI(i)) + " dBm)</option>";
    }
    WiFi.scanDelete();
    g_scan_options = opts;
    Serial.printf("[WiFi] Portal scan cached %d network(s)\n", n);
}

static String portalPageHtml() {
    // Use the cached scan list (see rescanNetworks). No blocking scan here.
    String opts = g_scan_options;

    String html =
        "<!DOCTYPE html><html><head><meta name='viewport' "
        "content='width=device-width,initial-scale=1'>"
        "<title>SpotWelder WiFi Setup</title><style>"
        "body{font-family:sans-serif;background:#0b1623;color:#eee;margin:0;"
        "padding:20px}h2{color:#39c}label{display:block;margin:14px 0 4px}"
        "input,select{width:100%;padding:10px;font-size:16px;border-radius:8px;"
        "border:1px solid #345;background:#13202f;color:#fff;box-sizing:border-box}"
        "button{margin-top:18px;width:100%;padding:14px;font-size:18px;"
        "background:#2a8;color:#fff;border:0;border-radius:8px}"
        ".card{max-width:420px;margin:auto;background:#13202f;padding:20px;"
        "border-radius:12px}</style></head><body><div class='card'>"
        "<h2>SpotWelder WiFi Setup</h2>"
        "<form action='/save' method='POST'>"
        "<label>Network</label><select name='ssid' id='ssid'>" +
        opts +
        "</select>"
        "<label>or type SSID</label><input name='ssid_manual' "
        "placeholder='(optional, overrides above)'>"
        "<label>Password</label><input name='pass' type='password' "
        "placeholder='WiFi password'>"
        "<button type='submit'>Save &amp; Connect</button>"
        "</form>"
        "<p style='text-align:center;margin-top:14px'>"
        "<a style='color:#39c' href='/?rescan=1'>&#x21bb; Rescan networks</a>"
        "</p>"
        "<p style='color:#789;font-size:13px;margin-top:16px'>"
        "The welder will reboot and join this network. If it fails it will "
        "return to this setup screen.</p></div></body></html>";
    return html;
}

// Send the portal page with no-cache headers so the phone does not cache a
// stale "no internet" verdict and re-shows the page on later probes.
static void sendPortalPage() {
    portalServer.sendHeader("Cache-Control",
                            "no-cache, no-store, must-revalidate");
    portalServer.sendHeader("Pragma", "no-cache");
    portalServer.sendHeader("Expires", "0");
    portalServer.send(200, "text/html", portalPageHtml());
}

static void handlePortalRoot() {
    if (portalServer.hasArg("rescan")) {
        rescanNetworks();
    }
    sendPortalPage();
}

// Captive-portal-detection (CPD) probe handler.
//
// When a phone joins the AP it immediately fetches an OS-specific "is there
// internet?" URL. To make the setup page pop up reliably we serve the portal
// page DIRECTLY (HTTP 200) for these probes instead of a redirect — iOS's
// Captive Network Assistant in particular is far more reliable when its probe
// (captive.apple.com/hotspot-detect.html) returns a non-"Success" page rather
// than a 302 to a bare IP. Android/Windows treat any non-expected response as
// "captive portal present" and surface the sign-in page too.
static void handleCaptive() { sendPortalPage(); }

static void handlePortalSave() {
    String ssid = portalServer.arg("ssid");
    String manual = portalServer.arg("ssid_manual");
    if (manual.length() > 0) ssid = manual;
    String pass = portalServer.arg("pass");
    ssid.trim();

    if (ssid.length() == 0) {
        portalServer.send(200, "text/html",
                          "<html><body style='font-family:sans-serif;"
                          "background:#0b1623;color:#eee;padding:24px'>"
                          "<h3>No network selected.</h3>"
                          "<a style='color:#39c' href='/'>Go back</a>"
                          "</body></html>");
        return;
    }

    save_wifi_creds_to_nvs(ssid, pass);
    wifi_ssid = ssid;
    wifi_pass = pass;

    portalServer.send(
        200, "text/html",
        "<html><head><meta name='viewport' "
        "content='width=device-width,initial-scale=1'></head>"
        "<body style='font-family:sans-serif;background:#0b1623;color:#eee;"
        "padding:24px'><h2>Saved!</h2><p>Connecting to <b>" +
            ssid +
            "</b>...</p><p>You can close this page. The welder display will "
            "show the connection status.</p></body></html>");

    wifi_creds_just_saved = true;  // loop() will reconnect
}

// Catch-all for any unmatched request (the bulk of OS captive-portal probes
// hit random hostnames/paths that DNS has already pointed at us).
//
// METHOD 3 (serve the portal page directly with HTTP 200) is used here rather
// than a 302 redirect, because it is the most reliable across all phone OSes:
//   - iOS Captive Network Assistant renders this HTML body straight into its
//     popup; it often abandons a 302 to a bare IP.
//   - Android sees a non-204 response and raises "Sign in to network".
//   - Windows sees content != "Microsoft Connect Test" and opens the browser.
// A 302 works on some devices but is the weaker option, which is likely why the
// page did not pop up automatically before.
static void handlePortalNotFound() { sendPortalPage(); }

// ---- Provisioning mode transitions ----
static void stopApPortal() {
    portalServer.stop();
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
}

static void startApPortal() {
    Serial.println("[WiFi] Starting AP + captive portal (setup mode)");

    // Drop any STA bridge client / server.
    if (client) client.stop();
    server.stop();
    setUiConnected(false);

    ap_ssid_str = makeApSsid();

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    // Open AP (no password) so the phone can auto-join from the QR code.
    WiFi.softAP(ap_ssid_str.c_str());
    delay(200);

    // Scan ONCE now, before any client connects, so portal page loads (and the
    // phone's captive-portal probe) are served instantly from cache.
    rescanNetworks();

    // Wildcard DNS: resolve every hostname to us. TTL 0 so phones don't cache.
    dnsServer.setTTL(0);
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, "*", apIP);

    portalServer.on("/", handlePortalRoot);
    portalServer.on("/portal", handlePortalRoot);  // explicit alias
    portalServer.on("/save", HTTP_POST, handlePortalSave);
    // Captive-portal-detection probe endpoints (Android / iOS / Windows /
    // Firefox). Serving the page directly here is what makes the setup screen
    // pop up reliably across phones.
    portalServer.on("/generate_204", handleCaptive);         // Android
    portalServer.on("/gen_204", handleCaptive);              // Android (older)
    portalServer.on("/hotspot-detect.html", handleCaptive);  // iOS / macOS
    portalServer.on("/library/test/success.html", handleCaptive);  // iOS
    portalServer.on("/connecttest.txt", handleCaptive);      // Windows
    portalServer.on("/ncsi.txt", handleCaptive);             // Windows
    portalServer.on("/redirect", handleCaptive);             // Windows
    portalServer.on("/fwlink", handleCaptive);               // Windows / MSFT
    portalServer.on("/canonical.html", handleCaptive);       // Firefox / Linux
    portalServer.on("/success.txt", handleCaptive);          // Firefox / Linux
    portalServer.onNotFound(handlePortalNotFound);
    portalServer.begin();

    wifi_prov_state = WIFI_PROV_AP_PORTAL;
    wifi_creds_just_saved = false;

    Serial.printf("[WiFi] AP '%s' up at %s\n", ap_ssid_str.c_str(),
                  apIP.toString().c_str());

    // Tell the UI to show the QR provisioning view.
    //
    // The QR now encodes the SETUP-PAGE URL (http://192.168.4.1) instead of a
    // "WIFI:" auto-join string. Once the phone is connected to the SpotWelder
    // AP, scanning this URL QR makes the camera offer "Open in browser" and
    // jumps straight to the setup page — bypassing the unreliable captive-
    // portal auto-detection entirely. (Auto-join from a WIFI: QR proved flaky,
    // and the captive portal still didn't always pop, so the user had to type
    // the IP by hand. A URL QR is the most reliable path.)
    String qr = "http://" + apIP.toString();
    ui_show_wifi_setup(qr.c_str(), ap_ssid_str.c_str(),
                       apIP.toString().c_str());
    pushWifiInfoToUi(true);
}

// Begin (or restart) a STA connection attempt with the saved creds.
static void startStaConnect() {
    Serial.printf("[WiFi] Connecting (STA) to '%s'\n", wifi_ssid.c_str());
    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(
        WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    WiFi.disconnect(true);
    delay(100);
    WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
    wifi_prov_state = WIFI_PROV_STA_CONNECTING;
    wifi_sta_attempt_start_ms = millis();
    pushWifiInfoToUi(true);
}

// Public entry used by the boot sequence + the "Reconfigure WiFi" callback.
static void beginWifiProvisioning() {
    load_wifi_creds_from_nvs();
    if (wifi_ssid.length() == 0) {
        // First boot / no saved network -> go straight to the portal.
        startApPortal();
    } else {
        startStaConnect();
    }
}

// =========================
// OTA (over-the-air firmware update) support
// =========================
// A full-screen opaque LVGL overlay shows update status/progress so the operator
// sees what is happening while the device is being reflashed over WiFi.
//
// GLITCH-AVOIDANCE STRATEGY (why this is structured the way it is):
// On this Sunton board the RGB panel streams the framebuffer out of PSRAM. A
// full-frame flush (lv_refr_now) is a large synchronous PSRAM->LCD transfer. An
// OTA download simultaneously hammers the same PSRAM/SoC bus (WiFi RX + flash
// write), and if we flush the screen on every progress tick the two fight for
// the bus and the panel tears/scrambles -- exactly the glitching reported.
// So during OTA we (1) FREEZE all normal UI work via ota_in_progress, and
// (2) flush the progress bar only every ~5%, keeping bus contention minimal
// while still showing live progress.

// volatile: written from the ArduinoOTA callbacks and read in loop() to gate
// all normal UI/UART/TCP work while a transfer is running.
volatile bool ota_in_progress = false;  // set while an OTA transfer is running

// Progress-UI throttle. Each show_firmware_progress() call forces a full-frame
// 800x480 PSRAM->LCD flush (lv_refr_now); doing that on EVERY OTA packet (1000+
// times) serialises hundreds of slow synchronous flushes against the flash
// writes and stretched a 30-60 s update to 5+ minutes. We instead repaint only
// when the integer percent advances by >= 2 (plus a forced 0% and 100%), cutting
// it to ~50 flushes while still looking smooth. -1 = "nothing drawn yet".
static int ota_last_drawn_pct = -1;

// The live OTA progress UI is now the shared full-screen modal
// show_firmware_progress() / hide_firmware_progress() (defined in ui.cpp). With
// XIP-from-PSRAM enabled the CPU + RGB panel keep running while SPI flash is
// written, so we no longer hard-blank the screen during a transfer — the
// operator sees a real "ESP32 Firmware Update" progress bar instead of a black
// screen. The modal itself throttles nothing; the OTA callbacks below decide how
// often to push an update (we update per progress tick, which LVGL coalesces).

// =========================================================================
// RGB-panel drift / glitch recovery helpers
// -------------------------------------------------------------------------
// Root cause of the intermittent "whole image shifted / overscanned" glitch
// (seen on a plain reboot AND, worse, during an OTA flash):
//
//   The Sunton ESP32-8048S043C streams its 800x480x16bpp framebuffer (768 KB)
//   CONTINUOUSLY out of QSPI PSRAM (quad, 80 MHz) to feed the RGB panel. The
//   esp_lcd RGB refill / VSYNC-restart interrupt in the *precompiled* Arduino
//   libraries is NOT IRAM-safe (CONFIG_LCD_RGB_ISR_IRAM_SAFE is unset). So
//   whenever the SPI-flash cache is briefly disabled - NVS reads and WiFi RF
//   calibration loads during boot, and especially the sustained flash writes
//   during an OTA update - the panel's line DMA underruns, the scan-out pointer
//   slips, and the picture drifts. Historically only a power cycle cleared it.
//
// Two stock esp_lcd RGB APIs (available even with the precompiled libs) let us
// fix this in software:
//   * esp_lcd_rgb_panel_set_pclk()  - slow the pixel clock while flash is busy
//                                     (OTA) so the panel's PSRAM read demand
//                                     drops and the bus has room for the writes.
//   * esp_lcd_rgb_panel_restart()   - request a DMA re-sync at the next VSYNC,
//                                     recovering an already-drifted frame
//                                     without a power cycle.
// =========================================================================
#if defined(DISPLAY_ST7262_PAR)
static esp_lcd_panel_handle_t getRgbPanelHandle() {
    lv_display_t* disp = lv_display_get_default();
    if (!disp) return nullptr;
    return (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
}

// Re-align the panel scan-out DMA and repaint a full clean frame. Use after a
// flash-heavy window (boot, OTA abort) to clear any drift in software. Retained
// for manual recovery; the automatic post-boot resyncs were removed once XIP
// made boot drift a non-issue (hence the possibly-unused attribute).
static void resyncDisplayPanel(const char* reason) __attribute__((unused));
static void resyncDisplayPanel(const char* reason) {
    esp_lcd_panel_handle_t panel = getRgbPanelHandle();
    if (!panel) return;
    esp_err_t err = esp_lcd_rgb_panel_restart(panel);
    Serial.printf("[Display] RGB resync (%s): %s\n", reason ? reason : "",
                  esp_err_to_name(err));
    // Force LVGL to redraw the whole framebuffer so the restarted scan-out is
    // fed complete, correct pixel data.
    lv_obj_invalidate(lv_scr_act());
    lv_refr_now(lv_display_get_default());
}

// =========================================================================
// BOOT DISPLAY STABILIZATION  (fix for garbled/overlapping UI on cold boot)
// -------------------------------------------------------------------------
// Symptom: right after a power-on/reset the UI is drawn garbled, overlapping
// and shifted; a manual reset later corrects it. Root cause is a startup race:
//   1) smartdisplay_init() allocates the 800x480x16bpp framebuffer in PSRAM but
//      does NOT clear it, so it starts full of random power-on garbage.
//   2) The RGB panel's scan-out DMA begins streaming that garbage immediately,
//      while the early-boot flash-heavy window (NVS reads + WiFi RF-cal loads)
//      starves the non-IRAM RGB refill ISR, so the DMA scan-out pointer is
//      already drifted before LVGL even draws its first frame.
//   3) LVGL then renders the UI on top of a drifted/garbage scan-out -> the
//      "garbled, overlapping, misaligned" first screen.
//
// Fix (hardened): BEFORE any UI is created, (a) zero BOTH PSRAM framebuffers so
// there is no garbage to scan out, (b) flush the CPU cache back to PSRAM so the
// DMA (which reads PSRAM directly) actually sees the cleared bytes, then
// (c) restart the scan-out DMA TWICE (10 ms apart) to reliably re-align to a
// clean VSYNC even if the first restart lands inside a still-drifting frame.
// The caller keeps the backlight OFF across this window AND across the first
// LVGL render, so the operator never sees the power-on garbage.
//
// Why this is stronger than the previous single-restart version: on some cold
// boots a single esp_lcd_rgb_panel_restart() could be issued while the panel
// was still mid-drift, so the scan-out re-locked onto a shifted line offset and
// the UI came up rotated/garbled. memset alone also was not always visible to
// the DMA because the zeros could still be sitting in the CPU cache. Flushing
// the cache and restarting the DMA twice removes both of those race windows.
// =========================================================================
static void displayBootStabilize() {
    esp_lcd_panel_handle_t panel = getRgbPanelHandle();
    if (!panel) {
        Serial.println("[Display] boot stabilize: no RGB panel handle");
        return;
    }

    // (a) Zero BOTH PSRAM framebuffers so the panel scans out solid black
    //     instead of random power-on garbage. The RGB driver keeps two frame
    //     buffers in the double-buffered (bounce) config used here; clear both
    //     so neither buffer can ever be scanned out as garbage.
    void* fb0 = nullptr;
    void* fb1 = nullptr;
    const size_t fb_bytes = (size_t)DISPLAY_WIDTH * DISPLAY_HEIGHT * 2;  // RGB565
    bool have_fb0 = false, have_fb1 = false;
    // Try two buffers first (double-buffered configs); fall back to one.
    if (esp_lcd_rgb_panel_get_frame_buffer(panel, 2, &fb0, &fb1) == ESP_OK) {
        if (fb0) { memset(fb0, 0, fb_bytes); have_fb0 = true; }
        if (fb1) { memset(fb1, 0, fb_bytes); have_fb1 = true; }
        Serial.println("[Display] boot stabilize: cleared 2 framebuffers");
    } else if (esp_lcd_rgb_panel_get_frame_buffer(panel, 1, &fb0) == ESP_OK &&
               fb0) {
        memset(fb0, 0, fb_bytes);
        have_fb0 = true;
        Serial.println("[Display] boot stabilize: cleared 1 framebuffer");
    } else {
        Serial.println("[Display] boot stabilize: framebuffer fetch failed");
    }

    // (b) Flush the CPU data cache back to PSRAM. memset() above writes through
    //     the cache, but the RGB panel DMA reads PSRAM directly and bypasses the
    //     cache, so without this write-back the DMA could still fetch stale
    //     pre-clear garbage. esp_cache_msync(..., C2M) pushes the zeros out to
    //     PSRAM before we restart the scan-out.
    if (have_fb0 && fb0) {
        esp_cache_msync(fb0, fb_bytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    }
    if (have_fb1 && fb1) {
        esp_cache_msync(fb1, fb_bytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    }
    Serial.println("[Display] boot stabilize: PSRAM cache flushed (C2M)");

    // (c) Re-align the scan-out DMA to a clean VSYNC now that the buffer is
    //     black and flushed. Do it TWICE, 10 ms apart: a single restart can land
    //     while the panel is still mid-drift and re-lock onto a shifted offset,
    //     so a second restart after the panel has had a frame to settle makes
    //     the re-alignment reliable.
    esp_err_t err1 = esp_lcd_rgb_panel_restart(panel);
    Serial.printf("[Display] boot stabilize: DMA restart #1: %s\n",
                  esp_err_to_name(err1));
    delay(10);
    esp_err_t err2 = esp_lcd_rgb_panel_restart(panel);
    Serial.printf("[Display] boot stabilize: DMA restart #2: %s\n",
                  esp_err_to_name(err2));
}

// =========================================================================
// NOTE: the old OTA "hard-blank" workaround has been REMOVED.
// -------------------------------------------------------------------------
// Historically, an OTA / SD flash write would momentarily DISABLE the SPI-flash
// cache on every erase/program op. With code running from flash, the non-IRAM
// RGB refill ISR could not run during those windows and the panel scanned out
// garbage (the "color loop" crash), so we used to turn the backlight off and
// halt panel scan-out (displayBlankForOta/displayUnblankAfterOta) for the whole
// transfer — leaving the operator staring at a black screen.
//
// That is no longer necessary: the firmware now executes from PSRAM (XIP) with
// an IRAM-safe RGB refill path, so the CPU and panel keep running normally while
// SPI flash is being written. We can therefore paint a LIVE progress bar
// throughout the update (see show_firmware_progress()/hide_firmware_progress()).
// =========================================================================
#else
// Non-RGB panel build: these are no-ops so the call sites stay clean.
static inline void resyncDisplayPanel(const char*) {}
static inline void displayBootStabilize() {}
#endif

void setupOTA() {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.setPort(OTA_PORT);

    ArduinoOTA.onStart([]() {
        // FREEZE the normal UI. From here until reboot/error, loop() does no
        // tab rendering, no UART polling and no TCP work (see the ota_in_progress
        // guard in loop()), so the firmware download gets the PSRAM/SoC bus to
        // itself.
        ota_in_progress = true;

        String type =
            (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("[OTA] Start updating " + type);

        // Stop the TCP bridge so it cannot fight for time/heap/bus mid-update.
        if (client) client.stop();

        // Show the live progress modal. The screen stays ON for the whole
        // transfer now (XIP-from-PSRAM), so the operator watches a real bar
        // instead of a black screen.
        ota_last_drawn_pct = -1;  // reset throttle for this transfer
        show_firmware_progress("ESP32", 0, "Downloading...");
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\n[OTA] Update complete - rebooting");
        show_firmware_progress("ESP32", 100, "Update complete - rebooting...");
        // The device reboots automatically after this callback returns; the
        // clean boot re-initialises the display + restores the full UI.
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        int percent =
            (total > 0) ? (int)((progress * 100ULL) / total) : 0;
        Serial.printf("[OTA] Progress: %d%%\r", percent);
        // THROTTLE: ArduinoOTA fires this per packet (1000+ times). Each repaint
        // is a full-frame PSRAM->LCD flush that fights the flash writes, so only
        // repaint when the percent advances by >= 2 (always draw the final 100%).
        if (percent >= ota_last_drawn_pct + 2 || percent >= 100) {
            ota_last_drawn_pct = percent;
            show_firmware_progress("ESP32", percent, "Writing firmware...");
        }
    });

    ArduinoOTA.onError([](ota_error_t error) {
        ota_in_progress = false;
        Serial.printf("[OTA] Error[%u]: ", error);
        const char* msg = "Update failed!";
        if (error == OTA_AUTH_ERROR) {
            Serial.println("Auth Failed");
            msg = "Auth Failed!";
        } else if (error == OTA_BEGIN_ERROR) {
            Serial.println("Begin Failed");
            msg = "Begin Failed!";
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println("Connect Failed");
            msg = "Connect Failed!";
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println("Receive Failed");
            msg = "Receive Failed!";
        } else if (error == OTA_END_ERROR) {
            Serial.println("End Failed");
            msg = "End Failed!";
        }
        // OTA aborted: the device keeps running. Show the error on the live
        // progress modal briefly, then tear it down and return to the UI.
        show_firmware_progress("ESP32", 0, msg);
        delay(3000);
        hide_firmware_progress();
    });

    ArduinoOTA.begin();
    Serial.printf("[OTA] Ready - host '%s.local' port %d\n", OTA_HOSTNAME,
                  OTA_PORT);
}

// Start mDNS + OTA once WiFi (STA) is up. Idempotent: safe to call again after
// a WiFi reconnect — mDNS is torn down and restarted, OTA re-armed.
static void startNetworkServices() {
    static bool ota_initialized = false;

    // (Re)start the mDNS responder so the device answers to "spotwelder.local".
    MDNS.end();
    if (MDNS.begin(OTA_HOSTNAME)) {
        Serial.println("[mDNS] responder started: " OTA_HOSTNAME ".local");
    } else {
        Serial.println("[mDNS] failed to start responder");
    }

    // ArduinoOTA only needs to be set up once; begin() re-arms its listener.
    if (!ota_initialized) {
        setupOTA();
        ota_initialized = true;
    } else {
        ArduinoOTA.begin();
    }
}

// =========================
// WiFi / server maintenance — drives the provisioning state machine.
// Called every loop().
// =========================
void ensureWiFiAndServer() {
    static unsigned long last_retry = 0;
    static unsigned long last_ui_refresh = 0;
    static bool server_started = false;

    uint32_t now = millis();

    // Periodically refresh WiFi info on the UI (RSSI/IP can drift).
    if (now - last_ui_refresh > 3000) {
        last_ui_refresh = now;
        pushWifiInfoToUi(true);
    }

    switch (wifi_prov_state) {
        case WIFI_PROV_AP_PORTAL:
            dnsServer.processNextRequest();
            portalServer.handleClient();
            if (wifi_creds_just_saved) {
                // New creds arrived from the portal: tear down AP, connect.
                Serial.println("[WiFi] New creds saved -> switching to STA");
                stopApPortal();
                ui_hide_wifi_setup();
                startStaConnect();
            }
            return;

        case WIFI_PROV_STA_CONNECTING:
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("[WiFi] Connected - IP: " +
                               WiFi.localIP().toString());
                wifi_prov_state = WIFI_PROV_STA_CONNECTED;
                pushWifiInfoToUi(true);
                return;
            }
            // Timed out waiting for the saved network -> fall back to portal.
            if (now - wifi_sta_attempt_start_ms > WIFI_STA_CONNECT_TIMEOUT_MS) {
                Serial.println(
                    "[WiFi] STA connect timed out -> AP portal fallback");
                startApPortal();
            }
            return;

        case WIFI_PROV_STA_CONNECTED:
            if (WiFi.status() == WL_CONNECTED) {
                if (!server_started) {
                    Serial.println("✅ WiFi connected - IP: " +
                                   WiFi.localIP().toString());
                    Serial.println("Starting TCP server on port 8888...");
                    server.begin();
                    server.setNoDelay(true);
                    server_started = true;

                    // Now that STA is up, (re)start mDNS + arm OTA so the
                    // device is flashable over WiFi at "spotwelder.local".
                    startNetworkServices();
                }
                return;
            }
            // Lost connection -> retry STA, then portal-fallback via timeout.
            server_started = false;
            server.stop();
            if (client) client.stop();
            setUiConnected(false);
            if (now - last_retry > 5000) {
                last_retry = now;
                Serial.println("⚠️ WiFi lost; reconnecting...");
                startStaConnect();
            }
            return;
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
    prefs.putUChar("touchSens", cfg.touch_sensitivity);
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
    cfg.touch_sensitivity =
        prefs.getUChar("touchSens", cfg.touch_sensitivity);
    if (cfg.touch_sensitivity < 1 || cfg.touch_sensitivity > 10)
        cfg.touch_sensitivity = 5;

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

// ---- Weld counter persistence (survives power cycles) ----
// Stored in its own NVS namespace so it is independent of recipe/config.
static void save_weld_count_to_nvs() {
    prefs.begin("weldstats", false);
    prefs.putULong("weldCount", weld_count);
    prefs.end();
}

static void load_weld_count_from_nvs() {
    prefs.begin("weldstats", true);  // read-only
    weld_count = prefs.getULong("weldCount", 0UL);
    prefs.end();
    Serial.printf("[Weld] Counter restored from NVS: %lu\n",
                  (unsigned long)weld_count);
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

    // Apply touch sensitivity to GT911 only when it actually changed (avoids
    // re-writing the controller config block on every unrelated config edit).
    static int8_t last_touch_sens = -1;  // -1 = never applied this session
    uint8_t ts = normalized_cfg.touch_sensitivity;
    if (ts < 1 || ts > 10) ts = 5;
    if ((int8_t)ts != last_touch_sens) {
        if (applyTouchSensitivity(ts)) last_touch_sens = (int8_t)ts;
    }

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

// Callback from UI (CONFIG tab) when AUTO CALIBRATE is pressed.
// Triggers the STM32 lead-resistance auto-calibration routine.
static void onCalibrate() {
    processCommand("CAL_LEAD_START");
    Serial.println("[Cal] Lead-resistance auto-calibration started via touch UI");
}

// Callback from UI (JOULE tab) when APPLY is pressed.
//   mode_joule = true -> JOULE control mode, false -> TIME control mode
//   target_j   = target energy (J), max_ms = max-duration safety limit (ms)
static void onJouleApply(bool mode_joule, float target_j, uint16_t max_ms) {
    char buf[48];

    // 1) Control mode (SET_MODE,<0=time|1=joule>)
    snprintf(buf, sizeof(buf), "SET_MODE,%d", mode_joule ? 1 : 0);
    processCommand(String(buf));
    control_mode = mode_joule ? 1 : 0;

    // 2) Target energy
    snprintf(buf, sizeof(buf), "SET_JOULE_TARGET,%.1f", (double)target_j);
    processCommand(String(buf));
    joule_target_j = target_j;

    // 3) Max-duration safety limit
    snprintf(buf, sizeof(buf), "SET_JOULE_MAX,%u", (unsigned)max_ms);
    processCommand(String(buf));
    joule_max_ms = max_ms;

    // Push an immediate STATUS so the Flask dashboard reflects the new control
    // mode right away instead of waiting for the next periodic STATUS tick.
    sendToPi(buildStatus());

    Serial.printf("[Joule] Applied mode=%s target=%.1fJ max=%ums via touch UI\n",
                  mode_joule ? "JOULE" : "TIME", (double)target_j,
                  (unsigned)max_ms);
}

// Callback from UI (CONFIG tab) when contact delay steps change.
// SET_CONTACT_HOLD is already forwarded by onConfigChange (driven by the same
// stepper), so this only logs to avoid a duplicate command / flash write.
static void onContactDelayChange(uint8_t steps) {
    Serial.printf("[Config] Contact delay set to %d step(s) via touch UI\n",
                  (int)steps);
}

// =========================
// Setup-tab maintenance callbacks
// =========================

// "Reconfigure WiFi": tear down the current STA connection and bring up the
// AP captive portal + QR code so the user can join a different network.
static void onWifiReconfigure() {
    Serial.println("[WiFi] Reconfigure requested from Setup tab");
    startApPortal();
}

// "Restart": simply reboot the ESP32. Saved WiFi creds are retained in NVS,
// so the device will reconnect normally on the next boot.
static void onDeviceRestart() {
    Serial.println("[Maint] Restart requested from Setup tab - rebooting...");
    delay(200);
    ESP.restart();
}

// "Factory Reset": wipe every NVS namespace this firmware uses (WiFi creds,
// recipes, config, stats) and reboot. The device will come up in AP/portal
// mode because the WiFi creds are gone.
static void onFactoryReset() {
    Serial.println("[Maint] FACTORY RESET requested - clearing all NVS...");
    Preferences p;
    const char* namespaces[] = {"wificfg", "weldcfg", "weldrecipe",
                                "spotwelder", "weldstats"};
    for (const char* ns : namespaces) {
        if (p.begin(ns, false)) {
            p.clear();
            p.end();
            Serial.printf("[Maint]   cleared NVS namespace '%s'\n", ns);
        }
    }
    Serial.println("[Maint] Factory reset complete - rebooting...");
    delay(300);
    ESP.restart();
}

// ===========================================================================
// DUAL FIRMWARE UPDATE  —  microSD card  ->  ESP32 (self-flash) & STM32G474
// ---------------------------------------------------------------------------
// Two independent flows, both driven from the CONFIG-tab "Firmware Update"
// buttons:
//
//   1) updateESP32FromSD()  reads /esp32_firmware.bin and re-flashes THIS
//      ESP32 with the Arduino Update.h API (same mechanism as OTA, using the
//      spare OTA partition -> safe rollback). The RGB panel is HARD-BLANKED for
//      the write because each flash erase/program disables the SPI-flash cache
//      (the documented "color-loop" hazard); progress is reported over serial,
//      and the device reboots into the new image on success.
//
//   2) flashSTM32FromSD()   reads /stm32_firmware.bin and programs the STM32
//      over the GPIO17/18 UART link using the ST ROM bootloader protocol
//      (AN3155). Bootloader entry is SOFTWARE-only: the ESP32 sends a
//      "BOOTLOADER" command and the STM32 application jumps to system memory
//      itself (no BOOT0/NRST wires). The new app is launched with the AN3155
//      "Go" command. Because the ESP32's own flash/PSRAM are untouched here,
//      the panel stays ON and shows a live progress bar, and touch keeps
//      working throughout. The ESP32 reboots afterwards only to cleanly
//      relaunch the STM32 application data link.
//
// Both flows are REQUESTED from the LVGL button callback (which only sets a
// flag) and EXECUTED from loop(), never from inside the LVGL event dispatch —
// this avoids re-entrancy and keeps the long blocking transfer out of the
// widget event path.
// ===========================================================================

// ---- SD card on a dedicated SPI bus (does not touch the display/UART pins) --
static SPIClass g_sdSPI(HSPI);
static bool g_sd_mounted = false;

// Mount the microSD card once (idempotent). Returns true if a card is present
// and the FAT filesystem mounted.
static bool sdEnsureMounted() {
    if (g_sd_mounted) return true;
    g_sdSPI.begin(TF_SPI_SCLK, TF_SPI_MISO, TF_SPI_MOSI, TF_CS);
    // 20 MHz is a conservative, reliable rate for the on-board slot.
    if (SD.begin(TF_CS, g_sdSPI, 20000000)) {
        g_sd_mounted = true;
        Serial.printf("[SD] mounted (type=%u, size=%lluMB)\n",
                      (unsigned)SD.cardType(),
                      (unsigned long long)(SD.cardSize() / (1024ULL * 1024ULL)));
        return true;
    }
    Serial.println("[SD] mount FAILED (no card inserted or wiring issue)");
    return false;
}

// True if `path` exists on the SD card with a non-zero size; size returned via
// out_size when provided.
static bool sdFileExists(const char* path, size_t* out_size) {
    if (!sdEnsureMounted()) return false;
    File f = SD.open(path, FILE_READ);
    if (!f) return false;
    size_t sz = f.size();
    f.close();
    if (out_size) *out_size = sz;
    return sz > 0;
}

// Deferred firmware-update request (set from the UI callback, run in loop()).
enum { FW_REQ_NONE = 0, FW_REQ_ESP32 = 1, FW_REQ_STM32 = 2 };
static volatile uint8_t g_fw_request = FW_REQ_NONE;

// ===========================================================================
// FLOW 1 — ESP32 self-flash from SD (Update.h)
// ===========================================================================
// Inner worker: flashes THIS ESP32 from /esp32_firmware.bin on the SD card.
// Returns true on success (new image written & verified, ready to run on the
// next boot). Fills 'msg' with a short human-readable result in both cases.
// Does NOT reboot and does NOT show the popup - the thin wrapper
// updateESP32FromSD() owns the completion popup and the reboot.
static bool esp32FlashFromSD(char* msg, size_t msgn) {
    show_firmware_progress("ESP32", 0, "Checking SD card...");

    size_t fw_size = 0;
    if (!sdFileExists(SD_ESP32_FW_PATH, &fw_size)) {
        snprintf(msg, msgn, "%s not found on SD card", SD_ESP32_FW_PATH);
        return false;
    }
    // Sanity: a real app image is at least a few KB and fits in the 16MB flash.
    if (fw_size < 4096 || fw_size > (4u * 1024u * 1024u)) {
        snprintf(msg, msgn, "ESP32 image size out of range");
        return false;
    }

    File f = SD.open(SD_ESP32_FW_PATH, FILE_READ);
    if (!f) {
        snprintf(msg, msgn, "Cannot open ESP32 image");
        return false;
    }
    // Header sanity: every ESP32 application image starts with magic byte 0xE9.
    if (f.peek() != 0xE9) {
        f.close();
        snprintf(msg, msgn, "Not a valid ESP32 image (magic != 0xE9)");
        return false;
    }

    if (!Update.begin(fw_size)) {
        f.close();
        snprintf(msg, msgn, "Update.begin failed (%s)", Update.errorString());
        return false;
    }

    ota_last_drawn_pct = -1;  // reset throttle for this transfer
    show_firmware_progress("ESP32", 0, "Writing firmware - do NOT power off!");
    delay(600);  // give the operator time to read the warning

    // Live progress during the write. With XIP-from-PSRAM the panel keeps
    // running while flash is programmed, so the bar updates in real time (no
    // more hard-blanking the screen). THROTTLE the repaint to >= 2% steps: each
    // repaint is a full-frame PSRAM->LCD flush that competes with the flash
    // writes, so painting every chunk would massively slow the update.
    Update.onProgress([](size_t done, size_t total) {
        int pct = total ? (int)((done * 100ULL) / total) : 0;
        Serial.printf("[SD->ESP32] %d%%\r", pct);
        if (pct >= ota_last_drawn_pct + 2 || pct >= 100) {
            ota_last_drawn_pct = pct;
            show_firmware_progress("ESP32", pct, "Writing firmware...");
        }
    });

    size_t written = Update.writeStream(f);
    f.close();

    if (written != fw_size || !Update.end(true)) {
        snprintf(msg, msgn, "ESP32 flash failed (%s)", Update.errorString());
        return false;
    }

    Serial.println("\n[SD->ESP32] flash OK");
    snprintf(msg, msgn, "ESP32 firmware updated successfully. Restarting...");
    return true;
}

// Public entry (latched from the Setup-tab "UPDATE ESP32" button). Runs the
// flash, shows the modal completion popup, and reboots into the new image on
// success.
static bool updateESP32FromSD() {
    ota_in_progress = true;  // gate normal loop() work (UI/UART/TCP)
    char msg[128] = {0};
    bool ok = esp32FlashFromSD(msg, sizeof(msg));

    if (ok) {
        // Success reboots into the new image. Show 100% briefly, then stash the
        // result and let the freshly-booted firmware show the completion popup
        // once the display has settled (see the boot-popup logic in loop()).
        show_firmware_progress("ESP32", 100, "Update complete - restarting...");
        armBootFirmwarePopup(true, "ESP32", msg);
        delay(200);
        ESP.restart();  // never returns
    }
    // Failure path stays in the running app (no reboot): tear down the progress
    // modal and show the result popup right away so the operator can read the
    // error and retry. This is normal operation, so it is NOT deferred.
    hide_firmware_progress();
    show_firmware_result_popup(false, msg, "ESP32");
    ota_in_progress = false;
    return ok;
}

// ===========================================================================
// FLOW 2 — STM32 flash from SD over the ST ROM bootloader (AN3155)
// ===========================================================================

// --- AN3155 / bootloader constants ---
static const uint8_t STM_ACK = 0x79;
static const uint8_t STM_NACK = 0x1F;
static const uint8_t STM_INIT = 0x7F;
#define STM32_FLASH_BASE 0x08000000UL

// Read one byte with timeout (ms). Returns -1 on timeout.
static int stmReadByte(uint32_t timeout_ms) {
    uint32_t t0 = millis();
    while (!STM32Serial.available()) {
        if (millis() - t0 > timeout_ms) return -1;
        yield();
    }
    return STM32Serial.read();
}

// Wait for an ACK (0x79). Logs NACK/timeout/unexpected bytes for diagnostics.
static bool stmWaitAck(uint32_t timeout_ms) {
    int b = stmReadByte(timeout_ms);
    if (b == STM_ACK) return true;
    if (b == STM_NACK)
        Serial.println("[STM] NACK");
    else if (b < 0)
        Serial.println("[STM] timeout waiting for ACK");
    else
        Serial.printf("[STM] unexpected byte 0x%02X\n", b);
    return false;
}

static void stmDrainRx() {
    while (STM32Serial.available()) STM32Serial.read();
}

// Send command byte + its complement, wait for ACK.
static bool stmSendCmd(uint8_t cmd) {
    STM32Serial.write(cmd);
    STM32Serial.write((uint8_t)(cmd ^ 0xFF));
    STM32Serial.flush();
    return stmWaitAck(800);
}

// Bootloader auto-baud handshake: send 0x7F until the device answers.
static bool stmSync() {
    for (int i = 0; i < 8; i++) {
        stmDrainRx();
        STM32Serial.write(STM_INIT);
        STM32Serial.flush();
        int b = stmReadByte(500);
        if (b == STM_ACK) {
            Serial.println("[STM] sync OK (ACK)");
            return true;
        }
        if (b == STM_NACK) {
            // Already initialised this power cycle -> treat as connected.
            Serial.println("[STM] sync OK (NACK = already initialised)");
            return true;
        }
        delay(50);
    }
    return false;
}

// Get ID command (0x02) -> 12-bit product ID (G474 = 0x469).
static bool stmGetId(uint16_t* id) {
    if (!stmSendCmd(0x02)) return false;
    int n = stmReadByte(500);  // number-of-bytes-minus-1 (=1 for a 2-byte ID)
    if (n < 0) return false;
    int hi = stmReadByte(500);
    int lo = stmReadByte(500);
    if (hi < 0 || lo < 0) return false;
    if (!stmWaitAck(500)) return false;
    if (id) *id = (uint16_t)(((hi & 0xFF) << 8) | (lo & 0xFF));
    return true;
}

// Extended Erase (0x44) global mass-erase (special code 0xFFFF, checksum 0x00).
static bool stmMassErase() {
    if (!stmSendCmd(0x44)) return false;
    STM32Serial.write((uint8_t)0xFF);
    STM32Serial.write((uint8_t)0xFF);
    STM32Serial.write((uint8_t)0x00);  // checksum = 0xFF ^ 0xFF
    STM32Serial.flush();
    return stmWaitAck(30000);  // a full mass erase can take several seconds
}

// Write Memory (0x31): up to 256 bytes (len = 1..256) at `addr`.
static bool stmWriteChunk(uint32_t addr, const uint8_t* data, int len) {
    if (len < 1 || len > 256) return false;
    if (!stmSendCmd(0x31)) return false;
    uint8_t a[4] = {(uint8_t)(addr >> 24), (uint8_t)(addr >> 16),
                    (uint8_t)(addr >> 8), (uint8_t)addr};
    uint8_t acs = a[0] ^ a[1] ^ a[2] ^ a[3];
    STM32Serial.write(a, 4);
    STM32Serial.write(acs);
    STM32Serial.flush();
    if (!stmWaitAck(800)) return false;

    uint8_t n = (uint8_t)(len - 1);  // device expects (count - 1)
    uint8_t cs = n;
    for (int i = 0; i < len; i++) cs ^= data[i];
    STM32Serial.write(n);
    STM32Serial.write(data, len);
    STM32Serial.write(cs);
    STM32Serial.flush();
    return stmWaitAck(2000);
}

// Go (0x21): jump to and run the application at `addr`. In the software-
// bootloader flow this is how the freshly written STM32 application is launched
// (there is no NRST line to pulse), so it is called at the end of a flash.
static bool stmGo(uint32_t addr) {
    if (!stmSendCmd(0x21)) return false;
    uint8_t a[4] = {(uint8_t)(addr >> 24), (uint8_t)(addr >> 16),
                    (uint8_t)(addr >> 8), (uint8_t)addr};
    uint8_t cs = a[0] ^ a[1] ^ a[2] ^ a[3];
    STM32Serial.write(a, 4);
    STM32Serial.write(cs);
    STM32Serial.flush();
    return stmWaitAck(1000);
}

// Software jump-to-bootloader: ask the running STM32 application to reboot into
// its built-in ROM bootloader. Sent over the NORMAL application link (2 Mbaud,
// 8N1) BEFORE we switch the ESP32 UART to bootloader settings. The STM32
// firmware must implement a matching handler that, on receiving this line,
// calls its jump-to-system-memory routine (see the notes returned to the user).
// There is no hardware BOOT0/NRST line, so this command is the only way in.
static void sendBootloaderCommand() {
    STM32Serial.print("BOOTLOADER");
    STM32Serial.print("\r\n");
    STM32Serial.flush();
}

// Restore the normal 2 Mbaud 8N1 application link to the STM32.
static void stm32RestoreAppLink() {
    STM32Serial.end();
    STM32Serial.setRxBufferSize(8192);
    STM32Serial.begin(2000000, SERIAL_8N1, STM32_TO_ESP32_PIN,
                      ESP32_TO_STM32_PIN);
}

// Inner worker: flashes the STM32 from /stm32_firmware.bin over the AN3155 ROM
// bootloader. Returns true on success and fills 'msg' with a short result in
// both cases. Does NOT reboot and does NOT show the popup - the thin wrapper
// flashSTM32FromSD() owns the completion popup and the (always-performed)
// reboot.
static bool stm32FlashFromSD(char* msg, size_t msgn) {
    show_firmware_progress("STM32", 0, "Checking SD card...");

    size_t fw_size = 0;
    if (!sdFileExists(SD_STM32_FW_PATH, &fw_size)) {
        snprintf(msg, msgn, "%s not found on SD card", SD_STM32_FW_PATH);
        return false;
    }
    // STM32G474CE has 512 KB flash; sanity-check the image fits.
    if (fw_size < 256 || fw_size > (512u * 1024u)) {
        snprintf(msg, msgn, "STM32 image size out of range");
        return false;
    }

    File f = SD.open(SD_STM32_FW_PATH, FILE_READ);
    if (!f) {
        snprintf(msg, msgn, "Cannot open STM32 image");
        return false;
    }

    // --- Disable WiFi for the duration of the STM32 flash ---------------------
    // The SD card, STM32 UART, the 800x480 RGB display and the WiFi radio all
    // contend for the SoC/SPI bus. With WiFi active this contention starves the
    // SD card and crashes the ESP32 (~14 s into the write, with a WiFi
    // disconnect "reason 8" logged right before the crash). The STM32 flash is
    // fast (10-20 s), so we shut the radio down for the duration to remove the
    // contention entirely, then reconnect afterwards. We remember whether WiFi
    // was connected so we only reconnect if it actually was.
    bool wifi_was_connected = (WiFi.status() == WL_CONNECTED);
    show_firmware_progress("STM32", 0,
                           "Flashing STM32... (WiFi temporarily disabled)");
    WiFi.disconnect(true);  // drop association AND power down the radio
    WiFi.mode(WIFI_OFF);
    delay(100);             // let WiFi cleanly shut down before we hit the bus

    // --- Ask the STM32 app to jump to its ROM bootloader (software trigger) --
    show_firmware_progress("STM32", 0, "Requesting bootloader...");
    // Send the command over the CURRENT application link (2 Mbaud, 8N1) while it
    // is still open, then give the STM32 time to reboot into system memory
    // before we switch the ESP32 UART to AN3155 bootloader settings.
    sendBootloaderCommand();
    delay(100);  // STM32 resets into the ROM bootloader
    STM32Serial.end();
    // ST ROM bootloader: 8 data bits, EVEN parity, 1 stop; auto-baud from 0x7F.
    STM32Serial.begin(115200, SERIAL_8E1, STM32_TO_ESP32_PIN,
                      ESP32_TO_STM32_PIN);
    delay(20);

    bool ok = false;
    const char* err = nullptr;
    do {
        if (!stmSync()) {
            err = "STM32 bootloader did not respond";
            break;
        }
        uint16_t id = 0;
        if (stmGetId(&id))
            Serial.printf("[STM] product ID = 0x%03X%s\n", id,
                          (id == 0x469) ? " (G474 OK)" : "");

        show_firmware_progress("STM32", 0, "Erasing flash...");
        if (!stmMassErase()) {
            err = "STM32 mass-erase failed";
            break;
        }

        show_firmware_progress("STM32", 0, "Writing firmware...");
        // STM32-ONLY throttle: repaint only every >= 10% (vs 2% for ESP32 OTA /
        // ESP32 SD flash). The SD card and the 800x480 PSRAM->LCD flush share
        // the SPI bus; repainting too often during the fast (10-20 s) STM32
        // write starves the SD card ("no token received"), drops WiFi, and can
        // crash/reboot the ESP32. Fewer, coarser repaints keep the bus free.
        int last_drawn_pct = -1;  // throttle: only repaint on >= 10% advance
        uint32_t addr = STM32_FLASH_BASE;
        size_t done = 0;
        // Smaller 128-byte reads (the AN3155 write chunk is also 128 here) keep
        // each SD/SPI transaction short so the display + WiFi can interleave.
        uint8_t buf[128];
        bool werr = false;
        bool readErr = false;
        while (done < fw_size) {
            size_t remain = fw_size - done;
            int want = (remain > sizeof(buf)) ? (int)sizeof(buf) : (int)remain;
            int r = f.read(buf, want);
            if (r <= 0) {
                // SD read failed/short before EOF -> real error, not success.
                readErr = true;
                break;
            }
            // Let the SPI bus recover after each SD read so the display/WiFi
            // tasks get a turn; also feeds the task watchdog (delay() yields).
            delay(2);
            yield();
            // Pad the final partial word up to a 4-byte boundary with 0xFF.
            while (r % 4) buf[r++] = 0xFF;
            if (!stmWriteChunk(addr, buf, r)) {
                werr = true;
                break;
            }
            addr += r;
            done += r;
            int pct = (int)((done * 100ULL) / fw_size);
            if (pct > 100) pct = 100;
            // THROTTLE the repaint (STM32: every >= 10%, always draw 100%).
            if (pct >= last_drawn_pct + 10 || pct >= 100) {
                last_drawn_pct = pct;
                show_firmware_progress("STM32", pct, "Writing firmware...");
            }
        }
        if (readErr) {
            err = "SD card read error during STM32 write";
            break;
        }
        if (werr) {
            err = "STM32 write failed";
            break;
        }
        if (done < fw_size) {
            err = "STM32 write incomplete";
            break;
        }
        show_firmware_progress("STM32", 100, "Verifying...");
        ok = true;
    } while (0);

    f.close();

    // Launch the freshly written application with the AN3155 "Go" command
    // (there is no NRST line to pulse in the software-bootloader approach),
    // then restore the normal 2 Mbaud 8N1 application link settings.
    show_firmware_progress("STM32", 100,
                           ok ? "Launching new firmware..."
                              : (err ? err : "STM32 update failed"));
    if (ok) stmGo(STM32_FLASH_BASE);  // jump to the new app at 0x08000000
    stm32RestoreAppLink();

    // --- Re-enable WiFi now that the bus-heavy flash is done ------------------
    // Only reconnect if WiFi was up before we started. startStaConnect() puts
    // the radio back into STA mode and kicks off the association; it finishes in
    // the background (the wrapper reboots the ESP32 shortly after, which also
    // re-establishes a clean link, so this mainly keeps the radio sane in the
    // interim and covers any future non-rebooting caller).
    if (wifi_was_connected && wifi_ssid.length() > 0) {
        show_firmware_progress("STM32", 100, "Reconnecting WiFi...");
        startStaConnect();
    }

    snprintf(msg, msgn, "%s",
             ok ? "STM32 firmware updated successfully. Restarting..."
                : (err ? err : "STM32 update failed"));
    return ok;
}

// Public entry (latched from the Setup-tab "UPDATE STM32" button). Runs the
// flash, shows the modal completion popup, then reboots the ESP32 so both sides
// start from a clean, re-synced application link.
static bool flashSTM32FromSD() {
    ota_in_progress = true;
    char msg[128] = {0};
    bool ok = stm32FlashFromSD(msg, sizeof(msg));

    // Touch (GPIO19/20) was never disturbed, but the STM32 link was reopened at
    // bootloader settings and the STM32 just rebooted into the (new) app. We
    // always reboot the ESP32 so both sides start from a clean, re-synced link.
    show_firmware_progress("STM32", 100,
                           ok ? "Update complete - restarting..."
                              : "Update failed - restarting...");

    // The STM32 flow always reboots the ESP32 (to re-sync the app link). Defer
    // the result popup to the next boot rather than painting it now: showing it
    // mid-reboot glitched the first boot frame. The freshly-booted firmware
    // shows it once the display has settled (see boot-popup logic in loop()).
    armBootFirmwarePopup(ok, "STM32", msg);
    delay(200);
    ESP.restart();  // never returns; relaunches clean app link on boot
    return ok;
}

// ---- UI button callbacks: only LATCH a request; loop() runs the transfer ----
static void onUpdateEsp32FromSD() { g_fw_request = FW_REQ_ESP32; }
static void onUpdateStm32FromSD() { g_fw_request = FW_REQ_STM32; }

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
    smartdisplay_init();  // creates the RGB panel + framebuffer and runs lv_init()

    // BOOT STABILIZATION: keep the backlight OFF, then clear the (uninitialised,
    // garbage-filled) PSRAM framebuffer and re-align the scan-out DMA BEFORE any
    // UI is created. This kills the garbled/overlapping/shifted first frame seen
    // on a cold boot (see displayBootStabilize() for the full root-cause notes).
    // The backlight is turned back on further below, only after LVGL has drawn
    // the real UI into the now-clean framebuffer, so the operator never sees the
    // power-on garbage.
    smartdisplay_lcd_set_backlight(0.0f);
    displayBootStabilize();
    Serial.println("✅ Display initialized (framebuffer cleared, DMA synced)");

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

    // Render the real UI into the (already-cleared) framebuffer, then turn the
    // backlight back on. The backlight is still OFF here, so everything below
    // happens off-screen. We render TWICE with a settle delay in between:
    //
    //   * First render: draws the freshly-created widgets into the framebuffer.
    //   * 150 ms settle (was 50 ms): gives the just-restarted scan-out DMA
    //     several full panel frames (~29 Hz => ~35 ms/frame) to lock onto the
    //     clean buffer before anything is shown. The longer delay is what fixes
    //     the *intermittent* corruption — 50 ms was sometimes too short for the
    //     DMA to fully re-align on a cold boot.
    //   * Second render: repaints the whole UI over the now-settled, correctly
    //     aligned scan-out, guaranteeing the first visible frame is clean even
    //     if the first render raced an in-flight re-alignment.
    //
    // Only after both renders is the backlight enabled, so the operator never
    // sees the power-on garbage, a half-drawn frame, or a shifted/rotated UI.
    lv_timer_handler();        // let LVGL process the freshly-created widgets
    lv_refr_now(NULL);         // first synchronous full-frame flush to the panel
    delay(150);                // settle: let the re-aligned DMA lock on (was 50)
    lv_refr_now(NULL);         // second flush over the now-stable scan-out
    smartdisplay_lcd_set_backlight(1.0f);
    Serial.println("✅ Backlight on (UI dual-rendered into clean framebuffer)");

    // --- Config: Load settings from NVS into memory (Phase 1: NO STM32 sends
    // here) ---
    {
        ConfigState cfg = load_config_from_nvs();

        // Apply brightness immediately (local display only, not STM32)
        apply_brightness(cfg.brightness);

        // Apply touch sensitivity to the GT911 (controller already initialised
        // by smartdisplay_init above, so its config block is writable now).
        applyTouchSensitivity(cfg.touch_sensitivity);

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
        load_weld_count_from_nvs();  // restore lifetime weld counter

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

    // --- New dashboard / Joule / Calibration callbacks ---
    ui_set_calibrate_cb(onCalibrate);
    ui_set_joule_apply_cb(onJouleApply);
    ui_set_contact_delay_cb(onContactDelayChange);

    // --- Setup-tab (WiFi provisioning / maintenance) callbacks ---
    ui_set_wifi_reconfigure_cb(onWifiReconfigure);
    ui_set_restart_cb(onDeviceRestart);
    ui_set_factory_reset_cb(onFactoryReset);

    // --- Firmware-update (SD card) callbacks ---
    ui_set_fw_update_esp32_cb(onUpdateEsp32FromSD);
    ui_set_fw_update_stm32_cb(onUpdateStm32FromSD);
    Serial.println("[Boot] UI callbacks registered");

    // --- microSD card (firmware-update source) ---
    if (sdEnsureMounted()) {
        size_t sz = 0;
        Serial.printf("[SD] esp32_firmware.bin present: %s\n",
                      sdFileExists(SD_ESP32_FW_PATH, &sz) ? "yes" : "no");
        Serial.printf("[SD] stm32_firmware.bin present: %s\n",
                      sdFileExists(SD_STM32_FW_PATH, &sz) ? "yes" : "no");
    }

    // --- Populate static System Info on the Setup tab ---
    ui_set_system_info(FW_VERSION, ESP.getChipModel(),
                       (uint32_t)ESP.getFlashChipSize(),
                       (uint32_t)weld_count);

    // ==========================================================
    // WiFi provisioning (non-blocking, cooperative with LVGL)
    //   - Loads creds from NVS and connects (STA), OR
    //   - Starts the AP captive portal + QR code if no creds.
    // ==========================================================
    Serial.println("[Boot] 4/4 Starting WiFi provisioning + TCP services...");
    beginWifiProvisioning();

    // Pick up a firmware-update result stashed before the last warm reboot.
    // We only LATCH it here (and immediately consume the RTC flag so it can
    // never show twice); the popup itself is shown later from loop(), once the
    // display has fully settled. The magic check rejects the uninitialised RTC
    // garbage seen on a true cold power-on.
    if (rtc_fw_popup_magic == FW_BOOT_POPUP_MAGIC) {
        boot_fw_popup_pending = true;
        boot_fw_popup_success = rtc_fw_popup_success;
        snprintf(boot_fw_popup_device, sizeof(boot_fw_popup_device), "%s",
                 rtc_fw_popup_device);
        snprintf(boot_fw_popup_msg, sizeof(boot_fw_popup_msg), "%s",
                 rtc_fw_popup_msg);
        rtc_fw_popup_magic = 0;  // consume: one-shot
        Serial.printf("[Boot] Deferred firmware popup pending (%s, %s)\n",
                      boot_fw_popup_device,
                      boot_fw_popup_success ? "OK" : "FAIL");
    }

    setup_done_ms = millis();
    Serial.println(
        "[Boot] Boot complete - waiting for STM32 BOOT/fallback sync");
    Serial.println("===========================================");
}

// Loop
// =========================
static unsigned long lv_last_tick = 0;

void loop() {
    // --- LVGL tick (must run before any early-return so timing stays sane) ---
    unsigned long now_ms = millis();
    lv_tick_inc(now_ms - lv_last_tick);
    lv_last_tick = now_ms;

    // --- Service OTA FIRST ---------------------------------------------------
    // ArduinoOTA.handle() is cheap when idle. When a transfer starts it BLOCKS
    // here for the whole download, driving the progress overlay from its own
    // (throttled) callbacks. The ota_in_progress guard below is the safety net
    // for the brief window before handle() blocks and for the post-error path:
    // while an OTA is active we skip ALL normal work (UI render, UART, TCP) so
    // the download owns the PSRAM/SoC bus and the screen does not glitch.
    if (WiFi.status() == WL_CONNECTED) {
        ArduinoOTA.handle();
    }
    if (ota_in_progress) {
        delay(10);  // yield to WiFi/flash; no UI/UART/TCP during the update
        return;
    }

    // --- Deferred firmware-update requests (run OUTSIDE the LVGL event path) -
    // The CONFIG-tab buttons only latch g_fw_request; the actual (long,
    // blocking) flash runs here so it never re-enters lv_timer_handler. Both
    // flows reboot on completion; the ESP32 error path returns here with the
    // panel/touch intact so the operator can read the message and retry.
    if (g_fw_request != FW_REQ_NONE) {
        uint8_t req = g_fw_request;
        g_fw_request = FW_REQ_NONE;
        if (req == FW_REQ_ESP32)
            updateESP32FromSD();
        else if (req == FW_REQ_STM32)
            flashSTM32FromSD();
        return;
    }

    // --- Normal UI servicing (only when NOT doing OTA) ---
    lv_timer_handler();

    // --- Post-boot display drift recovery (REMOVED) -------------------------
    // Previously the panel was re-aligned 3x over the first ~9 s (at 2/5/9 s) to
    // recover from the scan-out "drift" caused by the non-IRAM-safe RGB refill
    // ISR being starved during the flash-heavy early boot (NVS + WiFi RF cal).
    // That root cause is now fixed: the firmware executes from PSRAM (XIP) with
    // an IRAM-safe refill path, so the panel no longer drifts on boot. The three
    // periodic resyncs were therefore redundant and each caused a brief visible
    // flicker, so they have been removed for a clean, flicker-free startup. The
    // resyncDisplayPanel() helper is retained for manual recovery if ever needed.

    // --- Deferred firmware-update completion popup --------------------------
    // A firmware update that rebooted the ESP32 stashed its result in RTC
    // memory; setup() latched it into boot_fw_popup_pending. Show it ONCE, after
    // a short settle (FW_BOOT_POPUP_DELAY_MS, now 300ms) post-boot — the
    // framebuffer clear + dual DMA restart already ran in setup() and the panel
    // is solid from the first frame (XIP). This is gated to the boot window and
    // runs at most once per boot, so it never interferes with a running update.
    if (boot_fw_popup_pending && setup_done_ms > 0 &&
        (now_ms - setup_done_ms) >= FW_BOOT_POPUP_DELAY_MS) {
        boot_fw_popup_pending = false;  // one-shot
        show_firmware_result_popup(boot_fw_popup_success, boot_fw_popup_msg,
                                   boot_fw_popup_device);
        Serial.println("[Boot] Showed deferred firmware-update popup");
    }

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

    // Dual-path voltage forwarding over TCP:
    // 1) raw STATUS2 at 500ms for high-resolution graph/calc consumers
    if ((now - lastStatus2Forward) > STATUS2_INTERVAL) {
        if (hasRawStatus2Data) {
            sendToPi(rawStatus2Packet);
            lastStatus2Forward = now;
        }
    }

    // 2) smoothed DISPLAY at 1000ms for stable UI labels
    if ((now - lastDisplayUpdate) > DISPLAY_INTERVAL) {
        if (hasRawStatus2Data) {
            sendToPi(buildDisplayPacket());
            lastDisplayUpdate = now;
        }
    }

    if ((now - lastStatusReqMs) >= 1000) {
        requestStm32Status();
        lastStatusReqMs = now;
    }

    // Guaranteed periodic STATUS push to Flask (every 2s), INDEPENDENT of the
    // STM32 STATUS response path above. The normal path only forwards a STATUS
    // to Flask when the STM32 replies; if the STM32 is briefly busy/quiet, the
    // dashboard could stall on stale values. This fallback ensures control_mode,
    // joule_target_j and the rest are pushed continuously even with no activity.
    static uint32_t lastStatusPushMs = 0;
    if (client && client.connected() && (now - lastStatusPushMs) >= 2000) {
        if ((now - lastStatusForwardMs) >= STATUS_FORWARD_MIN_INTERVAL_MS) {
            String s = buildStatus();
            Serial.printf("[TCP] Periodic STATUS -> Flask (mode=%u target=%.1fJ)\n",
                          (unsigned)control_mode, (double)joule_target_j);
            sendToPi(s);
            lastStatusForwardMs = now;
        }
        lastStatusPushMs = now;
    }

    if (WiFi.status() == WL_CONNECTED) {
        WiFiClient newClient = server.accept();
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
            Serial.printf("[TCP] Flask connected -> sending STATUS immediately "
                          "(mode=%u target=%.1fJ)\n",
                          (unsigned)control_mode, (double)joule_target_j);
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
    // (Unreachable during OTA: loop() returns early above while ota_in_progress.)
    updateScreenDisplay();

    delay(5);
}

