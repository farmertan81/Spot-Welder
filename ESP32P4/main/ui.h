#ifndef UI_H
#define UI_H

#include <lvgl.h>

// ============================================================
// UI PUBLIC API
// Called from main.cpp to create and update the on-screen UI.
// ============================================================

// Forward-declared state struct so UI can read welder status
struct WelderDisplayState {
    float pack_voltage;
    float temperature;
    float charger_current;
    float cell1_v, cell2_v, cell3_v;

    // Phase 1B voltage/energy telemetry (from STM32 UART parser in main.cpp)
    float weld_v;
    float cap_v;
    float weld_v_b, weld_v_a;
    float cap_v_b, cap_v_a;
    float weld_v_drop;
    float cap_v_drop;
    float energy_cap_j;
    float energy_weld_j;
    float energy_loss_j;

    bool armed;
    bool welding;
    bool charging;
    uint32_t weld_count;

    // Recipe fields (authoritative – set by main.cpp from its globals)
    uint8_t  weld_mode;        // 1=single, 2=double, 3=triple
    uint16_t pulse_d1;
    uint16_t pulse_gap1;
    uint16_t pulse_d2;
    uint16_t pulse_gap2;
    uint16_t pulse_d3;
    uint8_t  power_pct;        // 50-100
    bool     preheat_enabled;
    uint16_t preheat_ms;
    uint8_t  preheat_pct;
    uint16_t preheat_gap_ms;

    uint8_t trigger_mode;       // 1=pedal, 2=contact
    uint8_t contact_hold_steps;
    bool    contact_with_pedal; // require contact detection when using pedal trigger

    // ---- Dashboard / Joule-mode telemetry (set by main.cpp) ----
    uint8_t  control_mode;        // 0=TIME, 1=JOULE
    float    joule_target_j;      // configured target energy (J)
    uint16_t joule_max_ms;        // configured max-duration safety limit (ms)
    float    joule_actual_j;      // live/last workpiece energy delivered (J)
    float    lead_resistance_mohm;// active lead resistance (mΩ)
    bool     cal_valid;           // true if a calibration has occurred this session
    uint32_t cal_age_sec;         // seconds since last calibration (this session)

    // ---- Last-weld results (latched on EVENT,WELD_DONE) ----
    bool     last_weld_valid;     // true once at least one weld has completed
    bool     last_weld_was_joule; // control_mode at the time of the weld
    float    last_weld_target_j;  // joule target at the time of the weld
    float    last_weld_energy_j;  // workpiece energy delivered (J)
    float    last_weld_accuracy_pct; // 100 * energy / target (joule mode)
    uint32_t last_weld_duration_ms;  // total pulse duration (ms)
    float    last_weld_peak_a;    // peak current (A)
    float    last_weld_avg_a;     // average current during pulse (A)
    float    last_weld_lead_loss_j;  // energy lost in leads (J)
};

// ============================================================
// CONFIG STATE – 6 global settings (Config tab v1 + contact_with_pedal)
// ============================================================
struct ConfigState {
    bool    hold_to_repeat;       // ON/OFF for +/- button hold-repeat
    uint8_t time_step_ms;         // 1, 5, or 10 ms
    uint8_t power_step_pct;       // 1, 5, or 10 %
    bool    load_last_on_boot;    // ON/OFF – restore last settings at startup
    uint8_t brightness;           // 0=LOW, 1=MED, 2=HIGH
    bool    contact_with_pedal;   // ON/OFF – require contact detection when using pedal trigger
    uint8_t contact_hold_steps;   // 1-10 (0.5s per step) – probe/contact hold time
    float   lead_resistance_mohm; // 0.5-5.0 mΩ
    uint8_t touch_sensitivity;    // 1-10 (10=most sensitive/lightest touch) – GT911 touch threshold
};

// Default config values
static inline ConfigState config_defaults() {
    ConfigState c;
    c.hold_to_repeat    = true;
    c.time_step_ms      = 1;
    c.power_step_pct    = 5;
    c.load_last_on_boot = true;
    c.brightness        = 2;  // HIGH
    c.contact_with_pedal = false;
    c.contact_hold_steps = 2;      // default 1.0s
    c.lead_resistance_mohm = 2.0f; // default 2.0 mΩ
    c.touch_sensitivity = 5;       // default mid sensitivity
    return c;
}

// Callback type for arm toggle from UI
typedef void (*arm_toggle_cb_t)(bool arm);

// Callback type for recipe apply from Pulse tab
typedef void (*recipe_apply_cb_t)(
    uint8_t mode, uint16_t d1, uint16_t gap1, uint16_t d2, uint16_t gap2, uint16_t d3,
    uint8_t power_pct,
    bool preheat_en, uint16_t preheat_ms, uint8_t preheat_pct, uint16_t preheat_gap_ms
);

// Callback type for config changes from Config tab
typedef void (*config_change_cb_t)(const ConfigState& cfg);

// Callback type for trigger source changes from Status tab
// trigger_mode: 1=Pedal, 2=Probe Contact
typedef void (*trigger_source_cb_t)(uint8_t trigger_mode);

// Callback type for weld counter reset (tap on weld counter display)
typedef void (*weld_count_reset_cb_t)(void);

// Callback type for contact-with-pedal toggle changes from Config tab
typedef void (*contact_with_pedal_cb_t)(bool enabled);

// Callback type for AUTO CALIBRATE button (Config tab calibration section)
typedef void (*calibrate_cb_t)(void);

// Callback type for Joule-tab APPLY: target energy (J) + max-duration (ms).
// mode_joule selects control mode (true=JOULE, false=TIME).
typedef void (*joule_apply_cb_t)(bool mode_joule, float target_j,
                                 uint16_t max_ms);

// Callback type for contact/probe delay change from Config tab (+/-).
// steps: 1-10, each step = 0.5 s.
typedef void (*contact_delay_cb_t)(uint8_t steps);

// ---- Setup tab (WiFi provisioning + maintenance) callbacks ----
// "Reconfigure WiFi": main.cpp should drop STA, start the AP + captive
// portal, and call ui_show_wifi_setup() with the QR payload.
typedef void (*wifi_reconfigure_cb_t)(void);
// "Restart Device": main.cpp should ESP.restart().
typedef void (*device_restart_cb_t)(void);
// "Factory Reset": main.cpp should erase all NVS (incl. WiFi creds) + restart.
typedef void (*factory_reset_cb_t)(void);

// Firmware-update buttons in the CONFIG tab. The callback should ONLY latch a
// request flag; the actual (long, blocking) flash operation must run later from
// loop(), OUTSIDE the LVGL event/callback path. See src/main.cpp.
typedef void (*fw_update_cb_t)(void);

// Phase 1B shared telemetry globals (defined in src/main.cpp)
// Kept here so ui.cpp and other modules can read the live parser outputs.
extern float weld_v;
extern float cap_v;
extern float weld_v_b;
extern float weld_v_a;
extern float cap_v_b;
extern float cap_v_a;
extern float vcap_b;  // legacy alias for weld_v_b
extern float vcap_a;  // legacy alias for weld_v_a
extern float energy_cap_j;
extern float energy_weld_j;
extern float energy_loss_j;

// Initialize the 5-screen tabview UI.  Call once after smartdisplay_init().
void ui_init(arm_toggle_cb_t on_arm_toggle, recipe_apply_cb_t on_recipe_apply);

// Set callback for config changes (call before or after ui_init)
void ui_set_config_cb(config_change_cb_t cb);

// Load config into UI (call after ui_init to set initial values from NVS)
void ui_load_config(const ConfigState& cfg);

// Get current config state from UI
ConfigState ui_get_config();

// Adopt the STM32's flash-persisted controller settings (reported via STATUS)
// into the Config tab. Called once at startup ("load last settings on boot")
// so the UI reflects what the controller actually loaded from its flash,
// instead of overwriting it with hard-coded ESP32 defaults. Updates only the
// controller-owned fields (contact hold, contact-with-pedal, lead R); UI-only
// fields (brightness, step sizes, etc.) are left untouched.
void ui_sync_persisted_from_status(const WelderDisplayState& st);

// Set callback for trigger source changes (call before or after ui_init)
void ui_set_trigger_source_cb(trigger_source_cb_t cb);

// Set callback for weld counter reset (call before or after ui_init)
void ui_set_weld_count_reset_cb(weld_count_reset_cb_t cb);

// Set callback for contact-with-pedal toggle (call before or after ui_init)
void ui_set_contact_with_pedal_cb(contact_with_pedal_cb_t cb);

// Set callback for the AUTO CALIBRATE button (call before or after ui_init)
void ui_set_calibrate_cb(calibrate_cb_t cb);

// Set callback for the Joule-tab APPLY button (call before or after ui_init)
void ui_set_joule_apply_cb(joule_apply_cb_t cb);

// Set callback for the Config-tab contact/probe delay +/- (call before/after ui_init)
void ui_set_contact_delay_cb(contact_delay_cb_t cb);

// Push latest data into the UI.  Call from loop() at ~20 Hz.
void ui_update(const WelderDisplayState& st);

// Query whether Pulse tab has unsaved (draft ≠ applied) changes.
bool ui_has_pending_changes();

// Force-sync draft settings to match applied settings, clear dirty flag.
// Call after boot config is echoed back from STM32 to prevent false
// "unsaved changes" indicator.
void ui_mark_settings_applied();


// Notify the UI that a hardware-level touch is active.
// Called from main.cpp based on the touch driver state machine.
// When active, ui_update() will skip ALL label/widget updates to prevent
// redraw churn from competing with LVGL's touch input processing.
void ui_set_touch_active(bool active);

// Feed a raw STM32 lead-resistance calibration line (one of:
//   CAL_STATUS=<state>, CAL_RESULT=<ohms>, CAL_ERROR=<reason>)
// into the UI so the CONFIG tab's calibration status line reflects live
// progress and terminates on success/error/timeout instead of being stuck
// on "Calibrating... keep leads shorted".  Safe to call from loop()/UART
// poll (same task as lv_timer_handler).
void ui_notify_cal_message(const char* line);

// ============================================================
// SETUP TAB API (WiFi status, system info, maintenance, provisioning)
// ============================================================

// Register Setup-tab maintenance callbacks (call before or after ui_init).
void ui_set_wifi_reconfigure_cb(wifi_reconfigure_cb_t cb);
void ui_set_restart_cb(device_restart_cb_t cb);
void ui_set_factory_reset_cb(factory_reset_cb_t cb);

// ============================================================
// FIRMWARE UPDATE API (CONFIG tab "FIRMWARE UPDATE" section)
// ============================================================

// Register the two firmware-update button callbacks (call before/after ui_init).
//   esp32 : "Update ESP32 from SD"  -> flashes /esp32_firmware.bin onto THIS MCU
//   stm32 : "Update STM32 from SD"  -> flashes /stm32_firmware.bin onto the STM32
void ui_set_fw_update_esp32_cb(fw_update_cb_t cb);
void ui_set_fw_update_stm32_cb(fw_update_cb_t cb);

// Show / hide the firmware-update progress bar (hidden by default). Safe to call
// from the same task as lv_timer_handler() (i.e. from loop()/flash routines).
void ui_fw_show_progress(bool show);

// Set the firmware-update progress bar value (0..100). Forces an immediate
// refresh so progress is visible even while the flash routine blocks loop().
void ui_fw_set_progress(int percent);

// Set the firmware-update status line text. Forces an immediate refresh.
void ui_fw_set_status(const char* text);

// Live full-screen firmware-update progress modal (shown DURING an update —
// OTA over WiFi, or an SD-card flash of the ESP32 / STM32). Builds itself on the
// first call and updates in place on subsequent calls. Now that the ESP32 runs
// from PSRAM (XIP) during flash writes, the panel stays live so the bar can be
// painted throughout the transfer (no more hard-blanking the screen).
//   device      : short device name shown as the title ("ESP32" / "STM32")
//   percent     : 0..100 progress (clamped); drives the bar + the "NN%" text
//   status_text : status line under the bar (e.g. "Writing firmware..."); pass
//                 NULL/"" to leave the current status unchanged.
// Forces an immediate refresh so progress is visible even while the flash/OTA
// routine blocks loop(). Call hide_firmware_progress() when finished.
void show_firmware_progress(const char* device, int percent,
                            const char* status_text);

// Tear down the live firmware-update progress modal. Safe to call more than once.
void hide_firmware_progress(void);

// Show a modal "firmware update complete" popup over the whole UI.
//   success : true  -> green check icon + success styling
//             false -> red cross icon + failure styling
//   message : result detail line (e.g. "ESP32 updated OK - restarting...")
//   device  : short device name shown as the title (e.g. "ESP32" or "STM32")
// The popup is modal (blocks touch to everything behind it), has an "OK" button
// that closes it immediately, and auto-dismisses after 5 seconds via an
// lv_timer. All LVGL objects are cleaned up when it closes. Safe to call from
// the same task that runs lv_timer_handler().
void show_firmware_result_popup(bool success, const char* message,
                                const char* device);

// Update the live WiFi status shown on the Setup tab + the Status-tab
// indicator. Call whenever WiFi state changes (connect / disconnect / AP).
//   connected : true if associated to a STA network (or AP is up)
//   ap_mode   : true if running the provisioning Access Point
//   ssid      : network name (STA) or AP name (AP mode); may be ""
//   ip        : dotted-quad IP string; may be ""
//   rssi      : signal strength in dBm (0 if unknown / AP mode)
void ui_set_wifi_info(bool connected, bool ap_mode, const char* ssid,
                      const char* ip, int rssi);

// Set the (mostly static) system-info fields shown on the Setup tab.
// Call once at boot; uptime/heap are refreshed via ui_update().
void ui_set_system_info(const char* fw_version, const char* chip_model,
                        uint32_t flash_size_bytes, uint32_t lifetime_welds);

// Enter the WiFi-provisioning view on the Setup tab: shows a QR code that
// encodes qr_payload (a WIFI:... join string or a URL), plus the AP name and
// portal IP as text + instructions. Switches the tabview to the Setup tab.
void ui_show_wifi_setup(const char* qr_payload, const char* ap_ssid,
                        const char* portal_ip);

// Leave the provisioning view and return the Setup tab to its normal
// (status/info/maintenance) layout.
void ui_hide_wifi_setup();

#endif // UI_H