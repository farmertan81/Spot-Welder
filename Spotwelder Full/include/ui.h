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
};

// ============================================================
// CONFIG STATE – 6 global settings (Config tab v1 + contact_with_pedal)
// ============================================================
struct ConfigState {
    bool    hold_to_repeat;     // ON/OFF for +/- button hold-repeat
    uint8_t time_step_ms;       // 1, 5, or 10 ms
    uint8_t power_step_pct;     // 1, 5, or 10 %
    bool    load_last_on_boot;  // ON/OFF – restore last settings at startup
    uint8_t brightness;         // 0=LOW, 1=MED, 2=HIGH
    bool    contact_with_pedal; // ON/OFF – require contact detection when using pedal trigger
    uint8_t contact_hold_steps; // 1-6 (0.5s per step) – probe/contact hold time
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

// Initialize the 5-screen tabview UI.  Call once after smartdisplay_init().
void ui_init(arm_toggle_cb_t on_arm_toggle, recipe_apply_cb_t on_recipe_apply);

// Set callback for config changes (call before or after ui_init)
void ui_set_config_cb(config_change_cb_t cb);

// Load config into UI (call after ui_init to set initial values from NVS)
void ui_load_config(const ConfigState& cfg);

// Get current config state from UI
ConfigState ui_get_config();

// Set callback for trigger source changes (call before or after ui_init)
void ui_set_trigger_source_cb(trigger_source_cb_t cb);

// Set callback for weld counter reset (call before or after ui_init)
void ui_set_weld_count_reset_cb(weld_count_reset_cb_t cb);

// Set callback for contact-with-pedal toggle (call before or after ui_init)
void ui_set_contact_with_pedal_cb(contact_with_pedal_cb_t cb);

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

#endif // UI_H