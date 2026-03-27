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

// Callback type for arm toggle from UI
typedef void (*arm_toggle_cb_t)(bool arm);

// Callback type for recipe apply from Pulse tab
typedef void (*recipe_apply_cb_t)(
    uint8_t mode, uint16_t d1, uint16_t gap1, uint16_t d2, uint16_t gap2, uint16_t d3,
    uint8_t power_pct,
    bool preheat_en, uint16_t preheat_ms, uint8_t preheat_pct, uint16_t preheat_gap_ms
);

// Initialize the 5-screen tabview UI.  Call once after smartdisplay_init().
void ui_init(arm_toggle_cb_t on_arm_toggle, recipe_apply_cb_t on_recipe_apply);

// Push latest data into the UI.  Call from loop() at ~20 Hz.
void ui_update(const WelderDisplayState& st);

// Query whether Pulse tab has unsaved (draft ≠ applied) changes.
bool ui_has_pending_changes();

// Notify the UI that a hardware-level touch is active.
// Called from main.cpp based on the touch driver state machine.
// When active, ui_update() will skip ALL label/widget updates to prevent
// redraw churn from competing with LVGL's touch input processing.
void ui_set_touch_active(bool active);

#endif // UI_H
