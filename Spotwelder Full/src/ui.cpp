/**
 * ui.cpp – Phase 2B: Production Pulse Tab with Anti-Shudder Fix
 *
 * ROOT CAUSE FIX:  lv_conf.h has LV_SPRINTF_USE_FLOAT = 0, so LVGL's
 * internal lv_snprintf does NOT support %f / %.2f etc.  All float formatting
 * uses standard C snprintf() into a local char buffer.
 *
 * ANTI-FLICKER:  ui_update() uses static "previous value" variables and
 * is gated by hardware touch state to prevent redraws during touch.
 *
 * ARM BUTTON:  Uses plain lv_obj (NOT lv_button) to avoid LVGL theme
 * GROW/transition animations that caused whole-screen twitch.
 *
 * ANTI-SHUDDER (Phase 2B fix):
 *   Diagnostic isolation confirmed that lv_button_create() (themed widget)
 *   causes shudder every 2-3 presses, while plain lv_obj_create() with
 *   lv_obj_remove_style_all() is completely stable.
 *
 *   ALL interactive controls now use the "Widget A" pattern:
 *     - lv_obj_create() + lv_obj_remove_style_all() + LV_OBJ_FLAG_CLICKABLE
 *     - NO lv_button_create() anywhere
 *     - make_interaction_safe() on all clickable/draggable widgets
 *     - Touch-aware ui_update() blocks rendering during active touch
 *     - Post-touch cooldown (reduced refresh rate briefly after release)
 *     - Tabview swipe/drag disabled to prevent scroll-induced redraws
 *
 * PULSE TAB:
 *   Draft/Apply model – user edits draft values on Pulse tab, clicks Apply
 *   to send recipe to STM32 via callback.  Applied state syncs FROM
 *   WelderDisplayState (authoritative source in main.cpp).
 *   ARM control remains available even when draft ≠ applied.
 *
 * CONFIG TAB (v1):
 *   5 global settings in 3 sections (Editing, Startup, Display).
 *   Hold-to-repeat, Time Step, Power Step, Load Last Settings, Brightness.
 *   Uses Widget A pattern for all controls (anti-shudder safe).
 *   Step sizes drive Pulse tab +/- buttons globally.
 *   Hold-to-repeat uses PRESSED/PRESSING events with millis() timing.
 *
 * Provides:
 *   Tab 0  "Status"     – Weld Counter, Trigger Source, Pack V, Temp, Charger
 * I, Cell voltages, Arm/Disarm toggle Tab 1  "Pulse"      – Mode, Durations,
 * Power (+/-), Preheat, Apply Tab 2  "Telemetry"  – Weld/cap voltage drop and
 * energy diagnostics Tab 3  "Config"     – Scrollable: Hold-to-repeat,
 * Time/Power Step, Contact Hold Time, Boot, Brightness Tab 4  "Logs"       –
 * placeholder
 */

#include "ui.h"

#include <Arduino.h>
#include <lvgl.h>
#include <math.h>
#include <stdio.h>   // for standard snprintf/sscanf (supports %f)
#include <string.h>  // for strncmp (calibration message parsing)

// ============================================================
// COLORS  (dark-theme, matching web UI)
// ============================================================
#define C_BG lv_color_hex(0x1A1A2E)
#define C_CARD lv_color_hex(0x222240)
#define C_ACCENT lv_color_hex(0xFF6600)
#define C_GREEN lv_color_hex(0x00CC66)
#define C_RED lv_color_hex(0xFF3333)
#define C_YELLOW lv_color_hex(0xFFDD44)
#define C_WHITE lv_color_hex(0xFFFFFF)
#define C_GREY lv_color_hex(0x888888)
#define C_DARK_GREY lv_color_hex(0x333355)

// ============================================================
// CALLBACKS
// ============================================================
static arm_toggle_cb_t _arm_cb = nullptr;
static recipe_apply_cb_t _recipe_cb = nullptr;
static config_change_cb_t _config_cb = nullptr;
static trigger_source_cb_t _trigger_cb = nullptr;
static weld_count_reset_cb_t _weld_reset_cb = nullptr;
static contact_with_pedal_cb_t _cwp_cb = nullptr;
static calibrate_cb_t _calibrate_cb = nullptr;
static joule_apply_cb_t _joule_apply_cb = nullptr;
static contact_delay_cb_t _contact_delay_cb = nullptr;
static wifi_reconfigure_cb_t _wifi_reconfigure_cb = nullptr;
static device_restart_cb_t _restart_cb = nullptr;
static factory_reset_cb_t _factory_reset_cb = nullptr;

// ============================================================
// GLOBAL CONFIG STATE (Config tab v1)
// ============================================================
static ConfigState _cfg;

// Forward declarations for config-driven updates
static void apply_time_step_to_spinboxes();
static void notify_config_changed();

// ============================================================
// ANTI-SHUDDER: Touch state tracking
// ============================================================
/* _hw_touch_active is driven by the hardware touch driver in main.cpp via
 * ui_set_touch_active() — this catches ALL touches, not just widget hits.
 * _widget_touch_active is a secondary flag from widget events
 * (belt-and-suspenders).
 */
static volatile bool _hw_touch_active = false;
static volatile bool _widget_touch_active = false;
static uint32_t _touch_release_ms = 0;

void ui_set_touch_active(bool active) {
    _hw_touch_active = active;
    if (!active) {
        _touch_release_ms = millis();
    }
}

// ============================================================
// ANTI-SHUDDER: Event handlers for interaction safety
// ============================================================

/* Prevent ANY touch on interactive elements from propagating to tabview/scroll.
 * Stops gesture recognition and scroll events from reaching parent containers.
 */
static void on_stop_bubble(lv_event_t* e) { lv_event_stop_bubbling(e); }

static void on_touch_begin(lv_event_t* e) {
    _widget_touch_active = true;
    lv_event_stop_bubbling(e);
}
static void on_touch_end(lv_event_t* e) {
    _widget_touch_active = false;
    _touch_release_ms = millis();
}

/* Register anti-bubble + touch-tracking events on an interactive obj.
 * Also removes scroll/gesture flags to prevent parent container redraws.
 * Call on ALL clickable/draggable widgets (buttons, sliders, switches, etc.)
 */
static void make_interaction_safe(lv_obj_t* obj) {
    if (!obj) return;
    /* Stop bubbling for ALL touch-related events */
    lv_obj_add_event_cb(obj, on_stop_bubble, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(obj, on_stop_bubble, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(obj, on_stop_bubble, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(obj, on_stop_bubble, LV_EVENT_GESTURE, nullptr);
    /* Track active touch for ui_update throttle */
    lv_obj_add_event_cb(obj, on_touch_begin, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(obj, on_touch_end, LV_EVENT_RELEASED, nullptr);
    /* Clear gesture bubble flag so parent doesn't receive gesture events */
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
}

// ============================================================
// HOLD-TO-REPEAT: event-based repeat for +/- buttons
// ============================================================
static const uint32_t HOLD_INITIAL_DELAY_MS = 400;
static const uint32_t HOLD_REPEAT_RATE_MS = 120;

// RAW physical touch state, published by the debounced touch reader in
// main.cpp.  We gate the repeat on this instead of LVGL's PRESSED state so a
// hold STOPS the instant the finger physically lifts.  LVGL keeps reporting
// PRESSED for TOUCH_RELEASE_DEBOUNCE_MS (and re-arms on release noise), which
// is exactly what caused the +/- overshoot after release.
extern bool touch_is_physically_down();

// Set to 1 to print hold-to-repeat diagnostics to the serial console:
//   start of press, every repeat (with running count), and release (with the
//   count of repeats that fired AFTER the finger physically lifted — should be
//   0).  Default 0 to keep the console quiet in normal operation.
#define HOLD_REPEAT_DEBUG 0

// Context for each repeatable button
struct HoldRepeatCtx {
    lv_obj_t* spinbox;  // target spinbox (nullptr for power buttons)
    bool is_increment;  // true = +, false = -
    bool is_power;      // true = power button (not spinbox)
    void (*step_fn)(bool inc);  // generic step action (nullptr = use spinbox/power)
    bool active;        // true only while the finger is physically down
    uint32_t press_start;
    uint32_t last_repeat;
#if HOLD_REPEAT_DEBUG
    uint32_t dbg_repeats;       // repeats fired during this press
    uint32_t dbg_after_release; // repeats that fired after physical release
#endif
};

#define MAX_REPEAT_BTNS 28
static HoldRepeatCtx _repeat_pool[MAX_REPEAT_BTNS];
static int _repeat_count = 0;

static HoldRepeatCtx* alloc_repeat_ctx(lv_obj_t* spin, bool inc,
                                       bool is_power) {
    if (_repeat_count >= MAX_REPEAT_BTNS) return nullptr;
    HoldRepeatCtx* ctx = &_repeat_pool[_repeat_count++];
    ctx->spinbox = spin;
    ctx->is_increment = inc;
    ctx->is_power = is_power;
    ctx->step_fn = nullptr;
    ctx->active = false;
    ctx->press_start = 0;
    ctx->last_repeat = 0;
    return ctx;
}

// Allocate a repeatable button bound to a generic step function
static HoldRepeatCtx* alloc_repeat_ctx_fn(void (*fn)(bool inc), bool inc) {
    HoldRepeatCtx* ctx = alloc_repeat_ctx(nullptr, inc, false);
    if (ctx) ctx->step_fn = fn;
    return ctx;
}

// Forward declaration for power step action
static void do_power_step(bool increment);

// Perform one increment/decrement for the given repeat context.
static void repeat_apply_step(HoldRepeatCtx* ctx) {
    if (ctx->step_fn) {
        ctx->step_fn(ctx->is_increment);
    } else if (ctx->is_power) {
        do_power_step(ctx->is_increment);
    } else if (ctx->spinbox) {
        if (ctx->is_increment)
            lv_spinbox_increment(ctx->spinbox);
        else
            lv_spinbox_decrement(ctx->spinbox);
        lv_obj_send_event(ctx->spinbox, LV_EVENT_VALUE_CHANGED, nullptr);
    }
}

// Hard stop: clear the active flag and zero the timers so no queued/stray
// PRESSING event can fire another step.
static void repeat_stop(HoldRepeatCtx* ctx) {
    ctx->active = false;
    ctx->press_start = 0;
    ctx->last_repeat = 0;
}

static void on_repeat_pressed(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_PRESSED) return;
    HoldRepeatCtx* ctx = (HoldRepeatCtx*)lv_event_get_user_data(e);
    if (!ctx) return;
    uint32_t now = millis();
    ctx->active = true;        // finger is physically down
    ctx->press_start = now;
    ctx->last_repeat = now;
#if HOLD_REPEAT_DEBUG
    ctx->dbg_repeats = 0;
    ctx->dbg_after_release = 0;
    Serial.printf("[REPEAT] PRESS start  ctx=%p phys=%d\n", (void*)ctx,
                  (int)touch_is_physically_down());
#endif

    // Perform the action once on press
    repeat_apply_step(ctx);
}

static void on_repeat_pressing(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_PRESSING) return;

    HoldRepeatCtx* ctx = (HoldRepeatCtx*)lv_event_get_user_data(e);
    if (!ctx) return;
    if (!ctx->active) return;   // finger already released — ignore stray events

    // CRITICAL overshoot fix: never fire a step while the finger is physically
    // up.  LVGL still reports PRESSED during the touch release-debounce window
    // (and re-arms on release noise), so relying on the RELEASED event alone let
    // the value keep climbing for several extra steps.  The RAW physical state
    // has no such lag.  We only SKIP here (not hard-stop) so that a momentary
    // single-read GT911 dropout mid-hold doesn't permanently cancel a genuine
    // hold — it resumes as soon as the finger is seen down again.  The true
    // stop is handled by on_repeat_released (RELEASED / PRESS_LOST).
    if (!touch_is_physically_down()) {
#if HOLD_REPEAT_DEBUG
        Serial.printf("[REPEAT] phys-up in PRESSING -> skip  ctx=%p\n",
                      (void*)ctx);
#endif
        return;
    }

    if (!_cfg.hold_to_repeat) return;  // hold-to-repeat disabled
    uint32_t now = millis();

    // Wait for initial delay
    if (now - ctx->press_start < HOLD_INITIAL_DELAY_MS) return;

    // Repeat at steady rate
    if (now - ctx->last_repeat >= HOLD_REPEAT_RATE_MS) {
        ctx->last_repeat = now;
#if HOLD_REPEAT_DEBUG
        ctx->dbg_repeats++;
        if (!touch_is_physically_down()) ctx->dbg_after_release++;
        Serial.printf("[REPEAT] step #%lu  ctx=%p phys=%d\n",
                      (unsigned long)ctx->dbg_repeats, (void*)ctx,
                      (int)touch_is_physically_down());
#endif
        repeat_apply_step(ctx);
    }
}

// Stop repeating the instant LVGL delivers RELEASED / PRESS_LOST.  This is the
// belt-and-braces companion to the physical-touch gate in on_repeat_pressing.
static void on_repeat_released(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_RELEASED && code != LV_EVENT_PRESS_LOST) return;
    HoldRepeatCtx* ctx = (HoldRepeatCtx*)lv_event_get_user_data(e);
    if (!ctx) return;
#if HOLD_REPEAT_DEBUG
    Serial.printf("[REPEAT] RELEASED  ctx=%p total_repeats=%lu after_release=%lu"
                  " (after_release should be 0)\n",
                  (void*)ctx, (unsigned long)ctx->dbg_repeats,
                  (unsigned long)ctx->dbg_after_release);
#endif
    repeat_stop(ctx);
}

// ============================================================
// CONFIG TAB: UI handles
// ============================================================
static lv_obj_t* lbl_cfg_hold_repeat = nullptr;
static lv_obj_t* btn_cfg_hold_repeat = nullptr;
static lv_obj_t* lbl_cfg_time_step = nullptr;
static lv_obj_t* btn_cfg_time_step = nullptr;
static lv_obj_t* lbl_cfg_power_step = nullptr;
static lv_obj_t* btn_cfg_power_step = nullptr;
static lv_obj_t* lbl_cfg_load_last = nullptr;
static lv_obj_t* btn_cfg_load_last = nullptr;
static lv_obj_t* lbl_cfg_brightness = nullptr;
static lv_obj_t* btn_cfg_brightness = nullptr;
static lv_obj_t* lbl_cfg_hold_time = nullptr;     // contact delay value label
static lv_obj_t* btn_cfg_hold_time_dec = nullptr; // contact delay [-]
static lv_obj_t* btn_cfg_hold_time_inc = nullptr; // contact delay [+]
static lv_obj_t* lbl_cfg_cwp = nullptr;  // Contact With Pedal label
static lv_obj_t* btn_cfg_cwp = nullptr;  // Contact With Pedal button

// Config: Calibration section handles
static lv_obj_t* lbl_cfg_cal_leadr = nullptr;   // read-only lead R value
static lv_obj_t* lbl_cfg_cal_age = nullptr;     // calibration age
static lv_obj_t* btn_cfg_calibrate = nullptr;   // AUTO CALIBRATE button
static lv_obj_t* lbl_cfg_calibrate = nullptr;   // button label
static lv_obj_t* lbl_cfg_cal_status = nullptr;  // status / hint line

// ============================================================
// STATIC UI HANDLES  (Status tab)
// ============================================================
// Power-monitoring value labels (Status dashboard)
static lv_obj_t* lbl_pack_v = nullptr;
static lv_obj_t* lbl_temp = nullptr;
static lv_obj_t* lbl_ichg = nullptr;
static lv_obj_t* lbl_cell1 = nullptr;
static lv_obj_t* lbl_cell2 = nullptr;
static lv_obj_t* lbl_cell3 = nullptr;

// System-info box labels (Status dashboard)
static lv_obj_t* btn_dash_mode = nullptr;     // mode box (long-press swap TIME<->JOULE)
static lv_obj_t* lbl_dash_mode = nullptr;     // control mode line
static lv_obj_t* btn_dash_trigger = nullptr;  // trigger line (long-press swap)
static lv_obj_t* lbl_dash_trigger = nullptr;  // trigger line text
static lv_obj_t* lbl_dash_leadr = nullptr;    // lead R + cal age line

// Last-weld result labels (Status dashboard)
// Layout: Duration | Peak | Avg | Joules
static lv_obj_t* lbl_lw_duration = nullptr;
static lv_obj_t* lbl_lw_peak = nullptr;
static lv_obj_t* lbl_lw_avg = nullptr;     // average current (A)
static lv_obj_t* lbl_lw_joules = nullptr;  // workpiece energy (J) – moved here from Joule tab

static lv_obj_t* btn_arm = nullptr;
static lv_obj_t* lbl_arm = nullptr;
static lv_obj_t* lbl_state = nullptr;
static lv_obj_t* lbl_weld_cnt = nullptr;  // Weld Counter value label
static lv_obj_t* btn_weld_cnt =
    nullptr;  // Weld Counter clickable area (for reset)

static bool _last_armed = false;
static uint8_t _trigger_mode = 1;  // 1=Pedal, 2=Probe Contact
static uint8_t _dash_contact_steps = 2;  // for trigger-line delay text

// Long-press tracking for the dashboard trigger line (swap Pedal<->Contact)
static const uint32_t TRIGGER_LONGPRESS_MS = 600;
static uint32_t _trig_press_start = 0;
static bool _trig_longpress_fired = false;

// Long-press tracking for the dashboard MODE box (swap TIME<->JOULE)
static uint32_t _mode_press_start = 0;
static bool _mode_longpress_fired = false;

// ============================================================
// STATIC UI HANDLES  (Joule tab)
// ============================================================
static lv_obj_t* lbl_joule_target = nullptr;
static lv_obj_t* lbl_joule_maxdur = nullptr;
static lv_obj_t* btn_joule_apply = nullptr;
static lv_obj_t* lbl_joule_apply = nullptr;

// Joule-tab draft state (edited locally; committed on APPLY)
static bool     _joule_dirty = false;        // true = unsent edits pending
static bool     _joule_draft_mode = false;   // false=TIME, true=JOULE
static float    _joule_draft_target = 50.0f; // J
static uint16_t _joule_draft_maxms = 40;     // ms
static bool     _joule_draft_init = false;   // synced from state once
static const float    JOULE_TARGET_MIN = 1.0f;
static const float    JOULE_TARGET_MAX = 300.0f;
static const float    JOULE_TARGET_STEP = 1.0f;
static const uint16_t JOULE_MAXMS_MIN = 10;
static const uint16_t JOULE_MAXMS_MAX = 200;
static const uint16_t JOULE_MAXMS_STEP = 5;

// ============================================================
// TAB OBJECT HANDLES  (file-scope so ui_init and ui_mark_settings_applied share
// them)
// ============================================================
static lv_obj_t* tab_status = nullptr;
static lv_obj_t* tab_pulse = nullptr;
static lv_obj_t* tab_joule = nullptr;
static lv_obj_t* tab_config = nullptr;
static lv_obj_t* tab_logs = nullptr;  // repurposed as the "Setup" tab

// ============================================================
// SETUP TAB + WiFi provisioning handles
// ============================================================
// WiFi status section
static lv_obj_t* lbl_setup_wifi_state = nullptr;
static lv_obj_t* lbl_setup_wifi_ssid = nullptr;
static lv_obj_t* lbl_setup_wifi_ip = nullptr;
static lv_obj_t* lbl_setup_wifi_rssi = nullptr;
// System info section
static lv_obj_t* lbl_setup_fw = nullptr;
static lv_obj_t* lbl_setup_chip = nullptr;
static lv_obj_t* lbl_setup_flash = nullptr;
static lv_obj_t* lbl_setup_welds = nullptr;
// Small WiFi indicator on the Status tab header
static lv_obj_t* lbl_status_wifi = nullptr;

// QR provisioning overlay (full-screen, shown during AP portal mode)
static lv_obj_t* prov_overlay = nullptr;
static lv_obj_t* prov_qr = nullptr;
static lv_obj_t* lbl_prov_ssid = nullptr;
static lv_obj_t* lbl_prov_ip = nullptr;

// Confirmation modal (Restart / Factory Reset)
static lv_obj_t* confirm_overlay = nullptr;
static lv_obj_t* lbl_confirm_msg = nullptr;
static int confirm_action = 0;  // 1 = restart, 2 = factory reset

// ============================================================
// STATIC UI HANDLES  (Pulse tab – Production)
// ============================================================
// Mode selector: 3 big toggle buttons
static lv_obj_t* btn_mode[3] = {nullptr, nullptr, nullptr};

// Timing rows: each row is a container holding label, -, spinbox, +, unit
static lv_obj_t* row_d1 = nullptr;
static lv_obj_t* row_gap1 = nullptr;
static lv_obj_t* row_d2 = nullptr;
static lv_obj_t* row_gap2 = nullptr;
static lv_obj_t* row_d3 = nullptr;

// Spinboxes inside the rows (for reading values)
static lv_obj_t* spin_d1 = nullptr;
static lv_obj_t* spin_gap1 = nullptr;
static lv_obj_t* spin_d2 = nullptr;
static lv_obj_t* spin_gap2 = nullptr;
static lv_obj_t* spin_d3 = nullptr;

// Power controls (+/- buttons, no slider)
static lv_obj_t* btn_power_minus = nullptr;
static lv_obj_t* btn_power_plus = nullptr;
static lv_obj_t* lbl_power_val = nullptr;

// Preheat (plain toggle object, no lv_switch)
static lv_obj_t* btn_preheat = nullptr;
static lv_obj_t* lbl_preheat = nullptr;
static lv_obj_t* row_ph_ms = nullptr;
static lv_obj_t* row_ph_pct = nullptr;
static lv_obj_t* row_ph_gap = nullptr;
static lv_obj_t* spin_ph_ms = nullptr;
static lv_obj_t* spin_ph_pct = nullptr;
static lv_obj_t* spin_ph_gap = nullptr;

// Action buttons (no Revert)
static lv_obj_t* btn_apply = nullptr;
static lv_obj_t* lbl_apply = nullptr;
static lv_obj_t* lbl_pending = nullptr;

// All interactive elements for lock/unlock
static const int MAX_LOCKABLE = 20;
static lv_obj_t* lockable_objs[MAX_LOCKABLE];
static int lockable_count = 0;

static void register_lockable(lv_obj_t* obj) {
    if (obj && lockable_count < MAX_LOCKABLE) {
        lockable_objs[lockable_count++] = obj;
    }
}

// ============================================================
// DRAFT / APPLIED STATE
// ============================================================
static uint8_t draft_mode = 1;
static uint16_t draft_d1 = 5;
static uint16_t draft_gap1 = 0;
static uint16_t draft_d2 = 0;
static uint16_t draft_gap2 = 0;
static uint16_t draft_d3 = 0;
static uint8_t draft_power = 100;
static bool draft_preheat_en = false;
static uint16_t draft_preheat_ms = 20;
static uint8_t draft_preheat_pct = 30;
static uint16_t draft_preheat_gap = 3;

static uint8_t applied_mode = 1;
static uint16_t applied_d1 = 5;
static uint16_t applied_gap1 = 0;
static uint16_t applied_d2 = 0;
static uint16_t applied_gap2 = 0;
static uint16_t applied_d3 = 0;
static uint8_t applied_power = 100;
static bool applied_preheat_en = false;
static uint16_t applied_preheat_ms = 20;
static uint8_t applied_preheat_pct = 30;
static uint16_t applied_preheat_gap = 3;

static bool draft_dirty = false;
static bool applied_initialized = false;

// ============================================================
// HELPERS
// ============================================================

static void update_draft_dirty() {
    draft_dirty = (draft_mode != applied_mode || draft_d1 != applied_d1 ||
                   draft_gap1 != applied_gap1 || draft_d2 != applied_d2 ||
                   draft_gap2 != applied_gap2 || draft_d3 != applied_d3 ||
                   draft_power != applied_power ||
                   draft_preheat_en != applied_preheat_en ||
                   draft_preheat_ms != applied_preheat_ms ||
                   draft_preheat_pct != applied_preheat_pct ||
                   draft_preheat_gap != applied_preheat_gap);
}

// _ui_dirty flag: set by callbacks, cleared by ui_update after visual refresh
static bool _ui_dirty = false;

static void sync_applied_from_state(const WelderDisplayState& st) {
    bool changed = false;

    if (applied_mode != st.weld_mode) {
        applied_mode = st.weld_mode;
        changed = true;
    }
    if (applied_d1 != st.pulse_d1) {
        applied_d1 = st.pulse_d1;
        changed = true;
    }
    if (applied_gap1 != st.pulse_gap1) {
        applied_gap1 = st.pulse_gap1;
        changed = true;
    }
    if (applied_d2 != st.pulse_d2) {
        applied_d2 = st.pulse_d2;
        changed = true;
    }
    if (applied_gap2 != st.pulse_gap2) {
        applied_gap2 = st.pulse_gap2;
        changed = true;
    }
    if (applied_d3 != st.pulse_d3) {
        applied_d3 = st.pulse_d3;
        changed = true;
    }
    if (applied_power != st.power_pct) {
        applied_power = st.power_pct;
        changed = true;
    }
    if (applied_preheat_en != st.preheat_enabled) {
        applied_preheat_en = st.preheat_enabled;
        changed = true;
    }
    if (applied_preheat_ms != st.preheat_ms) {
        applied_preheat_ms = st.preheat_ms;
        changed = true;
    }
    if (applied_preheat_pct != st.preheat_pct) {
        applied_preheat_pct = st.preheat_pct;
        changed = true;
    }
    if (applied_preheat_gap != st.preheat_gap_ms) {
        applied_preheat_gap = st.preheat_gap_ms;
        changed = true;
    }

    // Sync draft from applied when:
    //   (a) First time ever (applied not initialized), OR
    //   (b) User hasn't manually edited anything (draft_dirty == false)
    // This ensures that after sendBootConfig(), once STM32 echoes back the
    // settings via STATUS, the draft values track the applied values and
    // the UI won't show a false "unsaved changes" indicator.
    if (!applied_initialized || (!draft_dirty && changed)) {
        draft_mode = applied_mode;
        draft_d1 = applied_d1;
        draft_gap1 = applied_gap1;
        draft_d2 = applied_d2;
        draft_gap2 = applied_gap2;
        draft_d3 = applied_d3;
        draft_power = applied_power;
        draft_preheat_en = applied_preheat_en;
        draft_preheat_ms = applied_preheat_ms;
        draft_preheat_pct = applied_preheat_pct;
        draft_preheat_gap = applied_preheat_gap;
        applied_initialized = true;

        // Also update spinbox widgets to reflect new draft values
        if (spin_d1) lv_spinbox_set_value(spin_d1, (int32_t)draft_d1);
        if (spin_gap1) lv_spinbox_set_value(spin_gap1, (int32_t)draft_gap1);
        if (spin_d2) lv_spinbox_set_value(spin_d2, (int32_t)draft_d2);
        if (spin_gap2) lv_spinbox_set_value(spin_gap2, (int32_t)draft_gap2);
        if (spin_d3) lv_spinbox_set_value(spin_d3, (int32_t)draft_d3);
        if (spin_ph_ms)
            lv_spinbox_set_value(spin_ph_ms, (int32_t)draft_preheat_ms);
        if (spin_ph_pct)
            lv_spinbox_set_value(spin_ph_pct, (int32_t)draft_preheat_pct);
        if (spin_ph_gap)
            lv_spinbox_set_value(spin_ph_gap, (int32_t)draft_preheat_gap);

        _ui_dirty = true;  // trigger visual refresh
    }

    if (changed) {
        update_draft_dirty();
    }
}

// ============================================================
// HELPER – paint ARM button to a given state
// ============================================================
static void paint_arm_button(bool armed) {
    if (btn_arm) {
        lv_obj_set_style_bg_color(btn_arm, armed ? C_GREEN : C_RED, 0);
    }
    if (lbl_arm) {
        lv_label_set_text(lbl_arm, armed ? "ARMED  (tap to disarm)"
                                         : "DISARMED  (tap to arm)");
    }
}

// ============================================================
// WELD COUNTER RESET EVENT (tap on enlarged counter)
// ============================================================
static void on_weld_count_reset(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (_weld_reset_cb) _weld_reset_cb();
}

// ============================================================
// DASHBOARD TRIGGER LINE – long-press to swap Pedal <-> Contact
// ============================================================
static const char* trigger_mode_name(uint8_t mode) {
    return (mode == 2) ? "Contact" : "Pedal";
}

// Repaint the trigger line text + color. Highlight while a long-press is held.
static void paint_dash_trigger(bool holding) {
    if (lbl_dash_trigger) {
        char buf[64];
        // Pedal mode: show just "Pedal" (delay is not used).
        // Contact mode: show "Contact X.Xs" with the contact/probe delay.
        if (_trigger_mode == 2) {
            float delay_s = 0.5f * (float)_dash_contact_steps;
            snprintf(buf, sizeof(buf), "Contact %.1fs", (double)delay_s);
        } else {
            snprintf(buf, sizeof(buf), "Pedal");
        }
        lv_label_set_text(lbl_dash_trigger, buf);
        lv_obj_set_style_text_color(
            lbl_dash_trigger,
            holding ? C_YELLOW : (_trigger_mode == 2 ? C_GREEN : C_ACCENT),
            LV_PART_MAIN);
    }
    if (btn_dash_trigger) {
        lv_obj_set_style_bg_color(btn_dash_trigger,
                                  holding ? C_DARK_GREY : C_CARD, 0);
    }
}

// Long-press handler on the trigger line: hold ~600ms to toggle source.
static void on_dash_trigger_event(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        _trig_press_start = millis();
        _trig_longpress_fired = false;
        paint_dash_trigger(true);  // visual feedback: holding
    } else if (code == LV_EVENT_PRESSING) {
        if (!_trig_longpress_fired &&
            (millis() - _trig_press_start) >= TRIGGER_LONGPRESS_MS) {
            _trig_longpress_fired = true;
            // Swap Pedal <-> Contact
            _trigger_mode = (_trigger_mode == 1) ? 2 : 1;
            paint_dash_trigger(false);
            if (_trigger_cb) _trigger_cb(_trigger_mode);
        }
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        paint_dash_trigger(false);
    }
}

// ============================================================
// DASHBOARD MODE BOX – long-press to swap TIME <-> JOULE
// ============================================================
// Forward declaration: the MODE switcher applies the change immediately by
// reusing the Joule apply path (sends SET_MODE + targets to the STM32).
// (_joule_draft_mode / _target / _maxms are file-scope statics defined above.)
static void clamp_joule_draft();

// Repaint the MODE box text + color. Highlight while a long-press is held.
static void paint_dash_mode(bool holding) {
    bool joule = _joule_draft_mode;
    if (lbl_dash_mode) {
        lv_label_set_text(lbl_dash_mode, joule ? "JOULE" : "TIME");
        lv_obj_set_style_text_color(
            lbl_dash_mode,
            holding ? C_YELLOW : (joule ? C_GREEN : C_ACCENT),
            LV_PART_MAIN);
    }
    if (btn_dash_mode) {
        lv_obj_set_style_bg_color(btn_dash_mode,
                                  holding ? C_DARK_GREY : C_CARD, 0);
    }
}

// Long-press handler on the MODE box: hold ~600ms to toggle TIME<->JOULE.
// On toggle the change is applied immediately (SET_MODE to STM32; Flask syncs
// through the next STATUS packet) – there is no separate Apply step.
static void on_dash_mode_event(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        _mode_press_start = millis();
        _mode_longpress_fired = false;
        paint_dash_mode(true);  // visual feedback: holding
    } else if (code == LV_EVENT_PRESSING) {
        if (!_mode_longpress_fired &&
            (millis() - _mode_press_start) >= TRIGGER_LONGPRESS_MS) {
            _mode_longpress_fired = true;
            // Swap TIME <-> JOULE and apply immediately
            _joule_draft_mode = !_joule_draft_mode;
            paint_dash_mode(false);
            clamp_joule_draft();
            if (_joule_apply_cb)
                _joule_apply_cb(_joule_draft_mode, _joule_draft_target,
                                _joule_draft_maxms);
        }
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        paint_dash_mode(false);
    }
}

// ============================================================
// ARM BUTTON EVENT
// ============================================================
static void arm_btn_event(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    // Suppress the click that ends a long-press swap gesture on the
    // trigger line (defensive – they are separate widgets, but keep safe).
    bool requested = !_last_armed;
    if (_arm_cb) {
        _arm_cb(requested);
    }
}

// ============================================================
// HELPER – create a value card inside a parent
// ============================================================
static lv_obj_t* make_card(lv_obj_t* parent, const char* title,
                           lv_obj_t** out_value_label, int w, int h,
                           const lv_font_t* val_font = &lv_font_montserrat_14) {
    lv_obj_t* card = lv_obj_create(parent);
    lv_obj_set_size(card, w, h);
    lv_obj_set_style_bg_color(card, C_CARD, LV_PART_MAIN);
    lv_obj_set_style_border_color(card, C_DARK_GREY, LV_PART_MAIN);
    lv_obj_set_style_border_width(card, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(card, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_all(card, 8, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(card, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* lbl_t = lv_label_create(card);
    lv_label_set_text(lbl_t, title);
    lv_obj_set_style_text_color(lbl_t, C_GREY, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_t, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(lbl_t, LV_ALIGN_TOP_MID, 0, 0);

    lv_obj_t* lbl_v = lv_label_create(card);
    lv_label_set_text(lbl_v, "---");
    lv_obj_set_style_text_color(lbl_v, C_GREEN, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_v, val_font, LV_PART_MAIN);
    lv_obj_align(lbl_v, LV_ALIGN_CENTER, 0, 8);

    if (out_value_label) *out_value_label = lbl_v;
    return card;
}

// ============================================================
// STATUS TAB HELPERS – dashboard panels and stat cells
// ============================================================

// Create a bare panel (Widget-A safe, non-clickable container) for grouping.
static lv_obj_t* make_panel(lv_obj_t* parent, int x, int y, int w, int h) {
    lv_obj_t* p = lv_obj_create(parent);
    lv_obj_remove_style_all(p);
    lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(p, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(p, w, h);
    lv_obj_set_pos(p, x, y);
    lv_obj_set_style_bg_color(p, C_CARD, 0);
    lv_obj_set_style_bg_opa(p, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(p, 10, 0);
    lv_obj_set_style_border_color(p, C_DARK_GREY, 0);
    lv_obj_set_style_border_width(p, 1, 0);
    lv_obj_set_scrollbar_mode(p, LV_SCROLLBAR_MODE_OFF);
    return p;
}

// Create a small "title + value" stat cell inside a parent panel.
static void make_stat_cell(lv_obj_t* parent, const char* title,
                           lv_obj_t** out_val, int x, int y, int w, int h,
                           const lv_font_t* val_font) {
    lv_obj_t* cell = make_panel(parent, x, y, w, h);
    lv_obj_set_style_bg_color(cell, C_BG, 0);

    lv_obj_t* t = lv_label_create(cell);
    lv_label_set_text(t, title);
    lv_obj_set_style_text_color(t, C_GREY, 0);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_14, 0);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 6);

    lv_obj_t* v = lv_label_create(cell);
    lv_label_set_text(v, "--");
    lv_obj_set_style_text_color(v, C_WHITE, 0);
    lv_obj_set_style_text_font(v, val_font, 0);
    lv_obj_align(v, LV_ALIGN_BOTTOM_MID, 0, -8);
    if (out_val) *out_val = v;
}

// ============================================================
// STATUS TAB BUILDER – Dashboard layout
// ============================================================
static void build_status_tab(lv_obj_t* tab) {
    lv_obj_set_style_bg_color(tab, C_BG, LV_PART_MAIN);
    lv_obj_set_style_pad_all(tab, 10, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(tab, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);

    const int CONTENT_W = 780;
    const int GAP = 10;

    // ---------------------------------------------------------
    // ROW 1 (y=0..58): compact info strip
    //   MODE | TRIGGER (clickable) | LEAD R | WELDS (tap to reset)
    // Each box has a small grey title (top) and a value (bottom).
    // ---------------------------------------------------------
    const int R1_Y = 0;
    const int R1_H = 58;
    const int MODE_W = 150;
    const int TRIG_W = 236;  // shrunk to make room for the WiFi indicator
    const int WIFI_W = 34;   // small WiFi status indicator
    const int LEADR_W = 200;
    const int WELDS_W = 120;
    const int MODE_X = 0;
    const int TRIG_X = MODE_X + MODE_W + GAP;     // 160
    const int WIFI_X = TRIG_X + TRIG_W + GAP;     // 406
    const int LEADR_X = WIFI_X + WIFI_W + GAP;    // 450
    const int WELDS_X = LEADR_X + LEADR_W + GAP;  // 660 (ends at 780)

    // ---- MODE box (clickable; long-press toggles TIME<->JOULE) ----
    {
        btn_dash_mode = lv_obj_create(tab);
        lv_obj_remove_style_all(btn_dash_mode);
        lv_obj_add_flag(btn_dash_mode, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(btn_dash_mode, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_size(btn_dash_mode, MODE_W, R1_H);
        lv_obj_set_pos(btn_dash_mode, MODE_X, R1_Y);
        lv_obj_set_style_bg_color(btn_dash_mode, C_CARD, 0);
        lv_obj_set_style_bg_opa(btn_dash_mode, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(btn_dash_mode, 10, 0);
        lv_obj_set_style_border_color(btn_dash_mode, C_DARK_GREY, 0);
        lv_obj_set_style_border_width(btn_dash_mode, 1, 0);
        lv_obj_set_scrollbar_mode(btn_dash_mode, LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_event_cb(btn_dash_mode, on_dash_mode_event,
                            LV_EVENT_PRESSED, nullptr);
        lv_obj_add_event_cb(btn_dash_mode, on_dash_mode_event,
                            LV_EVENT_PRESSING, nullptr);
        lv_obj_add_event_cb(btn_dash_mode, on_dash_mode_event,
                            LV_EVENT_RELEASED, nullptr);
        lv_obj_add_event_cb(btn_dash_mode, on_dash_mode_event,
                            LV_EVENT_PRESS_LOST, nullptr);
        make_interaction_safe(btn_dash_mode);

        lv_obj_t* t = lv_label_create(btn_dash_mode);
        lv_label_set_text(t, "MODE");
        lv_obj_set_style_text_color(t, C_GREY, 0);
        lv_obj_set_style_text_font(t, &lv_font_montserrat_14, 0);
        lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 5);

        lbl_dash_mode = lv_label_create(btn_dash_mode);
        lv_label_set_text(lbl_dash_mode, "TIME");
        lv_obj_set_style_text_color(lbl_dash_mode, C_ACCENT, 0);
        lv_obj_set_style_text_font(lbl_dash_mode, &lv_font_montserrat_18, 0);
        lv_obj_align(lbl_dash_mode, LV_ALIGN_BOTTOM_MID, 0, -6);
    }

    // ---- TRIGGER box (clickable; long-press toggles Pedal<->Contact) ----
    {
        btn_dash_trigger = lv_obj_create(tab);
        lv_obj_remove_style_all(btn_dash_trigger);
        lv_obj_add_flag(btn_dash_trigger, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(btn_dash_trigger, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_size(btn_dash_trigger, TRIG_W, R1_H);
        lv_obj_set_pos(btn_dash_trigger, TRIG_X, R1_Y);
        lv_obj_set_style_bg_color(btn_dash_trigger, C_CARD, 0);
        lv_obj_set_style_bg_opa(btn_dash_trigger, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(btn_dash_trigger, 10, 0);
        lv_obj_set_style_border_color(btn_dash_trigger, C_DARK_GREY, 0);
        lv_obj_set_style_border_width(btn_dash_trigger, 1, 0);
        lv_obj_set_scrollbar_mode(btn_dash_trigger, LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_event_cb(btn_dash_trigger, on_dash_trigger_event,
                            LV_EVENT_PRESSED, nullptr);
        lv_obj_add_event_cb(btn_dash_trigger, on_dash_trigger_event,
                            LV_EVENT_PRESSING, nullptr);
        lv_obj_add_event_cb(btn_dash_trigger, on_dash_trigger_event,
                            LV_EVENT_RELEASED, nullptr);
        lv_obj_add_event_cb(btn_dash_trigger, on_dash_trigger_event,
                            LV_EVENT_PRESS_LOST, nullptr);
        make_interaction_safe(btn_dash_trigger);

        lv_obj_t* t = lv_label_create(btn_dash_trigger);
        lv_label_set_text(t, "TRIGGER");
        lv_obj_set_style_text_color(t, C_GREY, 0);
        lv_obj_set_style_text_font(t, &lv_font_montserrat_14, 0);
        lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 5);

        lbl_dash_trigger = lv_label_create(btn_dash_trigger);
        lv_label_set_text(lbl_dash_trigger, "Pedal");
        lv_obj_set_style_text_color(lbl_dash_trigger, C_ACCENT, 0);
        lv_obj_set_style_text_font(lbl_dash_trigger, &lv_font_montserrat_18, 0);
        lv_obj_align(lbl_dash_trigger, LV_ALIGN_BOTTOM_MID, 0, -6);
    }

    // ---- WiFi status indicator (small, between TRIGGER and LEAD R) ----
    {
        lv_obj_t* p = make_panel(tab, WIFI_X, R1_Y, WIFI_W, R1_H);
        lbl_status_wifi = lv_label_create(p);
        lv_label_set_text(lbl_status_wifi, LV_SYMBOL_WIFI);
        lv_obj_set_style_text_color(lbl_status_wifi, C_GREY, 0);
        lv_obj_set_style_text_font(lbl_status_wifi, &lv_font_montserrat_18, 0);
        lv_obj_center(lbl_status_wifi);
    }

    // ---- LEAD R box ----
    {
        lv_obj_t* p = make_panel(tab, LEADR_X, R1_Y, LEADR_W, R1_H);
        lv_obj_t* t = lv_label_create(p);
        lv_label_set_text(t, "LEAD R");
        lv_obj_set_style_text_color(t, C_GREY, 0);
        lv_obj_set_style_text_font(t, &lv_font_montserrat_14, 0);
        lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 5);

        lbl_dash_leadr = lv_label_create(p);
        lv_label_set_text(lbl_dash_leadr, "-- m");
        lv_obj_set_style_text_color(lbl_dash_leadr, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl_dash_leadr, &lv_font_montserrat_18, 0);
        lv_obj_align(lbl_dash_leadr, LV_ALIGN_BOTTOM_MID, 0, -6);
    }

    // ---- WELDS box (clickable; tap to reset counter) ----
    {
        btn_weld_cnt = lv_obj_create(tab);
        lv_obj_remove_style_all(btn_weld_cnt);
        lv_obj_add_flag(btn_weld_cnt, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(btn_weld_cnt, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_size(btn_weld_cnt, WELDS_W, R1_H);
        lv_obj_set_pos(btn_weld_cnt, WELDS_X, R1_Y);
        lv_obj_set_style_bg_color(btn_weld_cnt, C_CARD, 0);
        lv_obj_set_style_bg_opa(btn_weld_cnt, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(btn_weld_cnt, 10, 0);
        lv_obj_set_style_border_color(btn_weld_cnt, C_DARK_GREY, 0);
        lv_obj_set_style_border_width(btn_weld_cnt, 1, 0);
        lv_obj_set_scrollbar_mode(btn_weld_cnt, LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_event_cb(btn_weld_cnt, on_weld_count_reset, LV_EVENT_CLICKED,
                            nullptr);
        make_interaction_safe(btn_weld_cnt);

        lv_obj_t* lbl_title = lv_label_create(btn_weld_cnt);
        lv_label_set_text(lbl_title, "WELDS " LV_SYMBOL_REFRESH);
        lv_obj_set_style_text_color(lbl_title, C_GREY, LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_14,
                                   LV_PART_MAIN);
        lv_obj_align(lbl_title, LV_ALIGN_TOP_MID, 0, 5);

        lbl_weld_cnt = lv_label_create(btn_weld_cnt);
        lv_label_set_text(lbl_weld_cnt, "0");
        lv_obj_set_style_text_color(lbl_weld_cnt, C_GREEN, LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl_weld_cnt, &lv_font_montserrat_20,
                                   LV_PART_MAIN);
        lv_obj_align(lbl_weld_cnt, LV_ALIGN_BOTTOM_MID, 0, -6);
    }

    // hidden state handle (kept for ui_update internal state tracking)
    lbl_state = lv_label_create(tab);
    lv_label_set_text(lbl_state, "DISARMED");
    lv_obj_add_flag(lbl_state, LV_OBJ_FLAG_HIDDEN);

    // ---------------------------------------------------------
    // ROW 2 (y=68..160): Power monitoring – 4 columns
    //   PACK | CELLS | TEMPERATURE | CHARGING
    // ---------------------------------------------------------
    const int PWR_Y = 68;
    const int PWR_H = 92;
    const int PACK_W = 195;
    const int CELLS_W = 150;  // narrower / compact
    const int TEMP_W = 195;
    const int CHG_W = 210;
    const int PACK_X = 0;
    const int CELLS_X = PACK_X + PACK_W + GAP;   // 205
    const int TEMP_X = CELLS_X + CELLS_W + GAP;  // 365
    const int CHG_X = TEMP_X + TEMP_W + GAP;     // 570 (ends at 780)

    // PACK column – voltage only (charge current moved to CHARGING)
    {
        lv_obj_t* p = make_panel(tab, PACK_X, PWR_Y, PACK_W, PWR_H);
        lv_obj_t* t = lv_label_create(p);
        lv_label_set_text(t, "PACK");
        lv_obj_set_style_text_color(t, C_GREY, 0);
        lv_obj_set_style_text_font(t, &lv_font_montserrat_14, 0);
        lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 6);

        lbl_pack_v = lv_label_create(p);
        lv_label_set_text(lbl_pack_v, "-- V");
        lv_obj_set_style_text_color(lbl_pack_v, C_GREEN, 0);
        lv_obj_set_style_text_font(lbl_pack_v, &lv_font_montserrat_24, 0);
        lv_obj_align(lbl_pack_v, LV_ALIGN_CENTER, 0, 8);
    }

    // CELLS column – compact C1/C2/C3 (narrow panel)
    //   No "CELLS" title: the three cell voltages are bumped up one font size
    //   (16->18) and centred as a group inside the panel (offsets -26/0/+26).
    {
        lv_obj_t* p = make_panel(tab, CELLS_X, PWR_Y, CELLS_W, PWR_H);

        lbl_cell1 = lv_label_create(p);
        lv_label_set_text(lbl_cell1, "C1: -- V");
        lv_obj_set_style_text_color(lbl_cell1, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl_cell1, &lv_font_montserrat_18, 0);
        lv_obj_align(lbl_cell1, LV_ALIGN_CENTER, 0, -26);

        lbl_cell2 = lv_label_create(p);
        lv_label_set_text(lbl_cell2, "C2: -- V");
        lv_obj_set_style_text_color(lbl_cell2, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl_cell2, &lv_font_montserrat_18, 0);
        lv_obj_align(lbl_cell2, LV_ALIGN_CENTER, 0, 0);

        lbl_cell3 = lv_label_create(p);
        lv_label_set_text(lbl_cell3, "C3: -- V");
        lv_obj_set_style_text_color(lbl_cell3, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl_cell3, &lv_font_montserrat_18, 0);
        lv_obj_align(lbl_cell3, LV_ALIGN_CENTER, 0, 26);
    }

    // TEMPERATURE column – value only
    {
        lv_obj_t* p = make_panel(tab, TEMP_X, PWR_Y, TEMP_W, PWR_H);
        lv_obj_t* t = lv_label_create(p);
        lv_label_set_text(t, "TEMPERATURE");
        lv_obj_set_style_text_color(t, C_GREY, 0);
        lv_obj_set_style_text_font(t, &lv_font_montserrat_14, 0);
        lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 6);

        lbl_temp = lv_label_create(p);
        lv_label_set_text(lbl_temp, "-- \xC2\xB0\x43");
        lv_obj_set_style_text_color(lbl_temp, C_GREEN, 0);
        lv_obj_set_style_text_font(lbl_temp, &lv_font_montserrat_24, 0);
        lv_obj_align(lbl_temp, LV_ALIGN_CENTER, 0, 8);

        // lbl_health intentionally not created: status line removed.
        // The pointer stays nullptr so ui_update()'s guarded writes no-op.
    }

    // CHARGING column – charge current value only (no "Chg:" prefix)
    {
        lv_obj_t* p = make_panel(tab, CHG_X, PWR_Y, CHG_W, PWR_H);
        lv_obj_t* t = lv_label_create(p);
        lv_label_set_text(t, "CHARGING");
        lv_obj_set_style_text_color(t, C_GREY, 0);
        lv_obj_set_style_text_font(t, &lv_font_montserrat_14, 0);
        lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 6);

        lbl_ichg = lv_label_create(p);
        lv_label_set_text(lbl_ichg, "-- A");
        lv_obj_set_style_text_color(lbl_ichg, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl_ichg, &lv_font_montserrat_24, 0);
        lv_obj_align(lbl_ichg, LV_ALIGN_CENTER, 0, 8);
    }

    // ---------------------------------------------------------
    // ROW 3 (y=191..257): ARM/DISARM button – prominent but compact.
    // Centred horizontally; also centred VERTICALLY in the gap between ROW 2
    // (ends y=160) and the Last Weld bar (top y=288): gap=128, button=66, so
    // ARM_Y = 160 + (128-66)/2 = 191 (equal ~31px above & below).
    // ---------------------------------------------------------
    const int ARM_W = 320;
    const int ARM_H = 66;
    const int ARM_Y = 191;
    const int ARM_X = (CONTENT_W - ARM_W) / 2;  // 230 (centred)

    btn_arm = lv_obj_create(tab);
    lv_obj_remove_style_all(btn_arm);
    lv_obj_add_flag(btn_arm, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(btn_arm, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(btn_arm, ARM_W, ARM_H);
    lv_obj_set_pos(btn_arm, ARM_X, ARM_Y);
    lv_obj_set_style_bg_color(btn_arm, C_RED, 0);
    lv_obj_set_style_bg_opa(btn_arm, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(btn_arm, 12, 0);
    lv_obj_set_scrollbar_mode(btn_arm, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(btn_arm, arm_btn_event, LV_EVENT_CLICKED, nullptr);
    make_interaction_safe(btn_arm);

    lbl_arm = lv_label_create(btn_arm);
    lv_label_set_text(lbl_arm, "DISARMED  (tap to arm)");
    lv_obj_set_style_text_align(lbl_arm, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(lbl_arm, C_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_arm, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_center(lbl_arm);

    // ---------------------------------------------------------
    // ROW 4 (y=288..416): Last Weld results – pushed to the very bottom.
    //   Duration | Peak | Avg | Joules
    // ---------------------------------------------------------
    // IMPORTANT (display geometry + safety margin):
    //   The ESP32-8048S043C is an RGB-parallel panel. The last few scanlines
    //   at the very bottom of this board are prone to a DMA/PSRAM timing
    //   glitch ("scrambled line at the bottom").
    //   Coordinate math: a STATUS-tab child at child-y Y sits at absolute
    //   screen-y = 42 (top tab bar) + 10 (tab pad_top) + Y. The tab's usable
    //   content height is 418px (480 - 42 tab bar - 2*10 pad), so child-y
    //   ranges 0..418. We push Last Weld to child-y 288 (bottom child-y 416,
    //   absolute screen-y 468) — that's the practical maximum: it leaves a
    //   ~12px clear band above the screen bottom (480) so nothing lands in the
    //   glitchy scanlines. Do NOT push LW_Y past ~290 / bottom child-y past
    //   ~418. (Bottom here = 288+128 = 416 -> screen-y 468.)
    const int LW_Y = 288;
    const int LW_H = 128;
    {
        lv_obj_t* box = make_panel(tab, 0, LW_Y, CONTENT_W, LW_H);

        lv_obj_t* t = lv_label_create(box);
        lv_label_set_text(t, "LAST WELD");
        lv_obj_set_style_text_color(t, C_ACCENT, 0);
        lv_obj_set_style_text_font(t, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(t, 12, 6);

        const int cell_y = 30;
        const int cell_h = 88;
        const int cell_w = 180;
        const int cell_gap = (CONTENT_W - 4 * cell_w) / 5;  // even spacing
        int cx = cell_gap;

        make_stat_cell(box, "Duration", &lbl_lw_duration, cx, cell_y, cell_w,
                       cell_h, &lv_font_montserrat_24);
        cx += cell_w + cell_gap;
        make_stat_cell(box, "Peak", &lbl_lw_peak, cx, cell_y, cell_w,
                       cell_h, &lv_font_montserrat_24);
        cx += cell_w + cell_gap;
        make_stat_cell(box, "Avg", &lbl_lw_avg, cx, cell_y, cell_w,
                       cell_h, &lv_font_montserrat_24);
        cx += cell_w + cell_gap;
        make_stat_cell(box, "Joules", &lbl_lw_joules, cx, cell_y, cell_w,
                       cell_h, &lv_font_montserrat_24);
    }

    // Initialise trigger-line text
    paint_dash_trigger(false);
    // Initialise MODE box text
    paint_dash_mode(false);
}

// ============================================================
// PULSE TAB: show/hide row containers based on mode
// ============================================================
static void set_row_visible(lv_obj_t* row, bool show) {
    if (!row) return;
    if (show)
        lv_obj_clear_flag(row, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(row, LV_OBJ_FLAG_HIDDEN);
}

static void update_pulse_field_visibility() {
    bool show_gap1 = (draft_mode >= 2);
    bool show_d2 = (draft_mode >= 2);
    bool show_gap2 = (draft_mode >= 3);
    bool show_d3 = (draft_mode >= 3);

    set_row_visible(row_gap1, show_gap1);
    set_row_visible(row_d2, show_d2);
    set_row_visible(row_gap2, show_gap2);
    set_row_visible(row_d3, show_d3);
}

static void update_preheat_visibility() {
    bool show = draft_preheat_en;
    set_row_visible(row_ph_ms, show);
    set_row_visible(row_ph_pct, show);
    set_row_visible(row_ph_gap, show);
}

static void mark_dirty() {
    update_draft_dirty();
    _ui_dirty = true;
}

// ============================================================
// PULSE TAB: widget event callbacks
// ============================================================
// Data-only callback: just update draft_mode, set dirty
static void on_mode_btn_click(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    draft_mode = (uint8_t)(idx + 1);
    mark_dirty();
}

// Data-only callback: read spinbox value into draft, set dirty
static void on_spinbox_change(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    lv_obj_t* obj = (lv_obj_t*)lv_event_get_target(e);
    int32_t val = lv_spinbox_get_value(obj);

    if (obj == spin_d1)
        draft_d1 = (uint16_t)val;
    else if (obj == spin_gap1)
        draft_gap1 = (uint16_t)val;
    else if (obj == spin_d2)
        draft_d2 = (uint16_t)val;
    else if (obj == spin_gap2)
        draft_gap2 = (uint16_t)val;
    else if (obj == spin_d3)
        draft_d3 = (uint16_t)val;
    else if (obj == spin_ph_ms)
        draft_preheat_ms = (uint16_t)val;
    else if (obj == spin_ph_pct)
        draft_preheat_pct = (uint8_t)val;
    else if (obj == spin_ph_gap)
        draft_preheat_gap = (uint16_t)val;

    mark_dirty();
}

// Power step action – uses global config step size
static void do_power_step(bool increment) {
    uint8_t step = _cfg.power_step_pct;
    if (step == 0) step = 5;
    if (increment) {
        if (draft_power + step <= 100)
            draft_power += step;
        else
            draft_power = 100;
    } else {
        if (draft_power >= 50 + step)
            draft_power -= step;
        else
            draft_power = 50;
    }
    mark_dirty();
}

// Data-only callback: power +/- buttons (legacy CLICKED path, kept for safety)
static void on_power_minus(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    do_power_step(false);
}
static void on_power_plus(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    do_power_step(true);
}

// Data-only callback: preheat toggle (plain object click)
static void on_preheat_toggle(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    draft_preheat_en = !draft_preheat_en;
    mark_dirty();
}

// Data-only callback: apply sends recipe via callback
static void on_apply_click(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!draft_dirty) return;
    if (_recipe_cb) {
        _recipe_cb(draft_mode, draft_d1, draft_gap1, draft_d2, draft_gap2,
                   draft_d3, draft_power, draft_preheat_en, draft_preheat_ms,
                   draft_preheat_pct, draft_preheat_gap);
    }
}

// ============================================================
// PULSE TAB: helper – create a big touch-friendly spinbox row
// Row layout:  [Label 90px] [– 56x46] [Value 84x46] [+ 56x46] [unit 40px]
// ALL +/- buttons use Widget A pattern (plain lv_obj, no theme)
// ============================================================
static lv_obj_t* make_touch_row(lv_obj_t* parent, const char* title,
                                const char* unit, lv_obj_t** out_spinbox,
                                int min_val, int max_val, int init_val,
                                int digits, int step) {
    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, 370, 50);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* lbl = lv_label_create(row);
    lv_label_set_text(lbl, title);
    lv_obj_set_style_text_color(lbl, C_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_pos(lbl, 0, 14);

    const int BTN_W = 56;
    const int BTN_H = 46;
    const int VAL_W = 84;
    const int X_DEC = 95;
    const int X_VAL = X_DEC + BTN_W + 4;
    const int X_INC = X_VAL + VAL_W + 4;

    // Decrement button – Widget A pattern (plain lv_obj, NO lv_button)
    lv_obj_t* btn_dec = lv_obj_create(row);
    lv_obj_remove_style_all(btn_dec);
    lv_obj_add_flag(btn_dec, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(btn_dec, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(btn_dec, BTN_W, BTN_H);
    lv_obj_set_pos(btn_dec, X_DEC, 2);
    lv_obj_set_style_bg_color(btn_dec, C_DARK_GREY, 0);
    lv_obj_set_style_bg_opa(btn_dec, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(btn_dec, 8, 0);
    lv_obj_t* lbl_dec = lv_label_create(btn_dec);
    lv_label_set_text(lbl_dec, LV_SYMBOL_MINUS);
    lv_obj_set_style_text_color(lbl_dec, C_WHITE, 0);
    lv_obj_set_style_text_font(lbl_dec, &lv_font_montserrat_18, 0);
    lv_obj_center(lbl_dec);
    make_interaction_safe(btn_dec);  // <-- ANTI-SHUDDER

    // Spinbox (value display)
    lv_obj_t* spin = lv_spinbox_create(row);
    lv_spinbox_set_range(spin, min_val, max_val);
    lv_spinbox_set_digit_format(spin, digits, 0);
    lv_spinbox_set_step(spin, step);
    lv_spinbox_set_value(spin, init_val);
    lv_obj_set_size(spin, VAL_W, BTN_H);
    lv_obj_set_pos(spin, X_VAL, 2);
    lv_obj_set_style_bg_color(spin, C_CARD, LV_PART_MAIN);
    lv_obj_set_style_text_color(spin, C_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(spin, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_set_style_text_align(spin, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_border_color(spin, C_DARK_GREY, LV_PART_MAIN);
    lv_obj_set_style_border_width(spin, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(spin, 8, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(spin, LV_OPA_TRANSP, LV_PART_CURSOR);
    lv_obj_set_style_border_width(spin, 0, LV_PART_CURSOR);
    if (out_spinbox) *out_spinbox = spin;
    // Prevent direct touch on spinbox from moving cursor and changing step.
    // The +/- buttons are the sole input; API calls still work without
    // CLICKABLE.
    lv_obj_clear_flag(spin, LV_OBJ_FLAG_CLICKABLE);
    make_interaction_safe(spin);  // <-- ANTI-SHUDDER

    // Increment button – Widget A pattern (plain lv_obj, NO lv_button)
    lv_obj_t* btn_inc = lv_obj_create(row);
    lv_obj_remove_style_all(btn_inc);
    lv_obj_add_flag(btn_inc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(btn_inc, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(btn_inc, BTN_W, BTN_H);
    lv_obj_set_pos(btn_inc, X_INC, 2);
    lv_obj_set_style_bg_color(btn_inc, C_DARK_GREY, 0);
    lv_obj_set_style_bg_opa(btn_inc, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(btn_inc, 8, 0);
    lv_obj_t* lbl_inc = lv_label_create(btn_inc);
    lv_label_set_text(lbl_inc, LV_SYMBOL_PLUS);
    lv_obj_set_style_text_color(lbl_inc, C_WHITE, 0);
    lv_obj_set_style_text_font(lbl_inc, &lv_font_montserrat_18, 0);
    lv_obj_center(lbl_inc);
    make_interaction_safe(btn_inc);  // <-- ANTI-SHUDDER

    // Unit label
    lv_obj_t* lbl_unit = lv_label_create(row);
    lv_label_set_text(lbl_unit, unit);
    lv_obj_set_style_text_color(lbl_unit, C_GREY, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_unit, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_pos(lbl_unit, X_INC + BTN_W + 6, 16);

    // Wire +/- to spinbox using hold-to-repeat handlers
    HoldRepeatCtx* ctx_dec = alloc_repeat_ctx(spin, false, false);
    HoldRepeatCtx* ctx_inc = alloc_repeat_ctx(spin, true, false);

    if (ctx_dec) {
        lv_obj_add_event_cb(btn_dec, on_repeat_pressed, LV_EVENT_PRESSED,
                            ctx_dec);
        lv_obj_add_event_cb(btn_dec, on_repeat_pressing, LV_EVENT_PRESSING,
                            ctx_dec);
        lv_obj_add_event_cb(btn_dec, on_repeat_released, LV_EVENT_RELEASED,
                            ctx_dec);
        lv_obj_add_event_cb(btn_dec, on_repeat_released, LV_EVENT_PRESS_LOST,
                            ctx_dec);
    }
    if (ctx_inc) {
        lv_obj_add_event_cb(btn_inc, on_repeat_pressed, LV_EVENT_PRESSED,
                            ctx_inc);
        lv_obj_add_event_cb(btn_inc, on_repeat_pressing, LV_EVENT_PRESSING,
                            ctx_inc);
        lv_obj_add_event_cb(btn_inc, on_repeat_released, LV_EVENT_RELEASED,
                            ctx_inc);
        lv_obj_add_event_cb(btn_inc, on_repeat_released, LV_EVENT_PRESS_LOST,
                            ctx_inc);
    }

    lv_obj_add_event_cb(spin, on_spinbox_change, LV_EVENT_VALUE_CHANGED,
                        nullptr);

    register_lockable(btn_dec);
    register_lockable(spin);
    register_lockable(btn_inc);

    return row;
}

// ============================================================
// PULSE TAB: Lock/unlock all controls based on armed state
// ============================================================
static void lock_pulse_tab(bool locked) {
    static bool prev_locked = false;
    if (locked == prev_locked) return;
    prev_locked = locked;

    for (int i = 0; i < lockable_count; i++) {
        if (!lockable_objs[i]) continue;
        if (locked)
            lv_obj_add_state(lockable_objs[i], LV_STATE_DISABLED);
        else
            lv_obj_remove_state(lockable_objs[i], LV_STATE_DISABLED);
    }

    auto set_lock = [&](lv_obj_t* obj) {
        if (!obj) return;
        if (locked)
            lv_obj_add_state(obj, LV_STATE_DISABLED);
        else
            lv_obj_remove_state(obj, LV_STATE_DISABLED);
    };
    for (int i = 0; i < 3; i++) set_lock(btn_mode[i]);
    set_lock(btn_power_minus);
    set_lock(btn_power_plus);
    set_lock(btn_preheat);
    set_lock(btn_apply);
}

// ============================================================
// PULSE TAB: helper – create a section header
// ============================================================
static lv_obj_t* make_section_header(lv_obj_t* parent, const char* text, int x,
                                     int y) {
    lv_obj_t* lbl = lv_label_create(parent);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_color(lbl, C_ACCENT, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_set_pos(lbl, x, y);
    return lbl;
}

// ============================================================
// PULSE TAB: helper – create a mode toggle button
// Uses Widget A pattern (plain lv_obj, NO lv_button)
// ============================================================
static lv_obj_t* make_mode_button(lv_obj_t* parent, const char* text, int x,
                                  int y, int w, int h, int mode_idx) {
    lv_obj_t* btn = lv_obj_create(parent);
    lv_obj_remove_style_all(btn);
    lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_pos(btn, x, y);
    bool sel = ((int)draft_mode == mode_idx + 1);
    lv_obj_set_style_bg_color(btn, sel ? C_ACCENT : C_DARK_GREY, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(btn, 10, 0);

    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_color(lbl, C_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_center(lbl);

    lv_obj_add_event_cb(btn, on_mode_btn_click, LV_EVENT_CLICKED,
                        (void*)(intptr_t)mode_idx);
    make_interaction_safe(btn);  // <-- ANTI-SHUDDER
    return btn;
}

// ============================================================
// PULSE TAB BUILDER – Production layout with anti-shudder
// ============================================================
static void build_pulse_tab(lv_obj_t* tab) {
    lv_obj_set_style_bg_color(tab, C_BG, LV_PART_MAIN);
    lv_obj_set_style_pad_all(tab, 8, LV_PART_MAIN);
    // Disable scrolling on Pulse tab to prevent scroll-induced redraws
    lv_obj_set_scrollbar_mode(tab, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);

    lockable_count = 0;

    const int L = 4;
    const int R = 400;
    int ly, ry;

    // ======== LEFT COLUMN ========

    ly = 2;
    make_section_header(tab, "Weld Mode", L, ly);
    ly += 28;

    const int MODE_BTN_W = 115;
    const int MODE_BTN_H = 48;
    const int MODE_GAP = 8;
    const char* mode_names[3] = {"Single", "Double", "Triple"};
    for (int i = 0; i < 3; i++) {
        btn_mode[i] = make_mode_button(tab, mode_names[i],
                                       L + i * (MODE_BTN_W + MODE_GAP), ly,
                                       MODE_BTN_W, MODE_BTN_H, i);
    }
    ly += MODE_BTN_H + 16;

    make_section_header(tab, "Pulse Timing", L, ly);
    ly += 26;

    const int ROW_H = 54;

    row_d1 =
        make_touch_row(tab, "Pulse", "ms", &spin_d1, 1, 999, draft_d1, 3, 1);
    lv_obj_set_pos(row_d1, L, ly);
    ly += ROW_H;

    row_gap1 =
        make_touch_row(tab, "Gap", "ms", &spin_gap1, 1, 999, draft_gap1, 3, 1);
    lv_obj_set_pos(row_gap1, L, ly);
    ly += ROW_H;

    row_d2 =
        make_touch_row(tab, "Pulse 2", "ms", &spin_d2, 1, 999, draft_d2, 3, 1);
    lv_obj_set_pos(row_d2, L, ly);
    ly += ROW_H;

    row_gap2 = make_touch_row(tab, "Gap 2", "ms", &spin_gap2, 1, 999,
                              draft_gap2, 3, 1);
    lv_obj_set_pos(row_gap2, L, ly);
    ly += ROW_H;

    row_d3 =
        make_touch_row(tab, "Pulse 3", "ms", &spin_d3, 1, 999, draft_d3, 3, 1);
    lv_obj_set_pos(row_d3, L, ly);

    update_pulse_field_visibility();

    // ======== RIGHT COLUMN ========

    ry = 2;

    // Power row constants (declared early for PH_X calculation)
    const int PWR_BTN_W = 56;
    const int PWR_BTN_H = 46;
    const int PWR_VAL_W = 84;
    const int PWR_GAP = 8;  // symmetric gap between [-] [val] [+]

    // Derived positions for evenly-spaced  [-] [value] [+]  row
    const int X_PWR_MINUS = R;
    const int X_PWR_VAL = R + PWR_BTN_W + PWR_GAP;
    const int X_PWR_PLUS = X_PWR_VAL + PWR_VAL_W + PWR_GAP;

    make_section_header(tab, "Weld Power", R, ry);

    // "Preheat" label above the ON/OFF toggle, right of power + button with
    // 20px gap
    const int PH_X = X_PWR_PLUS + PWR_BTN_W + 20;
    make_section_header(tab, "Preheat", PH_X, ry);

    ry += 30;

    // Power row: [-] [value] [+]  (Widget A pattern, NO slider)

    btn_power_minus = lv_obj_create(tab);
    lv_obj_remove_style_all(btn_power_minus);
    lv_obj_add_flag(btn_power_minus, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(btn_power_minus, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(btn_power_minus, PWR_BTN_W, PWR_BTN_H);
    lv_obj_set_pos(btn_power_minus, X_PWR_MINUS, ry);
    lv_obj_set_style_bg_color(btn_power_minus, C_DARK_GREY, 0);
    lv_obj_set_style_bg_opa(btn_power_minus, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(btn_power_minus, 8, 0);
    {
        lv_obj_t* lbl = lv_label_create(btn_power_minus);
        lv_label_set_text(lbl, LV_SYMBOL_MINUS);
        lv_obj_set_style_text_color(lbl, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
        lv_obj_center(lbl);
    }
    {
        HoldRepeatCtx* ctx = alloc_repeat_ctx(nullptr, false, true);
        if (ctx) {
            lv_obj_add_event_cb(btn_power_minus, on_repeat_pressed,
                                LV_EVENT_PRESSED, ctx);
            lv_obj_add_event_cb(btn_power_minus, on_repeat_pressing,
                                LV_EVENT_PRESSING, ctx);
            lv_obj_add_event_cb(btn_power_minus, on_repeat_released,
                                LV_EVENT_RELEASED, ctx);
            lv_obj_add_event_cb(btn_power_minus, on_repeat_released,
                                LV_EVENT_PRESS_LOST, ctx);
        }
    }
    make_interaction_safe(btn_power_minus);

    // Fixed-width container for power value – centers text between [-] and [+]
    lv_obj_t* pwr_val_box = lv_obj_create(tab);
    lv_obj_remove_style_all(pwr_val_box);
    lv_obj_clear_flag(pwr_val_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(pwr_val_box, PWR_VAL_W, PWR_BTN_H);
    lv_obj_set_pos(pwr_val_box, X_PWR_VAL, ry);
    lv_obj_set_scrollbar_mode(pwr_val_box, LV_SCROLLBAR_MODE_OFF);

    lbl_power_val = lv_label_create(pwr_val_box);
    {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d %%", (int)draft_power);
        lv_label_set_text(lbl_power_val, buf);
    }
    lv_obj_set_style_text_color(lbl_power_val, C_ACCENT, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_power_val, &lv_font_montserrat_20,
                               LV_PART_MAIN);
    lv_obj_center(lbl_power_val);  // centered inside fixed-width container

    btn_power_plus = lv_obj_create(tab);
    lv_obj_remove_style_all(btn_power_plus);
    lv_obj_add_flag(btn_power_plus, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(btn_power_plus, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(btn_power_plus, PWR_BTN_W, PWR_BTN_H);
    lv_obj_set_pos(btn_power_plus, X_PWR_PLUS, ry);
    lv_obj_set_style_bg_color(btn_power_plus, C_DARK_GREY, 0);
    lv_obj_set_style_bg_opa(btn_power_plus, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(btn_power_plus, 8, 0);
    {
        lv_obj_t* lbl = lv_label_create(btn_power_plus);
        lv_label_set_text(lbl, LV_SYMBOL_PLUS);
        lv_obj_set_style_text_color(lbl, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
        lv_obj_center(lbl);
    }
    {
        HoldRepeatCtx* ctx = alloc_repeat_ctx(nullptr, true, true);
        if (ctx) {
            lv_obj_add_event_cb(btn_power_plus, on_repeat_pressed,
                                LV_EVENT_PRESSED, ctx);
            lv_obj_add_event_cb(btn_power_plus, on_repeat_pressing,
                                LV_EVENT_PRESSING, ctx);
            lv_obj_add_event_cb(btn_power_plus, on_repeat_released,
                                LV_EVENT_RELEASED, ctx);
            lv_obj_add_event_cb(btn_power_plus, on_repeat_released,
                                LV_EVENT_PRESS_LOST, ctx);
        }
    }
    make_interaction_safe(btn_power_plus);

    register_lockable(btn_power_minus);
    register_lockable(btn_power_plus);

    // Preheat ON/OFF toggle – same horizontal level as Weld Power +/- (Widget A
    // pattern)
    btn_preheat = lv_obj_create(tab);
    lv_obj_remove_style_all(btn_preheat);
    lv_obj_add_flag(btn_preheat, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(btn_preheat, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(btn_preheat, 80, PWR_BTN_H);
    lv_obj_set_pos(btn_preheat, PH_X, ry);  // same Y as power buttons
    lv_obj_set_style_bg_color(btn_preheat,
                              draft_preheat_en ? C_GREEN : C_DARK_GREY, 0);
    lv_obj_set_style_bg_opa(btn_preheat, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(btn_preheat, 16, 0);
    lbl_preheat = lv_label_create(btn_preheat);
    lv_label_set_text(lbl_preheat, draft_preheat_en ? "ON" : "OFF");
    lv_obj_set_style_text_color(lbl_preheat, C_WHITE, 0);
    lv_obj_set_style_text_font(lbl_preheat, &lv_font_montserrat_16, 0);
    lv_obj_center(lbl_preheat);
    lv_obj_add_event_cb(btn_preheat, on_preheat_toggle, LV_EVENT_CLICKED,
                        nullptr);
    make_interaction_safe(btn_preheat);
    register_lockable(btn_preheat);

    ry += PWR_BTN_H + 16;

    row_ph_ms = make_touch_row(tab, "Duration", "ms", &spin_ph_ms, 1, 200,
                               draft_preheat_ms, 3, 1);
    lv_obj_set_pos(row_ph_ms, R, ry);
    ry += ROW_H;

    row_ph_pct = make_touch_row(tab, "Power", "%", &spin_ph_pct, 10, 100,
                                draft_preheat_pct, 3, 5);
    lv_obj_set_pos(row_ph_pct, R, ry);
    ry += ROW_H;

    row_ph_gap = make_touch_row(tab, "Gap", "ms", &spin_ph_gap, 1, 100,
                                draft_preheat_gap, 3, 1);
    lv_obj_set_pos(row_ph_gap, R, ry);
    ry += ROW_H;

    update_preheat_visibility();

    // --- APPLY Button (Widget A pattern – plain lv_obj, NO lv_button, NO
    // Revert) ---
    const int BTN_W = 200;
    const int BTN_H = 50;
    ry += 18;

    btn_apply = lv_obj_create(tab);
    lv_obj_remove_style_all(btn_apply);
    lv_obj_add_flag(btn_apply, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(btn_apply, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(btn_apply, BTN_W, BTN_H);
    lv_obj_set_pos(btn_apply, R, ry);
    lv_obj_set_style_bg_color(btn_apply, C_GREEN,
                              0);  // green = synced (initial)
    lv_obj_set_style_bg_opa(btn_apply, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(btn_apply, 10, 0);
    lv_obj_add_event_cb(btn_apply, on_apply_click, LV_EVENT_CLICKED, nullptr);
    make_interaction_safe(btn_apply);

    lbl_apply = lv_label_create(btn_apply);
    lv_label_set_text(lbl_apply, "Apply Settings");
    lv_obj_set_style_text_color(lbl_apply, C_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_apply, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_center(lbl_apply);

    ry += BTN_H + 10;

    lbl_pending = lv_label_create(tab);
    lv_label_set_text(lbl_pending, LV_SYMBOL_WARNING " Changes pending");
    lv_obj_set_style_text_color(lbl_pending, C_YELLOW, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_pending, &lv_font_montserrat_16,
                               LV_PART_MAIN);
    lv_obj_set_pos(lbl_pending, R, ry);
    lv_obj_add_flag(lbl_pending, LV_OBJ_FLAG_HIDDEN);
}

// Forward declaration (defined with the Config tab helpers below)
static lv_obj_t* make_cfg_button(lv_obj_t* parent, const char* text,
                                 lv_obj_t** out_label, int x, int y, int w,
                                 int h, lv_color_t bg_color);

// ============================================================
// JOULE TAB: draft clamping + label refresh
// ============================================================
static void clamp_joule_draft() {
    if (!isfinite(_joule_draft_target)) _joule_draft_target = 50.0f;
    if (_joule_draft_target < JOULE_TARGET_MIN)
        _joule_draft_target = JOULE_TARGET_MIN;
    if (_joule_draft_target > JOULE_TARGET_MAX)
        _joule_draft_target = JOULE_TARGET_MAX;
    if (_joule_draft_maxms < JOULE_MAXMS_MIN)
        _joule_draft_maxms = JOULE_MAXMS_MIN;
    if (_joule_draft_maxms > JOULE_MAXMS_MAX)
        _joule_draft_maxms = JOULE_MAXMS_MAX;
}

static void paint_joule_tab() {
    clamp_joule_draft();

    if (lbl_joule_target) {
        char buf[24];
        snprintf(buf, sizeof(buf), "%.0f J", (double)_joule_draft_target);
        lv_label_set_text(lbl_joule_target, buf);
    }
    if (lbl_joule_maxdur) {
        char buf[24];
        snprintf(buf, sizeof(buf), "%u ms", (unsigned)_joule_draft_maxms);
        lv_label_set_text(lbl_joule_maxdur, buf);
    }

    // Apply button state machine:
    //   _joule_dirty == true  -> "APPLY"  (green, tappable)
    //   _joule_dirty == false -> "APPLIED" (grey, already sent / in sync)
    if (btn_joule_apply)
        lv_obj_set_style_bg_color(btn_joule_apply,
                                  _joule_dirty ? C_GREEN : C_DARK_GREY, 0);
    if (lbl_joule_apply)
        lv_label_set_text(lbl_joule_apply, _joule_dirty ? "APPLY" : "APPLIED");
}

// ============================================================
// JOULE TAB: event handlers
// ============================================================
// Generic step functions for the Joule +/- buttons. Driven either by a single
// tap or by the hold-to-repeat handler (when enabled in Config).
static void joule_target_step(bool inc) {
    if (inc)
        _joule_draft_target += JOULE_TARGET_STEP;
    else
        _joule_draft_target -= JOULE_TARGET_STEP;
    _joule_dirty = true;  // re-arm APPLY
    paint_joule_tab();    // clamps + repaints
}

static void joule_maxdur_step(bool inc) {
    if (inc) {
        _joule_draft_maxms += JOULE_MAXMS_STEP;
    } else {
        // Guard against uint16 underflow wrap before clamp_joule_draft runs
        if (_joule_draft_maxms >= JOULE_MAXMS_MIN + JOULE_MAXMS_STEP)
            _joule_draft_maxms -= JOULE_MAXMS_STEP;
        else
            _joule_draft_maxms = JOULE_MAXMS_MIN;
    }
    _joule_dirty = true;  // re-arm APPLY
    paint_joule_tab();    // clamps + repaints
}

// Commit the current Joule settings to the STM32 when the user taps APPLY.
// Tapping is the only commit path – edits stay local (dirty) until applied.
static void on_joule_apply(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (!_joule_dirty) return;  // nothing to send
    clamp_joule_draft();
    if (_joule_apply_cb)
        _joule_apply_cb(_joule_draft_mode, _joule_draft_target,
                        _joule_draft_maxms);
    _joule_dirty = false;  // back to "APPLIED" grey
    paint_joule_tab();
}

// ============================================================
// JOULE TAB BUILDER (replaces Telemetry): energy target + safety limit
// ============================================================
static void build_joule_tab(lv_obj_t* tab) {
    lv_obj_set_style_bg_color(tab, C_BG, LV_PART_MAIN);
    lv_obj_set_style_pad_all(tab, 10, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(tab, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);

    const int LABEL_X = 14;
    int y = 14;

    // ---- Header ----
    // The welding mode (TIME vs JOULE) is now chosen from the STATUS tab's
    // MODE switcher (long-press). This tab only sets the JOULE-mode targets.
    {
        lv_obj_t* hdr = lv_label_create(tab);
        lv_label_set_text(hdr, "ENERGY SETTINGS");
        lv_obj_set_style_text_color(hdr, C_WHITE, 0);
        lv_obj_set_style_text_font(hdr, &lv_font_montserrat_20, 0);
        lv_obj_set_pos(hdr, LABEL_X, y);

        lv_obj_t* sub = lv_label_create(tab);
        lv_label_set_text(sub, "Switch TIME / JOULE on the STATUS tab "
                               "(long-press the MODE box).");
        lv_obj_set_style_text_color(sub, C_GREY, 0);
        lv_obj_set_style_text_font(sub, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(sub, LABEL_X, y + 28);
    }
    y += 64;

    // Stepper geometry – steppers occupy the left column; the APPLY button
    // sits in the right column (x>=440), so keep the +/- value field narrow
    // enough that the increment button ends well left of the APPLY button.
    const int STEP_W = 80;
    const int VAL_W = 220;
    const int DEC_X = LABEL_X;
    const int VAL_X = DEC_X + STEP_W + 12;
    const int INC_X = VAL_X + VAL_W + 12;
    const int BTN_H = 72;

    // ---- Target Energy stepper ----
    {
        lv_obj_t* lbl = lv_label_create(tab);
        lv_label_set_text(lbl, "Target Energy");
        lv_obj_set_style_text_color(lbl, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
        lv_obj_set_pos(lbl, LABEL_X, y);
    }
    y += 30;
    {
        lv_obj_t* l = nullptr;
        lv_obj_t* d = make_cfg_button(tab, "-", &l, DEC_X, y, STEP_W, BTN_H,
                                      C_DARK_GREY);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_24, 0);
        HoldRepeatCtx* cd = alloc_repeat_ctx_fn(joule_target_step, false);
        lv_obj_add_event_cb(d, on_repeat_pressed, LV_EVENT_PRESSED, cd);
        lv_obj_add_event_cb(d, on_repeat_pressing, LV_EVENT_PRESSING, cd);
        lv_obj_add_event_cb(d, on_repeat_released, LV_EVENT_RELEASED, cd);
        lv_obj_add_event_cb(d, on_repeat_released, LV_EVENT_PRESS_LOST, cd);

        lbl_joule_target = lv_label_create(tab);
        lv_label_set_text(lbl_joule_target, "50 J");
        lv_obj_set_style_text_color(lbl_joule_target, C_ACCENT, 0);
        lv_obj_set_style_text_font(lbl_joule_target, &lv_font_montserrat_48, 0);
        lv_obj_set_style_text_align(lbl_joule_target, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_width(lbl_joule_target, VAL_W);
        lv_obj_set_pos(lbl_joule_target, VAL_X, y + 12);

        lv_obj_t* l2 = nullptr;
        lv_obj_t* i = make_cfg_button(tab, "+", &l2, INC_X, y, STEP_W, BTN_H,
                                      C_DARK_GREY);
        lv_obj_set_style_text_font(l2, &lv_font_montserrat_24, 0);
        HoldRepeatCtx* ci = alloc_repeat_ctx_fn(joule_target_step, true);
        lv_obj_add_event_cb(i, on_repeat_pressed, LV_EVENT_PRESSED, ci);
        lv_obj_add_event_cb(i, on_repeat_pressing, LV_EVENT_PRESSING, ci);
        lv_obj_add_event_cb(i, on_repeat_released, LV_EVENT_RELEASED, ci);
        lv_obj_add_event_cb(i, on_repeat_released, LV_EVENT_PRESS_LOST, ci);
    }
    y += BTN_H + 28;

    // ---- Max Duration (safety limit) stepper ----
    {
        lv_obj_t* lbl = lv_label_create(tab);
        lv_label_set_text(lbl, "Max Duration (safety limit)");
        lv_obj_set_style_text_color(lbl, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
        lv_obj_set_pos(lbl, LABEL_X, y);
    }
    y += 30;
    {
        lv_obj_t* l = nullptr;
        lv_obj_t* d = make_cfg_button(tab, "-", &l, DEC_X, y, STEP_W, BTN_H,
                                      C_DARK_GREY);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_24, 0);
        HoldRepeatCtx* cd = alloc_repeat_ctx_fn(joule_maxdur_step, false);
        lv_obj_add_event_cb(d, on_repeat_pressed, LV_EVENT_PRESSED, cd);
        lv_obj_add_event_cb(d, on_repeat_pressing, LV_EVENT_PRESSING, cd);
        lv_obj_add_event_cb(d, on_repeat_released, LV_EVENT_RELEASED, cd);
        lv_obj_add_event_cb(d, on_repeat_released, LV_EVENT_PRESS_LOST, cd);

        lbl_joule_maxdur = lv_label_create(tab);
        lv_label_set_text(lbl_joule_maxdur, "40 ms");
        lv_obj_set_style_text_color(lbl_joule_maxdur, C_YELLOW, 0);
        lv_obj_set_style_text_font(lbl_joule_maxdur, &lv_font_montserrat_48, 0);
        lv_obj_set_style_text_align(lbl_joule_maxdur, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_width(lbl_joule_maxdur, VAL_W);
        lv_obj_set_pos(lbl_joule_maxdur, VAL_X, y + 12);

        lv_obj_t* l2 = nullptr;
        lv_obj_t* i = make_cfg_button(tab, "+", &l2, INC_X, y, STEP_W, BTN_H,
                                      C_DARK_GREY);
        lv_obj_set_style_text_font(l2, &lv_font_montserrat_24, 0);
        HoldRepeatCtx* ci = alloc_repeat_ctx_fn(joule_maxdur_step, true);
        lv_obj_add_event_cb(i, on_repeat_pressed, LV_EVENT_PRESSED, ci);
        lv_obj_add_event_cb(i, on_repeat_pressing, LV_EVENT_PRESSING, ci);
        lv_obj_add_event_cb(i, on_repeat_released, LV_EVENT_RELEASED, ci);
        lv_obj_add_event_cb(i, on_repeat_released, LV_EVENT_PRESS_LOST, ci);
    }
    y += BTN_H + 22;

    // ---- APPLY button (right side, spans both steppers vertically) ----
    // Widget A pattern (plain lv_obj, no lv_button) to avoid touch shudder.
    // Shows "APPLY" (green) when there are unsent edits, "APPLIED" (grey)
    // once the draft has been committed to / is in sync with the STM32.
    {
        btn_joule_apply = lv_obj_create(tab);
        lv_obj_remove_style_all(btn_joule_apply);
        lv_obj_add_flag(btn_joule_apply, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(btn_joule_apply, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_size(btn_joule_apply, 330, 174);
        lv_obj_set_pos(btn_joule_apply, 440, 120);
        lv_obj_set_style_bg_color(btn_joule_apply, C_DARK_GREY, 0);
        lv_obj_set_style_bg_opa(btn_joule_apply, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(btn_joule_apply, 10, 0);

        lbl_joule_apply = lv_label_create(btn_joule_apply);
        lv_label_set_text(lbl_joule_apply, "APPLIED");
        lv_obj_set_style_text_color(lbl_joule_apply, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl_joule_apply, &lv_font_montserrat_24, 0);
        lv_obj_center(lbl_joule_apply);

        lv_obj_add_event_cb(btn_joule_apply, on_joule_apply, LV_EVENT_CLICKED,
                            nullptr);
        make_interaction_safe(btn_joule_apply);
    }

    // NOTE: the live workpiece-Joules readout was intentionally removed from
    // this tab. The measured energy is shown on the STATUS tab's "Last Weld"
    // section (Duration | Peak | Avg | Joules) instead.

    // ---- Explanatory note (full width, below the steppers) ----
    {
        lv_obj_t* note = lv_label_create(tab);
        lv_label_set_text(
            note,
            "JOULE mode fires until the measured weld energy reaches the "
            "target, then stops. Max Duration caps the pulse length for "
            "safety. TIME mode uses the pulse timing set on the PULSE tab. "
            "Adjust the values, then tap APPLY to send them to the welder.");
        lv_obj_set_style_text_color(note, C_GREY, 0);
        lv_obj_set_style_text_font(note, &lv_font_montserrat_16, 0);
        lv_label_set_long_mode(note, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(note, 760);
        lv_obj_set_pos(note, LABEL_X, y);
    }

    paint_joule_tab();
}

// ============================================================
// SETUP TAB helpers
// ============================================================

// Create a "key:  value" row inside a panel. Returns the value label.
static lv_obj_t* make_kv_row(lv_obj_t* parent, const char* key,
                             lv_obj_t** out_val, int x, int y, int key_w) {
    lv_obj_t* k = lv_label_create(parent);
    lv_label_set_text(k, key);
    lv_obj_set_style_text_color(k, C_GREY, 0);
    lv_obj_set_style_text_font(k, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(k, x, y + 2);

    lv_obj_t* v = lv_label_create(parent);
    lv_label_set_text(v, "--");
    lv_obj_set_style_text_color(v, C_WHITE, 0);
    lv_obj_set_style_text_font(v, &lv_font_montserrat_16, 0);
    lv_obj_set_pos(v, x + key_w, y);
    if (out_val) *out_val = v;
    return v;
}

// ---- Confirmation modal (Restart / Factory Reset) ----
static void confirm_yes_event(lv_event_t* e) {
    (void)e;
    int act = confirm_action;
    if (confirm_overlay) lv_obj_add_flag(confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    if (act == 1 && _restart_cb) {
        _restart_cb();
    } else if (act == 2 && _factory_reset_cb) {
        _factory_reset_cb();
    }
}

static void confirm_no_event(lv_event_t* e) {
    (void)e;
    if (confirm_overlay) lv_obj_add_flag(confirm_overlay, LV_OBJ_FLAG_HIDDEN);
}

// Build the modal once (lazily), on the top layer so it floats over any tab.
static void ensure_confirm_overlay() {
    if (confirm_overlay) return;

    confirm_overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(confirm_overlay);
    lv_obj_set_size(confirm_overlay, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(confirm_overlay, 0, 0);
    lv_obj_set_style_bg_color(confirm_overlay, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(confirm_overlay, LV_OPA_70, 0);
    lv_obj_clear_flag(confirm_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(confirm_overlay, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t* card = make_panel(confirm_overlay, 0, 0, 460, 220);
    lv_obj_align(card, LV_ALIGN_CENTER, 0, 0);

    lbl_confirm_msg = lv_label_create(card);
    lv_label_set_text(lbl_confirm_msg, "Are you sure?");
    lv_obj_set_style_text_color(lbl_confirm_msg, C_WHITE, 0);
    lv_obj_set_style_text_font(lbl_confirm_msg, &lv_font_montserrat_18, 0);
    lv_label_set_long_mode(lbl_confirm_msg, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_confirm_msg, 420);
    lv_obj_set_style_text_align(lbl_confirm_msg, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl_confirm_msg, LV_ALIGN_TOP_MID, 0, 30);

    lv_obj_t* lbl_no = nullptr;
    lv_obj_t* btn_no =
        make_cfg_button(card, "Cancel", &lbl_no, 40, 140, 170, 56, C_DARK_GREY);
    lv_obj_add_event_cb(btn_no, confirm_no_event, LV_EVENT_CLICKED, nullptr);

    lv_obj_t* lbl_yes = nullptr;
    lv_obj_t* btn_yes =
        make_cfg_button(card, "Confirm", &lbl_yes, 250, 140, 170, 56, C_RED);
    lv_obj_add_event_cb(btn_yes, confirm_yes_event, LV_EVENT_CLICKED, nullptr);
}

static void show_confirm(int action, const char* msg) {
    ensure_confirm_overlay();
    confirm_action = action;
    if (lbl_confirm_msg) lv_label_set_text(lbl_confirm_msg, msg);
    lv_obj_clear_flag(confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(confirm_overlay);
}

// ---- Setup-tab button events ----
static void on_setup_reconfigure(lv_event_t* e) {
    (void)e;
    if (_wifi_reconfigure_cb) _wifi_reconfigure_cb();
}

static void on_setup_restart(lv_event_t* e) {
    (void)e;
    show_confirm(1, "Restart the controller now?\nSaved WiFi will reconnect.");
}

static void on_setup_factory_reset(lv_event_t* e) {
    (void)e;
    show_confirm(2,
                 "FACTORY RESET erases WiFi credentials, recipes, config and "
                 "weld stats, then reboots into setup mode. Continue?");
}

// ============================================================
// SETUP TAB BUILDER  (replaces the old "Logs" placeholder)
//   Section 1: WiFi status + Reconfigure
//   Section 2: System info
//   Section 3: Maintenance (Restart / Factory Reset)
// ============================================================
static void build_setup_tab(lv_obj_t* tab) {
    lv_obj_set_style_bg_color(tab, C_BG, LV_PART_MAIN);
    lv_obj_set_style_pad_all(tab, 10, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(tab, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);

    const int GAP = 10;
    const int COL_W = 380;
    const int COL2_X = COL_W + GAP;  // 390

    // ---------------------------------------------------------
    // Section 1: WiFi / Connection (top-left panel)
    // ---------------------------------------------------------
    {
        lv_obj_t* p = make_panel(tab, 0, 0, COL_W, 190);

        lv_obj_t* title = lv_label_create(p);
        lv_label_set_text(title, LV_SYMBOL_WIFI "  Connection");
        lv_obj_set_style_text_color(title, C_ACCENT, 0);
        lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(title, 12, 10);

        make_kv_row(p, "Status", &lbl_setup_wifi_state, 12, 42, 95);
        make_kv_row(p, "Network", &lbl_setup_wifi_ssid, 12, 68, 95);
        make_kv_row(p, "IP", &lbl_setup_wifi_ip, 12, 94, 95);
        make_kv_row(p, "Signal", &lbl_setup_wifi_rssi, 12, 120, 95);

        lv_obj_t* lbl_rc = nullptr;
        lv_obj_t* btn_rc = make_cfg_button(p, LV_SYMBOL_REFRESH " Reconfigure",
                                           &lbl_rc, 12, 148, COL_W - 24, 34,
                                           C_ACCENT);
        lv_obj_add_event_cb(btn_rc, on_setup_reconfigure, LV_EVENT_CLICKED,
                            nullptr);
    }

    // ---------------------------------------------------------
    // Section 2: System info (top-right panel)
    // ---------------------------------------------------------
    {
        lv_obj_t* p = make_panel(tab, COL2_X, 0, COL_W, 190);

        lv_obj_t* title = lv_label_create(p);
        lv_label_set_text(title, LV_SYMBOL_LIST "  System Info");
        lv_obj_set_style_text_color(title, C_ACCENT, 0);
        lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(title, 12, 10);

        make_kv_row(p, "Firmware", &lbl_setup_fw, 12, 42, 110);
        make_kv_row(p, "Chip", &lbl_setup_chip, 12, 68, 110);
        make_kv_row(p, "Flash", &lbl_setup_flash, 12, 94, 110);
        make_kv_row(p, "Total welds", &lbl_setup_welds, 12, 120, 110);
    }

    // ---------------------------------------------------------
    // Section 3: Maintenance (full-width panel)
    // ---------------------------------------------------------
    {
        lv_obj_t* p = make_panel(tab, 0, 200, COL_W * 2 + GAP, 100);

        lv_obj_t* title = lv_label_create(p);
        lv_label_set_text(title, LV_SYMBOL_SETTINGS "  Maintenance");
        lv_obj_set_style_text_color(title, C_ACCENT, 0);
        lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(title, 12, 10);

        lv_obj_t* lbl_rs = nullptr;
        lv_obj_t* btn_rs = make_cfg_button(p, LV_SYMBOL_POWER " Restart",
                                           &lbl_rs, 12, 44, 360, 44,
                                           C_DARK_GREY);
        lv_obj_add_event_cb(btn_rs, on_setup_restart, LV_EVENT_CLICKED,
                            nullptr);

        lv_obj_t* lbl_fr = nullptr;
        lv_obj_t* btn_fr = make_cfg_button(p, LV_SYMBOL_TRASH " Factory Reset",
                                           &lbl_fr, 388, 44, 360, 44, C_RED);
        lv_obj_add_event_cb(btn_fr, on_setup_factory_reset, LV_EVENT_CLICKED,
                            nullptr);
    }
}

// ============================================================
// CONFIG TAB: Helper – create a toggle/cycle button (Widget A)
// Returns the button object. Sets *out_label to inner label.
// ============================================================
static lv_obj_t* make_cfg_button(lv_obj_t* parent, const char* text,
                                 lv_obj_t** out_label, int x, int y, int w,
                                 int h, lv_color_t bg_color) {
    lv_obj_t* btn = lv_obj_create(parent);
    lv_obj_remove_style_all(btn);
    lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_style_bg_color(btn, bg_color, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(btn, 10, 0);
    lv_obj_set_scrollbar_mode(btn, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_color(lbl, C_WHITE, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
    lv_obj_center(lbl);

    make_interaction_safe(btn);
    if (out_label) *out_label = lbl;
    return btn;
}

// ============================================================
// CONFIG TAB: Helpers – update label text for each setting
// ============================================================
static void update_cfg_hold_repeat_label() {
    if (lbl_cfg_hold_repeat)
        lv_label_set_text(lbl_cfg_hold_repeat,
                          _cfg.hold_to_repeat ? "ON" : "OFF");
    if (btn_cfg_hold_repeat)
        lv_obj_set_style_bg_color(btn_cfg_hold_repeat,
                                  _cfg.hold_to_repeat ? C_GREEN : C_DARK_GREY,
                                  0);
}

static const char* time_step_text(uint8_t ms) {
    switch (ms) {
        case 1:
            return "1 ms";
        case 5:
            return "5 ms";
        case 10:
            return "10 ms";
        default:
            return "1 ms";
    }
}

static void update_cfg_time_step_label() {
    if (lbl_cfg_time_step)
        lv_label_set_text(lbl_cfg_time_step, time_step_text(_cfg.time_step_ms));
}

static const char* power_step_text(uint8_t pct) {
    switch (pct) {
        case 1:
            return "1 %";
        case 5:
            return "5 %";
        case 10:
            return "10 %";
        default:
            return "5 %";
    }
}

static void update_cfg_power_step_label() {
    if (lbl_cfg_power_step)
        lv_label_set_text(lbl_cfg_power_step,
                          power_step_text(_cfg.power_step_pct));
}

static void update_cfg_load_last_label() {
    if (lbl_cfg_load_last)
        lv_label_set_text(lbl_cfg_load_last,
                          _cfg.load_last_on_boot ? "ON" : "OFF");
    if (btn_cfg_load_last)
        lv_obj_set_style_bg_color(
            btn_cfg_load_last, _cfg.load_last_on_boot ? C_GREEN : C_DARK_GREY,
            0);
}

static const char* brightness_text(uint8_t b) {
    switch (b) {
        case 0:
            return "LOW";
        case 1:
            return "MED";
        case 2:
            return "HIGH";
        default:
            return "HIGH";
    }
}

static void update_cfg_brightness_label() {
    if (lbl_cfg_brightness)
        lv_label_set_text(lbl_cfg_brightness, brightness_text(_cfg.brightness));
}

// Contact/Probe Hold Time – now stored in _cfg.contact_hold_steps

static const char* hold_time_text(uint8_t steps) {
    switch (steps) {
        case 1:
            return "0.5 s";
        case 2:
            return "1.0 s";
        case 3:
            return "1.5 s";
        case 4:
            return "2.0 s";
        case 5:
            return "2.5 s";
        case 6:
            return "3.0 s";
        case 7:
            return "3.5 s";
        case 8:
            return "4.0 s";
        case 9:
            return "4.5 s";
        case 10:
            return "5.0 s";
        default:
            return "1.0 s";
    }
}

static void update_cfg_hold_time_label() {
    if (lbl_cfg_hold_time)
        lv_label_set_text(lbl_cfg_hold_time,
                          hold_time_text(_cfg.contact_hold_steps));
}

static void update_cfg_cwp_label() {
    if (lbl_cfg_cwp)
        lv_label_set_text(lbl_cfg_cwp, _cfg.contact_with_pedal ? "ON" : "OFF");
    if (btn_cfg_cwp)
        lv_obj_set_style_bg_color(
            btn_cfg_cwp, _cfg.contact_with_pedal ? C_GREEN : C_DARK_GREY, 0);
}

// ============================================================
// CONFIG TAB: Calibration section helpers (read-only display)
// ============================================================
static void update_cfg_cal_labels() {
    if (lbl_cfg_cal_leadr) {
        char text[32];
        snprintf(text, sizeof(text), "%.1fm",
                 (double)_cfg.lead_resistance_mohm);
        lv_label_set_text(lbl_cfg_cal_leadr, text);
    }
}

// ============================================================
// CONFIG TAB: Notify main.cpp of config changes
// ============================================================
static void notify_config_changed() {
    if (_config_cb) _config_cb(_cfg);
}

// ============================================================
// CONFIG TAB: Apply time step to all timing spinboxes
// ============================================================
static void apply_time_step_to_spinboxes() {
    int step = (int)_cfg.time_step_ms;
    if (step <= 0) step = 1;
    lv_obj_t* timing_spins[] = {spin_d1, spin_gap1,  spin_d2,    spin_gap2,
                                spin_d3, spin_ph_ms, spin_ph_gap};
    for (int i = 0; i < 7; i++) {
        if (timing_spins[i]) lv_spinbox_set_step(timing_spins[i], step);
    }
    // Preheat power spinbox uses power step
    if (spin_ph_pct) lv_spinbox_set_step(spin_ph_pct, (int)_cfg.power_step_pct);
}

// ============================================================
// CONFIG TAB: Event callbacks
// ============================================================
static void on_cfg_hold_repeat(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    _cfg.hold_to_repeat = !_cfg.hold_to_repeat;
    update_cfg_hold_repeat_label();
    notify_config_changed();
}

static void on_cfg_time_step(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    // Cycle: 1 -> 5 -> 10 -> 1
    if (_cfg.time_step_ms == 1)
        _cfg.time_step_ms = 5;
    else if (_cfg.time_step_ms == 5)
        _cfg.time_step_ms = 10;
    else
        _cfg.time_step_ms = 1;
    update_cfg_time_step_label();
    apply_time_step_to_spinboxes();
    notify_config_changed();
}

static void on_cfg_power_step(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    // Cycle: 1 -> 5 -> 10 -> 1
    if (_cfg.power_step_pct == 1)
        _cfg.power_step_pct = 5;
    else if (_cfg.power_step_pct == 5)
        _cfg.power_step_pct = 10;
    else
        _cfg.power_step_pct = 1;
    update_cfg_power_step_label();
    apply_time_step_to_spinboxes();  // also updates preheat power step
    notify_config_changed();
}

static void on_cfg_load_last(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    _cfg.load_last_on_boot = !_cfg.load_last_on_boot;
    update_cfg_load_last_label();
    notify_config_changed();
}

static void on_cfg_brightness(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    // Cycle: LOW(0) -> MED(1) -> HIGH(2) -> LOW(0)
    _cfg.brightness = (_cfg.brightness + 1) % 3;
    update_cfg_brightness_label();
    notify_config_changed();
}

static void on_cfg_hold_time_dec(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (_cfg.contact_hold_steps > 1) _cfg.contact_hold_steps--;
    update_cfg_hold_time_label();
    notify_config_changed();
    if (_contact_delay_cb) _contact_delay_cb(_cfg.contact_hold_steps);
}

static void on_cfg_hold_time_inc(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (_cfg.contact_hold_steps < 10) _cfg.contact_hold_steps++;
    update_cfg_hold_time_label();
    notify_config_changed();
    if (_contact_delay_cb) _contact_delay_cb(_cfg.contact_hold_steps);
}

// ============================================================
// CONFIG TAB: AUTO CALIBRATE button handler
// ============================================================
static void on_cfg_calibrate(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (lbl_cfg_cal_status) {
        // Initial hint only; the live status is driven by ui_notify_cal_message()
        // as CAL_STATUS/CAL_RESULT/CAL_ERROR lines arrive from the STM32.
        lv_label_set_text(lbl_cfg_cal_status, "Starting calibration...");
        lv_obj_set_style_text_color(lbl_cfg_cal_status, C_ACCENT, LV_PART_MAIN);
    }
    if (_calibrate_cb) _calibrate_cb();
}

static void on_cfg_cwp_toggle(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    _cfg.contact_with_pedal = !_cfg.contact_with_pedal;
    update_cfg_cwp_label();
    notify_config_changed();
    if (_cwp_cb) _cwp_cb(_cfg.contact_with_pedal);
}

// ============================================================
// CONFIG TAB BUILDER – Scrollable container for future expansion
// ============================================================
static void build_config_tab(lv_obj_t* tab) {
    lv_obj_set_style_bg_color(tab, C_BG, LV_PART_MAIN);
    lv_obj_set_style_pad_all(tab, 0, LV_PART_MAIN);

    // Make tab scrollable vertically
    lv_obj_add_flag(tab, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(tab, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(tab, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_scroll_snap_y(tab, LV_SCROLL_SNAP_NONE);

    // Inner container for all settings. This is a *plain* container – the
    // scrolling is owned exclusively by the parent `tab`. Previously this
    // container kept its default LV_OBJ_FLAG_SCROLLABLE flag, which created a
    // nested scrollable inside the scrollable tab. The two scroll areas then
    // fought over each touch (scroll chaining), producing the slow / sticky
    // feel. Clearing the flag here makes the Config page scroll as one smooth
    // surface.
    lv_obj_t* cont = lv_obj_create(tab);
    lv_obj_remove_style_all(cont);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_width(cont, 780);
    lv_obj_set_style_bg_color(cont, C_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(cont, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(cont, 10, 0);
    // Height will be set after all items are added

    const int LABEL_X = 20;
    const int BTN_X = 430;
    const int BTN_W = 140;
    const int BTN_H = 48;
    const int ROW_H = 60;
    // +/- stepper geometry (dec [value] inc)
    const int STEP_W = 56;
    const int STEP_DEC_X = BTN_X;
    const int STEP_VAL_X = BTN_X + STEP_W + 8;
    const int STEP_VAL_W = 120;
    const int STEP_INC_X = STEP_VAL_X + STEP_VAL_W + 8;
    int y = 4;

    // ============================================================
    // ---- Section: CALIBRATION ----
    // ============================================================
    make_section_header(cont, "Calibration", LABEL_X, y);
    y += 32;

    // Measured lead resistance (read-only)
    {
        lv_obj_t* lbl = lv_label_create(cont);
        lv_label_set_text(lbl, "Lead Resistance (measured)");
        lv_obj_set_style_text_color(lbl, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(lbl, LABEL_X, y + 12);
    }
    lbl_cfg_cal_leadr = lv_label_create(cont);
    lv_label_set_text(lbl_cfg_cal_leadr, "-- m");
    lv_obj_set_style_text_color(lbl_cfg_cal_leadr, C_ACCENT, 0);
    lv_obj_set_style_text_font(lbl_cfg_cal_leadr, &lv_font_montserrat_20, 0);
    lv_obj_set_pos(lbl_cfg_cal_leadr, BTN_X, y + 10);
    y += ROW_H;

    // Calibration age (read-only)
    {
        lv_obj_t* lbl = lv_label_create(cont);
        lv_label_set_text(lbl, "Last Calibrated");
        lv_obj_set_style_text_color(lbl, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(lbl, LABEL_X, y + 12);
    }
    lbl_cfg_cal_age = lv_label_create(cont);
    lv_label_set_text(lbl_cfg_cal_age, "\xE2\x80\x94");
    lv_obj_set_style_text_color(lbl_cfg_cal_age, C_GREY, 0);
    lv_obj_set_style_text_font(lbl_cfg_cal_age, &lv_font_montserrat_16, 0);
    lv_obj_set_pos(lbl_cfg_cal_age, BTN_X, y + 12);
    y += ROW_H;

    // AUTO CALIBRATE button (large)
    btn_cfg_calibrate =
        make_cfg_button(cont, "AUTO CALIBRATE", &lbl_cfg_calibrate, LABEL_X, y,
                        320, 56, C_ACCENT);
    lv_obj_set_style_text_font(lbl_cfg_calibrate, &lv_font_montserrat_20, 0);
    lv_obj_add_event_cb(btn_cfg_calibrate, on_cfg_calibrate, LV_EVENT_CLICKED,
                        nullptr);
    // Calibration status line (right of the button)
    lbl_cfg_cal_status = lv_label_create(cont);
    lv_label_set_text(lbl_cfg_cal_status, "Short the leads, then press");
    lv_obj_set_style_text_color(lbl_cfg_cal_status, C_GREY, 0);
    lv_obj_set_style_text_font(lbl_cfg_cal_status, &lv_font_montserrat_14, 0);
    lv_label_set_long_mode(lbl_cfg_cal_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_cfg_cal_status, 380);
    lv_obj_set_pos(lbl_cfg_cal_status, LABEL_X + 332, y + 8);
    y += 56 + 16;

    // ============================================================
    // ---- Section: TRIGGER SETTINGS ----
    // ============================================================
    make_section_header(cont, "Trigger Settings", LABEL_X, y);
    y += 32;

    // Contact Delay  [-]  value  [+]
    {
        lv_obj_t* lbl = lv_label_create(cont);
        lv_label_set_text(lbl, "Contact Delay");
        lv_obj_set_style_text_color(lbl, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(lbl, LABEL_X, y + 12);
    }
    {
        lv_obj_t* lbl_dec = nullptr;
        btn_cfg_hold_time_dec =
            make_cfg_button(cont, "-", &lbl_dec, STEP_DEC_X, y, STEP_W, BTN_H,
                            C_DARK_GREY);
        lv_obj_set_style_text_font(lbl_dec, &lv_font_montserrat_24, 0);
        lv_obj_add_event_cb(btn_cfg_hold_time_dec, on_cfg_hold_time_dec,
                            LV_EVENT_CLICKED, nullptr);

        lbl_cfg_hold_time = lv_label_create(cont);
        lv_label_set_text(lbl_cfg_hold_time,
                          hold_time_text(_cfg.contact_hold_steps));
        lv_obj_set_style_text_color(lbl_cfg_hold_time, C_ACCENT, 0);
        lv_obj_set_style_text_font(lbl_cfg_hold_time, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_align(lbl_cfg_hold_time, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_width(lbl_cfg_hold_time, STEP_VAL_W);
        lv_obj_set_pos(lbl_cfg_hold_time, STEP_VAL_X, y + 14);

        lv_obj_t* lbl_inc = nullptr;
        btn_cfg_hold_time_inc =
            make_cfg_button(cont, "+", &lbl_inc, STEP_INC_X, y, STEP_W, BTN_H,
                            C_DARK_GREY);
        lv_obj_set_style_text_font(lbl_inc, &lv_font_montserrat_24, 0);
        lv_obj_add_event_cb(btn_cfg_hold_time_inc, on_cfg_hold_time_inc,
                            LV_EVENT_CLICKED, nullptr);
    }
    y += ROW_H;

    // Contact With Pedal ON/OFF
    {
        lv_obj_t* lbl = lv_label_create(cont);
        lv_label_set_text(lbl, "Contact With Pedal");
        lv_obj_set_style_text_color(lbl, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(lbl, LABEL_X, y + 12);
    }
    btn_cfg_cwp = make_cfg_button(
        cont, _cfg.contact_with_pedal ? "ON" : "OFF", &lbl_cfg_cwp, BTN_X, y,
        BTN_W, BTN_H, _cfg.contact_with_pedal ? C_GREEN : C_DARK_GREY);
    lv_obj_add_event_cb(btn_cfg_cwp, on_cfg_cwp_toggle, LV_EVENT_CLICKED,
                        nullptr);
    y += ROW_H + 12;

    // ============================================================
    // ---- Section: DISPLAY & INPUT ----
    // ============================================================
    make_section_header(cont, "Display & Input", LABEL_X, y);
    y += 32;

    // Screen Brightness
    {
        lv_obj_t* lbl = lv_label_create(cont);
        lv_label_set_text(lbl, "Screen Brightness");
        lv_obj_set_style_text_color(lbl, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(lbl, LABEL_X, y + 12);
    }
    btn_cfg_brightness =
        make_cfg_button(cont, brightness_text(_cfg.brightness),
                        &lbl_cfg_brightness, BTN_X, y, BTN_W, BTN_H, C_ACCENT);
    lv_obj_add_event_cb(btn_cfg_brightness, on_cfg_brightness, LV_EVENT_CLICKED,
                        nullptr);
    y += ROW_H;

    // Time Step
    {
        lv_obj_t* lbl = lv_label_create(cont);
        lv_label_set_text(lbl, "Time Step (timing +/-)");
        lv_obj_set_style_text_color(lbl, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(lbl, LABEL_X, y + 12);
    }
    btn_cfg_time_step =
        make_cfg_button(cont, time_step_text(_cfg.time_step_ms),
                        &lbl_cfg_time_step, BTN_X, y, BTN_W, BTN_H, C_ACCENT);
    lv_obj_add_event_cb(btn_cfg_time_step, on_cfg_time_step, LV_EVENT_CLICKED,
                        nullptr);
    y += ROW_H;

    // Power Step
    {
        lv_obj_t* lbl = lv_label_create(cont);
        lv_label_set_text(lbl, "Power Step (power +/-)");
        lv_obj_set_style_text_color(lbl, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(lbl, LABEL_X, y + 12);
    }
    btn_cfg_power_step =
        make_cfg_button(cont, power_step_text(_cfg.power_step_pct),
                        &lbl_cfg_power_step, BTN_X, y, BTN_W, BTN_H, C_ACCENT);
    lv_obj_add_event_cb(btn_cfg_power_step, on_cfg_power_step, LV_EVENT_CLICKED,
                        nullptr);
    y += ROW_H;

    // Hold-to-repeat
    {
        lv_obj_t* lbl = lv_label_create(cont);
        lv_label_set_text(lbl, "Hold-to-repeat (+/- buttons)");
        lv_obj_set_style_text_color(lbl, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(lbl, LABEL_X, y + 12);
    }
    btn_cfg_hold_repeat = make_cfg_button(
        cont, _cfg.hold_to_repeat ? "ON" : "OFF", &lbl_cfg_hold_repeat, BTN_X,
        y, BTN_W, BTN_H, _cfg.hold_to_repeat ? C_GREEN : C_DARK_GREY);
    lv_obj_add_event_cb(btn_cfg_hold_repeat, on_cfg_hold_repeat,
                        LV_EVENT_CLICKED, nullptr);
    y += ROW_H + 12;

    // ============================================================
    // ---- Section: STARTUP ----
    // ============================================================
    make_section_header(cont, "Startup", LABEL_X, y);
    y += 32;

    // Load Last Settings on Boot
    {
        lv_obj_t* lbl = lv_label_create(cont);
        lv_label_set_text(lbl, "Load last settings on boot");
        lv_obj_set_style_text_color(lbl, C_WHITE, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(lbl, LABEL_X, y + 12);
    }
    btn_cfg_load_last = make_cfg_button(
        cont, _cfg.load_last_on_boot ? "ON" : "OFF", &lbl_cfg_load_last, BTN_X,
        y, BTN_W, BTN_H, _cfg.load_last_on_boot ? C_GREEN : C_DARK_GREY);
    lv_obj_add_event_cb(btn_cfg_load_last, on_cfg_load_last, LV_EVENT_CLICKED,
                        nullptr);
    y += ROW_H + 16;

    // ============================================================
    // ---- Firmware info footer ----
    // ============================================================
    {
        lv_obj_t* line = lv_obj_create(cont);
        lv_obj_remove_style_all(line);
        lv_obj_set_size(line, 740, 2);
        lv_obj_set_pos(line, LABEL_X, y);
        lv_obj_set_style_bg_color(line, C_DARK_GREY, 0);
        lv_obj_set_style_bg_opa(line, LV_OPA_COVER, 0);
    }
    y += 14;
    {
        lv_obj_t* lbl = lv_label_create(cont);
        lv_label_set_text(lbl,
                          "Spot Welder UI  \xE2\x80\xA2  ESP32 + STM32G474  "
                          "\xE2\x80\xA2  LVGL 9.1");
        lv_obj_set_style_text_color(lbl, C_GREY, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(lbl, LABEL_X, y);
    }
    y += 24;

    update_cfg_cal_labels();

    // Set container height to fit all content
    lv_obj_set_height(cont, y);
}

// ============================================================
// PUBLIC: ui_init
// ============================================================
void ui_init(arm_toggle_cb_t on_arm_toggle, recipe_apply_cb_t on_recipe_apply) {
    _arm_cb = on_arm_toggle;
    _recipe_cb = on_recipe_apply;

    // Initialize config to defaults (may be overridden by ui_load_config later)
    _cfg = config_defaults();

    lv_obj_t* scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, C_BG, LV_PART_MAIN);

    // Tabview – 5 tabs, tab bar at top, 42 px tall (slightly larger for easier
    // touch)
    lv_obj_t* tv = lv_tabview_create(scr);
    lv_tabview_set_tab_bar_position(tv, LV_DIR_TOP);
    lv_tabview_set_tab_bar_size(tv, 42);
    lv_obj_set_size(tv, 800, 480);
    lv_obj_align(tv, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_bg_color(tv, C_BG, LV_PART_MAIN);

    // ===== CRITICAL: Disable tabview swipe/drag page switching =====
    // This prevents touch gestures from triggering tab switches which
    // cause full-screen redraws and contribute to shudder.
    lv_obj_t* tv_content = lv_tabview_get_content(tv);
    if (tv_content) {
        lv_obj_clear_flag(tv_content, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(tv_content, LV_OBJ_FLAG_SCROLL_ELASTIC);
        lv_obj_clear_flag(tv_content, LV_OBJ_FLAG_GESTURE_BUBBLE);
        lv_obj_set_scrollbar_mode(tv_content, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scroll_dir(tv_content, LV_DIR_NONE);
    }
    lv_obj_clear_flag(tv, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(tv, LV_SCROLLBAR_MODE_OFF);

    // Style the tab buttons (slightly larger font for easier touch)
    lv_obj_t* tab_btns = lv_tabview_get_tab_bar(tv);
    lv_obj_set_style_bg_color(tab_btns, C_DARK_GREY, LV_PART_MAIN);
    lv_obj_set_style_text_color(tab_btns, C_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(tab_btns, &lv_font_montserrat_14, LV_PART_MAIN);

    tab_status = lv_tabview_add_tab(tv, LV_SYMBOL_HOME " Status");
    tab_pulse = lv_tabview_add_tab(tv, LV_SYMBOL_CHARGE " Pulse");
    tab_joule = lv_tabview_add_tab(tv, LV_SYMBOL_BATTERY_FULL " Joule");
    tab_config = lv_tabview_add_tab(tv, LV_SYMBOL_SETTINGS " Config");
    tab_logs = lv_tabview_add_tab(tv, LV_SYMBOL_WIFI " Setup");

    // Build all tab contents now that tab objects exist
    build_status_tab(tab_status);
    build_pulse_tab(tab_pulse);
    build_joule_tab(tab_joule);
    build_config_tab(tab_config);
    build_setup_tab(tab_logs);

    // Apply initial step sizes to spinboxes
    apply_time_step_to_spinboxes();

    // Post-build: disable scrolling/gesture on tab containers
    lv_obj_clear_flag(tab_status, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
    lv_obj_clear_flag(tab_status, LV_OBJ_FLAG_GESTURE_BUBBLE);

    lv_obj_clear_flag(tab_pulse, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(tab_pulse, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
    lv_obj_clear_flag(tab_pulse, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_set_scrollbar_mode(tab_pulse, LV_SCROLLBAR_MODE_OFF);

    lv_obj_clear_flag(tab_joule, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(tab_joule, LV_OBJ_FLAG_GESTURE_BUBBLE);

    // Config tab keeps vertical scrolling but is tuned for a smooth feel:
    //  - GESTURE_BUBBLE off  : touch gestures don't bubble up to the tabview.
    //  - SCROLL_CHAIN off    : no scroll chaining to parent (the inner
    //                          container is now non-scrollable, so the only
    //                          scroller is this tab – keep it self-contained).
    //  - SCROLL_MOMENTUM on  : flick/throw scrolling continues with inertia
    //                          instead of stopping dead, which is what made
    //                          the page feel slow and sticky.
    //  - SCROLL_ELASTIC off  : remove the rubber-band drag-back that adds a
    //                          laggy, fighting sensation on a resistive panel.
    lv_obj_clear_flag(tab_config, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_clear_flag(tab_config, LV_OBJ_FLAG_SCROLL_CHAIN);
    lv_obj_clear_flag(tab_config, LV_OBJ_FLAG_SCROLL_ELASTIC);
    lv_obj_add_flag(tab_config, LV_OBJ_FLAG_SCROLL_MOMENTUM);

    lv_obj_clear_flag(tab_logs, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(tab_logs, LV_OBJ_FLAG_GESTURE_BUBBLE);
}

// ====
// PUBLIC: ui_mark_settings_applied – Force draft = applied, clear dirty
// Called from main.cpp after boot config is sent and confirmed by STM32.
// ====
void ui_mark_settings_applied() {
    // Copy applied -> draft
    draft_mode = applied_mode;
    draft_d1 = applied_d1;
    draft_gap1 = applied_gap1;
    draft_d2 = applied_d2;
    draft_gap2 = applied_gap2;
    draft_d3 = applied_d3;
    draft_power = applied_power;
    draft_preheat_en = applied_preheat_en;
    draft_preheat_ms = applied_preheat_ms;
    draft_preheat_pct = applied_preheat_pct;
    draft_preheat_gap = applied_preheat_gap;

    // Update spinbox widgets
    if (spin_d1) lv_spinbox_set_value(spin_d1, (int32_t)draft_d1);
    if (spin_gap1) lv_spinbox_set_value(spin_gap1, (int32_t)draft_gap1);
    if (spin_d2) lv_spinbox_set_value(spin_d2, (int32_t)draft_d2);
    if (spin_gap2) lv_spinbox_set_value(spin_gap2, (int32_t)draft_gap2);
    if (spin_d3) lv_spinbox_set_value(spin_d3, (int32_t)draft_d3);
    if (spin_ph_ms) lv_spinbox_set_value(spin_ph_ms, (int32_t)draft_preheat_ms);
    if (spin_ph_pct)
        lv_spinbox_set_value(spin_ph_pct, (int32_t)draft_preheat_pct);
    if (spin_ph_gap)
        lv_spinbox_set_value(spin_ph_gap, (int32_t)draft_preheat_gap);

    // Clear dirty and trigger UI refresh
    draft_dirty = false;
    _ui_dirty = true;

    Serial.println("[UI] Settings marked as applied (draft synced)");
}

// ============================================================
// PUBLIC: ui_has_pending_changes
// ============================================================
bool ui_has_pending_changes() { return draft_dirty; }

// ============================================================
// PUBLIC: ui_update – Touch-aware with anti-shudder gating
// ============================================================
void ui_update(const WelderDisplayState& st) {
    static uint32_t last_ms = 0;
    uint32_t now = millis();

    // ---- ANTI-SHUDDER: Block ALL visual updates during active touch ----
    // This is the core fix: when the user is touching the screen, LVGL is
    // processing input events and any label/style changes trigger redraws
    // that compete with touch rendering, causing visible shudder.
    bool any_touch = _hw_touch_active || _widget_touch_active;
    if (any_touch) {
        return;  // Completely skip – no redraws during touch
    }

    // ---- Post-touch cooldown: reduced rate briefly after release ----
    // After releasing touch, LVGL may still be settling animations/states.
    // Use a reduced refresh rate for 300ms to let it finish cleanly.
    uint32_t interval = 100;  // 10 Hz default
    if (now - _touch_release_ms < 300) {
        interval = 250;  // 4 Hz briefly after release
    }
    if (now - last_ms < interval) return;
    last_ms = now;

    // ---- Sync applied recipe from authoritative source ----
    sync_applied_from_state(st);

    // ---- Lock/unlock Pulse tab based on armed state ----
    // Removed: allow recipe editing while armed (draft/apply model still
    // enforced) lock_pulse_tab(st.armed);

    // ---- Change-detection state (static across calls) ----
    static float prev_pack_v = -999.0f;
    static float prev_temp = -999.0f;
    static bool prev_temp_fin = false;
    static float prev_ichg = -999.0f;
    static float prev_cell1 = -999.0f;
    static float prev_cell2 = -999.0f;
    static float prev_cell3 = -999.0f;
    static bool prev_armed = false;
    static bool prev_welding = false;
    static bool prev_charging = false;
    static bool first_run = true;

    char buf[32];

    // ---- Pack Voltage ----
    if (lbl_pack_v &&
        (first_run || fabsf(st.pack_voltage - prev_pack_v) >= 0.01f)) {
        snprintf(buf, sizeof(buf), "%.2f V", (double)st.pack_voltage);
        lv_label_set_text(lbl_pack_v, buf);
        lv_color_t c = (st.pack_voltage < 8.0f)   ? C_RED
                       : (st.pack_voltage > 9.0f) ? C_YELLOW
                                                  : C_GREEN;
        lv_obj_set_style_text_color(lbl_pack_v, c, LV_PART_MAIN);
        prev_pack_v = st.pack_voltage;
    }

    // ---- Temperature ----
    {
        bool fin = isfinite(st.temperature);
        if (lbl_temp && (first_run || fin != prev_temp_fin ||
                         (fin && fabsf(st.temperature - prev_temp) >= 0.05f))) {
            if (fin) {
                snprintf(buf, sizeof(buf),
                         "%.1f \xC2\xB0"
                         "C",
                         (double)st.temperature);
                lv_label_set_text(lbl_temp, buf);
                lv_color_t c = (st.temperature > 50.0f) ? C_RED : C_GREEN;
                lv_obj_set_style_text_color(lbl_temp, c, LV_PART_MAIN);
            } else {
                lv_label_set_text(lbl_temp, "ERR");
                lv_obj_set_style_text_color(lbl_temp, C_RED, LV_PART_MAIN);
            }
            prev_temp = st.temperature;
            prev_temp_fin = fin;
        }
    }

    // ---- Charger Current ----
    if (lbl_ichg &&
        (first_run || fabsf(st.charger_current - prev_ichg) >= 0.01f)) {
        snprintf(buf, sizeof(buf), "%.2f A", (double)st.charger_current);
        lv_label_set_text(lbl_ichg, buf);
        prev_ichg = st.charger_current;
    }

    // ---- Cell Voltages ----
    if (lbl_cell1 && (first_run || fabsf(st.cell1_v - prev_cell1) >= 0.001f)) {
        snprintf(buf, sizeof(buf), "C1: %.3f V", (double)st.cell1_v);
        lv_label_set_text(lbl_cell1, buf);
        prev_cell1 = st.cell1_v;
    }
    if (lbl_cell2 && (first_run || fabsf(st.cell2_v - prev_cell2) >= 0.001f)) {
        snprintf(buf, sizeof(buf), "C2: %.3f V", (double)st.cell2_v);
        lv_label_set_text(lbl_cell2, buf);
        prev_cell2 = st.cell2_v;
    }
    if (lbl_cell3 && (first_run || fabsf(st.cell3_v - prev_cell3) >= 0.001f)) {
        snprintf(buf, sizeof(buf), "C3: %.3f V", (double)st.cell3_v);
        lv_label_set_text(lbl_cell3, buf);
        prev_cell3 = st.cell3_v;
    }

    // ---- Dashboard: Mode line ----
    // The MODE box is now a long-press switcher (TIME<->JOULE). Keep the
    // locally-tracked _joule_draft_mode mirrored to the STM32's authoritative
    // control_mode so the switcher and the Joule tab always reflect reality.
    // Skip while a long-press toggle is in flight so we don't fight the user.
    {
        static uint8_t prev_mode = 0xFF;
        if (!_mode_longpress_fired &&
            (first_run || st.control_mode != prev_mode)) {
            _joule_draft_mode = (st.control_mode == 1);
            paint_dash_mode(false);
            prev_mode = st.control_mode;
        }
    }

    // ---- Dashboard: Lead resistance + Config calibration labels ----
    {
        // main.cpp latches lead_r so it only changes on boot / calibration /
        // SET_LEAD_R (never from periodic STATUS), so this change-detector now
        // effectively fires only on those events. Ignore non-positive values
        // so a transient 0 can never paint a "0.0m" reading.
        static float prev_leadr = -999.0f;
        if (st.lead_resistance_mohm > 0.0001f &&
            (first_run || fabsf(st.lead_resistance_mohm - prev_leadr) >= 0.005f)) {
            if (lbl_dash_leadr) {
                snprintf(buf, sizeof(buf), "%.1fm",
                         (double)st.lead_resistance_mohm);
                lv_label_set_text(lbl_dash_leadr, buf);
            }
            // keep config draft + read-only label in sync
            _cfg.lead_resistance_mohm = st.lead_resistance_mohm;
            update_cfg_cal_labels();
            prev_leadr = st.lead_resistance_mohm;
        }
    }

    // ---- Dashboard: Calibration age (Config tab) ----
    {
        static int32_t prev_cal_age = -2;
        int32_t age = st.cal_valid ? (int32_t)st.cal_age_sec : -1;
        if (lbl_cfg_cal_age && (first_run || age != prev_cal_age)) {
            if (!st.cal_valid) {
                lv_label_set_text(lbl_cfg_cal_age, "\xE2\x80\x94 (not this session)");
                lv_obj_set_style_text_color(lbl_cfg_cal_age, C_GREY,
                                            LV_PART_MAIN);
            } else {
                uint32_t s = st.cal_age_sec;
                if (s < 60)
                    snprintf(buf, sizeof(buf), "%lus ago", (unsigned long)s);
                else if (s < 3600)
                    snprintf(buf, sizeof(buf), "%lum ago",
                             (unsigned long)(s / 60));
                else
                    snprintf(buf, sizeof(buf), "%luh ago",
                             (unsigned long)(s / 3600));
                lv_label_set_text(lbl_cfg_cal_age, buf);
                lv_obj_set_style_text_color(lbl_cfg_cal_age, C_GREEN,
                                            LV_PART_MAIN);
            }
            prev_cal_age = age;
        }
    }

    // ---- Last Weld results: Duration | Peak | Avg | Joules ----
    {
        static bool  prev_lw_valid = false;
        static float prev_lw_energy = -999.0f;
        static float prev_lw_peak = -999.0f;
        static float prev_lw_avg = -999.0f;
        static uint32_t prev_lw_dur = 0xFFFFFFFF;
        bool changed = first_run || st.last_weld_valid != prev_lw_valid ||
                       fabsf(st.last_weld_energy_j - prev_lw_energy) >= 0.05f ||
                       fabsf(st.last_weld_peak_a - prev_lw_peak) >= 1.0f ||
                       fabsf(st.last_weld_avg_a - prev_lw_avg) >= 1.0f ||
                       st.last_weld_duration_ms != prev_lw_dur;
        if (changed) {
            if (!st.last_weld_valid) {
                if (lbl_lw_duration) lv_label_set_text(lbl_lw_duration, "--");
                if (lbl_lw_peak) lv_label_set_text(lbl_lw_peak, "--");
                if (lbl_lw_avg) lv_label_set_text(lbl_lw_avg, "--");
                if (lbl_lw_joules) lv_label_set_text(lbl_lw_joules, "--");
            } else {
                if (lbl_lw_duration) {
                    snprintf(buf, sizeof(buf), "%u ms",
                             (unsigned)st.last_weld_duration_ms);
                    lv_label_set_text(lbl_lw_duration, buf);
                }
                if (lbl_lw_peak) {
                    snprintf(buf, sizeof(buf), "%.0f A",
                             (double)st.last_weld_peak_a);
                    lv_label_set_text(lbl_lw_peak, buf);
                }
                if (lbl_lw_avg) {
                    snprintf(buf, sizeof(buf), "%.0f A",
                             (double)st.last_weld_avg_a);
                    lv_label_set_text(lbl_lw_avg, buf);
                }
                // Joules: measured workpiece energy delivered (moved from Joule tab).
                if (lbl_lw_joules) {
                    snprintf(buf, sizeof(buf), "%.1f J",
                             (double)st.last_weld_energy_j);
                    lv_label_set_text(lbl_lw_joules, buf);
                }
            }
            prev_lw_valid = st.last_weld_valid;
            prev_lw_energy = st.last_weld_energy_j;
            prev_lw_peak = st.last_weld_peak_a;
            prev_lw_avg = st.last_weld_avg_a;
            prev_lw_dur = st.last_weld_duration_ms;
        }
    }


    // ---- Weld Counter ----
    {
        static uint32_t prev_weld_count = 0xFFFFFFFF;
        if (lbl_weld_cnt && (first_run || st.weld_count != prev_weld_count)) {
            char wbuf[16];
            snprintf(wbuf, sizeof(wbuf), "%lu", (unsigned long)st.weld_count);
            lv_label_set_text(lbl_weld_cnt, wbuf);
            prev_weld_count = st.weld_count;
        }
    }

    // ---- Dashboard trigger line sync from state ----
    // Don't stomp the value while the user is holding a long-press swap.
    {
        static uint8_t prev_trigger = 0xFF;
        static uint8_t prev_contact_steps = 0xFF;
        bool changed = first_run || st.trigger_mode != prev_trigger ||
                       st.contact_hold_steps != prev_contact_steps;
        if (changed && !_trig_longpress_fired) {
            _trigger_mode = st.trigger_mode;
            _dash_contact_steps = st.contact_hold_steps;
            paint_dash_trigger(false);
            prev_trigger = st.trigger_mode;
            prev_contact_steps = st.contact_hold_steps;
        }
    }

    // ---- ARM button ----
    _last_armed = st.armed;
    bool arm_changed = (first_run || st.armed != prev_armed);
    if (arm_changed) {
        paint_arm_button(st.armed);
        prev_armed = st.armed;
    }

    // ---- State label (hidden but still tracked for internal state) ----
    if (arm_changed || st.welding != prev_welding ||
        st.charging != prev_charging) {
        if (lbl_state) {
            const char* txt;
            lv_color_t col;
            if (!st.armed) {
                txt = "DISARMED";
                col = C_RED;
            } else if (st.welding) {
                txt = "WELDING";
                col = C_YELLOW;
            } else if (st.charging) {
                txt = "CHARGING";
                col = C_ACCENT;
            } else {
                txt = "IDLE";
                col = C_GREEN;
            }
            lv_label_set_text(lbl_state, txt);
            lv_obj_set_style_text_color(lbl_state, col, LV_PART_MAIN);
            // NOTE: the old dashboard "HEALTH" column status line (lbl_health)
            // was removed when ROW 2 was reorganised into 4 columns
            // (PACK/CELLS/TEMPERATURE/CHARGING).  ARM/WELDING/CHARGING state is
            // still shown by the big ARM button + System Info box above.
        }
        prev_welding = st.welding;
        prev_charging = st.charging;
    }

    // ---- Joule tab: seed draft from applied state once (don't stomp edits) ----
    if (!_joule_draft_init) {
        _joule_draft_mode = (st.control_mode == 1);
        if (st.joule_target_j > 0.0f) _joule_draft_target = st.joule_target_j;
        if (st.joule_max_ms > 0) _joule_draft_maxms = st.joule_max_ms;
        _joule_draft_init = true;
        paint_joule_tab();
    }

    // ============================================================
    // PULSE TAB: All visual updates driven by _ui_dirty or applied changes
    // ============================================================
    {
        static bool prev_dirty = false;
        static uint8_t prev_pulse_mode = 0xFF;
        static uint8_t prev_pulse_power = 0xFF;
        static bool prev_preheat_en = false;

        update_draft_dirty();

        bool need_refresh =
            _ui_dirty || (draft_dirty != prev_dirty) || first_run;

        if (need_refresh) {
            // Mode buttons: repaint selection
            for (int i = 0; i < 3; i++) {
                if (!btn_mode[i]) continue;
                bool sel = ((int)draft_mode == i + 1);
                lv_obj_set_style_bg_color(btn_mode[i],
                                          sel ? C_ACCENT : C_DARK_GREY, 0);
            }

            // Field visibility based on mode
            if (draft_mode != prev_pulse_mode) {
                update_pulse_field_visibility();
                prev_pulse_mode = draft_mode;
            }

            // Sync spinbox widgets to current draft values
            // (guards prevent unnecessary redraws – anti-flicker safe)
            if (spin_d1 && lv_spinbox_get_value(spin_d1) != (int32_t)draft_d1)
                lv_spinbox_set_value(spin_d1, (int32_t)draft_d1);
            if (spin_gap1 &&
                lv_spinbox_get_value(spin_gap1) != (int32_t)draft_gap1)
                lv_spinbox_set_value(spin_gap1, (int32_t)draft_gap1);
            if (spin_d2 && lv_spinbox_get_value(spin_d2) != (int32_t)draft_d2)
                lv_spinbox_set_value(spin_d2, (int32_t)draft_d2);
            if (spin_gap2 &&
                lv_spinbox_get_value(spin_gap2) != (int32_t)draft_gap2)
                lv_spinbox_set_value(spin_gap2, (int32_t)draft_gap2);
            if (spin_d3 && lv_spinbox_get_value(spin_d3) != (int32_t)draft_d3)
                lv_spinbox_set_value(spin_d3, (int32_t)draft_d3);
            if (spin_ph_ms &&
                lv_spinbox_get_value(spin_ph_ms) != (int32_t)draft_preheat_ms)
                lv_spinbox_set_value(spin_ph_ms, (int32_t)draft_preheat_ms);
            if (spin_ph_pct &&
                lv_spinbox_get_value(spin_ph_pct) != (int32_t)draft_preheat_pct)
                lv_spinbox_set_value(spin_ph_pct, (int32_t)draft_preheat_pct);
            if (spin_ph_gap &&
                lv_spinbox_get_value(spin_ph_gap) != (int32_t)draft_preheat_gap)
                lv_spinbox_set_value(spin_ph_gap, (int32_t)draft_preheat_gap);

            // Power value label
            if (lbl_power_val && draft_power != prev_pulse_power) {
                char pbuf[16];
                snprintf(pbuf, sizeof(pbuf), "%d %%", (int)draft_power);
                lv_label_set_text(lbl_power_val, pbuf);
                prev_pulse_power = draft_power;
            }

            // Preheat toggle visual
            if (btn_preheat) {
                lv_obj_set_style_bg_color(
                    btn_preheat, draft_preheat_en ? C_GREEN : C_DARK_GREY, 0);
            }
            if (lbl_preheat) {
                lv_label_set_text(lbl_preheat, draft_preheat_en ? "ON" : "OFF");
            }
            if (draft_preheat_en != prev_preheat_en) {
                update_preheat_visibility();
                prev_preheat_en = draft_preheat_en;
            }

            // APPLY button color: green=synced, red=pending
            if (btn_apply) {
                lv_obj_set_style_bg_color(btn_apply,
                                          draft_dirty ? C_RED : C_GREEN, 0);
            }

            // Pending label
            if (lbl_pending) {
                if (draft_dirty) {
                    lv_label_set_text(lbl_pending,
                                      LV_SYMBOL_WARNING " Changes pending");
                    lv_obj_clear_flag(lbl_pending, LV_OBJ_FLAG_HIDDEN);
                } else {
                    lv_obj_add_flag(lbl_pending, LV_OBJ_FLAG_HIDDEN);
                }
            }

            // Update ARM button text if dirty state changed
            if (draft_dirty != prev_dirty) {
                paint_arm_button(_last_armed);
            }

            prev_dirty = draft_dirty;
            _ui_dirty = false;
        }
    }

    first_run = false;
}

// ============================================================
// PUBLIC: Config API
// ============================================================
void ui_set_config_cb(config_change_cb_t cb) { _config_cb = cb; }

void ui_load_config(const ConfigState& cfg) {
    _cfg = cfg;
    // Update all Config tab labels
    update_cfg_hold_repeat_label();
    update_cfg_time_step_label();
    update_cfg_power_step_label();
    update_cfg_load_last_label();
    update_cfg_brightness_label();
    update_cfg_cwp_label();
    update_cfg_hold_time_label();
    update_cfg_cal_labels();
    // Apply step sizes to spinboxes
    apply_time_step_to_spinboxes();
}

ConfigState ui_get_config() { return _cfg; }

void ui_set_trigger_source_cb(trigger_source_cb_t cb) { _trigger_cb = cb; }

void ui_set_weld_count_reset_cb(weld_count_reset_cb_t cb) {
    _weld_reset_cb = cb;
}

void ui_set_contact_with_pedal_cb(contact_with_pedal_cb_t cb) { _cwp_cb = cb; }

void ui_set_calibrate_cb(calibrate_cb_t cb) { _calibrate_cb = cb; }

// ----------------------------------------------------------------------------
// Calibration progress feedback (CONFIG tab status line)
//
// main.cpp's UART poll forwards every STM32 "CAL_*" line here so the touch UI
// stops being stuck on "Calibrating... keep leads shorted".  The STM32
// protocol (see performLeadCalibration() in STM32G474CE/src/main.c) emits:
//
//   CAL_STATUS=WAITING_FOR_TRIGGER        -> waiting for pedal press / contact
//   CAL_STATUS=CONTACT_DETECTED_HOLDING   -> tips touching, hold to fire
//   CAL_STATUS=TRIGGER_DETECTED           -> trigger seen
//   CAL_STATUS=MEASURING                  -> firing test pulse
//   CAL_RESULT=<ohms>                     -> SUCCESS (terminal)
//   CAL_ERROR=<reason>[,...]              -> FAILURE (terminal)
//
// The key UX fix: WAITING_FOR_TRIGGER means the operator must actually fire
// the configured trigger (press the pedal, or hold the tips shorted in
// contact mode) — simply shorting the leads is not enough in pedal mode.  We
// surface that instruction instead of the misleading static "keep leads
// shorted" text, and we always leave the in-progress state on a terminal
// CAL_RESULT / CAL_ERROR (or the STM32's 8 s TIMEOUT_NO_TRIGGER).
void ui_notify_cal_message(const char* line) {
    if (!line || !lbl_cfg_cal_status) return;

    const char* msg = nullptr;
    lv_color_t col = C_GREY;

    if (strncmp(line, "CAL_RESULT=", 11) == 0) {
        // Success: STM32 reports resistance in ohms.  Show it in mΩ.
        float ohms = 0.0f;
        char buf[48];
        if (sscanf(line + 11, "%f", &ohms) == 1) {
            snprintf(buf, sizeof(buf), "Done: %.1fm \xE2\x9C\x93",
                     (double)(ohms * 1000.0f));
        } else {
            snprintf(buf, sizeof(buf), "Calibration complete \xE2\x9C\x93");
        }
        lv_label_set_text(lbl_cfg_cal_status, buf);
        lv_obj_set_style_text_color(lbl_cfg_cal_status, C_GREEN, LV_PART_MAIN);
        return;
    }

    if (strncmp(line, "CAL_ERROR=", 10) == 0) {
        const char* reason = line + 10;
        char buf[64];
        // Friendly text for the common terminal errors.
        if (strncmp(reason, "TIMEOUT_NO_TRIGGER", 18) == 0) {
            msg = "Timed out: no trigger. Press pedal / short tips, retry.";
        } else if (strncmp(reason, "TIPS_NOT_SHORTED", 16) == 0) {
            msg = "Tips not shorted - clamp leads together, then retry.";
        } else if (strncmp(reason, "VOLTAGE_LOW", 11) == 0) {
            msg = "Pack voltage low - charge, then retry.";
        } else if (strncmp(reason, "NOT_READY", 9) == 0) {
            msg = "Not ready - wait for READY, then retry.";
        } else if (strncmp(reason, "BUSY", 4) == 0) {
            msg = "Busy - finish current action, then retry.";
        } else if (strncmp(reason, "OVERHEAT", 8) == 0 ||
                   strncmp(reason, "COOLDOWN", 8) == 0) {
            msg = "Too hot - let it cool, then retry.";
        } else {
            snprintf(buf, sizeof(buf), "Calibration failed: %s", reason);
            msg = buf;
        }
        lv_label_set_text(lbl_cfg_cal_status, msg);
        lv_obj_set_style_text_color(lbl_cfg_cal_status, C_RED, LV_PART_MAIN);
        return;
    }

    if (strncmp(line, "CAL_STATUS=", 11) == 0) {
        const char* state = line + 11;
        col = C_ACCENT;
        if (strncmp(state, "WAITING_FOR_TRIGGER", 19) == 0) {
            msg = "Now fire the trigger: press pedal (or hold tips shorted)";
            col = C_YELLOW;
        } else if (strncmp(state, "CONTACT_DETECTED_HOLDING", 24) == 0) {
            msg = "Contact detected - keep tips held...";
            col = C_YELLOW;
        } else if (strncmp(state, "TRIGGER_DETECTED", 16) == 0) {
            msg = "Trigger detected - measuring...";
        } else if (strncmp(state, "MEASURING", 9) == 0) {
            msg = "Measuring lead resistance...";
        } else {
            msg = "Calibrating...";
        }
        lv_label_set_text(lbl_cfg_cal_status, msg);
        lv_obj_set_style_text_color(lbl_cfg_cal_status, col, LV_PART_MAIN);
        return;
    }
}

void ui_set_joule_apply_cb(joule_apply_cb_t cb) { _joule_apply_cb = cb; }

void ui_set_contact_delay_cb(contact_delay_cb_t cb) { _contact_delay_cb = cb; }
// ============================================================
// SETUP TAB: callback registration
// ============================================================
void ui_set_wifi_reconfigure_cb(wifi_reconfigure_cb_t cb) {
    _wifi_reconfigure_cb = cb;
}

void ui_set_restart_cb(device_restart_cb_t cb) { _restart_cb = cb; }

void ui_set_factory_reset_cb(factory_reset_cb_t cb) { _factory_reset_cb = cb; }

// ============================================================
// SETUP TAB: live WiFi status
// ============================================================
void ui_set_wifi_info(bool connected, bool ap_mode, const char* ssid,
                      const char* ip, int rssi) {
    // --- Setup tab "Connection" panel ---
    if (lbl_setup_wifi_state) {
        if (ap_mode) {
            lv_label_set_text(lbl_setup_wifi_state, "Setup mode (AP)");
            lv_obj_set_style_text_color(lbl_setup_wifi_state, C_YELLOW, 0);
        } else if (connected) {
            lv_label_set_text(lbl_setup_wifi_state, "Connected");
            lv_obj_set_style_text_color(lbl_setup_wifi_state, C_GREEN, 0);
        } else {
            lv_label_set_text(lbl_setup_wifi_state, "Disconnected");
            lv_obj_set_style_text_color(lbl_setup_wifi_state, C_RED, 0);
        }
    }
    if (lbl_setup_wifi_ssid) {
        lv_label_set_text(lbl_setup_wifi_ssid,
                          (ssid && ssid[0]) ? ssid : "--");
    }
    if (lbl_setup_wifi_ip) {
        lv_label_set_text(lbl_setup_wifi_ip, (ip && ip[0]) ? ip : "--");
    }
    if (lbl_setup_wifi_rssi) {
        if (connected && !ap_mode) {
            char buf[24];
            snprintf(buf, sizeof(buf), "%d dBm", rssi);
            lv_label_set_text(lbl_setup_wifi_rssi, buf);
        } else {
            lv_label_set_text(lbl_setup_wifi_rssi, "--");
        }
    }

    // --- Small indicator on the Status tab header ---
    if (lbl_status_wifi) {
        lv_color_t col = C_GREY;
        if (ap_mode)
            col = C_YELLOW;
        else if (connected)
            col = C_GREEN;
        lv_obj_set_style_text_color(lbl_status_wifi, col, 0);
    }
}

// ============================================================
// SETUP TAB: static system info (set once at boot)
// ============================================================
void ui_set_system_info(const char* fw_version, const char* chip_model,
                        uint32_t flash_size_bytes, uint32_t lifetime_welds) {
    if (lbl_setup_fw && fw_version) lv_label_set_text(lbl_setup_fw, fw_version);
    if (lbl_setup_chip && chip_model)
        lv_label_set_text(lbl_setup_chip, chip_model);
    if (lbl_setup_flash) {
        char buf[24];
        snprintf(buf, sizeof(buf), "%u MB",
                 (unsigned)(flash_size_bytes / (1024U * 1024U)));
        lv_label_set_text(lbl_setup_flash, buf);
    }
    if (lbl_setup_welds) {
        char buf[24];
        snprintf(buf, sizeof(buf), "%u", (unsigned)lifetime_welds);
        lv_label_set_text(lbl_setup_welds, buf);
    }
}

// ============================================================
// WiFi provisioning QR overlay (full screen, top layer)
// ============================================================
static void ensure_prov_overlay() {
    if (prov_overlay) return;

    prov_overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(prov_overlay);
    lv_obj_set_size(prov_overlay, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(prov_overlay, 0, 0);
    lv_obj_set_style_bg_color(prov_overlay, C_BG, 0);
    lv_obj_set_style_bg_opa(prov_overlay, LV_OPA_COVER, 0);
    lv_obj_clear_flag(prov_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(prov_overlay, LV_OBJ_FLAG_HIDDEN);

    // ---- Heading (top, centered) ----
    lv_obj_t* head = lv_label_create(prov_overlay);
    lv_label_set_text(head, LV_SYMBOL_WIFI "  WiFi Setup");
    lv_obj_set_style_text_color(head, C_ACCENT, 0);
    lv_obj_set_style_text_font(head, &lv_font_montserrat_24, 0);
    lv_obj_align(head, LV_ALIGN_TOP_MID, 0, 12);

    // ---- Step 1: connect to the welder's WiFi (left column) ----
    lv_obj_t* step1 = lv_label_create(prov_overlay);
    lv_label_set_text(step1, "1.  Connect your phone to WiFi:");
    lv_obj_set_style_text_color(step1, C_WHITE, 0);
    lv_obj_set_style_text_font(step1, &lv_font_montserrat_18, 0);
    lv_obj_align(step1, LV_ALIGN_TOP_LEFT, 28, 70);

    // SSID shown large + highlighted (this is the network name to join)
    lbl_prov_ssid = lv_label_create(prov_overlay);
    lv_label_set_text(lbl_prov_ssid, "SpotWelder-XXXX");
    lv_obj_set_style_text_color(lbl_prov_ssid, C_GREEN, 0);
    lv_obj_set_style_text_font(lbl_prov_ssid, &lv_font_montserrat_24, 0);
    lv_obj_align(lbl_prov_ssid, LV_ALIGN_TOP_LEFT, 56, 100);

    // ---- Step 2: scan the QR (left column) ----
    lv_obj_t* step2 = lv_label_create(prov_overlay);
    lv_label_set_text(step2, "2.  Scan the QR code to open");
    lv_obj_set_style_text_color(step2, C_WHITE, 0);
    lv_obj_set_style_text_font(step2, &lv_font_montserrat_18, 0);
    lv_obj_align(step2, LV_ALIGN_TOP_LEFT, 28, 156);

    lv_obj_t* step2b = lv_label_create(prov_overlay);
    lv_label_set_text(step2b, "the setup page, then tap \"Open\".");
    lv_obj_set_style_text_color(step2b, C_GREY, 0);
    lv_obj_set_style_text_font(step2b, &lv_font_montserrat_16, 0);
    lv_obj_align(step2b, LV_ALIGN_TOP_LEFT, 56, 184);

    // ---- QR code (right column) ----
    prov_qr = lv_qrcode_create(prov_overlay);
    lv_qrcode_set_size(prov_qr, 190);
    lv_qrcode_set_dark_color(prov_qr, lv_color_hex(0x000000));
    lv_qrcode_set_light_color(prov_qr, lv_color_hex(0xFFFFFF));
    lv_obj_set_style_border_color(prov_qr, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(prov_qr, 8, 0);
    lv_obj_align(prov_qr, LV_ALIGN_TOP_RIGHT, -70, 74);

    lv_obj_t* qr_cap = lv_label_create(prov_overlay);
    lv_label_set_text(qr_cap, "Scan with phone camera");
    lv_obj_set_style_text_color(qr_cap, C_GREY, 0);
    lv_obj_set_style_text_font(qr_cap, &lv_font_montserrat_14, 0);
    lv_obj_align(qr_cap, LV_ALIGN_TOP_RIGHT, -90, 272);

    // ---- Manual fallback: "Or open a browser and go to:" ----
    lv_obj_t* or_lbl = lv_label_create(prov_overlay);
    lv_label_set_text(or_lbl, "Or open a browser and go to:");
    lv_obj_set_style_text_color(or_lbl, C_GREY, 0);
    lv_obj_set_style_text_font(or_lbl, &lv_font_montserrat_16, 0);
    lv_obj_align(or_lbl, LV_ALIGN_BOTTOM_MID, 0, -76);

    // Highlighted box around the IP so it is impossible to miss.
    lv_obj_t* ip_box = lv_obj_create(prov_overlay);
    lv_obj_remove_style_all(ip_box);
    lv_obj_clear_flag(ip_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(ip_box, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(ip_box, 360, 50);
    lv_obj_set_style_bg_color(ip_box, C_DARK_GREY, 0);
    lv_obj_set_style_bg_opa(ip_box, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(ip_box, 10, 0);
    lv_obj_set_style_border_color(ip_box, C_ACCENT, 0);
    lv_obj_set_style_border_width(ip_box, 2, 0);
    lv_obj_align(ip_box, LV_ALIGN_BOTTOM_MID, 0, -18);

    lbl_prov_ip = lv_label_create(ip_box);
    lv_label_set_text(lbl_prov_ip, "http://192.168.4.1");
    lv_obj_set_style_text_color(lbl_prov_ip, C_ACCENT, 0);
    lv_obj_set_style_text_font(lbl_prov_ip, &lv_font_montserrat_24, 0);
    lv_obj_center(lbl_prov_ip);
}

void ui_show_wifi_setup(const char* qr_payload, const char* ap_ssid,
                        const char* portal_ip) {
    ensure_prov_overlay();

    if (prov_qr && qr_payload) {
        lv_qrcode_update(prov_qr, qr_payload, strlen(qr_payload));
    }
    if (lbl_prov_ssid) {
        // Show just the SSID (e.g. "SpotWelder-XXXX") - large/green per new design
        lv_label_set_text(lbl_prov_ssid,
                          (ap_ssid && ap_ssid[0]) ? ap_ssid : "--");
    }
    if (lbl_prov_ip) {
        char buf[64];
        // Show the full URL (e.g. "http://192.168.4.1") inside the highlighted box
        snprintf(buf, sizeof(buf), "http://%s",
                 (portal_ip && portal_ip[0]) ? portal_ip : "192.168.4.1");
        lv_label_set_text(lbl_prov_ip, buf);
    }

    lv_obj_clear_flag(prov_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(prov_overlay);
}

void ui_hide_wifi_setup() {
    if (prov_overlay) lv_obj_add_flag(prov_overlay, LV_OBJ_FLAG_HIDDEN);
}
