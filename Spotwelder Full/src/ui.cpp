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
 *   ARM is blocked when draft ≠ applied.
 *
 * Provides:
 *   Tab 0  "Status"   – Pack V, Temp, Charger I, Cell voltages, Arm toggle
 *   Tab 1  "Pulse"    – Mode, Durations, Power (+/-), Preheat (toggle), Apply
 *   Tab 2  "Charger"  – placeholder
 *   Tab 3  "Config"   – placeholder
 *   Tab 4  "Logs"     – placeholder
 */

#include "ui.h"
#include <Arduino.h>
#include <lvgl.h>
#include <math.h>
#include <stdio.h>   // for standard snprintf (supports %f)

// ============================================================
// COLORS  (dark-theme, matching web UI)
// ============================================================
#define C_BG        lv_color_hex(0x1A1A2E)
#define C_CARD      lv_color_hex(0x222240)
#define C_ACCENT    lv_color_hex(0xFF6600)
#define C_GREEN     lv_color_hex(0x00CC66)
#define C_RED       lv_color_hex(0xFF3333)
#define C_YELLOW    lv_color_hex(0xFFDD44)
#define C_WHITE     lv_color_hex(0xFFFFFF)
#define C_GREY      lv_color_hex(0x888888)
#define C_DARK_GREY lv_color_hex(0x333355)

// ============================================================
// CALLBACKS
// ============================================================
static arm_toggle_cb_t    _arm_cb    = nullptr;
static recipe_apply_cb_t  _recipe_cb = nullptr;

// ============================================================
// ANTI-SHUDDER: Touch state tracking
// ============================================================
/* _hw_touch_active is driven by the hardware touch driver in main.cpp via
 * ui_set_touch_active() — this catches ALL touches, not just widget hits.
 * _widget_touch_active is a secondary flag from widget events (belt-and-suspenders).
 */
static volatile bool _hw_touch_active = false;
static volatile bool _widget_touch_active = false;
static uint32_t      _touch_release_ms = 0;

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
static void on_stop_bubble(lv_event_t* e) {
    lv_event_stop_bubbling(e);
}

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
// STATIC UI HANDLES  (Status tab)
// ============================================================
static lv_obj_t* lbl_pack_v   = nullptr;
static lv_obj_t* lbl_temp     = nullptr;
static lv_obj_t* lbl_ichg     = nullptr;
static lv_obj_t* lbl_cell1    = nullptr;
static lv_obj_t* lbl_cell2    = nullptr;
static lv_obj_t* lbl_cell3    = nullptr;
static lv_obj_t* btn_arm      = nullptr;
static lv_obj_t* lbl_arm      = nullptr;
static lv_obj_t* lbl_state    = nullptr;

static bool _last_armed = false;

// ============================================================
// STATIC UI HANDLES  (Pulse tab – Production)
// ============================================================
// Mode selector: 3 big toggle buttons
static lv_obj_t* btn_mode[3]     = {nullptr, nullptr, nullptr};

// Timing rows: each row is a container holding label, -, spinbox, +, unit
static lv_obj_t* row_d1       = nullptr;
static lv_obj_t* row_gap1     = nullptr;
static lv_obj_t* row_d2       = nullptr;
static lv_obj_t* row_gap2     = nullptr;
static lv_obj_t* row_d3       = nullptr;

// Spinboxes inside the rows (for reading values)
static lv_obj_t* spin_d1       = nullptr;
static lv_obj_t* spin_gap1     = nullptr;
static lv_obj_t* spin_d2       = nullptr;
static lv_obj_t* spin_gap2     = nullptr;
static lv_obj_t* spin_d3       = nullptr;

// Power controls (+/- buttons, no slider)
static lv_obj_t* btn_power_minus = nullptr;
static lv_obj_t* btn_power_plus  = nullptr;
static lv_obj_t* lbl_power_val   = nullptr;

// Preheat (plain toggle object, no lv_switch)
static lv_obj_t* btn_preheat   = nullptr;
static lv_obj_t* lbl_preheat   = nullptr;
static lv_obj_t* row_ph_ms     = nullptr;
static lv_obj_t* row_ph_pct    = nullptr;
static lv_obj_t* row_ph_gap    = nullptr;
static lv_obj_t* spin_ph_ms    = nullptr;
static lv_obj_t* spin_ph_pct   = nullptr;
static lv_obj_t* spin_ph_gap   = nullptr;

// Action buttons (no Revert)
static lv_obj_t* btn_apply     = nullptr;
static lv_obj_t* lbl_apply     = nullptr;
static lv_obj_t* lbl_pending   = nullptr;

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
static uint8_t  draft_mode       = 1;
static uint16_t draft_d1         = 5;
static uint16_t draft_gap1       = 0;
static uint16_t draft_d2         = 0;
static uint16_t draft_gap2       = 0;
static uint16_t draft_d3         = 0;
static uint8_t  draft_power      = 100;
static bool     draft_preheat_en = false;
static uint16_t draft_preheat_ms = 20;
static uint8_t  draft_preheat_pct = 30;
static uint16_t draft_preheat_gap = 3;

static uint8_t  applied_mode       = 1;
static uint16_t applied_d1         = 5;
static uint16_t applied_gap1       = 0;
static uint16_t applied_d2         = 0;
static uint16_t applied_gap2       = 0;
static uint16_t applied_d3         = 0;
static uint8_t  applied_power      = 100;
static bool     applied_preheat_en = false;
static uint16_t applied_preheat_ms = 20;
static uint8_t  applied_preheat_pct = 30;
static uint16_t applied_preheat_gap = 3;

static bool draft_dirty = false;
static bool applied_initialized = false;

// ============================================================
// HELPERS
// ============================================================

static void update_draft_dirty() {
    draft_dirty = (draft_mode != applied_mode ||
                   draft_d1   != applied_d1   ||
                   draft_gap1 != applied_gap1 ||
                   draft_d2   != applied_d2   ||
                   draft_gap2 != applied_gap2 ||
                   draft_d3   != applied_d3   ||
                   draft_power != applied_power ||
                   draft_preheat_en  != applied_preheat_en ||
                   draft_preheat_ms  != applied_preheat_ms ||
                   draft_preheat_pct != applied_preheat_pct ||
                   draft_preheat_gap != applied_preheat_gap);
}

static void sync_applied_from_state(const WelderDisplayState& st) {
    bool changed = false;

    if (applied_mode       != st.weld_mode)       { applied_mode       = st.weld_mode;       changed = true; }
    if (applied_d1         != st.pulse_d1)         { applied_d1         = st.pulse_d1;         changed = true; }
    if (applied_gap1       != st.pulse_gap1)       { applied_gap1       = st.pulse_gap1;       changed = true; }
    if (applied_d2         != st.pulse_d2)         { applied_d2         = st.pulse_d2;         changed = true; }
    if (applied_gap2       != st.pulse_gap2)       { applied_gap2       = st.pulse_gap2;       changed = true; }
    if (applied_d3         != st.pulse_d3)         { applied_d3         = st.pulse_d3;         changed = true; }
    if (applied_power      != st.power_pct)        { applied_power      = st.power_pct;        changed = true; }
    if (applied_preheat_en != st.preheat_enabled)  { applied_preheat_en = st.preheat_enabled;  changed = true; }
    if (applied_preheat_ms != st.preheat_ms)       { applied_preheat_ms = st.preheat_ms;       changed = true; }
    if (applied_preheat_pct != st.preheat_pct)     { applied_preheat_pct = st.preheat_pct;     changed = true; }
    if (applied_preheat_gap != st.preheat_gap_ms)  { applied_preheat_gap = st.preheat_gap_ms;  changed = true; }

    if (!applied_initialized) {
        draft_mode       = applied_mode;
        draft_d1         = applied_d1;
        draft_gap1       = applied_gap1;
        draft_d2         = applied_d2;
        draft_gap2       = applied_gap2;
        draft_d3         = applied_d3;
        draft_power      = applied_power;
        draft_preheat_en = applied_preheat_en;
        draft_preheat_ms = applied_preheat_ms;
        draft_preheat_pct = applied_preheat_pct;
        draft_preheat_gap = applied_preheat_gap;
        applied_initialized = true;
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
        lv_obj_set_style_bg_color(btn_arm,
                                  armed ? C_GREEN : C_RED, 0);
    }
    if (lbl_arm) {
        if (draft_dirty && !armed) {
            lv_label_set_text(lbl_arm, "Apply settings first!");
            if (btn_arm) lv_obj_set_style_bg_color(btn_arm, C_YELLOW, 0);
        } else {
            lv_label_set_text(lbl_arm,
                              armed ? "ARMED  (tap to disarm)"
                                    : "DISARMED  (tap to arm)");
        }
    }
}

// ============================================================
// ARM BUTTON EVENT
// ============================================================
static void arm_btn_event(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    // Block arming when draft != applied
    if (!_last_armed && draft_dirty) {
        paint_arm_button(false);
        return;
    }

    bool requested = !_last_armed;
    if (_arm_cb) {
        _arm_cb(requested);
    }
}

// ============================================================
// HELPER – create a value card inside a parent
// ============================================================
static lv_obj_t* make_card(lv_obj_t* parent, const char* title,
                           lv_obj_t** out_value_label,
                           int w, int h,
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
// STATUS TAB BUILDER
// ============================================================
static void build_status_tab(lv_obj_t* tab) {
    lv_obj_set_style_bg_color(tab, C_BG, LV_PART_MAIN);
    lv_obj_set_style_pad_all(tab, 10, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(tab, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);

    const int CONTENT_W = 780;
    const int GAP       = 10;

    lbl_state = lv_label_create(tab);
    lv_label_set_text(lbl_state, "DISARMED");
    lv_obj_set_style_text_color(lbl_state, C_RED, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_state, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(lbl_state, LV_ALIGN_TOP_MID, 0, 0);

    const int card_w = (CONTENT_W - 2 * GAP) / 3;
    const int card_h = 90;
    const int y_row1 = 30;
    int x = (CONTENT_W - (card_w * 3 + GAP * 2)) / 2;

    lv_obj_t* c1 = make_card(tab, "Pack Voltage", &lbl_pack_v, card_w, card_h);
    lv_obj_set_pos(c1, x, y_row1);
    lv_obj_t* c2 = make_card(tab, "Temperature", &lbl_temp, card_w, card_h);
    lv_obj_set_pos(c2, x + card_w + GAP, y_row1);
    lv_obj_t* c3 = make_card(tab, "Charger Current", &lbl_ichg, card_w, card_h);
    lv_obj_set_pos(c3, x + 2 * (card_w + GAP), y_row1);

    const int y_row2  = y_row1 + card_h + GAP;
    const int cell_w  = (CONTENT_W - 2 * GAP) / 3;
    const int cell_h  = 80;
    int cx = (CONTENT_W - (cell_w * 3 + GAP * 2)) / 2;

    lv_obj_t* cc1 = make_card(tab, "Cell 1", &lbl_cell1, cell_w, cell_h, &lv_font_montserrat_14);
    lv_obj_set_pos(cc1, cx, y_row2);
    lv_obj_t* cc2 = make_card(tab, "Cell 2", &lbl_cell2, cell_w, cell_h, &lv_font_montserrat_14);
    lv_obj_set_pos(cc2, cx + cell_w + GAP, y_row2);
    lv_obj_t* cc3 = make_card(tab, "Cell 3", &lbl_cell3, cell_w, cell_h, &lv_font_montserrat_14);
    lv_obj_set_pos(cc3, cx + 2 * (cell_w + GAP), y_row2);

    // ARM button – plain lv_obj (Widget A pattern – NO lv_button!)
    const int y_arm = y_row2 + cell_h + GAP + 6;
    btn_arm = lv_obj_create(tab);
    lv_obj_remove_style_all(btn_arm);
    lv_obj_add_flag(btn_arm, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(btn_arm, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(btn_arm, 260, 56);
    lv_obj_align(btn_arm, LV_ALIGN_TOP_MID, 0, y_arm);
    lv_obj_set_style_bg_color(btn_arm, C_RED, 0);
    lv_obj_set_style_bg_opa(btn_arm, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(btn_arm, 12, 0);
    lv_obj_set_scrollbar_mode(btn_arm, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(btn_arm, arm_btn_event, LV_EVENT_CLICKED, nullptr);
    make_interaction_safe(btn_arm);

    lbl_arm = lv_label_create(btn_arm);
    lv_label_set_text(lbl_arm, "DISARMED  (tap to arm)");
    lv_obj_set_style_text_color(lbl_arm, C_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_arm, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_center(lbl_arm);
}

// ============================================================
// PULSE TAB: show/hide row containers based on mode
// ============================================================
static void set_row_visible(lv_obj_t* row, bool show) {
    if (!row) return;
    if (show) lv_obj_clear_flag(row, LV_OBJ_FLAG_HIDDEN);
    else      lv_obj_add_flag(row, LV_OBJ_FLAG_HIDDEN);
}

static void update_pulse_field_visibility() {
    bool show_gap1 = (draft_mode >= 2);
    bool show_d2   = (draft_mode >= 2);
    bool show_gap2 = (draft_mode >= 3);
    bool show_d3   = (draft_mode >= 3);

    set_row_visible(row_gap1, show_gap1);
    set_row_visible(row_d2,   show_d2);
    set_row_visible(row_gap2, show_gap2);
    set_row_visible(row_d3,   show_d3);
}

static void update_preheat_visibility() {
    bool show = draft_preheat_en;
    set_row_visible(row_ph_ms,  show);
    set_row_visible(row_ph_pct, show);
    set_row_visible(row_ph_gap, show);
}

// _ui_dirty flag: set by callbacks, cleared by ui_update after visual refresh
static bool _ui_dirty = false;

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

    if      (obj == spin_d1)     draft_d1 = (uint16_t)val;
    else if (obj == spin_gap1)   draft_gap1 = (uint16_t)val;
    else if (obj == spin_d2)     draft_d2 = (uint16_t)val;
    else if (obj == spin_gap2)   draft_gap2 = (uint16_t)val;
    else if (obj == spin_d3)     draft_d3 = (uint16_t)val;
    else if (obj == spin_ph_ms)  draft_preheat_ms = (uint16_t)val;
    else if (obj == spin_ph_pct) draft_preheat_pct = (uint8_t)val;
    else if (obj == spin_ph_gap) draft_preheat_gap = (uint16_t)val;

    mark_dirty();
}

// Data-only callback: power +/- buttons
static void on_power_minus(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (draft_power > 50) { draft_power -= 5; mark_dirty(); }
}
static void on_power_plus(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (draft_power < 100) { draft_power += 5; mark_dirty(); }
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
        _recipe_cb(draft_mode, draft_d1, draft_gap1, draft_d2, draft_gap2, draft_d3,
                   draft_power,
                   draft_preheat_en, draft_preheat_ms, draft_preheat_pct, draft_preheat_gap);
    }
}

// ============================================================
// PULSE TAB: helper – create a big touch-friendly spinbox row
// Row layout:  [Label 90px] [– 56x46] [Value 84x46] [+ 56x46] [unit 40px]
// ALL +/- buttons use Widget A pattern (plain lv_obj, no theme)
// ============================================================
static lv_obj_t* make_touch_row(lv_obj_t* parent, const char* title,
                                const char* unit,
                                lv_obj_t** out_spinbox,
                                int min_val, int max_val,
                                int init_val, int digits, int step) {
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
    // The +/- buttons are the sole input; API calls still work without CLICKABLE.
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

    // Wire +/- to spinbox
    lv_obj_add_event_cb(btn_dec, [](lv_event_t* ev) {
        if (lv_event_get_code(ev) != LV_EVENT_CLICKED) return;
        lv_obj_t* s = (lv_obj_t*)lv_event_get_user_data(ev);
        lv_spinbox_decrement(s);
        lv_obj_send_event(s, LV_EVENT_VALUE_CHANGED, nullptr);
    }, LV_EVENT_CLICKED, spin);

    lv_obj_add_event_cb(btn_inc, [](lv_event_t* ev) {
        if (lv_event_get_code(ev) != LV_EVENT_CLICKED) return;
        lv_obj_t* s = (lv_obj_t*)lv_event_get_user_data(ev);
        lv_spinbox_increment(s);
        lv_obj_send_event(s, LV_EVENT_VALUE_CHANGED, nullptr);
    }, LV_EVENT_CLICKED, spin);

    lv_obj_add_event_cb(spin, on_spinbox_change, LV_EVENT_VALUE_CHANGED, nullptr);

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
        if (locked) lv_obj_add_state(lockable_objs[i], LV_STATE_DISABLED);
        else        lv_obj_remove_state(lockable_objs[i], LV_STATE_DISABLED);
    }

    auto set_lock = [&](lv_obj_t* obj) {
        if (!obj) return;
        if (locked) lv_obj_add_state(obj, LV_STATE_DISABLED);
        else        lv_obj_remove_state(obj, LV_STATE_DISABLED);
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
static lv_obj_t* make_section_header(lv_obj_t* parent, const char* text,
                                     int x, int y) {
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
static lv_obj_t* make_mode_button(lv_obj_t* parent, const char* text,
                                  int x, int y, int w, int h, int mode_idx) {
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

    row_d1 = make_touch_row(tab, "Pulse", "ms", &spin_d1, 1, 999, draft_d1, 3, 1);
    lv_obj_set_pos(row_d1, L, ly);
    ly += ROW_H;

    row_gap1 = make_touch_row(tab, "Gap", "ms", &spin_gap1, 1, 999, draft_gap1, 3, 1);
    lv_obj_set_pos(row_gap1, L, ly);
    ly += ROW_H;

    row_d2 = make_touch_row(tab, "Pulse 2", "ms", &spin_d2, 1, 999, draft_d2, 3, 1);
    lv_obj_set_pos(row_d2, L, ly);
    ly += ROW_H;

    row_gap2 = make_touch_row(tab, "Gap 2", "ms", &spin_gap2, 1, 999, draft_gap2, 3, 1);
    lv_obj_set_pos(row_gap2, L, ly);
    ly += ROW_H;

    row_d3 = make_touch_row(tab, "Pulse 3", "ms", &spin_d3, 1, 999, draft_d3, 3, 1);
    lv_obj_set_pos(row_d3, L, ly);

    update_pulse_field_visibility();

    // ======== RIGHT COLUMN ========

    ry = 2;

    // Power row constants (declared early for PH_X calculation)
    const int PWR_BTN_W = 56;
    const int PWR_BTN_H = 46;
    const int PWR_VAL_W = 84;
    const int PWR_GAP   = 8;   // symmetric gap between [-] [val] [+]

    // Derived positions for evenly-spaced  [-] [value] [+]  row
    const int X_PWR_MINUS = R;
    const int X_PWR_VAL   = R + PWR_BTN_W + PWR_GAP;
    const int X_PWR_PLUS  = X_PWR_VAL + PWR_VAL_W + PWR_GAP;

    make_section_header(tab, "Weld Power", R, ry);

    // "Preheat" label above the ON/OFF toggle, right of power + button with 20px gap
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
    lv_obj_add_event_cb(btn_power_minus, on_power_minus, LV_EVENT_CLICKED, nullptr);
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
    lv_obj_set_style_text_font(lbl_power_val, &lv_font_montserrat_20, LV_PART_MAIN);
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
    lv_obj_add_event_cb(btn_power_plus, on_power_plus, LV_EVENT_CLICKED, nullptr);
    make_interaction_safe(btn_power_plus);

    register_lockable(btn_power_minus);
    register_lockable(btn_power_plus);

    // Preheat ON/OFF toggle – same horizontal level as Weld Power +/- (Widget A pattern)
    btn_preheat = lv_obj_create(tab);
    lv_obj_remove_style_all(btn_preheat);
    lv_obj_add_flag(btn_preheat, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(btn_preheat, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(btn_preheat, 80, PWR_BTN_H);
    lv_obj_set_pos(btn_preheat, PH_X, ry);  // same Y as power buttons
    lv_obj_set_style_bg_color(btn_preheat, draft_preheat_en ? C_GREEN : C_DARK_GREY, 0);
    lv_obj_set_style_bg_opa(btn_preheat, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(btn_preheat, 16, 0);
    lbl_preheat = lv_label_create(btn_preheat);
    lv_label_set_text(lbl_preheat, draft_preheat_en ? "ON" : "OFF");
    lv_obj_set_style_text_color(lbl_preheat, C_WHITE, 0);
    lv_obj_set_style_text_font(lbl_preheat, &lv_font_montserrat_16, 0);
    lv_obj_center(lbl_preheat);
    lv_obj_add_event_cb(btn_preheat, on_preheat_toggle, LV_EVENT_CLICKED, nullptr);
    make_interaction_safe(btn_preheat);
    register_lockable(btn_preheat);

    ry += PWR_BTN_H + 16;

    row_ph_ms = make_touch_row(tab, "Duration", "ms", &spin_ph_ms,
                               1, 200, draft_preheat_ms, 3, 1);
    lv_obj_set_pos(row_ph_ms, R, ry);
    ry += ROW_H;

    row_ph_pct = make_touch_row(tab, "Power", "%", &spin_ph_pct,
                                10, 100, draft_preheat_pct, 3, 5);
    lv_obj_set_pos(row_ph_pct, R, ry);
    ry += ROW_H;

    row_ph_gap = make_touch_row(tab, "Gap", "ms", &spin_ph_gap,
                                1, 100, draft_preheat_gap, 3, 1);
    lv_obj_set_pos(row_ph_gap, R, ry);
    ry += ROW_H;

    update_preheat_visibility();

    // --- APPLY Button (Widget A pattern – plain lv_obj, NO lv_button, NO Revert) ---
    const int BTN_W = 200;
    const int BTN_H = 50;
    ry += 18;

    btn_apply = lv_obj_create(tab);
    lv_obj_remove_style_all(btn_apply);
    lv_obj_add_flag(btn_apply, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(btn_apply, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(btn_apply, BTN_W, BTN_H);
    lv_obj_set_pos(btn_apply, R, ry);
    lv_obj_set_style_bg_color(btn_apply, C_GREEN, 0);  // green = synced (initial)
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
    lv_obj_set_style_text_font(lbl_pending, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_pos(lbl_pending, R, ry);
    lv_obj_add_flag(lbl_pending, LV_OBJ_FLAG_HIDDEN);
}

// ============================================================
// PLACEHOLDER TAB BUILDER
// ============================================================
static void build_placeholder_tab(lv_obj_t* tab, const char* name) {
    lv_obj_set_style_bg_color(tab, C_BG, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(tab, LV_SCROLLBAR_MODE_OFF);

    char buf[64];
    snprintf(buf, sizeof(buf), "%s\n(coming in Phase 2)", name);

    lv_obj_t* lbl = lv_label_create(tab);
    lv_label_set_text(lbl, buf);
    lv_obj_set_style_text_color(lbl, C_GREY, LV_PART_MAIN);
    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_center(lbl);
}

// ============================================================
// PUBLIC: ui_init
// ============================================================
void ui_init(arm_toggle_cb_t on_arm_toggle, recipe_apply_cb_t on_recipe_apply) {
    _arm_cb    = on_arm_toggle;
    _recipe_cb = on_recipe_apply;

    lv_obj_t* scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, C_BG, LV_PART_MAIN);

    // Tabview – 5 tabs, tab bar at top, 36 px tall
    lv_obj_t* tv = lv_tabview_create(scr);
    lv_tabview_set_tab_bar_position(tv, LV_DIR_TOP);
    lv_tabview_set_tab_bar_size(tv, 36);
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

    // Style the tab buttons
    lv_obj_t* tab_btns = lv_tabview_get_tab_bar(tv);
    lv_obj_set_style_bg_color(tab_btns, C_DARK_GREY, LV_PART_MAIN);
    lv_obj_set_style_text_color(tab_btns, C_WHITE, LV_PART_MAIN);

    lv_obj_t* t0 = lv_tabview_add_tab(tv, LV_SYMBOL_HOME   " Status");
    lv_obj_t* t1 = lv_tabview_add_tab(tv, LV_SYMBOL_CHARGE  " Pulse");
    lv_obj_t* t2 = lv_tabview_add_tab(tv, LV_SYMBOL_BATTERY_FULL " Charger");
    lv_obj_t* t3 = lv_tabview_add_tab(tv, LV_SYMBOL_SETTINGS " Config");
    lv_obj_t* t4 = lv_tabview_add_tab(tv, LV_SYMBOL_LIST     " Logs");

    build_status_tab(t0);
    build_pulse_tab(t1);
    build_placeholder_tab(t2, "Charger Settings");
    build_placeholder_tab(t3, "Configuration");
    build_placeholder_tab(t4, "Logs");

    // Post-build: disable scrolling/gesture on tab containers
    lv_obj_clear_flag(t0, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
    lv_obj_clear_flag(t0, LV_OBJ_FLAG_GESTURE_BUBBLE);

    // Pulse tab: no scrolling at all to prevent shudder
    lv_obj_clear_flag(t1, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(t1, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
    lv_obj_clear_flag(t1, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_set_scrollbar_mode(t1, LV_SCROLLBAR_MODE_OFF);

    lv_obj_clear_flag(t2, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(t2, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_clear_flag(t3, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(t3, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_clear_flag(t4, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(t4, LV_OBJ_FLAG_GESTURE_BUBBLE);
}

// ============================================================
// PUBLIC: ui_has_pending_changes
// ============================================================
bool ui_has_pending_changes() {
    return draft_dirty;
}

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
        interval = 250;       // 4 Hz briefly after release
    }
    if (now - last_ms < interval) return;
    last_ms = now;

    // ---- Sync applied recipe from authoritative source ----
    sync_applied_from_state(st);

    // ---- Lock/unlock Pulse tab based on armed state ----
    // Removed: allow recipe editing while armed (draft/apply model still enforced)
    // lock_pulse_tab(st.armed);

    // ---- Change-detection state (static across calls) ----
    static float    prev_pack_v   = -999.0f;
    static float    prev_temp     = -999.0f;
    static bool     prev_temp_fin = false;
    static float    prev_ichg     = -999.0f;
    static float    prev_cell1    = -999.0f;
    static float    prev_cell2    = -999.0f;
    static float    prev_cell3    = -999.0f;
    static bool     prev_armed    = false;
    static bool     prev_welding  = false;
    static bool     prev_charging = false;
    static bool     first_run     = true;

    char buf[32];

    // ---- Pack Voltage ----
    if (lbl_pack_v && (first_run || fabsf(st.pack_voltage - prev_pack_v) >= 0.01f)) {
        snprintf(buf, sizeof(buf), "%.2f V", (double)st.pack_voltage);
        lv_label_set_text(lbl_pack_v, buf);
        lv_color_t c = (st.pack_voltage < 8.0f) ? C_RED :
                       (st.pack_voltage > 9.0f) ? C_YELLOW : C_GREEN;
        lv_obj_set_style_text_color(lbl_pack_v, c, LV_PART_MAIN);
        prev_pack_v = st.pack_voltage;
    }

    // ---- Temperature ----
    {
        bool fin = isfinite(st.temperature);
        if (lbl_temp && (first_run || fin != prev_temp_fin ||
                         (fin && fabsf(st.temperature - prev_temp) >= 0.05f))) {
            if (fin) {
                snprintf(buf, sizeof(buf), "%.1f \xC2\xB0""C", (double)st.temperature);
                lv_label_set_text(lbl_temp, buf);
                lv_color_t c = (st.temperature > 50.0f) ? C_RED : C_GREEN;
                lv_obj_set_style_text_color(lbl_temp, c, LV_PART_MAIN);
            } else {
                lv_label_set_text(lbl_temp, "ERR");
                lv_obj_set_style_text_color(lbl_temp, C_RED, LV_PART_MAIN);
            }
            prev_temp     = st.temperature;
            prev_temp_fin = fin;
        }
    }

    // ---- Charger Current ----
    if (lbl_ichg && (first_run || fabsf(st.charger_current - prev_ichg) >= 0.01f)) {
        snprintf(buf, sizeof(buf), "%.2f A", (double)st.charger_current);
        lv_label_set_text(lbl_ichg, buf);
        prev_ichg = st.charger_current;
    }

    // ---- Cell Voltages ----
    if (lbl_cell1 && (first_run || fabsf(st.cell1_v - prev_cell1) >= 0.001f)) {
        snprintf(buf, sizeof(buf), "%.3f V", (double)st.cell1_v);
        lv_label_set_text(lbl_cell1, buf);
        prev_cell1 = st.cell1_v;
    }
    if (lbl_cell2 && (first_run || fabsf(st.cell2_v - prev_cell2) >= 0.001f)) {
        snprintf(buf, sizeof(buf), "%.3f V", (double)st.cell2_v);
        lv_label_set_text(lbl_cell2, buf);
        prev_cell2 = st.cell2_v;
    }
    if (lbl_cell3 && (first_run || fabsf(st.cell3_v - prev_cell3) >= 0.001f)) {
        snprintf(buf, sizeof(buf), "%.3f V", (double)st.cell3_v);
        lv_label_set_text(lbl_cell3, buf);
        prev_cell3 = st.cell3_v;
    }

    // ---- ARM button ----
    _last_armed = st.armed;
    bool arm_changed = (first_run || st.armed != prev_armed);
    if (arm_changed) {
        paint_arm_button(st.armed);
        prev_armed = st.armed;
    }

    // ---- State label ----
    if (arm_changed ||
        st.welding != prev_welding || st.charging != prev_charging) {
        if (lbl_state) {
            const char* txt;
            lv_color_t col;
            if (!st.armed) {
                txt = "DISARMED"; col = C_RED;
            } else if (st.welding) {
                txt = "WELDING";  col = C_YELLOW;
            } else if (st.charging) {
                txt = "CHARGING"; col = C_ACCENT;
            } else {
                txt = "IDLE";     col = C_GREEN;
            }
            lv_label_set_text(lbl_state, txt);
            lv_obj_set_style_text_color(lbl_state, col, LV_PART_MAIN);
        }
        prev_welding  = st.welding;
        prev_charging = st.charging;
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

        bool need_refresh = _ui_dirty || (draft_dirty != prev_dirty) || first_run;

        if (need_refresh) {
            // Mode buttons: repaint selection
            for (int i = 0; i < 3; i++) {
                if (!btn_mode[i]) continue;
                bool sel = ((int)draft_mode == i + 1);
                lv_obj_set_style_bg_color(btn_mode[i], sel ? C_ACCENT : C_DARK_GREY, 0);
            }

            // Field visibility based on mode
            if (draft_mode != prev_pulse_mode) {
                update_pulse_field_visibility();
                prev_pulse_mode = draft_mode;
            }

            // Power value label
            if (lbl_power_val && draft_power != prev_pulse_power) {
                char pbuf[16];
                snprintf(pbuf, sizeof(pbuf), "%d %%", (int)draft_power);
                lv_label_set_text(lbl_power_val, pbuf);
                prev_pulse_power = draft_power;
            }

            // Preheat toggle visual
            if (btn_preheat) {
                lv_obj_set_style_bg_color(btn_preheat,
                    draft_preheat_en ? C_GREEN : C_DARK_GREY, 0);
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
                    lv_label_set_text(lbl_pending, LV_SYMBOL_WARNING " Changes pending");
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
