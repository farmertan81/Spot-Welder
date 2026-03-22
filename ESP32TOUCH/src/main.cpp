/**
 * ESP32-8048S043C Touch Fix - Main Application
 *
 * This firmware fixes GT911 touch responsiveness issues on the ESP32-8048S043C
 * display board, including:
 *   - Double taps being registered (ghost touches)
 *   - Missed taps
 *   - Erratic touch behavior
 *
 * FIXES APPLIED:
 * 1. GT911 INT pin disabled (GPIO_NUM_NC) - prevents interrupt-driven ghost
 * touches
 * 2. I2C speed increased to 400kHz - faster, more reliable touch reads
 * 3. Software debouncing with configurable thresholds
 * 4. Touch coordinate smoothing (low-pass filter)
 * 5. Press/release state machine with hysteresis
 * 6. Minimum press duration filter to reject noise
 *
 * The test UI includes:
 *   - Touch coordinate display (real-time X,Y readout)
 *   - Tap counter to verify single-tap accuracy
 *   - 12-button grid to visually verify tap responsiveness
 *   - Status indicators for touch state
 *
 * For the Spot Welder project: once touch is verified working, copy
 * the debouncing logic (TouchFilter) into your main project's main.cpp.
 *
 * Hardware: ESP32-8048S043C (Sunton 4.3" 800x480 with GT911 cap touch)
 * Framework: Arduino + LVGL 9.x + esp32-smartdisplay
 */

#include <Arduino.h>
#include <esp32_smartdisplay.h>
#include <lv_conf.h>
#include <lvgl.h>

/* ============================================================
 * TOUCH FILTER / DEBOUNCE CONFIGURATION
 * Tune these values if you still experience issues.
 * ============================================================ */

// Minimum time (ms) a touch must be held to register as a press.
// Rejects brief noise spikes. Increase if getting ghost taps.
#define TOUCH_MIN_PRESS_MS 30

// Minimum time (ms) between consecutive press events.
// Prevents double-tap registration. Increase if getting doubles.
#define TOUCH_DEBOUNCE_MS 80

// Minimum time (ms) touch must be released before allowing next press.
// Adds hysteresis to press/release transitions.
#define TOUCH_RELEASE_DEBOUNCE_MS 40

// Coordinate smoothing factor (0.0 = no smoothing, 0.9 = heavy smoothing).
// Higher values = smoother but more laggy cursor movement.
#define TOUCH_SMOOTH_FACTOR 0.3f

// GT911 touch coordinate scaling.
// The GT911 on this board reports coordinates in a smaller range than the
// display resolution. These define the observed max raw values from the GT911.
// Measured by touching the corners: top-right ~(470,10), bottom-right
// ~(470,265). We scale from this raw range to the full 800x480 display.
#define GT911_RAW_X_MAX 470
#define GT911_RAW_Y_MAX 265
#define DISPLAY_RES_X 800
#define DISPLAY_RES_Y 480

// Maximum jump distance (pixels) to accept. Larger jumps are treated as
// a new touch point rather than noise. Set to 0 to disable.
#define TOUCH_MAX_JUMP_PX 100

/* ============================================================
 * TOUCH FILTER STATE MACHINE
 * ============================================================ */

enum TouchState {
    TOUCH_IDLE,            // Not touched, waiting for press
    TOUCH_PENDING_PRESS,   // Touch detected, waiting for MIN_PRESS_MS
    TOUCH_PRESSED,         // Confirmed press - reporting to LVGL
    TOUCH_PENDING_RELEASE  // Touch released, waiting for RELEASE_DEBOUNCE_MS
};

static struct {
    TouchState state;
    uint32_t state_enter_ms;
    uint32_t last_press_ms;
    float smooth_x, smooth_y;
    int16_t last_raw_x, last_raw_y;  // for debug display
    bool has_prev_coords;
    uint32_t tap_count;
    uint32_t touch_reads;
    uint32_t rejected_noise;
} touch_filter = {};

/* ============================================================
 * ORIGINAL CALLBACK STORAGE
 *
 * The esp32-smartdisplay library installs its own indev read
 * callback (lvgl_touch_calibration_transform) which calls the
 * low-level GT911 driver, then applies calibration transforms.
 *
 * We save that callback and wrap it with our debouncing layer.
 * This preserves the library's calibration logic.
 * ============================================================ */

static lv_indev_read_cb_t original_read_cb = nullptr;

/* ============================================================
 * DEBOUNCED TOUCH READ CALLBACK
 *
 * Flow: GT911 hardware -> smartdisplay driver -> calibration ->
 *       OUR DEBOUNCE FILTER -> LVGL
 * ============================================================ */

static void debounced_touchpad_read(lv_indev_t* indev, lv_indev_data_t* data) {
    touch_filter.touch_reads++;

    // Step 1: Call the original smartdisplay read callback
    // This reads GT911 via I2C and applies calibration transforms
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

    // Save raw coords for debug display
    if (touched) {
        touch_filter.last_raw_x = raw_x;
        touch_filter.last_raw_y = raw_y;
    }

    // Step 1.5: Scale GT911 raw coordinates to display resolution.
    // The GT911 reports in its native range (~0-470 x ~0-265) which does NOT
    // match the 800x480 display. We scale linearly and clamp to bounds.
    int16_t tx = (int16_t)((int32_t)raw_x * DISPLAY_RES_X / GT911_RAW_X_MAX);
    int16_t ty = (int16_t)((int32_t)raw_y * DISPLAY_RES_Y / GT911_RAW_Y_MAX);

    // Clamp to display bounds after scaling
    if (tx < 0) tx = 0;
    if (ty < 0) ty = 0;
    if (tx >= DISPLAY_RES_X) tx = DISPLAY_RES_X - 1;
    if (ty >= DISPLAY_RES_Y) ty = DISPLAY_RES_Y - 1;

    uint32_t now = millis();

    // Step 2: Run our debounce state machine
    switch (touch_filter.state) {
        case TOUCH_IDLE:
            if (touched) {
                // New touch detected - start pending press timer
                touch_filter.smooth_x = (float)tx;
                touch_filter.smooth_y = (float)ty;
                touch_filter.state = TOUCH_PENDING_PRESS;
                touch_filter.state_enter_ms = now;
            }
            data->state = LV_INDEV_STATE_RELEASED;
            // Report last known position (LVGL needs valid coords)
            data->point.x = (int16_t)touch_filter.smooth_x;
            data->point.y = (int16_t)touch_filter.smooth_y;
            break;

        case TOUCH_PENDING_PRESS:
            if (!touched) {
                // Released before MIN_PRESS_MS - reject as noise
                touch_filter.state = TOUCH_IDLE;
                touch_filter.state_enter_ms = now;
                touch_filter.rejected_noise++;
                data->state = LV_INDEV_STATE_RELEASED;
                data->point.x = (int16_t)touch_filter.smooth_x;
                data->point.y = (int16_t)touch_filter.smooth_y;
            } else {
                // Still touching - update smoothed coords
                float alpha = TOUCH_SMOOTH_FACTOR;
                touch_filter.smooth_x =
                    alpha * (float)tx + (1.0f - alpha) * touch_filter.smooth_x;
                touch_filter.smooth_y =
                    alpha * (float)ty + (1.0f - alpha) * touch_filter.smooth_y;

                // Check if held long enough to confirm press
                if ((now - touch_filter.state_enter_ms) >= TOUCH_MIN_PRESS_MS) {
                    // Also check debounce from last confirmed press
                    if ((now - touch_filter.last_press_ms) >=
                        TOUCH_DEBOUNCE_MS) {
                        // CONFIRMED PRESS
                        touch_filter.state = TOUCH_PRESSED;
                        touch_filter.state_enter_ms = now;
                        touch_filter.last_press_ms = now;
                        touch_filter.tap_count++;

                        data->point.x = (int16_t)touch_filter.smooth_x;
                        data->point.y = (int16_t)touch_filter.smooth_y;
                        data->state = LV_INDEV_STATE_PRESSED;
                    } else {
                        // Within debounce window from previous press - reject
                        touch_filter.state = TOUCH_IDLE;
                        touch_filter.rejected_noise++;
                        data->state = LV_INDEV_STATE_RELEASED;
                        data->point.x = (int16_t)touch_filter.smooth_x;
                        data->point.y = (int16_t)touch_filter.smooth_y;
                    }
                } else {
                    // Still waiting for min press time - don't report yet
                    data->state = LV_INDEV_STATE_RELEASED;
                    data->point.x = (int16_t)touch_filter.smooth_x;
                    data->point.y = (int16_t)touch_filter.smooth_y;
                }
            }
            break;

        case TOUCH_PRESSED:
            if (!touched) {
                // Touch lifted - start release debounce
                touch_filter.state = TOUCH_PENDING_RELEASE;
                touch_filter.state_enter_ms = now;
                // Keep reporting pressed during release debounce
                data->point.x = (int16_t)touch_filter.smooth_x;
                data->point.y = (int16_t)touch_filter.smooth_y;
                data->state = LV_INDEV_STATE_PRESSED;
            } else {
                // Still touching - smooth the coordinates
                float alpha = TOUCH_SMOOTH_FACTOR;
                float new_x =
                    alpha * (float)tx + (1.0f - alpha) * touch_filter.smooth_x;
                float new_y =
                    alpha * (float)ty + (1.0f - alpha) * touch_filter.smooth_y;

                // Check for coordinate jump (noise)
                if (TOUCH_MAX_JUMP_PX > 0) {
                    float dx = new_x - touch_filter.smooth_x;
                    float dy = new_y - touch_filter.smooth_y;
                    float dist = sqrtf(dx * dx + dy * dy);
                    if (dist > TOUCH_MAX_JUMP_PX) {
                        // Large jump after smoothing - snap to new position
                        new_x = (float)tx;
                        new_y = (float)ty;
                    }
                }

                touch_filter.smooth_x = new_x;
                touch_filter.smooth_y = new_y;

                // Clamp to display bounds
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
                // Touch came back during release debounce - return to pressed
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
                // Confirmed release
                touch_filter.state = TOUCH_IDLE;
                touch_filter.state_enter_ms = now;
                touch_filter.has_prev_coords = false;
                data->state = LV_INDEV_STATE_RELEASED;
                data->point.x = (int16_t)touch_filter.smooth_x;
                data->point.y = (int16_t)touch_filter.smooth_y;
            } else {
                // Still in release debounce - keep reporting pressed
                data->point.x = (int16_t)touch_filter.smooth_x;
                data->point.y = (int16_t)touch_filter.smooth_y;
                data->state = LV_INDEV_STATE_PRESSED;
            }
            break;
    }

    data->continue_reading = false;
}

/* ============================================================
 * LVGL UI ELEMENTS
 * ============================================================ */
static lv_obj_t* label_coords = nullptr;
static lv_obj_t* label_state = nullptr;
static lv_obj_t* label_tap_count = nullptr;
static lv_obj_t* label_debug = nullptr;
static lv_obj_t* touch_indicator = nullptr;

static void create_test_ui(void);
static void update_ui(void);

/* ============================================================
 * TEST UI CREATION
 * ============================================================ */

// Button click handler - flashes green to confirm single tap
static void btn_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);

    if (code == LV_EVENT_CLICKED) {
        // Flash green to confirm tap
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x00FF00), LV_PART_MAIN);

        // Reset color after 200ms
        lv_timer_create(
            [](lv_timer_t* t) {
                lv_obj_t* obj = (lv_obj_t*)lv_timer_get_user_data(t);
                lv_obj_set_style_bg_color(obj, lv_color_hex(0x2196F3),
                                          LV_PART_MAIN);
                lv_timer_delete(t);
            },
            200, btn);

        // Log to serial for debugging
        lv_obj_t* label = lv_obj_get_child(btn, 0);
        if (label) {
            Serial.printf("CLICK: %s (tap #%lu)\n", lv_label_get_text(label),
                          (unsigned long)touch_filter.tap_count);
        }
    }
}

// Reset counter button handler
static void reset_counter_cb(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        touch_filter.tap_count = 0;
        touch_filter.rejected_noise = 0;
        touch_filter.touch_reads = 0;
        Serial.println("Counters reset.");
    }
}

static void create_test_ui(void) {
    lv_obj_t* scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1A1A2E), LV_PART_MAIN);

    // ===== TITLE =====
    lv_obj_t* title = lv_label_create(scr);
    lv_label_set_text(title, LV_SYMBOL_OK " ESP32-8048S043C Touch Test");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);

    // ===== STATUS PANEL (left side) =====
    lv_obj_t* panel = lv_obj_create(scr);
    lv_obj_set_size(panel, 280, 200);
    lv_obj_align(panel, LV_ALIGN_TOP_LEFT, 10, 28);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x16213E), LV_PART_MAIN);
    lv_obj_set_style_border_color(panel, lv_color_hex(0x0F3460), LV_PART_MAIN);
    lv_obj_set_style_border_width(panel, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(panel, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(panel, 10, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(panel, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t* panel_title = lv_label_create(panel);
    lv_label_set_text(panel_title, LV_SYMBOL_EYE_OPEN " Touch Status");
    lv_obj_set_style_text_color(panel_title, lv_color_hex(0xE94560),
                                LV_PART_MAIN);
    lv_obj_align(panel_title, LV_ALIGN_TOP_LEFT, 0, 0);

    label_coords = lv_label_create(panel);
    lv_label_set_text(label_coords, "X: ---  Y: ---");
    lv_obj_set_style_text_color(label_coords, lv_color_hex(0x00FF88),
                                LV_PART_MAIN);
    lv_obj_align(label_coords, LV_ALIGN_TOP_LEFT, 0, 22);

    label_state = lv_label_create(panel);
    lv_label_set_text(label_state, "State: IDLE");
    lv_obj_set_style_text_color(label_state, lv_color_hex(0xFFDD44),
                                LV_PART_MAIN);
    lv_obj_align(label_state, LV_ALIGN_TOP_LEFT, 0, 44);

    label_tap_count = lv_label_create(panel);
    lv_label_set_text(label_tap_count, LV_SYMBOL_BELL " Taps: 0");
    lv_obj_set_style_text_color(label_tap_count, lv_color_hex(0x44DDFF),
                                LV_PART_MAIN);
    lv_obj_align(label_tap_count, LV_ALIGN_TOP_LEFT, 0, 66);

    label_debug = lv_label_create(panel);
    lv_label_set_text(label_debug, "Reads: 0 | Noise: 0");
    lv_obj_set_style_text_color(label_debug, lv_color_hex(0x888888),
                                LV_PART_MAIN);
    lv_obj_align(label_debug, LV_ALIGN_TOP_LEFT, 0, 88);

    // Reset button
    lv_obj_t* btn_reset = lv_button_create(panel);
    lv_obj_set_size(btn_reset, 120, 35);
    lv_obj_align(btn_reset, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_set_style_bg_color(btn_reset, lv_color_hex(0xE94560), LV_PART_MAIN);
    lv_obj_add_event_cb(btn_reset, reset_counter_cb, LV_EVENT_CLICKED, nullptr);

    lv_obj_t* reset_label = lv_label_create(btn_reset);
    lv_label_set_text(reset_label, LV_SYMBOL_REFRESH " Reset");
    lv_obj_center(reset_label);

    // ===== TEST BUTTON GRID (right side) =====
    lv_obj_t* grid_panel = lv_obj_create(scr);
    lv_obj_set_size(grid_panel, 480, 200);
    lv_obj_align(grid_panel, LV_ALIGN_TOP_RIGHT, -10, 28);
    lv_obj_set_style_bg_color(grid_panel, lv_color_hex(0x16213E), LV_PART_MAIN);
    lv_obj_set_style_border_color(grid_panel, lv_color_hex(0x0F3460),
                                  LV_PART_MAIN);
    lv_obj_set_style_border_width(grid_panel, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(grid_panel, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(grid_panel, 8, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(grid_panel, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_flex_flow(grid_panel, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(grid_panel, LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_EVENLY);

    // 4x3 grid of test buttons
    for (int i = 1; i <= 12; i++) {
        lv_obj_t* btn = lv_button_create(grid_panel);
        lv_obj_set_size(btn, 100, 50);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x2196F3), LV_PART_MAIN);
        lv_obj_set_style_radius(btn, 6, LV_PART_MAIN);
        lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, nullptr);

        lv_obj_t* label = lv_label_create(btn);
        lv_label_set_text_fmt(label, "BTN %d", i);
        lv_obj_center(label);
    }

    // ===== BOTTOM INSTRUCTION PANEL =====
    lv_obj_t* bottom_bar = lv_obj_create(scr);
    lv_obj_set_size(bottom_bar, 780, 225);
    lv_obj_align(bottom_bar, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_set_style_bg_color(bottom_bar, lv_color_hex(0x0F3460), LV_PART_MAIN);
    lv_obj_set_style_border_width(bottom_bar, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(bottom_bar, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(bottom_bar, 10, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(bottom_bar, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t* instr = lv_label_create(bottom_bar);
    lv_label_set_text(
        instr, LV_SYMBOL_LIST
        " TOUCH TEST INSTRUCTIONS:\n"
        "1. Tap each numbered button ONCE - should flash GREEN exactly once\n"
        "2. Watch tap counter - should increment by exactly 1 per tap\n"
        "3. Touch all 4 corners - red dot should track your finger accurately\n"
        "4. Drag across screen - coordinates should update smoothly\n"
        "5. If position is off: adjust GT911_RAW_X/Y_MAX scaling constants\n"
        "6. 'Scaled' = LVGL coords (0-800,0-480)  'Raw' = GT911 native "
        "coords\n\n" LV_SYMBOL_SETTINGS " Scaling: GT911 " LV_SYMBOL_RIGHT
        " 470x265 -> 800x480  |  "
        "Debounce=80ms  Smooth=0.30");
    lv_obj_set_style_text_color(instr, lv_color_hex(0xCCCCCC), LV_PART_MAIN);
    lv_obj_set_width(instr, 760);
    lv_obj_align(instr, LV_ALIGN_TOP_LEFT, 0, 0);

    // Touch indicator (follows finger position)
    touch_indicator = lv_obj_create(scr);
    lv_obj_set_size(touch_indicator, 20, 20);
    lv_obj_set_style_bg_color(touch_indicator, lv_color_hex(0xFF0000),
                              LV_PART_MAIN);
    lv_obj_set_style_radius(touch_indicator, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_border_width(touch_indicator, 0, LV_PART_MAIN);
    lv_obj_set_style_opa(touch_indicator, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_remove_flag(touch_indicator, LV_OBJ_FLAG_CLICKABLE);
}

/* ============================================================
 * UI UPDATE (called from main loop)
 * ============================================================ */

static void update_ui(void) {
    static uint32_t last_update_ms = 0;
    uint32_t now = millis();
    if ((now - last_update_ms) < 50) return;  // 20Hz update rate
    last_update_ms = now;

    // Coordinate display (show both raw GT911 and scaled values)
    if (touch_filter.state == TOUCH_PRESSED ||
        touch_filter.state == TOUCH_PENDING_RELEASE) {
        lv_label_set_text_fmt(
            label_coords, "Scaled: %d,%d  Raw: %d,%d",
            (int)touch_filter.smooth_x, (int)touch_filter.smooth_y,
            (int)touch_filter.last_raw_x, (int)touch_filter.last_raw_y);
        lv_obj_set_style_opa(touch_indicator, LV_OPA_70, LV_PART_MAIN);
        lv_obj_set_pos(touch_indicator, (int)touch_filter.smooth_x - 10,
                       (int)touch_filter.smooth_y - 10);
    } else {
        lv_label_set_text(label_coords, "X: ---  Y: ---");
        lv_obj_set_style_opa(touch_indicator, LV_OPA_TRANSP, LV_PART_MAIN);
    }

    // State display
    const char* state_names[] = {"IDLE", "PENDING PRESS", "PRESSED",
                                 "PENDING RELEASE"};
    lv_label_set_text_fmt(label_state, "State: %s",
                          state_names[(int)touch_filter.state]);

    // Tap counter
    lv_label_set_text_fmt(label_tap_count, LV_SYMBOL_BELL " Taps: %lu",
                          (unsigned long)touch_filter.tap_count);

    // Debug info
    lv_label_set_text_fmt(label_debug, "Reads: %lu | Noise: %lu",
                          (unsigned long)touch_filter.touch_reads,
                          (unsigned long)touch_filter.rejected_noise);
}

/* ============================================================
 * SETUP & MAIN LOOP
 * ============================================================ */

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("===========================================");
    Serial.println("ESP32-8048S043C Touch Fix - Starting...");
    Serial.println("===========================================");
    Serial.printf("Touch debounce config:\n");
    Serial.printf("  MIN_PRESS_MS:         %d\n", TOUCH_MIN_PRESS_MS);
    Serial.printf("  DEBOUNCE_MS:          %d\n", TOUCH_DEBOUNCE_MS);
    Serial.printf("  RELEASE_DEBOUNCE_MS:  %d\n", TOUCH_RELEASE_DEBOUNCE_MS);
    Serial.printf("  SMOOTH_FACTOR:        %.2f\n", TOUCH_SMOOTH_FACTOR);
    Serial.printf("  MAX_JUMP_PX:          %d\n", TOUCH_MAX_JUMP_PX);
    Serial.printf("  GT911 raw range:      %dx%d -> scaled to %dx%d\n",
                  GT911_RAW_X_MAX, GT911_RAW_Y_MAX, DISPLAY_RES_X,
                  DISPLAY_RES_Y);
    Serial.println("===========================================");

    // Initialize smartdisplay library (handles display + GT911 init)
    smartdisplay_init();

    // Turn backlight on full
    smartdisplay_lcd_set_backlight(1.0f);

    Serial.println("Display initialized.");

    // Find the touch indev registered by smartdisplay and wrap its callback
    lv_indev_t* indev = lv_indev_get_next(nullptr);
    bool found_touch = false;

    while (indev != nullptr) {
        if (lv_indev_get_type(indev) == LV_INDEV_TYPE_POINTER) {
            Serial.println("Found touch indev - wrapping with debounce filter");

            // Save the library's callback (includes calibration transform)
            original_read_cb = indev->read_cb;

            // Replace with our debounced version
            indev->read_cb = debounced_touchpad_read;

            found_touch = true;
            break;
        }
        indev = lv_indev_get_next(indev);
    }

    if (!found_touch) {
        Serial.println("WARNING: No touch indev found from smartdisplay!");
        Serial.println("Touch may not work. Check board configuration.");
    }

    // Create test UI
    create_test_ui();

    Serial.println("UI created. Touch the screen to test!");
    Serial.println("===========================================");
}

// Track LVGL tick timing
static unsigned long lv_last_tick = 0;

void loop() {
    // Update LVGL tick
    unsigned long now = millis();
    lv_tick_inc(now - lv_last_tick);
    lv_last_tick = now;

    // Process LVGL tasks (including touch reads)
    lv_timer_handler();

    // Update our debug/status UI
    update_ui();

    // Small delay to prevent CPU hogging
    delay(5);
}
