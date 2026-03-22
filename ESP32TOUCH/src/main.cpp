#include <Arduino.h>
#include <esp32_smartdisplay.h>
#include <lvgl.h>

static lv_obj_t* label;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("Booting smartdisplay LVGL test...");

    smartdisplay_init();
    smartdisplay_lcd_set_backlight(1.0);

    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_black(),
                              LV_PART_MAIN);

    label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Spot Welder UI Test");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
    lv_obj_center(label);

    Serial.println("UI created");
}

auto lastLvTick = millis();

void loop() {
    auto now = millis();
    lv_tick_inc(now - lastLvTick);
    lastLvTick = now;
    lv_timer_handler();
    delay(5);
}