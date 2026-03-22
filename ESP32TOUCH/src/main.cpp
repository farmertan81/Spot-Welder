#include <Arduino.h>
#include <Arduino_GFX_Library.h>

#define GFX_BL 2

Arduino_ESP32RGBPanel* bus = new Arduino_ESP32RGBPanel(
    40, 41, 39, 42,      // DE, VSYNC, HSYNC, PCLK
    45, 48, 47, 21, 14,  // R0-R4
    5, 6, 7, 15, 16, 4,  // G0-G5
    8, 3, 46, 9, 1,      // B0-B4
    0, 8, 4, 8,          // HSYNC polarity, front porch, pulse width, back porch
    0, 8, 4, 8,          // VSYNC polarity, front porch, pulse width, back porch
    1,                   // PCLK active negative
    16000000             // prefer speed
);

Arduino_RGB_Display* gfx = new Arduino_RGB_Display(800, 480, bus, 0, true);

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("Booting display test...");

    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);

    gfx->begin();
    gfx->fillScreen(BLACK);

    gfx->setTextColor(WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(40, 40);
    gfx->println("SPOT WELDER");

    gfx->setTextColor(GREEN);
    gfx->setCursor(40, 80);
    gfx->println("DISPLAY TEST OK");

    Serial.println("Display initialized");
}

void loop() { delay(1000); }