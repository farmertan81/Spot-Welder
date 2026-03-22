#include <Arduino.h>
#include <Wire.h>

#define TOUCH_SDA 19
#define TOUCH_SCL 20
#define TOUCH_ADDR 0x5D

#define GT911_REG_PRODUCT_ID 0x8140
#define GT911_REG_STATUS 0x814E
#define GT911_REG_POINTS 0x814F

bool gt911Read(uint16_t reg, uint8_t* data, size_t len) {
    Wire.beginTransmission(TOUCH_ADDR);
    Wire.write(reg >> 8);
    Wire.write(reg & 0xFF);
    if (Wire.endTransmission(false) != 0) return false;

    size_t n = Wire.requestFrom((int)TOUCH_ADDR, (int)len);
    if (n != len) return false;

    for (size_t i = 0; i < len; i++) data[i] = Wire.read();

    return true;
}

bool gt911Write8(uint16_t reg, uint8_t value) {
    Wire.beginTransmission(TOUCH_ADDR);
    Wire.write(reg >> 8);
    Wire.write(reg & 0xFF);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

void printProductID() {
    uint8_t buf[4];
    if (gt911Read(GT911_REG_PRODUCT_ID, buf, 4)) {
        Serial.print("GT911 Product ID: ");
        for (int i = 0; i < 4; i++) Serial.write(buf[i]);
        Serial.println();
    } else {
        Serial.println("Failed to read GT911 product ID");
    }
}

void readTouch() {
    uint8_t status = 0;
    if (!gt911Read(GT911_REG_STATUS, &status, 1)) {
        Serial.println("Failed to read touch status");
        delay(200);
        return;
    }

    if ((status & 0x80) == 0) {
        delay(20);
        return;
    }

    uint8_t points = status & 0x0F;
    if (points == 0 || points > 5) {
        gt911Write8(GT911_REG_STATUS, 0);
        delay(20);
        return;
    }

    uint8_t data[8];
    if (!gt911Read(GT911_REG_POINTS, data, 8)) {
        Serial.println("Failed to read touch point data");
        gt911Write8(GT911_REG_STATUS, 0);
        delay(20);
        return;
    }

    uint16_t x = data[1] | (data[2] << 8);
    uint16_t y = data[3] | (data[4] << 8);

    Serial.print("Touch: x=");
    Serial.print(x);
    Serial.print(" y=");
    Serial.print(y);
    Serial.print(" points=");
    Serial.println(points);

    gt911Write8(GT911_REG_STATUS, 0);
    delay(20);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("GT911 touch read test");

    Wire.begin(TOUCH_SDA, TOUCH_SCL);
    delay(100);

    printProductID();
}

void loop() { readTouch(); }