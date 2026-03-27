/**
 * Spot Welder – Merged Firmware (Milestone 1) – I2C FIX
 *
 * Base: Logic project (WiFi/TCP, UART to STM32, INA226 battery, charger)
 * Added: Display (smartdisplay + LVGL), touch (GT911 with debounce filter),
 *        minimal Status screen with arm toggle.
 *
 * I2C COEXISTENCE STRATEGY (FIXED):
 *   smartdisplay_init() claims I2C_NUM_0 for GT911 touch on GPIO19/GPIO20.
 *   DO NOT call Wire.begin() – it conflicts with smartdisplay's I2C driver.
 *   Instead, INA226 sensors use Wire1 (I2C_NUM_1) on GPIO11(SDA)/GPIO13(SCL).
 *   This keeps touch and sensors on completely separate I2C buses.
 *   Wire1 transactions are protected by i2c_mutex.
 *
 * TOUCH:
 *   The debounced_touchpad_read callback wraps the smartdisplay driver,
 *   adding a state-machine filter with coordinate scaling.  Copied verbatim
 *   from the proven Touch project.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <math.h>
#include <Preferences.h>

#include <esp32_smartdisplay.h>
#include <lv_conf.h>
#include <lvgl.h>

#include "../lib/INA226/INA226.h"
#include "ui.h"

// =========================
// Sunton hardware mapping
// =========================
#define STM32_TO_ESP32_PIN 17  // ESP RX  <- STM32 TX
#define ESP32_TO_STM32_PIN 18  // ESP TX  -> STM32 RX

// Touch I2C (I2C_NUM_0) – managed by smartdisplay, DO NOT use Wire
#define TOUCH_SDA 19
#define TOUCH_SCL 20

// INA226 I2C (I2C_NUM_1) – separate bus via Wire1
#define INA_SDA 11
#define INA_SCL 13

#define BUTTON_PIN 0   // GPIO0 (BOOT button on Sunton board)
#define FET_CHARGE 12

HardwareSerial STM32Serial(2);

// =========================
// WiFi / TCP
// =========================
const char* ssid = "Jaime's Wi-Fi Network";
const char* password = "jackaustin";

WiFiServer server(8888);
WiFiClient client;

static uint32_t lastTcpRxMs = 0;
static const uint32_t TCP_IDLE_TIMEOUT_MS = 600000;

// =========================
// I2C / INA226
// =========================
SemaphoreHandle_t i2c_mutex = NULL;

INA226 inaPack(0x40);
INA226 inaCell1(0x41);
INA226 inaCell2(0x44);

float vpack = 0.0f;
float current_charge = 0.0f;
unsigned long last_battery_read = 0;
const unsigned long BATTERY_READ_INTERVAL = 100;

const float V_NODE1_SCALE = 1.000f;
const float V_NODE2_SCALE = 0.9860f;
const float VPACK_SCALE = 1.000f;

// Rev1 charge current correction: INA shunt sense path over-reads by ~10x
const float CHARGE_CURRENT_CORRECTION = 0.10f;

// =========================
// Charger thresholds
// =========================
const float CHARGE_LIMIT = 9.065f;
const float CHARGE_RESUME = 8.70f;
const float HARD_LIMIT = 9.10f;

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

uint8_t trigger_mode = 1;        // 1=pedal, 2=contact
uint8_t contact_hold_steps = 2;  // each step = 0.5s
bool contact_with_pedal = false; // require contact detection when pedal trigger

uint32_t weld_count = 0;        // running weld counter (incremented on WELD_DONE)

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
// READY heartbeat
// =========================
static bool uiConnected = false;
static uint32_t lastReadySentMs = 0;
static const uint32_t READY_PERIOD_MS = 1000;

// =========================
// Cell voltages (for display)
// =========================
static float disp_cell1 = 0, disp_cell2 = 0, disp_cell3 = 0;

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
void pollStm32Uart();
void processCommand(String cmd);
void ensureWiFiAndServer();
void setUiConnected(bool connected);
void serviceReadyHeartbeat();
void requestStm32Status();

void updateBattery();
void controlCharger();
void handleButton();

bool readCellsOnce(float& V1, float& V2, float& V3, float& C1, float& C2,
                   float& C3);
String buildStatus();

static void save_recipe_to_nvs();

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

static bool readBusV_0x02(uint8_t addr, float& v_out) {
    Wire1.beginTransmission(addr);
    Wire1.write(0x02);
    if (Wire1.endTransmission(false) != 0) return false;

    if (Wire1.requestFrom(addr, (uint8_t)2) != 2) return false;
    if (Wire1.available() < 2) return false;

    uint16_t raw = ((uint16_t)Wire1.read() << 8) | Wire1.read();
    v_out = raw * 0.00125f;
    return true;
}

float getDisplayCurrent() {
    bool charging_active = (digitalRead(FET_CHARGE) == HIGH);

    if (!welding_now && !charging_active) {
        return 0.0f;
    }

    return (fabs(current_charge) < 0.2f) ? 0.0f : current_charge;
}

// =========================
// Status builder
// =========================
String buildStatus() {
    bool enabled = stm_armed;
    bool charge_on = (digitalRead(FET_CHARGE) == HIGH);

    String state;
    if (!enabled) {
        state = "DISABLED";
    } else if (welding_now) {
        state = "WELDING";
    } else if (charge_on) {
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

    float ichg = getDisplayCurrent();
    float iweld = (fabs(stm_weld_current) < 0.002f) ? 0.0f : stm_weld_current;

    String status = "STATUS";
    status += ",enabled=" + String(enabled ? 1 : 0);
    status += ",state=" + state;
    status += ",vpack=" + String(vpack, 3);
    status += ",ichg=" + String(ichg, 3);
    status += ",iweld=" + String(iweld, 3);
    status += ",temp=" + t_str;
    status += ",cooldown_ms=" + String(cooldown_ms);
    status += ",pulse_ms=" + String(weld_d1);
    status += ",power_pct=" + String(weld_power_pct);
    status += ",preheat_en=" + String(preheat_enabled ? 1 : 0);
    status += ",preheat_ms=" + String(preheat_ms);
    status += ",preheat_pct=" + String(preheat_pct);
    status += ",preheat_gap_ms=" + String(preheat_gap_ms);
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

    return status;
}

// =========================
// TCP / UART helpers
// =========================
void sendToPi(const String& msg) {
    if (client && client.connected()) {
        client.print(msg);
        client.print("\n");
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

void requestStm32Status() { forwardToStm32("STATUS"); }

// =========================
// INA battery / cells
// =========================
bool readCellsOnce(float& V1, float& V2, float& V3, float& C1, float& C2,
                   float& C3) {
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    float v1_raw = NAN, v2_raw = NAN, vp_raw = NAN;
    bool ok = readBusV_0x02(0x41, v1_raw) && readBusV_0x02(0x44, v2_raw) &&
              readBusV_0x02(0x40, vp_raw);

    xSemaphoreGive(i2c_mutex);

    if (!ok) return false;

    float v_node1 = v1_raw * V_NODE1_SCALE;
    float v_node2 = v2_raw * V_NODE2_SCALE;
    float v_pack = vp_raw * VPACK_SCALE;

    V1 = v_node1;
    V2 = v_node2;
    V3 = v_pack;

    C1 = v_node1;
    C2 = v_node2 - v_node1;
    C3 = v_pack - v_node2;

    return true;
}

void updateBattery() {
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        float raw_vpack = inaPack.readBusVoltage();
        vpack = raw_vpack * VPACK_SCALE;

        Wire1.beginTransmission(0x40);
        Wire1.write(0x01);
        if (Wire1.endTransmission(false) == 0) {
            if (Wire1.requestFrom((uint8_t)0x40, (uint8_t)2) == 2 &&
                Wire1.available() >= 2) {
                int16_t raw_shunt = ((int16_t)Wire1.read() << 8) | Wire1.read();
                float shunt_voltage_v = raw_shunt * 0.000025f;
                float calculated_current = shunt_voltage_v / 0.0002f;

                current_charge =
                    (-calculated_current) * CHARGE_CURRENT_CORRECTION;

                if (fabs(current_charge) < 0.05f) current_charge = 0.0f;
                if (fabs(current_charge) > 50.0f) current_charge = 0.0f;
            }
        }

        xSemaphoreGive(i2c_mutex);
    }
}

void controlCharger() {
    if (!stm_armed) {
        digitalWrite(FET_CHARGE, LOW);
        return;
    }

    static int high_count = 0;

    if (vpack >= HARD_LIMIT) {
        high_count++;
    } else {
        high_count = 0;
    }

    if (high_count >= 2) {
        digitalWrite(FET_CHARGE, LOW);
        Serial.printf("HARD LIMIT - Charging OFF (vpack=%.3fV)\n", vpack);
        return;
    }

    if (vpack >= CHARGE_LIMIT) {
        digitalWrite(FET_CHARGE, LOW);
    } else if (vpack < CHARGE_RESUME) {
        digitalWrite(FET_CHARGE, HIGH);
    }
}

// =========================
// Front button
// =========================
void handleButton() {
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

    while (STM32Serial.available()) {
        char ch = (char)STM32Serial.read();

        if (ch == '\r') continue;

        if (ch == '\n') {
            stmLine.trim();

            if (stmLine.length() > 0) {
                if (!isNoisyKeepaliveLine(stmLine)) {
                    Serial.printf("[STM32->UART] %s\n", stmLine.c_str());
                }

                if (stmLine.startsWith("ACK,ARM,")) {
                    int v = stmLine.substring(8).toInt();
                    stm_armed = (v == 1);
                    stm_synced = true;
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("ACK,READY,")) {
                    int v = stmLine.substring(10).toInt();
                    stm_ready = (v == 1);

                } else if (stmLine.startsWith("ACK,SET_PULSE,")) {
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("ACK,SET_POWER,")) {
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("ACK,SET_PREHEAT,")) {
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("ACK,SET_TRIGGER_MODE,")) {
                    int iv = 0;
                    if (extractIntField(stmLine, "mode=", iv)) {
                        trigger_mode = (uint8_t)iv;
                    }
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("ACK,SET_CONTACT_HOLD,")) {
                    int iv = 0;
                    if (extractIntField(stmLine, "steps=", iv)) {
                        contact_hold_steps = (uint8_t)iv;
                    }
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("ACK,SET_CONTACT_WITH_PEDAL,")) {
                    // BUG FIX: STM32 ACK format is "ACK,SET_CONTACT_WITH_PEDAL,<0|1>"
                    // (bare integer, no field= prefix).  Previous code looked for
                    // "enabled=", "value=", "contact_with_pedal=" which never matched,
                    // so the ACK was silently ignored and ESP32 never synced from
                    // the STM32's confirmed state.
                    int v = stmLine.substring(strlen("ACK,SET_CONTACT_WITH_PEDAL,")).toInt();
                    contact_with_pedal = (v == 1);
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

                } else if (stmLine.startsWith("STATUS,")) {
                    int iv = 0;
                    float fv = NAN;

                    if (extractIntField(stmLine, "armed=", iv)) {
                        stm_armed = (iv == 1);
                    }

                    if (extractIntField(stmLine, "ready=", iv)) {
                        stm_ready = (iv == 1);
                    }

                    if (extractFloatField(stmLine, "temp=", fv)) {
                        temperature_c = fv;
                        last_temp_update_ms = millis();
                    }

                    if (extractFloatField(stmLine, "iwfilt=", fv)) {
                        stm_weld_current = fv;
                    } else if (extractFloatField(stmLine, "iweld_v=", fv)) {
                        stm_weld_current = fv;
                    }

                    if (extractIntField(stmLine, "trigger_mode=", iv)) {
                        trigger_mode = (uint8_t)iv;
                    }

                    if (extractIntField(stmLine, "contact_hold_steps=", iv)) {
                        contact_hold_steps = (uint8_t)iv;
                    }

                    if (extractIntField(stmLine, "contact_with_pedal=", iv)) {
                        contact_with_pedal = (iv == 1);
                    }

                    stm_last_status_ms = millis();
                    stm_synced = true;

                    static uint32_t lastStatusForwardMs = 0;
                    uint32_t now = millis();
                    if ((now - lastStatusForwardMs) >= 500) {
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
                    sendToPi(stmLine);
                    sendToPi(buildStatus());

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

                } else if (stmLine.startsWith("RXHEALTH,")) {
                    // keep local only
                }
            }

            stmLine = "";
        } else {
            if (stmLine.length() < 220) {
                stmLine += ch;
            } else {
                stmLine = "";
            }
        }
    }
}

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
        float V1, V2, V3, C1, C2, C3;
        if (readCellsOnce(V1, V2, V3, C1, C2, C3)) {
            char buf[160];
            snprintf(buf, sizeof(buf),
                     "CELLS,V1=%.3f,V2=%.3f,V3=%.3f,C1=%.3f,C2=%.3f,C3=%.3f",
                     V1, V2, V3, C1, C2, C3);
            sendToPi(String(buf));
        } else {
            sendToPi("ERR:CELLS_READ_FAIL");
        }
        return;
    }

    if (cmd.startsWith("ACK,")) {
        Serial.printf("[TCP] Ignoring UI ACK: %s\n", cmd.c_str());
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
        return;
    }

    if (cmd.startsWith("SET_POWER,")) {
        weld_power_pct = cmd.substring(10).toInt();
        if (weld_power_pct < 50) weld_power_pct = 50;
        if (weld_power_pct > 100) weld_power_pct = 100;

        forwardToStm32(cmd);
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
        return;
    }

    if (cmd.startsWith("SET_TRIGGER_MODE,")) {
        int mode = cmd.substring(strlen("SET_TRIGGER_MODE,")).toInt();
        if (mode < 1) mode = 1;
        if (mode > 2) mode = 2;
        trigger_mode = (uint8_t)mode;

        forwardToStm32(cmd);
        return;
    }

    if (cmd.startsWith("SET_CONTACT_HOLD,")) {
        int steps = cmd.substring(strlen("SET_CONTACT_HOLD,")).toInt();
        if (steps < 1) steps = 1;
        if (steps > 10) steps = 10;
        contact_hold_steps = (uint8_t)steps;

        forwardToStm32(cmd);
        return;
    }

    if (cmd.startsWith("SET_CONTACT_WITH_PEDAL,")) {
        int val = cmd.substring(strlen("SET_CONTACT_WITH_PEDAL,")).toInt();
        contact_with_pedal = (val == 1);

        forwardToStm32(cmd);
        return;
    }

    if (cmd == "RESET_WELD_COUNT") {
        weld_count = 0;
        Serial.println("[Weld] Counter reset to 0");
        sendToPi(buildStatus());
        return;
    }

    if (cmd.startsWith("ARM,")) {
        forwardToStm32(cmd);
        return;
    }

    if (cmd == "IWZERO" || cmd == "IWDBG") {
        forwardToStm32(cmd);
        return;
    }

    if (cmd == "FIRE") {
        sendToPi("ACK:FIRE_IGNORED");
        return;
    }

    if (cmd == "CHARGE_ON") {
        digitalWrite(FET_CHARGE, HIGH);
        Serial.println("Charging ON (manual)");
        sendToPi("ACK:CHARGE_ON");
        sendToPi(buildStatus());
        return;
    }

    if (cmd == "CHARGE_OFF") {
        digitalWrite(FET_CHARGE, LOW);
        Serial.println("Charging OFF (manual)");
        sendToPi("ACK:CHARGE_OFF");
        sendToPi(buildStatus());
        return;
    }

    if (cmd == "STATUS") {
        sendToPi(buildStatus());
        requestStm32Status();
        return;
    }

    sendToPi("ERR:UNKNOWN_CMD");
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
        requestStm32Status();
    } else {
        // ── TEMP-FIX: Flask disconnect no longer sends READY,0.
        //    READY,1 heartbeat now runs unconditionally so that STM32
        //    keeps broadcasting STATUS (with temperature) at all times.
        //    ARM,0 is still sent for safety when the TCP client drops.
        Serial.println("[READY] UI (TCP) disconnected -> ARM,0");
        forwardToStm32("ARM,0");
        stm_synced = false;
    }
}

void serviceReadyHeartbeat() {
    // ── TEMP-FIX: Always send READY,1 heartbeat so STM32 continuously
    //    broadcasts STATUS (temperature, armed state, etc.) regardless
    //    of whether a Flask/TCP client is connected.
    uint32_t now = millis();
    if ((now - lastReadySentMs) >= READY_PERIOD_MS) {
        forwardToStm32("READY,1");
        lastReadySentMs = now;
    }
}

// =========================
// WiFi / server maintenance
// =========================
void ensureWiFiAndServer() {
    static unsigned long last_attempt = 0;
    static bool server_started = false;

    if (WiFi.status() == WL_CONNECTED) {
        if (!server_started) {
            Serial.println("✅ WiFi connected (re) - IP: " +
                           WiFi.localIP().toString());
            Serial.println("Starting TCP server on port 8888...");
            server.begin();
            server.setNoDelay(true);
            server_started = true;
        }
        return;
    }

    server_started = false;
    server.stop();

    if (client) {
        client.stop();
    }

    setUiConnected(false);

    unsigned long now = millis();
    if (now - last_attempt > 5000) {
        last_attempt = now;
        Serial.println("⚠️ WiFi disconnected; retrying...");
        WiFi.disconnect(true);
        delay(200);
        WiFi.begin(ssid, password);
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
static void onRecipeApply(uint8_t mode, uint16_t d1, uint16_t gap1,
                          uint16_t d2, uint16_t gap2, uint16_t d3,
                          uint8_t power_pct_val,
                          bool preheat_en, uint16_t ph_ms,
                          uint8_t ph_pct, uint16_t ph_gap_ms) {
    char buf[80];

    // SET_PULSE – reuse existing processCommand path (updates locals + forwards to STM32)
    snprintf(buf, sizeof(buf), "SET_PULSE,%d,%d,%d,%d,%d,%d",
             mode, d1, gap1, d2, gap2, d3);
    processCommand(String(buf));

    // SET_POWER
    snprintf(buf, sizeof(buf), "SET_POWER,%d", power_pct_val);
    processCommand(String(buf));

    // SET_PREHEAT
    snprintf(buf, sizeof(buf), "SET_PREHEAT,%d,%d,%d,%d",
             preheat_en ? 1 : 0, ph_ms, ph_pct, ph_gap_ms);
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
    float val;
    switch (level) {
        case 0:  val = 0.3f; break;  // LOW
        case 1:  val = 0.6f; break;  // MED
        case 2:  val = 1.0f; break;  // HIGH
        default: val = 1.0f; break;
    }
    smartdisplay_lcd_set_backlight(val);
    Serial.printf("[Config] Brightness set to %s (%.1f)\n",
                  level == 0 ? "LOW" : level == 1 ? "MED" : "HIGH", (double)val);
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
    cfg.contact_hold_steps = prefs.getUChar("holdSteps", cfg.contact_hold_steps);
    contact_hold_steps = cfg.contact_hold_steps;  // sync global

    prefs.end();
    Serial.println("[Config] Loaded from NVS");
    return cfg;
}

static void save_recipe_to_nvs() {
    prefs.begin("weldrecipe", false);
    prefs.putUChar("mode",      weld_mode);
    prefs.putUShort("d1",       weld_d1);
    prefs.putUShort("gap1",     weld_gap1);
    prefs.putUShort("d2",       weld_d2);
    prefs.putUShort("gap2",     weld_gap2);
    prefs.putUShort("d3",       weld_d3);
    prefs.putUChar("power",     weld_power_pct);
    prefs.putBool("phEn",       preheat_enabled);
    prefs.putUShort("phMs",     preheat_ms);
    prefs.putUChar("phPct",     preheat_pct);
    prefs.putUShort("phGap",    preheat_gap_ms);
    prefs.end();
    Serial.println("[Config] Recipe saved to NVS");
}

static void load_recipe_from_nvs() {
    prefs.begin("weldrecipe", true);
    weld_mode       = prefs.getUChar("mode",   weld_mode);
    weld_d1         = prefs.getUShort("d1",    weld_d1);
    weld_gap1       = prefs.getUShort("gap1",  weld_gap1);
    weld_d2         = prefs.getUShort("d2",    weld_d2);
    weld_gap2       = prefs.getUShort("gap2",  weld_gap2);
    weld_d3         = prefs.getUShort("d3",    weld_d3);
    weld_power_pct  = prefs.getUChar("power",  weld_power_pct);
    preheat_enabled = prefs.getBool("phEn",    preheat_enabled);
    preheat_ms      = prefs.getUShort("phMs",  preheat_ms);
    preheat_pct     = prefs.getUChar("phPct",  preheat_pct);
    preheat_gap_ms  = prefs.getUShort("phGap", preheat_gap_ms);
    prefs.end();
    Serial.println("[Config] Recipe loaded from NVS");
}

// Callback from UI when config changes
static void onConfigChange(const ConfigState& cfg) {
    apply_brightness(cfg.brightness);
    // BUG FIX: Do NOT overwrite contact_with_pedal here.
    // CWP is managed exclusively by its own callback (onContactWithPedalChange)
    // and STM32 ACK/STATUS parsing.  Writing it here caused any unrelated
    // config change (brightness, hold-repeat, etc.) to silently overwrite the
    // authoritative CWP value with a potentially stale _cfg copy.

    // Sync contact_hold_steps from ConfigState to main.cpp global and STM32
    if (cfg.contact_hold_steps != contact_hold_steps) {
        contact_hold_steps = cfg.contact_hold_steps;
        char buf[40];
        snprintf(buf, sizeof(buf), "SET_CONTACT_HOLD,%d", (int)contact_hold_steps);
        processCommand(String(buf));
    }

    save_config_to_nvs(cfg);
    // Note: recipe persistence happens in onRecipeApply, not here
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

// =========================
// Setup
// =========================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("=== Spot Welder Merged Firmware (Milestone 1) ===");

    // --- UART to STM32 ---
    STM32Serial.begin(115200, SERIAL_8N1, STM32_TO_ESP32_PIN,
                      ESP32_TO_STM32_PIN);
    Serial.println("✅ STM32 UART bridge ready (Serial2 @ 115200)");

    // --- Front button ---
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // --- Charger FET ---
    pinMode(FET_CHARGE, OUTPUT);
    digitalWrite(FET_CHARGE, LOW);
    Serial.println("✅ FET_CHARGE initialized (GPIO 12 set LOW)");

    // --- I2C mutex ---
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        Serial.println("❌ Failed to create I2C mutex");
    } else {
        Serial.println("✅ I2C mutex created");
    }

    // ==========================================================
    // DISPLAY + TOUCH INIT
    // smartdisplay_init() claims I2C_NUM_0 for GT911 at 400kHz
    // DO NOT call Wire.begin() after this – it conflicts!
    // ==========================================================
    Serial.println("Initializing display (smartdisplay)...");
    smartdisplay_init();
    smartdisplay_lcd_set_backlight(1.0f);
    Serial.println("✅ Display initialized");

    // ==========================================================
    // Wire1 INIT  (I2C_NUM_1 – dedicated bus for INA226 sensors)
    // Completely separate from smartdisplay's I2C_NUM_0 bus.
    // ==========================================================
    Wire1.begin(INA_SDA, INA_SCL);
    Wire1.setClock(400000);
    Wire1.setTimeOut(1000);
    Serial.printf("✅ Wire1.begin(%d,%d) @ 400 kHz for INA226 sensors\n", INA_SDA, INA_SCL);

    // --- INA226 probing on Wire1 ---
    Serial.println("=== I2C device check on Wire1 (expected 0x40, 0x41, 0x44) ===");
    uint8_t addrs[] = {0x40, 0x41, 0x44};
    for (uint8_t addr : addrs) {
        Wire1.beginTransmission(addr);
        uint8_t err = Wire1.endTransmission();
        Serial.printf("Wire1 addr 0x%02X -> %s\n", addr,
                      (err == 0) ? "OK" : "NO ACK");
        delay(10);
    }

    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        if (inaPack.begin(&Wire1)) {
            Serial.println("✅ INA226 pack (0x40) initialized on Wire1");

            Wire1.beginTransmission(0x40);
            Wire1.write(0x00);
            Wire1.write(0x45);
            Wire1.write(0x27);
            Wire1.endTransmission();
            delay(10);

            uint16_t cal_value = 41967;
            Wire1.beginTransmission(0x40);
            Wire1.write(0x05);
            Wire1.write((cal_value >> 8) & 0xFF);
            Wire1.write(cal_value & 0xFF);
            Wire1.endTransmission();
            delay(10);

            if (inaCell1.begin(&Wire1)) {
                Serial.println("✅ INA226 cell1 (0x41) initialized on Wire1");
            } else {
                Serial.println("⚠️ INA226 cell1 (0x41) init failed on Wire1");
            }

            if (inaCell2.begin(&Wire1)) {
                Serial.println("✅ INA226 cell2 (0x44) initialized on Wire1");
            } else {
                Serial.println("⚠️ INA226 cell2 (0x44) init failed on Wire1");
            }

            updateBattery();
            controlCharger();
        } else {
            Serial.println("⚠️ INA226 pack init failed on Wire1");
        }

        xSemaphoreGive(i2c_mutex);
    }

    // ==========================================================
    // TOUCH INDEV OVERRIDE – wrap smartdisplay driver with filter
    // ==========================================================
    lv_indev_t* indev = lv_indev_get_next(nullptr);
    bool found_touch = false;

    while (indev != nullptr) {
        if (lv_indev_get_type(indev) == LV_INDEV_TYPE_POINTER) {
            Serial.println("✅ Touch indev found – wrapping with debounce filter");
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

    // --- Config: Load settings from NVS and apply ---
    {
        ConfigState cfg = load_config_from_nvs();

        // Apply brightness immediately
        apply_brightness(cfg.brightness);

        // Restore recipe from NVS if enabled, and push it to STM32
        if (cfg.load_last_on_boot) {
            load_recipe_from_nvs();
            Serial.println(
                "[Config] Load-last-on-boot: restored recipe from NVS");

            char buf[80];

            snprintf(buf, sizeof(buf), "SET_PULSE,%d,%d,%d,%d,%d,%d", weld_mode,
                     weld_d1, weld_gap1, weld_d2, weld_gap2, weld_d3);
            processCommand(String(buf));

            snprintf(buf, sizeof(buf), "SET_POWER,%d", weld_power_pct);
            processCommand(String(buf));

            snprintf(buf, sizeof(buf), "SET_PREHEAT,%d,%d,%d,%d",
                     preheat_enabled ? 1 : 0, preheat_ms, preheat_pct,
                     preheat_gap_ms);
            processCommand(String(buf));
        }

        // Sync global from loaded config
        contact_with_pedal = cfg.contact_with_pedal;

        // Push saved trigger/contact settings to STM32
        {
            char buf[40];

            snprintf(buf, sizeof(buf), "SET_TRIGGER_MODE,%d",
                     (int)trigger_mode);
            processCommand(String(buf));

            snprintf(buf, sizeof(buf), "SET_CONTACT_HOLD,%d",
                     (int)contact_hold_steps);
            processCommand(String(buf));

            snprintf(buf, sizeof(buf), "SET_CONTACT_WITH_PEDAL,%d",
                     contact_with_pedal ? 1 : 0);
            processCommand(String(buf));
        }

        // Push loaded config to UI
        ui_set_config_cb(onConfigChange);
        ui_load_config(cfg);
        Serial.println("✅ Config loaded and applied");
    }


    // --- Register remaining UI callbacks ---
    ui_set_trigger_source_cb(onTriggerSourceChange);
    ui_set_weld_count_reset_cb(onWeldCountReset);
    ui_set_contact_with_pedal_cb(onContactWithPedalChange);
    Serial.println("✅ All UI callbacks registered");

    // ==========================================================
    // WiFi  (non-blocking – removed blocking wait)
    // ==========================================================
    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(
        WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    // No blocking while-loop.  ensureWiFiAndServer() in loop() handles reconnect.

    // ── TEMP-FIX: Kick-start STM32 status broadcasting at boot.
    //    Send READY,1 so the STM32 begins its periodic STATUS reports
    //    (which include temperature).  Previously this only happened when
    //    a Flask/TCP client connected, leaving temp as ERR until then.
    //    A small delay ensures the STM32 UART is ready after power-on.
    delay(100);
    forwardToStm32("READY,1");
    lastReadySentMs = millis();
    requestStm32Status();
    Serial.println("=== Setup complete ===");
}

// =========================
// Loop
// =========================
static unsigned long lv_last_tick = 0;

void loop() {
    // --- LVGL tick + task handler ---
    unsigned long now_ms = millis();
    lv_tick_inc(now_ms - lv_last_tick);
    lv_last_tick = now_ms;
    lv_timer_handler();

    // --- Existing logic ---
    ensureWiFiAndServer();
    pollStm32Uart();
    serviceReadyHeartbeat();

    static uint32_t lastStatusReqMs = 0;
    uint32_t now = millis();

    // ── TEMP-FIX: Always poll STM32 for status (temperature, armed, etc.)
    //    regardless of TCP/Flask client connection.  Previously this was gated
    //    on uiConnected, which caused temperature to show ERR until Flask
    //    connected and triggered the first READY,1 / STATUS cycle.
    if ((now - lastStatusReqMs) >= 1000) {
        requestStm32Status();
        lastStatusReqMs = now;
    }

    if (WiFi.status() == WL_CONNECTED) {
        WiFiClient newClient = server.available();
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
            sendToPi(buildStatus());
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

    if (millis() - last_battery_read >= BATTERY_READ_INTERVAL) {
        last_battery_read = millis();
        updateBattery();
        controlCharger();

        static unsigned long last_cells_push = 0;
        if (millis() - last_cells_push >= 2000) {
            last_cells_push = millis();

            float V1, V2, V3, C1, C2, C3;
            if (readCellsOnce(V1, V2, V3, C1, C2, C3)) {
                disp_cell1 = C1;
                disp_cell2 = C2;
                disp_cell3 = C3;

                if (client && client.connected()) {
                    char buf[160];
                    snprintf(
                        buf, sizeof(buf),
                        "CELLS,V1=%.3f,V2=%.3f,V3=%.3f,C1=%.3f,C2=%.3f,C3=%.3f",
                        V1, V2, V3, C1, C2, C3);
                    sendToPi(String(buf));
                }
            }
        }
    }

    handleButton();

    // --- Drive UI touch-active flag from hardware touch driver state ---
    // This catches ALL touches (not just widget hits) and tells ui_update()
    // to completely pause label/widget redraws during active touch.
    {
        bool hw_touch = (touch_filter.state == TOUCH_PRESSED ||
                         touch_filter.state == TOUCH_PENDING_PRESS ||
                         touch_filter.state == TOUCH_PENDING_RELEASE);
        ui_set_touch_active(hw_touch);
    }

    // --- Update on-screen UI ---
    {
        WelderDisplayState ds;
        ds.pack_voltage = vpack;
        ds.temperature = temperature_c;
        ds.charger_current = getDisplayCurrent();
        ds.cell1_v = disp_cell1;
        ds.cell2_v = disp_cell2;
        ds.cell3_v = disp_cell3;
        ds.armed = stm_armed;
        ds.welding = welding_now;
        ds.charging = (digitalRead(FET_CHARGE) == HIGH);
        ds.weld_count = weld_count;
        ds.weld_mode     = weld_mode;
        ds.pulse_d1      = weld_d1;
        ds.pulse_gap1    = weld_gap1;
        ds.pulse_d2      = weld_d2;
        ds.pulse_gap2    = weld_gap2;
        ds.pulse_d3      = weld_d3;
        ds.power_pct     = weld_power_pct;
        ds.preheat_enabled = preheat_enabled;
        ds.preheat_ms    = preheat_ms;
        ds.preheat_pct   = preheat_pct;
        ds.preheat_gap_ms = preheat_gap_ms;
        ds.trigger_mode  = trigger_mode;
        ds.contact_hold_steps = contact_hold_steps;
        ui_update(ds);
    }

    delay(5);
}
