#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <math.h>

#include "../lib/INA226/INA226.h"

// ---- I²C Mutex ----
SemaphoreHandle_t i2c_mutex = NULL;

// ---- Pin Definitions ----
#define FET_CHARGE 5
#define FET_WELD 4   // SAFETY: held LOW, ESP32 no longer welds
#define PEDAL_PIN 6  // UNUSED now (pedal moves to STM32)
#define I2C_SDA 2
#define I2C_SCL 3
#define THERM_PIN 7
#define LED_PIN 21
#define BUTTON_PIN 1

// ---- STM32 UART bridge pins ----
#define STM32_TO_ESP32_PIN 11  // ESP32 RX  <- STM32 TX (PA9)
#define ESP32_TO_STM32_PIN 12  // ESP32 TX  -> STM32 RX (PA10)
HardwareSerial STM32Serial(2);

// ---- WiFi / TCP Settings ----
const char* ssid = "Jaime's Wi-Fi Network";
const char* password = "jackaustin";

WiFiServer server(8888);
WiFiClient client;

static uint32_t lastTcpRxMs = 0;
static const uint32_t TCP_IDLE_TIMEOUT_MS = 600000;  // 10min

// ---- INA226 (cell voltage only) ----
INA226 ina(0x40);       // Pack voltage
INA226 inaCell1(0x41);  // Node 1
INA226 inaCell2(0x44);  // Node 2

// ---- LED & Button ----
Adafruit_NeoPixel led(1, LED_PIN, NEO_RGB + NEO_KHZ800);
volatile bool welding_now = false;  // driven by STM32 events

// ---- Battery monitoring ----
float vpack = 0.0f;
float current_charge = 0.0f;
unsigned long last_battery_read = 0;
const unsigned long BATTERY_READ_INTERVAL = 100;

const float V_NODE1_SCALE = 1.000f;
const float V_NODE2_SCALE = 0.9860f;
const float VPACK_SCALE = 1.000f;

// ---- Battery thresholds ----
const float CHARGE_LIMIT = 9.065f;
const float CHARGE_RESUME = 8.70f;
const float HARD_LIMIT = 9.10f;

// ---- Weld settings (forwarded to STM32, mirrored locally only) ----
uint8_t weld_mode = 1;  // 1=single, 2=double, 3=triple
uint16_t weld_d1 = 5;
uint16_t weld_gap1 = 0;
uint16_t weld_d2 = 0;
uint16_t weld_gap2 = 0;
uint16_t weld_d3 = 0;
uint8_t weld_power_pct = 100;  // 50-100%
bool preheat_enabled = false;
uint16_t preheat_ms = 20;
uint8_t preheat_pct = 30;
uint16_t preheat_gap_ms = 3;

uint16_t weld_duration_ms = 5;  // legacy: still shown in STATUS
unsigned long last_weld_time = 0;
const unsigned long WELD_COOLDOWN = 500;

// ---- Thermistor / status mirrored from STM32 ----
float temperature_c = NAN;
static uint32_t last_temp_update_ms = 0;

// ---- Forward declarations ----
void updateBattery();
void controlCharger();
void processCommand(String cmd);
String buildStatus();
void sendToPi(const String& msg);
void forwardToStm32(const String& line);
void pollStm32Uart();
void updateLED();
void handleButton();
void requestStm32Status();
void setUiConnected(bool connected);
void serviceReadyHeartbeat();
void ensureWiFiAndServer();

// ---- STM truth (source of truth for UI) ----
static bool stm_armed = false;
static bool stm_ready = false;
static uint32_t stm_last_status_ms = 0;
static bool stm_synced = false;
static float stm_weld_current = 0.0f;

// ---- STM32 READY heartbeat ----
static bool uiConnected = false;
static uint32_t lastReadySentMs = 0;
static const uint32_t READY_PERIOD_MS = 250;

// ---- Cell reading helpers ----
static bool readBusV_0x02(uint8_t addr, float& v_out) {
    Wire.beginTransmission(addr);
    Wire.write(0x02);  // Bus Voltage register
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom(addr, (uint8_t)2) != 2) return false;
    if (Wire.available() < 2) return false;

    uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
    v_out = raw * 0.00125f;  // 1.25mV/bit
    return true;
}

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

float getDisplayCurrent() {
    bool charging_active = (digitalRead(FET_CHARGE) == HIGH);

    if (!welding_now && !charging_active) {
        return 0.0f;
    }

    return (fabs(current_charge) < 0.2f) ? 0.0f : current_charge;
}

// ---- small parsing helpers ----
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

// ---- Helper: Build STATUS line ----
String buildStatus() {
    bool charge_on = (digitalRead(FET_CHARGE) == HIGH);
    bool enabled = stm_armed;

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

    String status = "STATUS";
    status += ",enabled=" + String(enabled ? 1 : 0);
    status += ",state=" + state;
    status += ",vpack=" + String(vpack, 3);
    float ichg = getDisplayCurrent();
    float iweld = (fabs(stm_weld_current) < 0.2f) ? 0.0f : stm_weld_current;

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

    return status;
}

// ---- Helper: Send line to Pi ----
void sendToPi(const String& msg) {
    if (client && client.connected()) {
        client.print(msg);
        client.print("\n");
        Serial.printf("[TCP] TX: %s\n", msg.c_str());
    }
}

// ---- UART forward helper ----
// FIX: Added flush() to guarantee bytes are physically sent before returning.
//      Without flush(), bytes can sit in the ESP32 TX FIFO/buffer and arrive
//      late or get coalesced — especially problematic for time-critical
//      commands like ARM,1 that the STM32 must process immediately.
void forwardToStm32(const String& line) {
    String out = line;
    out.trim();
    if (out.length() == 0) return;

    // Send command + CRLF line ending (STM32 parser expects \n or \r)
    STM32Serial.print(out);
    STM32Serial.print("\r\n");
    STM32Serial.flush();  // FIX: Block until all bytes physically transmitted

    if (!out.startsWith("READY,")) {
        Serial.printf("[UART->STM32] %s (len=%d)\n", out.c_str(), out.length());
    }
}

void requestStm32Status() { forwardToStm32("STATUS"); }

// ---- STM32 UART RX -> Pi relay ----
static inline bool isNoisyKeepaliveLine(const String& s) {
    return s.startsWith("ACK,READY");
}

void pollStm32Uart() {
    static String stmLine;

    while (STM32Serial.available()) {
        char ch = (char)STM32Serial.read();

        if (ch == '\r') {
            continue;
        }

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

                    if (extractFloatField(stmLine, "i=", fv)) {
                        stm_weld_current = fv;
                    }

                    stm_last_status_ms = millis();
                    stm_synced = true;

                    static uint32_t lastStatusForwardMs = 0;
                    uint32_t now = millis();
                    if ((now - lastStatusForwardMs) >= 500) {
                        sendToPi(buildStatus());
                        lastStatusForwardMs = now;
                    }
                }
            }

            stmLine = "";
        } else {
            if (stmLine.length() < 220) {
                stmLine += ch;
            } else {
                // prevent runaway line growth
                stmLine = "";
            }
        }
    }
}
// ---- Battery / Charger ----
void updateBattery() {
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        float raw_vpack = ina.readBusVoltage();
        vpack = raw_vpack * VPACK_SCALE;

        // Read shunt voltage register directly from INA at 0x40
        Wire.beginTransmission(0x40);
        Wire.write(0x01);  // shunt voltage reg
        if (Wire.endTransmission(false) == 0) {
            if (Wire.requestFrom((uint8_t)0x40, (uint8_t)2) == 2 &&
                Wire.available() >= 2) {
                int16_t raw_shunt = ((int16_t)Wire.read() << 8) | Wire.read();
                float shunt_voltage_v = raw_shunt * 0.0000025f;
                float calculated_current = shunt_voltage_v / 0.0002f;
                current_charge = -calculated_current;
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
        Serial.printf("⚠️ HARD LIMIT - Charging OFF (vpack=%.2fV)\n", vpack);
        return;
    }

    if (vpack >= CHARGE_LIMIT) {
        digitalWrite(FET_CHARGE, LOW);
    } else if (vpack < CHARGE_RESUME) {
        digitalWrite(FET_CHARGE, HIGH);
    }
}

// ---- Command Parser ----
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
        weld_duration_ms = weld_d1;

        Serial.printf("✅ Pulse: mode=%d d1=%d gap1=%d d2=%d gap2=%d d3=%d\n",
                      weld_mode, weld_d1, weld_gap1, weld_d2, weld_gap2,
                      weld_d3);

        forwardToStm32(cmd);
        return;
    }

    if (cmd.startsWith("SET_POWER,")) {
        weld_power_pct = cmd.substring(10).toInt();
        if (weld_power_pct < 50) weld_power_pct = 50;
        if (weld_power_pct > 100) weld_power_pct = 100;

        Serial.printf("✅ Weld power set to %d%%\n", weld_power_pct);

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

        Serial.printf("✅ Preheat: %s, %dms, %d%%, gap=%dms\n",
                      preheat_enabled ? "ON" : "OFF", preheat_ms, preheat_pct,
                      preheat_gap_ms);

        forwardToStm32(cmd);
        return;
    }

    if (cmd.startsWith("ARM,")) {
        // Forward only; real success comes back from STM32 as ACK,ARM,x
        forwardToStm32(cmd);
        return;
    }

    if (cmd == "FIRE") {
        Serial.println("⚠️ FIRE ignored (STM32 pedal-only mode)");
        sendToPi("ACK:FIRE_IGNORED");
        return;
    }

    if (cmd == "CHARGE_ON") {
        digitalWrite(FET_CHARGE, HIGH);
        Serial.println("✅ Charging ON (manual)");
        sendToPi("ACK:CHARGE_ON");
        sendToPi(buildStatus());
        return;
    }

    if (cmd == "CHARGE_OFF") {
        digitalWrite(FET_CHARGE, LOW);
        Serial.println("✅ Charging OFF (manual)");
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

// ---- LED update ----
void updateLED() {
    if (!stm_armed) {
        led.setPixelColor(0, led.Color(255, 0, 0));
    } else if (welding_now) {
        led.setPixelColor(0, led.Color(255, 255, 255));
    } else if (digitalRead(FET_CHARGE)) {
        led.setPixelColor(0, led.Color(0, 0, 255));
    } else {
        led.setPixelColor(0, led.Color(0, 255, 0));
    }
    led.show();
}

// ---- Button handler ----
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
            Serial.println("🔘 Long press → entering deep sleep");
            waiting_for_double = false;
            esp_deep_sleep_start();
        } else if (press_duration >= 50) {
            if (waiting_for_double && time_since_last <= 500) {
                Serial.println("🔘 Double tap → RESETTING ESP32...");
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

// ---- READY heartbeat impl ----
void setUiConnected(bool connected) {
    if (connected == uiConnected) return;
    uiConnected = connected;

    if (uiConnected) {
        Serial.println("[READY] UI connected -> READY,1 (no auto-arm)");
        forwardToStm32("READY,1");
        lastReadySentMs = millis();
        stm_synced = false;
        requestStm32Status();
    } else {
        Serial.println("[READY] UI disconnected -> READY,0 + ARM,0");
        forwardToStm32("READY,0");
        forwardToStm32("ARM,0");
        lastReadySentMs = 0;
        stm_synced = false;
    }
}

void serviceReadyHeartbeat() {
    if (!uiConnected) return;

    uint32_t now = millis();

    if ((now - lastReadySentMs) >= READY_PERIOD_MS) {
        forwardToStm32("READY,1");
        lastReadySentMs = now;
    }
}

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

// ---- Setup ----
void setup() {
    Serial.begin(115200);
    delay(500);

    STM32Serial.begin(115200, SERIAL_8N1, STM32_TO_ESP32_PIN,
                      ESP32_TO_STM32_PIN);
    Serial.println("✅ STM32 UART bridge ready (Serial2 @ 115200)");

    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        Serial.println("❌ Failed to create I²C mutex!");
    } else {
        Serial.println("✅ I²C mutex created");
    }

    Serial.println(
        "\n=== ESP32 Weld Controller - GATEWAY MODE (STM32 weld) ===");

    pinMode(FET_WELD, OUTPUT);
    digitalWrite(FET_WELD, LOW);

    pinMode(PEDAL_PIN, INPUT_PULLUP);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);

    led.begin();
    led.setBrightness(50);
    led.setPixelColor(0, led.Color(255, 255, 0));
    led.show();

    analogReadResolution(12);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(50000);
    Wire.setTimeOut(1000);

    Serial.println("\n=== I²C Device Check (expected 0x40, 0x41, 0x44) ===");
    uint8_t addrs[] = {0x40, 0x41, 0x44};
    for (uint8_t addr : addrs) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        Serial.printf("I2C addr 0x%02X -> %s\n", addr,
                      (err == 0) ? "OK" : "NO ACK");
        delay(10);
    }

    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        if (ina.begin(&Wire)) {
            Serial.println("✅ INA226 (pack) initialized");

            // Config register
            Wire.beginTransmission(0x40);
            Wire.write(0x00);
            Wire.write(0x45);
            Wire.write(0x27);
            Wire.endTransmission();
            delay(10);

            // Calibration register
            uint16_t cal_value = 41967;
            Wire.beginTransmission(0x40);
            Wire.write(0x05);
            Wire.write((cal_value >> 8) & 0xFF);
            Wire.write(cal_value & 0xFF);
            Wire.endTransmission();
            delay(10);

            if (inaCell1.begin(&Wire)) {
                Serial.println("✅ INA226 cell1 (0x41) initialized");
            } else {
                Serial.println("⚠️ INA226 cell1 (0x41) init failed");
            }

            if (inaCell2.begin(&Wire)) {
                Serial.println("✅ INA226 cell2 (0x44) initialized");
            } else {
                Serial.println("⚠️ INA226 cell2 (0x44) init failed");
            }

            updateBattery();
            controlCharger();
        } else {
            Serial.println("⚠️ INA226 init failed - charging disabled");
        }
        xSemaphoreGive(i2c_mutex);
    } else {
        Serial.println("❌ Failed to acquire I²C mutex in setup!");
    }

    WiFi.mode(WIFI_STA);
    esp_wifi_set_protocol(
        WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

    Serial.print("Connecting to: ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;
        if (attempts % 10 == 0) {
            Serial.print(" [");
            Serial.print((int)WiFi.status());
            Serial.print("] ");
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi CONNECTED!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\n❌ WiFi FAILED!");
    }

    pinMode(FET_CHARGE, OUTPUT);
    digitalWrite(FET_CHARGE, LOW);
    Serial.println("✅ FET_CHARGE initialized (GPIO 5 set LOW)");

    Serial.println("\n=== GATEWAY READY (STM32 does weld timing) ===");
    delay(500);

    // Intentionally DO NOT push defaults into STM32 at boot.
    // STM32 is the source of truth; UI/user commands update settings
    // explicitly.
    requestStm32Status();
}

// ---- Loop ----
void loop() {
    ensureWiFiAndServer();
    pollStm32Uart();
    serviceReadyHeartbeat();

    static uint32_t lastStatusReqMs = 0;
    uint32_t now = millis();

    if (uiConnected && (now - lastStatusReqMs) >= 1000) {
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

        static unsigned long last_print = 0;
        if (millis() - last_print >= 2000) {
            last_print = millis();

            bool charging_active = (digitalRead(FET_CHARGE) == HIGH);

            Serial.printf("📊 Vpack=%.2fV I=%.2fA %s", vpack,
                          getDisplayCurrent(),
                          charging_active ? "⚡CHARGING" : "⏸️IDLE");

            if (isfinite(temperature_c)) {
                Serial.printf("  Temp=%.1fC\n", temperature_c);
            } else {
                Serial.println();
            }

            float V1, V2, V3, C1, C2, C3;
            if (readCellsOnce(V1, V2, V3, C1, C2, C3)) {
                char buf[160];
                snprintf(
                    buf, sizeof(buf),
                    "CELLS,V1=%.3f,V2=%.3f,V3=%.3f,C1=%.3f,C2=%.3f,C3=%.3f", V1,
                    V2, V3, C1, C2, C3);
                sendToPi(String(buf));
            }
        }
    }

    handleButton();
    updateLED();

    delay(10);
}