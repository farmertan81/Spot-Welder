#include <Arduino.h>

// ===================== Pin Map (STM32) =====================
// NOTE: moved weld output to a "known-good" PWM pin (TIM1_CH1).
#define WELD_OUT_PIN PA8   // Weld gate/driver control output (active HIGH)
#define PEDAL_IN_PIN PB12  // Foot pedal input (active LOW, INPUT_PULLUP)

// ===================== Limits / Timing =====================
static const uint32_t WELD_COOLDOWN_MS = 500;
static const uint16_t MAX_WELD_MS = 200;
static const uint32_t PEDAL_DEBOUNCE_MS = 40;
static const uint32_t BOOT_INHIBIT_MS = 5000;

// Arm timeout (0 disables)
static const uint32_t ARM_TIMEOUT_MS = 0;

// ===================== PWM Settings ========================
static const uint32_t PWM_FREQ_HZ = 10000;
static const uint16_t PWM_MAX = 1023;  // 10-bit

// ===================== Weld Params (defaults) ==============
static volatile uint8_t weld_mode = 1;  // 1=single,2=double,3=triple
static volatile uint16_t weld_d1_ms = 10;
static volatile uint16_t weld_gap1_ms = 0;
static volatile uint16_t weld_d2_ms = 0;
static volatile uint16_t weld_gap2_ms = 0;
static volatile uint16_t weld_d3_ms = 0;

static volatile uint8_t weld_power_pct = 100;  // 50-100% (clamped)
static volatile bool preheat_enabled = false;
static volatile uint16_t preheat_ms = 20;
static volatile uint8_t preheat_pct = 30;  // 0-100% (clamped)
static volatile uint16_t preheat_gap_ms = 3;

// ===================== State ===============================
static volatile bool welding_now = false;
static uint32_t last_weld_ms = 0;

static bool armed = true;
static uint32_t armed_until_ms = 0;
static uint32_t boot_ms = 0;

// UART RX line buffer
static String rxLine;

// Pedal debounce state
static int pedal_last_raw = HIGH;
static int pedal_stable = HIGH;
static uint32_t pedal_last_change_ms = 0;

// ===================== Helpers =============================
static void uartSend(const String& s) {
    Serial1.println(s);  // to ESP32
    Serial.println(s);   // USB debug echo
}

static void clampParams() {
    if (weld_mode < 1) weld_mode = 1;
    if (weld_mode > 3) weld_mode = 3;

    if (weld_d1_ms > MAX_WELD_MS) weld_d1_ms = MAX_WELD_MS;
    if (weld_d2_ms > MAX_WELD_MS) weld_d2_ms = MAX_WELD_MS;
    if (weld_d3_ms > MAX_WELD_MS) weld_d3_ms = MAX_WELD_MS;

    if (weld_power_pct < 50) weld_power_pct = 50;
    if (weld_power_pct > 100) weld_power_pct = 100;

    if (preheat_pct > 100) preheat_pct = 100;
    if (preheat_ms > MAX_WELD_MS) preheat_ms = MAX_WELD_MS;
}

static void applyArmTimeout() {
    if (ARM_TIMEOUT_MS == 0) return;
    if (!armed) return;
    if (armed_until_ms == 0) return;
    if ((int32_t)(millis() - armed_until_ms) >= 0) {
        armed = false;
        armed_until_ms = 0;
        uartSend("EVENT,ARM_TIMEOUT");
    }
}

static uint16_t pctToDuty(uint8_t pct) {
    if (pct >= 100) return PWM_MAX;
    return (uint16_t)((uint32_t)pct * PWM_MAX / 100U);
}

static inline void pwmOff() {
    analogWrite(WELD_OUT_PIN, 0);
    digitalWrite(WELD_OUT_PIN, LOW);
}

static inline void pwmOnDuty(uint16_t duty) {
    if (duty > PWM_MAX) duty = PWM_MAX;
    analogWrite(WELD_OUT_PIN, duty);
}

static inline void delayMsExact(uint16_t ms) {
    if (ms == 0) return;
    if (ms > MAX_WELD_MS) ms = MAX_WELD_MS;
    delayMicroseconds((uint32_t)ms * 1000UL);
}

static void doPulseMsPwm(uint16_t ms, uint16_t duty) {
    if (ms == 0) return;
    if (ms > MAX_WELD_MS) ms = MAX_WELD_MS;

    pwmOnDuty(duty);
    delayMsExact(ms);
    pwmOff();
}

static void fireRecipe() {
    uint32_t now_ms = millis();

    if (now_ms - boot_ms < BOOT_INHIBIT_MS) {
        uartSend("DENY,BOOT_INHIBIT,ms=" +
                 String(BOOT_INHIBIT_MS - (now_ms - boot_ms)));
        return;
    }

    applyArmTimeout();
    if (!armed) {
        uartSend("DENY,NOT_ARMED");
        return;
    }
    if (welding_now) {
        uartSend("DENY,ALREADY_WELDING");
        return;
    }

    uint32_t since = now_ms - last_weld_ms;
    if (since < WELD_COOLDOWN_MS) {
        uartSend("DENY,COOLDOWN,ms=" + String(WELD_COOLDOWN_MS - since));
        return;
    }

    clampParams();

    welding_now = true;
    uartSend("EVENT,WELD_START");

    // Deadtime / ensure off
    pwmOff();
    delayMicroseconds(2000);

    uint32_t t0 = micros();

    // Preheat
    if (preheat_enabled && preheat_ms > 0) {
        doPulseMsPwm(preheat_ms, pctToDuty(preheat_pct));
        if (preheat_gap_ms > 0) {
            pwmOff();
            delayMsExact(preheat_gap_ms);
        }
    }

    // Main
    uint16_t mainDuty = pctToDuty(weld_power_pct);

    if (weld_mode >= 1) {
        doPulseMsPwm(weld_d1_ms, mainDuty);
    }
    if (weld_mode >= 2) {
        if (weld_gap1_ms) delayMsExact(weld_gap1_ms);
        doPulseMsPwm(weld_d2_ms, mainDuty);
    }
    if (weld_mode >= 3) {
        if (weld_gap2_ms) delayMsExact(weld_gap2_ms);
        doPulseMsPwm(weld_d3_ms, mainDuty);
    }

    uint32_t total_us = micros() - t0;

    pwmOff();
    welding_now = false;
    last_weld_ms = millis();

    uartSend("EVENT,WELD_DONE,total_us=" + String(total_us) + ",mode=" +
             String((int)weld_mode) + ",d1=" + String((int)weld_d1_ms) +
             ",gap1=" + String((int)weld_gap1_ms) + ",d2=" +
             String((int)weld_d2_ms) + ",gap2=" + String((int)weld_gap2_ms) +
             ",d3=" + String((int)weld_d3_ms) +
             ",power_pct=" + String((int)weld_power_pct) +
             ",preheat_en=" + String(preheat_enabled ? 1 : 0) +
             ",preheat_ms=" + String((int)preheat_ms) +
             ",preheat_pct=" + String((int)preheat_pct) +
             ",preheat_gap_ms=" + String((int)preheat_gap_ms));
}

static void handleCmd(const String& cmd) {
    if (cmd.startsWith("ARM,")) {
        int v = cmd.substring(4).toInt();
        if (v == 1) {
            armed = true;
            armed_until_ms =
                (ARM_TIMEOUT_MS == 0) ? 0 : (millis() + ARM_TIMEOUT_MS);
            uartSend("ACK,ARM,1");
        } else {
            armed = false;
            armed_until_ms = 0;
            uartSend("ACK,ARM,0");
        }
        return;
    }

    if (cmd.startsWith("SET_PULSE,")) {
        int v_mode = 1, v_d1 = 0, v_gap1 = 0, v_d2 = 0, v_gap2 = 0, v_d3 = 0;
        int n = sscanf(cmd.c_str(), "SET_PULSE,%d,%d,%d,%d,%d,%d", &v_mode,
                       &v_d1, &v_gap1, &v_d2, &v_gap2, &v_d3);
        if (n < 2) {
            uartSend("DENY,BAD_SET_PULSE");
            return;
        }

        weld_mode = (uint8_t)v_mode;
        weld_d1_ms = (uint16_t)v_d1;
        weld_gap1_ms = (uint16_t)v_gap1;
        weld_d2_ms = (uint16_t)v_d2;
        weld_gap2_ms = (uint16_t)v_gap2;
        weld_d3_ms = (uint16_t)v_d3;

        clampParams();
        uartSend("ACK,SET_PULSE,mode=" + String((int)weld_mode));
        return;
    }

    if (cmd.startsWith("SET_POWER,")) {
        weld_power_pct = (uint8_t)cmd.substring(10).toInt();
        clampParams();
        uartSend("ACK,SET_POWER,pct=" + String((int)weld_power_pct));
        return;
    }

    if (cmd.startsWith("SET_PREHEAT,")) {
        int idx = 12;
        int c1 = cmd.indexOf(',', idx);
        int c2 = cmd.indexOf(',', c1 + 1);
        int c3 = cmd.indexOf(',', c2 + 1);
        if (c1 < 0 || c2 < 0 || c3 < 0) {
            uartSend("DENY,BAD_SET_PREHEAT");
            return;
        }

        preheat_enabled = (cmd.substring(idx, c1).toInt() == 1);
        preheat_ms = (uint16_t)cmd.substring(c1 + 1, c2).toInt();
        preheat_pct = (uint8_t)cmd.substring(c2 + 1, c3).toInt();
        preheat_gap_ms = (uint16_t)cmd.substring(c3 + 1).toInt();

        clampParams();
        uartSend("ACK,SET_PREHEAT,en=" + String(preheat_enabled ? 1 : 0));
        return;
    }

    if (cmd == "STATUS") {
        int32_t cd =
            (int32_t)WELD_COOLDOWN_MS - (int32_t)(millis() - last_weld_ms);
        if (cd < 0) cd = 0;

        uartSend(String("STATUS,armed=") + (armed ? "1" : "0") +
                 ",cooldown_ms=" + String((int)cd) +
                 ",welding=" + String(welding_now ? 1 : 0) +
                 ",mode=" + String((int)weld_mode) +
                 ",power_pct=" + String((int)weld_power_pct) +
                 ",preheat_en=" + String(preheat_enabled ? 1 : 0));
        return;
    }
}

static void pollUart() {
    while (Serial1.available()) {
        char ch = (char)Serial1.read();
        if (ch == '\r') continue;

        if (ch == '\n') {
            rxLine.trim();
            if (rxLine.length()) handleCmd(rxLine);
            rxLine = "";
        } else {
            if (rxLine.length() < 220) rxLine += ch;
        }
    }
}

static void pollPedal() {
    int raw = digitalRead(PEDAL_IN_PIN);
    uint32_t now = millis();

    if (raw != pedal_last_raw) {
        pedal_last_change_ms = now;
        pedal_last_raw = raw;
    }

    if ((now - pedal_last_change_ms) >= PEDAL_DEBOUNCE_MS) {
        if (raw != pedal_stable) {
            int prev = pedal_stable;
            pedal_stable = raw;
            if (prev == HIGH && pedal_stable == LOW) {
                fireRecipe();
            }
        }
    }
}

// ===================== Arduino entry ======================
void setup() {
    pinMode(PEDAL_IN_PIN, INPUT_PULLUP);
    pinMode(WELD_OUT_PIN, OUTPUT);
    digitalWrite(WELD_OUT_PIN, LOW);

    Serial.begin(115200);
    Serial1.begin(115200);
    boot_ms = millis();

    // Debounce init
    int r = digitalRead(PEDAL_IN_PIN);
    pedal_last_raw = r;
    pedal_stable = r;
    pedal_last_change_ms = boot_ms;

    // PWM setup supported by your core:
    analogWriteResolution(10);
    analogWriteFrequency(PWM_FREQ_HZ);

    pwmOff();
    uartSend("BOOT,STM32_WELD_BRAIN_PWM_READY");
}

void loop() {
    pollUart();
    pollPedal();
    applyArmTimeout();
}