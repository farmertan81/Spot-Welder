// arduino_compat.h
// Minimal Arduino-compatibility shim so the ported ui.cpp (originally written
// for the Arduino/PlatformIO Sunton ESP32-S3 build) compiles unchanged under
// ESP-IDF on the ESP32-P4. Provides only what ui.cpp actually uses:
//   - millis()  -> esp_timer based monotonic milliseconds
//   - Serial.printf()/println() -> routed to ESP_LOG / printf
#pragma once

#include <stdio.h>
#include <stdint.h>
#include "esp_timer.h"

#ifdef __cplusplus

// Monotonic milliseconds since boot (matches Arduino millis()).
static inline uint32_t millis() {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// Tiny stand-in for the Arduino HardwareSerial debug object. ui.cpp only uses
// Serial.printf() and Serial.println() for diagnostics, so we forward them to
// the standard console (printf), which ESP-IDF routes to the monitor UART.
struct _ArduinoSerialShim {
    template <typename... Args>
    int printf(const char* fmt, Args... args) {
        return ::printf(fmt, args...);
    }
    void println(const char* s) { ::printf("%s\n", s); }
    void println() { ::printf("\n"); }
    void print(const char* s) { ::printf("%s", s); }
};

static _ArduinoSerialShim Serial;

#endif  // __cplusplus
