// wifi_bridge.h
// WiFi provisioning + TCP bridge for the ESP32-P4 spot-welder display.
//
// Ported from the original Sunton ESP32-S3 Arduino firmware (captive-portal
// provisioning). Behaviour mirror:
//   - Boot: load saved credentials from NVS ("wificfg" namespace).
//   - No creds OR STA connect times out -> bring up a soft-AP ("SpotWelder-XXXX")
//     + captive portal + on-screen QR code so the user can pick a network.
//   - Creds present -> connect in STA mode; once an IP is acquired, start the
//     TCP bridge (port 8888) used by the Flask web dashboard, plus mDNS so the
//     device answers at "spotwelder.local".
//   - "Reconfigure WiFi" (Setup tab) re-enters the AP portal.
//
// On the ESP32-P4 the radio itself lives on the onboard ESP32-C6 and is reached
// over SDIO via esp_hosted / esp_wifi_remote, but the esp_wifi API used here is
// identical to a native-WiFi target.

#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Invoked (from the TCP bridge task) for every newline-delimited command line
// received from the Flask web client. The implementation should forward the
// line to the STM32. May be NULL.
typedef void (*wifi_bridge_cmd_cb_t)(const char *line);

// Initialise NVS + WiFi, load saved credentials and start the provisioning
// state machine and the TCP bridge. Non-blocking: spawns its own tasks and
// returns immediately. Safe to call once from app_main() after the UI is up
// (it calls ui_set_wifi_info / ui_show_wifi_setup as state changes).
void wifi_bridge_start(wifi_bridge_cmd_cb_t cmd_cb);

// Tear down the current STA connection and bring up the AP + captive portal
// (Setup tab "Reconfigure WiFi" button).
void wifi_bridge_reconfigure(void);

// Erase the saved WiFi credentials from NVS. Caller typically reboots after.
void wifi_bridge_factory_reset(void);

// Push one line (STATUS / EVENT / WAVEFORM / ...) to the connected Flask
// client. Thread-safe; silently drops the line if no client is connected.
void wifi_bridge_broadcast(const char *line);

#ifdef __cplusplus
}
#endif
