// ============================================================
//  STM32 remote firmware flashing (AN3155 ROM bootloader over UART)
// ============================================================
// Programs the STM32G474 weld controller with a new firmware image that was
// delivered wirelessly to the ESP32-P4 (see POST /stm32 in wifi_bridge.cpp).
// The ESP32 drives the STM32 into its factory ROM bootloader (hardware BOOT0
// strap + a "BOOTLOADER" command the running app obeys), then programs it over
// the existing UART link using ST's AN3155 protocol, and finally reboots so
// both sides come up on a clean, re-synced application link.
#ifndef STM32_FLASH_H
#define STM32_FLASH_H

#include <stdint.h>
#include <stddef.h>

#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

// Register the wireless STM32-flash HTTP endpoint (POST /stm32) with the given
// HTTP server. Mirrors ota_register_handler(): call it for both the AP captive
// portal server and the LAN (STA-mode) server so the STM32 can be updated over
// either link. The endpoint receives a raw STM32 .bin and kicks off the flash.
void stm32_flash_register_handler(httpd_handle_t server);

// Start an asynchronous STM32 flash from an in-RAM firmware image.
//
// Ownership: this function TAKES OWNERSHIP of `fw` and frees it when done (it is
// expected to be a heap buffer, ideally in PSRAM). It spawns a background task
// that parks the STM32 comm task, programs the STM32 over UART, shows on-screen
// progress, and reboots the ESP32 on completion. Safe to call from the HTTP
// server task. Returns immediately (the actual flashing happens in the task).
//
// `fw`  : pointer to the raw STM32 .bin image (will be free()d by the flasher).
// `len` : image length in bytes (256 .. 512 KB for the G474CE).
void stm32_flash_start(uint8_t *fw, size_t len);

// True while an STM32 flash is in progress (so callers can reject a second
// concurrent request). Cleared only by the reboot at the end of a flash.
bool stm32_flash_in_progress(void);

#ifdef __cplusplus
}
#endif

#endif  // STM32_FLASH_H
