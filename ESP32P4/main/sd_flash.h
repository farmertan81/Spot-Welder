// ============================================================
//  SD Card Firmware Flashing (ESP32-P4 + STM32G474)
// ============================================================
// Provides firmware updates from the on-board microSD card for BOTH chips:
//   1) ESP32-P4 self-flash from /esp32_firmware.bin (OTA API)
//   2) STM32G474 flash from /stm32_firmware.bin (AN3155 bootloader over UART)
//
// ARCHITECTURE (Simplified for SPI WiFi):
//   Since WiFi now runs over SPI (external XIAO C6), not SDIO, the SDMMC
//   controller is free for the SD card. We mount it at boot and keep it mounted.
//   Flashing happens synchronously when the user presses the UI button.
//
// Hardware: ESP32-P4 SDMMC pins (1-bit mode, proven Elecrow config):
//   CLK  = GPIO43
//   CMD  = GPIO44
//   D0   = GPIO39
//   (D1/D2/D3 not wired on CrowPanel Advance 5")
//
// Usage:
//   1. In app_main, call sd_flash_init() AFTER PSRAM/LVGL init (before UI build)
//   2. In UI button callbacks, call sd_flash_esp32() or sd_flash_stm32()
//   3. Progress is shown via ui.h: show_firmware_progress() / hide_firmware_progress()
#ifndef SD_FLASH_H
#define SD_FLASH_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize and mount the SD card (SDMMC SLOT_0, 1-bit, 10 MHz).
// Call once in app_main after PSRAM/LVGL setup, before building the UI.
// Returns true if mounted successfully. If the card isn't present or fails,
// logs an error and returns false (non-fatal: the welder continues without SD).
bool sd_flash_init(void);

// Check if SD card is currently mounted (valid after sd_flash_init()).
bool sd_flash_is_mounted(void);

// Flash the ESP32-P4 from /esp32_firmware.bin on the SD card.
// Blocks until complete, shows progress UI, then REBOOTS on success.
// On failure, shows error popup and returns false (no reboot).
// Safe to call from LVGL button callbacks (runs in app_main task context).
bool sd_flash_esp32(void);

// Flash the STM32G474 from /stm32_firmware.bin on the SD card.
// Spawns async task, shows progress UI, reboots ESP32 on completion.
// Returns immediately (flashing happens in background). On failure, shows
// error popup. Safe to call from LVGL button callbacks.
bool sd_flash_stm32(void);

#ifdef __cplusplus
}
#endif

#endif  // SD_FLASH_H
