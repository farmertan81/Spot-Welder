/**
 * @file stm32_settings_flash.h
 * @brief Flash-based persistent settings for STM32G474 Spot Welder
 *
 * Uses last Flash page (0x0807F800, 2KB) for EEPROM emulation.
 * Survives power cycles, watchdog resets, and firmware updates (as long as
 * the page isn't erased during reflash).
 */

#ifndef STM32_SETTINGS_FLASH_H
#define STM32_SETTINGS_FLASH_H

#include <stdbool.h>
#include <stdint.h>

/* Flash page configuration for STM32G474CE (512KB Flash, 2KB pages) */
#define SETTINGS_FLASH_PAGE_SIZE 2048U
#define SETTINGS_FLASH_PAGE_NUMBER 255U       /* Last page */
#define SETTINGS_FLASH_BASE_ADDR 0x0807F800UL /* Page 255 start */

/* Magic number for settings validity check */
#define SETTINGS_MAGIC 0x53504F54UL /* "SPOT" in ASCII */
#define SETTINGS_VERSION 2U

/**
 * Persistent settings structure.
 * NOTE: Keep crc32 as the final field.
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;   // Magic number (0x53504F54 = "SPOT")
    uint8_t version;  // Settings format version

    // Existing
    float lead_resistance_ohms;

    // Full configurable settings set
    uint8_t pulse_mode;          // 1=single, 2=double, 3=triple
    uint16_t pulse_d1_ms;        // First pulse duration
    uint16_t pulse_gap1_ms;      // Gap after first pulse
    uint16_t pulse_d2_ms;        // Second pulse duration
    uint16_t pulse_gap2_ms;      // Gap after second pulse
    uint16_t pulse_d3_ms;        // Third pulse duration
    uint8_t power_pct;           // Power percentage 0-100
    uint8_t trigger_mode;        // 0=auto, 1=manual, 2=pedal
    uint8_t contact_hold_steps;  // Contact hold steps
    uint8_t contact_with_pedal;  // 0 or 1
    uint8_t preheat_en;          // 0 or 1
    uint16_t preheat_ms;         // Preheat duration
    uint8_t preheat_pct;         // Preheat power
    uint16_t preheat_gap_ms;     // Gap after preheat

    uint32_t crc32;  // Keep at end
} PersistentSettings;

/**
 * Initialize Flash settings system.
 * Call once during boot, before loading settings.
 * @return true on success
 */
bool settings_flash_init(void);

/**
 * Load settings from Flash into provided struct.
 * If Flash is empty or corrupted, returns false and leaves struct unchanged.
 * @param settings Pointer to settings struct to populate
 * @return true if valid settings loaded, false if using defaults
 */
bool settings_flash_load(PersistentSettings* settings);

/**
 * Save settings to Flash.
 * Erases page and writes new data with CRC.
 * @param settings Pointer to settings struct to save
 * @return true on success
 */
bool settings_flash_save(const PersistentSettings* settings);

/**
 * Erase all settings from Flash (factory reset).
 * @return true on success
 */
bool settings_flash_erase(void);

/* Backward-compatible aliases expected by main.c patch */
#define lead_r lead_resistance_ohms
#define flash_settings_init settings_flash_init
#define flash_settings_load settings_flash_load
#define flash_settings_save settings_flash_save
#define flash_settings_erase settings_flash_erase

#endif  // STM32_SETTINGS_FLASH_H
