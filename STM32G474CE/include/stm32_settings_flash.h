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

#include <stdint.h>
#include <stdbool.h>

/* Flash page configuration for STM32G474CE (512KB Flash, 2KB pages) */
#define SETTINGS_FLASH_PAGE_SIZE    2048U
#define SETTINGS_FLASH_PAGE_NUMBER  255U  /* Last page */
#define SETTINGS_FLASH_BASE_ADDR    0x0807F800UL  /* Page 255 start */

/* Magic number for settings validity check */
#define SETTINGS_MAGIC 0x53504F54UL  /* "SPOT" in ASCII */
#define SETTINGS_VERSION 1U

/**
 * Persistent settings structure (56 bytes)
 * Stored in Flash, survives power cycles.
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;                   // Magic number (0x53504F54 = "SPOT")
    uint8_t version;                  // Settings format version
    
    // Lead resistance (critical for power calculations)
    float lead_resistance_ohms;       // 0.0001 - 0.0100 (0.1 - 10.0 mΩ)
    
    // Weld pulse configuration
    uint8_t weld_mode;                // 1=single, 2=double, 3=triple
    uint16_t weld_d1;                 // Duration pulse 1 (ms)
    uint16_t weld_gap1;               // Gap after pulse 1 (ms)
    uint16_t weld_d2;                 // Duration pulse 2 (ms)
    uint16_t weld_gap2;               // Gap after pulse 2 (ms)
    uint16_t weld_d3;                 // Duration pulse 3 (ms)
    uint8_t weld_power_pct;           // Power % (0-100)
    
    // Preheat configuration
    bool preheat_enabled;
    uint16_t preheat_ms;
    uint8_t preheat_pct;
    uint16_t preheat_gap_ms;
    
    // Trigger configuration
    uint8_t trigger_mode;             // 1=pedal, 2=contact
    uint8_t contact_hold_steps;       // Contact hold duration (0.5s per step)
    
    // Padding to 8-byte alignment for Flash write efficiency
    uint8_t _reserved[6];
    
    // CRC32 checksum (must be last field)
    uint32_t crc32;                   // CRC32 of all bytes before this field
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

#endif // STM32_SETTINGS_FLASH_H
