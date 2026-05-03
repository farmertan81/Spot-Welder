/**
 * @file stm32_settings_flash.c
 * @brief Flash-based persistent settings implementation
 */

#include "stm32_settings_flash.h"
#include "stm32g4xx_hal.h"
#include <string.h>

/* CRC32 polynomial (Ethernet/ZIP standard) */
#define CRC32_POLY 0xEDB88320UL

/**
 * Calculate CRC32 checksum (software implementation).
 * @param data Pointer to data buffer
 * @param length Number of bytes to checksum
 * @return CRC32 value
 */
static uint32_t crc32_calculate(const uint8_t* data, uint32_t length) {
    uint32_t crc = 0xFFFFFFFFUL;
    
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (CRC32_POLY & (-(crc & 1)));
        }
    }
    
    return ~crc;
}

/**
 * Verify settings struct CRC.
 * @param settings Pointer to settings struct
 * @return true if CRC matches
 */
static bool settings_verify_crc(const PersistentSettings* settings) {
    // CRC is calculated over all bytes EXCEPT the crc32 field itself
    size_t crc_data_size = sizeof(PersistentSettings) - sizeof(uint32_t);
    uint32_t calculated_crc = crc32_calculate((const uint8_t*)settings, crc_data_size);
    return (calculated_crc == settings->crc32);
}

/**
 * Update CRC field in settings struct.
 * @param settings Pointer to settings struct to update
 */
static void settings_update_crc(PersistentSettings* settings) {
    size_t crc_data_size = sizeof(PersistentSettings) - sizeof(uint32_t);
    settings->crc32 = crc32_calculate((const uint8_t*)settings, crc_data_size);
}

bool settings_flash_init(void) {
    // HAL Flash module is initialized during HAL_Init() in main()
    // Nothing additional needed here
    return true;
}

bool settings_flash_load(PersistentSettings* settings) {
    if (!settings) return false;
    
    // Read settings directly from Flash memory (no HAL call needed for read)
    const PersistentSettings* flash_settings = (const PersistentSettings*)SETTINGS_FLASH_BASE_ADDR;
    
    // Verify magic number
    if (flash_settings->magic != SETTINGS_MAGIC) {
        return false;  // Flash empty or invalid
    }
    
    // Verify version
    if (flash_settings->version != SETTINGS_VERSION) {
        return false;  // Incompatible version
    }
    
    // Verify CRC
    if (!settings_verify_crc(flash_settings)) {
        return false;  // Corrupted data
    }
    
    // Copy valid settings
    memcpy(settings, flash_settings, sizeof(PersistentSettings));
    return true;
}

bool settings_flash_save(const PersistentSettings* settings) {
    if (!settings) return false;
    
    HAL_StatusTypeDef status;
    
    // Create local copy and update CRC
    PersistentSettings settings_with_crc;
    memcpy(&settings_with_crc, settings, sizeof(PersistentSettings));
    settings_with_crc.magic = SETTINGS_MAGIC;
    settings_with_crc.version = SETTINGS_VERSION;
    settings_update_crc(&settings_with_crc);
    
    // Unlock Flash for write/erase
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) return false;
    
    // Erase the settings page
    FLASH_EraseInitTypeDef erase_config;
    erase_config.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_config.Page = SETTINGS_FLASH_PAGE_NUMBER;
    erase_config.NbPages = 1;
    
    uint32_t page_error = 0;
    status = HAL_FLASHEx_Erase(&erase_config, &page_error);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }
    
    // Write settings to Flash (64-bit chunks for STM32G4)
    // STM32G4 Flash writes must be 64-bit (double word) aligned
    const uint64_t* data_ptr = (const uint64_t*)&settings_with_crc;
    uint32_t num_dwords = (sizeof(PersistentSettings) + 7) / 8;  // Round up
    
    for (uint32_t i = 0; i < num_dwords; i++) {
        uint32_t address = SETTINGS_FLASH_BASE_ADDR + (i * 8);
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data_ptr[i]);
        
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }
    
    // Lock Flash
    HAL_FLASH_Lock();
    
    // Verify write by reading back
    PersistentSettings verify;
    if (!settings_flash_load(&verify)) {
        return false;
    }
    
    return true;
}

bool settings_flash_erase(void) {
    HAL_StatusTypeDef status;
    
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) return false;
    
    FLASH_EraseInitTypeDef erase_config;
    erase_config.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_config.Page = SETTINGS_FLASH_PAGE_NUMBER;
    erase_config.NbPages = 1;
    
    uint32_t page_error = 0;
    status = HAL_FLASHEx_Erase(&erase_config, &page_error);
    
    HAL_FLASH_Lock();
    
    return (status == HAL_OK);
}
