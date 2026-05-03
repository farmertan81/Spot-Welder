/**
 * @file stm32_settings_flash.c
 * @brief Flash-based persistent settings implementation
 */

#include "stm32_settings_flash.h"

#include <stdio.h>
#include <string.h>

#include "stm32g4xx_hal.h"

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
    uint32_t calculated_crc =
        crc32_calculate((const uint8_t*)settings, crc_data_size);
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

extern UART_HandleTypeDef huart1;

static void flash_debug_send(const char* msg) {
    if (!msg || huart1.Instance == NULL) return;
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, (uint16_t)strlen(msg), 50U);
}

static void flash_debug_u32(const char* tag, uint32_t value) {
    char buf[96];
    snprintf(buf, sizeof(buf), "DBG,%s=0x%08lX\r\n", tag, (unsigned long)value);
    flash_debug_send(buf);
}

static void flash_prepare_erase_config(FLASH_EraseInitTypeDef* erase_config) {
    memset(erase_config, 0, sizeof(*erase_config));
    erase_config->TypeErase = FLASH_TYPEERASE_PAGES;

#if defined(FLASH_BANK_1)
#if defined(FLASH_OPTR_DBANK) && defined(FLASH_BANK_2)
    uint32_t dual_bank =
        (READ_BIT(FLASH->OPTR, FLASH_OPTR_DBANK) != 0U) ? 1U : 0U;
    if (dual_bank) {
        if (SETTINGS_FLASH_PAGE_NUMBER >= 128U) {
            erase_config->Banks = FLASH_BANK_2;
            erase_config->Page = SETTINGS_FLASH_PAGE_NUMBER - 128U;
        } else {
            erase_config->Banks = FLASH_BANK_1;
            erase_config->Page = SETTINGS_FLASH_PAGE_NUMBER;
        }
    } else {
        erase_config->Banks = FLASH_BANK_1;
        erase_config->Page = SETTINGS_FLASH_PAGE_NUMBER;
    }
#else
    erase_config->Banks = FLASH_BANK_1;
    erase_config->Page = SETTINGS_FLASH_PAGE_NUMBER;
#endif
#else
    erase_config->Page = SETTINGS_FLASH_PAGE_NUMBER;
#endif

    erase_config->NbPages = 1U;
}

bool settings_flash_init(void) {
    // HAL Flash module is initialized during HAL_Init() in main()
    // Nothing additional needed here
    return true;
}

bool settings_flash_load(PersistentSettings* settings) {
    if (!settings) return false;

    // Read settings directly from Flash memory (no HAL call needed for read)
    const PersistentSettings* flash_settings =
        (const PersistentSettings*)SETTINGS_FLASH_BASE_ADDR;

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
    if (!settings) {
        flash_debug_send("DBG,FLASH_SAVE_NULL_INPUT\r\n");
        return false;
    }

    flash_debug_send("DBG,FLASH_SAVE_START\r\n");
    flash_debug_u32("FLASH_BASE", SETTINGS_FLASH_BASE_ADDR);
    flash_debug_u32("FLASH_PAGE_CFG", SETTINGS_FLASH_PAGE_NUMBER);

    HAL_StatusTypeDef status;

    // Create local copy and update CRC
    PersistentSettings settings_with_crc;
    memset(&settings_with_crc, 0, sizeof(settings_with_crc));
    memcpy(&settings_with_crc, settings, sizeof(PersistentSettings));
    settings_with_crc.magic = SETTINGS_MAGIC;
    settings_with_crc.version = SETTINGS_VERSION;
    settings_update_crc(&settings_with_crc);

    flash_debug_u32("FLASH_CRC", settings_with_crc.crc32);

    // Unlock Flash for write/erase
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        flash_debug_send("DBG,FLASH_UNLOCK_FAILED\r\n");
        flash_debug_u32("FLASH_UNLOCK_ERR", HAL_FLASH_GetError());
        return false;
    }
    flash_debug_send("DBG,FLASH_UNLOCKED\r\n");

    // Erase the settings page
    FLASH_EraseInitTypeDef erase_config;
    flash_prepare_erase_config(&erase_config);

    flash_debug_u32("FLASH_ERASE_PAGE", erase_config.Page);
#if defined(FLASH_BANK_1)
    flash_debug_u32("FLASH_ERASE_BANK", erase_config.Banks);
#endif

    uint32_t page_error = 0xFFFFFFFFUL;
    status = HAL_FLASHEx_Erase(&erase_config, &page_error);
    if (status != HAL_OK) {
        flash_debug_send("DBG,FLASH_ERASE_FAILED\r\n");
        flash_debug_u32("FLASH_ERASE_STATUS", (uint32_t)status);
        flash_debug_u32("FLASH_ERASE_PAGE_ERR", page_error);
        flash_debug_u32("FLASH_ERASE_HAL_ERR", HAL_FLASH_GetError());
        HAL_FLASH_Lock();
        return false;
    }
    flash_debug_send("DBG,FLASH_ERASED\r\n");

    // Write settings to Flash (64-bit chunks for STM32G4)
    // STM32G4 Flash writes must be 64-bit (double word) aligned
    const uint64_t* data_ptr = (const uint64_t*)&settings_with_crc;
    uint32_t num_dwords = (sizeof(PersistentSettings) + 7U) / 8U;
    flash_debug_u32("FLASH_DWORDS", num_dwords);

    for (uint32_t i = 0; i < num_dwords; i++) {
        uint32_t address = SETTINGS_FLASH_BASE_ADDR + (i * 8U);
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address,
                                   data_ptr[i]);

        if (status != HAL_OK) {
            flash_debug_send("DBG,FLASH_WRITE_FAILED\r\n");
            flash_debug_u32("FLASH_WRITE_INDEX", i);
            flash_debug_u32("FLASH_WRITE_ADDR", address);
            flash_debug_u32("FLASH_WRITE_STATUS", (uint32_t)status);
            flash_debug_u32("FLASH_WRITE_HAL_ERR", HAL_FLASH_GetError());
            HAL_FLASH_Lock();
            return false;
        }
    }
    flash_debug_send("DBG,FLASH_WRITTEN\r\n");

    // Lock Flash
    status = HAL_FLASH_Lock();
    if (status != HAL_OK) {
        flash_debug_send("DBG,FLASH_LOCK_FAILED\r\n");
        flash_debug_u32("FLASH_LOCK_ERR", HAL_FLASH_GetError());
        return false;
    }
    flash_debug_send("DBG,FLASH_LOCKED\r\n");

    // Verify write by reading back
    PersistentSettings verify;
    if (!settings_flash_load(&verify)) {
        flash_debug_send("DBG,FLASH_VERIFY_LOAD_FAILED\r\n");
        return false;
    }

    if (verify.crc32 != settings_with_crc.crc32) {
        flash_debug_send("DBG,FLASH_VERIFY_CRC_MISMATCH\r\n");
        flash_debug_u32("FLASH_VERIFY_CRC_RD", verify.crc32);
        flash_debug_u32("FLASH_VERIFY_CRC_WR", settings_with_crc.crc32);
        return false;
    }

    flash_debug_send("DBG,FLASH_SAVE_SUCCESS\r\n");
    return true;
}

bool settings_flash_erase(void) {
    HAL_StatusTypeDef status;

    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) return false;

    FLASH_EraseInitTypeDef erase_config;
    flash_prepare_erase_config(&erase_config);

    uint32_t page_error = 0;
    status = HAL_FLASHEx_Erase(&erase_config, &page_error);

    HAL_FLASH_Lock();

    return (status == HAL_OK);
}
