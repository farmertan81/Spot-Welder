// ============================================================
//  SD Card Firmware Flashing (ESP32-P4 + STM32G474)
// ============================================================
#include "sd_flash.h"
#include "ui.h"          // show_firmware_progress / hide_firmware_progress / show_firmware_result_popup
#include "stm32_flash.h" // stm32_flash_start / stm32_flash_in_progress

#include "esp_log.h"
#include "esp_err.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_heap_caps.h"   // heap_caps_malloc / heap_caps_free / MALLOC_CAP_SPIRAM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ESP-IDF SD card API (SDMMC mode)
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>

static const char *TAG = "SD_FLASH";

// SD card mount point and handle
#define SD_MOUNT_POINT "/sdcard"
static sdmmc_card_t *g_sd_card = NULL;
static bool g_sd_mounted = false;

// Firmware filenames on SD card root
#define SD_ESP32_FW_PATH  SD_MOUNT_POINT "/esp32_firmware.bin"
#define SD_STM32_FW_PATH  SD_MOUNT_POINT "/stm32_firmware.bin"

// OTA buffer size (balance between speed and RAM usage; use PSRAM for large buffer)
#define OTA_BUF_SIZE (16 * 1024)

// ============================================================
//  SD CARD INITIALIZATION (SDMMC SLOT_0, 1-bit)
// ============================================================
// The CrowPanel P4 SD card is wired for SDMMC SLOT_0 in 1-bit mode:
//   CLK = GPIO43, CMD = GPIO44, D0 = GPIO39
//   (D1/D2/D3 are NOT routed on this board)
//
// Since WiFi now runs over SPI (external XIAO C6 via J7 header), the SDMMC
// controller is free and we can mount the SD card at boot permanently.
bool sd_flash_init(void)
{
    if (g_sd_mounted) {
        ESP_LOGI(TAG, "SD card already mounted at %s", SD_MOUNT_POINT);
        return true;
    }

    ESP_LOGI(TAG, "Initializing SD card (SDMMC SLOT_0, 1-bit, 10 MHz)");
    ESP_LOGI(TAG, "SD pins: CLK=43, CMD=44, D0=39 (D1/D2/D3 not wired)");

    // SDMMC host config (matches proven Elecrow reference design)
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;   // SLOT_0 for SD card (SLOT_1 was old onboard C6/WiFi)
    host.max_freq_khz = 10000;       // 10 MHz (conservative, proven stable on 1-bit)

    // Slot config (1-bit mode, internal pull-ups)
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = GPIO_NUM_43;
    slot_config.cmd = GPIO_NUM_44;
    slot_config.d0  = GPIO_NUM_39;
    slot_config.width = 1;  // 1-bit mode (only D0 wired on this board)
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    // VFS FAT mount config
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,  // do NOT auto-format if no card
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false,
        .use_one_fat = false
    };

    // Mount the card
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot_config,
                                            &mount_config, &g_sd_card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "SD card mount FAILED (no card inserted or filesystem corrupt)");
        } else {
            ESP_LOGE(TAG, "SD card mount error: %s (0x%x)", esp_err_to_name(ret), ret);
        }
        g_sd_mounted = false;
        return false;
    }

    // Log card info
    ESP_LOGI(TAG, "SD card mounted successfully at %s", SD_MOUNT_POINT);
    sdmmc_card_print_info(stdout, g_sd_card);
    
    g_sd_mounted = true;
    return true;
}

bool sd_flash_is_mounted(void)
{
    return g_sd_mounted;
}

// ============================================================
//  ESP32-P4 SELF-FLASH (OTA from SD)
// ============================================================
// Reads /esp32_firmware.bin from SD, writes to the inactive OTA partition
// using the esp_ota_* API, validates, switches boot partition, and reboots.
// Shows progress UI during the flash.
bool sd_flash_esp32(void)
{
    if (!g_sd_mounted) {
        ESP_LOGE(TAG, "SD card not mounted — cannot flash ESP32");
        show_firmware_result_popup(false, "SD card not mounted", "ESP32-P4");
        return false;
    }

    ESP_LOGI(TAG, "Starting ESP32-P4 flash from %s", SD_ESP32_FW_PATH);

    // DEBUG: List SD card root to verify file presence
    ESP_LOGI(TAG, "SD card root directory listing:");
    DIR *dir = opendir(SD_MOUNT_POINT);
    if (dir) {
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            ESP_LOGI(TAG, "  %s", entry->d_name);
        }
        closedir(dir);
    } else {
        ESP_LOGE(TAG, "Failed to open SD root: %s", strerror(errno));
    }

    // Check if file exists and get size
    struct stat st;
    if (stat(SD_ESP32_FW_PATH, &st) != 0) {
        ESP_LOGE(TAG, "Firmware file not found: %s", SD_ESP32_FW_PATH);
        show_firmware_result_popup(false, "esp32_firmware.bin not found on SD card", "ESP32-P4");
        return false;
    }
    size_t fw_size = st.st_size;
    ESP_LOGI(TAG, "Firmware file size: %zu bytes (%.2f MB)", fw_size, fw_size / (1024.0 * 1024.0));

    if (fw_size < 256 * 1024 || fw_size > 8 * 1024 * 1024) {
        ESP_LOGE(TAG, "Firmware size suspicious: %zu bytes (expected 256 KB - 8 MB)", fw_size);
        show_firmware_result_popup(false, "Invalid firmware size", "ESP32-P4");
        return false;
    }

    // Open file
    FILE *f = fopen(SD_ESP32_FW_PATH, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open %s: %s", SD_ESP32_FW_PATH, strerror(errno));
        show_firmware_result_popup(false, "Failed to open firmware file", "ESP32-P4");
        return false;
    }

    // Begin OTA
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        ESP_LOGE(TAG, "No OTA update partition available");
        fclose(f);
        show_firmware_result_popup(false, "No OTA partition found", "ESP32-P4");
        return false;
    }
    ESP_LOGI(TAG, "Writing to OTA partition: %s (offset 0x%lx, size %lu bytes)",
             update_partition->label, update_partition->address, update_partition->size);

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, fw_size, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        fclose(f);
        show_firmware_result_popup(false, "OTA begin failed", "ESP32-P4");
        return false;
    }

    // Allocate read buffer in PSRAM (large buffer for speed)
    uint8_t *buf = (uint8_t*)heap_caps_malloc(OTA_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate %d-byte OTA buffer in PSRAM", OTA_BUF_SIZE);
        esp_ota_abort(ota_handle);
        fclose(f);
        show_firmware_result_popup(false, "Out of memory", "ESP32-P4");
        return false;
    }

    // Show progress UI (initial state: 0%)
    show_firmware_progress("ESP32-P4", 0, "Writing firmware...");

    // Read and write firmware in chunks
    size_t bytes_written = 0;
    int last_pct = -1;
    while (true) {
        size_t rd = fread(buf, 1, OTA_BUF_SIZE, f);
        if (rd == 0) {
            break;  // EOF
        }

        err = esp_ota_write(ota_handle, buf, rd);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed at %zu bytes: %s", bytes_written, esp_err_to_name(err));
            heap_caps_free(buf);
            esp_ota_abort(ota_handle);
            fclose(f);
            hide_firmware_progress();
            show_firmware_result_popup(false, "OTA write failed", "ESP32-P4");
            return false;
        }

        bytes_written += rd;
        int pct = (int)((bytes_written * 100) / fw_size);
        if (pct != last_pct && pct <= 100) {
            show_firmware_progress("ESP32-P4", pct, NULL);
            last_pct = pct;
        }

        // Yield to LVGL task periodically (every 64 KB written)
        if ((bytes_written & 0xFFFF) == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    heap_caps_free(buf);
    fclose(f);

    if (bytes_written != fw_size) {
        ESP_LOGW(TAG, "File size mismatch: expected %zu, wrote %zu", fw_size, bytes_written);
    }

    ESP_LOGI(TAG, "OTA write complete: %zu bytes", bytes_written);

    // End OTA and validate
    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s (image validation failed)", esp_err_to_name(err));
        hide_firmware_progress();
        show_firmware_result_popup(false, "Firmware validation failed", "ESP32-P4");
        return false;
    }

    // Set new boot partition
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        hide_firmware_progress();
        show_firmware_result_popup(false, "Failed to set boot partition", "ESP32-P4");
        return false;
    }

    ESP_LOGI(TAG, "ESP32-P4 firmware flash SUCCESS — rebooting in 2 seconds");
    hide_firmware_progress();
    show_firmware_result_popup(true, "Flash complete! Rebooting...", "ESP32-P4");

    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();

    // Never reached
    return true;
}

// ============================================================
//  STM32G474 FLASH (from SD via existing stm32_flash.h API)
// ============================================================
// Reads /stm32_firmware.bin from SD, loads it into PSRAM, and calls
// stm32_flash_start() which spawns an async task to program the STM32.
bool sd_flash_stm32(void)
{
    if (!g_sd_mounted) {
        ESP_LOGE(TAG, "SD card not mounted — cannot flash STM32");
        show_firmware_result_popup(false, "SD card not mounted", "STM32");
        return false;
    }

    if (stm32_flash_in_progress()) {
        ESP_LOGW(TAG, "STM32 flash already in progress — ignoring duplicate request");
        return false;
    }

    ESP_LOGI(TAG, "Starting STM32 flash from %s", SD_STM32_FW_PATH);

    // Check if file exists and get size
    struct stat st;
    if (stat(SD_STM32_FW_PATH, &st) != 0) {
        ESP_LOGE(TAG, "Firmware file not found: %s", SD_STM32_FW_PATH);
        show_firmware_result_popup(false, "stm32_firmware.bin not found on SD card", "STM32");
        return false;
    }
    size_t fw_size = st.st_size;
    ESP_LOGI(TAG, "STM32 firmware file size: %zu bytes (%.2f KB)", fw_size, fw_size / 1024.0);

    if (fw_size < 8 * 1024 || fw_size > 512 * 1024) {
        ESP_LOGE(TAG, "Firmware size suspicious: %zu bytes (expected 8-512 KB for STM32G474)", fw_size);
        show_firmware_result_popup(false, "Invalid firmware size", "STM32");
        return false;
    }

    // Open file
    FILE *f = fopen(SD_STM32_FW_PATH, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open %s: %s", SD_STM32_FW_PATH, strerror(errno));
        show_firmware_result_popup(false, "Failed to open firmware file", "STM32");
        return false;
    }

    // Allocate firmware buffer in PSRAM (stm32_flash_start takes ownership and frees it)
    uint8_t *fw_buf = (uint8_t*)heap_caps_malloc(fw_size, MALLOC_CAP_SPIRAM);
    if (!fw_buf) {
        ESP_LOGE(TAG, "Failed to allocate %zu-byte STM32 firmware buffer in PSRAM", fw_size);
        fclose(f);
        show_firmware_result_popup(false, "Out of memory", "STM32");
        return false;
    }

    // Read entire firmware into PSRAM
    size_t rd = fread(fw_buf, 1, fw_size, f);
    fclose(f);

    if (rd != fw_size) {
        ESP_LOGE(TAG, "Failed to read full firmware: got %zu of %zu bytes", rd, fw_size);
        heap_caps_free(fw_buf);
        show_firmware_result_popup(false, "File read error", "STM32");
        return false;
    }

    ESP_LOGI(TAG, "Firmware loaded into PSRAM — starting async flash task");

    // Hand off to existing stm32_flash API (it takes ownership of fw_buf and frees it)
    stm32_flash_start(fw_buf, fw_size);

    // stm32_flash_start spawns a task that shows progress UI and reboots on completion
    return true;
}
