// ============================================================
//  STM32 remote firmware flashing (AN3155 ROM bootloader over UART)
// ============================================================
// ESP-IDF-native port of the OLD board's proven AN3155 flasher
// (ESP32_8048S043C/src/main.cpp). A new STM32G474 firmware image is delivered
// wirelessly to the ESP32-P4 (POST /stm32 in wifi_bridge.cpp), which then drives
// the STM32 into its factory ROM bootloader and programs it over the existing
// UART_NUM_1 link, finally rebooting so both sides come up on a clean app link.
//
// Entry into the ROM bootloader uses the hardware reset method (guaranteed to
// work per AN2606):
//   1. BOOT0 (GPIO31 -> STM32) is driven HIGH before the reset.
//   2. NRST (GPIO32 -> STM32) is pulsed LOW for 10ms, then released HIGH. The
//      chip resets and, with BOOT0 HIGH, boots into the ROM bootloader.
// This sidesteps all software jump complexity and matches the official ST boot
// mode selection documented in AN2606.
//
// Timing note (carried over from the OLD board): the build runs code from PSRAM
// (CONFIG_SPIRAM_XIP_FROM_PSRAM). PSRAM fetches stall under bus contention, which
// can delay UART RX servicing enough to drop the timing-sensitive 8E1 bootloader
// ACK bytes. So every AN3155 helper below is pinned in fast internal RAM with
// IRAM_ATTR, and welder_prep_stm32_flash() (in welder_main.cpp) slows the RGB
// panel pixel clock to quiet the PSRAM bus for the duration of the flash.

#include "stm32_flash.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"

#include "ui.h"  // show_firmware_progress()

static const char *TAG = "STM32_FLASH";

// ---- Pins / UART (MUST match welder_main.cpp) ----
#define STM_BOOT_UART       UART_NUM_1
#define STM_TX_PIN          29   // ESP32 TX -> STM32 RX
#define STM_RX_PIN          30   // ESP32 RX <- STM32 TX
#define STM_BOOT0_PIN       31   // ESP32 -> STM32 BOOT0 strap
#define STM_NRST_PIN        32   // ESP32 -> STM32 NRST (hardware reset)
#define STM_APP_BAUD        1000000  // normal application link (8N1)

// ---- AN3155 / bootloader constants ----
static const uint8_t STM_ACK  = 0x79;
static const uint8_t STM_NACK = 0x1F;
#define STM32_FLASH_BASE 0x08000000UL

// Implemented in welder_main.cpp: parks stm32_task at its safe pause point and
// quiets the LCD bus before we take over UART_NUM_1.
extern "C" void welder_prep_stm32_flash(void);

// ---- Flasher state ----
static volatile bool s_in_progress = false;
static uint8_t      *s_fw  = NULL;   // owned heap buffer (freed by the task)
static size_t        s_len = 0;

bool stm32_flash_in_progress(void) { return s_in_progress; }

// ============================================================
//  LOW-LEVEL AN3155 UART HELPERS (all IRAM_ATTR — see header note)
// ============================================================

// Write a buffer of bytes to the bootloader UART.
static IRAM_ATTR void stmWrite(const uint8_t *data, size_t len) {
    uart_write_bytes(STM_BOOT_UART, (const char *)data, len);
}

// Write a single byte to the bootloader UART.
static IRAM_ATTR void stmWriteByte(uint8_t b) {
    uart_write_bytes(STM_BOOT_UART, (const char *)&b, 1);
}

// Block until the bootloader UART TX FIFO has fully drained.
static IRAM_ATTR void stmFlushTx() {
    uart_wait_tx_done(STM_BOOT_UART, pdMS_TO_TICKS(1000));
}

// How many bytes are waiting in the bootloader UART RX ring buffer.
static IRAM_ATTR int stmRxAvailable() {
    size_t len = 0;
    uart_get_buffered_data_len(STM_BOOT_UART, &len);
    return (int)len;
}

// Read one byte with timeout (ms). Returns -1 on timeout.
static IRAM_ATTR int stmReadByte(uint32_t timeout_ms) {
    uint8_t b;
    int n = uart_read_bytes(STM_BOOT_UART, &b, 1, pdMS_TO_TICKS(timeout_ms));
    return (n == 1) ? (int)b : -1;
}

// Wait for an ACK (0x79). Logs NACK/timeout/unexpected bytes for diagnostics.
static IRAM_ATTR bool stmWaitAck(uint32_t timeout_ms) {
    int b = stmReadByte(timeout_ms);
    if (b == STM_ACK) return true;
    if (b == STM_NACK)
        ESP_LOGW(TAG, "NACK");
    else if (b < 0)
        ESP_LOGW(TAG, "timeout waiting for ACK");
    else
        ESP_LOGW(TAG, "unexpected byte 0x%02X", b);
    return false;
}

// Drain (discard) any pending RX bytes. Returns how many were dropped.
static IRAM_ATTR int stmDrainRx() {
    int n = stmRxAvailable();
    if (n > 0) uart_flush_input(STM_BOOT_UART);
    return n;
}

// Send command byte + its complement, wait for ACK.
static IRAM_ATTR bool stmSendCmd(uint8_t cmd) {
    uint8_t b[2] = {cmd, (uint8_t)(cmd ^ 0xFF)};
    stmWrite(b, 2);
    stmFlushTx();
    return stmWaitAck(800);
}

// Bootloader auto-baud handshake: send 0x7F until the device answers.
static IRAM_ATTR bool stmSync() {
    const int kAttempts = 10;
    for (int i = 0; i < kAttempts; i++) {
        int drained = stmDrainRx();
        if (drained > 0)
            ESP_LOGI(TAG, "drained %d stale byte(s) before sync", drained);
        ESP_LOGI(TAG, "sync attempt %d/%d: sending 0x7F", i + 1, kAttempts);
        uart_write_bytes(STM_BOOT_UART, "\x7F", 1);
        uart_wait_tx_done(STM_BOOT_UART, pdMS_TO_TICKS(100));
        int b = stmReadByte(500);
        if (b == STM_ACK) {
            ESP_LOGI(TAG, "sync OK (ACK)");
            return true;
        }
        if (b == STM_NACK) {
            ESP_LOGI(TAG, "sync OK (NACK = already initialised)");
            return true;
        }
        if (b < 0)
            ESP_LOGI(TAG, "no response (timeout)");
        else
            ESP_LOGI(TAG, "unexpected 0x%02X", b & 0xFF);
        vTaskDelay(pdMS_TO_TICKS(200));  // give the ROM more time between tries
    }
    ESP_LOGE(TAG, "no response after %d attempts (RX avail=%d)", kAttempts,
             stmRxAvailable());
    return false;
}

// Get ID command (0x02) -> 12-bit product ID (G474 = 0x469).
static IRAM_ATTR bool stmGetId(uint16_t *id) {
    if (!stmSendCmd(0x02)) return false;
    int n = stmReadByte(500);  // number-of-bytes-minus-1 (=1 for a 2-byte ID)
    if (n < 0) return false;
    int hi = stmReadByte(500);
    int lo = stmReadByte(500);
    if (hi < 0 || lo < 0) return false;
    if (!stmWaitAck(500)) return false;
    if (id) *id = (uint16_t)(((hi & 0xFF) << 8) | (lo & 0xFF));
    return true;
}

// Extended Erase (0x44) global mass-erase (special code 0xFFFF, checksum 0x00).
static IRAM_ATTR bool stmMassErase() {
    if (!stmSendCmd(0x44)) return false;
    uint8_t e[3] = {0xFF, 0xFF, 0x00};  // special mass-erase code + checksum
    stmWrite(e, 3);
    stmFlushTx();
    return stmWaitAck(30000);  // a full mass erase can take several seconds
}

// Write Memory (0x31): up to 256 bytes (len = 1..256) at `addr`.
static IRAM_ATTR bool stmWriteChunk(uint32_t addr, const uint8_t *data, int len) {
    if (len < 1 || len > 256) return false;
    if (!stmSendCmd(0x31)) return false;
    uint8_t a[4] = {(uint8_t)(addr >> 24), (uint8_t)(addr >> 16),
                    (uint8_t)(addr >> 8), (uint8_t)addr};
    uint8_t acs = a[0] ^ a[1] ^ a[2] ^ a[3];
    uint8_t addrpkt[5] = {a[0], a[1], a[2], a[3], acs};
    stmWrite(addrpkt, 5);
    stmFlushTx();
    if (!stmWaitAck(800)) return false;

    uint8_t n = (uint8_t)(len - 1);  // device expects (count - 1)
    uint8_t cs = n;
    for (int i = 0; i < len; i++) cs ^= data[i];
    stmWriteByte(n);
    stmWrite(data, len);
    stmWriteByte(cs);
    stmFlushTx();
    return stmWaitAck(2000);
}

// Go (0x21): jump to and run the application at `addr`.
static IRAM_ATTR bool stmGo(uint32_t addr) {
    if (!stmSendCmd(0x21)) return false;
    uint8_t a[4] = {(uint8_t)(addr >> 24), (uint8_t)(addr >> 16),
                    (uint8_t)(addr >> 8), (uint8_t)addr};
    uint8_t cs = a[0] ^ a[1] ^ a[2] ^ a[3];
    uint8_t pkt[5] = {a[0], a[1], a[2], a[3], cs};
    stmWrite(pkt, 5);
    stmFlushTx();
    return stmWaitAck(1000);
}

// ============================================================
//  FLASH SEQUENCE
// ============================================================

// Switch the link from the app's 1 Mbaud 8N1 to the ROM bootloader's 115200 8E1.
//
// ROOT-CAUSE NOTE (2026-06-19): This board's STM32 SD/UART flash provably worked
// on the OLD Sunton firmware (git 8b63ab5 + 6cebf05). That working code did a
// FULL Arduino HardwareSerial teardown + reopen:
//        STM32Serial.end();
//        STM32Serial.begin(115200, SERIAL_8E1, RX, TX);
// i.e. the driver was completely re-initialised for the parity change. The flash
// only broke AFTER the screen-glitch fixes, when the code was changed (git
// 8b13ade / 5bdc3fb) to flip parity IN PLACE on the live driver to avoid a
// feared "TX glitch" during teardown. An in-place uart_set_parity() does NOT
// cleanly reset the 8E1 framing state, so the ROM's auto-baud mis-locks — the
// exact 0x9A/0xFA garbage / silent-ACK symptom seen on the P4.
//
// So we go back to the proven behaviour: fully delete and reinstall the driver
// at 115200 8E1, which is the ESP-IDF equivalent of .end()/.begin(). To remove
// the original "floating TX" concern entirely, we first park the TX pin as a
// plain GPIO driven HIGH (UART idle level) so the line is never released low
// during the swap — the very first edge the ROM ever sees is our clean 0x7F.
static void switch_uart_to_bootloader(void) {
    // Make sure everything queued at 1 Mbaud has physically left the wire before
    // we tear the driver down.
    uart_wait_tx_done(STM_BOOT_UART, pdMS_TO_TICKS(50));

    // Hold TX idle-high as a GPIO across the teardown gap so the line never
    // floats / glitches low (which the ROM would read as a fake start bit).
    gpio_set_direction((gpio_num_t)STM_TX_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)STM_TX_PIN, 1);

    // Full clean re-init = .end()/.begin(). Delete the live driver, reconfigure
    // for 115200 8E1, re-mux the pins, and reinstall. This resets ALL framing
    // state (the in-place retune did not), so the ROM auto-baud locks on cleanly.
    uart_driver_delete(STM_BOOT_UART);

    uart_config_t cfg = {};
    cfg.baud_rate = 115200;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity    = UART_PARITY_EVEN;   // ROM bootloader is 8E1
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#if defined(UART_SCLK_XTAL)
    // Clock from XTAL so the baud is exact and immune to power-management clock
    // scaling (an inaccurate baud is itself a classic auto-baud mis-lock cause).
    cfg.source_clk = UART_SCLK_XTAL;
#endif
    uart_param_config(STM_BOOT_UART, &cfg);
    uart_set_pin(STM_BOOT_UART, STM_TX_PIN, STM_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(STM_BOOT_UART, 4096, 0, 0, NULL, 0);

    uart_flush_input(STM_BOOT_UART);
}

// Worker: programs the STM32 from the in-RAM image. Returns true on success and
// writes a short result string to `msg`. Does NOT free the buffer or reboot —
// the task wrapper owns that.
static bool flash_worker(const uint8_t *fw, size_t len, char *msg, size_t msgn) {
    // 1) Park stm32_task and quiet the LCD bus (frees the UART + PSRAM bandwidth).
    welder_prep_stm32_flash();

    show_firmware_progress("STM32", 0, "Entering bootloader...");

    // 2) Assert BOOT0 HIGH so the upcoming reset lands in the ROM bootloader.
    gpio_set_level((gpio_num_t)STM_BOOT0_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));  // let BOOT0 settle before the reset
    // DIAGNOSTIC: confirm the pin is actually driving HIGH (catches a broken /
    // mis-configured GPIO before we blame the protocol).
    int b0 = gpio_get_level((gpio_num_t)STM_BOOT0_PIN);
    ESP_LOGI(TAG, "BOOT0 (GPIO%d) set HIGH, readback=%d %s", STM_BOOT0_PIN, b0,
             b0 ? "(OK)" : "(FAILED — pin not driving high!)");
    if (b0 != 1) {
        snprintf(msg, msgn, "BOOT0 (GPIO%d) cannot drive HIGH", STM_BOOT0_PIN);
        return false;
    }

    // 3) Hardware reset pulse on NRST (active-LOW). Per AN2606, the ROM bootloader
    //    entry sequence is: BOOT0=HIGH, then hardware reset. Pull NRST LOW for
    //    10ms (well above the STM32 minimum reset pulse width of ~100ns), then
    //    release HIGH. The chip resets and samples BOOT0, entering the ROM.
    ESP_LOGI(TAG, "Pulsing NRST (GPIO%d) LOW for 10ms...", STM_NRST_PIN);
    gpio_set_level((gpio_num_t)STM_NRST_PIN, 0);  // assert reset
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)STM_NRST_PIN, 1);  // release reset
    // Verify NRST drives both LOW and HIGH (catches wiring faults).
    vTaskDelay(pdMS_TO_TICKS(5));
    int nrst = gpio_get_level((gpio_num_t)STM_NRST_PIN);
    ESP_LOGI(TAG, "NRST released HIGH, readback=%d %s", nrst,
             nrst ? "(OK)" : "(FAILED — stuck low!)");
    if (nrst != 1) {
        snprintf(msg, msgn, "NRST (GPIO%d) stuck LOW after release", STM_NRST_PIN);
        return false;
    }

    // 4) Let the STM32 ROM bootloader complete its startup. Per AN2606 Figure 58,
    //    the ROM configures its 72MHz clock, GPIO, and peripherals, then enters
    //    the polling loop. 50ms is generous (the ROM is ready in ~2ms, but this
    //    matches the proven old-board timing and ensures we never race it).
    vTaskDelay(pdMS_TO_TICKS(50));

    // 5) Switch the UART from the app's 1Mbaud 8N1 to the ROM bootloader's
    //    115200 8E1. Full driver reinstall (the proven .end()/.begin() flow),
    //    with TX held idle-high across the gap so the line never glitches.
    switch_uart_to_bootloader();

    // 6) Settling delay after driver reinstall to ensure the UART is stable.
    vTaskDelay(pdMS_TO_TICKS(50));

    // 7) Drain any residual bytes so the FIRST thing we actively send is a
    //    pristine 0x7F sync.
    stmDrainRx();

    bool ok = false;
    const char *err = NULL;
    do {
        if (!stmSync()) {
            err = "STM32 bootloader did not respond";
            // POST-FAILURE DIAGNOSTIC. The hardware reset (BOOT0=HIGH + NRST pulse)
            // guarantees the chip entered the ROM bootloader per AN2606. If the sync
            // failed, it's a UART/protocol issue, not an entry failure. Possible causes:
            //   1. TX/RX swapped (we send 0x7F, never get ACK 0x79 back)
            //   2. Baud rate mismatch (framing errors, silence, or garbage)
            //   3. USART1 not the active bootloader interface (wrong pins, or the chip
            //      defaulted to USB/I2C/SPI per AN2606 Table 101)
            // Stay at 8E1 (ROM config) and passively listen for any traffic.
            uart_flush_input(STM_BOOT_UART);
            char rp[256];
            int rt = 0;
            for (int i = 0; i < 10 && rt < (int)sizeof(rp) - 1; i++) {
                int n = uart_read_bytes(STM_BOOT_UART, (uint8_t *)rp + rt,
                                        sizeof(rp) - 1 - rt, pdMS_TO_TICKS(100));
                if (n > 0) rt += n;
            }
            rp[rt > 0 ? rt : 0] = '\0';
            if (rt > 0) {
                ESP_LOGI(TAG, "Passive listen (8E1): heard %d bytes. First 32 (text): "
                              "[%.*s]  (hex):", rt, (rt < 32 ? rt : 32), rp);
                for (int i = 0; i < rt && i < 16; i++)
                    printf(" %02X", (uint8_t)rp[i]);
                printf("\n");
            } else {
                ESP_LOGE(TAG, "Passive listen (8E1): complete silence. Likely wiring "
                              "fault (TX/RX swapped, disconnected, or wrong USART pins).");
            }
            break;
        }

        uint16_t id = 0;
        if (stmGetId(&id))
            ESP_LOGI(TAG, "product ID = 0x%03X%s", id,
                     (id == 0x469) ? " (G474 OK)" : "");

        show_firmware_progress("STM32", 5, "Erasing...");
        if (!stmMassErase()) { err = "STM32 mass-erase failed"; break; }

        // ---- WRITE LOOP: 256-byte chunks from the in-RAM image ----
        show_firmware_progress("STM32", 10, "Writing...");
        uint32_t addr = STM32_FLASH_BASE;
        size_t done = 0;
        uint8_t buf[256];
        bool werr = false;
        int last_drawn_pct = 10;
        while (done < len) {
            size_t remain = len - done;
            int r = (remain > sizeof(buf)) ? (int)sizeof(buf) : (int)remain;
            memcpy(buf, fw + done, r);
            // Pad the final partial word up to a 4-byte boundary with 0xFF.
            while (r % 4) buf[r++] = 0xFF;
            if (!stmWriteChunk(addr, buf, r)) { werr = true; break; }
            addr += r;
            done += r;
            // Map write progress into the 10..95% band of the bar.
            int pct = 10 + (int)((done * 85) / len);
            if (pct > 95) pct = 95;
            if (pct >= last_drawn_pct + 10 || done >= len) {
                last_drawn_pct = pct;
                show_firmware_progress("STM32", pct, "Writing...");
            }
            vTaskDelay(1);  // let other FreeRTOS tasks run (also feeds WDT)
        }
        if (werr) { err = "STM32 write failed"; break; }
        ok = true;
    } while (0);

    // 5) Drop BOOT0 LOW so future resets boot the (new) application, not ROM.
    //    Done unconditionally so a failed flash never leaves the STM32 in DFU.
    gpio_set_level((gpio_num_t)STM_BOOT0_PIN, 0);

    // 6) On success, launch the freshly written app via AN3155 "Go".
    if (ok) {
        show_firmware_progress("STM32", 98, "Launching...");
        stmGo(STM32_FLASH_BASE);
    }

    snprintf(msg, msgn, "%s",
             ok ? "STM32 firmware updated. Restarting..."
                : (err ? err : "STM32 update failed"));
    return ok;
}

// Background task: runs the flash, shows the result, then reboots the ESP32 so
// both sides come up on a clean, re-synced 1 Mbaud app link.
static void stm32_flash_task(void *arg) {
    char msg[128] = {0};
    bool ok = flash_worker(s_fw, s_len, msg, sizeof(msg));

    ESP_LOGI(TAG, "flash %s: %s", ok ? "OK" : "FAIL", msg);
    show_firmware_progress("STM32", 100,
                           ok ? "Update complete - restarting..."
                              : "Update failed - restarting...");

    if (s_fw) { free(s_fw); s_fw = NULL; }
    s_len = 0;

    // Always reboot so both sides start from a clean app link (mirrors the OLD
    // board). s_in_progress is implicitly cleared by the reboot.
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();  // never returns
}

void stm32_flash_start(uint8_t *fw, size_t len) {
    if (s_in_progress) {            // refuse a second concurrent request
        if (fw) free(fw);
        return;
    }
    s_fw = fw;
    s_len = len;
    s_in_progress = true;
    // Internal-RAM stack (xTaskCreate, not PinnedToCore-PSRAM) for the timing-
    // sensitive flasher. 8 KB stack; priority 6 (above stm32_task's 4).
    xTaskCreate(stm32_flash_task, "stm32_flash", 8192, NULL, 6, NULL);
}

// ============================================================
//  HTTP UPLOAD HANDLER  (POST /stm32)
// ============================================================
// Receives a raw STM32 .bin image into a PSRAM buffer, then hands it to
// stm32_flash_start() (which owns + frees the buffer and reboots when done).
// Mirrors ota.cpp's /ota handler. The G474CE has 512 KB flash, so we accept
// images in the 256 B .. 512 KB range.
#define STM32_MIN_IMG  256u
#define STM32_MAX_IMG  (512u * 1024u)
#define STM32_RECV_CHUNK 1024

static esp_err_t stm32_post_handler(httpd_req_t *req) {
    if (s_in_progress) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_sendstr(req, "STM32 flash already in progress\n");
        return ESP_OK;
    }

    int total = req->content_len;
    ESP_LOGI(TAG, "STM32 upload started (content-length=%d)", total);
    if (total < (int)STM32_MIN_IMG || total > (int)STM32_MAX_IMG) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "STM32 image size out of range (256B..512KB)\n");
        return ESP_OK;
    }

    uint8_t *buf = (uint8_t *)heap_caps_malloc(total, MALLOC_CAP_SPIRAM);
    if (!buf) buf = (uint8_t *)malloc(total);  // fall back to internal RAM
    if (!buf) {
        ESP_LOGE(TAG, "failed to allocate %d bytes for STM32 image", total);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int received = 0;
    while (received < total) {
        int to_read = total - received;
        if (to_read > STM32_RECV_CHUNK) to_read = STM32_RECV_CHUNK;
        int r = httpd_req_recv(req, (char *)(buf + received), to_read);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) {
            ESP_LOGE(TAG, "httpd_req_recv failed at %d/%d", received, total);
            free(buf);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        received += r;
    }
    ESP_LOGI(TAG, "STM32 image received (%d bytes), starting flash", received);

    // Respond BEFORE flashing: the flash reboots the ESP32, so the client must
    // get its 200 first. The browser/script then loses the connection on reboot.
    httpd_resp_sendstr(req, "STM32 image received - flashing now, device will "
                            "reboot when done.\n");

    // Hand off ownership of the buffer to the async flasher.
    stm32_flash_start(buf, received);
    return ESP_OK;
}

void stm32_flash_register_handler(httpd_handle_t server) {
    if (!server) {
        ESP_LOGW(TAG, "stm32_flash_register_handler: no HTTP server (skipped)");
        return;
    }
    httpd_uri_t uri = {
        .uri      = "/stm32",
        .method   = HTTP_POST,
        .handler  = stm32_post_handler,
        .user_ctx = NULL
    };
    esp_err_t err = httpd_register_uri_handler(server, &uri);
    if (err == ESP_OK)
        ESP_LOGI(TAG, "STM32 flash endpoint registered: POST /stm32");
    else
        ESP_LOGE(TAG, "failed to register /stm32 handler: %s",
                 esp_err_to_name(err));
}
