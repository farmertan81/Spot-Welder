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
#include <strings.h>   // strcasecmp() for the ?method= query parse
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_cpu.h"             // esp_cpu_get_cycle_count() for bit-bang timing
#include "esp_private/esp_clk.h" // esp_clk_cpu_freq()
#include "esp_rom_gpio.h"        // esp_rom_gpio_connect_out_signal()
#include "esp_rom_sys.h"         // esp_rom_delay_us()
#include "soc/gpio_sig_map.h"    // SIG_GPIO_OUT_IDX
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

// ROM-entry method selector, set from the /stm32?method= query before each flash:
//   ENTRY_AUTO (default) = try the 2-wire software jump first, fall back to the
//                          BOOT0+NRST hardware pulse if the ROM stays silent.
//   ENTRY_SW             = software 2-wire jump ONLY (no BOOT0/NRST). For proving
//                          the wire-free path in isolation.
//   ENTRY_HW             = BOOT0+NRST hardware pulse ONLY. For proving the
//                          hardware path in isolation.
enum { ENTRY_AUTO = 0, ENTRY_SW = 1, ENTRY_HW = 2 };
static volatile int s_entry_method = ENTRY_AUTO;

// UART event queue — lets us detect PARITY/FRAME errors on RX. This is the
// decisive diagnostic: if the STM32 ROM IS responding but the byte arrives with
// a parity/framing mismatch, the driver silently DROPS it and it looks like
// "silence". The event queue still reports the error, proving the STM32 is alive
// and the problem is a config/baud mismatch (not a dead link).
static QueueHandle_t s_uart_evt_q = NULL;

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

// ---- SOFTWARE BIT-BANG TX (decisive test for a P4 HW-UART parity-pad bug) ----
// The HW-UART loopback proves the P4 encodes 8E1 *internally*, and the app proves
// GPIO29 drives the pad fine at 1Mbaud *8N1* (no parity). The ONE thing never
// proven is that the P4 emits a correct EVEN-PARITY frame on the physical pad.
// This function bypasses the UART peripheral entirely: it detaches GPIO29 from
// the UART matrix, manually toggles the pin to clock out one 8E1 frame (start +
// 8 data LSB-first + even-parity + stop) with cycle-accurate timing, then
// reattaches the UART so the HW RX path can capture the ROM's ACK.
//   * If the ROM now ACKs -> the P4 HW-UART parity output on the pad WAS broken,
//     and bit-banging is the fix (we'll convert the whole protocol to it).
//   * If still silent -> the ESP TX path is definitively NOT the problem.
static IRAM_ATTR void stmBitbangByte8E1(uint8_t byte) {
    const uint32_t cyc_per_bit = (uint32_t)(esp_clk_cpu_freq() / 115200);

    // Build the 11 line levels: start(0), d0..d7 (LSB first), even parity, stop(1).
    int levels[11];
    levels[0] = 0;
    int ones = 0;
    for (int i = 0; i < 8; i++) {
        int bit = (byte >> i) & 1;
        levels[1 + i] = bit;
        ones += bit;
    }
    levels[9]  = ones & 1;   // even parity: make total number of 1s even
    levels[10] = 1;          // stop bit

    // Detach the UART TXD signal from GPIO29 and take manual GPIO control.
    esp_rom_gpio_connect_out_signal(STM_TX_PIN, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction((gpio_num_t)STM_TX_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)STM_TX_PIN, 1);   // idle high
    esp_rom_delay_us(50);                         // let the line settle high

    // Clock out the frame with interrupts off so no ISR jitters a bit edge.
    // 11 bits @115200 ≈ 95µs — safe to keep interrupts disabled this long.
    portDISABLE_INTERRUPTS();
    uint32_t t = esp_cpu_get_cycle_count();
    for (int i = 0; i < 11; i++) {
        gpio_set_level((gpio_num_t)STM_TX_PIN, levels[i]);
        t += cyc_per_bit;
        while ((int32_t)(esp_cpu_get_cycle_count() - t) < 0) { /* spin */ }
    }
    gpio_set_level((gpio_num_t)STM_TX_PIN, 1);   // hold idle high after stop
    portENABLE_INTERRUPTS();

    // Reattach the UART TXD signal so the normal driver owns GPIO29 again.
    uart_set_pin(STM_BOOT_UART, STM_TX_PIN, STM_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// Bootloader auto-baud handshake: send 0x7F until the device answers.
static IRAM_ATTR bool stmSync() {
    const int kAttempts = 10;
    for (int i = 0; i < kAttempts; i++) {
        int drained = stmDrainRx();
        if (drained > 0)
            ESP_LOGI(TAG, "drained %d stale byte(s) before sync", drained);
        // First half of the attempts: HW-UART TX (the proven-broken path, kept
        // for direct comparison). Second half: SOFTWARE BIT-BANG 8E1 TX — bypasses
        // the UART peripheral's parity logic to test for a P4 parity-pad bug.
        bool use_bitbang = (i >= kAttempts / 2);
        ESP_LOGI(TAG, "sync attempt %d/%d: sending 0x7F (%s)", i + 1, kAttempts,
                 use_bitbang ? "BIT-BANG 8E1" : "HW-UART 8E1");
        if (use_bitbang) {
            stmBitbangByte8E1(0x7F);
        } else {
            uart_write_bytes(STM_BOOT_UART, "\x7F", 1);
            uart_wait_tx_done(STM_BOOT_UART, pdMS_TO_TICKS(100));
        }
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

// Read Memory (0x11): read up to 256 bytes (len = 1..256) from `addr` into
// `out`. Returns true if the ROM ACKed every stage and all bytes were received.
// Used by the post-write VERIFY pass to read flash back and compare it against
// the image we just programmed — the definitive proof the flash is correct.
static IRAM_ATTR bool stmReadChunk(uint32_t addr, uint8_t *out, int len) {
    if (len < 1 || len > 256) return false;
    if (!stmSendCmd(0x11)) return false;                 // Read Memory cmd + ACK
    uint8_t a[4] = {(uint8_t)(addr >> 24), (uint8_t)(addr >> 16),
                    (uint8_t)(addr >> 8), (uint8_t)addr};
    uint8_t acs = a[0] ^ a[1] ^ a[2] ^ a[3];
    uint8_t addrpkt[5] = {a[0], a[1], a[2], a[3], acs};
    stmWrite(addrpkt, 5);
    stmFlushTx();
    if (!stmWaitAck(800)) return false;                  // address ACK

    uint8_t n = (uint8_t)(len - 1);                      // device expects (count-1)
    uint8_t pkt[2] = {n, (uint8_t)(n ^ 0xFF)};           // N + its complement
    stmWrite(pkt, 2);
    stmFlushTx();
    if (!stmWaitAck(800)) return false;                  // N ACK -> data follows

    // Read exactly len bytes back (per-byte timeout keeps us from hanging).
    for (int i = 0; i < len; i++) {
        int b = stmReadByte(800);
        if (b < 0) return false;
        out[i] = (uint8_t)b;
    }
    return true;
}

// Go (0x21): jump to and run the application at `addr`.
// NOTE: No longer used for launch (we now pulse NRST to cleanly boot the new app
// via hardware reset). Kept for reference / optional AN3155 Go support.
__attribute__((unused))
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
    // 115200 is the proven AN3155 default (old board used it). Baud is NOT the
    // problem — 57600 failed identically — so we stay on the standard value while
    // the event-queue diagnostic isolates the real cause.
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
    
    // CRITICAL: Set TX pin to MAXIMUM drive strength. The P4's default drive (5mA)
    // may be too weak for reliable 8E1 signaling, especially if there's capacitance
    // on the wire or a long PCB trace. The loopback test passes (internal routing),
    // but the external pad might need stronger drive for the parity bit to reach
    // the STM32 cleanly. Drive strength 3 = ~20mA on P4 (per TRM Table 6-5).
    gpio_set_drive_capability((gpio_num_t)STM_TX_PIN, GPIO_DRIVE_CAP_3);
    
    // Install WITH an event queue (depth 30) so we can detect UART_PARITY_ERR /
    // UART_FRAME_ERR — i.e. whether the STM32 is replying but we're dropping the
    // byte on a config mismatch. s_uart_evt_q is drained in the post-fail probe.
    uart_driver_install(STM_BOOT_UART, 4096, 0, 30, &s_uart_evt_q, 0);

    uart_flush_input(STM_BOOT_UART);
    // Clear any stale events from the previous driver instance.
    if (s_uart_evt_q) xQueueReset(s_uart_evt_q);
}

// Internal UART loopback self-test. Uses the ESP32 hardware loopback (TX wired
// to RX internally, no external wiring) to prove the UART can SEND and RECEIVE
// a 0x7F byte at the CURRENT config (57600/115200 8E1). This isolates an ESP-side UART
// config bug from an external (wiring / STM32) problem:
//   * reads back 0x7F -> the ESP UART 8E1 path is perfect; the failure is external
//     (signal integrity on the wire, or the STM32 ROM locked onto USB/I2C/SPI).
//   * reads back garbage / nothing -> the ESP's own 8E1 config is broken.
// Must be called while the driver is installed at the config we want to test.
static void uart_loopback_selftest(void) {
    uint32_t baud = 0;
    uart_get_baudrate(STM_BOOT_UART, &baud);
    ESP_LOGI(TAG, "UART loopback self-test @%lu 8E1 (internal, no wires)...", baud);
    uart_flush_input(STM_BOOT_UART);
    // Enable internal TX->RX loopback.
    uart_set_loop_back(STM_BOOT_UART, true);
    vTaskDelay(pdMS_TO_TICKS(5));

    const uint8_t tx = 0x7F;
    uart_write_bytes(STM_BOOT_UART, &tx, 1);
    uart_wait_tx_done(STM_BOOT_UART, pdMS_TO_TICKS(100));
    uint8_t rx = 0;
    int n = uart_read_bytes(STM_BOOT_UART, &rx, 1, pdMS_TO_TICKS(200));

    // Disable loopback so the real flash can drive the external pins.
    uart_set_loop_back(STM_BOOT_UART, false);
    uart_flush_input(STM_BOOT_UART);

    if (n == 1 && rx == 0x7F) {
        ESP_LOGI(TAG, "  LOOPBACK PASS: sent 0x7F, read 0x7F back. ESP UART 8E1 is "
                      "GOOD -> failure is EXTERNAL (wire signal or STM32 ROM on "
                      "USB/I2C/SPI, not USART1).");
    } else if (n == 1) {
        ESP_LOGE(TAG, "  LOOPBACK FAIL: sent 0x7F, read 0x%02X back. ESP UART 8E1 "
                      "config is CORRUPT (framing/parity bug on the P4).", rx);
    } else {
        ESP_LOGE(TAG, "  LOOPBACK FAIL: sent 0x7F, read NOTHING back. ESP UART RX "
                      "path is dead at 8E1 (driver/config bug on the P4).");
    }
}

// Drain the UART event queue and report what happened on the RX line during sync.
// THE decisive diagnostic: distinguishes "STM32 is replying but config mismatch"
// from "STM32 is genuinely silent".
static void report_uart_events(void) {
    if (!s_uart_evt_q) {
        ESP_LOGW(TAG, "EVENTS: no event queue installed");
        return;
    }
    int parity = 0, frame = 0, data = 0, fifo_ovf = 0, buf_full = 0, brk = 0, other = 0;
    uart_event_t ev;
    // Non-blocking drain of everything queued during the sync attempts.
    while (xQueueReceive(s_uart_evt_q, &ev, 0) == pdTRUE) {
        switch (ev.type) {
            case UART_DATA:        data++;     break;
            case UART_PARITY_ERR:  parity++;   break;
            case UART_FRAME_ERR:   frame++;    break;
            case UART_FIFO_OVF:    fifo_ovf++; break;
            case UART_BUFFER_FULL: buf_full++; break;
            case UART_BREAK:       brk++;      break;
            default:               other++;    break;
        }
    }
    ESP_LOGI(TAG, "EVENTS during sync: DATA=%d PARITY_ERR=%d FRAME_ERR=%d "
                  "FIFO_OVF=%d BUF_FULL=%d BREAK=%d other=%d",
             data, parity, frame, fifo_ovf, buf_full, brk, other);
    if (parity > 0 || frame > 0) {
        ESP_LOGW(TAG, "  >> STM32 IS RESPONDING but bytes are dropped on "
                      "PARITY/FRAME mismatch -> the ROM auto-baud locked to a "
                      "DIFFERENT baud/format than the ESP. This is a baud/timing "
                      "lock issue, NOT a dead link.");
    } else if (data == 0) {
        ESP_LOGW(TAG, "  >> Zero RX activity of ANY kind (no good bytes, no error "
                      "frames). The STM32 ROM is genuinely not transmitting -> it "
                      "did not accept our 0x7F (TX not reaching PA10) OR it is on a "
                      "different bootloader interface.");
    }
}

// ---------------------------------------------------------------------------
// Bootloader ENTRY strategies. Each returns true if, after the entry attempt,
// the STM32 ROM bootloader answers our 0x7F sync at 115200 8E1.
//
// PRIMARY = software jump. SECONDARY/fallback = hardware BOOT0+NRST pulse.
//
// Why software is primary: on the STM32G4 the factory-default option bit
// nBOOT_SEL=1 makes the chip IGNORE the physical BOOT0 *pin* — boot mode is
// taken from the nBOOT0 option bit instead. So strapping GPIO31->PB8 HIGH does
// nothing on a stock chip and the ROM never starts (exactly what the field log
// showed: NRST/BOOT0/UART all test good, yet the ROM is silent). The running
// app, however, can jump to the ROM in software: it receives "BOOTLOADER",
// calls jumpToBootloader() and lands in the ROM on USART1 with NO reset, NO
// TAMP magic and NO dependence on the BOOT0 pin. This is the method that worked
// on the old board, so we try it first and only fall back to the GPIO pulse if
// the app is unresponsive (e.g. already bricked).
// ---------------------------------------------------------------------------

// ============================================================
//  BOOT0 / NRST line ownership  (avoid fighting an attached ST-Link)
// ============================================================
// The STM32's NRST and BOOT0 lines are shared: the on-board ST-Link/SWD debugger
// also drives NRST. If the ESP holds these as strong push-pull outputs all the
// time, the ST-Link can't connect ("DEV_TARGET_HELD_UNDER_RESET") and an attempt
// to flash over SWD can be corrupted mid-program. So the ESP keeps both pins
// HIGH-IMPEDANCE (input) whenever it is NOT mid-flash, and only briefly drives
// them during its own hardware-entry / launch pulses. The board's own pulls give
// the correct idle state with the ESP off the lines:
//   BOOT0 (PB8): 10k board pulldown  -> floats LOW  -> app mode
//   NRST       : STM32 internal pull-up -> floats HIGH -> not in reset
// Drive a defined level, THEN switch to output, to avoid a glitch on the line.
static inline void stm_pin_release(int pin) {
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);   // Hi-Z: hand the line back
}
static inline void stm_pin_drive(int pin, int level) {
    gpio_set_level((gpio_num_t)pin, level);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level((gpio_num_t)pin, level);
}
// Put both strap lines back to Hi-Z so an attached ST-Link can own NRST/BOOT0.
static inline void stm_release_strap_lines(void) {
    stm_pin_release(STM_BOOT0_PIN);
    stm_pin_release(STM_NRST_PIN);
}

// PRIMARY: ask the running app (1Mbaud 8N1) to jump straight into the ROM.
static bool stm32_enter_bootloader_software(void) {
    ESP_LOGI(TAG, "SOFTWARE bootloader entry: sending 'BOOTLOADER' to the running "
                  "app @%d 8N1 (app jumps directly to ROM, no reset/BOOT0 pin)...",
             STM_APP_BAUD);

    // Talk to the app at its normal link config.
    uart_wait_tx_done(STM_BOOT_UART, pdMS_TO_TICKS(50));
    uart_set_baudrate(STM_BOOT_UART, STM_APP_BAUD);
    uart_set_word_length(STM_BOOT_UART, UART_DATA_8_BITS);
    uart_set_parity(STM_BOOT_UART, UART_PARITY_DISABLE);
    uart_set_stop_bits(STM_BOOT_UART, UART_STOP_BITS_1);
    uart_flush_input(STM_BOOT_UART);

    // Keep BOOT0/NRST fully Hi-Z: the software path never touches them, and
    // releasing the lines means we don't fight an attached ST-Link. Board pulls
    // hold BOOT0 LOW (app mode) and NRST HIGH (running) on their own.
    stm_release_strap_lines();

    // The app accepts "BOOTLOADER" or "CMD,BOOTLOADER" (CR/LF tolerant). Send it
    // a couple of times in case a status line is mid-transmission.
    const char *cmd = "BOOTLOADER\r\n";
    for (int i = 0; i < 2; i++) {
        uart_write_bytes(STM_BOOT_UART, cmd, strlen(cmd));
        uart_wait_tx_done(STM_BOOT_UART, pdMS_TO_TICKS(30));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // The app prints ACK,BOOTLOADER + a debug line, waits ~50ms, then tears down
    // its clocks/peripherals (HAL_DeInit) and jumps. Give it generous time to
    // land in the ROM and start polling USART1 before we switch to 8E1.
    vTaskDelay(pdMS_TO_TICKS(200));

    // Now talk to the ROM at AN3155's 115200 8E1 and try to sync.
    switch_uart_to_bootloader();
    uart_loopback_selftest();          // one-time: prove ESP 8E1 path is good
    // CRITICAL: the app sent ACK,BOOTLOADER + debug lines at 1Mbaud 8N1 BEFORE
    // jumping. Those bytes are still arriving into the RX buffer (the driver just
    // switched to 8E1, so they'll be decoded as garbage). Let ALL of the app's
    // output arrive, then drain it in one shot so the ROM's 0x79 ACK is clean.
    vTaskDelay(pdMS_TO_TICKS(100));    // let the last app bytes trickle in
    stmDrainRx();
    if (s_uart_evt_q) xQueueReset(s_uart_evt_q);
    bool synced = stmSync();
    ESP_LOGI(TAG, "  software-entry sync: %s",
             synced ? "ACK — ROM is up on USART1!" : "no response");
    return synced;
}

// FALLBACK: hardware BOOT0 strap + NRST pulse. Only effective if the STM32
// option bytes have nBOOT_SEL=0 (BOOT0 pin honored). On a stock G4 (nBOOT_SEL=1)
// this cannot work — the chip ignores the pin. Kept for bricked-app recovery on
// boards configured for pin-based boot.
static bool stm32_enter_bootloader_hardware(void) {
    ESP_LOGI(TAG, "HARDWARE bootloader entry: BOOT0 HIGH + NRST pulse "
                  "(GPIO%d->BOOT0, GPIO%d->NRST)...", STM_BOOT0_PIN, STM_NRST_PIN);
    uart_flush_input(STM_BOOT_UART);

    // a) CLAIM the strap lines (they idle Hi-Z so an ST-Link can use them), then
    //    drive BOOT0 HIGH and let the strap settle BEFORE reset is released.
    stm_pin_drive(STM_BOOT0_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "  BOOT0 driven HIGH, readback=%d",
             gpio_get_level((gpio_num_t)STM_BOOT0_PIN));

    // b) Pulse NRST LOW->HIGH. BOOT0 stays HIGH so the chip would latch "boot
    //    from System memory" at the reset edge (IF the pin is honored).
    stm_pin_drive(STM_NRST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level((gpio_num_t)STM_NRST_PIN, 1);
    // Multi-sample NRST as it comes back up (RC cap on the WeAct board).
    int nrst_rb = 0, nrst_rise_ms = -1;
    for (int i = 0; i < 16; i++) {
        nrst_rb = gpio_get_level((gpio_num_t)STM_NRST_PIN);
        if (nrst_rb == 1) { nrst_rise_ms = i * 2; break; }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    int boot0_rb = gpio_get_level((gpio_num_t)STM_BOOT0_PIN);
    if (nrst_rise_ms >= 0)
        ESP_LOGI(TAG, "  NRST released: rose to HIGH after ~%dms (RC cap, OK). "
                      "BOOT0 readback=%d (want 1)", nrst_rise_ms, boot0_rb);
    else
        ESP_LOGE(TAG, "  NRST STUCK LOW >32ms (readback=%d) -> short/clamp or the "
                      "chip is holding its own reset; board/wiring fault.", nrst_rb);

    // c) Brief window for the ROM to come up, then sync at 8E1.
    vTaskDelay(pdMS_TO_TICKS(10));
    switch_uart_to_bootloader();
    stmDrainRx();
    if (s_uart_evt_q) xQueueReset(s_uart_evt_q);
    bool synced = stmSync();
    ESP_LOGI(TAG, "  hardware-entry sync: %s",
             synced ? "ACK — ROM is up!" : "no response (pin likely ignored: "
             "nBOOT_SEL=1 on a stock G4 -> use software entry)");
    return synced;
}

// Worker: programs the STM32 from the in-RAM image. Returns true on success and
// writes a short result string to `msg`. Does NOT free the buffer or reboot —
// the task wrapper owns that.
static bool flash_worker(const uint8_t *fw, size_t len, char *msg, size_t msgn) {
    // 1) Park stm32_task and quiet the LCD bus (frees the UART + PSRAM bandwidth).
    welder_prep_stm32_flash();

    show_firmware_progress("STM32", 0, "Entering bootloader...");

    // 2) ENTER the ROM bootloader. Which path we use depends on s_entry_method,
    //    set from the /stm32?method= query so each path can be proven on its own:
    //      ENTRY_SW  -> 2-wire software jump ONLY (no BOOT0/NRST).
    //      ENTRY_HW  -> BOOT0+NRST hardware pulse ONLY.
    //      ENTRY_AUTO-> software jump first, hardware pulse as fallback (default).
    bool entered = false;
    if (s_entry_method == ENTRY_SW) {
        ESP_LOGI(TAG, "Entry method = SOFTWARE-ONLY (2-wire, no BOOT0/NRST)");
        entered = stm32_enter_bootloader_software();
    } else if (s_entry_method == ENTRY_HW) {
        ESP_LOGI(TAG, "Entry method = HARDWARE-ONLY (BOOT0+NRST pulse)");
        entered = stm32_enter_bootloader_hardware();
    } else {
        ESP_LOGI(TAG, "Entry method = AUTO (software first, hardware fallback)");
        entered = stm32_enter_bootloader_software();
        if (!entered) {
            ESP_LOGW(TAG, "Software ROM entry did not sync -> trying HARDWARE "
                          "BOOT0+NRST fallback (only works if option bytes have "
                          "nBOOT_SEL=0)...");
            entered = stm32_enter_bootloader_hardware();
        }
    }


    bool ok = false;
    const char *err = NULL;
    do {
        if (!entered) {
            err = "STM32 bootloader did not respond";
            // DECISIVE: report parity/frame errors captured during the 10 sync
            // attempts. This tells us if the STM32 replied but we dropped it.
            report_uart_events();
            // POST-FAILURE DIAGNOSTIC. The hardware reset should have forced the STM32
            // into the ROM bootloader, but if the sync failed we need to determine WHY.
            // Two possibilities:
            //   A) STM32 IS in ROM bootloader (reset worked), but UART config/wiring issue
            //   B) STM32 did NOT reset (NRST wire fault), app still running at 1Mbaud 8N1
            // Test: listen at 8E1 for ROM traffic, then flip to 8N1 for app traffic.
            
            // 1) Listen at 8E1 (ROM bootloader config) for any response
            uart_flush_input(STM_BOOT_UART);
            char rp8e1[256];
            int n8e1 = 0;
            for (int i = 0; i < 10 && n8e1 < (int)sizeof(rp8e1) - 1; i++) {
                int n = uart_read_bytes(STM_BOOT_UART, (uint8_t *)rp8e1 + n8e1,
                                        sizeof(rp8e1) - 1 - n8e1, pdMS_TO_TICKS(100));
                if (n > 0) n8e1 += n;
            }
            rp8e1[n8e1 > 0 ? n8e1 : 0] = '\0';
            
            // 2) Switch back to 8N1 (app config) and ACTIVELY POLL the app.
            //    CRITICAL FIX: the app does NOT broadcast unsolicited — it only
            //    answers a "STATUS" command. The old passive listen heard nothing
            //    even if the app was alive (nobody polled it), so it could NOT
            //    distinguish "app running" from "app gone". We now SEND STATUS at
            //    1Mbaud 8N1 (exact app config) and watch for a reply. This is the
            //    decisive test for whether the GPIO reset actually entered the
            //    bootloader during a WIRELESS flash (physical-button DFU success
            //    does NOT prove the ESP's GPIO31/GPIO32 control works).
            uart_wait_tx_done(STM_BOOT_UART, pdMS_TO_TICKS(50));
            uart_set_baudrate(STM_BOOT_UART, 1000000);
            uart_set_word_length(STM_BOOT_UART, UART_DATA_8_BITS);
            uart_set_parity(STM_BOOT_UART, UART_PARITY_DISABLE);
            uart_set_stop_bits(STM_BOOT_UART, UART_STOP_BITS_1);
            uart_flush_input(STM_BOOT_UART);
            char rp8n1[256];
            int n8n1 = 0;
            // Poll 5x: send STATUS\n, then listen ~120ms for the reply.
            for (int poll = 0; poll < 5 && n8n1 < (int)sizeof(rp8n1) - 1; poll++) {
                const char *q = "STATUS\n";
                uart_write_bytes(STM_BOOT_UART, q, strlen(q));
                uart_wait_tx_done(STM_BOOT_UART, pdMS_TO_TICKS(20));
                for (int i = 0; i < 6 && n8n1 < (int)sizeof(rp8n1) - 1; i++) {
                    int n = uart_read_bytes(STM_BOOT_UART, (uint8_t *)rp8n1 + n8n1,
                                            sizeof(rp8n1) - 1 - n8n1, pdMS_TO_TICKS(20));
                    if (n > 0) n8n1 += n;
                }
                if (n8n1 > 0) break;  // got a reply, no need to keep polling
            }
            rp8n1[n8n1 > 0 ? n8n1 : 0] = '\0';
            
            // 3) Report results. ANY reply to the active STATUS poll @1Mbaud 8N1
            //    means the app firmware is alive and answering — i.e. the chip is
            //    NOT in the ROM bootloader. (We poll with the exact app config, so
            //    a reply is unambiguous proof the app is running.)
            if (n8n1 > 0) {
                ESP_LOGE(TAG, "DIAGNOSTIC: STM32 APP IS STILL RUNNING — it answered our "
                              "STATUS poll with %d bytes @1Mbaud 8N1.", n8n1);
                ESP_LOGE(TAG, "  >> The chip did NOT enter the ROM bootloader. The GPIO "
                              "reset (BOOT0=GPIO31 / NRST=GPIO32) did not take effect "
                              "during this WIRELESS flash.");
                ESP_LOGE(TAG, "  >> NOTE: USB-DFU success with the PHYSICAL boot/reset "
                              "buttons does NOT prove the ESP's GPIO control works. "
                              "Verify GPIO31->PB8(BOOT0) and GPIO32->NRST physically, "
                              "AND that the app isn't holding/overriding those pins.");
                ESP_LOGW(TAG, "  App reply: %.*s", (n8n1 < 96 ? n8n1 : 96), rp8n1);
            } else if (n8e1 > 0) {
                ESP_LOGI(TAG, "DIAGNOSTIC: heard %d bytes @115200 8E1 (ROM config), but "
                              "no valid ACK. First 32 chars: [%.*s]  Hex:",
                         n8e1, (n8e1 < 32 ? n8e1 : 32), rp8e1);
                for (int i = 0; i < n8e1 && i < 16; i++)
                    printf(" %02X", (uint8_t)rp8e1[i]);
                printf("\n");
                ESP_LOGE(TAG, "  Likely: TX/RX swapped, or ROM defaulted to USB/I2C/SPI "
                              "instead of USART1");
            } else {
                ESP_LOGE(TAG, "DIAGNOSTIC: complete silence — no ROM reply @8E1 AND the "
                              "app did NOT answer an active STATUS poll @1Mbaud 8N1.");
                ESP_LOGE(TAG, "  >> The app is GONE (so the reset DID happen) but the ROM "
                              "bootloader is not answering on USART1 PA9/PA10. The chip is "
                              "in the bootloader on a DIFFERENT interface, OR our 0x7F is "
                              "not reaching PA10. Watch the BIT-BANG sync attempts (6-10): "
                              "if those ACK, the P4 HW-UART parity output was the culprit.");
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

        // ---- VERIFY PASS: read the flash back and compare to the image ----
        // Per-chunk ACKs prove the ROM ACCEPTED each packet, not that the bytes
        // landed correctly in flash. Reading it back and comparing is the
        // definitive proof. We read in 256-byte chunks and memcmp against `fw`.
        show_firmware_progress("STM32", 96, "Verifying...");
        ESP_LOGI(TAG, "VERIFY: reading %u bytes back from flash to compare...",
                 (unsigned)len);
        uint32_t vaddr = STM32_FLASH_BASE;
        size_t vdone = 0;
        uint8_t vbuf[256];
        bool verr = false;
        while (vdone < len) {
            size_t remain = len - vdone;
            int r = (remain > sizeof(vbuf)) ? (int)sizeof(vbuf) : (int)remain;
            // Read Memory (0x11) is byte-granular, so read exactly `r` bytes.
            if (!stmReadChunk(vaddr, vbuf, r)) {
                ESP_LOGE(TAG, "VERIFY: read-back FAILED at offset %u (ROM did not "
                              "return data)", (unsigned)vdone);
                verr = true; break;
            }
            if (memcmp(vbuf, fw + vdone, r) != 0) {
                // Find the first differing byte for a precise diagnostic.
                int off = 0;
                while (off < r && vbuf[off] == fw[vdone + off]) off++;
                ESP_LOGE(TAG, "VERIFY: MISMATCH at 0x%08lX (offset %u): "
                              "flash=0x%02X expected=0x%02X",
                         (unsigned long)(vaddr + off), (unsigned)(vdone + off),
                         vbuf[off], fw[vdone + off]);
                verr = true; break;
            }
            vaddr += r;
            vdone += r;
            vTaskDelay(1);  // feed the WDT / yield
        }
        if (verr) { err = "STM32 verify failed (flash does not match image)"; break; }
        ESP_LOGI(TAG, "VERIFY OK: all %u bytes read back match the image. "
                      "Flash is confirmed correct.", (unsigned)len);
        ok = true;
    } while (0);

    // 5) Release BOOT0 to Hi-Z: the board's 10k pulldown holds it LOW so the next
    //    reset boots the (new) application, not ROM. Releasing (vs driving low)
    //    also hands the line back to any attached ST-Link. Done unconditionally so
    //    a failed flash never leaves the STM32 in DFU.
    stm_pin_release(STM_BOOT0_PIN);
    vTaskDelay(pdMS_TO_TICKS(5));  // let the BOOT0 strap settle LOW before reset

    // 6) Launch the freshly written app with a CLEAN HARDWARE RESET (BOOT0 is now
    //    LOW, so the chip boots from main flash 0x08000000). A NRST pulse is far
    //    more reliable than the AN3155 "Go" command, which can leave ROM-configured
    //    peripherals/clocks in a bad state for the new app. We drive NRST LOW to
    //    assert reset, then RELEASE to Hi-Z so the STM32's internal pull-up brings
    //    it back HIGH. Done on success only — on failure we leave the chip in ROM
    //    so the ESP reboot below can retry cleanly.
    if (ok) {
        show_firmware_progress("STM32", 98, "Launching...");
        stm_pin_drive(STM_NRST_PIN, 0);    // assert reset
        vTaskDelay(pdMS_TO_TICKS(20));
        stm_pin_release(STM_NRST_PIN);     // release: pull-up raises NRST HIGH
        ESP_LOGI(TAG, "New app launched via NRST pulse (BOOT0 released LOW).");
    }

    // 7) ALWAYS finish with BOTH strap lines Hi-Z so an attached ST-Link/SWD
    //    debugger can fully own NRST/BOOT0 (no DEV_TARGET_HELD_UNDER_RESET, no
    //    corrupted SWD programming) even after a failed flash.
    stm_release_strap_lines();

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

    // Parse the optional ?method=sw|hw|auto query so each ROM-entry path can be
    // proven independently. Anything missing/unrecognised falls back to AUTO.
    s_entry_method = ENTRY_AUTO;
    {
        size_t qlen = httpd_req_get_url_query_len(req) + 1;
        if (qlen > 1 && qlen < 128) {
            char qbuf[128];
            if (httpd_req_get_url_query_str(req, qbuf, qlen) == ESP_OK) {
                char val[16];
                if (httpd_query_key_value(qbuf, "method", val, sizeof(val)) == ESP_OK) {
                    if (strcasecmp(val, "sw") == 0 || strcasecmp(val, "software") == 0)
                        s_entry_method = ENTRY_SW;
                    else if (strcasecmp(val, "hw") == 0 || strcasecmp(val, "hardware") == 0)
                        s_entry_method = ENTRY_HW;
                    else
                        s_entry_method = ENTRY_AUTO;
                }
            }
        }
    }
    ESP_LOGI(TAG, "STM32 flash entry method = %s",
             s_entry_method == ENTRY_SW ? "SOFTWARE-ONLY" :
             s_entry_method == ENTRY_HW ? "HARDWARE-ONLY" : "AUTO");

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
