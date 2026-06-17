// touch_gt911.c — see touch_gt911.h
#include "touch_gt911.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "GT911";

// ---- Board wiring (CrowPanel Advance ESP32-P4 5") ----
#define GT911_RST_PIN   36
#define GT911_INT_PIN   42
#define GT911_I2C_ADDR  0x5D
#define GT911_I2C_HZ    400000

// Panel resolution (landscape native).
#define GT911_X_MAX     800
#define GT911_Y_MAX     480

// GT911 16-bit registers
#define GT911_REG_STATUS    0x814E  // buffer status / number of points
#define GT911_REG_POINT1    0x8150  // first touch point (8 bytes)

static i2c_master_dev_handle_t s_dev = NULL;
static bool s_is_pressed = false;  // Tracks current touch state

// Read `len` bytes from a 16-bit register address.
static esp_err_t gt911_read(uint16_t reg, uint8_t *data, size_t len)
{
    uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    return i2c_master_transmit_receive(s_dev, addr, 2, data, len,
                                       100 / portTICK_PERIOD_MS);
}

// Write a single byte to a 16-bit register address.
static esp_err_t gt911_write_u8(uint16_t reg, uint8_t val)
{
    uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), val };
    return i2c_master_transmit(s_dev, buf, 3, 100 / portTICK_PERIOD_MS);
}

// INT/RST power-on reset sequence that selects I2C address 0x5D.
static void gt911_reset_sequence(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << GT911_RST_PIN) | (1ULL << GT911_INT_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    // Hold both low.
    gpio_set_level(GT911_RST_PIN, 0);
    gpio_set_level(GT911_INT_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(11));

    // INT low while releasing RST -> address 0x5D.
    gpio_set_level(GT911_INT_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(GT911_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(6));

    // Keep INT low a bit longer, then release it as a floating input.
    gpio_set_level(GT911_INT_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(51));

    gpio_config_t int_in = {
        .pin_bit_mask = (1ULL << GT911_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&int_in);
    vTaskDelay(pdMS_TO_TICKS(50));
}

// LVGL v9 indev read callback. Latches the last coordinate while pressed so a
// touch-up event reports the correct release position.
static void gt911_lvgl_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    static int16_t last_x = 0, last_y = 0;
    static int16_t last_raw_x = 0, last_raw_y = 0;
    static uint8_t last_bytes[8] = {0};

    uint8_t status = 0;
    if (gt911_read(GT911_REG_STATUS, &status, 1) != ESP_OK) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    // Buffer-ready flag (bit7) must be set before the point count is valid.
    if (status & 0x80) {
        uint8_t points = status & 0x0F;
        if (points > 0 && points <= 5) {
            uint8_t p[8];
            if (gt911_read(GT911_REG_POINT1, p, sizeof(p)) == ESP_OK) {
                // Raw coordinates from GT911 (before any transform)
                int16_t raw_x = (int16_t)(p[1] | (p[2] << 8));
                int16_t raw_y = (int16_t)(p[3] | (p[4] << 8));
                last_raw_x = raw_x;
                last_raw_y = raw_y;
                memcpy(last_bytes, p, sizeof(last_bytes));

                // ---- CALIBRATION MODE: identity transform ----
                // We are temporarily passing the RAW digitizer values straight
                // through (no transform) and logging them, so we can read the
                // GROUND TRUTH for the four corners and compute the exact
                // mapping in one shot instead of guessing. Once we know the
                // raw values for each corner, this block gets replaced with the
                // correct fixed transform.
                int16_t x = raw_x;
                int16_t y = raw_y;

                // Clamp to display bounds
                if (x < 0) x = 0;
                if (x >= GT911_X_MAX) x = GT911_X_MAX - 1;
                if (y < 0) y = 0;
                if (y >= GT911_Y_MAX) y = GT911_Y_MAX - 1;
                last_x = x;
                last_y = y;
            }
        }
        // Clearing the status register tells GT911 we consumed this report.
        gt911_write_u8(GT911_REG_STATUS, 0);

        data->point.x = last_x;
        data->point.y = last_y;
        data->state = (points > 0) ? LV_INDEV_STATE_PRESSED
                                   : LV_INDEV_STATE_RELEASED;
        // DEBUG: log only on a press/release transition so we can confirm the
        // GT911 is delivering touches (and at what coordinates) without
        // flooding the console. Remove once touch is verified working.
        bool now_pressed = (points > 0);
        if (now_pressed != s_is_pressed) {
            // Dump all 8 raw point bytes so we can determine the exact GT911
            // byte layout (track-id vs X-low offset) once and for all.
            ESP_LOGI(TAG, "touch %s bytes=[%02X %02X %02X %02X %02X %02X %02X %02X] "
                     "(xguess=%d yguess=%d) pts=%d",
                     now_pressed ? "DOWN" : "UP  ",
                     last_bytes[0], last_bytes[1], last_bytes[2], last_bytes[3],
                     last_bytes[4], last_bytes[5], last_bytes[6], last_bytes[7],
                     last_raw_x, last_raw_y, points);
        }
        s_is_pressed = now_pressed;
    } else {
        data->point.x = last_x;
        data->point.y = last_y;
        data->state = LV_INDEV_STATE_RELEASED;
        s_is_pressed = false;
    }
}

bool touch_gt911_init(i2c_master_bus_handle_t bus, lv_display_t *disp)
{
    gt911_reset_sequence();

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = GT911_I2C_ADDR,
        .scl_speed_hz = GT911_I2C_HZ,
    };
    if (i2c_master_bus_add_device(bus, &dev_cfg, &s_dev) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GT911 device on I2C bus");
        return false;
    }

    // Probe the product-id block (0x8140) to confirm presence.
    uint8_t id[4] = {0};
    if (gt911_read(0x8140, id, 4) == ESP_OK) {
        ESP_LOGI(TAG, "GT911 product id: %c%c%c%c", id[0], id[1], id[2], id[3]);
    } else {
        ESP_LOGW(TAG, "GT911 id read failed (continuing anyway)");
    }

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, gt911_lvgl_read);
    lv_indev_set_display(indev, disp);

    ESP_LOGI(TAG, "GT911 touch initialized (addr 0x%02X, %dx%d)",
             GT911_I2C_ADDR, GT911_X_MAX, GT911_Y_MAX);
    return true;
}

bool touch_is_physically_down(void)
{
    return s_is_pressed;
}
