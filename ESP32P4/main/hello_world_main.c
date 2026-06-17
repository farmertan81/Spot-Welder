#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_timer.h"
#include "lvgl.h"
#include "driver/i2c_master.h"

static const char *TAG = "WELDER_UI";

// UART pins for STM32
#define STM32_UART_NUM      UART_NUM_1
#define STM32_TX_PIN        29
#define STM32_RX_PIN        30
#define STM32_BOOT0_PIN     31
#define BUF_SIZE            2048
#define STM32_BAUD          2000000

// Display settings (800x480 RGB LCD)
#define LCD_H_RES           800
#define LCD_V_RES           480
#define LCD_PIXEL_CLOCK_HZ  (18 * 1000 * 1000)

// RGB LCD GPIO pins (CrowPanel ESP32-P4)
#define LCD_PCLK                3
#define LCD_DE                  2
#define LCD_VSYNC              41
#define LCD_HSYNC              40

#define LCD_B0                  8
#define LCD_B1                  7
#define LCD_B2                  6
#define LCD_B3                  5
#define LCD_B4                  4

#define LCD_G0                 14
#define LCD_G1                 13
#define LCD_G2                 12
#define LCD_G3                 11
#define LCD_G4                 10
#define LCD_G5                  9

#define LCD_R0                 19
#define LCD_R1                 18
#define LCD_R2                 17
#define LCD_R3                 16
#define LCD_R4                 15  // VERIFIED: GPIO15 per schematic (was incorrectly 12 = collision with G2)


// I2C for backlight control
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_SDA_GPIO        45
#define I2C_SCL_GPIO        46
#define BACKLIGHT_I2C_ADDR  0x2F  // ← WAS 0x30, NOW 0x2F!

// LVGL
static lv_display_t *lvgl_disp = NULL;
static lv_obj_t *label_vcap, *label_temp, *label_cell1, *label_cell2, *label_cell3, *label_status;

// Welder data
typedef struct {
    float vcap;
    float temp;
    float cell1, cell2, cell3;
    int armed;
    int ready;
} welder_data_t;

static welder_data_t welder = {0};

// LVGL flush callback
static bool rgb_lcd_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

// LVGL tick timer
static void lv_tick_task(void *arg) {
    lv_tick_inc(10);
}

void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = STM32_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    
    ESP_ERROR_CHECK(uart_param_config(STM32_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(STM32_UART_NUM, STM32_TX_PIN, STM32_RX_PIN, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(STM32_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    
    ESP_LOGI(TAG, "UART initialized: 2Mbaud on TX=%d, RX=%d", STM32_TX_PIN, STM32_RX_PIN);
}

void gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STM32_BOOT0_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(STM32_BOOT0_PIN, 0);
    
    ESP_LOGI(TAG, "GPIO%d (BOOT0) set LOW", STM32_BOOT0_PIN);
}

// Global I2C handle for STC8 (needed for runtime backlight control)
static i2c_master_dev_handle_t stc8_dev_handle = NULL;

void backlight_init(void)
{
    // Create I2C master bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    
    // Add STC8 co-processor device at 0x2F
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x2F,
        .scl_speed_hz = 400000,
    };
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &stc8_dev_handle));
    
    // CRITICAL: Enable LCD panel power FIRST (register 0x18)
    uint8_t panel_power[2] = {0x18, 1};
    ESP_ERROR_CHECK(i2c_master_transmit(stc8_dev_handle, panel_power, 2, 1000 / portTICK_PERIOD_MS));
    vTaskDelay(pdMS_TO_TICKS(30));  // Wait for panel power to stabilize
    ESP_LOGI(TAG, "LCD panel power enabled (reg 0x18)");
    
    // Enable backlight power rail (register 0x1B)
    uint8_t backlight_power[2] = {0x1B, 1};
    ESP_ERROR_CHECK(i2c_master_transmit(stc8_dev_handle, backlight_power, 2, 1000 / portTICK_PERIOD_MS));
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "Backlight power enabled (reg 0x1B)");
    
    // Set PWM brightness to 0 initially (will enable after RGB panel init)
    uint8_t brightness_off[2] = {0x20, 0};
    ESP_ERROR_CHECK(i2c_master_transmit(stc8_dev_handle, brightness_off, 2, 1000 / portTICK_PERIOD_MS));
    
    ESP_LOGI(TAG, "Display power & backlight initialized via STC8 @ 0x2F");
}

void backlight_set(uint8_t brightness)
{
    if (stc8_dev_handle == NULL) {
        ESP_LOGE(TAG, "STC8 not initialized!");
        return;
    }
    
    // Set PWM brightness (register 0x20, range 0-100)
    uint8_t cmd[2] = {0x20, brightness};
    esp_err_t err = i2c_master_transmit(stc8_dev_handle, cmd, 2, 1000 / portTICK_PERIOD_MS);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Backlight PWM set to %d%%", brightness);
    } else {
        ESP_LOGE(TAG, "Failed to set backlight: %s", esp_err_to_name(err));
    }
}

void send_stm32_command(const char *cmd)
{
    char buffer[128];
    int len = snprintf(buffer, sizeof(buffer), "%s\n", cmd);
    uart_write_bytes(STM32_UART_NUM, buffer, len);
}

void parse_status_response(const char *data)
{
    const char *p;
    
    if ((p = strstr(data, "vcap=")) != NULL) {
        sscanf(p, "vcap=%f", &welder.vcap);
    }
    if ((p = strstr(data, "temp=")) != NULL) {
        sscanf(p, "temp=%f", &welder.temp);
    }
    if ((p = strstr(data, "cell1=")) != NULL) {
        sscanf(p, "cell1=%f", &welder.cell1);
    }
    if ((p = strstr(data, "cell2=")) != NULL) {
        sscanf(p, "cell2=%f", &welder.cell2);
    }
    if ((p = strstr(data, "cell3=")) != NULL) {
        sscanf(p, "cell3=%f", &welder.cell3);
    }
    if ((p = strstr(data, "armed=")) != NULL) {
        sscanf(p, "armed=%d", &welder.armed);
    }
    if ((p = strstr(data, "ready=")) != NULL) {
        sscanf(p, "ready=%d", &welder.ready);
    }
}

void read_stm32_response(void)
{
    uint8_t data[BUF_SIZE];
    int length = uart_read_bytes(STM32_UART_NUM, data, BUF_SIZE - 1, pdMS_TO_TICKS(100));
    
    if (length > 0) {
        data[length] = '\0';
        parse_status_response((char*)data);
    }
}

void update_ui(void)
{
    char buf[64];
    
    snprintf(buf, sizeof(buf), "Vcap:    %.2f V", welder.vcap);
    lv_label_set_text(label_vcap, buf);
    
    snprintf(buf, sizeof(buf), "Temp:    %.2f C", welder.temp);
    lv_label_set_text(label_temp, buf);
    
    snprintf(buf, sizeof(buf), "Cell 1:  %.2f V", welder.cell1);
    lv_label_set_text(label_cell1, buf);
    
    snprintf(buf, sizeof(buf), "Cell 2:  %.2f V", welder.cell2);
    lv_label_set_text(label_cell2, buf);
    
    snprintf(buf, sizeof(buf), "Cell 3:  %.2f V", welder.cell3);
    lv_label_set_text(label_cell3, buf);
    
    if (welder.armed && welder.ready) {
        lv_label_set_text(label_status, "Status: READY");
        lv_obj_set_style_text_color(label_status, lv_color_hex(0x00FF00), 0);
    } else if (welder.armed) {
        lv_label_set_text(label_status, "Status: ARMED");
        lv_obj_set_style_text_color(label_status, lv_color_hex(0xFFFF00), 0);
    } else {
        lv_label_set_text(label_status, "Status: IDLE");
        lv_obj_set_style_text_color(label_status, lv_color_hex(0x00FFFF), 0);
    }
}

void create_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    
    // Title
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "SPOT WELDER STATUS");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);
    
    // Data labels
    int y_pos = 100;
    int y_gap = 50;
    
    label_vcap = lv_label_create(scr);
    lv_obj_set_style_text_color(label_vcap, lv_color_hex(0x00FF00), 0);
    lv_obj_set_pos(label_vcap, 50, y_pos);
    
    label_temp = lv_label_create(scr);
    lv_obj_set_style_text_color(label_temp, lv_color_hex(0x00FF00), 0);
    lv_obj_set_pos(label_temp, 50, y_pos + y_gap);
    
    label_cell1 = lv_label_create(scr);
    lv_obj_set_style_text_color(label_cell1, lv_color_hex(0xFFFF00), 0);
    lv_obj_set_pos(label_cell1, 50, y_pos + y_gap * 2);
    
    label_cell2 = lv_label_create(scr);
    lv_obj_set_style_text_color(label_cell2, lv_color_hex(0xFFFF00), 0);
    lv_obj_set_pos(label_cell2, 50, y_pos + y_gap * 3);
    
    label_cell3 = lv_label_create(scr);
    lv_obj_set_style_text_color(label_cell3, lv_color_hex(0xFFFF00), 0);
    lv_obj_set_pos(label_cell3, 50, y_pos + y_gap * 4);
    
    label_status = lv_label_create(scr);
    lv_obj_align(label_status, LV_ALIGN_BOTTOM_MID, 0, -30);
    
    update_ui();
}

void lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_timer_handler();
    }
}

void stm32_comm_task(void *arg)
{
    ESP_LOGI(TAG, "Starting STM32 communication task");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    while (1) {
        send_stm32_command("STATUS");
        read_stm32_response();
        update_ui();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "===== ESP32-P4 Welder Display =====");
    
    // Initialize hardware
    gpio_init();
    backlight_init();
    uart_init();
    
    // Initialize LVGL
    lv_init();
    
    // Allocate draw buffers (smaller, using internal RAM for now)
void *buf1 = heap_caps_malloc(LCD_H_RES * 50 * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
assert(buf1);
void *buf2 = heap_caps_malloc(LCD_H_RES * 50 * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
assert(buf2);
ESP_LOGI(TAG, "Allocated LVGL buffers: %d bytes each", LCD_H_RES * 50 * sizeof(lv_color_t));
    
    // RGB panel configuration
    esp_lcd_rgb_panel_config_t panel_conf = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .data_width = 16,
        .de_gpio_num = LCD_DE,
        .pclk_gpio_num = LCD_PCLK,
        .vsync_gpio_num = LCD_VSYNC,
        .hsync_gpio_num = LCD_HSYNC,
        .data_gpio_nums = {
            LCD_B0, LCD_B1, LCD_B2, LCD_B3, LCD_B4,  // B0-B4
            LCD_G0, LCD_G1, LCD_G2, LCD_G3, LCD_G4, LCD_G5,  // G0-G5
            LCD_R0, LCD_R1, LCD_R2, LCD_R3, LCD_R4,  // R0-R4
        },
        .timings = {
            .pclk_hz = LCD_PIXEL_CLOCK_HZ,
            .h_res = LCD_H_RES,
            .v_res = LCD_V_RES,
            .hsync_back_porch = 8,      // FIXED: Was 30, now matches Elecrow official (bsp_illuminate.h line 47)
            .hsync_front_porch = 8,     // FIXED: Was 210, now matches Elecrow official (bsp_illuminate.h line 48)
            .hsync_pulse_width = 4,
            .vsync_back_porch = 16,     // FIXED: Was 4, now matches Elecrow official (bsp_illuminate.h line 50)
            .vsync_front_porch = 16,    // FIXED: Was 4, now matches Elecrow official (bsp_illuminate.h line 51)
            .vsync_pulse_width = 4,
        },
        .flags.fb_in_psram = 1,
    };
    
    esp_lcd_panel_handle_t panel_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_conf, &panel_handle));
    ESP_LOGI(TAG, "RGB LCD panel created");
    
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_LOGI(TAG, "RGB LCD initialized: %dx%d", LCD_H_RES, LCD_V_RES);
    
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_LOGI(TAG, "Display ON");
    
    // MATCH ELECROW OFFICIAL: Enable backlight AFTER panel init (with small delay)
    vTaskDelay(pdMS_TO_TICKS(100));
    backlight_set(100);
    
    // Create LVGL display
    lvgl_disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_flush_cb(lvgl_disp, NULL);  // RGB direct mode - no flush needed
    lv_display_set_buffers(lvgl_disp, buf1, buf2, LCD_H_RES * 50 * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
    
    // Register vsync callback
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = rgb_lcd_on_vsync_event,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, lvgl_disp));
    
    // Create UI
    create_ui();
    
    ESP_LOGI(TAG, "UI created!");
    
    // Start tasks
    xTaskCreate(lvgl_port_task, "lvgl", 8192, NULL, 5, NULL);
    xTaskCreate(stm32_comm_task, "stm32", 4096, NULL, 4, NULL);
    
    // Start LVGL tick timer
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "lv_tick"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 10000));
    
    ESP_LOGI(TAG, "System initialized - Display should show welder data!");
}