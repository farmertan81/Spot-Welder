// touch_gt911.h
// Minimal GT911 capacitive-touch driver for the CrowPanel Advance ESP32-P4 5".
// Shares the I2C master bus with the STC8 backlight controller.
//   I2C: SDA=GPIO45, SCL=GPIO46   RST=GPIO36   INT=GPIO42   addr=0x5D
// Exposes an LVGL v9 indev read callback so the ported tabview UI is touchable.
#pragma once

#include <stdbool.h>
#include "driver/i2c_master.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize GT911 on an existing i2c_master bus. Performs the INT/RST reset
// sequence to select the 0x5D address, then registers an LVGL input device
// bound to the given display. Returns true on success.
bool touch_gt911_init(i2c_master_bus_handle_t bus, lv_display_t *disp);

#ifdef __cplusplus
}
#endif
