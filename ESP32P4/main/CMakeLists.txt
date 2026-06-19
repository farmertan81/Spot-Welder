idf_component_register(SRCS "welder_main.cpp"
                            "ui.cpp"
                            "touch_gt911.c"
                            "wifi_bridge.cpp"
                            "ota.cpp"
                            "stm32_flash.cpp"
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS ""
                       REQUIRES esp_driver_uart esp_driver_gpio
                                esp_lcd lvgl esp_timer esp_driver_i2c
                                esp_wifi nvs_flash esp_netif esp_event
                                esp_http_server mdns app_update)
