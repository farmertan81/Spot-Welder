idf_component_register(SRCS "hello_world_main.c"
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS "" 
                       REQUIRES esp_driver_uart esp_driver_gpio 
                                esp_lcd lvgl esp_timer esp_driver_i2c)
                       
