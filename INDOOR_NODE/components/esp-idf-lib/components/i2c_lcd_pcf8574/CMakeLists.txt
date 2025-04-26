idf_component_register(SRCS "i2c_lcd_pcf8574.c"
                    INCLUDE_DIRS .
                    REQUIRES "driver esp_rom i2cdev log")
