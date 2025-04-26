idf_component_register(SRCS ./app_driver.c ./app_main.c 
                       INCLUDE_DIRS "." 
                       PRIV_REQUIRES esp_http_client nvs_flash esp_rainmaker app_network app_insights app_reset freertos
                       REQUIRES bme680 i2cdev esp_idf_lib_helpers i2c_lcd_pcf8574)

target_link_libraries(${COMPONENT_LIB} m)  