idf_component_register(SRCS "driver.cpp" "main.cpp"
                    PRIV_REQUIRES nvs_flash esp_rainmaker app_network app_insights app_reset freertos esp_netif  
                    REQUIRES arduino mq135 bme680 i2cdev esp_idf_lib_helpers esp_http_server 
                    INCLUDE_DIRS "")
