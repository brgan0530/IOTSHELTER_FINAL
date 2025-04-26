/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "bme680.h"  
#include "esp_idf_lib_helpers.h"
#include "i2cdev.h"
#include "i2c_lcd_pcf8574.h"

#define REPORTING_PERIOD    10 /* Seconds */

extern esp_rmaker_device_t *bme_sensor_device;
extern esp_rmaker_param_t *temp_param;
extern esp_rmaker_param_t *pressure_param; 
extern i2c_lcd_pcf8574_handle_t lcd;  


void app_driver_init(void);
float app_get_current_temperature();
float app_get_current_pressure();
