#pragma once

#include <Arduino.h>
#include "MQ135.h"
#include <esp_rmaker_core.h>
#include <stdint.h>
#include <stdbool.h>
#include "bme680.h"
#include "esp_idf_lib_helpers.h"
#include "i2cdev.h"

#ifdef __cplusplus
extern "C" {
#endif

#define REPORTING_PERIOD    10 /* Seconds */ //update timing

// Declare this so main.cpp can access the param
//mq135
extern MQ135 mq135_sensor; //so i can use this in both main and driver
extern esp_rmaker_param_t *gas_param;
//bme680
extern esp_rmaker_device_t *bme_sensor_device;
extern esp_rmaker_param_t *pressure_param; 

void app_driver_init(void);
//void bme_update(void);
float app_get_current_pressure();
float app_get_current_gas_ppm();

#ifdef __cplusplus
}
#endif