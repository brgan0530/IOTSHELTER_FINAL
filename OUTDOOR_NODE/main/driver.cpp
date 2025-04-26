#include <Arduino.h>
#include "MQ135.h"

extern "C"{
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <sdkconfig.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h> 
#include <esp_rmaker_standard_params.h> 
#include <app_reset.h>
#include "esp_mac.h"
#include <string.h>  // Add this for memset(
#include "bme680.h"
#include "esp_idf_lib_helpers.h"
#include "i2cdev.h"
}

#include "driver.h"

//define for mq135, to be updated with bme680
static float temperature = 29.0; // Assume current temperature. Using SG data.
static float humidity = 65.0; // Assume current humidity. Using SG data.

//find param in app_main file 
extern esp_rmaker_param_t *gas_param;
extern esp_rmaker_param_t *pressure_param; 

#define PIN_MQ135 4 //for MQ135 sensor 
MQ135 mq135_sensor(PIN_MQ135);

//Wifi reset 
#define BUTTON_GPIO          ((gpio_num_t)9) //for strapping pin
#define BUTTON_ACTIVE_LEVEL  ((button_active_t)0)
#define WIFI_RESET_BUTTON_TIMEOUT       3
#define FACTORY_RESET_BUTTON_TIMEOUT    10

//Sensors
static float bme_pressure;
static float correctedPPM; 
#define BME680_I2C_ADDR 0x77 //found online 
#define I2C_PORT ((i2c_port_t)0)
#define I2C_MASTER_SDA_PIN ((gpio_num_t)5) //esp32 c6 properties 
#define I2C_MASTER_SCL_PIN ((gpio_num_t)6)
#define APP_CPU_NUM 0 //define core number for task, c6 only is single core 

static bme680_t sensor; //intialise sensor

//BME680 Functions 
static void bme_init()
{
    memset(&sensor, 0, sizeof(bme680_t));//all bytes in sensor set to 0, &sensor means that a copy is not created 

    ESP_ERROR_CHECK(bme680_init_desc(&sensor, BME680_I2C_ADDR, I2C_PORT, I2C_MASTER_SDA_PIN, I2C_MASTER_SCL_PIN));

    // init the sensor
    ESP_ERROR_CHECK(bme680_init_sensor(&sensor));

    // Changes the oversampling rates to 4x oversampling for temperature
    // and 2x oversampling for humidity. Pressure measurement is skipped.
    bme680_set_oversampling_rates(&sensor, BME680_OSR_4X, BME680_OSR_4X, BME680_OSR_4X);

    // Change the IIR filter size for temperature and pressure to 7.
    bme680_set_filter_size(&sensor, BME680_IIR_SIZE_7);

    // Set ambient temperature to 27 degree Celsius, maybe need to use this to calibrate sensor
    bme680_set_ambient_temperature(&sensor, temperature);
}


//for measuring sensor 
void bme680_measure()
{
    // as long as sensor configuration isn't changed, duration is constant
    uint32_t duration;
    bme680_get_measurement_duration(&sensor, &duration);
    bme680_values_float_t values;
    
    // trigger the sensor to start one TPHG measurement cycle
    if (bme680_force_measurement(&sensor) == ESP_OK)
    {
        // passive waiting until measurement results are available
        vTaskDelay(duration);

        // get the results and do something with them
        if (bme680_get_results_float(&sensor, &values) == ESP_OK){
            printf("BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa \n",
                    values.temperature, values.humidity, values.pressure);
            bme_pressure = values.pressure;
            //update mq135 dynamically as sensor requires temp and humid data 
            temperature = values.temperature;
            humidity = values.humidity;
        }
    }
    
}

//main functions
void sensor_update(void *param) {
    while (true) {
        bme680_measure();
        esp_rmaker_param_update_and_report(pressure_param, esp_rmaker_float(bme_pressure));

        float ppm = mq135_sensor.getPPM();
        correctedPPM = mq135_sensor.getCorrectedPPM(temperature, humidity);
        Serial.printf("PPM: %.2f\t Corrected PPM: %.2f ppm\n", ppm, correctedPPM);
        esp_rmaker_param_update_and_report(gas_param, esp_rmaker_float(correctedPPM));

        vTaskDelay(REPORTING_PERIOD * 1000 / portTICK_PERIOD_MS); // Delay for the same reporting period
    }
}

//retrieves parameters 
float app_get_current_pressure()
{
    return bme_pressure;
}

float app_get_current_gas_ppm()
{
    return correctedPPM;
}

//Rtos to help with looping
esp_err_t app_sensor_init(void)
{
    //init bme parameters 
    ESP_ERROR_CHECK(i2cdev_init()); //intialise i2c library for bme 
    bme_init();//intialise bme 
    // Create a task instead of a timer to avoid stack overflow
    xTaskCreatePinnedToCore(sensor_update, "Sensor Update", 4096, NULL, 5, NULL, APP_CPU_NUM);
    return ESP_OK;
}

void app_driver_init()
{
    app_sensor_init();
    app_reset_button_register(app_reset_button_create(BUTTON_GPIO, BUTTON_ACTIVE_LEVEL),
                WIFI_RESET_BUTTON_TIMEOUT, FACTORY_RESET_BUTTON_TIMEOUT);

}



