/*  Modified from temp sensor example
*/
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <sdkconfig.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h> 
#include <esp_rmaker_standard_params.h> 

#include <app_reset.h>

#include "app_priv.h"
#include "esp_mac.h"

#include "bme680.h"
#include "esp_idf_lib_helpers.h"
#include "i2cdev.h"
#include <string.h>  // Add this for memset()
#include "i2c_lcd_pcf8574.h"
#include "esp_log.h"    

//find param in app_main file 
extern esp_rmaker_param_t *temp_param;
extern esp_rmaker_param_t *pressure_param;  // declare temperature parameter

//Wifi reset
#define BUTTON_GPIO          9 //for strapping pin
#define BUTTON_ACTIVE_LEVEL  0
#define WIFI_RESET_BUTTON_TIMEOUT       3
#define FACTORY_RESET_BUTTON_TIMEOUT    10

//Rtos
static TimerHandle_t sensor_timer;

//Sensor
static float bme_temp;
static float bme_pressure;
#define BME680_I2C_ADDR 0x77 //found online 
#define I2C_PORT 0
#define I2C_MASTER_SDA_PIN 5 //esp32 c6 properties 
#define I2C_MASTER_SCL_PIN 6
#define LCD_ADDR 0x27 //for 1602
#define LCD_COLS 16                 // Number of columns in the LCD
#define LCD_ROWS 2  

#define APP_CPU_NUM 0 //define core number for task, c6 only is single core 

static bme680_t sensor; //intialise sensor
i2c_lcd_pcf8574_handle_t lcd; //global

//Error logging
static const char *TAG = "driver_debug";

//BME680 functions
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

    // Change the heater profile 0 to 200 degree Celsius for 100 ms.
    //bme680_set_heater_profile(&sensor, 0, 200, 100);
    //bme680_use_heater_profile(&sensor, 0);

    // Set ambient temperature to 27 degree Celsius, maybe need to use this to calibrate sensor
    bme680_set_ambient_temperature(&sensor, 27);
}

//for measuring sensor 
void bme680_measure()
{
    // as long as sensor configuration isn't changed, duration is constant
    uint32_t duration;
    bme680_get_measurement_duration(&sensor, &duration);

    //TickType_t last_wakeup = xTaskGetTickCount();

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
            bme_temp = values.temperature; //change to 2dp 
            bme_pressure = values.pressure;
        }
    }
}

//Update parameters in rainmaker
static void app_sensor_update(TimerHandle_t handle)
{
    //remeasure sensor data 
    bme680_measure();
    esp_rmaker_param_update_and_report(
            temp_param,
            esp_rmaker_float(bme_temp));
    esp_rmaker_param_update_and_report(
            pressure_param,
            esp_rmaker_float(bme_pressure));
}

//Retrieves sensor data
float app_get_current_temperature()

{
    return bme_temp;
}

float app_get_current_pressure()

{
    return bme_pressure;
}

//LCD Functions 
static void lcd_setup()
{
    lcd_init(&lcd, LCD_ADDR, I2C_PORT, I2C_MASTER_SDA_PIN, I2C_MASTER_SCL_PIN);
    ESP_LOGI(TAG, "Initializing LCD");
    lcd_begin(&lcd, LCD_COLS, LCD_ROWS);
    ESP_LOGI(TAG, "LCD initialized");

    // Turn on the backlight
    lcd_set_backlight(&lcd, 255);
    // Print a message
    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "INDOOR SENSORS!!");
}

//Rtos timer loop
esp_err_t app_sensor_init(void)
{
    //init bme parameters 
    ESP_ERROR_CHECK(i2cdev_init()); //intialise i2c library for bme 
    bme_init();//intialise bme 
    lcd_setup();
    sensor_timer = xTimerCreate("app_sensor_update_tm", (REPORTING_PERIOD * 1000) / portTICK_PERIOD_MS,
                            pdTRUE, NULL, app_sensor_update);
    if (sensor_timer) {
        xTimerStart(sensor_timer, 0);
        return ESP_OK;
    }
    return ESP_FAIL;
}

void app_driver_init()
{
    app_sensor_init();
    app_reset_button_register(app_reset_button_create(BUTTON_GPIO, BUTTON_ACTIVE_LEVEL),
                WIFI_RESET_BUTTON_TIMEOUT, FACTORY_RESET_BUTTON_TIMEOUT);
}
