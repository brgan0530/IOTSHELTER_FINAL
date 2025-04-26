//modified from temperature sensor example from esp-idf examples
/* ============================================================================
 * ESP32 Indoor Sensor Node - Industrial Chemical Shelter Monitor
 * 
 * Main functions:
 *  - Reads local BME680 sensor (temperature, pressure)
 *  - Fetches outside data (pressure, gas ppm) from another ESP32 via HTTP
 *  - Calculates delta pressure (ΔP) for shelter status
 *  - Displays gas safety status on LCD
 *  - Reports all values to ESP RainMaker cloud with time series enabled
 *
 * NOTE: This is the INDOOR unit code.
 * ============================================================================
 */
 
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include "esp_http_client.h"
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <stdlib.h>
#include <app_network.h>
#include <app_insights.h>

#include "app_priv.h"

#include "bme680.h"
#include "esp_idf_lib_helpers.h"
#include "i2cdev.h"
// ----------------------------
// Global Declarations
// ----------------------------

static const char *TAG = "app_main";// ESP-IDF logging tag

// RainMaker device and parameters
esp_rmaker_device_t *bme_sensor_device;
esp_rmaker_param_t *temp_param;  // declare temperature parameter
esp_rmaker_param_t *pressure_param;  // declare temperature parameter
esp_rmaker_param_t *delta_p_param;// Delta pressure parameter 

// Data values (shared globals)
float pressure_inside = 0.0;
float pressure_outside = 0.0;
float delta_p = 0.0;

// Structure for outside sensor data received via HTTP
typedef struct {
    float pressure;
    float gas_ppm;
} SensorData;
 
// Gas safety threshold
#define GAS_PPM_THRESHOLD 2000.0f //define gas level threshold here. if greater than this value, means gas level is unsafe 


// ----------------------------
// HTTP Fetch - Outside Sensor Data
// ----------------------------

/**
 * Fetch pressure and gas_ppm from the OUTSIDE sensor node via HTTP GET.
 * Expects response: "pressure_value,gas_ppm_value"
 * Returns: SensorData struct; values -1 if request fails.
 */
SensorData fetch_outside_data() {
    esp_http_client_config_t config = {
        .url = "http://X.X.X.X:8082/data", // Replace with X.X.X.X with actual server ESP32 IP
        .timeout_ms = 5000,
    };

    SensorData result = {-1, -1};//returns this when there is something wrong with HTTP
    esp_http_client_handle_t client = esp_http_client_init(&config);
    
    if (!client) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        return result;
    }

    esp_http_client_set_method(client, HTTP_METHOD_GET);
    ESP_LOGI(TAG, "Opening HTTP connection to %s", config.url);

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return result;
    }

    int content_length = esp_http_client_fetch_headers(client);
    ESP_LOGI(TAG, "Content length: %d", content_length);

    char buffer[128] = {0};
    int len = esp_http_client_read(client, buffer, sizeof(buffer) - 1);
    ESP_LOGI(TAG, "Read length = %d", len);
    ESP_LOGI(TAG, "Buffer content: '%s'", buffer);

    if (len > 0) {
        buffer[len] = '\0';
        // Split at comma: "pressure,gas_ppm"
        char *comma = strchr(buffer, ',');
        if (comma) {
            *comma = '\0';
            result.pressure = atof(buffer);
            result.gas_ppm = atof(comma + 1);
            ESP_LOGI(TAG, "Parsed Pressure = %.2f, Gas PPM = %.2f", result.pressure, result.gas_ppm);
        } else {
            ESP_LOGE(TAG, "Malformed response: comma not found");
        }
    } else {
        ESP_LOGE(TAG, "Failed to read response body");
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return result;
}


// ----------------------------
// Task: Data Monitoring and Reporting
// ----------------------------

/**
 * data_monitor_task
 * - Reads indoor sensor data
 * - Fetches outdoor data via HTTP
 * - Calculates delta pressure
 * - Reports to RainMaker and LCD
 */
void data_monitor_task(void *pvParameters) { 

    char buf[17];       // LCD line buffer
    float gas_ppm = 0;  // must match what fetch_outside_data writes into
    float pressure;     // temporary for outside pressure

    // Initial LCD setup
    lcd_clear(&lcd);
    lcd_set_cursor(&lcd, 0, 0);
    lcd_print(&lcd, "GAS PPM:");

    while (1) { 
        // Read latest indoor sensor
        pressure_inside = app_get_current_pressure(); 
        // Fetch latest outdoor data
        SensorData outside = fetch_outside_data(); 
        
        //Calculates Delta Pressure and uploads to rainmaker
        if (outside.pressure > 0 && pressure_inside > 0) { 
            delta_p = outside.pressure - pressure_inside; 
            ESP_LOGI(TAG, "\u0394P = %.2f hPa", delta_p); 
            esp_rmaker_param_update_and_report(delta_p_param, esp_rmaker_float(delta_p)); 
            ESP_LOGI(TAG, "Reporting delta P: %.2f", delta_p);
        } 

        // Process gas sensor safety and LCD display
        bool is_safe = outside.gas_ppm <= GAS_PPM_THRESHOLD;

        lcd_clear(&lcd);                  
        lcd_set_cursor(&lcd, 0, 0);
        if (is_safe) {
            lcd_print(&lcd, "GAS PPM: SAFE");
        } else {
            lcd_print(&lcd, "GAS PPM: UNSAFE");
        }
 
         // Show gas value on second LCD line
         snprintf(buf, sizeof(buf), "%.3f ppm", outside.gas_ppm);
         lcd_set_cursor(&lcd, 0, 1);
         lcd_print(&lcd, buf);

        // Delay before next reading (REPORTING_PERIOD in seconds)
        vTaskDelay(pdMS_TO_TICKS(REPORTING_PERIOD*1000)); 
    } 
}

// ----------------------------
// Main Application Entry Point
// ----------------------------
void app_main()
{
    /* Initialize Application specific hardware drivers and
     * set initial state.
     */
    app_driver_init();

    /* Initialize NVS flash (required by RainMaker and WiFi) */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    /* Initialize Wi-Fi. Note that, this should be called before esp_rmaker_node_init()
     */
    app_network_init();

    /* Initialize the ESP RainMaker Agent.
     * Note that this should be called after app_network_init() but before app_network_start()
     * */
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = true, //set to true because I want to plot time series data
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "Indoor pressure sensor", "Other");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }
    
    /* Create a device and add the relevant parameters to it */
    bme_sensor_device = esp_rmaker_device_create("BME Sensor", NULL,NULL);
    
    // Create custom parameter for temperature
    temp_param = esp_rmaker_param_create("BME 1 Temp", NULL, esp_rmaker_float(app_get_current_temperature()), PROP_FLAG_READ | PROP_FLAG_TIME_SERIES); //time series data flag to enable plot graph
    pressure_param = esp_rmaker_param_create("BME 1 Pressure", NULL, esp_rmaker_float(app_get_current_pressure()), PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    delta_p_param=esp_rmaker_param_create("BME 1 and 2 Delta Pressure", NULL, esp_rmaker_float(0), PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);

    // Add the parameter to the device
    esp_rmaker_device_add_param(bme_sensor_device, temp_param);
    esp_rmaker_device_add_param(bme_sensor_device, pressure_param);
    esp_rmaker_device_add_param(bme_sensor_device, delta_p_param);
    esp_rmaker_node_add_device(node, bme_sensor_device);

    /* Enable OTA */
    esp_rmaker_ota_enable_default();

    /* Enable Insights. Requires CONFIG_ESP_INSIGHTS_ENABLED=y */
    app_insights_enable();

    /* Start the ESP RainMaker Agent */
    esp_rmaker_start();

    /* Start the Wi-Fi
     * If the node is provisioned, it will start connection attempts,
     * else, it will start Wi-Fi provisioning. The function will return
     * after a connection has been successfully established
     */
    err = app_network_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wifi. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }
    xTaskCreate(&data_monitor_task, "data_monitor_task", 4096, NULL, 5, NULL);
}
