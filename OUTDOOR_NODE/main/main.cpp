#include <Arduino.h>
#include "MQ135.h"
#include "driver.h"
#include <WiFi.h>
#include "esp_netif.h"

 // ESP-IDF (C code) includes for RainMaker, HTTP server, FreeRTOS, and BME680 sensor
extern "C"{
  #include <string.h>
  #include <freertos/FreeRTOS.h>
  #include <freertos/task.h>
  #include <esp_log.h>
  #include <nvs_flash.h>
  #include <esp_rmaker_core.h>
  #include <esp_rmaker_standard_params.h>
  #include <esp_rmaker_standard_devices.h>
  #include <app_network.h>
  #include <app_insights.h>

  #include "bme680.h"
  #include "esp_idf_lib_helpers.h"
  #include "i2cdev.h"

  #include "esp_http_server.h"


}
// -----------------------------------------------------------------
// GLOBALS AND DEVICE/RAINMAKER SETUP
// -----------------------------------------------------------------


// Tag for ESP logging
static const char *TAG = "main"; //tag for esp error logging
// RainMaker device and parameters for GAS sensor
esp_rmaker_device_t *gas_sensor_device = NULL;
esp_rmaker_param_t *gas_param = NULL;
// RainMaker device and parameters for BME680 pressure sensor
esp_rmaker_device_t *bme_sensor_device;
esp_rmaker_param_t *pressure_param;  // declare temperature parameter

// -----------------------------------------------------------------
// HTTP SERVER: Handles requests from INDOOR ESP32 node
// -----------------------------------------------------------------

/**
 * @brief Handler for GET /data
 * Returns CSV: "<pressure>,<gas_ppm>"
 * Used by indoor ESP32 to fetch current values for comparison.
 */
esp_err_t data_get_handler(httpd_req_t *req)
{
    float pressure = app_get_current_pressure(); // Read current pressure (hPa)
    float gas_ppm = app_get_current_gas_ppm();   // Read current gas sensor value (ppm)

    char response[64];
    snprintf(response, sizeof(response), "%.2f,%.2f", pressure, gas_ppm);  // CSV style

    httpd_resp_set_type(req, "text/plain");  // Set Content-Type header
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}
/**
 * @brief Starts HTTP server at port 8082, serving /data endpoint.
 * Allows INDOOR unit to fetch latest values easily.
 */
void start_http_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 8082;     // Set non-default port to avoid conflicts
    config.stack_size = 8192;  // More stack for heavier requests
    config.task_priority = 5;  // Adjust priority if needed
    config.max_open_sockets = 4; // Prevents memory exhaustion
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 3;
    config.send_wait_timeout = 3;
    ESP_LOGI(TAG, "HTTP server listening on port %d", config.server_port);
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t data_uri = {
            .uri      = "/data",
            .method   = HTTP_GET,  //GET not POST because send data from server 
            .handler  = data_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &data_uri);
    }
}

// -----------------------------------------------------------------
// ARDUINO SETUP (Runs once on boot)
// -----------------------------------------------------------------
void setup() {
  Serial.begin(115200); //baud rate changed for esp32
  /* Initialize Application specific hardware drivers and
     * set initial state.
     */
  //contains rtos loop

  app_driver_init();

  // Initialize Non-volatile storage for WiFi & RainMaker
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
  esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "Outdoor Sensors", "Other");
  if (!node) {
    ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
    vTaskDelay(5000/portTICK_PERIOD_MS);
    abort();
  }

  /* Create a device and add the relevant parameters to it */
  gas_sensor_device = esp_rmaker_device_create("MQ135", NULL,NULL);
  bme_sensor_device = esp_rmaker_device_create("BME680", NULL,NULL);
  // Create custom parameter for temperature
  gas_param = esp_rmaker_param_create("MQ 135 CO2/PPM", NULL, esp_rmaker_float(0), PROP_FLAG_READ); //displays value on rainmaker
  pressure_param = esp_rmaker_param_create("BME 2 Pressure/hPa", NULL, esp_rmaker_float(0), PROP_FLAG_READ);
  // Add the parameter to the device
  esp_rmaker_device_add_param(gas_sensor_device, gas_param);
  esp_rmaker_device_add_param(bme_sensor_device, pressure_param);

  esp_rmaker_node_add_device(node,gas_sensor_device);
  esp_rmaker_node_add_device(node,bme_sensor_device);
 
  /* Enable OTA */
  esp_rmaker_ota_enable_default();

  /* Enable Insights. Requires CONFIG_ESP_INSIGHTS_ENABLED=y */
  app_insights_enable();

  /* Start the ESP RainMaker Agent */
  esp_rmaker_start();

  /* Start the Wi-Fi.
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
  start_http_server();


}

void loop() {
  vTaskDelay(portMAX_DELAY); // Prevents this task from continuously consuming CPU time
}
