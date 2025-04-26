Prateeti and Bei Ru 

| Supported Targets | ESP32-C6-DevKitC-1| 

Credits: We used Unclerus, ESPRESSIF'S REGISTRY COMPONENTS and rainmaker example files to create our codes

# **IOT GAS SHELTER MONITORING SYSTEM**

# Outdoor Sensor Integrated Code 
This outdoor ESP node connects to a BME680 sensor to retrieve pressure values and MQ135. It is currently designed to run Arduino cpp framework within ESPIDF. Sensor readings are uploaded to rainmaker periodically. On top of that to talk to the indoor ESP node, sensor data is being transmitted over using http get method. Outdoor is the http server. Note to connect all ESPs and website to same wifi to view http values.

# Indoor Sensor Integrated Code 
This indoor ESP node connects to a BME680 sensor to retrieve pressure values and LCD to display gas level sensor data and status from the outdoor node. This indoor ESP node is a HTTP client and it receives sensor data from the outdoor ESP node. The LCD panel component library is modified from Espressif's component registry to be compatible with Unclerus BME sensor library. Espressif's PCF5874 lcd i2c library cannot be directly used as it uses ESP's own i2c library while uncle rus uses its own i2c library. 

## How to use example
1) Note to seperately run INDOOR and OUTDOOR node codes. Create seperate projects for each node
2) Build and flash code and wait for Rainmaker Wifi Provisioning QR Code
3) Read esp terminal log for esp_netif_handlers for sta ip (note that this changes according to the wifi your rainmaker is provisioned with). Use this ip address for indoor sensor code
4) Sensor data should be seen updating on rainmaker application.
5) You can open http uri link to view sensor data transmitted (both gas and pressure are transmitted) 

Extra: 
Below is the file directory of remaining files in the project folder. Note to set your RMAKER Path to the path where rainmaker was gitcloned

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── app_main.c
│   └── app_driver.c
└── README.md                
```
