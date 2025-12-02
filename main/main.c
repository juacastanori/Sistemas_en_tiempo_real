/**
 * Application entry point.
 */

#include "nvs_flash.h"
#include "driver/gpio.h"

#include "wifi_app.h"
#include "http_server.h"
#include "thermistor_reader.h"
#include "pir_sensor.h"
#include "fan_control.h"


void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize sensors and actuators
    thermistor_init();  // Temperature sensor
    pir_sensor_init();  // PIR motion sensor
    fan_control_init(); // Fan PWM control
    
    // Start Wifi (se asume que llama a http_server_start internamente)
    wifi_app_start();
}
