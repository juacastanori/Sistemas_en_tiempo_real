/**
 * Application entry point.
 */

#include "nvs_flash.h"
#include "driver/gpio.h"

#include "wifi_app.h"
#include "rgb_led.h"
#include "http_server.h"
#include "thermistor_reader.h"

// Se elimina la referencia a BLINK_GPIO ya que el control es vía rgb_led
// Se asume que rgb_led_pwm_init() configura los GPIOs necesarios para PWM.
static void configure_led(void)
{
    // gpio_reset_pin(BLINK_GPIO); // Se elimina
    // gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT); // Se elimina
    rgb_led_pwm_init();
}


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
    
    // Configura el LED (solo llama a la inicialización PWM RGB)
    configure_led();
    
    // Start Wifi (se asume que llama a http_server_start internamente)
    wifi_app_start();

    // Start thermistor reader (inicializa la cola y la tarea)
    thermistor_init();
}