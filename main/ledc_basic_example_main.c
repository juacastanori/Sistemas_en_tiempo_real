#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_rgb_library.h"

//Define GPIO pins for the RGB LED
#define GPIO_R GPIO_NUM_0
#define GPIO_G GPIO_NUM_1
#define GPIO_B GPIO_NUM_3

void app_main(void)
{
    // Configurate the RGB LED
    LED_RGB_t led = configure_LED_RGB(
        GPIO_R, GPIO_G, GPIO_B,
        LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2,
        LEDC_TIMER_0,
        8,      // duty resolution bits
        5000    // PWM frequency
    );

    uint32_t max_duty = (1u << 8) - 1u; // 255

    // Cycle through some colors
    ESP_LOGI("RGB", "Turning RED");
    set_LED_RGB_color(&led, max_duty, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI("RGB", "Turning GREEN");
    set_LED_RGB_color(&led, 0, max_duty, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI("RGB", "Turning BLUE");
    set_LED_RGB_color(&led, 0, 0, max_duty);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI("RGB", "Turning MAGENTA");
    set_LED_RGB_color(&led, max_duty/2, 0, max_duty/2);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI("RGB", "Turning OFF");
    set_LED_RGB_color(&led, 0, 0, 0);
}
