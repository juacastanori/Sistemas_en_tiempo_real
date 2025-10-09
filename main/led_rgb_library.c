#include <stdint.h>
#include "driver/ledc.h"
#include "led_rgb_library.h"
#include "esp_err.h"

LED_RGB_t configure_LED_RGB(int gpio_num_red, int gpio_num_green, int gpio_num_blue,
                            ledc_channel_t channel_red, ledc_channel_t channel_green, ledc_channel_t channel_blue,
                            ledc_timer_t timer_sel, int duty_resolution, int frequency)
{
    LED_RGB_t led_rgb;

    led_rgb.red.gpio_num = gpio_num_red;
    led_rgb.red.channel = channel_red;
    led_rgb.red.duty = 0;

    led_rgb.green.gpio_num = gpio_num_green;
    led_rgb.green.channel = channel_green;
    led_rgb.green.duty = 0;

    led_rgb.blue.gpio_num = gpio_num_blue;
    led_rgb.blue.channel = channel_blue;
    led_rgb.blue.duty = 0;

    led_rgb.timer_sel = timer_sel;
    led_rgb.duty_resolution = duty_resolution;

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = duty_resolution,
        .timer_num = led_rgb.timer_sel,
        .freq_hz = frequency,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configurar cada canal
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = led_rgb.red.channel,
        .timer_sel = led_rgb.timer_sel,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = led_rgb.red.gpio_num,
        .duty = 0,
        .hpoint = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.channel = led_rgb.green.channel;
    ledc_channel.gpio_num = led_rgb.green.gpio_num;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.channel = led_rgb.blue.channel;
    ledc_channel.gpio_num = led_rgb.blue.gpio_num;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    return led_rgb;
}

static uint32_t max_duty_from_res(int res_bits) {
    return (1u << res_bits) - 1u;
}

void set_LED_RGB_color(LED_RGB_t *led_rgb, uint32_t red_duty, uint32_t green_duty, uint32_t blue_duty)
{
    set_led_blue(&led_rgb->blue, blue_duty);
    set_led_green(&led_rgb->green, green_duty);
    set_led_red(&led_rgb->red, red_duty);
}

void set_led_red(LED_t *led, uint32_t duty)
{
    led->duty = duty;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, led->channel, led->duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, led->channel));
}

void set_led_green(LED_t *led, uint32_t duty)
{
    led->duty = duty;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, led->channel, led->duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, led->channel));
}

void set_led_blue(LED_t *led, uint32_t duty)
{
    led->duty = duty;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, led->channel, led->duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, led->channel));
}
