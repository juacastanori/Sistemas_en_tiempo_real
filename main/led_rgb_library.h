#ifndef LED_RGB_LIBRARY_H
#define LED_RGB_LIBRARY_H

#include <stdint.h>
#include "driver/ledc.h"

typedef struct {
    int gpio_num;
    ledc_channel_t channel;
    uint32_t duty;
} LED_t;

typedef struct {
    LED_t red;
    LED_t green;
    LED_t blue;
    ledc_timer_t timer_sel;
    int duty_resolution; // guardar resolución para calcular max_duty si hace falta
} LED_RGB_t;

LED_RGB_t configure_LED_RGB(int gpio_num_red, int gpio_num_green, int gpio_num_blue,
                            ledc_channel_t channel_red, ledc_channel_t channel_green, ledc_channel_t channel_blue,
                            ledc_timer_t timer_sel, int duty_resolution, int frequency);

void set_LED_RGB_color(LED_RGB_t *led_rgb, uint32_t red_duty, uint32_t green_duty, uint32_t blue_duty);

void set_led_red(LED_t *led, uint32_t duty);
void set_led_green(LED_t *led, uint32_t duty);
void set_led_blue(LED_t *led, uint32_t duty);

#endif
