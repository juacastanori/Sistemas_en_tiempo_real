/*
 * SPDX-FileCopyrightText: 2020-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"


// LED en GPIO4
#define GPIO_OUTPUT_IO_0    4
#define GPIO_OUTPUT_PIN_SEL (1ULL<<GPIO_OUTPUT_IO_0)

// Botón BOOT en GPIO9
#define GPIO_INPUT_IO_0     9
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)

// Cola para eventos del botón
static QueueHandle_t gpio_evt_queue = NULL;

// Variable global que controla si el LED parpadea o se apaga
static volatile int led_enabled = 1;


// ISR del botón -> envía el número de pin a la cola
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Task que maneja eventos del botón
static void gpio_task(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            int val = gpio_get_level(io_num);
            if (val == 0) {
                // Botón presionado -> apagar LED y detener parpadeo
                led_enabled = 0;
                gpio_set_level(GPIO_OUTPUT_IO_0, 0);
                printf("Botón presionado -> LED apagado\n");
            } else {
                // Botón soltado -> reanudar parpadeo
                led_enabled = 1;
                printf("Botón soltado -> LED vuelve a parpadear\n");
            }
        }
    }
}

// Task que maneja el parpadeo del LED
static void blink_task(void* arg)
{
    int cnt = 0;
    while (1) {
        if (led_enabled) {
            gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
            printf("cnt: %d\n", cnt++);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


// Configuración de GPIOs
static void setup_gpio(void)
{
    gpio_config_t io_conf = {};

    // LED como salida
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Botón como entrada (activo en bajo)
    io_conf.intr_type = GPIO_INTR_ANYEDGE;  // Interrupción en ambos flancos
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0; 
    gpio_config(&io_conf);
}


void app_main(void)
{
    // Configuración inicial de GPIOs
    setup_gpio();

    // Crear cola para eventos de botón
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // Instalar servicio de interrupciones e ISR para botón
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    // Crear tareas
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);
    xTaskCreate(blink_task, "blink_task", 2048, NULL, 10, NULL);
}
