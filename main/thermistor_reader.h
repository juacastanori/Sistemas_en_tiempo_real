#ifndef THERMISTOR_READER_H
#define THERMISTOR_READER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief Inicializa el hardware ADC, la cola de temperatura, y la tarea de lectura del termistor.
 * Debe llamarse desde app_main.
 */
// Initialize thermistor hardware. Returns an opaque context pointer that
// must be passed as `pvParameters` to `thermistor_read_task` when creating the task.
void *thermistor_init(void);

/**
 * @brief Task function that reads the thermistor and writes temperature float to the temperature queue.
 * This function should be started with xTaskCreate() from `app_main`.
 */
void thermistor_read_task(void *pvParameters);

/**
 * @brief Obtiene el handle de la cola de FreeRTOS que contiene la última lectura de temperatura (float).
 * Retorna la cola central gestionada por `queues.c`.
 */
QueueHandle_t get_temperature_queue_handle(void);

#endif // THERMISTOR_READER_H