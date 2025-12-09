#ifndef THERMISTOR_READER_H
#define THERMISTOR_READER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief Inicializa el hardware ADC, la cola de temperatura, y la tarea de lectura del termistor.
 * Debe llamarse desde `app_main`.
 * Retorna un puntero opaco a un contexto que debe pasarse como `pvParameters`
 * a `thermistor_read_task` cuando se crea la tarea.
 */
void *thermistor_init(void);

/**
 * @brief Función de tarea que lee el termistor y escribe la temperatura (float) en la cola central.
 * Esta función debe iniciarse con `xTaskCreate()` desde `app_main`.
 */
void thermistor_read_task(void *pvParameters);

/**
 * @brief Obtiene el handle de la cola de FreeRTOS que contiene la última lectura de temperatura (float).
 * Retorna la cola central gestionada por `queues.c`.
 */
QueueHandle_t get_temperature_queue_handle(void);

#endif // THERMISTOR_READER_H