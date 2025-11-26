#ifndef THERMISTOR_READER_H
#define THERMISTOR_READER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief Inicializa el hardware ADC, la cola de temperatura, y la tarea de lectura del termistor.
 * Debe llamarse desde app_main.
 */
void thermistor_init(void);

/**
 * @brief Obtiene el handle de la cola de FreeRTOS que contiene la última lectura de temperatura (float).
 * El tamaño de la cola es 1 y el valor se sobrescribe.
 * @return QueueHandle_t Handle de la cola.
 */
QueueHandle_t get_temperature_queue_handle(void);

#endif // THERMISTOR_READER_H