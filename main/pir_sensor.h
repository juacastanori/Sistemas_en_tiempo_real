#ifndef PIR_SENSOR_H
#define PIR_SENSOR_H

#include "freertos/FreeRTOS.h"
#include <stdbool.h>

/**
 * @brief Inicializa el sensor PIR (GPIO).
 * - Configura el GPIO como entrada con pull-down
 * - Crea la tarea de lectura del PIR
 * - El resultado se almacena en una variable global que se expone a http_server.c
 */
void pir_sensor_init(void);

/**
 * @brief Task function that processes PIR events (debounce, normalization).
 * Start this task from `app_main` with xTaskCreate().
 */
void pir_event_task(void *pvParameters);

/**
 * @brief Obtiene el estado actual del PIR (1 = presencia detectada, 0 = no presencia).
 * @return int 1 si hay presencia, 0 si no.
 */
// PIR state is published to the central PIR queue; consumers should read from queues_get_pir_queue().
// Legacy getter removed to avoid global state.

/**
 * @brief Reinicia el contador de decaimiento del PIR (timeout).
 * Se usa si queremos implementar un timeout de presencia.
 */
void pir_sensor_reset_timeout(void);

/**
 * @brief Indica si el PIR está configurado como activo-bajo.
 * @return true si PIR es activo-bajo (LEVEL=0 -> presencia), false si activo-alto.
 */
bool pir_sensor_is_active_low(void);

#endif // PIR_SENSOR_H
