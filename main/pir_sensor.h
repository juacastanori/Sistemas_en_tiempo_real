#ifndef PIR_SENSOR_H
#define PIR_SENSOR_H

#include "freertos/FreeRTOS.h"

/**
 * @brief Inicializa el sensor PIR (GPIO).
 * - Configura el GPIO como entrada con pull-down
 * - Crea la tarea de lectura del PIR
 * - El resultado se almacena en una variable global que se expone a http_server.c
 */
void pir_sensor_init(void);

/**
 * @brief Obtiene el estado actual del PIR (1 = presencia detectada, 0 = no presencia).
 * @return int 1 si hay presencia, 0 si no.
 */
int pir_sensor_get_state(void);

/**
 * @brief Reinicia el contador de decaimiento del PIR (timeout).
 * Se usa si queremos implementar un timeout de presencia.
 */
void pir_sensor_reset_timeout(void);

#endif // PIR_SENSOR_H
