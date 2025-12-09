#ifndef PIR_SENSOR_H
#define PIR_SENSOR_H

#include "freertos/FreeRTOS.h"
#include <stdbool.h>

/**
 * @brief Inicializa el sensor PIR (GPIO).
 * - Configura el GPIO como entrada y ajusta pull según la polaridad
 * - Crea la tarea que procesa eventos del PIR
 * - Publica el estado normalizado en la cola central del PIR
 */
void pir_sensor_init(void);

/**
 * @brief Función de tarea que procesa eventos del PIR (debounce y normalización).
 * Iniciar esta tarea desde `app_main` con `xTaskCreate()`.
 */
void pir_event_task(void *pvParameters);

/**
 * @brief Reinicia el contador de decaimiento del PIR (timeout).
 * Utilizar si se implementa un timeout de presencia.
 */
void pir_sensor_reset_timeout(void);

/**
 * @brief Indica si el PIR está configurado como activo-bajo.
 * @return true si el PIR es activo-bajo (LEVEL=0 -> presencia), false si es activo-alto.
 */
bool pir_sensor_is_active_low(void);

#endif // PIR_SENSOR_H
