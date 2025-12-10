/*
 * system_state.h
 *
 * Gestión del estado del sistema y lógica de control del ventilador
 * Este módulo lee desde colas e implementa la lógica de control
 */

#ifndef MAIN_SYSTEM_STATE_H_
#define MAIN_SYSTEM_STATE_H_

#include "freertos/FreeRTOS.h"
#include "http_server.h"

/* ============================================================
 *          TAREA DE ESTADO DEL SISTEMA (Lógica de ventilador)
 * ============================================================*/

/**
 * @brief Tarea principal de control del sistema
 * Ejecuta la lógica del ventilador cada 500 ms
 * - Lee temperatura, PIR y configuración
 * - Calcula el PWM según el modo
 * - Envía el PWM al ventilador y actualiza el estado del sistema
 *
 * @param pvParameters Parámetros de la tarea (no usados)
 * @return void
 */
void system_control_task(void *pvParameters);

/**
 * @brief Copia los 3 registros programados en el buffer del llamador
 *
 * @param out Array de 3 `scheduled_register_t` donde se colocarán los registros
 * @return int 0 en éxito, -1 en caso de error
 */
int system_state_get_registers(scheduled_register_t out[3]);

#endif /* MAIN_SYSTEM_STATE_H_ */
