/*
 * queues.h
 *
 * Gestión central de colas y semáforos para la comunicación entre tareas
 * Toda comunicación global se realiza mediante colas/semaforos, no variables globales
 */

#ifndef MAIN_QUEUES_H_
#define MAIN_QUEUES_H_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "http_server.h"

/* ============================================================
 *                ESTRUCTURAS DE MENSAJES DE COLA
 * ============================================================*/

/**
 * Mensaje de estado del sistema (enviado al servidor HTTP y al control del ventilador)
 */
typedef struct {
    float temperature;
    int pir_state;
    int current_mode;
    int current_pwm;
    float auto_t_min;
    float auto_t_max;
    int active_register;
    scheduled_register_t registers[3];
} system_state_t;

/**
 * Mensaje de actualización de configuración (HTTP -> Sistema)
 */
typedef struct {
    int mode;               // -1 = sin cambio
    int manual_pwm;         // -1 = sin cambio
    float auto_tmin;        // NaN = sin cambio
    float auto_tmax;        // NaN = sin cambio
    scheduled_register_t registers[3];  // Registros para modo programado
    int update_registers;   // 1 = actualizar registros
} config_update_t;

/**
 * Señal de control del ventilador (Sistema -> Control del ventilador)
 */
typedef struct {
    int pwm_value;          // 0-100
} fan_signal_t;

/* ============================================================
 *                 MANEJO DE HANDLES DE COLA (Global)
 * ============================================================*/

// Funciones de inicialización - llamar una vez al inicio
int queues_init(void);
int queues_deinit(void);

// Obtener handles de las colas
QueueHandle_t queues_get_config_update_queue(void);
QueueHandle_t queues_get_system_state_queue(void);
QueueHandle_t queues_get_fan_signal_queue(void);
QueueHandle_t queues_get_temperature_queue(void);
QueueHandle_t queues_get_pir_queue(void);

// Cola del monitor del servidor HTTP (centralizada)
QueueHandle_t queues_get_http_monitor_queue(void);

// Cola de estado del servidor HTTP (contiene int fw_update_status)
QueueHandle_t queues_get_http_status_queue(void);

// Semáforo para acceso a configuración (protege registros, modo actual, etc.)
SemaphoreHandle_t queues_get_config_mutex(void);

#endif /* MAIN_QUEUES_H_ */
