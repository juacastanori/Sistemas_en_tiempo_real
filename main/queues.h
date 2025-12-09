/*
 * queues.h
 *
 * Central queue and semaphore management for inter-task communication
 * All global communication goes through queues/semaphores, not global variables
 */

#ifndef MAIN_QUEUES_H_
#define MAIN_QUEUES_H_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "http_server.h"

/* ============================================================
 *                QUEUE MESSAGE STRUCTURES
 * ============================================================*/

/**
 * System state message (sent to HTTP server and fan control)
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
 * Configuration update message (HTTP -> System)
 */
typedef struct {
    int mode;               // -1 = no change
    int manual_pwm;         // -1 = no change
    float auto_tmin;        // NaN = no change
    float auto_tmax;        // NaN = no change
    scheduled_register_t registers[3];  // Programmed mode registers
    int update_registers;   // 1 = update registers
} config_update_t;

/**
 * Fan control signal (System -> Fan Control)
 */
typedef struct {
    int pwm_value;          // 0-100
} fan_signal_t;

/* ============================================================
 *                 QUEUE HANDLES (Global)
 * ============================================================*/

// Initialization function - call once at startup
int queues_init(void);
int queues_deinit(void);

// Get queue handles
QueueHandle_t queues_get_config_update_queue(void);
QueueHandle_t queues_get_system_state_queue(void);
QueueHandle_t queues_get_fan_signal_queue(void);
QueueHandle_t queues_get_temperature_queue(void);
QueueHandle_t queues_get_pir_queue(void);

// HTTP server monitor queue (centralized)
QueueHandle_t queues_get_http_monitor_queue(void);

// HTTP server status queue (holds int fw_update_status)
QueueHandle_t queues_get_http_status_queue(void);

// Semaphore for configuration access (protects g_registers, g_current_mode, etc)
SemaphoreHandle_t queues_get_config_mutex(void);

#endif /* MAIN_QUEUES_H_ */
