/*
 * system_state.h
 *
 * System state management and fan control logic
 * This module reads from queues and implements the control logic
 */

#ifndef MAIN_SYSTEM_STATE_H_
#define MAIN_SYSTEM_STATE_H_

#include "freertos/FreeRTOS.h"
#include "http_server.h"

/* ============================================================
 *          SYSTEM STATE TASK (Fan Control Logic)
 * ============================================================*/

/**
 * @brief Main system control task
 * Runs the fan control logic every 500ms
 * - Reads temperature, PIR, and configuration
 * - Calculates PWM based on mode
 * - Sends PWM to fan, updates system state
 */
void system_control_task(void *pvParameters);

/**
 * @brief Copy the 3 programmed registers into caller buffer
 * Returns 0 on success
 */
int system_state_get_registers(scheduled_register_t out[3]);

#endif /* MAIN_SYSTEM_STATE_H_ */
