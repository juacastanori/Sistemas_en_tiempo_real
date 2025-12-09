/*
 * nvs_config.h
 *
 * Non-Volatile Storage (Flash) configuration module
 * Handles persistent storage and retrieval of system configuration
 * (Manual PWM, Automatic Tmin/Tmax, Programmed Registers)
 */

#ifndef MAIN_NVS_CONFIG_H_
#define MAIN_NVS_CONFIG_H_

#include <stdint.h>
#include "http_server.h"  // For scheduled_register_t

/**
 * @brief Initialize NVS (flash storage)
 * Call this once at startup before any read/write operations
 * @return 0 on success, -1 on error
 */
int nvs_config_init(void);

/**
 * @brief Deinitialize NVS
 * @return 0 on success, -1 on error
 */
int nvs_config_deinit(void);

/* ============================================================
 *           MANUAL MODE: Store and Load PWM
 * ============================================================*/

/**
 * @brief Save manual mode PWM value to flash
 * @param pwm_value PWM percentage (0-100)
 * @return 0 on success, -1 on error
 */
int nvs_config_save_manual_pwm(int pwm_value);

/**
 * @brief Load manual mode PWM from flash
 * @param pwm_value Pointer to store the loaded value
 * @return 0 on success, -1 on error or not found
 */
int nvs_config_load_manual_pwm(int *pwm_value);

/* ============================================================
 *        AUTOMATIC MODE: Store and Load Tmin/Tmax
 * ============================================================*/

/**
 * @brief Save automatic mode temperature range (Tmin, Tmax) to flash
 * @param tmin Minimum temperature
 * @param tmax Maximum temperature
 * @return 0 on success, -1 on error
 */
int nvs_config_save_auto_temps(float tmin, float tmax);

/**
 * @brief Load automatic mode temperatures from flash
 * @param tmin Pointer to store Tmin
 * @param tmax Pointer to store Tmax
 * @return 0 on success, -1 on error or not found
 */
int nvs_config_load_auto_temps(float *tmin, float *tmax);

/* ============================================================
 *      PROGRAMMED MODE: Store and Load Registers
 * ============================================================*/

/**
 * @brief Save one programmed register (0-2) to flash
 * @param index Register index (0, 1, or 2)
 * @param reg Pointer to scheduled_register_t structure
 * @return 0 on success, -1 on error
 */
int nvs_config_save_register(int index, const scheduled_register_t *reg);

/**
 * @brief Load one programmed register from flash
 * @param index Register index (0, 1, or 2)
 * @param reg Pointer to store the loaded register
 * @return 0 on success, -1 on error or not found
 */
int nvs_config_load_register(int index, scheduled_register_t *reg);

/**
 * @brief Save all 3 programmed registers at once
 * @param registers Pointer to array of 3 scheduled_register_t structures
 * @return 0 on success, -1 on error
 */
int nvs_config_save_all_registers(const scheduled_register_t *registers);

/**
 * @brief Load all 3 programmed registers at once
 * @param registers Pointer to array that will store all 3 registers
 * @return 0 on success, -1 on error
 */
int nvs_config_load_all_registers(scheduled_register_t *registers);

/* ============================================================
 *           HELPER: Clear all configurations
 * ============================================================*/

/**
 * @brief Erase all stored configurations (reset to defaults)
 * @return 0 on success, -1 on error
 */
int nvs_config_erase_all(void);

#endif /* MAIN_NVS_CONFIG_H_ */
