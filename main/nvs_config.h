/*
 * nvs_config.h
 *
 * Módulo de configuración en almacenamiento no volátil (Flash)
 * Gestiona el guardado y la recuperación persistente de la
 * configuración del sistema
 * (PWM Manual, Tmin/Tmax Automático, Registros Programados)
 */

#ifndef MAIN_NVS_CONFIG_H_
#define MAIN_NVS_CONFIG_H_

#include <stdint.h>
#include "http_server.h"  // Para scheduled_register_t

/**
 * @brief Inicializa NVS (almacenamiento Flash)
 * Llamar una vez al inicio antes de realizar operaciones de lectura/escritura
 * @return 0 en éxito, -1 en error
 */
int nvs_config_init(void);

/**
 * @brief Desinicializa NVS
 * @return 0 en éxito, -1 en error
 */
int nvs_config_deinit(void);

/* ============================================================
 *           MODO MANUAL: Guardar y cargar PWM
 * ============================================================*/

/**
 * @brief Guarda en flash el valor de PWM para el modo manual
 * @param pwm_value Porcentaje de PWM (0-100)
 * @return 0 en éxito, -1 en error
 */
int nvs_config_save_manual_pwm(int pwm_value);

/**
 * @brief Carga desde flash el valor de PWM para el modo manual
 * @param pwm_value Puntero donde almacenar el valor cargado
 * @return 0 en éxito, -1 en error o si no se encuentra
 */
int nvs_config_load_manual_pwm(int *pwm_value);

/* ============================================================
 *        MODO AUTOMÁTICO: Guardar y cargar Tmin/Tmax
 * ============================================================*/

/**
 * @brief Guarda en flash el rango de temperatura del modo automático (Tmin, Tmax)
 * @param tmin Temperatura mínima
 * @param tmax Temperatura máxima
 * @return 0 en éxito, -1 en error
 */
int nvs_config_save_auto_temps(float tmin, float tmax);

/**
 * @brief Carga desde flash las temperaturas del modo automático
 * @param tmin Puntero donde almacenar Tmin
 * @param tmax Puntero donde almacenar Tmax
 * @return 0 en éxito, -1 en error o si no se encuentra
 */
int nvs_config_load_auto_temps(float *tmin, float *tmax);

/* ============================================================
 *      MODO PROGRAMADO: Guardar y cargar registros
 * ============================================================*/

/**
 * @brief Guarda un registro programado (índice 0-2) en flash
 * @param index Índice del registro (0, 1 o 2)
 * @param reg Puntero a la estructura scheduled_register_t
 * @return 0 en éxito, -1 en error
 */
int nvs_config_save_register(int index, const scheduled_register_t *reg);

/**
 * @brief Carga un registro programado desde flash
 * @param index Índice del registro (0, 1 o 2)
 * @param reg Puntero donde almacenar el registro cargado
 * @return 0 en éxito, -1 en error o si no se encuentra
 */
int nvs_config_load_register(int index, scheduled_register_t *reg);

/**
 * @brief Guarda los 3 registros programados a la vez
 * @param registers Puntero al array de 3 estructuras scheduled_register_t
 * @return 0 en éxito, -1 en error
 */
int nvs_config_save_all_registers(const scheduled_register_t *registers);

/**
 * @brief Carga los 3 registros programados a la vez
 * @param registers Puntero al array donde se almacenarán los 3 registros
 * @return 0 en éxito, -1 en error
 */
int nvs_config_load_all_registers(scheduled_register_t *registers);

/* ============================================================
 *        CREDENCIALES WiFi STA: Guardar y cargar SSID/Password
 * ============================================================*/

/**
 * @brief Guarda las credenciales WiFi STA (SSID y contraseña) en flash
 * @param ssid SSID de la red WiFi a conectarse (máx 32 caracteres)
 * @param password Contraseña de la red WiFi (máx 64 caracteres)
 * @return 0 en éxito, -1 en error
 */
int nvs_config_save_wifi_credentials(const char *ssid, const char *password);

/**
 * @brief Carga las credenciales WiFi STA desde flash
 * @param ssid Buffer donde almacenar el SSID cargado (debe tener al menos 33 bytes)
 * @param password Buffer donde almacenar la contraseña (debe tener al menos 65 bytes)
 * @return 0 en éxito, -1 en error o si no se encuentran credenciales guardadas
 */
int nvs_config_load_wifi_credentials(char *ssid, char *password);

/* ============================================================
 *           AYUDA: Borrar todas las configuraciones
 * ============================================================*/

/**
 * @brief Borra todas las configuraciones almacenadas (restablece a valores por defecto)
 * @return 0 en éxito, -1 en error
 */
int nvs_config_erase_all(void);

#endif /* MAIN_NVS_CONFIG_H_ */
