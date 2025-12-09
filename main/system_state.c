/**
 * @file system_state.c
 * @brief Gestión del estado del sistema y lógica de control del ventilador.
 *
 * Implementa la tarea de control única que posee el estado de ejecución,
 * aplica actualizaciones de configuración, calcula el PWM deseado según el
 * modo (manual/automático/programado) y persiste parámetros relevantes en NVS.
 */

#include "system_state.h"
#include "queues.h"
#include "fan_control.h"
#include "pir_sensor.h"
#include "nvs_config.h"
#include "esp_log.h"
#include <time.h>
#include <math.h>

static const char TAG[] = "SYSTEM_STATE";

/**
 * @brief Carga la configuración desde flash al inicio
 */
static void load_initial_config(int *current_pwm_out, float *auto_t_min_out, float *auto_t_max_out, scheduled_register_t registers_out[3])
{
    ESP_LOGI(TAG, "Loading initial configuration from flash...");
    /* Cargar PWM manual */
    int saved_pwm = 0;
    if (nvs_config_load_manual_pwm(&saved_pwm) == 0) {
        *current_pwm_out = saved_pwm;
        ESP_LOGI(TAG, "Loaded manual PWM: %d%%", *current_pwm_out);
    } else {
        *current_pwm_out = 0;
        ESP_LOGW(TAG, "No saved manual PWM, using default: 0%%");
    }

    /* Cargar temperaturas automáticas */
    float saved_tmin = 20.0f, saved_tmax = 30.0f;
    if (nvs_config_load_auto_temps(&saved_tmin, &saved_tmax) == 0) {
        *auto_t_min_out = saved_tmin;
        *auto_t_max_out = saved_tmax;
        ESP_LOGI(TAG, "Loaded auto temps: %.1f-%.1f°C", *auto_t_min_out, *auto_t_max_out);
    } else {
        *auto_t_min_out = 20.0f;
        *auto_t_max_out = 30.0f;
        ESP_LOGW(TAG, "No saved auto temps, using defaults: 20-30°C");
    }

    /* Cargar registros programados */
    nvs_config_load_all_registers(registers_out);
    ESP_LOGI(TAG, "Programmed registers loaded");
}

/**
 * @brief Gestiona actualizaciones de configuración recibidas desde el servidor HTTP.
 *
 * Esta función aplica los valores pendientes de `config_update_t` en la
 * configuración de ejecución local de la tarea y persiste los campos relevantes en NVS.
 * Sigue la convención de que un campo contiene un indicador de "sin cambio"
 * (p. ej. `-1` para enteros, `NaN` para floats) y solo aplica valores que
 * se proporcionan explícitamente.
 *
 * @param update Puntero al mensaje de actualización de configuración (solo lectura).
 * @param current_mode Puntero a la variable local de tarea que contiene el modo actual.
 * @param current_pwm Puntero al valor local de PWM manual a actualizar.
 * @param auto_t_min Puntero al valor local mínimo de temperatura automática.
 * @param auto_t_max Puntero al valor local máximo de temperatura automática.
 * @param registers Array de 3 registros programados (locales) a actualizar cuando se solicite.
 */
static void handle_config_update(const config_update_t *update, int *current_mode, int *current_pwm, float *auto_t_min, float *auto_t_max, scheduled_register_t registers[3])
{
    if (update->mode != -1) {
        *current_mode = update->mode;
        ESP_LOGI(TAG, "Mode updated: %d", *current_mode);
    }

    if (update->manual_pwm != -1) {
        *current_pwm = update->manual_pwm;
        nvs_config_save_manual_pwm(*current_pwm);
        ESP_LOGI(TAG, "Manual PWM updated: %d%%", *current_pwm);
    }

    if (!isnan(update->auto_tmin) && !isnan(update->auto_tmax)) {
        *auto_t_min = update->auto_tmin;
        *auto_t_max = update->auto_tmax;
        nvs_config_save_auto_temps(*auto_t_min, *auto_t_max);
        ESP_LOGI(TAG, "Auto temps updated: %.1f-%.1f°C", *auto_t_min, *auto_t_max);
    }

    if (update->update_registers) {
        for (int i = 0; i < 3; i++) {
            registers[i] = update->registers[i];
        }
        nvs_config_save_all_registers(registers);
        ESP_LOGI(TAG, "Registers updated and saved to flash");
    }
}

int system_state_get_registers(scheduled_register_t out[3])
{
    if (out == NULL) return -1;
    /* Leer los registros desde el estado publicado más reciente (cola de estado) */
    QueueHandle_t state_q = queues_get_system_state_queue();
    if (state_q == NULL) return -1;
    system_state_t st;
    if (xQueuePeek(state_q, &st, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < 3; i++) out[i] = st.registers[i];
        return 0;
    }
    return -1;
}

/**
 * @brief Tarea de control del sistema (bucle de control del ventilador, escritor único).
 *
 * La `system_control_task` es la propietaria autorizada de la configuración de
 * ejecución y la toma de decisiones. Realiza las siguientes acciones en un
 * bucle continuo (cada 500 ms):
 *
 * - Consume mensajes `config_update_t` desde `config_update_queue` y los aplica
 *   (vía `handle_config_update`).
 * - Lee la última temperatura desde `temperature_queue` y el estado del PIR
 *   desde `pir_queue`.
 * - Calcula el PWM deseado según el modo activo:
 *   - Manual: usar el valor de PWM manual.
 *   - Automático: si se detecta PIR, mapear la temperatura linealmente entre
 *     `auto_t_min` y `auto_t_max` a 0-100%; en caso contrario 0%.
 *   - Programado: comprobar los 3 registros programados y, si existe un registro
 *     activo que coincide con la hora actual y el PIR está presente, calcular el PWM
 *     proporcional entre `temp_min` y `temp_max` del registro.
 * - Publicar el `system_state_t` resultante en `system_state_queue` y
 *   reenviar comandos PWM al hardware del ventilador mediante `fan_control_set_pwm` y
 *   `fan_signal_queue`.
 *
 * La tarea mantiene todo el estado mutable local a sí misma (patrón escritor único)
 * y persiste los cambios en NVS cuando se aplican actualizaciones de configuración.
 *
 * @param pvParameters No usado.
 */
void system_control_task(void *pvParameters)
{
    float temperature = -99.9f;
    int pir_state = 0;
    int calculated_pwm = 0;
    time_t now;
    struct tm timeinfo;
    int current_hour, current_min;
    int register_active = -1;

    /* Estado local (local de la tarea; no global) */
    int current_mode = 0;
    int current_pwm = 0;
    float auto_t_min = 20.0f;
    float auto_t_max = 30.0f;
    scheduled_register_t registers[3] = {0};

    /* Cargar configuración desde flash al iniciar */
    load_initial_config(&current_pwm, &auto_t_min, &auto_t_max, registers);

    QueueHandle_t config_queue = queues_get_config_update_queue();
    QueueHandle_t temp_queue = queues_get_temperature_queue();
    QueueHandle_t state_queue = queues_get_system_state_queue();
    QueueHandle_t pir_queue = queues_get_pir_queue();
    QueueHandle_t fan_queue = queues_get_fan_signal_queue();

    config_update_t config_update;

    while (1) {
        /* Revisar actualizaciones de configuración (sin bloqueo) */
        if (xQueueReceive(config_queue, &config_update, 0) == pdTRUE) {
            handle_config_update(&config_update, &current_mode, &current_pwm, &auto_t_min, &auto_t_max, registers);
        }

        /* Obtener temperatura actual */
        if (xQueuePeek(temp_queue, &temperature, 0) != pdTRUE) {
            temperature = -99.9f;
        }

        /* Obtener estado del PIR desde la cola central de PIR */
        if (pir_queue != NULL) {
            if (xQueuePeek(pir_queue, &pir_state, 0) != pdTRUE) {
                pir_state = 0;
            }
        } else {
            pir_state = 0;
        }

        /* Obtener hora actual */
        time(&now);
        localtime_r(&now, &timeinfo);
        current_hour = timeinfo.tm_hour;
        current_min = timeinfo.tm_min;

        /* Resetear PWM y registro activo */
        calculated_pwm = 0;
        register_active = -1;

        // ==== FAN CONTROL LOGIC ====
        switch (current_mode) {
            case 0:  // MANUAL MODE
                calculated_pwm = current_pwm;
                ESP_LOGD(TAG, "MODE MANUAL: PWM=%d", calculated_pwm);
                break;

            case 1:  // AUTOMATIC MODE
                if (pir_state) {
                    /* Calcular PWM proporcional basado en la temperatura */
                    if (temperature <= auto_t_min) {
                        calculated_pwm = 0;
                    } else if (temperature >= auto_t_max) {
                        calculated_pwm = 100;
                    } else {
                        // Linear interpolation
                        calculated_pwm = (int)(100.0f * (temperature - auto_t_min) / 
                                              (auto_t_max - auto_t_min));
                    }
                } else {
                    calculated_pwm = 0;  // No PIR, no ventilation
                }
                ESP_LOGD(TAG, "MODE AUTOMATIC: T=%.1f, PIR=%d, PWM=%d", 
                         temperature, pir_state, calculated_pwm);
                break;

            case 2:  /* MODO PROGRAMADO */
                /* Comprobar si la hora actual coincide con algún registro activo */
                for (int i = 0; i < 3; i++) {
                    if (!registers[i].active) continue;

                    int current_total_min = current_hour * 60 + current_min;
                    int start_total_min = registers[i].start_hour * 60 + registers[i].start_min;
                    int end_total_min = registers[i].end_hour * 60 + registers[i].end_min;

                    if (current_total_min >= start_total_min && current_total_min < end_total_min) {
                        register_active = i;

                        if (pir_state) {
                            if (temperature <= registers[i].temp_min) {
                                calculated_pwm = 0;
                            } else if (temperature >= registers[i].temp_max) {
                                calculated_pwm = 100;
                            } else {
                                calculated_pwm = (int)(100.0f * 
                                    (temperature - registers[i].temp_min) / 
                                    (registers[i].temp_max - registers[i].temp_min));
                            }
                        } else {
                            calculated_pwm = 0;
                        }
                        break;  // Use first matching register
                    }
                }
                ESP_LOGD(TAG, "MODE PROGRAMMED: Reg=%d, T=%.1f, PIR=%d, PWM=%d", 
                         register_active, temperature, pir_state, calculated_pwm);
                break;

            default:
                calculated_pwm = 0;
                break;
        }

        /* Limitar PWM entre 0 y 100 */
        if (calculated_pwm < 0) calculated_pwm = 0;
        if (calculated_pwm > 100) calculated_pwm = 100;

        /* Enviar comando PWM al ventilador */
        fan_signal_t fan_signal = { .pwm_value = calculated_pwm };
        xQueueOverwrite(fan_queue, &fan_signal);
        fan_control_set_pwm(calculated_pwm);

        /* Actualizar cola de estado del sistema para HTTP/pantalla */
        system_state_t state = {
            .temperature = temperature,
            .pir_state = pir_state,
            .current_mode = current_mode,
            .current_pwm = calculated_pwm,
            .auto_t_min = auto_t_min,
            .auto_t_max = auto_t_max,
            .active_register = register_active,
        };
        /* Copiar registros al estado */
        for (int i = 0; i < 3; i++) state.registers[i] = registers[i];
        xQueueOverwrite(state_queue, &state);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
