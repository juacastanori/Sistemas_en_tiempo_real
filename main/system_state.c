/*
 * system_state.c
 *
 * System state management and fan control logic implementation
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
 * @brief Load configuration from flash on startup
 */
static void load_initial_config(int *current_pwm_out, float *auto_t_min_out, float *auto_t_max_out, scheduled_register_t registers_out[3])
{
    ESP_LOGI(TAG, "Loading initial configuration from flash...");
    // Load manual PWM
    int saved_pwm = 0;
    if (nvs_config_load_manual_pwm(&saved_pwm) == 0) {
        *current_pwm_out = saved_pwm;
        ESP_LOGI(TAG, "Loaded manual PWM: %d%%", *current_pwm_out);
    } else {
        *current_pwm_out = 0;
        ESP_LOGW(TAG, "No saved manual PWM, using default: 0%%");
    }

    // Load automatic temperatures
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

    // Load programmed registers
    nvs_config_load_all_registers(registers_out);
    ESP_LOGI(TAG, "Programmed registers loaded");
}

/**
 * @brief Handle configuration updates from HTTP server
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
    // Read the registers from the latest published system state (state queue)
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
 * @brief Main system control task
 * Implements the fan control logic
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

    // Local state (task-local; not global)
    int current_mode = 0;
    int current_pwm = 0;
    float auto_t_min = 20.0f;
    float auto_t_max = 30.0f;
    scheduled_register_t registers[3] = {0};

    // Load configuration from flash on startup
    load_initial_config(&current_pwm, &auto_t_min, &auto_t_max, registers);

    QueueHandle_t config_queue = queues_get_config_update_queue();
    QueueHandle_t temp_queue = queues_get_temperature_queue();
    QueueHandle_t state_queue = queues_get_system_state_queue();
    QueueHandle_t pir_queue = queues_get_pir_queue();
    QueueHandle_t fan_queue = queues_get_fan_signal_queue();

    config_update_t config_update;

    while (1) {
        // Check for configuration updates (non-blocking)
        if (xQueueReceive(config_queue, &config_update, 0) == pdTRUE) {
            handle_config_update(&config_update, &current_mode, &current_pwm, &auto_t_min, &auto_t_max, registers);
        }

        // Get current temperature
        if (xQueuePeek(temp_queue, &temperature, 0) != pdTRUE) {
            temperature = -99.9f;
        }

        // Get PIR state from central PIR queue
        if (pir_queue != NULL) {
            if (xQueuePeek(pir_queue, &pir_state, 0) != pdTRUE) {
                pir_state = 0;
            }
        } else {
            pir_state = 0;
        }

        // Get current time
        time(&now);
        localtime_r(&now, &timeinfo);
        current_hour = timeinfo.tm_hour;
        current_min = timeinfo.tm_min;

        // Reset PWM and active register
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
                    // Calculate proportional PWM based on temperature
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

            case 2:  // PROGRAMMED MODE
                // Check if current time matches any active register
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

        // Clamp PWM to 0-100
        if (calculated_pwm < 0) calculated_pwm = 0;
        if (calculated_pwm > 100) calculated_pwm = 100;

        // Send PWM command to fan
        fan_signal_t fan_signal = { .pwm_value = calculated_pwm };
        xQueueOverwrite(fan_queue, &fan_signal);
        fan_control_set_pwm(calculated_pwm);

        // Update system state queue for HTTP/display
        system_state_t state = {
            .temperature = temperature,
            .pir_state = pir_state,
            .current_mode = current_mode,
            .current_pwm = calculated_pwm,
            .auto_t_min = auto_t_min,
            .auto_t_max = auto_t_max,
            .active_register = register_active,
        };
        // copy registers into state
        for (int i = 0; i < 3; i++) state.registers[i] = registers[i];
        xQueueOverwrite(state_queue, &state);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
