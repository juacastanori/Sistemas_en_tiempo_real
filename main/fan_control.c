#include "fan_control.h"
#include "config_app.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "queues.h"

static const char *TAG = "FAN_CONTROL";

// No module-level PWM state; system_state is the source of truth.

/**
 * @brief Inicializa el controlador del ventilador
 */
void fan_control_init(void)
{
    ESP_LOGI(TAG, "Inicializando control del ventilador (PWM) en GPIO %d", FAN_PWM_GPIO);

    // --- Configuración del Timer LEDC ---
    ledc_timer_config_t ledc_timer = {
        .speed_mode = FAN_LEDC_MODE,
        .timer_num = FAN_LEDC_TIMER,
        .duty_resolution = FAN_LEDC_RESOLUTION,  // 10-bit resolution
        .freq_hz = FAN_LEDC_FREQUENCY,           // 5 kHz
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ESP_LOGI(TAG, "Timer LEDC configurado: frecuencia=%d Hz", FAN_LEDC_FREQUENCY);

    // --- Configuración del Canal LEDC ---
    ledc_channel_config_t ledc_channel = {
        .speed_mode = FAN_LEDC_MODE,
        .channel = FAN_LEDC_CHANNEL,
        .timer_sel = FAN_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = FAN_PWM_GPIO,
        .duty = 0,                      // Comienza en 0% (apagado)
        .hpoint = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_LOGI(TAG, "Canal LEDC configurado en GPIO %d", FAN_PWM_GPIO);

    ESP_LOGI(TAG, "Fan control inicializado correctamente");
}

/**
 * @brief Establece el PWM del ventilador (0-100%)
 */
void fan_control_set_pwm(int pwm_percent)
{
    // Limitar el valor entre 0 y 100
    if (pwm_percent < 0) pwm_percent = 0;
    if (pwm_percent > 100) pwm_percent = 100;

    // Convertir el porcentaje (0-100) al rango del LEDC (0-1023 para 10-bit)
    // Máximo duty = (2^10) - 1 = 1023
    uint32_t duty = (pwm_percent * 1023) / 100;

    // Establecer el duty cycle
    ESP_ERROR_CHECK(ledc_set_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL, duty));

    // Aplicar el cambio (es necesario llamar a ledc_update_duty)
    ESP_ERROR_CHECK(ledc_update_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL));

    (void)pwm_percent; // current PWM tracked centrally by system_state
    ESP_LOGI(TAG, "PWM establecido a %d%% (duty=%lu)", pwm_percent, duty);
}

/**
 * @brief Obtiene el PWM actual
 */
int fan_control_get_pwm(void)
{
    // Read latest system state from state queue
    QueueHandle_t state_q = queues_get_system_state_queue();
    if (state_q == NULL) return 0;
    system_state_t st;
    if (xQueuePeek(state_q, &st, 0) == pdTRUE) {
        return st.current_pwm;
    }
    return 0;
}
