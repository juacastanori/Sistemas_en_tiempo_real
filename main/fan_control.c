/*
 * fan_control.c
 *
 * Control del ventilador mediante PWM usando LEDC.
 * Inicialización y APIs de set/get. El módulo deliberadamente no
 * mantiene estado mutable interno; el valor PWM autorizado es proporcionado
 * por el módulo `system_state` y leído a través de la cola de estado del sistema.
 */

#include "fan_control.h"
#include "config_app.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "queues.h"

static const char *TAG = "FAN_CONTROL";

/* No hay estado de PWM a nivel de módulo; system_state es la fuente de verdad. */

/**
 * @brief Inicializa el controlador del ventilador
 * Configura el timer y el canal LEDC usados para generar la señal PWM
 * que controla la velocidad del ventilador. No mantiene estado interno;
 * la referencia de PWM proviene del módulo `system_state`.
 *
 * @param None
 * @return void
 */
void fan_control_init(void)
{
    ESP_LOGI(TAG, "Starting PWM in GPIO %d", FAN_PWM_GPIO);

    // --- Configuración del Timer LEDC ---
    ledc_timer_config_t ledc_timer = {
        .speed_mode = FAN_LEDC_MODE,
        .timer_num = FAN_LEDC_TIMER,
        .duty_resolution = FAN_LEDC_RESOLUTION,  // 10-bit resolution
        .freq_hz = FAN_LEDC_FREQUENCY,           // 5 kHz
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ESP_LOGI(TAG, "LEDC timer configured: frequency=%d Hz", FAN_LEDC_FREQUENCY);

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
    ESP_LOGI(TAG, "LEDC channel configured on GPIO %d", FAN_PWM_GPIO);

    ESP_LOGI(TAG, "Fan control initialized successfully");
}

/**
 * @brief Establece el PWM del ventilador en porcentaje.
 *
 * Normaliza el valor entre 0 y 100, lo convierte al rango del LEDC
 * (resolución configurada en `FAN_LEDC_RESOLUTION`) y aplica el duty
 * correspondiente al canal LEDC configurado.
 *
 * @param pwm_percent El porcentaje de PWM deseado (0 = apagado, 100 = máximo).
 * @return void
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

    (void)pwm_percent; // PWM actual ya usado arriba, evitar advertencia si no se usa
    ESP_LOGI(TAG, "PWM: %d%% (duty=%lu)", pwm_percent, duty);
}

/**
 * @brief Recupera el PWM actual desde la cola de estado del sistema.
 *
 * Busca el último estado publicado en la cola de `system_state` y devuelve
 * el valor `current_pwm` encontrado. Si la cola no está disponible o
 * no hay estado, devuelve 0.
 *
 * @param None
 * @return int El porcentaje de PWM actual (0-100). 0 si no hay estado disponible.
 */
int fan_control_get_pwm(void)
{
    // Leer el PWM actual desde la cola de estado del sistema
    QueueHandle_t state_q = queues_get_system_state_queue();
    if (state_q == NULL) return 0;
    system_state_t st;
    if (xQueuePeek(state_q, &st, 0) == pdTRUE) {
        return st.current_pwm;
    }
    return 0;
}
