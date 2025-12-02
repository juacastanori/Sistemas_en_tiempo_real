#include "pir_sensor.h"
#include "config_app.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "PIR_SENSOR";

// Variable global que almacena el estado del PIR
static int pir_state = 0;

// Cola opcional para comunicación entre tareas (similar a temperatura)
static QueueHandle_t pir_queue_handle = NULL;

/**
 * @brief Tarea que lee el estado del PIR periódicamente
 */
static void pir_read_task(void *pvParameters)
{
    while (1) {
        // Leer el nivel del GPIO del PIR
        pir_state = gpio_get_level(PIR_SENSOR_GPIO);

        // Opcionalmente, enviar a la cola si se usa
        if (pir_queue_handle != NULL) {
            xQueueOverwrite(pir_queue_handle, &pir_state);
        }

        ESP_LOGI(TAG, "PIR State: %d", pir_state);

        vTaskDelay(pdMS_TO_TICKS(PIR_READ_INTERVAL_MS));
    }
}

/**
 * @brief Inicializa el sensor PIR
 */
void pir_sensor_init(void)
{
    ESP_LOGI(TAG, "Inicializando sensor PIR en GPIO %d", PIR_SENSOR_GPIO);

    // Crear cola opcional para PIR (similar a temperatura)
    pir_queue_handle = xQueueCreate(1, sizeof(int));
    if (pir_queue_handle == NULL) {
        ESP_LOGW(TAG, "Failed to create pir_queue_handle");
    }

    // Configurar GPIO del PIR como entrada
    gpio_config_t pir_config = {
        .intr_type = GPIO_INTR_DISABLE,     // Sin interrupciones, lectura polling
        .mode = GPIO_MODE_INPUT,             // Entrada
        .pin_bit_mask = (1ULL << PIR_SENSOR_GPIO),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,   // Pull-down por defecto (sin presencia = LOW)
        .pull_up_en = GPIO_PULLUP_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&pir_config));

    // Crear la tarea de lectura del PIR
    xTaskCreate(pir_read_task, "pir_read_task", 2048, NULL, 4, NULL);

    ESP_LOGI(TAG, "PIR sensor inicializado correctamente");
}

/**
 * @brief Obtiene el estado del PIR
 */
int pir_sensor_get_state(void)
{
    return pir_state;
}

/**
 * @brief Reinicia el timeout del PIR (opcional, no usado actualmente)
 */
void pir_sensor_reset_timeout(void)
{
    // Implementación futura si se requiere timeout de presencia
}
