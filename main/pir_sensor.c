#include "pir_sensor.h"
#include "config_app.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

static const char *TAG = "PIR_SENSOR";

// Estado lógico del PIR: 1 = presencia, 0 = no presencia
static int pir_state = 0;

// Cola que expone el estado al resto del sistema (opcional)
static QueueHandle_t pir_queue_handle = NULL;

// Cola para eventos generados por la ISR (niveles crudos)
static QueueHandle_t pir_evt_queue = NULL;

// Forward
static void pir_event_task(void *pvParameters);

static void IRAM_ATTR pir_gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    int level = gpio_get_level(gpio_num);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (pir_evt_queue != NULL) {
        xQueueSendFromISR(pir_evt_queue, &level, &xHigherPriorityTaskWoken);
    }
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Tarea que procesa eventos del PIR (debounce y normalización de nivel)
 */
static void pir_event_task(void *pvParameters)
{
    int raw_level = 0;
    while (1) {
        if (xQueueReceive(pir_evt_queue, &raw_level, portMAX_DELAY) == pdTRUE) {
            // Debounce simple: esperar 50 ms y verificar nivel estable
            vTaskDelay(pdMS_TO_TICKS(50));
            int stable = gpio_get_level(PIR_SENSOR_GPIO);
            if (stable != raw_level) {
                // Nivel cambió durante debounce; ignorar este evento
                continue;
            }

            // Normalizar según si el PIR es activo-bajo o activo-alto
#if PIR_ACTIVE_LOW
            int detected = (stable == 0) ? 1 : 0;
#else
            int detected = (stable == 1) ? 1 : 0;
#endif

            pir_state = detected;

            if (pir_queue_handle != NULL) {
                xQueueOverwrite(pir_queue_handle, &pir_state);
            }

            ESP_LOGI(TAG, "PIR State: %d (raw=%d)", pir_state, stable);
        }
    }
}

/**
 * @brief Inicializa el sensor PIR usando interrupciones GPIO para mayor rapidez
 */
void pir_sensor_init(void)
{
    ESP_LOGI(TAG, "Inicializando sensor PIR en GPIO %d (active_low=%d)", PIR_SENSOR_GPIO, PIR_ACTIVE_LOW);

    // Cola para exponer estado a otras tareas
    pir_queue_handle = xQueueCreate(1, sizeof(int));
    if (pir_queue_handle == NULL) {
        ESP_LOGW(TAG, "Failed to create pir_queue_handle");
    }

    // Cola para eventos desde la ISR
    pir_evt_queue = xQueueCreate(10, sizeof(int));
    if (pir_evt_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create pir_evt_queue");
        return;
    }

    // Configurar GPIO del PIR como entrada con interrupciones
    gpio_config_t pir_config = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIR_SENSOR_GPIO),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };

    // Ajustar pull según la polaridad del sensor
#if PIR_ACTIVE_LOW
    pir_config.pull_up_en = GPIO_PULLUP_ENABLE;    // idle = HIGH
    pir_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
#else
    pir_config.pull_up_en = GPIO_PULLUP_DISABLE;
    pir_config.pull_down_en = GPIO_PULLDOWN_ENABLE; // idle = LOW
#endif

    ESP_ERROR_CHECK(gpio_config(&pir_config));

    // Instalar servicio ISR (0 = default flags)
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // Registrar el handler para el pin del PIR
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIR_SENSOR_GPIO, pir_gpio_isr_handler, (void *)PIR_SENSOR_GPIO));

    // Crear tarea que procesa los eventos (debounce y actualiza pir_state)
    xTaskCreate(pir_event_task, "pir_event_task", 2048, NULL, 10, NULL);

    ESP_LOGI(TAG, "PIR sensor inicializado correctamente (ISR)");
}

int pir_sensor_get_state(void)
{
    return pir_state;
}

void pir_sensor_reset_timeout(void)
{
    // Implementación futura si se requiere timeout de presencia
}

bool pir_sensor_is_active_low(void)
{
    return (PIR_ACTIVE_LOW != 0);
}
