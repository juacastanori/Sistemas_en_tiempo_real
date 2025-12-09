/*
 * pir_sensor.c
 *
 * Controlador del sensor de movimiento PIR y tarea de procesamiento de eventos.
 * - Instala una ISR de GPIO para capturar transiciones crudas del nivel del PIR
 * - Encola los niveles crudos desde la ISR en una cola pequeña
 * - `pir_event_task` realiza debounce, normaliza la semántica activo-bajo/activo-alto
 *   y publica una bandera única de presencia en la cola central del PIR
 *   (accesible mediante `queues_get_pir_queue()`).
 */

#include "pir_sensor.h"
#include "config_app.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "queues.h"

static const char *TAG = "PIR_SENSOR";

// Estado lógico del PIR: 1 = presencia, 0 = no presencia
// Cola para eventos generados por la ISR (niveles crudos)
static QueueHandle_t pir_evt_queue = NULL;

// Forward
void pir_event_task(void *pvParameters);

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
void pir_event_task(void *pvParameters)
{
    int raw_level = 0;
    while (1) {
        if (xQueueReceive(pir_evt_queue, &raw_level, portMAX_DELAY) == pdTRUE) {
            // Debounce: esperar 150 ms y verificar nivel estable (reduced false negatives)
            vTaskDelay(pdMS_TO_TICKS(150));
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

            /* Publicar a la cola central de PIR */
            QueueHandle_t q = queues_get_pir_queue();
            if (q != NULL) {
                xQueueOverwrite(q, &detected);
            }

            ESP_LOGI(TAG, "PIR State published: %d (raw=%d)", detected, stable);
        }
    }
}

void pir_sensor_init(void)
{
    ESP_LOGI(TAG, "Inicializando sensor PIR en GPIO %d (active_low=%d)", PIR_SENSOR_GPIO, PIR_ACTIVE_LOW);

    // Cola para eventos desde la ISR (tamaño aumentado para evitar perder eventos)
    pir_evt_queue = xQueueCreate(20, sizeof(int));
    if (pir_evt_queue == NULL) {
        ESP_LOGE(TAG, "No se pudo crear pir_evt_queue");
        return;
    }

    // Configurar GPIO del PIR como entrada con interrupciones
    gpio_config_t pir_config = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIR_SENSOR_GPIO),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    // Ajustar pull y tipo de interrupción según la polaridad del sensor
    // Ambos flancos (ANYEDGE) capturan ambas transiciones para mayor robustez
#if PIR_ACTIVE_LOW
    // Sensor activo cuando el pin baja -> pull-up, capturar ambos flancos para debounce
    pir_config.pull_up_en = GPIO_PULLUP_ENABLE;    // reposo = HIGH
    pir_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pir_config.intr_type = GPIO_INTR_ANYEDGE;      // Capturar flanco ascendente y descendente
#else
    // Sensor activo cuando el pin sube -> pull-down, capturar ambos flancos para debounce
    pir_config.pull_up_en = GPIO_PULLUP_DISABLE;
    pir_config.pull_down_en = GPIO_PULLDOWN_ENABLE; // reposo = LOW
    pir_config.intr_type = GPIO_INTR_ANYEDGE;       // Capturar flanco ascendente y descendente
#endif

    ESP_ERROR_CHECK(gpio_config(&pir_config));

    // Instalar servicio ISR (0 = banderas por defecto)
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    /* Registrar el manejador para el pin del PIR */
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIR_SENSOR_GPIO, pir_gpio_isr_handler, (void *)PIR_SENSOR_GPIO));

    ESP_LOGI(TAG, "Sensor PIR inicializado correctamente (ISR)");
}

/**
 * @brief Reinicia el temporizador de presencia del PIR
 *
 * Esta función permite reiniciar el temporizador que controla el timeout
 * de presencia detectada. Implementación futura si se requiere.
 */
void pir_sensor_reset_timeout(void)
{
    // Implementación futura si se requiere timeout de presencia
}

/**
 * @brief Verifica si el sensor PIR está configurado como activo-bajo
 *
 * @return true si el PIR es activo-bajo; false si es activo-alto
 */
bool pir_sensor_is_active_low(void)
{
    return (PIR_ACTIVE_LOW != 0);
}
