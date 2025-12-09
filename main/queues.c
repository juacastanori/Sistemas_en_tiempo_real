/*
 * queues.c
 *
 * Inicialización y gestión de colas y semáforos
 */

#include "queues.h"
#include "http_server.h"
#include "esp_log.h"

static const char TAG[] = "QUEUES";

// Handles de las colas
static QueueHandle_t config_update_queue = NULL;      // HTTP -> Config
static QueueHandle_t system_state_queue = NULL;       // Sistema -> HTTP/Visualización
static QueueHandle_t fan_signal_queue = NULL;         // Sistema -> Ventilador
static QueueHandle_t temperature_queue = NULL;        // Termistor -> Sistema
static QueueHandle_t pir_state_queue = NULL;         // PIR -> Sistema

// Cola del monitor del servidor HTTP y cola de estado
static QueueHandle_t http_server_monitor_queue = NULL; // mensajes del monitor HTTP
static QueueHandle_t http_server_status_queue = NULL; // int fw_update_status (un int)

// Mutex para acceso a configuración
static SemaphoreHandle_t config_mutex = NULL;

/**
 * @brief Inicializa todas las colas y semáforos
 */
int queues_init(void)
{
    // Cola de actualizaciones de configuración (el servidor HTTP envía cambios)
    config_update_queue = xQueueCreate(5, sizeof(config_update_t));
    if (config_update_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create config_update_queue");
        return -1;
    }

    // Cola de estado del sistema (para mostrar/monitorizar)
    system_state_queue = xQueueCreate(1, sizeof(system_state_t));
    if (system_state_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create system_state_queue");
        return -1;
    }

    // Cola de señales del ventilador (comandos PWM)
    fan_signal_queue = xQueueCreate(1, sizeof(fan_signal_t));
    if (fan_signal_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create fan_signal_queue");
        return -1;
    }

    // Cola de temperatura (desde el termistor)
    temperature_queue = xQueueCreate(1, sizeof(float));
    if (temperature_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create temperature_queue");
        return -1;
    }

    // Cola de estado del PIR (último int)
    pir_state_queue = xQueueCreate(1, sizeof(int));
    if (pir_state_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create pir_state_queue");
        return -1;
    }

    // Cola del monitor del servidor HTTP (para mensajes como resultado de OTA, start/stop)
    http_server_monitor_queue = xQueueCreate(5, sizeof(http_server_queue_message_t));
    if (http_server_monitor_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create http_server_monitor_queue");
        return -1;
    }

    // Cola de estado del servidor HTTP (un int)
    http_server_status_queue = xQueueCreate(1, sizeof(int));
    if (http_server_status_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create http_server_status_queue");
        return -1;
    }

    // Mutex para proteger variables de configuración
    config_mutex = xSemaphoreCreateMutex();
    if (config_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create config_mutex");
        return -1;
    }

    ESP_LOGI(TAG, "All queues and semaphores initialized successfully");
    return 0;
}

/**
 * @brief Desinicializa todas las colas y semáforos
 */
int queues_deinit(void)
{
    if (config_update_queue) vQueueDelete(config_update_queue);
    if (system_state_queue) vQueueDelete(system_state_queue);
    if (fan_signal_queue) vQueueDelete(fan_signal_queue);
    if (temperature_queue) vQueueDelete(temperature_queue);
    if (config_mutex) vSemaphoreDelete(config_mutex);

    if (pir_state_queue) vQueueDelete(pir_state_queue);
    if (http_server_monitor_queue) vQueueDelete(http_server_monitor_queue);
    if (http_server_status_queue) vQueueDelete(http_server_status_queue);

    ESP_LOGI(TAG, "All queues and semaphores deinitialized successfully");
    return 0;
}

/**
 * @brief Obtener handles de las colas (para otros módulos)
 */
QueueHandle_t queues_get_config_update_queue(void)
{
    return config_update_queue;
}

QueueHandle_t queues_get_system_state_queue(void)
{
    return system_state_queue;
}

QueueHandle_t queues_get_fan_signal_queue(void)
{
    return fan_signal_queue;
}

QueueHandle_t queues_get_temperature_queue(void)
{
    return temperature_queue;
}

QueueHandle_t queues_get_pir_queue(void)
{
    return pir_state_queue;
}

QueueHandle_t queues_get_http_monitor_queue(void)
{
    return http_server_monitor_queue;
}

QueueHandle_t queues_get_http_status_queue(void)
{
    return http_server_status_queue;
}

SemaphoreHandle_t queues_get_config_mutex(void)
{
    return config_mutex;
}
