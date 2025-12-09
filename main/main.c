/*
 * main.c
 *
 * Punto de entrada de la aplicación. Este fichero centraliza la creación de
 * tareas del sistema e inicializa los módulos (NVS, colas, periféricos).
 * Todas las llamadas a `xTaskCreate` se realizan aquí para mantener la
 * secuencia de arranque en un único lugar.
 */

#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "queues.h"
#include "system_state.h"
#include "wifi_app.h"
#include "thermistor_reader.h"
#include "pir_sensor.h"
#include "fan_control.h"
#include "nvs_config.h"
#include "config_app.h"
#include "tasks_common.h"
#include "http_server.h"

static const char TAG[] = "APP_MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting application...");

    /**
     * @brief Inicializa NVS (almacenamiento flash)
     */
    if (nvs_config_init() != 0) {
        ESP_LOGE(TAG, "NVS config initialization failed");
    }

    /**
     * @brief Inicializa la comunicación inter-tareas (colas y semáforos)
     */
    if (queues_init() != 0) {
        ESP_LOGE(TAG, "Queues initialization failed");
        return;
    }

    /**
     * @brief Inicializa periféricos y contextos de hardware
     */
    void *therm_ctx = thermistor_init();  // Contexto del sensor de temperatura (pasar a la tarea)
    pir_sensor_init();  // Inicializa sensor PIR
    fan_control_init(); // Inicializa control PWM del ventilador

    ESP_LOGI(TAG, "All peripherals initialized");

    // ========== CREACIÓN CENTRALIZADA DE TAREAS ==========

    // Inicializa la cola del monitor del servidor HTTP (la tarea se creará en main)
    http_server_init_monitor_queue();

    // Inicializadores de módulo que NO crean tareas (preparan periféricos, colas, ISRs, etc.)
    wifi_app_start(); // crea la cola del wifi_app, pero no crea la tarea

    // Crear tareas desde main (responsabilidad centralizada)

    // Tarea lectora del termistor (pasar el contexto retornado por thermistor_init)
    xTaskCreate(
        thermistor_read_task,
        "thermistor_read_task",
        2048,
        therm_ctx,
        5,
        NULL
    );
    ESP_LOGI(TAG, "Created thermistor_read_task");

    // Tarea de eventos PIR (debounce + publicación de estado)
    xTaskCreate(
        pir_event_task,
        "pir_event_task",
        2048,
        NULL,
        PIR_ISR_TASK_PRIO,
        NULL
    );
    ESP_LOGI(TAG, "Created pir_event_task");

    // Tarea de la aplicación WiFi (anclada a core)
    TaskHandle_t wifi_handle = NULL;
    xTaskCreatePinnedToCore(
        wifi_app_task,
        "wifi_app_task",
        WIFI_APP_TASK_STACK_SIZE,
        NULL,
        WIFI_APP_TASK_PRIORITY,
        &wifi_handle,
        WIFI_APP_TASK_CORE_ID
    );
    ESP_LOGI(TAG, "Created wifi_app_task");

    // Tarea monitor del servidor HTTP (anclada a core)
    TaskHandle_t http_mon_handle = NULL;
    xTaskCreatePinnedToCore(
        http_server_monitor,
        "http_server_monitor",
        HTTP_SERVER_MONITOR_STACK_SIZE,
        NULL,
        HTTP_SERVER_MONITOR_PRIORITY,
        &http_mon_handle,
        HTTP_SERVER_MONITOR_CORE_ID
    );
    /* Informar al servidor HTTP del handle para que pueda detener la tarea si es necesario */
    http_server_set_monitor_task_handle(http_mon_handle);
    ESP_LOGI(TAG, "Created http_server_monitor task");

    // Tarea de control del sistema (lógica del ventilador)
    xTaskCreate(
        system_control_task,
        "system_control_task",
        4096,
        NULL,
        5,
        NULL
    );
    ESP_LOGI(TAG, "Created system_control_task");

    ESP_LOGI(TAG, "Application startup complete");
}
