/**
 * Application entry point.
 * All tasks are created here (thermistor, PIR, fan control, WiFi)
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

    // Initialize NVS (flash storage)
    if (nvs_config_init() != 0) {
        ESP_LOGE(TAG, "NVS config initialization failed");
    }

    // Initialize inter-task communication (queues and semaphores)
    if (queues_init() != 0) {
        ESP_LOGE(TAG, "Queues initialization failed");
        return;
    }

    // Initialize hardware peripherals
    void *therm_ctx = thermistor_init();  // Temperature sensor context (pass to task)
    pir_sensor_init();  // PIR motion sensor
    fan_control_init(); // Fan PWM control

    ESP_LOGI(TAG, "All peripherals initialized");

    // ========== CREATE ALL SYSTEM TASKS ==========

    // Initialize HTTP server monitor queue (task will be created in main)
    http_server_init_monitor_queue();

    // Start module-level initializers that do NOT create tasks
    // (they prepare peripherals, queues, ISRs, etc.)
    wifi_app_start(); // creates wifi_app queue but NOT the task

    // Create tasks from main (centralized responsibility)

    // Thermistor reader task (pass context returned by thermistor_init)
    xTaskCreate(
        thermistor_read_task,
        "thermistor_read_task",
        2048,
        therm_ctx,
        5,
        NULL
    );
    ESP_LOGI(TAG, "Created thermistor_read_task");

    // PIR event task (debounce + state)
    xTaskCreate(
        pir_event_task,
        "pir_event_task",
        2048,
        NULL,
        PIR_ISR_TASK_PRIO,
        NULL
    );
    ESP_LOGI(TAG, "Created pir_event_task");

    // WiFi app task (pinned)
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

    // HTTP server monitor task (pinned)
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
    // Let http_server know about the handle so it can stop the task later
    http_server_set_monitor_task_handle(http_mon_handle);
    ESP_LOGI(TAG, "Created http_server_monitor task");

    // System control task (fan control logic)
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
