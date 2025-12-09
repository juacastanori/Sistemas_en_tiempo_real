/*
 * http_server.h
 *
 * Created on: Oct 20, 2021
 * Author: kjagu
 */

#ifndef MAIN_HTTP_SERVER_H_
#define MAIN_HTTP_SERVER_H_
#include "esp_http_server.h"
#include "esp_err.h"

// OTA states
#define OTA_UPDATE_PENDING        0
#define OTA_UPDATE_SUCCESSFUL     1
#define OTA_UPDATE_FAILED        -1

/**
 * Messages for the HTTP monitor
 */
typedef enum http_server_message
{
    HTTP_MSG_WIFI_CONNECT_INIT = 0,
    HTTP_MSG_WIFI_CONNECT_SUCCESS,
    HTTP_MSG_WIFI_CONNECT_FAIL,
    HTTP_MSG_OTA_UPDATE_SUCCESSFUL,
    HTTP_MSG_OTA_UPDATE_FAILED,
    HTTP_MSG_START_SERVER,
    HTTP_MSG_STOP_SERVER,

} http_server_message_e;

/**
 * Structure for the message queue
 */
typedef struct http_server_queue_message
{
    http_server_message_e msgID;

} http_server_queue_message_t;

/* ============================================================
 *      PROGRAM STRUCTURES USED BY WEB SERVER
 * ============================================================*/
typedef struct {
    int active;
    uint8_t start_hour, start_min;
    uint8_t end_hour, end_min;
    float temp_min;
    float temp_max;

} scheduled_register_t;

/**
 * Array of the 3 programmed registers (access via system_state API)
 */

/* ============================================================
 *            PUBLIC HANDLERS USED IN http_server.c
 * ============================================================*/

/**
 * Sends a message to the queue
 */
BaseType_t http_server_monitor_send_message(http_server_message_e msgID);

/**
 * Starts the HTTP server.
 */
void http_server_start(void);

/**
 * @brief Initialize the internal monitor queue for the HTTP server.
 * Call this from `app_main` before creating the `http_server_monitor` task.
 */
void http_server_init_monitor_queue(void);

/**
 * @brief The HTTP server monitor task function (create this from main).
 */
void http_server_monitor(void *parameter);

/**
 * @brief Set the internal monitor task handle (call from main after creating task).
 */
void http_server_set_monitor_task_handle(TaskHandle_t handle);

/**
 * Stops the HTTP server.
 */
void http_server_stop(void);

/**
 * Timer callback executed after a successful OTA update.
 */
void http_server_fw_update_reset_callback(void *arg);



#endif /* MAIN_HTTP_SERVER_H_ */
