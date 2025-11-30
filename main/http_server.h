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

} http_server_message_e;

/**
 * Structure for the message queue
 */
typedef struct http_server_queue_message
{
    http_server_message_e msgID;

} http_server_queue_message_t;

/* ============================================================
 *      GLOBAL SYSTEM VARIABLES (USED BY WEB SERVER)
 * ============================================================*/
extern int g_current_mode;
extern int g_current_pwm;
extern float g_auto_t_min;
extern float g_auto_t_max;
extern int g_pir_state;

/**
 * Programmed schedule register structure
 */
typedef struct {
    int active;
    uint8_t start_hour, start_min;
    uint8_t end_hour, end_min;
    float temp_min;
    float temp_max;

} scheduled_register_t;

/**
 * Array of the 3 programmed registers
 */
extern scheduled_register_t g_registers[3];

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
 * Stops the HTTP server.
 */
void http_server_stop(void);

/**
 * Timer callback executed after a successful OTA update.
 */
void http_server_fw_update_reset_callback(void *arg);

#endif /* MAIN_HTTP_SERVER_H_ */
