/*
 * http_server.h
 * Definiciones del servidor HTTP y API de la tarea monitor.
 */

#ifndef MAIN_HTTP_SERVER_H_
#define MAIN_HTTP_SERVER_H_
#include "esp_http_server.h"
#include "esp_err.h"

// Estados de OTA
#define OTA_UPDATE_PENDING        0
#define OTA_UPDATE_SUCCESSFUL     1
#define OTA_UPDATE_FAILED        -1

/**
 * Mensajes para el monitor HTTP
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
 * Estructura para la cola de mensajes
 */
typedef struct http_server_queue_message
{
    http_server_message_e msgID;

} http_server_queue_message_t;

/* ============================================================
 *      ESTRUCTURAS DE PROGRAMA UTILIZADAS POR EL SERVIDOR WEB
 * ============================================================*/
typedef struct {
    int active;
    uint8_t start_hour, start_min;
    uint8_t end_hour, end_min;
    float temp_min;
    float temp_max;

} scheduled_register_t;

/**
 * Array de los 3 registros programados (acceso vía API de system_state)
 */

/* ============================================================
 *            MANEJADORES PÚBLICOS UTILIZADOS EN http_server.c
 * ============================================================*/

/**
 * @brief Envía un mensaje a la cola
 */
BaseType_t http_server_monitor_send_message(http_server_message_e msgID);

/**
 * @brief Inicia el servidor HTTP.
 */
void http_server_start(void);

/**
 * @brief Inicializa la cola interna del monitor para el servidor HTTP.
 * Llamar desde `app_main` antes de crear la tarea `http_server_monitor`.
 */
void http_server_init_monitor_queue(void);

/**
 * @brief Función de tarea del monitor del servidor HTTP (crear desde main).
 */
void http_server_monitor(void *parameter);

/**
 * @brief Establece el handle de la tarea monitor interna (llamar desde main después de crear la tarea).
 */
void http_server_set_monitor_task_handle(TaskHandle_t handle);

/**
 * @brief Detiene el servidor HTTP.
 */
void http_server_stop(void);

/**
 * @brief Callback del timer ejecutado después de una actualización de firmware exitosa.
 */
void http_server_fw_update_reset_callback(void *arg);



#endif /* MAIN_HTTP_SERVER_H_ */
