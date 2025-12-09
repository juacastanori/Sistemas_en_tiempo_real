/*
 * wifi_app.h
 *
 * Interfaz pública de la aplicación WiFi.
 * La documentación está en Español (Doxygen). Las cadenas de log deben
 * mantenerse en inglés según la convención del proyecto.
 */

#ifndef MAIN_WIFI_APP_H_
#define MAIN_WIFI_APP_H_

#include "esp_netif.h"
#include "esp_wifi_types.h"
#include "freertos/FreeRTOS.h"

// Callback typedef
typedef void (*wifi_connected_event_callback_t)(void);

// === Configuración de la red Wi-Fi (Modo Access Point - AP) ===
#define WIFI_AP_SSID                    "Elpepe"                // Nombre del AP
#define WIFI_AP_PASSWORD                "contrasena"            // Contraseña del AP
#define WIFI_AP_CHANNEL                 1                       // Canal del AP
#define WIFI_AP_SSID_HIDDEN             0                       // Visibilidad del AP
#define WIFI_AP_MAX_CONNECTIONS         5                       // Máximo de clientes
#define WIFI_AP_BEACON_INTERVAL         100                     // Beacon: 100 ms recomendado
#define WIFI_AP_IP                      "192.168.0.1"           // IP por defecto del AP
#define WIFI_AP_GATEWAY                 "192.168.0.1"           // Gateway por defecto del AP
#define WIFI_AP_NETMASK                 "255.255.255.0"         // Máscara de red del AP
#define WIFI_AP_BANDWIDTH               WIFI_BW_HT20            // Ancho de banda 20 MHz

// === Configuración de la red Wi-Fi (Modo Station - STA) ===
// NOTA: En una app real, estas credenciales deberían venir de la configuración NVS
#define WIFI_STA_SSID                   "Linda"         // SSID de la red a la que conectarse
#define WIFI_STA_PASSWORD               "Hostallinda0409"     // Contraseña de la red
#define WIFI_STA_POWER_SAVE             WIFI_PS_NONE            // No usar ahorro de energía

#define MAX_SSID_LENGTH                 32                      // Estándar IEEE máximo
#define MAX_PASSWORD_LENGTH             64                      // Estándar IEEE máximo
#define MAX_CONNECTION_RETRIES          5                       // Número de reintentos de desconexión

/* Objeto netif para la estación (STA) y punto de acceso (AP).
 * Internals: los objetos `esp_netif` son locales a `wifi_app.c`. Si se necesita
 * acceso externo, exponer funciones accesoras en lugar de variables globales. */

/**
 * @brief Identificadores de mensaje para la cola de la tarea WiFi.
 */
typedef enum wifi_app_message
{
    WIFI_APP_MSG_START_HTTP_SERVER = 0,
    WIFI_APP_MSG_CONNECTING_FROM_HTTP_SERVER,
    WIFI_APP_MSG_STA_CONNECTED_GOT_IP,
    WIFI_APP_MSG_USER_REQUESTED_STA_DISCONNECT,
    WIFI_APP_MSG_LOAD_SAVED_CREDENTIALS,
    WIFI_APP_MSG_STA_DISCONNECTED,
    WIFI_APP_MSG_STA_CONNECTED_FAIL, // Nuevo: Falla de conexión STA
} wifi_app_message_e;

/**
 * Structure for the message queue
 * @note Expand this based on application requirements e.g. add another type and parameter as required
 */
typedef struct wifi_app_queue_message
{
    wifi_app_message_e msgID;
} wifi_app_queue_message_t;

/**
 * @brief Envía un mensaje a la cola de la aplicación WiFi.
 * @param msgID Identificador del mensaje a enviar.
 * @return pdTRUE si el envío tuvo éxito, pdFALSE en caso contrario.
 */
BaseType_t wifi_app_send_message(wifi_app_message_e msgID);

/**
 * @brief Inicializa y crea la tarea WiFi (debe ser invocada desde `main`).
 */
void wifi_app_start(void);

/**
 * @brief Tarea RTOS principal de la aplicación WiFi.
 * @param pvParameters Parámetro pasado a la tarea (no usado).
 */
void wifi_app_task(void *pvParameters);

/**
 * @brief Obtiene la estructura de configuración WiFi (si está disponible).
 * @return Puntero a `wifi_config_t` o NULL si no está implementado.
 */
wifi_config_t* wifi_app_get_wifi_config(void);

/**
 * @brief Registra un callback que será invocado cuando se establezca conexión.
 */
void wifi_app_set_callback(wifi_connected_event_callback_t cb);

/**
 * @brief Invoca el callback registrado (si existe).
 */
void wifi_app_call_callback(void);

/**
 * @brief Devuelve el RSSI actual de la conexión WiFi.
 * @return Nivel RSSI actual.
 */
int8_t wifi_app_get_rssi(void);

/**
 * Intenta conectar la estación (STA) a la red.
 */
void wifi_app_connect_sta(void);

#endif /* MAIN_WIFI_APP_H_ */