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
#define WIFI_STA_SSID                   "BLANCA"         // SSID de la red a la que conectarse
#define WIFI_STA_PASSWORD               "24431016"     // Contraseña de la red
#define WIFI_STA_POWER_SAVE             WIFI_PS_NONE            // No usar ahorro de energía

#define MAX_SSID_LENGTH                 32                      // Estándar IEEE máximo
#define MAX_PASSWORD_LENGTH             64                      // Estándar IEEE máximo
#define MAX_CONNECTION_RETRIES          5                       // Número de reintentos de desconexión

// netif object for the Station and Access Point
extern esp_netif_t* esp_netif_sta;
extern esp_netif_t* esp_netif_ap;

/**
 * Message IDs for the WiFi application task
 * @note Expand this based on your application requirements.
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
 * Sends a message to the queue
 * @param msgID message ID from the wifi_app_message_e enum.
 * @return pdTRUE if an item was successfully sent to the queue, otherwise pdFALSE.
 * @note Expand the parameter list based on your requirements e.g. how you've expanded the wifi_app_queue_message_t.
 */
BaseType_t wifi_app_send_message(wifi_app_message_e msgID);

/**
 * Starts the WiFi RTOS task
 */
void wifi_app_start(void);

/**
 * Gets the wifi configuration
 */
wifi_config_t* wifi_app_get_wifi_config(void);

/**
 * Sets the callback function.
 */
void wifi_app_set_callback(wifi_connected_event_callback_t cb);

/**
 * Calls the callback function.
 */
void wifi_app_call_callback(void);

/**
 * Gets the RSSI value of the Wifi connection.
 * @return current RSSI level.
 */
int8_t wifi_app_get_rssi(void);

/**
 * Intenta conectar la estación (STA) a la red.
 */
void wifi_app_connect_sta(void);

#endif /* MAIN_WIFI_APP_H_ */