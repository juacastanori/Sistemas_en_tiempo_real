/**
 * @file wifi_app.c
 * @brief Tarea y utilidades de la aplicación WiFi.
 *
 * Este módulo inicializa la pila TCP/IP, configura los modos Station y
 * SoftAP, registra los manejadores de eventos de WiFi/IP y publica eventos
 * relevantes a la cola de la aplicación. Expone la tarea `wifi_app_task`
 * y una API basada en colas para solicitar acciones desde otros módulos.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "lwip/netdb.h"

#include "http_server.h"
#include "tasks_common.h"
#include "wifi_app.h"
#include "sntp_time_sync.h"
#include "nvs_config.h"

static const char TAG [] = "wifi_app";

static QueueHandle_t wifi_app_queue_handle;

// Variables para el modo Station (STA)
/* retry_count se movió a wifi_app_task para evitar variables globales de alcance de archivo */

/* Los manejadores esp_netif se crean por tarea y no son globales. */

/**
 * @brief Manejador de eventos WiFi/IP para la aplicación.
 *
 * Procesa eventos del driver WiFi (WIFI_EVENT) y del stack IP (IP_EVENT)
 * y publica mensajes apropiados a la cola de la aplicación.
 *
 * @param arg Puntero de usuario (no utilizado)
 * @param event_base Base del evento (WIFI_EVENT o IP_EVENT)
 * @param event_id Identificador del evento
 * @param event_data Datos específicos del evento
 * @return void
 */
static void wifi_app_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "WIFI_EVENT_AP_START: AP 'Elpepe' started."); // AP confirmation
                break;

            case WIFI_EVENT_AP_STOP:
                ESP_LOGI(TAG, "WIFI_EVENT_AP_STOP");
                break;

            case WIFI_EVENT_AP_STACONNECTED:
                ESP_LOGI(TAG, "WIFI_EVENT_AP_STACONNECTED");
                break;

            case WIFI_EVENT_AP_STADISCONNECTED:
                ESP_LOGI(TAG, "WIFI_EVENT_AP_STADISCONNECTED");
                break;

            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WIFI_EVENT_STA_START. Attempting to connect to network...");
                break;

            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED. Waiting for IP...");
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
                        {
                            /* Delegar manejo de desconexión a la tarea wifi_app (mantiene estado de reintentos en tarea) */
                            wifi_event_sta_disconnected_t *disconnected = (wifi_event_sta_disconnected_t *)event_data;
                            ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED. Reason: %d", disconnected->reason);
                            sntp_time_sync_stop();
                            wifi_app_send_message(WIFI_APP_MSG_STA_DISCONNECTED);
                        }
                break;
        }
    }
    else if (event_base == IP_EVENT)
    {
        switch (event_id)
        {
                case IP_EVENT_STA_GOT_IP:
                {
                    ip_event_got_ip_t *ip_event = (ip_event_got_ip_t *)event_data;
                    ESP_LOGI(TAG, "IP_EVENT_STA_GOT_IP. IP: " IPSTR, IP2STR(&ip_event->ip_info.ip));

                    /* Notificar a la aplicación que tenemos una IP (iniciar servidor HTTP y SNTP) */
                    wifi_app_send_message(WIFI_APP_MSG_STA_CONNECTED_GOT_IP);
                }
                break;
        }
    }
}

/**
 * @brief Inicializa los manejadores de eventos WiFi e IP.
 */
static void wifi_app_event_handler_init(void)
{
    /* Bucle de eventos para el driver de WiFi */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Manejador de eventos para la conexión */
    esp_event_handler_instance_t instance_wifi_event;
    esp_event_handler_instance_t instance_ip_event;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_app_event_handler, NULL, &instance_wifi_event));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &wifi_app_event_handler, NULL, &instance_ip_event));
}

/**
 * @brief Inicializa la pila TCP/IP y la configuración WiFi por defecto.
 */
static void wifi_app_default_wifi_init(esp_netif_t **out_sta, esp_netif_t **out_ap)
{
    // Inicializa la pila TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());

    /* Configuración WiFi por defecto - las operaciones deben estar en este orden */
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    
    // Crear interfaces de red para STA y AP
    if (out_sta) *out_sta = esp_netif_create_default_wifi_sta();
    if (out_ap) *out_ap = esp_netif_create_default_wifi_ap();
}

/**
 * @brief Configura el punto de acceso (SoftAP) y asigna la IP estática.
 */
static void wifi_app_soft_ap_config(esp_netif_t *esp_netif_ap)
{
    // SoftAP - WiFi access point configuration
    wifi_config_t ap_config =
    {
        .ap = {
                .ssid = WIFI_AP_SSID,
                .ssid_len = strlen(WIFI_AP_SSID),
                .password = WIFI_AP_PASSWORD,
                .channel = WIFI_AP_CHANNEL,
                .ssid_hidden = WIFI_AP_SSID_HIDDEN,
                .authmode = WIFI_AUTH_WPA2_PSK,
                .max_connection = WIFI_AP_MAX_CONNECTIONS,
                .beacon_interval = WIFI_AP_BEACON_INTERVAL,
        },
    };

    /* Configurar DHCP para el AP */
    esp_netif_ip_info_t ap_ip_info;
    memset(&ap_ip_info, 0x00, sizeof(ap_ip_info));

    esp_netif_dhcps_stop(esp_netif_ap);             /* Debe llamarse primero */
    inet_pton(AF_INET, WIFI_AP_IP, &ap_ip_info.ip);   /// > Assign access point's static IP, GW, and netmask
    inet_pton(AF_INET, WIFI_AP_GATEWAY, &ap_ip_info.gw);
    inet_pton(AF_INET, WIFI_AP_NETMASK, &ap_ip_info.netmask);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(esp_netif_ap, &ap_ip_info));         /// > Configura estáticamente la interfaz de red
    ESP_ERROR_CHECK(esp_netif_dhcps_start(esp_netif_ap));          /// > Inicia el servidor DHCP del AP (para estaciones que se conecten, p.ej. tu dispositivo móvil)

    /* Establecer el modo antes de configurar */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));           /// > Setting the mode as Access Point / Station Mode
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config));          /// > Set our configuration
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_AP_BANDWIDTH));        /// > Our default bandwidth 20 MHz
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_STA_POWER_SAVE));         /* Establecer power save a "NINGUNO" */
}

/**
 * @brief Configura la estación WiFi (STA) con las credenciales por defecto (ejemplo).
 */
static void wifi_app_sta_config(void)
{
    // Station - WiFi station configuration
    wifi_config_t sta_config =
    {
        .sta = {
                .ssid = WIFI_STA_SSID,
                .password = WIFI_STA_PASSWORD,
                .bssid_set = false
        },
    };
    
    // Establecer la configuración STA
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_config));
}

/**
 * @brief Tarea principal de la aplicación WiFi.
 *
 * Ejecuta el bucle de la aplicación, procesa mensajes de la cola y gestiona
 * el ciclo de conexión/reintentos de la estación.
 * @param pvParameters Parámetro pasado a la tarea (no usado).
 */
void wifi_app_task(void *pvParameters)
{
    wifi_app_queue_message_t msg;
    esp_netif_t *esp_netif_sta = NULL;
    esp_netif_t *esp_netif_ap = NULL;
    int retry_count = 0;

    // 1. Inicializar el manejador de eventos
    wifi_app_event_handler_init();

    // 2. Inicializar la pila TCP/IP y la configuración WiFi
    wifi_app_default_wifi_init(&esp_netif_sta, &esp_netif_ap);
    
    // 3. Iniciar SNTP (solo se inicializa)
    sntp_time_sync_init();

    // 4. SoftAP config (Configura AP/STA en modo APSTA)
    wifi_app_soft_ap_config(esp_netif_ap);
    
    // 5. STA config (Solo establece la configuración de la red STA)
    wifi_app_sta_config(); 

    // 6. Iniciar WiFi (OBLIGATORIO: arranca el driver de WiFi, incluyendo el AP "Elpepe")
    ESP_LOGI(TAG, "Calling esp_wifi_start()");
    ESP_ERROR_CHECK(esp_wifi_start());

    // 7. Intentar la conexión STA (Ahora es seguro llamar a connect)
    wifi_app_connect_sta();
    
    // 8. Send first event message
    wifi_app_send_message(WIFI_APP_MSG_START_HTTP_SERVER);
    
    ESP_LOGI(TAG, "WIFI app task running...");

    for (;;)
    {
        if (xQueueReceive(wifi_app_queue_handle, &msg, portMAX_DELAY))
        {
            switch (msg.msgID)
            {
                case WIFI_APP_MSG_START_HTTP_SERVER:
                    ESP_LOGI(TAG, "WIFI_APP_MSG_START_HTTP_SERVER");

                    http_server_start();
                    break;

                case WIFI_APP_MSG_CONNECTING_FROM_HTTP_SERVER:
                    ESP_LOGI(TAG, "WIFI_APP_MSG_CONNECTING_FROM_HTTP_SERVER");
                    // Aquí puedes iniciar la conexión STA con las credenciales recibidas del HTTP Server
                    break;

                case WIFI_APP_MSG_STA_CONNECTED_GOT_IP:
                    ESP_LOGI(TAG, "WIFI_APP_MSG_STA_CONNECTED_GOT_IP");
                    retry_count = 0; /* resetear el contador de reintentos dentro de la tarea */
                    break;
                    
                case WIFI_APP_MSG_STA_DISCONNECTED:
                    ESP_LOGI(TAG, "WIFI_APP_MSG_STA_DISCONNECTED received; handling retry in task");
                    if (retry_count < MAX_CONNECTION_RETRIES) {
                        retry_count++;
                        esp_wifi_connect();
                        ESP_LOGI(TAG, "Retrying STA connection... (%d/%d)", retry_count, MAX_CONNECTION_RETRIES);
                    } else {
                        ESP_LOGI(TAG, "Maximum retries reached.");
                        /* Manejar desconexión permanente (podría notificar UI/log) */
                    }
                    break;

                case WIFI_APP_MSG_LOAD_SAVED_CREDENTIALS:
                {
                    ESP_LOGI(TAG, "WIFI_APP_MSG_LOAD_SAVED_CREDENTIALS received");
                    // Desconectar primero
                    esp_wifi_disconnect();
                    vTaskDelay(pdMS_TO_TICKS(500)); // Esperar a que se desconecte

                    // Cargar credenciales desde NVS
                    char loaded_ssid[MAX_SSID_LENGTH + 1] ={0};
                    char loaded_password[MAX_PASSWORD_LENGTH + 1] ={0};
                    if (nvs_config_load_wifi_credentials(loaded_ssid, loaded_password) == 0) {
                        // Actualizar la configuración WiFi STA
                        wifi_config_t wifi_config = {0};
                        strncpy((char *)wifi_config.sta.ssid, loaded_ssid, sizeof(wifi_config.sta.ssid));
                        strncpy((char *)wifi_config.sta.password, loaded_password, sizeof(wifi_config.sta.password));
                        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
                        wifi_config.sta.pmf_cfg.capable = true;
                        wifi_config.sta.pmf_cfg.required = false;

                        esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
                        if (err == ESP_OK) {
                            ESP_LOGI(TAG, "WiFi config updated with new credentials: SSID=%s", loaded_ssid);
                            // Reconectar con las nuevas credenciales
                            retry_count = 0;
                            esp_wifi_connect();
                        } else {
                            ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(err));
                        }
                    } else {
                        ESP_LOGW(TAG, "Failed to load WiFi credentials from NVS");
                    }
                    break;
                }
                default:
                    break;

            }
        }
    }
}

/**
 * @brief Intenta conectar la estación WiFi (STA).
 *
 * Realiza una llamada segura a `esp_wifi_connect()` para iniciar la conexión
 * con las credenciales configuradas.
 *
 * @param None
 * @return void
 */
void wifi_app_connect_sta(void)
{
    ESP_LOGI(TAG, "Connecting STA (safe call)...");
    ESP_ERROR_CHECK(esp_wifi_connect());
}

/**
 * @brief Envía un mensaje al hilo/cola de la aplicación WiFi.
 *
 * @param msgID Identificador del mensaje (`wifi_app_message_e`)
 * @return BaseType_t `pdTRUE` si el envío fue exitoso, `pdFALSE` en caso contrario
 */
BaseType_t wifi_app_send_message(wifi_app_message_e msgID)
{
    wifi_app_queue_message_t msg;
    msg.msgID = msgID;
    return xQueueSend(wifi_app_queue_handle, &msg, portMAX_DELAY);
}

/**
 * @brief Inicializa la parte de la aplicación WiFi usada por otros módulos.
 *
 * Configura el nivel de logs de WiFi y crea la cola de mensajes usada por
 * `wifi_app_task`. No crea la tarea principal; esa responsabilidad recae en `main`.
 *
 * @param None
 * @return void
 */
void wifi_app_start(void)
{
    ESP_LOGI(TAG, "STARTING WIFI APPLICATION");

    /* Deshabilitar mensajes de log de WiFi por defecto */
    esp_log_level_set("wifi", ESP_LOG_NONE);

    /* Crear cola de mensajes (la tarea debe crearse desde main) */
    wifi_app_queue_handle = xQueueCreate(3, sizeof(wifi_app_queue_message_t));
}

/* Implementación de funciones dummy (o que requieren lógica NVS) */
/**
 * @brief Devuelve la configuración WiFi actual (dummy en esta implementación).
 *
 * @return wifi_config_t* Puntero a la estructura de configuración o NULL si no disponible
 */
wifi_config_t* wifi_app_get_wifi_config(void) { return NULL; }

/**
 * @brief Registra un callback para notificaciones de conexión WiFi (dummy).
 *
 * @param cb Puntero a la función callback
 * @return void
 */
void wifi_app_set_callback(wifi_connected_event_callback_t cb) {}

/**
 * @brief Invoca el callback registrado para evento de conexión (dummy).
 *
 * @return void
 */
void wifi_app_call_callback(void) {}

/**
 * @brief Obtiene la intensidad de señal RSSI de la red conectada (dummy).
 *
 * @return int8_t Valor RSSI (dBm) o 0 si no disponible
 */
int8_t wifi_app_get_rssi(void) { return 0; }