#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "lwip/netdb.h"

#include "http_server.h"
#include "rgb_led.h"
#include "tasks_common.h"
#include "wifi_app.h"
#include "sntp_time_sync.h"

static const char TAG [] = "wifi_app";

static QueueHandle_t wifi_app_queue_handle;

// Variables para el modo Station (STA)
static int g_retry_count = 0;

esp_netif_t* esp_netif_sta = NULL;
esp_netif_t* esp_netif_ap = NULL;

/**
 * WiFi application event handler
 */
static void wifi_app_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "WIFI_EVENT_AP_START. AP 'Elpepe' Activo."); // Confirmación de AP
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
                ESP_LOGI(TAG, "WIFI_EVENT_STA_START. Intentando conectar a la red...");
                break;

            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED. Esperando IP...");
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
                {
                    wifi_event_sta_disconnected_t *disconnected = (wifi_event_sta_disconnected_t *)event_data;
                    ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED. Reason: %d", disconnected->reason);
                    
                    // Detener SNTP si estaba activo
                    sntp_time_sync_stop(); 

                    if (g_retry_count < MAX_CONNECTION_RETRIES)
                    {
                        g_retry_count++;
                        
                        // IMPORTANTE: esp_wifi_connect() es seguro aquí porque el driver ya inició.
                        esp_wifi_connect(); 
                        
                        ESP_LOGI(TAG, "Reintentando conexión STA... (%d/%d)", g_retry_count, MAX_CONNECTION_RETRIES);
                    }
                    else
                    {
                        wifi_app_send_message(WIFI_APP_MSG_STA_DISCONNECTED);
                    }
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
                    g_retry_count = 0; // Reiniciar contador de reintentos
                    
                    // Notificar a la aplicación que tenemos IP (para iniciar HTTP Server y SNTP)
                    wifi_app_send_message(WIFI_APP_MSG_STA_CONNECTED_GOT_IP);
                }
                break;
        }
    }
}

/**
 * Initializes the WiFi application event handler for WiFi and IP events.
 */
static void wifi_app_event_handler_init(void)
{
    // Event loop for the WiFi driver
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Event handler for the connection
    esp_event_handler_instance_t instance_wifi_event;
    esp_event_handler_instance_t instance_ip_event;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_app_event_handler, NULL, &instance_wifi_event));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &wifi_app_event_handler, NULL, &instance_ip_event));
}

/**
 * Initializes the TCP stack and default WiFi configuration.
 */
static void wifi_app_default_wifi_init(void)
{
    // Initialize the TCP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Default WiFi config - operations must be in this order!
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    
    // Crear interfaces de red para STA y AP
    esp_netif_sta = esp_netif_create_default_wifi_sta();
    esp_netif_ap = esp_netif_create_default_wifi_ap();
}

/**
 * Configures the WiFi access point settings and assigns the static IP to the SoftAP.
 */
static void wifi_app_soft_ap_config(void)
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

    // Configure DHCP for the AP
    esp_netif_ip_info_t ap_ip_info;
    memset(&ap_ip_info, 0x00, sizeof(ap_ip_info));

    esp_netif_dhcps_stop(esp_netif_ap); 		    ///> must call this first
    inet_pton(AF_INET, WIFI_AP_IP, &ap_ip_info.ip);  ///> Assign access point's static IP, GW, and netmask
    inet_pton(AF_INET, WIFI_AP_GATEWAY, &ap_ip_info.gw);
    inet_pton(AF_INET, WIFI_AP_NETMASK, &ap_ip_info.netmask);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(esp_netif_ap, &ap_ip_info)); 		   ///> Statically configure the network interface
    ESP_ERROR_CHECK(esp_netif_dhcps_start(esp_netif_ap)); 		   ///> Start the AP DHCP server (for connecting stations e.g. your mobile device)

    // Set the mode before config
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA)); 		   ///> Setting the mode as Access Point / Station Mode
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config)); 		   ///> Set our configuration
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_AP_BANDWIDTH)); 	   ///> Our default bandwidth 20 MHz
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_STA_POWER_SAVE)); 		   ///> Power save set to "NONE"
}

/**
 * Configura la estación WiFi (STA) con credenciales predeterminadas (para el ejemplo).
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
    
    // *** CORRECCIÓN: NO llamar a wifi_app_connect_sta() aquí. Se llama después de esp_wifi_start() en wifi_app_task.
    // wifi_app_connect_sta(); 
}

/**
 * Main task for the WiFi application
 * @param pvParameters parameter which can be passed to the task
 */
static void wifi_app_task(void *pvParameters)
{
    wifi_app_queue_message_t msg;

    // 1. Initialize the event handler
    wifi_app_event_handler_init();

    // 2. Initialize the TCP/IP stack and WiFi config
    wifi_app_default_wifi_init();
    
    // 3. Iniciar SNTP (solo se inicializa)
    sntp_time_sync_init();

    // 4. SoftAP config (Configura AP/STA en modo APSTA)
    wifi_app_soft_ap_config();
    
    // 5. STA config (Solo establece la configuración de la red STA)
    wifi_app_sta_config(); 

    // 6. Start WiFi (MANDATORIO: Arranca el driver de WiFi, incluyendo el AP "Elpepe")
    ESP_LOGI(TAG, "Llamando a esp_wifi_start()");
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
                    //rgb_led_http_server_started();

                    break;

                case WIFI_APP_MSG_CONNECTING_FROM_HTTP_SERVER:
                    ESP_LOGI(TAG, "WIFI_APP_MSG_CONNECTING_FROM_HTTP_SERVER");
                    // Aquí puedes iniciar la conexión STA con las credenciales recibidas del HTTP Server
                    break;

                case WIFI_APP_MSG_STA_CONNECTED_GOT_IP:
                    ESP_LOGI(TAG, "WIFI_APP_MSG_STA_CONNECTED_GOT_IP");
                    
                    //rgb_led_wifi_connected();
                    break;
                    
                case WIFI_APP_MSG_STA_DISCONNECTED:
                    ESP_LOGI(TAG, "WIFI_APP_MSG_STA_DISCONNECTED. Fallo maximo de reintentos.");
                    // Manejar desconexión permanente (ej: cambiar el led a un color de error)
                    break;

                default:
                    break;

            }
        }
    }
}

void wifi_app_connect_sta(void)
{
    g_retry_count = 0;
    ESP_LOGI(TAG, "Conectando STA (llamada segura)...");
    ESP_ERROR_CHECK(esp_wifi_connect());
}

BaseType_t wifi_app_send_message(wifi_app_message_e msgID)
{
    wifi_app_queue_message_t msg;
    msg.msgID = msgID;
    return xQueueSend(wifi_app_queue_handle, &msg, portMAX_DELAY);
}

void wifi_app_start(void)
{
    ESP_LOGI(TAG, "STARTING WIFI APPLICATION");

    // Start WiFi started LED
    //rgb_led_wifi_app_started();

    // Disable default WiFi logging messages
    esp_log_level_set("wifi", ESP_LOG_NONE);

    // Create message queue
    wifi_app_queue_handle = xQueueCreate(3, sizeof(wifi_app_queue_message_t));

    // Start the WiFi application task
    xTaskCreatePinnedToCore(&wifi_app_task, "wifi_app_task", WIFI_APP_TASK_STACK_SIZE, NULL, WIFI_APP_TASK_PRIORITY, NULL, WIFI_APP_TASK_CORE_ID);
}

// Implementación de funciones dummy (o que requieren lógica NVS) - dejadas así para el ejemplo
wifi_config_t* wifi_app_get_wifi_config(void) { return NULL; }
void wifi_app_set_callback(wifi_connected_event_callback_t cb) {}
void wifi_app_call_callback(void) {}
int8_t wifi_app_get_rssi(void) { return 0; }