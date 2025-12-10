/**
 * @file http_server.c
 * @brief Manejadores del servidor HTTP y tarea de monitor.
 *
 * Este módulo implementa los endpoints de la API usados por la interfaz
 * web embebida para leer el estado del sistema, cambiar la configuración
 * (manual/auto/programado) y realizar actualizaciones OTA. La gestión del
 * ciclo de vida del servidor y el manejo de resultados OTA se delegan a una
 * tarea local de monitor que posee la instancia `httpd_handle_t`.
 */

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "sys/param.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "cJSON.h"
#include <time.h>
#include <math.h>

#include "http_server.h"
#include "tasks_common.h"
#include "wifi_app.h"
#include "thermistor_reader.h"
#include "sntp_time_sync.h"
#include "queues.h"
#include "nvs_config.h"
#include "system_state.h"

/**
 * @brief Etiqueta para mensajes por consola ESP
 */
static const char TAG[] = "http_server";

/*
 * Estado de actualización de firmware (a nivel de archivo).
 * No existen variables globales mutables; se usan colas centralizadas
 * para comunicar el estado entre tareas y el monitor.
 *
 * Nota: las variables globales tipo `g_` fueron eliminadas. El estado y los
 * registros se acceden a través de la API de `system_state` / colas para
 * garantizar seguridad entre hilos y claridad arquitectónica.
 */

/* ===== Archivos de la interfaz web embebida ===== */
extern const uint8_t jquery_3_3_1_min_js_start[]    asm("_binary_jquery_3_3_1_min_js_start");
extern const uint8_t jquery_3_3_1_min_js_end[]      asm("_binary_jquery_3_3_1_min_js_end");
extern const uint8_t index_html_start[]             asm("_binary_index_html_start");
extern const uint8_t index_html_end[]               asm("_binary_index_html_end");
extern const uint8_t app_css_start[]                asm("_binary_app_css_start");
extern const uint8_t app_css_end[]                  asm("_binary_app_css_end");
extern const uint8_t app_js_start[]                 asm("_binary_app_js_start");
extern const uint8_t app_js_end[]                   asm("_binary_app_js_end");
extern const uint8_t favicon_ico_start[]            asm("_binary_favicon_ico_start");
extern const uint8_t favicon_ico_end[]              asm("_binary_favicon_ico_end");

/* Declaración adelantada */
static httpd_handle_t http_server_configure(void);

/*******************************************************
 * Manejador de tiempo SNTP
 ********************************************************/

/**
 * @brief Handler HTTP que devuelve la hora actual en formato JSON.
 *
 * Consulta el módulo SNTP para obtener la hora y devuelve un objeto JSON
 * con la hora formateada y un campo `status` indicando si está sincronizada.
 *
 * @param req Puntero a la petición HTTP recibida
 * @return esp_err_t ESP_OK si la respuesta se envía correctamente
 */
static esp_err_t http_server_get_time_json_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/time.json requested");

    struct tm timeinfo = sntp_time_sync_get_time();
    char strftime_buf[64];
    char timeJSON[128];

    if (timeinfo.tm_year < (2016 - 1900)) {
        sprintf(timeJSON, "{\"time\":\"N/A\",\"status\":\"unsynced\"}");
    } else {
        strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
        sprintf(timeJSON, "{\"time\":\"%s\",\"status\":\"synced\"}", strftime_buf);
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, timeJSON, strlen(timeJSON));
    return ESP_OK;
}
/*******************************************************
 * Manejadores JSON del sistema
 ********************************************************/
/**
 * @brief Handler HTTP que devuelve el estado del sistema en JSON.
 *
 * Lee el último `system_state` desde la cola de estado y construye un
 * JSON con temperatura, estado PIR, modo, PWM y límites automáticos.
 *
 * @param req Puntero a la petición HTTP recibida
 * @return esp_err_t ESP_OK si la respuesta se envía correctamente
 */
static esp_err_t http_server_get_system_state_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/systemState requested");
    /* Obtener el estado más reciente de la cola (sin bloqueo) */
    system_state_t local_state = {0};
    QueueHandle_t state_queue = queues_get_system_state_queue();
    if (state_queue != NULL) {
        xQueuePeek(state_queue, &local_state, 0);
    }

    char stateJSON[512];
    snprintf(stateJSON, sizeof(stateJSON),
        "{\"temperature\":%.1f,\"pir\":%d,\"mode\":%d,\"pwm\":%d,"
        "\"tMin\":%.1f,\"tMax\":%.1f,\"activeRegister\":%d}",
        local_state.temperature,
        local_state.pir_state,
        local_state.current_mode,
        local_state.current_pwm,
        local_state.auto_t_min,
        local_state.auto_t_max,
        local_state.active_register
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, stateJSON, strlen(stateJSON));
    return ESP_OK;
}

/**
 * @brief Handler HTTP que establece el modo de operación del sistema.
 *
 * Espera un JSON con el campo `mode` y envía una estructura `config_update_t`
 * a la cola de configuración para que otras tareas apliquen el cambio.
 *
 * @param req Puntero a la petición HTTP recibida
 * @return esp_err_t ESP_OK si el request fue procesado
 */
static esp_err_t http_server_set_mode_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/setMode requested");

    char buf[100];
    int len = httpd_req_recv(req, buf, sizeof(buf));
    if (len <= 0) return ESP_FAIL;

    cJSON* root = cJSON_ParseWithLength(buf, len);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON* mode = cJSON_GetObjectItem(root, "mode");
    if (mode && mode->type == cJSON_Number) {
        config_update_t update = {0};
        update.mode = mode->valueint;
        update.manual_pwm = -1;
        update.auto_tmin = NAN;
        update.auto_tmax = NAN;
        update.update_registers = 0;

        QueueHandle_t config_queue = queues_get_config_update_queue();
        if (config_queue != NULL) {
            xQueueSend(config_queue, &update, portMAX_DELAY);
            ESP_LOGI(TAG, "Mode update sent to queue: %d", update.mode);
        }
    }

    cJSON_Delete(root);
    httpd_resp_send(req, "{\"status\":\"ok\"}", 15);
    return ESP_OK;
}

/**
 * @brief Handler HTTP para guardar configuración manual (PWM).
 *
 * Recibe JSON con `pwm` y envía un `config_update_t` con `manual_pwm`
 * a la cola de configuración.
 *
 * @param req Puntero a la petición HTTP recibida
 * @return esp_err_t ESP_OK si el request fue procesado
 */
static esp_err_t http_server_save_manual_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/saveManual requested");

    char buf[100];
    int len = httpd_req_recv(req, buf, sizeof(buf));
    if (len <= 0) return ESP_FAIL;

    cJSON* root = cJSON_ParseWithLength(buf, len);
    if (!root) return ESP_FAIL;

    cJSON* pwm = cJSON_GetObjectItem(root, "pwm");
    if (pwm && pwm->type == cJSON_Number) {
        config_update_t update = {0};
        update.mode = -1;
        update.manual_pwm = pwm->valueint;
        update.auto_tmin = NAN;
        update.auto_tmax = NAN;
        update.update_registers = 0;

        QueueHandle_t config_queue = queues_get_config_update_queue();
        if (config_queue != NULL) {
            xQueueSend(config_queue, &update, portMAX_DELAY);
            ESP_LOGI(TAG, "Manual PWM update sent to queue: %d", update.manual_pwm);
        }
    }

    cJSON_Delete(root);
    httpd_resp_send(req, "{\"status\":\"ok\"}", 15);
    return ESP_OK;
}


/**
 * @brief Handler HTTP para guardar configuración automática (rangos de temperatura).
 *
 * Recibe JSON con `tMin` y `tMax` y envía un `config_update_t` a la cola
 * de configuración con los límites automáticos.
 *
 * @param req Puntero a la petición HTTP recibida
 * @return esp_err_t ESP_OK si el request fue procesado
 */
static esp_err_t http_server_save_auto_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/saveAuto requested");

    char buf[200];
    int len = httpd_req_recv(req, buf, sizeof(buf));
    if (len <= 0) return ESP_FAIL;

    cJSON* root = cJSON_ParseWithLength(buf, len);
    if (!root) return ESP_FAIL;

    cJSON* tMin = cJSON_GetObjectItem(root, "tMin");
    cJSON* tMax = cJSON_GetObjectItem(root, "tMax");

    if (tMin && tMax) {
        config_update_t update = {0};
        update.mode = -1;
        update.manual_pwm = -1;
        update.auto_tmin = (float)tMin->valuedouble;
        update.auto_tmax = (float)tMax->valuedouble;
        update.update_registers = 0;

        QueueHandle_t config_queue = queues_get_config_update_queue();
        if (config_queue != NULL) {
            xQueueSend(config_queue, &update, portMAX_DELAY);
            ESP_LOGI(TAG, "Auto temps update sent to queue: %.1f-%.1f", 
                     update.auto_tmin, update.auto_tmax);
        }
    }

    cJSON_Delete(root);
    httpd_resp_send(req, "{\"status\":\"ok\"}", 15);
    return ESP_OK;
}

/**
 * @brief Handler HTTP para guardar registros programados (schedules).
 *
 * Procesa un arreglo `registers` en el JSON de la petición y lo transforma
 * en una estructura `config_update_t` que se envía a la cola de configuración.
 *
 * @param req Puntero a la petición HTTP recibida
 * @return esp_err_t ESP_OK si el request fue procesado
 */
static esp_err_t http_server_save_programmed_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/saveProgrammed requested");

    char buf[512]={0};
    int len = httpd_req_recv(req, buf, sizeof(buf));
    if (len <= 0) return ESP_FAIL;

    cJSON* root = cJSON_ParseWithLength(buf, len);
    if (!root) return ESP_FAIL;

    cJSON* registers = cJSON_GetObjectItem(root, "registers");
    if (!registers || registers->type != cJSON_Array) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    /* Analizar registros desde JSON */
    config_update_t update = {0};
    update.mode = -1;
    update.manual_pwm = -1;
    update.auto_tmin = NAN;
    update.auto_tmax = NAN;
    update.update_registers = 1;

    int i = 0;
    cJSON* reg;
    cJSON_ArrayForEach(reg, registers)
    {
        if (i >= 3) break;

        update.registers[i].active = cJSON_GetObjectItem(reg, "active")->valueint;
        sscanf(cJSON_GetObjectItem(reg, "startTime")->valuestring, "%hhu:%hhu",
               &update.registers[i].start_hour, &update.registers[i].start_min);
        sscanf(cJSON_GetObjectItem(reg, "endTime")->valuestring, "%hhu:%hhu",
               &update.registers[i].end_hour, &update.registers[i].end_min);

        update.registers[i].temp_min = cJSON_GetObjectItem(reg, "tempMin")->valuedouble;
        update.registers[i].temp_max = cJSON_GetObjectItem(reg, "tempMax")->valuedouble;

        i++;
    }

    // Enviar actualización a la cola
    QueueHandle_t config_queue = queues_get_config_update_queue();
    if (config_queue != NULL) {
        xQueueSend(config_queue, &update, portMAX_DELAY);
        ESP_LOGI(TAG, "Programmed registers update sent to queue");
    }

    cJSON_Delete(root);
    httpd_resp_send(req, "{\"status\":\"ok\"}", 15);
    return ESP_OK;
}

/**
 * @brief Handler HTTP que devuelve los registros programados en JSON.
 *
 * Lee los registros programados desde `system_state` y devuelve un arreglo
 * JSON con la información de cada registro (horarios, límites y si está activo).
 *
 * @param req Puntero a la petición HTTP recibida
 * @return esp_err_t ESP_OK si la respuesta se envía correctamente
 */
static esp_err_t http_server_get_programmed_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/getProgrammed requested");

    char buf[512]={0};
    int offset = 0;

    scheduled_register_t regs[3] ={0};
    if (system_state_get_registers(regs) != 0) {
        // Si no podemos obtener los registros, devolver un arreglo vacío
        offset += snprintf(buf + offset, sizeof(buf) - offset, "{\"registers\":[]}");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, buf, offset);
        return ESP_OK;
    }

    offset += snprintf(buf + offset, sizeof(buf) - offset, "{\"registers\":[");

    for (int i = 0; i < 3; i++) {
        offset += snprintf(buf + offset, sizeof(buf) - offset,
            "{\"index\":%d,\"active\":%d,\"startTime\":\"%02d:%02d\",\"endTime\":\"%02d:%02d\",\"tempMin\":%.1f,\"tempMax\":%.1f}%s",
            i,
            regs[i].active,
            regs[i].start_hour, regs[i].start_min,
            regs[i].end_hour, regs[i].end_min,
            regs[i].temp_min, regs[i].temp_max,
            (i < 2) ? "," : ""
        );
    }

    offset += snprintf(buf + offset, sizeof(buf) - offset, "]}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, offset);
    return ESP_OK;
}

/*******************************************************
 * Manejador de configuración WiFi
 ********************************************************/

/**
 * @brief Manejador HTTP para actualizar credenciales WiFi STA
 *
 * Recibe SSID y contraseña en JSON, las guarda en NVS y envía un mensaje
 * a la cola WiFi para reconectar con las nuevas credenciales.
 *
 * @param req Solicitud HTTP del servidor
 * @return ESP_OK si se procesó correctamente
 */
static esp_err_t http_server_save_wifi_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/saveWiFi requested");

    char buf[256];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"No data received\"}", 52);
        return ESP_FAIL;
    }

    buf[len] = '\0';
    cJSON* root = cJSON_ParseWithLength(buf, len);
    if (!root) {
        httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"Invalid JSON\"}", 47);
        return ESP_FAIL;
    }

    cJSON* ssid = cJSON_GetObjectItem(root, "ssid");
    cJSON* password = cJSON_GetObjectItem(root, "password");

    if (!ssid || !password || ssid->type != cJSON_String || password->type != cJSON_String) {
        cJSON_Delete(root);
        httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"Missing SSID or password\"}", 60);
        return ESP_FAIL;
    }

    // Validar longitudes
    if (strlen(ssid->valuestring) > MAX_SSID_LENGTH || strlen(password->valuestring) > MAX_PASSWORD_LENGTH) {
        cJSON_Delete(root);
        httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"SSID or password too long\"}", 63);
        return ESP_FAIL;
    }

    // Guardar en NVS
    int result = nvs_config_save_wifi_credentials(ssid->valuestring, password->valuestring);
    if (result != 0) {
        cJSON_Delete(root);
        httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"Failed to save credentials\"}", 63);
        return ESP_FAIL;
    }

    // Enviar mensaje a la cola WiFi para aplicar los nuevos cambios
    BaseType_t queue_result = wifi_app_send_message(WIFI_APP_MSG_LOAD_SAVED_CREDENTIALS);

    if (queue_result == pdTRUE) {
        ESP_LOGI(TAG, "WiFi credentials updated: SSID=%s", ssid->valuestring);
        httpd_resp_send(req, "{\"status\":\"ok\",\"message\":\"WiFi credentials saved\"}", 59);
    } else {
        httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"Failed to send WiFi update message\"}", 71);
    }

    cJSON_Delete(root);
    return ESP_OK;
}

/*******************************************************
 * Manejadores de archivos estáticos (HTML/JS/CSS)
 ********************************************************/

 /**
 * @brief Handler HTTP para servir la página index (HTML embebido).
 *
 * Administra la respuesta para la ruta `/` sirviendo el recurso HTML
 * embebido en la sección de binarios del firmware.
 *
 * @param req Puntero a la petición HTTP recibida
 * @return esp_err_t ESP_OK si la respuesta se envía correctamente
 */
static esp_err_t http_server_index_html_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char*)index_html_start,
                    index_html_end - index_html_start);
    return ESP_OK;
}

/**
 * @brief Handler HTTP para servir `jquery-3.3.1.min.js` embebido.
 *
 * @param req Puntero a la petición HTTP recibida
 * @return esp_err_t ESP_OK si la respuesta se envía correctamente
 */
static esp_err_t http_server_jquery_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/javascript");
    httpd_resp_send(req, (const char*)jquery_3_3_1_min_js_start,
                    jquery_3_3_1_min_js_end - jquery_3_3_1_min_js_start);
    return ESP_OK;
}

/**
 * @brief Handler HTTP para servir `app.css` embebido.
 *
 * @param req Puntero a la petición HTTP recibida
 * @return esp_err_t ESP_OK si la respuesta se envía correctamente
 */
static esp_err_t http_server_app_css_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/css");
    httpd_resp_send(req, (const char*)app_css_start,
                    app_css_end - app_css_start);
    return ESP_OK;
}

/**
 * @brief Handler HTTP para servir `app.js` embebido.
 *
 * @param req Puntero a la petición HTTP recibida
 * @return esp_err_t ESP_OK si la respuesta se envía correctamente
 */
static esp_err_t http_server_app_js_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/javascript");
    httpd_resp_send(req, (const char*)app_js_start,
                    app_js_end - app_js_start);
    return ESP_OK;
}

/**
 * @brief Handler HTTP para servir el `favicon.ico` embebido.
 *
 * @param req Puntero a la petición HTTP recibida
 * @return esp_err_t ESP_OK si la respuesta se envía correctamente
 */
static esp_err_t http_server_favicon_ico_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, (const char*)favicon_ico_start,
                    favicon_ico_end - favicon_ico_start);
    return ESP_OK;
}

/*******************************************************
 * Manejador de actualización OTA
 ********************************************************/

/**
 * @brief Manejador HTTP para actualización del firmware OTA
 *
 * Recibe el binario del firmware desde la solicitud HTTP, lo escribe en la
 * partición de actualización y envía un mensaje a la tarea monitor indicando
 * éxito o fallo.
 *
 * @param req Solicitud HTTP del servidor
 * @return ESP_OK siempre (el estado real se comunica vía cola de monitor)
 */
esp_err_t http_server_OTA_update_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "OTA update requested");

    esp_ota_handle_t ota_handle;
    char ota_buff[1024];
    int content_length = req->content_len;
    int content_received = 0;
    int recv_len;
    bool is_req_body_started = false;
    bool flash_successful = false;

    const esp_partition_t* update_partition =
        esp_ota_get_next_update_partition(NULL);

    do {
        recv_len = httpd_req_recv(req, ota_buff,
                                  MIN(content_length, sizeof(ota_buff)));

        if (recv_len <= 0) break;
        content_received += recv_len;

        if (!is_req_body_started)
        {
            is_req_body_started = true;
            char *body = strstr(ota_buff, "\r\n\r\n") + 4;
            int body_len = recv_len - (body - ota_buff);

            esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
            esp_ota_write(ota_handle, body, body_len);
        }
        else
        {
            esp_ota_write(ota_handle, ota_buff, recv_len);
        }

    } while (content_received < content_length);

    if (esp_ota_end(ota_handle) == ESP_OK &&
        esp_ota_set_boot_partition(update_partition) == ESP_OK)
    {
        flash_successful = true;
    }

    http_server_monitor_send_message(
        flash_successful ? HTTP_MSG_OTA_UPDATE_SUCCESSFUL
                         : HTTP_MSG_OTA_UPDATE_FAILED
    );

    return ESP_OK;
}

/**
 * @brief Manejador HTTP para consultar el estado de la actualización OTA
 *
 * Devuelve en JSON el estado actual de la actualización OTA, la hora de
 * compilación y la fecha de compilación del firmware.
 *
 * @param req Solicitud HTTP del servidor
 * @return ESP_OK si se envió la respuesta correctamente
 */
esp_err_t http_server_OTA_status_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "OTAstatus requested");

    char json[100];
    /* Leer el estado OTA de la cola centralizada (si está disponible) */
    int fw_update_status = OTA_UPDATE_PENDING;
    QueueHandle_t status_q = queues_get_http_status_queue();
    if (status_q != NULL) {
        xQueuePeek(status_q, &fw_update_status, 0);
    }

    sprintf(json,
        "{\"ota_update_status\":%d,\"compile_time\":\"%s\",\"compile_date\":\"%s\"}",
        fw_update_status, __TIME__, __DATE__);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

/*******************************************************
 * Tarea monitor del servidor HTTP
 ********************************************************/

/**
 * @brief Tarea monitor que gestiona el ciclo de vida del servidor HTTP
 *
 * Esta tarea crea, inicia y detiene el servidor HTTP según los mensajes
 * recibidos en la cola del monitor. Mantiene estado local sobre la instancia
 * del servidor y el estado de actualización OTA.
 *
 * @param parameter No utilizado
 */
void http_server_monitor(void *parameter)
{
    /* Manejadores y estado locales de la tarea monitor */
    http_server_queue_message_t msg;
    QueueHandle_t monitor_q = queues_get_http_monitor_queue();
    QueueHandle_t status_q = queues_get_http_status_queue();
    httpd_handle_t http_server_handle = NULL;
    int fw_update_status_local = OTA_UPDATE_PENDING;

    // Inicializar la cola de estado con el valor por defecto
    if (status_q != NULL) {
        xQueueOverwrite(status_q, &fw_update_status_local);
    }

    for (;;)
    {
        if (monitor_q != NULL && xQueueReceive(monitor_q, &msg, portMAX_DELAY))
        {
            switch (msg.msgID)
            {
                case HTTP_MSG_START_SERVER:
                case HTTP_MSG_WIFI_CONNECT_INIT:
                case HTTP_MSG_WIFI_CONNECT_SUCCESS:
                    if (http_server_handle == NULL) {
                        http_server_handle = http_server_configure();
                        ESP_LOGI(TAG, "HTTP server started by monitor");
                    }
                    break;

                case HTTP_MSG_STOP_SERVER:
                    if (http_server_handle) {
                        httpd_stop(http_server_handle);
                        http_server_handle = NULL;
                        ESP_LOGI(TAG, "HTTP server stopped by monitor");
                    }
                    break;

                case HTTP_MSG_OTA_UPDATE_SUCCESSFUL:
                    fw_update_status_local = OTA_UPDATE_SUCCESSFUL;
                    if (status_q) xQueueOverwrite(status_q, &fw_update_status_local);
                    esp_restart();
                    break;

                case HTTP_MSG_OTA_UPDATE_FAILED:
                    fw_update_status_local = OTA_UPDATE_FAILED;
                    if (status_q) xQueueOverwrite(status_q, &fw_update_status_local);
                    break;

                default:
                    break;
            }
        }
    }
}

/*******************************************************
 * SERVER CONFIGURATION
 ********************************************************/

static httpd_handle_t http_server_configure(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 20;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;
    httpd_handle_t handle = NULL;
    if (httpd_start(&handle, &config) == ESP_OK)
    {
        ESP_LOGI(TAG, "Registering HTTP handlers...");

        httpd_register_uri_handler(handle, &(httpd_uri_t){"/", HTTP_GET, http_server_index_html_handler, NULL});
        httpd_register_uri_handler(handle, &(httpd_uri_t){"/jquery-3.3.1.min.js", HTTP_GET, http_server_jquery_handler, NULL});
        httpd_register_uri_handler(handle, &(httpd_uri_t){"/app.css", HTTP_GET, http_server_app_css_handler, NULL});
        httpd_register_uri_handler(handle, &(httpd_uri_t){"/app.js", HTTP_GET, http_server_app_js_handler, NULL});
        httpd_register_uri_handler(handle, &(httpd_uri_t){"/favicon.ico", HTTP_GET, http_server_favicon_ico_handler, NULL});

        // JSON endpoints
        httpd_register_uri_handler(handle, &(httpd_uri_t){"/time.json", HTTP_GET, http_server_get_time_json_handler, NULL});
        httpd_register_uri_handler(handle, &(httpd_uri_t){"/systemState", HTTP_GET, http_server_get_system_state_handler, NULL});
        httpd_register_uri_handler(handle, &(httpd_uri_t){"/setMode", HTTP_POST, http_server_set_mode_handler, NULL});
        httpd_register_uri_handler(handle, &(httpd_uri_t){"/saveManual", HTTP_POST, http_server_save_manual_handler, NULL});
        httpd_register_uri_handler(handle, &(httpd_uri_t){"/saveAuto", HTTP_POST, http_server_save_auto_handler, NULL});
        httpd_register_uri_handler(handle, &(httpd_uri_t){"/saveProgrammed", HTTP_POST, http_server_save_programmed_handler, NULL});
        httpd_register_uri_handler(handle, &(httpd_uri_t){"/getProgrammed", HTTP_GET, http_server_get_programmed_handler, NULL});
        httpd_register_uri_handler(handle, &(httpd_uri_t){"/saveWiFi", HTTP_POST, http_server_save_wifi_handler, NULL});

        // OTA routes
        httpd_register_uri_handler(handle, &(httpd_uri_t){"/OTAupdate", HTTP_POST, http_server_OTA_update_handler, NULL});
        httpd_register_uri_handler(handle, &(httpd_uri_t){"/OTAstatus", HTTP_POST, http_server_OTA_status_handler, NULL});

        return handle;
    }

    return NULL;
}

/*******************************************************
 * LOAD CONFIGURATIONS FROM FLASH (NVS)
 ********************************************************/

/*******************************************************
 * START / STOP HTTP SERVER
 ********************************************************/

/**
 * @brief Inicia el servidor HTTP
 *
 * Envía un mensaje a la tarea monitor solicitando que inicie el servidor HTTP
 * si aún no está en ejecución.
 */
void http_server_start(void)
{
    /* Solicitar a la tarea monitor que inicie el servidor HTTP */
    QueueHandle_t monitor_q = queues_get_http_monitor_queue();
    if (monitor_q != NULL) {
        http_server_queue_message_t msg = { .msgID = HTTP_MSG_START_SERVER };
        xQueueSend(monitor_q, &msg, portMAX_DELAY);
    }
}

/**
 * @brief Inicializa la cola del monitor del servidor HTTP
 *
 * Esta función se conserva para mantener compatibilidad de API.
 * La cola del monitor se crea centralmente en `queues_init`.
 */
void http_server_init_monitor_queue(void)
{
    /* La cola de monitor se crea centralmente en `queues_init`; nada que hacer aquí. */
}

/**
 * @brief Detiene el servidor HTTP
 *
 * Envía un mensaje a la tarea monitor solicitando que detenga el servidor HTTP.
 */
void http_server_stop(void)
{
    /* Solicitar al monitor que detenga el servidor */
    QueueHandle_t monitor_q = queues_get_http_monitor_queue();
    if (monitor_q != NULL) {
        http_server_queue_message_t msg = { .msgID = HTTP_MSG_STOP_SERVER };
        xQueueSend(monitor_q, &msg, portMAX_DELAY);
    }
}

/**
 * @brief Envía un mensaje a la tarea monitor del servidor HTTP
 *
 * Encapsula un mensaje de tipo `http_server_message_e` en una estructura
 * y lo envía a la cola del monitor.
 *
 * @param msgID Identificador del mensaje (HTTP_MSG_*)
 * @return pdTRUE si el mensaje se envió correctamente; pdFALSE en caso de error
 */
BaseType_t http_server_monitor_send_message(http_server_message_e msgID)
{
    QueueHandle_t monitor_q = queues_get_http_monitor_queue();
    if (monitor_q == NULL) return pdFALSE;
    http_server_queue_message_t msg = { msgID };
    return xQueueSend(monitor_q, &msg, portMAX_DELAY);
}

/**
 * @brief Función callback para reiniciar el sistema después de actualización OTA
 *
 * Se ejecuta cuando la actualización del firmware se ha completado exitosamente.
 * Reinicia el ESP32 para que el nuevo firmware comience a ejecutarse.
 *
 * @param arg No utilizado
 */
void http_server_fw_update_reset_callback(void *arg)
{
    esp_restart();
}

/**
 * @brief Establece el manejador de la tarea monitor del servidor HTTP
 *
 * Esta función se conserva para mantener compatibilidad de API. El monitor
 * no almacena un manejador de tarea global; el parámetro se ignora.
 *
 * @param handle Manejador de tarea del monitor (no utilizado)
 */
void http_server_set_monitor_task_handle(TaskHandle_t handle)
{
    (void)handle; /* el manejador de tarea del monitor no se almacena; el monitor usa estado local */
}