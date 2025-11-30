#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "sys/param.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "cJSON.h"

#include "http_server.h"
#include "tasks_common.h"
#include "wifi_app.h"
#include "rgb_led.h"
#include "thermistor_reader.h"
#include "sntp_time_sync.h"

// Tag used for ESP serial console messages
static const char TAG[] = "http_server";

// Firmware update status
static int g_fw_update_status = OTA_UPDATE_PENDING;

// HTTP server task handle
static httpd_handle_t http_server_handle = NULL;

// HTTP server monitor task handle
static TaskHandle_t task_http_server_monitor = NULL;

// Queue handle used to manipulate the main queue of events
static QueueHandle_t http_server_monitor_queue_handle;

// LED States
static uint8_t red_state = 0;
static uint8_t green_state = 0;
static uint8_t blue_state = 0;

// ===== Global variables declared in http_server.h =====
int g_current_mode = 0;
int g_current_pwm = 0;
float g_auto_t_min = 20.0f;
float g_auto_t_max = 30.0f;
int g_pir_state = 0;

scheduled_register_t g_registers[3] = {0};

// ===== Embedded Frontend Files =====
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

/*******************************************************
 *              SNTP TIME HANDLER
 ********************************************************/

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
 *              NEW SYSTEM JSON HANDLERS
 ********************************************************/

static esp_err_t http_server_get_system_state_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/systemState requested");

    float temperature = -99.9f;
    QueueHandle_t temp_queue = get_temperature_queue_handle();

    if (temp_queue != NULL) {
        xQueuePeek(temp_queue, &temperature, 0);
    }

    if (g_pir_state > 0)
        g_pir_state--;

    char stateJSON[512];
    snprintf(stateJSON, sizeof(stateJSON),
        "{\"temperature\":%.1f,\"pir\":%d,\"mode\":%d,\"pwm\":%d,"
        "\"tMin\":%.1f,\"tMax\":%.1f}",
        temperature,
        g_pir_state > 0 ? 1 : 0,
        g_current_mode,
        g_current_pwm,
        g_auto_t_min,
        g_auto_t_max
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, stateJSON, strlen(stateJSON));
    return ESP_OK;
}

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
        g_current_mode = mode->valueint;
        ESP_LOGI(TAG, "Mode set to %d", g_current_mode);
    }

    cJSON_Delete(root);
    httpd_resp_send(req, "{\"status\":\"ok\"}", 15);
    return ESP_OK;
}

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
        g_current_pwm = pwm->valueint;
        ESP_LOGI(TAG, "Manual PWM set: %d", g_current_pwm);
    }

    cJSON_Delete(root);
    httpd_resp_send(req, "{\"status\":\"ok\"}", 15);
    return ESP_OK;
}

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
        g_auto_t_min = tMin->valuedouble;
        g_auto_t_max = tMax->valuedouble;
        ESP_LOGI(TAG, "Auto config: %.1f to %.1f", g_auto_t_min, g_auto_t_max);
    }

    cJSON_Delete(root);
    httpd_resp_send(req, "{\"status\":\"ok\"}", 15);
    return ESP_OK;
}

static esp_err_t http_server_save_programmed_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/saveProgrammed requested");

    char buf[1024];
    int len = httpd_req_recv(req, buf, sizeof(buf));
    if (len <= 0) return ESP_FAIL;

    cJSON* root = cJSON_ParseWithLength(buf, len);
    if (!root) return ESP_FAIL;

    cJSON* registers = cJSON_GetObjectItem(root, "registers");
    if (!registers || registers->type != cJSON_Array) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    int i = 0;
    cJSON* reg;
    cJSON_ArrayForEach(reg, registers)
    {
        if (i >= 3) break;

        g_registers[i].active = cJSON_GetObjectItem(reg, "active")->valueint;
        sscanf(cJSON_GetObjectItem(reg, "startTime")->valuestring, "%hhu:%hhu",
               &g_registers[i].start_hour, &g_registers[i].start_min);
        sscanf(cJSON_GetObjectItem(reg, "endTime")->valuestring, "%hhu:%hhu",
               &g_registers[i].end_hour, &g_registers[i].end_min);

        g_registers[i].temp_min = cJSON_GetObjectItem(reg, "tempMin")->valuedouble;
        g_registers[i].temp_max = cJSON_GetObjectItem(reg, "tempMax")->valuedouble;

        i++;
    }

    cJSON_Delete(root);
    httpd_resp_send(req, "{\"status\":\"ok\"}", 15);
    return ESP_OK;
}
/*******************************************************
 *          STATIC FILE HANDLERS (HTML/JS/CSS)
 ********************************************************/

static esp_err_t http_server_index_html_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char*)index_html_start,
                           index_html_end - index_html_start);
    return ESP_OK;
}

static esp_err_t http_server_jquery_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/javascript");
    httpd_resp_send(req, (const char*)jquery_3_3_1_min_js_start,
                           jquery_3_3_1_min_js_end - jquery_3_3_1_min_js_start);
    return ESP_OK;
}

static esp_err_t http_server_app_css_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/css");
    httpd_resp_send(req, (const char*)app_css_start,
                           app_css_end - app_css_start);
    return ESP_OK;
}

static esp_err_t http_server_app_js_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/javascript");
    httpd_resp_send(req, (const char*)app_js_start,
                           app_js_end - app_js_start);
    return ESP_OK;
}

static esp_err_t http_server_favicon_ico_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, (const char*)favicon_ico_start,
                           favicon_ico_end - favicon_ico_start);
    return ESP_OK;
}

/*******************************************************
 *                  OTA UPDATE HANDLER
 ********************************************************/

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

esp_err_t http_server_OTA_status_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "OTAstatus requested");

    char json[100];
    sprintf(json,
        "{\"ota_update_status\":%d,\"compile_time\":\"%s\",\"compile_date\":\"%s\"}",
        g_fw_update_status, __TIME__, __DATE__);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}
/*******************************************************
 *                   LED TOGGLE HANDLERS
 ********************************************************/

static esp_err_t http_server_toggle_red_led_handler(httpd_req_t *req)
{
    red_state = !red_state;
    rgb_led_set_color(red_state * 255, green_state * 255, blue_state * 255);
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t http_server_toggle_green_led_handler(httpd_req_t *req)
{
    green_state = !green_state;
    rgb_led_set_color(red_state * 255, green_state * 255, blue_state * 255);
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t http_server_toggle_blue_led_handler(httpd_req_t *req)
{
    blue_state = !blue_state;
    rgb_led_set_color(red_state * 255, green_state * 255, blue_state * 255);
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/*******************************************************
 *                 HTTP SERVER MONITOR TASK
 ********************************************************/

static void http_server_monitor(void *parameter)
{
    http_server_queue_message_t msg;

    for (;;)
    {
        if (xQueueReceive(http_server_monitor_queue_handle, &msg, portMAX_DELAY))
        {
            switch (msg.msgID)
            {
                case HTTP_MSG_OTA_UPDATE_SUCCESSFUL:
                    g_fw_update_status = OTA_UPDATE_SUCCESSFUL;
                    esp_restart();
                    break;

                case HTTP_MSG_OTA_UPDATE_FAILED:
                    g_fw_update_status = OTA_UPDATE_FAILED;
                    break;

                default:
                    break;
            }
        }
    }
}

/*******************************************************
 *              SERVER CONFIGURATION
 ********************************************************/

static httpd_handle_t http_server_configure(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 20;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;

    if (httpd_start(&http_server_handle, &config) == ESP_OK)
    {
        ESP_LOGI(TAG, "Registering HTTP handlers...");

        httpd_register_uri_handler(http_server_handle, &(httpd_uri_t){"/", HTTP_GET, http_server_index_html_handler, NULL});
        httpd_register_uri_handler(http_server_handle, &(httpd_uri_t){"/jquery-3.3.1.min.js", HTTP_GET, http_server_jquery_handler, NULL});
        httpd_register_uri_handler(http_server_handle, &(httpd_uri_t){"/app.css", HTTP_GET, http_server_app_css_handler, NULL});
        httpd_register_uri_handler(http_server_handle, &(httpd_uri_t){"/app.js", HTTP_GET, http_server_app_js_handler, NULL});
        httpd_register_uri_handler(http_server_handle, &(httpd_uri_t){"/favicon.ico", HTTP_GET, http_server_favicon_ico_handler, NULL});

        // JSON endpoints
        httpd_register_uri_handler(http_server_handle, &(httpd_uri_t){"/time.json", HTTP_GET, http_server_get_time_json_handler, NULL});
        httpd_register_uri_handler(http_server_handle, &(httpd_uri_t){"/systemState", HTTP_GET, http_server_get_system_state_handler, NULL});
        httpd_register_uri_handler(http_server_handle, &(httpd_uri_t){"/setMode", HTTP_POST, http_server_set_mode_handler, NULL});
        httpd_register_uri_handler(http_server_handle, &(httpd_uri_t){"/saveManual", HTTP_POST, http_server_save_manual_handler, NULL});
        httpd_register_uri_handler(http_server_handle, &(httpd_uri_t){"/saveAuto", HTTP_POST, http_server_save_auto_handler, NULL});
        httpd_register_uri_handler(http_server_handle, &(httpd_uri_t){"/saveProgrammed", HTTP_POST, http_server_save_programmed_handler, NULL});

        // OTA routes
        httpd_register_uri_handler(http_server_handle, &(httpd_uri_t){"/OTAupdate", HTTP_POST, http_server_OTA_update_handler, NULL});
        httpd_register_uri_handler(http_server_handle, &(httpd_uri_t){"/OTAstatus", HTTP_POST, http_server_OTA_status_handler, NULL});

        // LED toggles
        httpd_register_uri_handler(http_server_handle, &(httpd_uri_t){"/toggle_red.json", HTTP_POST, http_server_toggle_red_led_handler, NULL});
        httpd_register_uri_handler(http_server_handle, &(httpd_uri_t){"/toggle_green.json", HTTP_POST, http_server_toggle_green_led_handler, NULL});
        httpd_register_uri_handler(http_server_handle, &(httpd_uri_t){"/toggle_blue.json", HTTP_POST, http_server_toggle_blue_led_handler, NULL});

        return http_server_handle;
    }

    return NULL;
}

/*******************************************************
 *          START / STOP HTTP SERVER
 ********************************************************/

void http_server_start(void)
{
    if (http_server_handle == NULL)
    {
        http_server_monitor_queue_handle =
            xQueueCreate(3, sizeof(http_server_queue_message_t));

        xTaskCreatePinnedToCore(
            &http_server_monitor,
            "http_server_monitor",
            HTTP_SERVER_MONITOR_STACK_SIZE,
            NULL,
            HTTP_SERVER_MONITOR_PRIORITY,
            &task_http_server_monitor,
            HTTP_SERVER_MONITOR_CORE_ID
        );

        http_server_configure();
    }
}

void http_server_stop(void)
{
    if (http_server_handle) {
        httpd_stop(http_server_handle);
        http_server_handle = NULL;
    }

    if (task_http_server_monitor) {
        vTaskDelete(task_http_server_monitor);
        task_http_server_monitor = NULL;
    }
}

BaseType_t http_server_monitor_send_message(http_server_message_e msgID)
{
    http_server_queue_message_t msg = { msgID };
    return xQueueSend(http_server_monitor_queue_handle, &msg, portMAX_DELAY);
}

void http_server_fw_update_reset_callback(void *arg)
{
    esp_restart();
}
