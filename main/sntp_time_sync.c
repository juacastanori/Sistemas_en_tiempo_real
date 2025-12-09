/**
 * @file sntp_time_sync.c
 * @brief Sincronización de hora mediante SNTP.
 *
 * Configura el cliente SNTP, espera la sincronización y proporciona
 * una API pequeña para obtener la hora actual del sistema como `struct tm`.
 * El módulo es utilizado por el servidor HTTP para mostrar la fecha/hora
 * sincronizada en la interfaz del usuario.
 */

#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sntp_time_sync.h"

static const char TAG[] = "sntp_sync";

/**
 * @brief Callback invocado cuando la hora ha sido sincronizada.
 */
void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification that time has been synchronized. Time: %ld", tv->tv_sec);
    sntp_time_sync_print_time(); /* Opcional: imprimir inmediatamente */
}

/* Función de inicialización SNTP */

void sntp_time_sync_init(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    
    // Usar prefijo esp_sntp_
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL); // Equivalente moderno
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");
    esp_sntp_setservername(2, "time.windows.com");

    // Configurar callback (opcional, si usas la version con notificacion)
    // sntp_set_time_sync_notification_cb(time_sync_notification_cb);

    esp_sntp_init(); // <--- esp_sntp_init en lugar de sntp_init

    // ... (El resto del código de espera y zona horaria se mantiene igual)
    /* Esperar a que el tiempo esté sincronizado... */
    int retry = 0;
    const int retry_count = 10;
    
    setenv("TZ", SNTP_TIME_ZONE, 1);
    tzset();

    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for time synchronization... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    // ...
}

void sntp_time_sync_stop(void)
{
    
    if (esp_sntp_enabled()) { 
        esp_sntp_stop();
        ESP_LOGI(TAG, "SNTP stopped");
    }
}

struct tm sntp_time_sync_get_time(void)
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    return timeinfo;
}

/**
 * @brief Imprime la hora y fecha actual en la consola
 *
 * Esta función obtiene la hora actual desde el reloj del sistema (sincronizada
 * con SNTP si está disponible) y la imprime formateada en la consola mediante
 * `ESP_LOGI`.
 */
void sntp_time_sync_print_time(void)
{
    char strftime_buf[64];
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "Current time in %s is: %s", SNTP_TIME_ZONE, strftime_buf);
}