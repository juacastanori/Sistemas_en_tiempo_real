#ifndef MAIN_SNTP_TIME_SYNC_H_
#define MAIN_SNTP_TIME_SYNC_H_

#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h"

// Zona horaria por defecto: UTC -5 (Colombia, Perú, Ecuador)
// Formato: <TZ_NAME><OFFSET>
// Por ejemplo, para UTC-5: "UTC-5"
// Para UTC+1 (España con horario de verano, por ejemplo): "CET-1CEST,M3.5.0/2,M10.5.0"
#define SNTP_TIME_ZONE "<-05>5"

/**
 * Inicializa y configura el cliente SNTP.
 * También establece la zona horaria.
 */
void sntp_time_sync_init(void);

/**
 * Detiene el servicio SNTP.
 */
void sntp_time_sync_stop(void);

/**
 * Obtiene la hora actual del sistema.
 * @return La hora actual del sistema como un struct tm.
 */
struct tm sntp_time_sync_get_time(void);

/**
 * Imprime la hora actual en el log.
 */
void sntp_time_sync_print_time(void);

#endif /* MAIN_SNTP_TIME_SYNC_H_ */