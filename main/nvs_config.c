/*
 * nvs_config.c
 *
 * Implementación de configuración en almacenamiento no volátil (Flash)
 * Utiliza la librería NVS de ESP-IDF para almacenar de forma persistente
 * la configuración del sistema
 */

#include "nvs_config.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "wifi_app.h"
#include <string.h>

static const char TAG[] = "NVS_CONFIG";

/* Espacio de nombres NVS para la aplicación */
#define NVS_NAMESPACE "config"

/* Claves NVS para almacenar valores */
#define NVS_KEY_MANUAL_PWM      "manual_pwm"
#define NVS_KEY_AUTO_TMIN       "auto_tmin"
#define NVS_KEY_AUTO_TMAX       "auto_tmax"
#define NVS_KEY_REG_0           "reg_0"
#define NVS_KEY_REG_1           "reg_1"
#define NVS_KEY_REG_2           "reg_2"
#define NVS_KEY_WIFI_SSID       "wifi_ssid"
#define NVS_KEY_WIFI_PASSWORD   "wifi_password"

/**
 * @brief Inicializa NVS (almacenamiento Flash).
 *
 * Inicializa la partición NVS y maneja su re-inicialización si la
 * partición requiere borrado. Debe llamarse antes de cualquier operación
 * de lectura/escritura en NVS.
 *
 * @param None
 * @return int 0 si la inicialización fue exitosa, -1 en caso de error
 */
int nvs_config_init(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* Partición NVS fue truncada y necesita ser borrada */
        /* Reintentar nvs_flash_init */
        ESP_LOGW(TAG, "NVS partition needs erasing; erasing and reinitializing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(err));
        return -1;
    }

    ESP_LOGI(TAG, "NVS initialized successfully");
    return 0;
}

/**
 * @brief Desinicializa NVS.
 *
 * Libera recursos usados por el módulo NVS. No es estrictamente necesario
 * en todas las plataformas, pero se proporciona por completitud.
 *
 * @param None
 * @return int 0 siempre (actualmente no falla)
 */
int nvs_config_deinit(void)
{
    nvs_flash_deinit();
    ESP_LOGI(TAG, "NVS deinitialized");
    return 0;
}

/* ============================================================
 *           Modo Manual: Guardar y cargar PWM
 * ============================================================*/

/**
 * @brief Guarda el valor PWM manual en NVS.
 *
 * El valor se normaliza al rango 0-100 y se almacena bajo la clave
 * `manual_pwm`.
 *
 * @param pwm_value Valor de PWM a guardar (0-100)
 * @return int 0 si se guardó correctamente, -1 en caso de error
 */
int nvs_config_save_manual_pwm(int pwm_value)
{
    nvs_handle_t handle;
    esp_err_t err;

    /* Limitar el valor de PWM al rango válido */
    if (pwm_value < 0) pwm_value = 0;
    if (pwm_value > 100) pwm_value = 100;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return -1;
    }

    err = nvs_set_i32(handle, NVS_KEY_MANUAL_PWM, (int32_t)pwm_value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_i32 failed for manual_pwm: %s", esp_err_to_name(err));
        nvs_close(handle);
        return -1;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Manual PWM saved: %d", pwm_value);
        return 0;
    } else {
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
        return -1;
    }
}

/**
 * @brief Carga el valor PWM manual desde NVS.
 *
 * Lee la clave `manual_pwm` y devuelve el valor a través del puntero
 * proporcionado.
 *
 * @param pwm_value Puntero donde se almacenará el valor leído
 * @return int 0 si se cargó correctamente, -1 en caso de error
 */
int nvs_config_load_manual_pwm(int *pwm_value)
{
    nvs_handle_t handle;
    esp_err_t err;
    int32_t stored_pwm = 0;

    if (pwm_value == NULL) {
        ESP_LOGE(TAG, "pwm_value pointer is NULL");
        return -1;
    }

    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s (key may not exist)", esp_err_to_name(err));
        return -1;
    }

    err = nvs_get_i32(handle, NVS_KEY_MANUAL_PWM, &stored_pwm);
    nvs_close(handle);

    if (err == ESP_OK) {
        *pwm_value = (int)stored_pwm;
        ESP_LOGI(TAG, "Manual PWM loaded: %d", *pwm_value);
        return 0;
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Manual PWM key not found in NVS");
        return -1;
    } else {
        ESP_LOGE(TAG, "nvs_get_i32 failed: %s", esp_err_to_name(err));
        return -1;
    }
}

/* ============================================================
 *        Modo Automático: Guardar y cargar Tmin/Tmax
 * ============================================================*/

/**
 * @brief Guarda los límites automáticos de temperatura en NVS.
 *
 * Los floats se reinterpretan como `uint32_t` y se almacenan bajo las
 * claves `auto_tmin` y `auto_tmax`.
 *
 * @param tmin Temperatura mínima automática
 * @param tmax Temperatura máxima automática
 * @return int 0 si se guardó correctamente, -1 en caso de error
 */
int nvs_config_save_auto_temps(float tmin, float tmax)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return -1;
    }

    /* Almacenar como uint32_t reinterpretado (float tiene 4 bytes) */
    uint32_t tmin_bits = *(uint32_t *)&tmin;
    uint32_t tmax_bits = *(uint32_t *)&tmax;

    err = nvs_set_u32(handle, NVS_KEY_AUTO_TMIN, tmin_bits);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_u32 failed for auto_tmin: %s", esp_err_to_name(err));
        nvs_close(handle);
        return -1;
    }

    err = nvs_set_u32(handle, NVS_KEY_AUTO_TMAX, tmax_bits);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_u32 failed for auto_tmax: %s", esp_err_to_name(err));
        nvs_close(handle);
        return -1;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Auto temps saved: Tmin=%.2f, Tmax=%.2f", tmin, tmax);
        return 0;
    } else {
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
        return -1;
    }
}

/**
 * @brief Carga los límites automáticos de temperatura desde NVS.
 *
 * Reinterpreta los valores almacenados como floats y los devuelve
 * mediante los punteros provistos.
 *
 * @param tmin Puntero para recibir la temperatura mínima
 * @param tmax Puntero para recibir la temperatura máxima
 * @return int 0 si se cargó correctamente, -1 en caso de error
 */
int nvs_config_load_auto_temps(float *tmin, float *tmax)
{
    nvs_handle_t handle;
    esp_err_t err;
    uint32_t tmin_bits = 0, tmax_bits = 0;

    if (tmin == NULL || tmax == NULL) {
        ESP_LOGE(TAG, "tmin or tmax pointer is NULL");
        return -1;
    }

    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s (key may not exist)", esp_err_to_name(err));
        return -1;
    }

    err = nvs_get_u32(handle, NVS_KEY_AUTO_TMIN, &tmin_bits);
    if (err != ESP_OK) {
        nvs_close(handle);
        ESP_LOGW(TAG, "Auto Tmin key not found or error: %s", esp_err_to_name(err));
        return -1;
    }

    err = nvs_get_u32(handle, NVS_KEY_AUTO_TMAX, &tmax_bits);
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Auto Tmax key not found or error: %s", esp_err_to_name(err));
        return -1;
    }

    /* Reinterpretar bits de vuelta a float */
    *tmin = *(float *)&tmin_bits;
    *tmax = *(float *)&tmax_bits;

    ESP_LOGI(TAG, "Auto temps loaded: Tmin=%.2f, Tmax=%.2f", *tmin, *tmax);
    return 0;
}

/* ============================================================
 *      Modo Programado: Guardar y cargar registros
 * ============================================================*/

/**
 * @brief Guarda un registro programado en NVS.
 *
 * Serializa la estructura `scheduled_register_t` como blob y la guarda
 * bajo la clave correspondiente al índice (0..2).
 *
 * @param index Índice del registro (0..2)
 * @param reg Puntero a la estructura del registro a guardar
 * @return int 0 si se guardó correctamente, -1 en caso de error
 */
int nvs_config_save_register(int index, const scheduled_register_t *reg)
{
    nvs_handle_t handle;
    esp_err_t err;
    const char *key = NULL;

    if (index < 0 || index > 2) {
        ESP_LOGE(TAG, "Invalid register index: %d", index);
        return -1;
    }

    if (reg == NULL) {
        ESP_LOGE(TAG, "register pointer is NULL");
        return -1;
    }

    /* Seleccionar clave según índice */
    switch (index) {
        case 0: key = NVS_KEY_REG_0; break;
        case 1: key = NVS_KEY_REG_1; break;
        case 2: key = NVS_KEY_REG_2; break;
    }

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return -1;
    }

    /* Almacenar como blob (datos binarios crudos) */
    err = nvs_set_blob(handle, key, reg, sizeof(scheduled_register_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_blob failed for register %d: %s", index, esp_err_to_name(err));
        nvs_close(handle);
        return -1;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Register %d saved: active=%d, time=%02d:%02d-%02d:%02d, T=%.1f-%.1f",
                 index, reg->active, reg->start_hour, reg->start_min, 
                 reg->end_hour, reg->end_min, reg->temp_min, reg->temp_max);
        return 0;
    } else {
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
        return -1;
    }
}

/**
 * @brief Carga un registro programado desde NVS.
 *
 * Lee el blob correspondiente al índice solicitado y lo deserializa en
 * la estructura proporcionada.
 *
 * @param index Índice del registro (0..2)
 * @param reg Puntero donde se almacenará el registro cargado
 * @return int 0 si se cargó correctamente, -1 en caso de error
 */
int nvs_config_load_register(int index, scheduled_register_t *reg)
{
    nvs_handle_t handle;
    esp_err_t err;
    const char *key = NULL;
    size_t required_size = sizeof(scheduled_register_t);

    if (index < 0 || index > 2) {
        ESP_LOGE(TAG, "Invalid register index: %d", index);
        return -1;
    }

    if (reg == NULL) {
        ESP_LOGE(TAG, "register pointer is NULL");
        return -1;
    }

    /* Seleccionar clave según índice */
    switch (index) {
        case 0: key = NVS_KEY_REG_0; break;
        case 1: key = NVS_KEY_REG_1; break;
        case 2: key = NVS_KEY_REG_2; break;
    }

    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s (register may not exist)", esp_err_to_name(err));
        return -1;
    }

    err = nvs_get_blob(handle, key, reg, &required_size);
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Register %d loaded: active=%d, time=%02d:%02d-%02d:%02d, T=%.1f-%.1f",
                 index, reg->active, reg->start_hour, reg->start_min,
                 reg->end_hour, reg->end_min, reg->temp_min, reg->temp_max);
        return 0;
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Register %d key not found in NVS", index);
        return -1;
    } else {
        ESP_LOGE(TAG, "nvs_get_blob failed for register %d: %s", index, esp_err_to_name(err));
        return -1;
    }
}

/**
 * @brief Guarda los 3 registros programados en NVS.
 *
 * Invoca `nvs_config_save_register` para cada registro y devuelve
 * error si cualquiera de las operaciones falla.
 *
 * @param registers Puntero al arreglo de 3 registros
 * @return int 0 si se guardaron correctamente, -1 en caso de error
 */
int nvs_config_save_all_registers(const scheduled_register_t *registers)
{
    int ret;

    if (registers == NULL) {
        ESP_LOGE(TAG, "registers pointer is NULL");
        return -1;
    }

    for (int i = 0; i < 3; i++) {
        ret = nvs_config_save_register(i, &registers[i]);
        if (ret != 0) {
            ESP_LOGE(TAG, "Failed to save register %d", i);
            return -1;
        }
    }

    ESP_LOGI(TAG, "All 3 registers saved successfully");
    return 0;
}

/**
 * @brief Carga los 3 registros programados desde NVS.
 *
 * Si un registro no existe o falla la carga, se inicializa con valores
 * por defecto y la función continúa con el siguiente registro.
 *
 * @param registers Puntero al arreglo donde se almacenarán los registros
 * @return int 0 si la operación completó (registros faltantes usan valores por defecto)
 */
int nvs_config_load_all_registers(scheduled_register_t *registers)
{
    int ret;

    if (registers == NULL) {
        ESP_LOGE(TAG, "registers pointer is NULL");
        return -1;
    }

    for (int i = 0; i < 3; i++) {
        ret = nvs_config_load_register(i, &registers[i]);
        if (ret != 0) {
            // Si un registro no se pudo cargar, inicializarlo con valores por defecto
            registers[i].active = 0;
            registers[i].start_hour = 0;
            registers[i].start_min = 0;
            registers[i].end_hour = 0;
            registers[i].end_min = 0;
            registers[i].temp_min = 20.0f;
            registers[i].temp_max = 30.0f;
            ESP_LOGW(TAG, "Register %d not found, using defaults", i);
        }
    }

    ESP_LOGI(TAG, "All registers loaded (missing registers use defaults)");
    return 0;
}

/* ============================================================
 *           HELPER: Limpiar todas las configuraciones
 * ============================================================*/

/**
 * @brief Borra todas las configuraciones almacenadas en NVS.
 *
 * Elimina todas las claves en el espacio de nombres de la aplicación.
 *
 * @param None
 * @return int 0 si se borró correctamente, -1 en caso de error
 */
int nvs_config_erase_all(void)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return -1;
    }

    err = nvs_erase_all(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_erase_all failed: %s", esp_err_to_name(err));
        nvs_close(handle);
        return -1;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "All configurations erased");
        return 0;
    } else {
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
        return -1;
    }
}

/* ============================================================
 *     Credenciales WiFi STA: Guardar y cargar SSID/Password
 * ============================================================*/

/**
 * @brief Guarda las credenciales WiFi STA (SSID y contraseña) en flash
 */
int nvs_config_save_wifi_credentials(const char *ssid, const char *password)
{
    nvs_handle_t handle;
    esp_err_t err;

    if (ssid == NULL || password == NULL) {
        ESP_LOGE(TAG, "SSID or password pointer is NULL");
        return -1;
    }

    // Validar longitudes
    if (strlen(ssid) > MAX_SSID_LENGTH || strlen(password) > MAX_PASSWORD_LENGTH) {
        ESP_LOGE(TAG, "SSID or password exceeds maximum length");
        return -1;
    }

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return -1;
    }

    // Guardar SSID
    err = nvs_set_str(handle, NVS_KEY_WIFI_SSID, ssid);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_str failed for SSID: %s", esp_err_to_name(err));
        nvs_close(handle);
        return -1;
    }

    // Guardar Password
    err = nvs_set_str(handle, NVS_KEY_WIFI_PASSWORD, password);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_str failed for password: %s", esp_err_to_name(err));
        nvs_close(handle);
        return -1;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "WiFi credentials saved: SSID=%s", ssid);
        return 0;
    } else {
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
        return -1;
    }
}

/**
 * @brief Carga las credenciales WiFi STA desde flash
 */
int nvs_config_load_wifi_credentials(char *ssid, char *password)
{
    nvs_handle_t handle;
    esp_err_t err;
    size_t ssid_len = MAX_SSID_LENGTH + 1;
    size_t password_len = MAX_PASSWORD_LENGTH + 1;

    if (ssid == NULL || password == NULL) {
        ESP_LOGE(TAG, "SSID or password pointer is NULL");
        return -1;
    }

    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s (credentials may not exist)", esp_err_to_name(err));
        return -1;
    }

    // Cargar SSID
    err = nvs_get_str(handle, NVS_KEY_WIFI_SSID, ssid, &ssid_len);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "WiFi SSID key not found in NVS");
        } else {
            ESP_LOGE(TAG, "nvs_get_str failed for SSID: %s", esp_err_to_name(err));
        }
        nvs_close(handle);
        return -1;
    }

    // Cargar Password
    err = nvs_get_str(handle, NVS_KEY_WIFI_PASSWORD, password, &password_len);
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "WiFi credentials loaded: SSID=%s", ssid);
        return 0;
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "WiFi password key not found in NVS");
        return -1;
    } else {
        ESP_LOGE(TAG, "nvs_get_str failed for password: %s", esp_err_to_name(err));
        return -1;
    }
}
