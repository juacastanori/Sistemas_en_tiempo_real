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
 * @brief Inicializa NVS (almacenamiento Flash)
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
 * @brief Desinicializa NVS
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
