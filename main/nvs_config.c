/*
 * nvs_config.c
 *
 * Non-Volatile Storage (Flash) configuration implementation
 * Uses ESP-IDF NVS library to persistently store system configuration
 */

#include "nvs_config.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include <string.h>

static const char TAG[] = "NVS_CONFIG";

// NVS namespace for our application
#define NVS_NAMESPACE "config"

// NVS key names
#define NVS_KEY_MANUAL_PWM      "manual_pwm"
#define NVS_KEY_AUTO_TMIN       "auto_tmin"
#define NVS_KEY_AUTO_TMAX       "auto_tmax"
#define NVS_KEY_REG_0           "reg_0"
#define NVS_KEY_REG_1           "reg_1"
#define NVS_KEY_REG_2           "reg_2"

/**
 * @brief Initialize NVS
 */
int nvs_config_init(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
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
 * @brief Deinitialize NVS
 */
int nvs_config_deinit(void)
{
    nvs_flash_deinit();
    ESP_LOGI(TAG, "NVS deinitialized");
    return 0;
}

/* ============================================================
 *           MANUAL MODE: Store and Load PWM
 * ============================================================*/

int nvs_config_save_manual_pwm(int pwm_value)
{
    nvs_handle_t handle;
    esp_err_t err;

    // Clamp PWM to valid range
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
 *        AUTOMATIC MODE: Store and Load Tmin/Tmax
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

    // Store as uint32_t bit-cast (float is 4 bytes)
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

    // Bit-cast back to float
    *tmin = *(float *)&tmin_bits;
    *tmax = *(float *)&tmax_bits;

    ESP_LOGI(TAG, "Auto temps loaded: Tmin=%.2f, Tmax=%.2f", *tmin, *tmax);
    return 0;
}

/* ============================================================
 *      PROGRAMMED MODE: Store and Load Registers
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

    // Select key based on index
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

    // Store as blob (raw binary data)
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

    // Select key based on index
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
            // If a register fails to load, initialize it with defaults
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
 *           HELPER: Clear all configurations
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
