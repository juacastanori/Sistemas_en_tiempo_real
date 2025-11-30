#include "thermistor_reader.h"
#include "config_app.h" // Asumo que esta tiene las constantes (VCC_MV, R_FIXED_OHMS, etc.)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" // Nuevo: Incluir FreeRTOS Queue
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "THERMISTOR";
// La temperatura global se reemplaza por el handle de la cola.
static QueueHandle_t temp_queue_handle = NULL; 

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t cali_term_handle = NULL;

// --- Funciones de Calibracion ADC (se mantienen sin cambios) ---

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
    if (!handle) return;
    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
        ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
    #elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
        ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
    #endif
}

// --- Tarea de lectura del termistor ---

static void thermistor_read_task(void *pvParameters)
{
    int raw_term;
    int volt_term; // mV
    float temp_c;

    while(1) {
        // 1. Leer el ADC del termistor
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN_TERM, &raw_term));
        
        // 2. Convertir RAW a Voltaje (mV)
        if (cali_term_handle) {
            adc_cali_raw_to_voltage(cali_term_handle, raw_term, &volt_term);
        } else {
            volt_term = (int)((float)raw_term * VCC_MV / ADC_MAX_RAW);
        }

        // 3. Cálculo de Temperatura (Ecuación Beta)
        float v_therm_mv = (float)volt_term;
        
        float v_over_vcc = v_therm_mv / VCC_MV;
        if (v_over_vcc < 0.0001f) v_over_vcc = 0.0001f;
        if (v_over_vcc > 0.9999f) v_over_vcc = 0.9999f;

        float r_therm = (R_FIXED_OHMS * v_therm_mv) / (VCC_MV - v_therm_mv);
        
        float temp_k = 1.0f / ( (1.0f / T0_K) + (1.0f / BETA_CONST) * logf(r_therm / R0_OHMS) );
        
        // Convertir a Celsius
        temp_c = temp_k - 273.15f;

        // 4. Enviar la temperatura a la cola (sobrescribiendo el valor anterior)
        if (temp_queue_handle != NULL) {
             // Usamos xQueueOverwrite ya que solo nos interesa el valor más reciente
             xQueueOverwrite(temp_queue_handle, &temp_c); 
        }

        ESP_LOGI(TAG, "Raw: %d, Volt: %d mV, Temp: %.2f C", raw_term, volt_term, temp_c);

        vTaskDelay(pdMS_TO_TICKS(500)); // Lee y actualiza cada 500 ms
    }
}

// --- Implementación de la Interfaz Pública ---

void thermistor_init(void)
{
    // --- Crear Cola (tamaño 1 para la última lectura float) ---
    temp_queue_handle = xQueueCreate(1, sizeof(float));
    if (temp_queue_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create temp_queue_handle");
        return;
    }

    // --- ADC1 Init ---
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // --- ADC1 Config ---
    adc_oneshot_chan_cfg_t config = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN_TERM, &config));

    // --- ADC1 Calibration Init ---
    if (!example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN_TERM, EXAMPLE_ADC_ATTEN, &cali_term_handle)) {
        ESP_LOGW(TAG, "Thermistor calibration failed or not supported. Using raw scaling.");
    }
    
    // --- Crear Tarea ---
    xTaskCreate(thermistor_read_task, "thermistor_read_task", 2048, NULL, 5, NULL);
}

QueueHandle_t get_temperature_queue_handle(void)
{
    // Retorna el handle de la cola para que otras tareas puedan leer de ella
    return temp_queue_handle;
}