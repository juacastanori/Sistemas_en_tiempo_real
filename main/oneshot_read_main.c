/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>  
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "led_rgb_library.h"

const static char *TAG = "ADC";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels

#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_1
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_0
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12

static int adc_raw[2];
static int voltage[2];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

// Led RGB configuration

#define GPIO_LED_R    GPIO_NUM_4   // Red
#define GPIO_LED_G    GPIO_NUM_3   // Green
#define GPIO_LED_B    GPIO_NUM_5   // Blue

#define LED_CHANNEL_R LEDC_CHANNEL_0
#define LED_CHANNEL_G LEDC_CHANNEL_1
#define LED_CHANNEL_B LEDC_CHANNEL_2


#define LED_TIMER     LEDC_TIMER_0
#define LED_DUTY_RES_BITS LEDC_TIMER_8_BIT
#define LED_FREQ_HZ   5000

// Raw -> 8-bit mapping limits
#define RAW_MIN 150
#define RAW_MAX 4095

#define VCC_MV 3300.0f
#define R_FIXED_OHMS 100.0f       // Rfixed (100 ohm)
#define R0_OHMS 100.0f            // R0 Termistor at T0 (100 ohm)
#define T0_K 298.15f              // 25 °C in Kelvin
#define BETA_CONST 3100.0f        // Beta in Kelvin

void app_main(void)
{
    // Configure Led RGB
    LED_RGB_t my_led = configure_LED_RGB(GPIO_LED_R, GPIO_LED_G, GPIO_LED_B,
                                         LED_CHANNEL_R, LED_CHANNEL_G, LED_CHANNEL_B,
                                         LED_TIMER, LED_DUTY_RES_BITS, LED_FREQ_HZ);
    // Start with Green
    set_led_green(&my_led.green, 0);

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN1, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    adc_cali_handle_t adc1_cali_chan1_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);
    bool do_calibration1_chan1 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN1, EXAMPLE_ADC_ATTEN, &adc1_cali_chan1_handle);

     // Timer for printing once per second
    int64_t last_print = esp_timer_get_time();

    while (1) {
        /* Read ADC1 */
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0]));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw[1]));

        // Set green led brightness according to measured voltage
        int raw_val = adc_raw[0];
        if (raw_val < RAW_MIN) raw_val = RAW_MIN;
        if (raw_val > RAW_MAX) raw_val = RAW_MAX;

        // linear mapping with rounding
        uint32_t green_8bit = (uint32_t)(((raw_val - RAW_MIN) * 255 + ((RAW_MAX - RAW_MIN) / 2)) / (RAW_MAX - RAW_MIN));
        // Update green LED duty (resolution set to 8 bits in configure_LED_RGB)
        set_led_green(&my_led.green, green_8bit);

        // Calculate temperature from thermistor reading (ADC1_CHAN1) 
        float v_therm_mv = 0.0f;
        bool got_voltage_chan1 = false;
        if (do_calibration1_chan1 && adc1_cali_chan1_handle != NULL) {
            esp_err_t err = adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[1], &voltage[1]);
            if (err == ESP_OK) {
                v_therm_mv = (float)voltage[1];
                got_voltage_chan1 = true;
            } else {
                ESP_LOGW(TAG, "adc_cali_raw_to_voltage chan1 failed (err %d). Using approx", err);
            }
        }
        if (!got_voltage_chan1) {
            // Aproximate voltage if no calibration
            v_therm_mv = (float)adc_raw[1] * (VCC_MV / 4095.0f);
        }

        // Avoid div by zero and out of range values
        float v_over_vcc = v_therm_mv / VCC_MV;
        if (v_over_vcc < 0.0001f) v_over_vcc = 0.0001f;
        if (v_over_vcc > 0.9999f) v_over_vcc = 0.9999f;

        // Calculate thermistor resistance (divisor: Vcc - R_fixed - R_therm - GND)
        // Vout = Vcc * R_therm / (R_fixed + R_therm)  => R_therm = R_fixed * Vout / (Vcc - Vout)
        float r_therm = (R_FIXED_OHMS * v_therm_mv) / (VCC_MV - v_therm_mv);

        // Beta equation to get temperature in Kelvin and Celsius
        float temp_k = 1.0f / ( (1.0f / T0_K) + (1.0f / BETA_CONST) * logf(r_therm / R0_OHMS) );
        float temp_c = temp_k - 273.15f;

        // Map 10°C -> 0, 50°C -> 255
        float t_min = 10.0f;
        float t_max = 50.0f;
        uint32_t red_8bit;
        if (temp_c <= t_min) {
            red_8bit = 0;
        } else if (temp_c >= t_max) {
            red_8bit = 255;
        } else {
            float scaled = (temp_c - t_min) * 255.0f / (t_max - t_min);
            red_8bit = (uint32_t) (roundf(scaled));
        }

        // Set red LED brightness according to temperature
        set_led_red(&my_led.red, red_8bit);

        //Print once per second
        int64_t now = esp_timer_get_time();
        if (now - last_print >= 1000000) {
            last_print = now;
            
            //Print ADC1 raw data
            ESP_LOGI(TAG, "POTENTIOMETER: ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw[0]);
            //Convert adc_reading to voltage in mV
            if (do_calibration1_chan0) {
                esp_err_t err = adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0], &voltage[0]);
                if (err == ESP_OK) { 
                    ESP_LOGI(TAG, "POTENTIOMETER: ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, voltage[0]);
                } else {
                    ESP_LOGW(TAG, "adc_cali_raw_to_voltage failed (err %d). Using approx", err);
                }
            } 

            ESP_LOGI(TAG, "POTENTIOMETER: Mapped green duty (0..255): %u (from RAW=%d)", green_8bit, adc_raw[0]);
            ESP_LOGI(TAG, "TERMISTOR: ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN1, adc_raw[1]);
            ESP_LOGI(TAG, "TERMISTOR: Voltage ~ %.1f mV, Rtherm ~ %.1f ohm, Temp ~ %.2f °C, Mapped red duty (0..255): %u", v_therm_mv, r_therm, temp_c, red_8bit);
        }
        //Delay a bit between readings
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0) {
        example_adc_calibration_deinit(adc1_cali_chan0_handle);
    }
    if (do_calibration1_chan1) {
        example_adc_calibration_deinit(adc1_cali_chan1_handle);
    }
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
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
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
