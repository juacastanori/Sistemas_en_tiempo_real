/* UART Events Example + ADC + LED RGB control + Button control
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "led_rgb_library.h"
#include "config_term_pot.h"


//ADC1 Channels
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_1 //Potentiometer
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_0 //Thermistor
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12

static int adc_raw[2];
static int voltage[2];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

// Led RGB configuration
#define GPIO_LED_R    GPIO_NUM_3   // Red
#define GPIO_LED_G    GPIO_NUM_4   // Green
#define GPIO_LED_B    GPIO_NUM_5   // Blue

#define LED_CHANNEL_R LEDC_CHANNEL_0
#define LED_CHANNEL_G LEDC_CHANNEL_1
#define LED_CHANNEL_B LEDC_CHANNEL_2

#define LED_TIMER     LEDC_TIMER_0
#define LED_DUTY_RES_BITS LEDC_TIMER_8_BIT
#define LED_FREQ_HZ   5000

// Boot Button configuration 
#define GPIO_BUTTON     GPIO_NUM_9
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_BUTTON)
static QueueHandle_t gpio_evt_queue = NULL;

//UART configuration
static const char *TAG = "UART: ";

#define TXD_PIN GPIO_NUM_21
#define RXD_PIN GPIO_NUM_20
#define EX_UART_NUM UART_NUM_0

#define BUF_SIZE (1024)
static QueueHandle_t uart0_queue;

static volatile bool print_temp=true;
static uint8_t led_brightness = 0;
static LED_RGB_t my_led;

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t cali_pot = NULL, cali_term = NULL;

// Function to read temperature from thermistor
static float read_temperature(void){

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw[1]));    
    if (cali_term) {
        adc_cali_raw_to_voltage(cali_term, adc_raw[1], &voltage[1]);
    } else {
        ESP_LOGW(TAG, "Calibration handle not initialized");
        //No calibration, raw to voltage conversion
        voltage[1] = (adc_raw[1] * VCC_MV) / ADC_MAX_RAW;
    }
    float v_therm_mv = (float)voltage[1];
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

    return temp_c;
}

// Function to control LED color based on temperature thresholds

static void led_temp_control(float temp){
    uint8_t r = 0, g = 0, b = 0;

    if (temp >= thr_red.min && temp <= thr_red.max) r = led_brightness;
    if (temp >= thr_green.min && temp <= thr_green.max) g = led_brightness;
    if (temp >= thr_blue.min && temp <= thr_blue.max) b = led_brightness;

    set_LED_RGB_color(&my_led, r, g, b);
}

// GPIO ISR handler
static void IRAM_ATTR gpio_isr_handler(void* arg){
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// GPIO task to handle button events
static void gpio_task_handler(void* arg){
    uint32_t io_num;
    while(1){
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)){
            int level = gpio_get_level(io_num);
            if (level == 0) {
                print_temp = false;
            } else {
                print_temp = true;
            }
        }
    }
}

// ADC potentiometer reading task
static void adc_pot_task(void *pvParameters){
    while(1){
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0]));
        if (cali_pot) {
            adc_cali_raw_to_voltage(cali_pot, adc_raw[0], &voltage[0]);
        } else {
            ESP_LOGW(TAG, "Calibration handle not initialized");
            //No calibration, raw to voltage conversion
            voltage[0] = (adc_raw[0] * VCC_MV) / 4095;
        }

        // Set led brightness according to measured voltage
        int raw_val = adc_raw[0];
        if (raw_val < RAW_MIN) raw_val = RAW_MIN;
        if (raw_val > RAW_MAX) raw_val = RAW_MAX;

        // linear mapping with rounding
        led_brightness = (uint8_t)(((raw_val - RAW_MIN) * 255 + ((RAW_MAX - RAW_MIN) / 2)) / (RAW_MAX - RAW_MIN));
        float temp = read_temperature();
        led_temp_control(temp);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// UART task to write temperature periodically
static void write_temp_uart_task(void *pvParameters){
    while(1){
        float temp = read_temperature();
        led_temp_control(temp);
        if(print_temp){
            char temp_str[50];
            snprintf(temp_str, sizeof(temp_str), "Temperature: %.2f °C\r\n", temp);
            uart_write_bytes(EX_UART_NUM, temp_str, strlen(temp_str));
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
// Command format to set thresholds: THR <R/G/B> <min> <max>
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t data[64];

    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                int len =uart_read_bytes(EX_UART_NUM, data, event.size, portMAX_DELAY);
                data[len] = '\0';
                if (strncmp((char *)data, "THR", 3) == 0) {
                    char color;
                    float min, max;
                    if (sscanf((char *)data, "THR %c %f %f", &color, &min, &max) == 3) {
                        switch (color) {
                            case 'R':
                                thr_red.min = min;
                                thr_red.max = max;
                                break;
                            case 'G':
                                thr_green.min = min;
                                thr_green.max = max;
                                break;
                            case 'B':
                                thr_blue.min = min;
                                thr_blue.max = max;
                                break;
                            default:
                                break;
                    }
                    char response[64];
                    snprintf(response, sizeof(response), "Thresholds updated for %c: %.1f - %.1f\r\n", color, min, max);
                    uart_write_bytes(EX_UART_NUM, response, strlen(response));
                }
            }        
        }
    }

}
}

void app_main(void)
{
    // Configure Led RGB
    my_led = configure_LED_RGB(GPIO_LED_R, GPIO_LED_G, GPIO_LED_B,
                                         LED_CHANNEL_R, LED_CHANNEL_G, LED_CHANNEL_B,
                                         LED_TIMER, LED_DUTY_RES_BITS, LED_FREQ_HZ);
    // Start with leds off
    set_LED_RGB_color(&my_led, 0, 0, 0);

    //-------------ADC1 Init---------------//

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
    
    example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &cali_pot);
    example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN1, EXAMPLE_ADC_ATTEN, &cali_term);

    //----------UART config----------//
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);
    uart_set_pin(EX_UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //----------Button GPIO config----------//
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,   // Interrupt of rising edge and falling edge
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    //Create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_BUTTON, gpio_isr_handler, (void*) GPIO_BUTTON);

    //Create tasks
    xTaskCreate(adc_pot_task, "adc_pot_task", 2048, NULL, 10, NULL);
    xTaskCreate(write_temp_uart_task, "write_temp_uart_task", 4096, NULL, 11, NULL);
    xTaskCreate(gpio_task_handler, "gpio_task_handler", 2048, NULL, 10, NULL);
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
}

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

