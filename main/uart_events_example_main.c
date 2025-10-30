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

// ADC handles

typedef struct {
    adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t cali_pot;
    adc_cali_handle_t cali_term;
} adc_ctx_t;

typedef struct {
    int raw_pot;
    int raw_term;
    int volt_pot;    // mV
    int volt_term;   // mV
    float temp_c;
    uint8_t brightness; // 0..255
} adc_msg_t;

typedef enum {
    COLOR_R,
    COLOR_G,
    COLOR_B
} color_e;

typedef struct {
    color_e color;
    float min_v;
    float max_v;
} thr_msg_t;

static QueueHandle_t adc_queue = NULL;
static QueueHandle_t thr_queue = NULL;
static QueueHandle_t uart_ctrl_queue = NULL;   // used previously to toggle printing
static QueueHandle_t uart_rate_queue = NULL;   // new: used to set rate (ms)

static SemaphoreHandle_t uart_mux = NULL;


// GPIO ISR handler
static void IRAM_ATTR gpio_isr_handler(void* arg){
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// ADC potentiometer reading task
static void adc_pot_term_task(void *pvParameters){
    adc_ctx_t *ctx = (adc_ctx_t *) pvParameters;
    adc_msg_t msg;
    while(1){
        // Read potentiometer
        ESP_ERROR_CHECK(adc_oneshot_read(ctx->adc1_handle, EXAMPLE_ADC1_CHAN0, &msg.raw_pot));
        if (ctx->cali_pot) {
            adc_cali_raw_to_voltage(ctx->cali_pot, msg.raw_pot, &msg.volt_pot);
        } else {
            ESP_LOGW(TAG, "Calibration handle not initialized");
            //No calibration, raw to voltage conversion
            msg.volt_pot = (msg.raw_pot * VCC_MV) / ADC_MAX_RAW;
        }

        // Read thermistor
        ESP_ERROR_CHECK(adc_oneshot_read(ctx->adc1_handle, EXAMPLE_ADC1_CHAN1, &msg.raw_term));
        if (ctx->cali_term) {
            adc_cali_raw_to_voltage(ctx->cali_term, msg.raw_term, &msg.volt_term);
        } else {
            ESP_LOGW(TAG, "Calibration handle not initialized");
            //No calibration, raw to voltage conversion
            msg.volt_term = (msg.raw_term * VCC_MV) / ADC_MAX_RAW;
        }

        float v_therm_mv = (float)msg.volt_term;
        // Avoid div by zero and out of range values
        float v_over_vcc = v_therm_mv / VCC_MV;
        if (v_over_vcc < 0.0001f) v_over_vcc = 0.0001f;
        if (v_over_vcc > 0.9999f) v_over_vcc = 0.9999f;
        // Calculate thermistor resistance (divisor: Vcc - R_fixed - R_therm - GND)
        float r_therm = (R_FIXED_OHMS * v_therm_mv) / (VCC_MV - v_therm_mv);
        // Beta equation to get temperature in Kelvin and Celsius
        float temp_k = 1.0f / ( (1.0f / T0_K) + (1.0f / BETA_CONST) * logf(r_therm / R0_OHMS) );
        msg.temp_c = temp_k - 273.15f;

        // Set led brightness according to measured voltage
        int raw_val = msg.raw_pot;
        if (raw_val < RAW_MIN) raw_val = RAW_MIN;
        if (raw_val > RAW_MAX) raw_val = RAW_MAX;

        // linear mapping with rounding
        msg.brightness = (uint8_t)(((raw_val - RAW_MIN) * 255 + ((RAW_MAX - RAW_MIN) / 2)) / (RAW_MAX - RAW_MIN));

        if (adc_queue){
            xQueueOverwrite(adc_queue, &msg);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void controller_task(void *pvParameters){

    LED_RGB_t *led = (LED_RGB_t *) pvParameters;
    
    threshold_t thr_r = { THR_RED_MIN_V, THR_RED_MAX_V };
    threshold_t thr_g = { THR_GREEN_MIN_V, THR_GREEN_MAX_V };
    threshold_t thr_b = { THR_BLUE_MIN_V, THR_BLUE_MAX_V };

    adc_msg_t adcmsg;
    thr_msg_t thrmsg;
    uint32_t gpio_evt;

    bool print_temp_local = true;
    bool led_forced_off = false;


    for(;;){
        // If LED not forced off, peek ADC message to update LED color
        if (!led_forced_off && adc_queue && xQueuePeek(adc_queue, &adcmsg, pdMS_TO_TICKS(50))==pdPASS){
            // Decide LED color based on temperature and thresholds
            uint8_t r = 0, g = 0, b = 0;
            if (adcmsg.temp_c >= thr_r.min && adcmsg.temp_c <= thr_r.max) r = adcmsg.brightness;
            if (adcmsg.temp_c >= thr_g.min && adcmsg.temp_c <= thr_g.max) g = adcmsg.brightness;
            if (adcmsg.temp_c >= thr_b.min && adcmsg.temp_c <= thr_b.max) b = adcmsg.brightness;
            set_LED_RGB_color(led, r, g, b);
        }

        // Check for threshold updates from UART
        if (thr_queue && xQueueReceive(thr_queue, &thrmsg, 0)==pdPASS){
            switch (thrmsg.color){
                case COLOR_R:
                    thr_r.min = thrmsg.min_v;
                    thr_r.max = thrmsg.max_v;
                    break;
                case COLOR_G:
                    thr_g.min = thrmsg.min_v;
                    thr_g.max = thrmsg.max_v;
                    break;
                case COLOR_B:
                    thr_b.min = thrmsg.min_v;
                    thr_b.max = thrmsg.max_v;
                    break;
                default:
                    break;
            }
            // Notify the controller task about the updated thresholds
            char response[64];
            const char *cname = (thrmsg.color == COLOR_R) ? "R" : (thrmsg.color == COLOR_G) ? "G" : "B";
            snprintf(response, sizeof(response), "Thresholds updated for %s: %.1f - %.1f\r\n", cname, thrmsg.min_v, thrmsg.max_v);
            if (xSemaphoreTake(uart_mux, pdMS_TO_TICKS(1000)) == pdTRUE) {
                uart_write_bytes(EX_UART_NUM, response, strlen(response));
                xSemaphoreGive(uart_mux);
            }
        }

        // GPIO button event handling
        if (gpio_evt_queue && xQueueReceive(gpio_evt_queue, &gpio_evt, 0) == pdPASS) {
            if ((uint32_t)gpio_evt == (uint32_t)GPIO_BUTTON) {
                // Read button level with debounce
                int level1 = gpio_get_level((gpio_num_t)GPIO_BUTTON);
                vTaskDelay(pdMS_TO_TICKS(30)); // debounce
                int level2 = gpio_get_level((gpio_num_t)GPIO_BUTTON);
                if (level1 == level2) {
                    
                    if (level2 == 0) {
                        // Toggle forced-off state
                        led_forced_off = !led_forced_off;
                        if (led_forced_off) {
                            // Force LED off
                            set_LED_RGB_color(led, 0, 0, 0);
                        } else {
                            // Restore with last ADC if available
                            adc_msg_t adc_local;
                            if (adc_queue && xQueuePeek(adc_queue, &adc_local, 0) == pdPASS) {
                                uint8_t rr = 0, gg = 0, bb = 0;
                                if (adc_local.temp_c >= thr_r.min && adc_local.temp_c <= thr_r.max) rr = adc_local.brightness;
                                if (adc_local.temp_c >= thr_g.min && adc_local.temp_c <= thr_g.max) gg = adc_local.brightness;
                                if (adc_local.temp_c >= thr_b.min && adc_local.temp_c <= thr_b.max) bb = adc_local.brightness;
                                set_LED_RGB_color(led, rr, gg, bb);
                            }
                        }

                        // Mensaje UART indicando nuevo estado
                        if (xSemaphoreTake(uart_mux, pdMS_TO_TICKS(500)) == pdTRUE) {
                            const char *msg = led_forced_off ? "TURNING LED OFF\r\n" : "TURNING LED ON\r\n";
                            uart_write_bytes(EX_UART_NUM, msg, strlen(msg));
                            xSemaphoreGive(uart_mux);
                        }

                        // Esperar hasta que se libere el botón (evita múltiples toggles por rebote)
                        while (gpio_get_level((gpio_num_t)GPIO_BUTTON) == 0) {
                            vTaskDelay(pdMS_TO_TICKS(20));
                        }
                        // pequeña pausa tras la liberación
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// UART task to write temperature periodically
static void write_temp_uart_task(void *pvParameters){
    adc_msg_t latest;
    bool print_temp_local = true;
    uint32_t period_ms = 1000; // default 1s

    for(;;){
        // Check if there's a new rate command (non-blocking)
        if (uart_rate_queue) {
            uint32_t new_period;
            if (xQueueReceive(uart_rate_queue, &new_period, 0) == pdPASS) {
                if (new_period < 10) new_period = 10; // clamp a minimum (10 ms)
                period_ms = new_period;
                // optionally report? we prefer uart_event_task to confirm
            }
        }

        // sleep -> uses the possibly-updated period
        vTaskDelay(pdMS_TO_TICKS(period_ms));

        if (uart_ctrl_queue){
            bool tmp;
            if (xQueueReceive(uart_ctrl_queue, &tmp, 0) == pdPASS){
                print_temp_local = tmp;
            }
        } // if printing disabled, skip
        if (!print_temp_local) {
            continue;
        }

        // Peek the latest ADC message (does not remove it)
        if (adc_queue && xQueuePeek(adc_queue, &latest, 0) == pdPASS) {
            char temp_str[80];
            snprintf(temp_str, sizeof(temp_str), "Temperature: %.2f C\r\n", latest.temp_c);
            if (xSemaphoreTake(uart_mux, pdMS_TO_TICKS(500)) == pdTRUE) {
                uart_write_bytes(EX_UART_NUM, temp_str, strlen(temp_str));
                xSemaphoreGive(uart_mux);
            }
        } else {
            // No ADC data yet — optionally notify or skip
            // (we skip to avoid spamming)
        }
    }
}
// Command formats
// Thresholds: THR <R/G/B> <min> <max>
// Time printing: RATE <ms>
// Read Potentiometer voltage: READ POT
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t data[128];

    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                int len = uart_read_bytes(EX_UART_NUM, data, sizeof(data)-1, pdMS_TO_TICKS(200));
                if (len <= 0) continue;
                data[len] = '\0';

                // Trim trailing CR/LF
                while (len > 0 && (data[len-1] == '\r' || data[len-1] == '\n')) {
                    data[len-1] = '\0';
                    len--;
                }

                // --- THR command (existing) ---
                if (strncmp((char *)data, "THR", 3) == 0) {
                    char color;
                    float min, max;
                    if (sscanf((char *)data, "THR %c %f %f", &color, &min, &max) == 3) {
                        thr_msg_t thrmsg;
                        switch (color) {
                            case 'R':
                                thrmsg.color = COLOR_R;
                                break;
                            case 'G':
                                thrmsg.color = COLOR_G;
                                break;
                            case 'B':
                                thrmsg.color = COLOR_B;
                                break;
                            default:
                                if (xSemaphoreTake(uart_mux, pdMS_TO_TICKS(500)) == pdTRUE) {
                                    uart_write_bytes(EX_UART_NUM, "Invalid color. Use R, G, or B.\r\n", 34);
                                    xSemaphoreGive(uart_mux);
                                }
                                continue;
                        }
                        thrmsg.min_v = min;
                        thrmsg.max_v = max;
                        if (thr_queue) {
                            xQueueSend(thr_queue, &thrmsg, portMAX_DELAY);
                        }
                    } else {
                        if (xSemaphoreTake(uart_mux, pdMS_TO_TICKS(1000)) == pdTRUE) {
                            uart_write_bytes(EX_UART_NUM, "Invalid command format. Use: THR <R/G/B> <min> <max>\r\n", 61);
                            xSemaphoreGive(uart_mux);
                        }
                    }
                    continue;
                }

                // --- RATE command (new) ---
                if (strncmp((char *)data, "RATE", 4) == 0) {
                    uint32_t ms;
                    if (sscanf((char *)data, "RATE %" SCNu32, &ms) == 1) {
                        if (ms < 10) ms = 10; // minimum clamp
                        if (uart_rate_queue) {
                            xQueueOverwrite(uart_rate_queue, &ms);
                        }
                        if (xSemaphoreTake(uart_mux, pdMS_TO_TICKS(500)) == pdTRUE) {
                            char resp[80];
                            snprintf(resp, sizeof(resp), "Print rate set to %" PRIu32 " ms\r\n", ms);
                            uart_write_bytes(EX_UART_NUM, resp, strlen(resp));
                            xSemaphoreGive(uart_mux);
                        }
                    } else {
                        if (xSemaphoreTake(uart_mux, pdMS_TO_TICKS(500)) == pdTRUE) {
                            uart_write_bytes(EX_UART_NUM, "Invalid RATE format. Use: RATE <ms>\r\n", 36);
                            xSemaphoreGive(uart_mux);
                        }
                    }
                    continue;
                }

                // --- READ POT (immediate) command (new) ---
                if (strncmp((char *)data, "READ POT", 8) == 0 ) {
                    adc_msg_t sample;
                    if (adc_queue && xQueuePeek(adc_queue, &sample, 0) == pdPASS) {
                        if (xSemaphoreTake(uart_mux, pdMS_TO_TICKS(500)) == pdTRUE) {
                            char resp[96];
                            snprintf(resp, sizeof(resp), "POT raw: %d   volt: %d mV\r\n", sample.raw_pot, sample.volt_pot);
                            uart_write_bytes(EX_UART_NUM, resp, strlen(resp));
                            xSemaphoreGive(uart_mux);
                        }
                    } else {
                        if (xSemaphoreTake(uart_mux, pdMS_TO_TICKS(500)) == pdTRUE) {
                            uart_write_bytes(EX_UART_NUM, "No pot data available yet\r\n", 27);
                            xSemaphoreGive(uart_mux);
                        }
                    }
                    continue;
                }

                // If not matched, optionally echo or send help
                if (xSemaphoreTake(uart_mux, pdMS_TO_TICKS(500)) == pdTRUE) {
                    uart_write_bytes(EX_UART_NUM, "Unknown command. Supported: THR, RATE, READ POT, RPOT\r\n", 55);
                    xSemaphoreGive(uart_mux);
                }

            }        
        }
    }

}

void app_main(void)
{
    // Create queues
    adc_queue = xQueueCreate(1, sizeof(adc_msg_t));      
    thr_queue = xQueueCreate(5, sizeof(thr_msg_t));        
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    uart_ctrl_queue = xQueueCreate(1, sizeof(bool));
    uart_rate_queue = xQueueCreate(1, sizeof(uint32_t)); 

    if (!adc_queue || !thr_queue || !gpio_evt_queue || !uart_ctrl_queue || !uart_rate_queue) {
        ESP_LOGE(TAG, "Error creando colas");
        // Error handling
        return;
    }

    // Create UART mutex
    uart_mux = xSemaphoreCreateMutex();
    if (uart_mux == NULL) {
        ESP_LOGE(TAG, "Error creating UART mutex");
        return;
    }

    LED_RGB_t *my_led = malloc(sizeof(LED_RGB_t));
    if (my_led == NULL) {
        ESP_LOGE(TAG, "Error allocating memory for LED_RGB_t");
        return;
    }

    // Initialize RGB LED
    *my_led = configure_LED_RGB(GPIO_LED_R, GPIO_LED_G, GPIO_LED_B,
                                 LED_CHANNEL_R, LED_CHANNEL_G, LED_CHANNEL_B,
                                 LED_TIMER, LED_DUTY_RES_BITS, LED_FREQ_HZ);
    // Set initial color to off
    set_LED_RGB_color(my_led, 0, 0, 0);

    // ADC context
    adc_ctx_t *adc_ctx = malloc(sizeof(adc_ctx_t));
    if (adc_ctx == NULL) {
        ESP_LOGE(TAG, "Error allocating memory for ADC context");
        return;
    }
    memset(adc_ctx, 0, sizeof(adc_ctx_t));

    //-------------ADC1 Init---------------//

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_ctx->adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_ctx->adc1_handle, EXAMPLE_ADC1_CHAN0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_ctx->adc1_handle, EXAMPLE_ADC1_CHAN1, &config));

    //-------------ADC1 Calibration Init---------------//
    
    example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc_ctx->cali_pot);
    example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN1, EXAMPLE_ADC_ATTEN, &adc_ctx->cali_term);

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
        .intr_type = GPIO_INTR_NEGEDGE,   // detectar sólo flanco de bajada (presionado)
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .pull_up_en = GPIO_PULLUP_DISABLE,   // sin pull-up interno (tienes pull-up externo)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    //Install GPIO ISR service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_BUTTON, gpio_isr_handler, (void*) GPIO_BUTTON);

    //Create tasks
    xTaskCreate(adc_pot_term_task, "adc_pot_term_task", 2048, adc_ctx, 10, NULL);
    xTaskCreate(write_temp_uart_task, "write_temp_uart_task", 4096, NULL, 11, NULL);
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
    xTaskCreate(controller_task, "controller_task", 4096, my_led, 13, NULL);

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
