#ifndef CONFIG_APP_H
#define CONFIG_APP_H

#include "driver/adc.h"

// ============= CONFIGURACION DE PINES ESP32-C3 =============
// Nota: Ajusta estos pines según tu hardware específico

// --- PIR Sensor ---
#define PIR_SENSOR_GPIO         GPIO_NUM_3      // GPIO para el sensor PIR (detecta presencia)
// Si tu sensor PIR entrega nivel LOW cuando detecta movimiento, define PIR_ACTIVE_LOW a 1
// De lo contrario déjalo en 0 para sensores activos en HIGH
#define PIR_ACTIVE_LOW          1

// --- Fan Control (LEDC/PWM) ---
#define FAN_PWM_GPIO            GPIO_NUM_1      // GPIO para el control PWM del ventilador
#define FAN_LEDC_TIMER          LEDC_TIMER_0    // Timer LEDC para el PWM
#define FAN_LEDC_MODE           LEDC_LOW_SPEED_MODE
#define FAN_LEDC_CHANNEL        LEDC_CHANNEL_0
#define FAN_LEDC_FREQUENCY      5000            // 5 kHz - frecuencia PWM recomendada para motores DC
#define FAN_LEDC_RESOLUTION     LEDC_TIMER_10_BIT // 10-bit resolution (0-1023, pero usaremos 0-100%)

// --- ADC para Termistor ---
#define EXAMPLE_ADC1_CHAN_TERM  ADC_CHANNEL_0 
#define EXAMPLE_ADC_ATTEN       ADC_ATTEN_DB_12 // Atenuación de 12dB
#define ADC_MAX_RAW             4095.0f         // Valor máximo RAW del ADC (asumiendo 12 bits)

// --- Parametros del Sistema y Termistor ---
#define VCC_MV                  3300.0f         // Voltaje de alimentacion del sistema en mV
#define R_FIXED_OHMS            100.0f          // Resistencia fija de la red divisora
#define R0_OHMS                 100.0f          // Resistencia del termistor a T0
#define T0_K                    298.15f         // Temperatura de referencia T0 en Kelvin (25 C)
#define BETA_CONST              3100.0f         // Constante Beta del termistor

// --- Tareas y Timing ---
#define PIR_READ_INTERVAL_MS    100             // Leer PIR cada 100 ms
#define FAN_CONTROL_INTERVAL_MS 500             // Actualizar control del fan cada 500 ms

// PIR debounce (ms) y prioridad de la tarea que procesa eventos ISR
#define PIR_DEBOUNCE_MS         200             // Debounce para evitar rebotes y ráfagas
#define PIR_ISR_TASK_PRIO       5               // Prioridad de la tarea de procesamiento (no muy alta)

#endif // CONFIG_APP_H