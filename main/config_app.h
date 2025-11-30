#ifndef CONFIG_APP_H
#define CONFIG_APP_H

#include "driver/adc.h"

// --- Configuracion de Hardware ---
// Canal ADC1 para el termistor (ADC_CHANNEL_0 corresponde al GPIO_NUM_0 o GPIO_NUM_4 dependiendo del chip, debe verificarse)
#define EXAMPLE_ADC1_CHAN_TERM  ADC_CHANNEL_0 
#define EXAMPLE_ADC_ATTEN       ADC_ATTEN_DB_12 // Atenuación de 12dB
#define ADC_MAX_RAW             4095.0f         // Valor máximo RAW del ADC (asumiendo 12 bits)

// --- Parametros del Sistema y Termistor (extraidos de tu descripción) ---
#define VCC_MV                  3300.0f         // Voltaje de alimentacion del sistema en mV
#define R_FIXED_OHMS            100.0f          // Resistencia fija de la red divisora
#define R0_OHMS                 100.0f          // Resistencia del termistor a T0
#define T0_K                    298.15f         // Temperatura de referencia T0 en Kelvin (25 C)
#define BETA_CONST              3100.0f         // Constante Beta del termistor

#endif // CONFIG_APP_H