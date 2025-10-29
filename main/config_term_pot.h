#ifndef CONFIG_TERM_POT_H
#define CONFIG_TERM_POT_H

#include <stdint.h>

/* ADC resolution / limits for Potentiometer */
#define ADC_MAX_RAW     ((1 << 12) - 1) // 12 bits => 4095
#define RAW_MIN         150
#define RAW_MAX         ADC_MAX_RAW

/* System voltage and thermistor parameters */
#define VCC_MV          3300.0f
#define R_FIXED_OHMS    100.0f
#define R0_OHMS         100.0f
#define T0_K            298.15f
#define BETA_CONST      3100.0f

/* Threshold struct */
typedef struct {
    float min;
    float max;
} threshold_t;

/* Default threshold values (usados para inicializar en el .c) */
#define THR_RED_MIN_V    60.0f
#define THR_RED_MAX_V    100.0f
#define THR_GREEN_MIN_V  10.0f
#define THR_GREEN_MAX_V  50.0f
#define THR_BLUE_MIN_V   40.0f
#define THR_BLUE_MAX_V   60.0f

#endif // CONFIG_TERM_POT_H
