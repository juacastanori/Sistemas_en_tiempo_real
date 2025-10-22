// config_term_pot.h
#ifndef CONFIG_TERM_POT_H
#define CONFIG_TERM_POT_H

#include <stdint.h>

/* ADC resolution / limits for Potentiometer */
static const int ADC_MAX_RAW       = (1 << 12) - 1; // 12 bits => 4095
static const int RAW_MIN_VALUE    = 150;
static const int RAW_MAX_VALUE    = ADC_MAX_RAW;

/* System voltage and thermistor parameters */
static const float VCC_MILLIVOLTS = 3300.0f;
static const float R_FIXED_OHMS   = 100.0f;
static const float R0_OHMS        = 100.0f;
static const float T0_KELVIN      = 298.15f;
static const float BETA_VALUE     = 3100.0f;

/* Alias names used by main code */
#define RAW_MIN RAW_MIN_VALUE
#define RAW_MAX RAW_MAX_VALUE
#define VCC_MV  VCC_MILLIVOLTS
#define R_FIXED_OHMS R_FIXED_OHMS
#define R0_OHMS R0_OHMS
#define T0_K T0_KELVIN
#define BETA_CONST BETA_VALUE

/* Threshold struct */
typedef struct {
    float min;
    float max;
} threshold_t;

/* Default threshold values */
static const float THR_RED_MIN_V   = 60.0f;
static const float THR_RED_MAX_V   = 100.0f;
static const float THR_GREEN_MIN_V = 10.0f;
static const float THR_GREEN_MAX_V = 50.0f;
static const float THR_BLUE_MIN_V  = 40.0f;
static const float THR_BLUE_MAX_V  = 60.0f;

/* Instances (static = local to each compilation unit) */
static threshold_t thr_red   = { THR_RED_MIN_V,   THR_RED_MAX_V };
static threshold_t thr_green = { THR_GREEN_MIN_V, THR_GREEN_MAX_V };
static threshold_t thr_blue  = { THR_BLUE_MIN_V,  THR_BLUE_MAX_V };

#endif // CONFIG_TERM_POT_H
