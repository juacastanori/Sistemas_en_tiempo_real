#ifndef FAN_CONTROL_H
#define FAN_CONTROL_H

/**
 * @brief Inicializa el controlador del ventilador con PWM (LEDC).
 * - Configura el timer LEDC
 * - Configura el GPIO de salida PWM
 * - Establece PWM inicial en 0%
 */
void fan_control_init(void);

/**
 * @brief Establece el nivel de PWM del ventilador (0-100%).
 * @param pwm_percent Porcentaje de PWM (0-100)
 */
void fan_control_set_pwm(int pwm_percent);

/**
 * @brief Obtiene el nivel actual de PWM del ventilador.
 * @return int Porcentaje de PWM actual (0-100)
 */
int fan_control_get_pwm(void);

#endif // FAN_CONTROL_H
