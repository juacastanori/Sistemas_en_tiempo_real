/**
 * ============================================================================
 * RESUMEN DE CAMBIOS IMPLEMENTADOS - PIR & FAN CONTROL
 * ============================================================================
 * 
 * BACKEND (C/ESP32):
 * ==================
 * 
 * ✓ config_app.h
 *   - Pines PIR y FAN configurables
 *   - LEDC settings (timer, channel, frequency)
 *   - Intervalos de lectura
 * 
 * ✓ pir_sensor.h/c (NUEVO)
 *   - GPIO polling cada 100ms
 *   - Variable global g_pir_state accesible desde http_server.c
 *   - Pull-down habilitado (LOW = sin presencia)
 * 
 * ✓ fan_control.h/c (NUEVO)
 *   - PWM control via LEDC (5 kHz, 10-bit)
 *   - fan_control_set_pwm(0-100%)
 *   - fan_control_get_pwm()
 * 
 * ✓ main.c
 *   - thermistor_init()
 *   - pir_sensor_init()
 *   - fan_control_init()
 *   Todo antes de wifi_app_start()
 * 
 * ✓ http_server.c (IMPORTANTE)
 *   - fan_control_task() [nueva tarea]
 *     * Ejecuta cada 500ms
 *     * Lee: temperatura, PIR, hora actual
 *     * Calcula PWM según modo (MANUAL, AUTO, PROGRAMADO)
 *     * Aplica PWM al fan
 *     * Actualiza g_current_pwm
 *   
 *   - GET /getProgrammed [nuevo endpoint]
 *     * Retorna registros actuales en JSON
 *   
 *   - GET /systemState [mejorado]
 *     * Agrega "activeRegister" al JSON
 *     * Detecta qué registro programado está activo
 *   
 *   - POST /setMode, /saveManual, /saveAuto, /saveProgrammed
 *     * Ya existían, ahora integrados con la lógica de fan_control_task
 * 
 * ✓ CMakeLists.txt
 *   - Agregados pir_sensor.c y fan_control.c
 * 
 * 
 * FRONTEND (HTML/CSS/JS):
 * =======================
 * 
 * ✓ index.html
 *   - YA TIENE: PIR card, PWM card con barra de progreso
 *   - YA TIENE: 3 registros programados con hora y temp
 *   - No requería cambios
 * 
 * ✓ app.css
 *   - YA TIENE: Estilos para PIR, PWM, registros
 *   - Colores, gradientes, animaciones
 *   - Responsive design
 *   - No requería cambios
 * 
 * ✓ app.js
 *   - CONFIG.API_ENDPOINTS.GET_PROGRAMMED (agregado)
 *   
 *   - loadSavedRegisters() [nueva función]
 *     * Obtiene registros guardados en ESP32 al cargar página
 *     * Los carga en los campos HTML correspondientes
 *   
 *   - updatePWMDisplay() [mejorada]
 *     * Actualiza texto: g_current_pwm
 *     * Actualiza barra visual: pwmFill width %
 *   
 *   - updateUIWithSystemState() [existente]
 *     * Ahora procesa "activeRegister" del JSON
 *     * Llama a updateActiveRegisterDisplay()
 * 
 * 
 * FLUJO DE EJECUCIÓN:
 * ====================
 * 
 * 1. STARTUP (app_main)
 *    └─ thermistor_init()
 *    └─ pir_sensor_init()
 *    └─ fan_control_init()
 *    └─ wifi_app_start()
 *       └─ http_server_start()
 *          └─ Crea fan_control_task()
 * 
 * 2. RUNTIME - Tarea fan_control_task (cada 500ms)
 *    ├─ Lee temperatura (cola del termistor)
 *    ├─ Lee PIR (gpio_get_level)
 *    ├─ Obtiene hora actual (time/localtime_r)
 *    ├─ Calcula PWM según modo:
 *    │  ├─ MANUAL: g_current_pwm directamente
 *    │  ├─ AUTO: interpolación si hay PIR
 *    │  └─ PROGRAMADO: busca registro activo, luego interpolación
 *    ├─ Aplica: fan_control_set_pwm(calculated_pwm)
 *    └─ Actualiza: g_current_pwm = calculated_pwm
 * 
 * 3. FRONTEND - Actualización (cada 2s)
 *    ├─ GET /systemState
 *    ├─ Procesa JSON (temp, pir, mode, pwm, activeRegister)
 *    └─ Actualiza UI
 * 
 * 
 * VARIABLES GLOBALES:
 * ===================
 * 
 * int g_current_mode;          // 0=MANUAL, 1=AUTO, 2=PROGRAMADO
 * int g_current_pwm;           // 0-100 (actualizado por fan_control_task)
 * float g_auto_t_min;          // Temperatura mínima (modo AUTO)
 * float g_auto_t_max;          // Temperatura máxima (modo AUTO)
 * int g_pir_state;             // 1=detectado, 0=no detectado
 * scheduled_register_t g_registers[3];  // Registros programados
 * 
 * 
 * PUNTOS CRÍTICOS A VERIFICAR:
 * =============================
 * 
 * ⚠️  PINES GPIO correctos para tu hardware (config_app.h)
 * ⚠️  SNTP sincronizado para modo PROGRAMADO (hora debe ser correcta)
 * ⚠️  PIR conectado correctamente (HIGH = presencia)
 * ⚠️  FAN PWM funciona con tu motor DC
 * ⚠️  NVS implementado si necesitas persistencia en modo programado
 * 
 * 
 * TESTING RECOMENDADO:
 * ====================
 * 
 * 1. Compilar y flashear el firmware
 * 2. Monitor serial para ver logs (ESP_LOGI)
 * 3. Acceder a http://ESP32_IP/
 * 4. Verificar:
 *    - Temperatura se actualiza correctamente
 *    - PIR cambia cuando hay movimiento
 *    - PWM se aplica correctamente en modo MANUAL
 *    - Interpolación funciona en modo AUTOMÁTICO
 *    - Registros se guardan y cargan en modo PROGRAMADO
 * 
 * ============================================================================
 */

// Este es un archivo de documentación, no contiene código executable.
// Ver IMPLEMENTACION_PIR_FAN.md para detalles completos.
