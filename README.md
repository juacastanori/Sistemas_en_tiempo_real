# Sistema de Control de Ventilación (ESP32)

Proyecto de control de ventilación basado en ESP32 (ESP32-C3/C6 compatible) desarrollado como trabajo de curso universitario.

Resumen
- Plataforma: ESP-IDF (FreeRTOS, C)
- Función: Control de ventilador con modos Manual / Automático / Programado, sensor PIR, sincronización SNTP, interfaz web embebida y OTA.
- Arquitectura: tarea única de control (`system_control_task`) que posee el estado, comunicación entre tareas mediante colas (no variables globales mutables), y persistencia en NVS.

Estructura del repositorio (resumen)
- `CMakeLists.txt`, `sdkconfig*`, `build/` - archivos de build ESP-IDF
- `main/` - código fuente principal
  - `system_state.c/h` - lógica de control y persistencia
  - `http_server.c/h` - endpoints HTTP y servir la web embebida
  - `wifi_app.c/h` - gestión WiFi, reconexión y carga de credenciales
  - `sntp_time_sync.c/h` - sincronización de hora y reinicio SNTP
  - `nvs_config.c/h` - abstracción de persistencia NVS (keys: `manual_pwm`, `auto_tmin`, `auto_tmax`, `system_mode`, `wifi_ssid`, `wifi_password`, `reg_0`/`reg_1`/`reg_2`)
  - `fan_control.c`, `pir_sensor.c`, `thermistor_reader.c` - control hardware
  - `webpage/` - `index.html`, `app.js`, `app.css`, `jquery-3.3.1.min.js` (interfaz web embebida)

Principales características
- Modos de operación:
  - Manual: PWM fijo (0-100%)
  - Automático: ventilador solo se activa si el PIR detecta presencia; PWM proporcional a la temperatura entre `T_min` y `T_max`
  - Programado: hasta 3 registros con ventana horaria y rangos de temperatura
- Persistencia en NVS: cambios de configuración (modo, PWM manual, temps automáticas, registros programados, credenciales WiFi) se guardan en flash
- Interfaz web embebida: página UI que consulta `/systemState`, cambia modo `/setMode`, guarda configuraciones (`/saveManual`, `/saveAuto`, `/saveProgrammed`), y `/saveWiFi` para credenciales
- SNTP: sincroniza hora y reintenta/reinicia SNTP cuando el dispositivo vuelve a obtener IP
- OTA: actualización de firmware vía endpoint y web UI

Endpoints HTTP principales
- `GET /systemState` → JSON con `temperature`, `pir`, `mode`, `pwm`, `tMin`, `tMax`, `activeRegister`.
- `POST /setMode` → body `{ "mode": 0|1|2 }` envía `config_update_t` a la cola para que la tarea de control aplique y persista el cambio.
- `POST /saveManual`, `/saveAuto`, `/saveProgrammed` → guardan parámetros correspondientes en NVS vía colas.
- `POST /saveWiFi` → guarda `ssid` y `password` en NVS y desencadena recarga de credenciales en el módulo WiFi.
- `GET /time.json` → devuelve hora sincronizada por SNTP (o `N/A` si no sincronizada).

Notas de diseño importantes
- Patrón escritor único: `system_control_task` posee todo el estado mutable y es la única tarea que escribe en `system_state_queue` y en hardware (ventilador). Las actualizaciones desde la web se envían como `config_update_t` a `config_update_queue`.
- Evitamos variables globales mutables accedidas concurrentemente; usamos colas (`queues.c/h`) para comunicación.
- Persistencia: `nvs_config.c/h` centraliza las operaciones NVS; se deben usar sus funciones para leer/guardar claves.

Requisitos para desarrollar/compilar
- ESP-IDF versión compatible (usar la versión con la que se desarrolló el proyecto).
- Toolchain y entorno ESP-IDF correctamente configurado.

Comandos típicos (Windows PowerShell)
```powershell
cd "c:\Users\jeron\Downloads\Universidad\2025-2\Real_time_system\project\http_1"
# Configurar environment si no está hecho (ej. idf export)
idf.py set-target esp32c3
# Compilar
idf.py build
# Flashear (ajusta el puerto COM)
idf.py -p COM3 flash
# Monitor serial
idf.py -p COM3 monitor
```

Probando comportamiento clave (modo persistente)
1. Abrir la web embebida (`http://<IP_DEL_DISPOSITIVO>/`) y cambiar el modo a "AUTOMÁTICO".
2. Asegurarse de que la petición `/setMode` responde `{"status":"ok"}` y revisar logs del ESP32 que muestren `Mode update sent to queue` y `nvs_config_save_mode` llamada.
3. Reiniciar el dispositivo (por hardware o software). Al arrancar, `system_control_task` leerá `system_mode` desde NVS y aplicará el modo guardado; la UI consulta `/systemState` periódicamente y mostrará el modo correcto.

Cómo depurar problemas de UI (reset de la página)
- La página recarga su estado del dispositivo periódicamente consultando `/systemState` cada 2 segundos. Si al pulsar "Reset" del navegador la página vuelve al modo por defecto visualmente, asegúrate de que el endpoint `/systemState` retorna el `mode` correcto en su JSON. Si el servidor devuelve el modo guardado y la UI sigue mostrando otro, revisar `app.js` (función `updateUIWithSystemState`) y confirmar que no haya lógica local que sobrescriba `STATE.currentMode`.

Puntos de verificación rápida
- Verificar que `nvs_config_save_mode()` y `nvs_config_load_mode()` existen en `nvs_config.c/h`.
- `system_state.c` ahora carga el `system_mode` al iniciar y guarda el modo cuando se aplica un `config_update_t`.
- `app.js` hace polling de `/systemState` y actualiza la UI; no debe forzar modo por defecto si el servidor devuelve otro valor.

Contribuir
- Para cambios grandes, abrir un branch y enviar un PR con descripción clara de la motivación y pruebas realizadas.

Licencia
- Añadir licencia conforme a las necesidades del curso/autor (no incluida en este repositorio). Si quieres, te ayudo a añadir una `LICENSE` (MIT, Apache-2.0, etc.).

Contacto
- Autor: Jeron (repositorio local). Para integración en GitHub, puedo:
  - crear un `.gitignore` recomendado,
  - crear un `LICENSE` y
  - preparar un `README` extendido con capturas o diagramas si lo deseas.

---
Archivo creado automáticamente por la herramienta de desarrollo; revisa y edita según prefieras antes de publicar en GitHub.
