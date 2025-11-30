idf_component_register(SRCS "sntp_time_sync.c" "thermistor_reader.c" "main.c" "rgb_led.c" "wifi_app.c" "http_server.c" 
                    # PRIV_REQUIRES: Dependencias internas
                    # Eliminado "sntp" (no existe), añadido "esp_netif" (necesario para la red)
                    PRIV_REQUIRES esp_wifi nvs_flash driver esp_http_server esp_timer app_update json esp_driver_uart lwip esp_netif

                    INCLUDE_DIRS "."
                    
                    # REQUIRES: Dependencias que tus .h exponen (thermistor_reader.h y sntp_time_sync.h usan tipos de estas librerías)
                    # Eliminado "esp_sntp" (no existe), mantenemos "lwip"
                    REQUIRES driver esp_timer esp_adc lwip

                    EMBED_FILES webpage/app.css webpage/app.js webpage/favicon.ico webpage/index.html webpage/jquery-3.3.1.min.js
                    )

