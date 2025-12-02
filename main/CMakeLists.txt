idf_component_register(SRCS "sntp_time_sync.c" "thermistor_reader.c" "pir_sensor.c" "fan_control.c" "main.c" "wifi_app.c" "http_server.c" 
                    PRIV_REQUIRES esp_wifi nvs_flash driver esp_http_server esp_timer app_update json mqtt esp_driver_uart lwip esp_netif

                    INCLUDE_DIRS "."
                    
                    REQUIRES driver esp_timer esp_adc lwip

                    EMBED_FILES webpage/app.css webpage/app.js webpage/favicon.ico webpage/index.html webpage/jquery-3.3.1.min.js
                    )
