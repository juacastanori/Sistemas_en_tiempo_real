idf_component_register(SRCS "led_rgb_library.c" "oneshot_read_main.c"
                    PRIV_REQUIRES esp_adc
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_adc esp_timer)
