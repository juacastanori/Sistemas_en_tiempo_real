idf_component_register(SRCS "led_rgb_library.c" "uart_events_example_main.c"
                    PRIV_REQUIRES esp_driver_uart
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_timer esp_adc)
