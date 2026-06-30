idf_component_register(
    SRCS "main.c" "ledc.c" "nvs_driver.c" "espnow.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_wifi
        esp_driver_gpio
        esp_driver_ledc
)
