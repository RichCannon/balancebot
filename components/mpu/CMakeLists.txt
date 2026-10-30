idf_component_register(
    SRCS "mpu.cpp"
    INCLUDE_DIRS "."
    REQUIRES config esp_driver_gpio esp_driver_spi esp_timer
)
