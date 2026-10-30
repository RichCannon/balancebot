idf_component_register(
    SRCS "motor.cpp"
    INCLUDE_DIRS "."
    REQUIRES config esp_driver_gpio esp_driver_ledc
)
