idf_component_register(
    SRCS "balance.cpp"
    INCLUDE_DIRS "."
    REQUIRES config mpu motor esp_timer
)
