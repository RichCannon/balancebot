idf_component_register(
    SRCS "web.cpp"
    INCLUDE_DIRS "."
    REQUIRES config esp_http_server esp_wifi esp_netif esp_event esp_timer nvs_flash
)
