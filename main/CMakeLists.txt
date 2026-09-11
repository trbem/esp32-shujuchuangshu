idf_component_register(
    SRCS "sensor_dashboard.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES "dashboard.html"
    REQUIRES
        esp_driver_cam
        esp_driver_i2c
        esp_driver_ledc
        esp_driver_tsens
        esp_psram
        esp_timer
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
        mdns
        nvs_flash
)
