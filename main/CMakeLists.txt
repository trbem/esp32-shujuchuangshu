idf_component_register(
    SRCS "sensor_dashboard.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES "dashboard.html"
    REQUIRES
        esp_driver_cam
        esp_driver_i2c
        esp_driver_ledc
        esp_driver_spi
        esp_driver_tsens
        esp_lcd
        esp_psram
        esp_timer
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
        esp_http_client
        mbedtls
        esp_partition
        mdns
        nvs_flash
)
