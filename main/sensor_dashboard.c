/*
 * ESP32-S3-EYE sensor node with a built-in web dashboard.
 *
 * The board joins a 2.4 GHz Wi-Fi network, starts an HTTP server and serves:
 *   GET /       -> the dashboard page (embedded dashboard.html)
 *   GET /data   -> one JSON sample: {"temp_c":..,"heap_int":..,"heap_psram":..,
 *                                    "fps":..,"luma":..,"uptime_s":..,"rssi":..}
 *
 * The same JSON line is also printed on the USB Serial/JTAG console for debugging.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "esp_private/esp_clk.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_cache.h"
#include "mdns.h"
#include "driver/temperature_sensor.h"
#include "driver/i2c_master.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_dvp.h"
#include "hal/cam_ctlr_types.h"
#include "example_sensor_init.h"
#include "qma6100p.h"

static const char *TAG = "sensor_dash";

/* ------------- ESP32-S3-EYE camera pin mapping ------------- */
#define CAM_SCCB_SCL_IO     (5)
#define CAM_SCCB_SDA_IO     (4)
#define CAM_XCLK_IO         (15)
#define CAM_PCLK_IO         (13)
#define CAM_DE_IO           (7)     /* HREF */
#define CAM_VSYNC_IO        (6)
#define CAM_D0_IO           (11)
#define CAM_D1_IO           (9)
#define CAM_D2_IO           (8)
#define CAM_D3_IO           (10)
#define CAM_D4_IO           (12)
#define CAM_D5_IO           (18)
#define CAM_D6_IO           (17)
#define CAM_D7_IO           (16)
#define CAM_XCLK_FREQ_HZ    (20000000)
#define CAM_DATA_WIDTH      (8)
#define CAM_BYTES_PER_PIXEL (2)     /* RGB565 */

/* OV2640: hardware JPEG, 320x240, 20 MHz input clock.
 * The DVP controller scans for the JPEG EOI marker and reports the real frame
 * length in trans->received_size, so the buffers only need to hold the worst
 * case (h_res * v_res bytes). */
#define CAM_FORMAT_NAME     "DVP_8bit_20Minput_JPEG_320x240_50fps"
#define CAM_JPEG_W          (320)
#define CAM_JPEG_H          (240)
#define CAM_FRAME_MAX       ((size_t)CAM_JPEG_W * CAM_JPEG_H)

/* Embedded dashboard page (see EMBED_TXTFILES in CMakeLists.txt) */
extern const char _binary_dashboard_html_start[];
extern const char _binary_dashboard_html_end[];
#define DASHBOARD_HTML_LEN  ((size_t)(_binary_dashboard_html_end - _binary_dashboard_html_start))

#define WIFI_CONNECTED_BIT BIT0

static EventGroupHandle_t s_wifi_event_group;
static SemaphoreHandle_t  s_json_lock;
static char s_json[768] = "{\"ready\":false}";
static char s_ip[16] = "";
static int  s_rssi = 0;

/* QMA6100P accelerometer, driven by the official Espressif component
 * (espressif/qma6100p) - the same driver model the display_rotation example uses.
 * The OV2640 SCCB and the QMA share the camera's I2C0 bus (GPIO4 / GPIO5). */
static qma6100p_handle_t s_qma = NULL;
static bool s_qma_available = false;
static uint8_t s_qma_id = 0;      /* WHO_AM_I (register 0x00) read at init */
static uint8_t s_qma_addr = 0;    /* 0x12 or 0x13, whichever answered */

/* Two buffers so the HTTP server can send one frame while the camera fills the
 * other one - otherwise /shot would regularly return a half-written JPEG. */
typedef struct {
    uint8_t  *buf[2];
    size_t    len[2];
    size_t    buf_size;
    volatile int       writing;   /* buffer the DMA is currently filling */
    volatile int       ready;     /* buffer holding the last complete frame, -1 = none */
    volatile int       lock;      /* buffer reserved by the HTTP handler, -1 = none */
    volatile uint32_t  frame_count;
} cam_ctx_t;

static cam_ctx_t s_cam_ctx = {0};

/* ------------------------- helpers ------------------------- */

/* Runs in ISR context: hand the DMA whichever buffer is not being sent. */
static bool s_camera_get_new_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    cam_ctx_t *ctx = (cam_ctx_t *)user_data;
    int next = (ctx->writing == 0) ? 1 : 0;
    if (ctx->lock == next) {
        next = 1 - next;
    }
    trans->buffer = ctx->buf[next];
    trans->buflen = ctx->buf_size;
    ctx->writing = next;
    return false;
}

/* The current IDF v6.0.3 DVP driver can occasionally report received_size=0
 * for OV2640 JPEG although the JPEG bytes are present in PSRAM.  Re-sync the
 * complete DMA buffer and recover the frame boundary from the JPEG EOI marker.
 * This runs in ISR context; the driver itself uses esp_cache_msync here too. */
static size_t IRAM_ATTR jpeg_size_from_eoi(const uint8_t *buf, size_t max_len)
{
    if (buf == NULL || max_len < 4 || buf[0] != 0xFF || buf[1] != 0xD8) {
        return 0;
    }
    for (size_t off = max_len - 2; off > 2; off--) {
        if (buf[off] == 0xFF && buf[off + 1] == 0xD9) {
            return off + 2;
        }
    }
    return 0;
}

/* Runs in ISR context: record where the finished frame landed. */
static bool s_camera_get_finished_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    cam_ctx_t *ctx = (cam_ctx_t *)user_data;
    int idx = (trans->buffer == ctx->buf[0]) ? 0 : 1;
    size_t n = trans->received_size;

    if (n == 0) {
        esp_cache_msync(trans->buffer, ctx->buf_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        n = jpeg_size_from_eoi((const uint8_t *)trans->buffer, ctx->buf_size);
    }
    if (n > ctx->buf_size) {
        n = ctx->buf_size;
    }
    ctx->len[idx] = n;
    if (n > 0) {
        ctx->ready = idx;
    }
    ctx->frame_count++;
    return false;
}

static esp_err_t camera_init(esp_cam_ctlr_handle_t *out_handle, i2c_master_bus_handle_t *out_i2c_bus)
{
    esp_cam_ctlr_handle_t cam_handle = NULL;

    esp_cam_ctlr_dvp_pin_config_t pin_cfg = {
        .data_width = CAM_DATA_WIDTH,
        .data_io = {
            CAM_D0_IO, CAM_D1_IO, CAM_D2_IO, CAM_D3_IO,
            CAM_D4_IO, CAM_D5_IO, CAM_D6_IO, CAM_D7_IO,
        },
        .vsync_io = CAM_VSYNC_IO,
        .de_io = CAM_DE_IO,
        .pclk_io = CAM_PCLK_IO,
        .xclk_io = CAM_XCLK_IO,
    };

    esp_cam_ctlr_dvp_config_t dvp_config = {
        .ctlr_id = 0,
        .clk_src = CAM_CLK_SRC_DEFAULT,
        .h_res = CAM_JPEG_W,
        .v_res = CAM_JPEG_H,
        .pic_format_jpeg = 1,               /* hardware JPEG, colour fields ignored */
        .input_data_color_type = CAM_CTLR_COLOR_RGB565,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .conv_std = COLOR_CONV_STD_RGB_YUV_BT601,
        .input_range = COLOR_RANGE_LIMIT,
        .output_range = COLOR_RANGE_LIMIT,
        .dma_burst_size = 64,
        .pin = &pin_cfg,
        .bk_buffer_dis = 1,
        .xclk_freq = CAM_XCLK_FREQ_HZ,
    };

    esp_err_t ret = esp_cam_new_dvp_ctlr(&dvp_config, &cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "dvp controller init failed: %d", ret);
        return ret;
    }

    /* one allocation, split in half - 320*240 is a multiple of the DMA alignment */
    uint8_t *cam_buffer = esp_cam_ctlr_alloc_buffer(cam_handle, CAM_FRAME_MAX * 2,
                                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (cam_buffer == NULL) {
        ESP_LOGE(TAG, "camera buffer alloc failed");
        return ESP_ERR_NO_MEM;
    }
    s_cam_ctx.buf[0] = cam_buffer;
    s_cam_ctx.buf[1] = cam_buffer + CAM_FRAME_MAX;
    s_cam_ctx.buf_size = CAM_FRAME_MAX;
    s_cam_ctx.writing = 1;
    s_cam_ctx.ready = -1;
    s_cam_ctx.lock = -1;

    example_sensor_config_t sensor_cfg = {
        .i2c_port_num = I2C_NUM_0,
        .i2c_sda_io_num = CAM_SCCB_SDA_IO,
        .i2c_scl_io_num = CAM_SCCB_SCL_IO,
        .port = ESP_CAM_SENSOR_DVP,
        .format_name = CAM_FORMAT_NAME,
    };
    example_sensor_handle_t sensor_handle = {.sccb_handle = NULL, .i2c_bus_handle = NULL};
    example_sensor_init(&sensor_cfg, &sensor_handle);   /* returns void */

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = s_camera_get_new_trans,
        .on_trans_finished = s_camera_get_finished_trans,
    };
    ret = esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, &s_cam_ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "camera callbacks register failed: %d", ret);
        return ret;
    }

    ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam_handle));
    ret = esp_cam_ctlr_start(cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "camera start failed: %d", ret);
        return ret;
    }

    *out_handle = cam_handle;
    *out_i2c_bus = sensor_handle.i2c_bus_handle;
    return ESP_OK;
}

/* ----------------- QMA6100P / QMA7981 (espressif/qma6100p) ----------------- */

/* Espressif only publishes a qma6100p component, but QMA7981 (the part named on
 * some S3-EYE schematics) is the same QST family with the same register map -
 * the two are told apart purely by WHO_AM_I in register 0x00. */
#define QMA7981_WHO_AM_I_VAL (0xE7)

static const char *qma_chip_name(uint8_t id)
{
    if (id == QMA6100P_WHO_AM_I_VAL) return "QMA6100P";   /* 0x90 */
    if (id == QMA7981_WHO_AM_I_VAL)  return "QMA7981";    /* 0xE7 */
    return "Unknown";
}

/* Probe both I2C addresses (AD0 selects 0x12 / 0x13) and read WHO_AM_I.
 * Note: qma6100p_create() only adds an I2C device and never touches the bus,
 * so qma6100p_get_deviceid() is what actually proves a chip is answering. */
static void qma_init(i2c_master_bus_handle_t bus)
{
    const uint8_t probe_addrs[] = {QMA6100P_I2C_ADDRESS, QMA6100P_I2C_ADDRESS_1};

    for (size_t i = 0; i < sizeof(probe_addrs) / sizeof(probe_addrs[0]); i++) {
        qma6100p_handle_t handle = NULL;
        esp_err_t ret = qma6100p_create(bus, probe_addrs[i], &handle);
        if (ret != ESP_OK || handle == NULL) {
            continue;
        }

        uint8_t device_id = 0;
        if (qma6100p_get_deviceid(handle, &device_id) == ESP_OK) {
            ESP_LOGI(TAG, "IMU %s detected: I2C 0x%02x, WHO_AM_I(0x00)=0x%02x",
                     qma_chip_name(device_id), probe_addrs[i], device_id);
            ret = qma6100p_wake_up(handle);
            if (ret == ESP_OK) {
                ret = qma6100p_config(handle, ACCE_FS_4G);
            }
            if (ret == ESP_OK) {
                s_qma = handle;
                s_qma_available = true;
                s_qma_id = device_id;
                s_qma_addr = probe_addrs[i];
                ESP_LOGI(TAG, "accel ready: %s at 0x%02x, +/-4g",
                         qma_chip_name(device_id), probe_addrs[i]);
                return;
            }
            ESP_LOGW(TAG, "%s init failed at 0x%02x: %s",
                     qma_chip_name(device_id), probe_addrs[i], esp_err_to_name(ret));
        } else {
            ESP_LOGD(TAG, "no chip answering at 0x%02x", probe_addrs[i]);
        }
        qma6100p_delete(handle);
    }
    ESP_LOGW(TAG, "no QST accelerometer detected; dashboard stays up without it");
}

/* The component's qma6100p_get_raw_acce() shifts the 16-bit register pair right
 * by 2 ("/4"), which is one bit too many for the QST part fitted here: measured
 * at rest, the +/-2g setting yields ~2023 counts on the gravity axis and the
 * +/-4g setting ~1041, i.e. exactly half of the 4096 / 2048 LSB/g the driver's
 * sensitivity table expects.  Undoing that extra shift (x2) puts |a| at ~1.0 g
 * when the board is still, which is the physical sanity check. */
#define QMA_RAW_SCALE_CORRECTION (2.0f)

static bool qma_read_accel(float *x_g, float *y_g, float *z_g)
{
    if (!s_qma_available || s_qma == NULL) {
        return false;
    }
    float sensitivity = 0;
    qma6100p_raw_acce_value_t raw = {0};
    if (qma6100p_get_acce_sensitivity(s_qma, &sensitivity) != ESP_OK || sensitivity <= 0) {
        return false;
    }
    if (qma6100p_get_raw_acce(s_qma, &raw) != ESP_OK) {
        return false;
    }
    *x_g = raw.raw_acce_x * QMA_RAW_SCALE_CORRECTION / sensitivity;
    *y_g = raw.raw_acce_y * QMA_RAW_SCALE_CORRECTION / sensitivity;
    *z_g = raw.raw_acce_z * QMA_RAW_SCALE_CORRECTION / sensitivity;
    /* ~0 on every axis is not a valid gravity reading - treat as unavailable. */
    if (*x_g == 0.0f && *y_g == 0.0f && *z_g == 0.0f) {
        return false;
    }
    return true;
}

/* ------------------------- Wi-Fi ------------------------- */

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi started, connecting to %s ...", CONFIG_SENSOR_DASH_WIFI_SSID);
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "got IP: %s", s_ip);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    /* ESP32-S3 is 2.4 GHz only - the AP must broadcast on 2.4 GHz */
    strncpy((char *)wifi_config.sta.ssid, CONFIG_SENSOR_DASH_WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, CONFIG_SENSOR_DASH_WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;   /* accept WPA/WPA2/WPA3 */

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "waiting for Wi-Fi (%s)...", CONFIG_SENSOR_DASH_WIFI_SSID);
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

/* ---------------------- HTTP handlers ---------------------- */

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    esp_err_t e = httpd_resp_send(req, _binary_dashboard_html_start, DASHBOARD_HTML_LEN);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "page send failed: %s", esp_err_to_name(e));
    }
    return e;
}

/* GET /shot -> the most recent JPEG frame */
static esp_err_t shot_handler(httpd_req_t *req)
{
    int idx = s_cam_ctx.ready;
    size_t len = (idx >= 0) ? s_cam_ctx.len[idx] : 0;
    if (idx < 0 || len < 512) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "no frame yet\r\n");
        return ESP_FAIL;
    }

    /* keep the DMA out of this buffer while we stream it out */
    s_cam_ctx.lock = idx;
    vTaskDelay(pdMS_TO_TICKS(40));      /* ~2 frames @ 50 fps */

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t e = httpd_resp_send(req, (const char *)s_cam_ctx.buf[idx], len);
    s_cam_ctx.lock = -1;

    if (e != ESP_OK) {
        ESP_LOGW(TAG, "shot send failed: %s", esp_err_to_name(e));
    }
    return e;
}

static esp_err_t not_found_handler(httpd_req_t *req, httpd_err_code_t err)
{
    ESP_LOGW(TAG, "no handler for '%s' (err %d)", req->uri, (int)err);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "404 not found");
    return ESP_FAIL;
}

static void start_mdns(void)
{
    if (mdns_init() != ESP_OK) {
        ESP_LOGW(TAG, "mDNS init failed");
        return;
    }
    mdns_hostname_set("s3eye");
    mdns_instance_name_set("ESP32-S3-EYE sensor dashboard");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    ESP_LOGI(TAG, "mDNS up -> http://s3eye.local/");
}

static esp_err_t data_handler(httpd_req_t *req)
{
    char buf[sizeof(s_json)];
    xSemaphoreTake(s_json_lock, portMAX_DELAY);
    memcpy(buf, s_json, sizeof(s_json));
    xSemaphoreGive(s_json_lock);

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.stack_size = 8192;

    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t index_uri = {.uri = "/", .method = HTTP_GET, .handler = index_handler};
    httpd_uri_t data_uri  = {.uri = "/data", .method = HTTP_GET, .handler = data_handler};
    httpd_uri_t shot_uri  = {.uri = "/shot", .method = HTTP_GET, .handler = shot_handler};

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &index_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &data_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &shot_uri));
    httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, not_found_handler);

    ESP_LOGI(TAG, "web dashboard ready -> http://%s/  (also http://s3eye.local/)", s_ip);
}

/* ------------------------- main ------------------------- */

void app_main(void)
{
    s_json_lock = xSemaphoreCreateMutex();

    /* NVS is required by the Wi-Fi driver */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* --- on-chip temperature sensor --- */
    temperature_sensor_handle_t temp_sensor = NULL;
    /* the narrow room-temperature range selects the calibrated high-accuracy band */
    temperature_sensor_config_t temp_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    bool temp_ok = false;
    if (temperature_sensor_install(&temp_cfg, &temp_sensor) == ESP_OK &&
        temperature_sensor_enable(temp_sensor) == ESP_OK) {
        temp_ok = true;
    } else {
        ESP_LOGW(TAG, "temperature sensor unavailable");
    }

    /* --- camera + shared I2C bus --- */
    esp_cam_ctlr_handle_t cam_handle = NULL;
    i2c_master_bus_handle_t shared_i2c_bus = NULL;
    bool cam_ok = (camera_init(&cam_handle, &shared_i2c_bus) == ESP_OK);
    ESP_LOGI(TAG, "camera %s", cam_ok ? "ready" : "unavailable - fps will be 0");
    if (shared_i2c_bus != NULL) {
        qma_init(shared_i2c_bus);
    }
    ESP_LOGI(TAG, "free PSRAM: %u bytes", (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    ESP_LOGI(TAG, "embedded dashboard.html is %u bytes", (unsigned)DASHBOARD_HTML_LEN);

    /* --- Wi-Fi + web server --- */
    wifi_init_sta();
    start_webserver();
    start_mdns();

    const TickType_t interval = pdMS_TO_TICKS(CONFIG_SENSOR_DASH_INTERVAL_MS);
    const float interval_s = CONFIG_SENSOR_DASH_INTERVAL_MS / 1000.0f;

    /* discard the first window: it accumulates everything captured during boot */
    vTaskDelay(interval);
    s_cam_ctx.frame_count = 0;

    while (1) {
        vTaskDelay(interval);

        float temp_c = 0;
        if (temp_ok) {
            esp_err_t terr = temperature_sensor_get_celsius(temp_sensor, &temp_c);
            if (terr != ESP_OK) {
                static int warned = 0;
                if (warned++ < 3) {
                    ESP_LOGW(TAG, "temperature read failed: 0x%x (%s)", terr, esp_err_to_name(terr));
                }
                temp_c = 0;
            }
        }

        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            s_rssi = ap_info.rssi;
        }

        uint32_t frames = s_cam_ctx.frame_count;
        s_cam_ctx.frame_count = 0;
        float fps = cam_ok ? (frames / interval_s) : 0.0f;

        float accel_x_g = 0, accel_y_g = 0, accel_z_g = 0;
        bool accel_ok = qma_read_accel(&accel_x_g, &accel_y_g, &accel_z_g);
        float accel_mag_g = sqrtf(accel_x_g * accel_x_g + accel_y_g * accel_y_g + accel_z_g * accel_z_g);
        if (s_qma_available && !accel_ok) {
            static int accel_warned = 0;
            if (accel_warned++ < 3) {
                ESP_LOGW(TAG, "QMA6100P read failed");
            }
        }

        xSemaphoreTake(s_json_lock, portMAX_DELAY);
        int shot_idx = s_cam_ctx.ready;
        snprintf(s_json, sizeof(s_json),
                 "{\"temp_c\":%.1f,\"heap_int\":%u,\"heap_psram\":%u,\"fps\":%.1f,"
                 "\"jpeg\":%lu,\"accel_ok\":%s,\"accel_x_g\":%.3f,\"accel_y_g\":%.3f,\"accel_z_g\":%.3f,"
                 "\"accel_mag_g\":%.3f,\"accel_range_g\":4,"
                 "\"accel_id\":\"0x%02X\",\"accel_chip\":\"%s\","
                 "\"uptime_s\":%lu,\"cpu_mhz\":%lu,\"rssi\":%d,\"ip\":\"%s\"}",
                 temp_c,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),   /* internal SRAM only */
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                 fps,
                 (unsigned long)(shot_idx >= 0 ? s_cam_ctx.len[shot_idx] : 0),
                 accel_ok ? "true" : "false",
                 accel_x_g, accel_y_g, accel_z_g, accel_mag_g,
                 s_qma_id, qma_chip_name(s_qma_id),
                 (unsigned long)(esp_timer_get_time() / 1000000ULL),
                 (unsigned long)(esp_clk_cpu_freq() / 1000000UL),
                 s_rssi,
                 s_ip);
        xSemaphoreGive(s_json_lock);

        /* same sample on the serial console, handy for debugging */
        printf("%s\n", s_json);
        fflush(stdout);
    }
}
