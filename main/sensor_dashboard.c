/*
 * ESP32-S3-EYE sensor node with a built-in web dashboard.
 *
 * The board joins a 2.4 GHz Wi-Fi network, starts an HTTP server and serves:
 *   GET /       -> the dashboard page (embedded dashboard.html)
 *   GET /data   -> one JSON sample: {"temp_c":..,"heap_int":..,"heap_psram":..,
 *                                    "fps":..,"accel_x_g":..,"uptime_s":..,"rssi":..}
 *   GET /shot   -> the current viewfinder frame as an uncompressed 24-bit BMP
 *
 * The OV2640 is captured as raw RGB565 at 240x240 and pushed straight to the
 * on-board ST7789, so the panel shows a live 25 fps viewfinder.  /shot hands the
 * same frame to the browser as a BMP, which every <img> renders natively.
 *
 * Samples are also journalled to the "telemetry" Flash partition and uploaded to
 * the collector configured in menuconfig; the same JSON line is printed on the
 * USB Serial/JTAG console for debugging.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <sys/time.h>
#include <inttypes.h>
#include <stddef.h>
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
#include "nvs.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_partition.h"
#include "esp_netif_sntp.h"
#include "esp_cache.h"
#include "mdns.h"
#include "driver/temperature_sensor.h"
#include "driver/i2c_master.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_dvp.h"
#include "hal/cam_ctlr_types.h"
#include "example_sensor_init.h"
#include "qma6100p.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

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

/* ------------- ESP32-S3-EYE ST7789 LCD (SPI3) ------------- */
#define LCD_SPI_HOST            (SPI3_HOST)
#define LCD_SPI_MOSI_IO         (47)
#define LCD_SPI_CLK_IO          (21)
#define LCD_SPI_CS_IO           (44)
#define LCD_DC_IO               (43)
#define LCD_RST_IO              (GPIO_NUM_NC)   /* the panel is reset by the RC on-board */
#define LCD_BACKLIGHT_IO        (48)
#define LCD_PIXEL_CLOCK_HZ      (80 * 1000 * 1000)
#define LCD_CMD_BITS            (8)
#define LCD_PARAM_BITS          (8)
/* The camera peripheral drives XCLK from its own clock divider, so the backlight
 * is free to use LEDC timer 1 without fighting the DVP controller for timer 0. */
#define LCD_BL_LEDC_TIMER       (LEDC_TIMER_1)
#define LCD_BL_LEDC_CHANNEL     (LEDC_CHANNEL_0)

/* ------------- capture format ------------- */
/* The DVP controller captures raw RGB565 at the panel's native 240x240 so the
 * LCD gets a smooth 25 fps viewfinder.  ESP32-S3 has no JPEG encoder, therefore
 * local /shot and remotely requested photos use a BMP made from that same frame. */
#define CAM_FORMAT_NAME     "DVP_8bit_20Minput_RGB565_240x240_25fps"
#define CAM_LCD_W           (240)
#define CAM_LCD_H           (240)
#define CAM_BYTES_PER_PIXEL (2)     /* RGB565 */
#define CAM_FRAME_MAX       ((size_t)CAM_LCD_W * CAM_LCD_H * CAM_BYTES_PER_PIXEL)

/* Largest single SPI transfer the LCD bus will be asked for.  It also sets the
 * size of the internal-RAM bounce buffer the SPI driver needs per transfer, so
 * it is intentionally far smaller than a whole frame - see lcd_init(). */
#define LCD_SPI_CHUNK_BYTES (4 * 1024)
/* How many chunk transfers may be in flight at once.  Each one holds its own
 * internal-RAM bounce buffer, so the peak internal-RAM cost of the viewfinder
 * is roughly LCD_SPI_CHUNK_BYTES * LCD_SPI_QUEUE_DEPTH - keep that well under
 * 32 KB, because Wi-Fi already holds a large share of internal DMA RAM and the
 * bounce buffer allocation is what fails first when it runs out. */
#define LCD_SPI_QUEUE_DEPTH (4)

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

/* The Flash journal is deliberately independent from NVS: each record has a
 * CRC and an acknowledged bit, so a reset while writing or uploading can be
 * recovered by scanning the telemetry partition. */
#define TLM_MAGIC              0x544C4D31UL /* TLM1 */
#define TLM_VERSION            1
#define TLM_STATE_PENDING      0xFEU
#define TLM_STATE_ACKED        0xFCU
#define TLM_RECORD_SIZE        128U
#define TLM_BATCH_MAX          20U
#define TLM_SECTOR_SIZE        4096U

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t state;
    uint8_t version;
    uint16_t reserved0;
    uint64_t boot_id;
    uint32_t sequence;
    int64_t captured_at_ms;
    float temp_c;
    uint32_t heap_int;
    uint32_t heap_psram;
    float fps;
    uint32_t jpeg;
    uint32_t accel_ok;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float accel_mag_g;
    uint32_t accel_range_g;
    uint8_t accel_id;
    uint8_t reserved1[3];
    uint32_t uptime_s;
    uint32_t cpu_mhz;
    int32_t rssi;
    char ip[16];
    uint32_t crc32;
    /* Carved out of what used to be padding: no field moves, so records written
     * by an earlier build still parse - their zeroed padding reads back as
     * lcd_frames = 0, which is exactly right for a build without the LCD. */
    uint32_t lcd_frames;
    float lcd_fps;
    uint32_t lcd_err;
    uint8_t padding[TLM_RECORD_SIZE - 120U];
} telemetry_record_t;

typedef struct {
    float temp_c;
    uint32_t heap_int, heap_psram;
    float fps;
    uint32_t jpeg;
    bool accel_ok;
    float accel_x_g, accel_y_g, accel_z_g, accel_mag_g;
    uint8_t accel_id;
    uint32_t uptime_s, cpu_mhz;
    int32_t rssi;
    int64_t captured_at_ms;
    uint32_t lcd_frames;
    float lcd_fps;
    uint32_t lcd_err;
    char ip[16];
} telemetry_snapshot_t;

static const esp_partition_t *s_tlm_partition;
static SemaphoreHandle_t s_tlm_lock;
static telemetry_snapshot_t s_latest_sample;
static uint64_t s_boot_id;
static uint32_t s_sample_sequence;
static size_t s_queue_write_slot;
static size_t s_queue_slots;
static size_t s_queue_depth;
static uint32_t s_queue_dropped;
static bool s_upload_ok;
static int64_t s_last_upload_ms;
static uint32_t s_live_sequence;
static bool s_live_upload_ok;
static int64_t s_last_live_upload_ms;
static char s_active_task_id[37];
static bool s_task_result_pending;
static telemetry_snapshot_t s_task_result_sample;
static char s_photo_task_id[37];
static bool s_photo_upload_pending;
static int64_t s_photo_captured_at_ms;

/* QMA6100P accelerometer, driven by the official Espressif component
 * (espressif/qma6100p) - the same driver model the display_rotation example uses.
 * The OV2640 SCCB and the QMA share the camera's I2C0 bus (GPIO4 / GPIO5). */
static qma6100p_handle_t s_qma = NULL;
static bool s_qma_available = false;
static uint8_t s_qma_id = 0;      /* WHO_AM_I (register 0x00) read at init */
static uint8_t s_qma_addr = 0;    /* 0x12 or 0x13, whichever answered */

/* Two buffers so the LCD / the JPEG encoder can consume one frame while the
 * camera fills the other one - otherwise a transfer would regularly read a
 * half-written frame. */
typedef struct {
    uint8_t  *buf[2];
    size_t    len[2];
    size_t    buf_size;
    volatile int       writing;   /* buffer the DMA is currently filling */
    volatile int       ready;     /* buffer holding the last complete frame, -1 = none */
    volatile int       lock;      /* buffer reserved by a consumer, -1 = none */
    volatile uint32_t  frame_count; /* reset by the metrics loop to derive fps */
    volatile uint32_t  ready_seq;   /* never reset; lets the LCD task spot new frames */
} cam_ctx_t;

static cam_ctx_t s_cam_ctx = {0};

/* ST7789 viewfinder panel + the BMP snapshot buffer that backs /shot.
 *
 * The ESP32-S3 has no hardware JPEG *encoder* (only the P4 does) and the ROM
 * only carries tjpgd for decoding, so instead of compressing we hand the browser
 * the frame as an uncompressed 24-bit BMP - every browser renders it natively
 * through a plain <img>, so the dashboard needs no JavaScript changes. */
static esp_lcd_panel_handle_t s_lcd_panel = NULL;
static uint8_t               *s_shot_bmp = NULL;
static uint8_t               *s_photo_bmp = NULL;

/* Monotonic count of frames actually shifted into the panel.  The SPI panel IO
 * raises on_color_trans_done once per whole frame (after every chunk of a split
 * transfer has gone out), which is the only hard evidence that the viewfinder
 * path is alive when nobody is looking at the screen.  The metrics loop turns
 * the delta into lcd_fps. */
static volatile uint32_t s_lcd_frames = 0;
static uint32_t          s_lcd_frames_last = 0;
/* Dropped pushes.  Kept because esp_lcd_panel_draw_bitmap() failing is silent
 * at the application level - the only other symptom is the frame counter going
 * flat, which is easy to miss. */
static volatile uint32_t s_lcd_errors = 0;

#define SHOT_BMP_HEADER_SIZE  (54U)                                     /* 14 + 40 */
#define SHOT_BMP_PIXELS       ((size_t)CAM_LCD_W * CAM_LCD_H * 3U)      /* BGR888  */
#define SHOT_BMP_SIZE         (SHOT_BMP_HEADER_SIZE + SHOT_BMP_PIXELS)

static bool camera_copy_latest_to_bmp(uint8_t *out);

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

/* Runs in ISR context: record where the finished frame landed.  A raw RGB565
 * frame is always the whole buffer, so a reported size of 0 just means the
 * driver did not fill the field in - it never means "half a frame" here. */
static bool s_camera_get_finished_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    cam_ctx_t *ctx = (cam_ctx_t *)user_data;
    int idx = (trans->buffer == ctx->buf[0]) ? 0 : 1;
    size_t n = trans->received_size;

    if (n == 0 || n > ctx->buf_size) {
        n = ctx->buf_size;
    }
    ctx->len[idx] = n;
    ctx->ready = idx;
    ctx->ready_seq++;
    ctx->frame_count++;
    return false;
}

static uint32_t s_msync_errors = 0;

/* The camera writes PSRAM through DMA while the LCD and the /shot handler read
 * it back through the same cache.  Dropping the cached lines of a finished
 * frame keeps the consumers from rendering a stale copy. */
static inline void cam_frame_sync(const uint8_t *buf)
{
    esp_err_t err = esp_cache_msync((void *)buf, CAM_FRAME_MAX, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    if (err != ESP_OK) {
        /* Called ~25x/s, so only the first few failures are worth printing -
         * but they must not be swallowed: a failing sync means the LCD and
         * /shot quietly render stale pixels, which is very hard to notice. */
        if (++s_msync_errors <= 5) {
            ESP_LOGW(TAG, "cache sync failed: %s (total %u)", esp_err_to_name(err),
                     (unsigned)s_msync_errors);
        }
    }
}

/* Push finished frames to the ST7789.  This deliberately lives in a task rather
 * than in the capture callback: esp_lcd_panel_draw_bitmap() queues an SPI
 * transfer and is not meant to be called from interrupt context. */
static void lcd_refresh_task(void *arg)
{
    uint32_t seen = 0;
    while (true) {
        uint32_t seq = s_cam_ctx.ready_seq;
        int idx = s_cam_ctx.ready;
        if (s_lcd_panel != NULL && idx >= 0 && seq != seen) {
            seen = seq;
            cam_frame_sync(s_cam_ctx.buf[idx]);
            esp_err_t err = esp_lcd_panel_draw_bitmap(s_lcd_panel, 0, 0, CAM_LCD_W, CAM_LCD_H,
                                                      s_cam_ctx.buf[idx]);
            if (err != ESP_OK) {
                /* Do not burn the whole frame budget on retries: skip this frame
                 * and try the next one.  If it keeps failing, the counter below
                 * makes it visible in /data instead of just going quiet. */
                uint32_t n = ++s_lcd_errors;
                if (n <= 5 || (n % 250) == 0) {
                    ESP_LOGW(TAG, "lcd draw failed: %s (total %u)", esp_err_to_name(err),
                             (unsigned)n);
                }
            }
        }
        /* One whole tick, not pdMS_TO_TICKS(2): CONFIG_FREERTOS_HZ is 100, so a
         * 2 ms delay rounds down to 0 and vTaskDelay(0) only yields to equal or
         * higher priorities - which starves the idle task and trips the task
         * watchdog.  A frame every 40 ms is picked up within one 10 ms tick. */
        vTaskDelay(1);
    }
}

/* ------------------------- ST7789 viewfinder ------------------------- */

/* Runs in ISR context once the panel IO has finished a whole frame.  Only the
 * counter is touched here: the LCD task owns everything else. */
static bool lcd_on_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    (void)panel_io;
    (void)edata;
    (void)user_ctx;
    s_lcd_frames++;
    return false;
}

static void lcd_init(void)
{
    const ledc_timer_config_t bl_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LCD_BL_LEDC_TIMER,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&bl_timer));
    const ledc_channel_config_t bl_channel = {
        .gpio_num   = LCD_BACKLIGHT_IO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LCD_BL_LEDC_CHANNEL,
        .timer_sel  = LCD_BL_LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .duty       = 0,                    /* dark until the panel is configured */
        .hpoint     = 0,
        .flags.output_invert = true,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&bl_channel));

    const spi_bus_config_t bus_cfg = {
        .sclk_io_num     = LCD_SPI_CLK_IO,
        .mosi_io_num     = LCD_SPI_MOSI_IO,
        .miso_io_num     = GPIO_NUM_NC,
        .quadwp_io_num   = GPIO_NUM_NC,
        .quadhd_io_num   = GPIO_NUM_NC,
        /* Deliberately NOT CAM_FRAME_MAX.  The frame buffers live in PSRAM, so
         * for every transfer the SPI driver allocates a temporary internal-RAM
         * bounce buffer of the chunk size, memcpy's into it, sends, then frees.
         * With max_transfer_sz = 115200 that bounce buffer is a whole frame, and
         * once internal RAM gets tight the allocation fails - the driver logs
         * "Failed to allocate priv TX buffer", esp_lcd_panel_draw_bitmap()
         * returns ESP_ERR_NO_MEM, and the viewfinder freezes for good (observed
         * after ~4 min of uptime).  Keeping the chunk small means the bounce
         * buffer is always trivially allocatable; the panel IO splits the frame
         * into chunks of this size with CS held active.  The total memcpy per
         * frame is the same either way.
         *
         * Do not "optimise" this by setting psram_dma_direct instead: SPI DMA
         * straight out of PSRAM cannot keep up with the 80 MHz pixel clock and
         * the driver dies with "DMA TX underflow detected", after which every
         * later call fails with ESP_ERR_INVALID_STATE. */
        .max_transfer_sz = LCD_SPI_CHUNK_BYTES,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num       = LCD_DC_IO,
        .cs_gpio_num       = LCD_SPI_CS_IO,
        .pclk_hz           = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits      = LCD_CMD_BITS,
        .lcd_param_bits    = LCD_PARAM_BITS,
        .spi_mode          = 2,
        .trans_queue_depth = LCD_SPI_QUEUE_DEPTH,
    };
    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_cfg, &io_handle));

    /* Must be installed before the first transfer - the driver refuses a second
     * registration, and the panel init below already pushes parameters. */
    const esp_lcd_panel_io_callbacks_t io_callbacks = {
        .on_color_trans_done = lcd_on_color_trans_done,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &io_callbacks, NULL));

    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_RST_IO,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_cfg, &s_lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_lcd_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_lcd_panel, true));

    const uint32_t duty = (1023 * 100) / 100;   /* 100 % backlight */
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_BL_LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_BL_LEDC_CHANNEL));
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
        .h_res = CAM_LCD_W,
        .v_res = CAM_LCD_H,
        .pic_format_jpeg = 0,               /* raw RGB565 straight to the panel */
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

    /* one allocation, split in half - 240*240*2 is a multiple of the DMA and
     * cache-line alignment, which the JPEG encoder also relies on */
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
    /* Modem sleep parks the radio between beacons and inflates the TCP round-trip
     * to ~45 ms, which caps a single stream at snd_buf/RTT.  /shot pushes an
     * uncompressed 240x240 frame (173 KB), so keep the radio awake: the board is
     * USB powered and the extra draw buys a several-fold throughput gain. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "waiting for Wi-Fi (%s)...", CONFIG_SENSOR_DASH_WIFI_SSID);
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

/* ------------------- Persistent telemetry queue ------------------- */

static uint32_t tlm_crc32(const void *data, size_t len)
{
    const uint8_t *p = data;
    uint32_t hash = 2166136261UL; /* FNV-1a: detects partial Flash writes. */
    for (size_t i = 0; i < len; ++i) {
        hash ^= p[i];
        hash *= 16777619UL;
    }
    return hash;
}

static bool tlm_record_valid(const telemetry_record_t *record)
{
    return record->magic == TLM_MAGIC && record->version == TLM_VERSION &&
           record->state != 0xFFU &&
           record->crc32 == tlm_crc32(&record->boot_id,
                                      offsetof(telemetry_record_t, crc32) - offsetof(telemetry_record_t, boot_id));
}

static esp_err_t tlm_read_slot(size_t slot, telemetry_record_t *record)
{
    return esp_partition_read(s_tlm_partition, slot * TLM_RECORD_SIZE, record, sizeof(*record));
}

static bool tlm_slot_blank(size_t slot)
{
    uint32_t header[2] = {0};
    if (esp_partition_read(s_tlm_partition, slot * TLM_RECORD_SIZE, header, sizeof(header)) != ESP_OK) {
        return false;
    }
    return header[0] == 0xFFFFFFFFUL && header[1] == 0xFFFFFFFFUL;
}

static void tlm_store_drop_count(void)
{
    nvs_handle_t nvs;
    if (nvs_open("telemetry", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u32(nvs, "dropped", s_queue_dropped);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

static void tlm_restore_drop_count(void)
{
    nvs_handle_t nvs;
    if (nvs_open("telemetry", NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u32(nvs, "dropped", &s_queue_dropped);
        nvs_close(nvs);
    }
}

static size_t tlm_find_blank_from(size_t start)
{
    for (size_t offset = 0; offset < s_queue_slots; ++offset) {
        size_t slot = (start + offset) % s_queue_slots;
        if (tlm_slot_blank(slot)) {
            return slot;
        }
    }
    return s_queue_slots;
}

static size_t tlm_reclaim_oldest_sector(void)
{
    const size_t slots_per_sector = TLM_SECTOR_SIZE / TLM_RECORD_SIZE;
    const size_t sector_count = s_tlm_partition->size / TLM_SECTOR_SIZE;
    size_t selected = 0;
    int64_t oldest = INT64_MAX;
    bool found_pending = false;

    for (size_t sector = 0; sector < sector_count; ++sector) {
        for (size_t j = 0; j < slots_per_sector; ++j) {
            telemetry_record_t record;
            if (tlm_read_slot(sector * slots_per_sector + j, &record) != ESP_OK || !tlm_record_valid(&record)) {
                continue;
            }
            if (record.state == TLM_STATE_PENDING && record.captured_at_ms < oldest) {
                oldest = record.captured_at_ms;
                selected = sector;
                found_pending = true;
            }
        }
    }
    if (!found_pending) {
        selected = 0;
    }

    size_t discarded = 0;
    for (size_t j = 0; j < slots_per_sector; ++j) {
        telemetry_record_t record;
        if (tlm_read_slot(selected * slots_per_sector + j, &record) == ESP_OK &&
            tlm_record_valid(&record) && record.state == TLM_STATE_PENDING) {
            ++discarded;
        }
    }
    if (esp_partition_erase_range(s_tlm_partition, selected * TLM_SECTOR_SIZE, TLM_SECTOR_SIZE) != ESP_OK) {
        ESP_LOGE(TAG, "telemetry queue sector erase failed");
        return s_queue_slots;
    }
    if (discarded) {
        s_queue_depth = s_queue_depth > discarded ? s_queue_depth - discarded : 0;
        s_queue_dropped += discarded;
        tlm_store_drop_count();
        ESP_LOGW(TAG, "telemetry queue full; dropped %u oldest sample(s)", (unsigned)discarded);
    }
    return selected * slots_per_sector;
}

static void tlm_queue_init(void)
{
    s_tlm_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                               ESP_PARTITION_SUBTYPE_ANY, "telemetry");
    if (s_tlm_partition == NULL || s_tlm_partition->size < TLM_SECTOR_SIZE) {
        ESP_LOGE(TAG, "telemetry partition missing; remote storage disabled");
        return;
    }
    s_queue_slots = s_tlm_partition->size / TLM_RECORD_SIZE;
    for (size_t slot = 0; slot < s_queue_slots; ++slot) {
        telemetry_record_t record;
        if (tlm_read_slot(slot, &record) == ESP_OK && tlm_record_valid(&record) &&
            record.state == TLM_STATE_PENDING) {
            ++s_queue_depth;
        }
    }
    s_queue_write_slot = tlm_find_blank_from(0);
    tlm_restore_drop_count();
    ESP_LOGI(TAG, "telemetry queue: %u pending / %u slots", (unsigned)s_queue_depth,
             (unsigned)s_queue_slots);
}

static bool tlm_enqueue(const telemetry_snapshot_t *sample)
{
    if (s_tlm_partition == NULL) return false;
    if (s_queue_write_slot >= s_queue_slots || !tlm_slot_blank(s_queue_write_slot)) {
        s_queue_write_slot = tlm_find_blank_from(s_queue_write_slot % s_queue_slots);
    }
    if (s_queue_write_slot >= s_queue_slots) {
        s_queue_write_slot = tlm_reclaim_oldest_sector();
        if (s_queue_write_slot >= s_queue_slots) return false;
    }

    telemetry_record_t record = {
        .magic = TLM_MAGIC, .state = TLM_STATE_PENDING, .version = TLM_VERSION,
        .boot_id = s_boot_id, .sequence = ++s_sample_sequence,
        .captured_at_ms = sample->captured_at_ms,
        .temp_c = sample->temp_c, .heap_int = sample->heap_int, .heap_psram = sample->heap_psram,
        .fps = sample->fps, .jpeg = sample->jpeg, .accel_ok = sample->accel_ok,
        .accel_x_g = sample->accel_x_g, .accel_y_g = sample->accel_y_g,
        .accel_z_g = sample->accel_z_g, .accel_mag_g = sample->accel_mag_g,
        .accel_range_g = 4, .accel_id = sample->accel_id,
        .uptime_s = sample->uptime_s, .cpu_mhz = sample->cpu_mhz, .rssi = sample->rssi,
        .lcd_frames = sample->lcd_frames, .lcd_fps = sample->lcd_fps,
        .lcd_err = sample->lcd_err,
    };
    strncpy(record.ip, sample->ip, sizeof(record.ip) - 1);
    record.crc32 = tlm_crc32(&record.boot_id,
                             offsetof(telemetry_record_t, crc32) - offsetof(telemetry_record_t, boot_id));
    if (esp_partition_write(s_tlm_partition, s_queue_write_slot * TLM_RECORD_SIZE,
                            &record, sizeof(record)) != ESP_OK) {
        ESP_LOGE(TAG, "telemetry queue write failed");
        return false;
    }
    ++s_queue_depth;
    s_queue_write_slot = (s_queue_write_slot + 1) % s_queue_slots;
    return true;
}

static size_t tlm_load_batch(telemetry_record_t *records, size_t *slots, size_t limit)
{
    if (s_queue_depth == 0) {
        return 0;
    }
    /* PENDING records form one contiguous run that ends just before the write
     * cursor: records are appended at the cursor and acknowledged oldest-first,
     * so the run always has exactly s_queue_depth entries.  Starting one
     * run-length back turns this from "read all 48640 slots of the 6 MB
     * partition" (~2.8 s per upload cycle) into a handful of reads.  The loop is
     * still bounded by s_queue_slots and only stops once it has actually found
     * `want` records, so if the layout ever surprises us the cost is time, not
     * correctness. */
    const size_t want = (s_queue_depth < limit) ? s_queue_depth : limit;
    const size_t back = (s_queue_depth < s_queue_slots) ? s_queue_depth : s_queue_slots;
    const size_t start = (s_queue_write_slot + s_queue_slots - back) % s_queue_slots;

    size_t count = 0;
    for (size_t offset = 0; offset < s_queue_slots && count < want; ++offset) {
        size_t slot = (start + offset) % s_queue_slots;
        telemetry_record_t record;
        if (tlm_read_slot(slot, &record) == ESP_OK && tlm_record_valid(&record) &&
            record.state == TLM_STATE_PENDING) {
            records[count] = record;
            slots[count++] = slot;
        }
    }
    return count;
}

static void tlm_ack_batch(const size_t *slots, size_t count)
{
    const uint8_t acked = TLM_STATE_ACKED;
    for (size_t i = 0; i < count; ++i) {
        if (esp_partition_write(s_tlm_partition, slots[i] * TLM_RECORD_SIZE +
                                offsetof(telemetry_record_t, state), &acked, sizeof(acked)) == ESP_OK) {
            if (s_queue_depth) --s_queue_depth;
        }
    }
}

static int64_t tlm_now_ms(void)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    /* Return 0 before SNTP has set a plausible Unix clock. */
    return now.tv_sec >= 1577836800 ? (int64_t)now.tv_sec * 1000 + now.tv_usec / 1000 : 0;
}

static void tlm_start_sntp(void)
{
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_err_t err = esp_netif_sntp_init(&config);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SNTP init failed: %s", esp_err_to_name(err));
    }
}

static bool tlm_post_batch(const telemetry_record_t *records, size_t count)
{
    char *body = heap_caps_malloc(12288, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (body == NULL) body = malloc(12288);
    if (body == NULL) return false;
    size_t used = (size_t)snprintf(body, 12288, "{\"device_id\":\"%s\",\"samples\":[",
                                   CONFIG_SENSOR_DASH_DEVICE_ID);
    for (size_t i = 0; i < count && used < 12000; ++i) {
        const telemetry_record_t *r = &records[i];
        int written = snprintf(body + used, 12288 - used,
            "%s{\"boot_id\":\"%016" PRIx64 "\",\"sequence\":%" PRIu32
            ",\"captured_at_ms\":%" PRId64 ",\"temp_c\":%.2f,\"heap_int\":%" PRIu32
            ",\"heap_psram\":%" PRIu32 ",\"fps\":%.2f,\"jpeg\":%" PRIu32
            ",\"accel_ok\":%s,\"accel_x_g\":%.4f,\"accel_y_g\":%.4f,\"accel_z_g\":%.4f"
            ",\"accel_mag_g\":%.4f,\"accel_range_g\":%" PRIu32 ",\"accel_id\":\"0x%02X\""
            ",\"accel_chip\":\"%s\",\"uptime_s\":%" PRIu32 ",\"cpu_mhz\":%" PRIu32 ",\"rssi\":%" PRId32
            ",\"lcd_frames\":%" PRIu32 ",\"lcd_fps\":%.2f,\"lcd_err\":%" PRIu32
            ",\"ip\":\"%s\"}",
            i ? "," : "", r->boot_id, r->sequence, r->captured_at_ms, r->temp_c,
            r->heap_int, r->heap_psram, r->fps, r->jpeg, r->accel_ok ? "true" : "false",
            r->accel_x_g, r->accel_y_g, r->accel_z_g, r->accel_mag_g, r->accel_range_g,
            r->accel_id, qma_chip_name(r->accel_id), r->uptime_s, r->cpu_mhz, r->rssi,
            r->lcd_frames, r->lcd_fps, r->lcd_err, r->ip);
        if (written < 0 || (size_t)written >= 12288 - used) {
            free(body);
            return false;
        }
        used += (size_t)written;
    }
    snprintf(body + used, 12288 - used, "]}");
    esp_http_client_config_t config = {.url = CONFIG_SENSOR_DASH_COLLECTOR_URL,
                                       .method = HTTP_METHOD_POST, .timeout_ms = 5000};
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) { free(body); return false; }
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "X-Api-Key", CONFIG_SENSOR_DASH_COLLECTOR_API_KEY);
    esp_http_client_set_post_field(client, body, strlen(body));
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    free(body);
    return err == ESP_OK && status >= 200 && status < 300;
}

/* This uses the same schema and authentication as durable telemetry, but the
 * destination only retains the latest sample in RAM.  It is deliberately not
 * put in the Flash queue: a missed live update is harmless, whereas filling
 * the durable queue at display refresh rate would make recovery slower. */
static bool tlm_post_live_snapshot(const telemetry_snapshot_t *sample)
{
    char body[1536];
    int written = snprintf(body, sizeof(body),
        "{\"device_id\":\"%s\",\"samples\":[{\"boot_id\":\"%016" PRIx64
        "\",\"sequence\":%" PRIu32 ",\"captured_at_ms\":%" PRId64
        ",\"temp_c\":%.2f,\"heap_int\":%" PRIu32 ",\"heap_psram\":%" PRIu32
        ",\"fps\":%.2f,\"jpeg\":%" PRIu32 ",\"accel_ok\":%s,\"accel_x_g\":%.4f"
        ",\"accel_y_g\":%.4f,\"accel_z_g\":%.4f,\"accel_mag_g\":%.4f"
        ",\"accel_range_g\":4,\"accel_id\":\"0x%02X\",\"accel_chip\":\"%s\""
        ",\"uptime_s\":%" PRIu32 ",\"cpu_mhz\":%" PRIu32 ",\"rssi\":%" PRId32
        ",\"lcd_frames\":%" PRIu32 ",\"lcd_fps\":%.2f,\"lcd_err\":%" PRIu32
        ",\"ip\":\"%s\"}]}",
        CONFIG_SENSOR_DASH_DEVICE_ID, s_boot_id, ++s_live_sequence,
        sample->captured_at_ms, sample->temp_c, sample->heap_int, sample->heap_psram,
        sample->fps, sample->jpeg, sample->accel_ok ? "true" : "false",
        sample->accel_x_g, sample->accel_y_g, sample->accel_z_g, sample->accel_mag_g,
        sample->accel_id, qma_chip_name(sample->accel_id), sample->uptime_s,
        sample->cpu_mhz, sample->rssi, sample->lcd_frames, sample->lcd_fps,
        sample->lcd_err, sample->ip);
    if (written < 0 || (size_t)written >= sizeof(body)) return false;

    const char *api = strstr(CONFIG_SENSOR_DASH_COLLECTOR_URL, "/api/v1/");
    size_t base_len = api ? (size_t)(api - CONFIG_SENSOR_DASH_COLLECTOR_URL)
                          : strlen(CONFIG_SENSOR_DASH_COLLECTOR_URL);
    char url[192];
    snprintf(url, sizeof(url), "%.*s/api/v1/live-telemetry", (int)base_len,
             CONFIG_SENSOR_DASH_COLLECTOR_URL);
    esp_http_client_config_t config = {.url = url, .method = HTTP_METHOD_POST, .timeout_ms = 3000};
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) return false;
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "X-Api-Key", CONFIG_SENSOR_DASH_COLLECTOR_API_KEY);
    esp_http_client_set_post_field(client, body, written);
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    return err == ESP_OK && status >= 200 && status < 300;
}

static void telemetry_upload_task(void *arg)
{
    const TickType_t delay = pdMS_TO_TICKS(CONFIG_SENSOR_DASH_UPLOAD_INTERVAL_MS);
    while (true) {
        vTaskDelay(delay);
        if (!(xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) || s_tlm_partition == NULL) continue;
        telemetry_snapshot_t sample;
        xSemaphoreTake(s_tlm_lock, portMAX_DELAY);
        sample = s_latest_sample;
        xSemaphoreGive(s_tlm_lock);
        if (sample.uptime_s) tlm_enqueue(&sample);

        telemetry_record_t records[TLM_BATCH_MAX];
        size_t slots[TLM_BATCH_MAX];
        size_t count = tlm_load_batch(records, slots, TLM_BATCH_MAX);
        if (count) {
            s_upload_ok = tlm_post_batch(records, count);
            if (s_upload_ok) {
                tlm_ack_batch(slots, count);
                s_last_upload_ms = tlm_now_ms();
            }
        }
    }
}

static void live_telemetry_task(void *arg)
{
    const TickType_t delay = pdMS_TO_TICKS(CONFIG_SENSOR_DASH_LIVE_UPLOAD_INTERVAL_MS);
    while (true) {
        vTaskDelay(delay);
        if (!(xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT)) continue;
        telemetry_snapshot_t sample;
        xSemaphoreTake(s_tlm_lock, portMAX_DELAY);
        sample = s_latest_sample;
        xSemaphoreGive(s_tlm_lock);
        if (!sample.uptime_s) continue;
        s_live_upload_ok = tlm_post_live_snapshot(&sample);
        if (s_live_upload_ok) s_last_live_upload_ms = tlm_now_ms();
    }
}

/* -------------------- On-demand capture tasks -------------------- */

typedef struct {
    char *buf;
    size_t cap;
    size_t len;
} task_http_response_t;

static esp_err_t task_http_event(esp_http_client_event_t *event)
{
    task_http_response_t *response = event->user_data;
    if (event->event_id == HTTP_EVENT_ON_DATA && response && event->data_len > 0) {
        size_t room = response->cap > response->len ? response->cap - response->len - 1 : 0;
        size_t take = (size_t)event->data_len < room ? (size_t)event->data_len : room;
        if (take) {
            memcpy(response->buf + response->len, event->data, take);
            response->len += take;
            response->buf[response->len] = '\0';
        }
    }
    return ESP_OK;
}

static void task_url(char *out, size_t out_len, const char *path)
{
    const char *api = strstr(CONFIG_SENSOR_DASH_COLLECTOR_URL, "/api/v1/");
    size_t base_len = api ? (size_t)(api - CONFIG_SENSOR_DASH_COLLECTOR_URL)
                          : strlen(CONFIG_SENSOR_DASH_COLLECTOR_URL);
    snprintf(out, out_len, "%.*s%s", (int)base_len, CONFIG_SENSOR_DASH_COLLECTOR_URL, path);
}

static bool task_http_request(const char *url, esp_http_client_method_t method, const char *body,
                              char *response_buf, size_t response_len)
{
    task_http_response_t response = {.buf = response_buf, .cap = response_len, .len = 0};
    if (response_buf && response_len) response_buf[0] = '\0';
    esp_http_client_config_t config = {.url = url, .method = method, .timeout_ms = 5000,
                                       .event_handler = task_http_event, .user_data = &response};
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return false;
    esp_http_client_set_header(client, "X-Api-Key", CONFIG_SENSOR_DASH_COLLECTOR_API_KEY);
    if (body) {
        esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_http_client_set_post_field(client, body, strlen(body));
    }
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    return err == ESP_OK && status >= 200 && status < 300;
}

static bool task_post_ack(const char *request_id)
{
    char url[192], body[112];
    task_url(url, sizeof(url), "/api/v1/capture-tasks/");
    size_t used = strlen(url);
    snprintf(url + used, sizeof(url) - used, "%s/ack", request_id);
    snprintf(body, sizeof(body), "{\"device_id\":\"%s\"}", CONFIG_SENSOR_DASH_DEVICE_ID);
    return task_http_request(url, HTTP_METHOD_POST, body, NULL, 0);
}

static bool task_post_result(const char *request_id, const telemetry_snapshot_t *s)
{
    char url[192];
    task_url(url, sizeof(url), "/api/v1/capture-tasks/");
    size_t url_used = strlen(url);
    snprintf(url + url_used, sizeof(url) - url_used, "%s/result", request_id);
    char *body = heap_caps_malloc(4096, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!body) body = malloc(4096);
    if (!body) return false;
    uint32_t sequence = ++s_sample_sequence;
    int written = snprintf(body, 4096,
        "{\"device_id\":\"%s\",\"sample\":{\"boot_id\":\"%016" PRIx64
        "\",\"sequence\":%" PRIu32 ",\"captured_at_ms\":%" PRId64
        ",\"temp_c\":%.2f,\"heap_int\":%" PRIu32 ",\"heap_psram\":%" PRIu32
        ",\"fps\":%.2f,\"jpeg\":%" PRIu32 ",\"accel_ok\":%s,\"accel_x_g\":%.4f"
        ",\"accel_y_g\":%.4f,\"accel_z_g\":%.4f,\"accel_mag_g\":%.4f"
        ",\"accel_range_g\":4,\"accel_id\":\"0x%02X\",\"accel_chip\":\"%s\""
        ",\"uptime_s\":%" PRIu32 ",\"cpu_mhz\":%" PRIu32 ",\"rssi\":%" PRId32
        ",\"ip\":\"%s\",\"lcd_frames\":%" PRIu32 ",\"lcd_fps\":%.2f,\"lcd_err\":%" PRIu32 "}}",
        CONFIG_SENSOR_DASH_DEVICE_ID, s_boot_id, sequence, s->captured_at_ms, s->temp_c,
        s->heap_int, s->heap_psram, s->fps, s->jpeg, s->accel_ok ? "true" : "false",
        s->accel_x_g, s->accel_y_g, s->accel_z_g, s->accel_mag_g, s->accel_id,
        qma_chip_name(s->accel_id), s->uptime_s, s->cpu_mhz, s->rssi, s->ip,
        s->lcd_frames, s->lcd_fps, s->lcd_err);
    bool ok = written > 0 && written < 4096 && task_http_request(url, HTTP_METHOD_POST, body, NULL, 0);
    free(body);
    return ok;
}

static bool task_post_failure(const char *request_id, const char *error)
{
    char url[192], body[300];
    task_url(url, sizeof(url), "/api/v1/capture-tasks/");
    size_t url_used = strlen(url);
    snprintf(url + url_used, sizeof(url) - url_used, "%s/fail", request_id);
    snprintf(body, sizeof(body), "{\"device_id\":\"%s\",\"error\":\"%s\"}",
             CONFIG_SENSOR_DASH_DEVICE_ID, error);
    return task_http_request(url, HTTP_METHOD_POST, body, NULL, 0);
}

static bool task_post_photo(const char *request_id, int64_t captured_at_ms)
{
    if (s_photo_bmp == NULL) return false;
    char url[192], captured_at[32];
    task_url(url, sizeof(url), "/api/v1/capture-tasks/");
    size_t url_used = strlen(url);
    snprintf(url + url_used, sizeof(url) - url_used, "%s/photo", request_id);
    snprintf(captured_at, sizeof(captured_at), "%" PRId64, captured_at_ms);

    esp_http_client_config_t config = {.url = url, .method = HTTP_METHOD_POST, .timeout_ms = 10000};
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return false;
    esp_http_client_set_header(client, "Content-Type", "image/bmp");
    esp_http_client_set_header(client, "X-Api-Key", CONFIG_SENSOR_DASH_COLLECTOR_API_KEY);
    esp_http_client_set_header(client, "X-Device-Id", CONFIG_SENSOR_DASH_DEVICE_ID);
    esp_http_client_set_header(client, "X-Captured-At-Ms", captured_at);
    esp_http_client_set_post_field(client, (const char *)s_photo_bmp, SHOT_BMP_SIZE);
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    return err == ESP_OK && status >= 200 && status < 300;
}

static void capture_task_poll_task(void *arg)
{
    const TickType_t delay = pdMS_TO_TICKS(CONFIG_SENSOR_DASH_TASK_POLL_INTERVAL_MS);
    while (true) {
        vTaskDelay(delay);
        if (!(xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT)) continue;
        if (s_photo_upload_pending) {
            if (task_post_photo(s_photo_task_id, s_photo_captured_at_ms)) {
                ESP_LOGI(TAG, "capture-photo uploaded for %.8s", s_photo_task_id);
                s_photo_upload_pending = false;
                s_photo_task_id[0] = '\0';
            } else {
                ESP_LOGW(TAG, "capture-photo upload failed for %.8s", s_photo_task_id);
            }
            continue;
        }
        if (s_task_result_pending) {
            if (task_post_result(s_active_task_id, &s_task_result_sample)) {
                s_task_result_pending = false;
                s_active_task_id[0] = '\0';
            }
            continue;
        }
        char path[128], url[192], response[512];
        snprintf(path, sizeof(path), "/api/v1/devices/%s/tasks/next", CONFIG_SENSOR_DASH_DEVICE_ID);
        task_url(url, sizeof(url), path);
        if (!task_http_request(url, HTTP_METHOD_GET, NULL, response, sizeof(response))) {
            ESP_LOGW(TAG, "capture-task poll request failed");
            continue;
        }
        const char *marker = strstr(response, "\"request_id\":\"");
        if (!marker) {
            if (!strstr(response, "\"task\":null")) {
                ESP_LOGW(TAG, "capture-task poll returned unexpected payload: %s", response);
            }
            continue;
        }
        if (strlen(marker + 14) < 37 || marker[14 + 36] != '\"') {
            ESP_LOGW(TAG, "capture-task request_id is malformed");
            continue;
        }
        strncpy(s_active_task_id, marker + 14, sizeof(s_active_task_id) - 1);
        s_active_task_id[sizeof(s_active_task_id) - 1] = '\0';
        if (!task_post_ack(s_active_task_id)) {
            ESP_LOGW(TAG, "capture-task ACK failed for %.8s", s_active_task_id);
            continue;
        }
        ESP_LOGI(TAG, "capture-task ACK sent for %.8s", s_active_task_id);

        if (strstr(response, "\"task_type\":\"capture_photo\"")) {
            /* Wait beyond the 500 ms metric refresh period so this is a frame
             * captured after the server acknowledged the command, not a cached
             * picture from before the request. */
            vTaskDelay(pdMS_TO_TICKS(CONFIG_SENSOR_DASH_INTERVAL_MS + 100));
            if (!camera_copy_latest_to_bmp(s_photo_bmp)) {
                ESP_LOGW(TAG, "capture-photo camera frame unavailable for %.8s", s_active_task_id);
                if (task_post_failure(s_active_task_id, "camera frame unavailable")) {
                    s_active_task_id[0] = '\0';
                }
                continue;
            }
            strncpy(s_photo_task_id, s_active_task_id, sizeof(s_photo_task_id) - 1);
            s_photo_task_id[sizeof(s_photo_task_id) - 1] = '\0';
            s_active_task_id[0] = '\0';
            s_photo_captured_at_ms = tlm_now_ms();
            s_photo_upload_pending = true;
            if (task_post_photo(s_photo_task_id, s_photo_captured_at_ms)) {
                ESP_LOGI(TAG, "capture-photo uploaded for %.8s", s_photo_task_id);
                s_photo_upload_pending = false;
                s_photo_task_id[0] = '\0';
            } else {
                ESP_LOGW(TAG, "capture-photo upload failed for %.8s", s_photo_task_id);
            }
            continue;
        }

        /* The metrics loop refreshes s_latest_sample every 500 ms.  Waiting one
         * interval after the receipt guarantees this task returns a new source
         * read, rather than a snapshot that pre-dates its request_id. */
        vTaskDelay(pdMS_TO_TICKS(CONFIG_SENSOR_DASH_INTERVAL_MS + 100));
        xSemaphoreTake(s_tlm_lock, portMAX_DELAY);
        s_task_result_sample = s_latest_sample;
        xSemaphoreGive(s_tlm_lock);
        s_task_result_pending = true;
        if (task_post_result(s_active_task_id, &s_task_result_sample)) {
            ESP_LOGI(TAG, "capture-task result sent for %.8s", s_active_task_id);
            s_task_result_pending = false;
            s_active_task_id[0] = '\0';
        } else {
            ESP_LOGW(TAG, "capture-task result send failed for %.8s", s_active_task_id);
        }
    }
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

/* ------------------------- BMP snapshot ------------------------- */

static inline void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static inline void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

/* 14-byte BITMAPFILEHEADER + 40-byte BITMAPINFOHEADER, 24 bpp, BI_RGB.
 * A negative height marks the rows as top-down, which matches the order the
 * camera fills its buffer in.  240 * 3 bytes is already a multiple of 4, so
 * BMP's 4-byte row padding is a no-op here. */
static void shot_bmp_write_header(uint8_t *out)
{
    memset(out, 0, SHOT_BMP_HEADER_SIZE);
    out[0] = 'B';
    out[1] = 'M';
    put_le32(out + 2,  (uint32_t)SHOT_BMP_SIZE);
    put_le32(out + 10, SHOT_BMP_HEADER_SIZE);          /* pixel data offset */
    put_le32(out + 14, 40);                            /* BITMAPINFOHEADER size */
    put_le32(out + 18, CAM_LCD_W);
    put_le32(out + 22, (uint32_t)(-(int32_t)CAM_LCD_H));
    put_le16(out + 26, 1);                             /* planes */
    put_le16(out + 28, 24);                            /* bits per pixel */
    put_le32(out + 30, 0);                             /* BI_RGB, no compression */
    put_le32(out + 34, (uint32_t)SHOT_BMP_PIXELS);
    put_le32(out + 38, 2835);                          /* ~72 dpi */
    put_le32(out + 42, 2835);
}

/* The OV2640 hands the DVP bus big-endian RGB565, which is exactly the byte
 * order the ST7789 expects; BMP wants BGR888, so expand and swap here. */
static void shot_bmp_fill_pixels(const uint8_t *rgb565, uint8_t *bgr)
{
    for (size_t i = 0; i < (size_t)CAM_LCD_W * CAM_LCD_H; ++i) {
        uint16_t px = (uint16_t)(((uint16_t)rgb565[0] << 8) | rgb565[1]);
        rgb565 += 2;
        const uint32_t r5 = (px >> 11) & 0x1FU;
        const uint32_t g6 = (px >> 5) & 0x3FU;
        const uint32_t b5 = px & 0x1FU;
        *bgr++ = (uint8_t)((b5 << 3) | (b5 >> 2));
        *bgr++ = (uint8_t)((g6 << 2) | (g6 >> 4));
        *bgr++ = (uint8_t)((r5 << 3) | (r5 >> 2));
    }
}

/* Copy exactly one completed camera frame into a caller-owned BMP buffer.  The
 * expensive HTTP upload happens after this returns, so it never holds a camera
 * buffer or slows the LCD viewfinder. */
static bool camera_copy_latest_to_bmp(uint8_t *out)
{
    int idx = s_cam_ctx.ready;
    if (idx < 0 || out == NULL) return false;
    s_cam_ctx.lock = idx;
    cam_frame_sync(s_cam_ctx.buf[idx]);
    shot_bmp_fill_pixels(s_cam_ctx.buf[idx], out + SHOT_BMP_HEADER_SIZE);
    s_cam_ctx.lock = -1;
    return true;
}

/* GET /shot -> the newest viewfinder frame as an uncompressed 24-bit BMP.
 *
 * The capture pipeline runs in RGB565 for the panel, so the snapshot is a plain
 * colour-space conversion of the very same frame.  The capture buffer is only
 * reserved for the ~3 ms the conversion needs; the (much slower) network send
 * happens afterwards so the 25 fps LCD feed is not stalled. */
static esp_err_t shot_handler(httpd_req_t *req)
{
    if (s_shot_bmp == NULL || !camera_copy_latest_to_bmp(s_shot_bmp)) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "no frame yet\r\n");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/bmp");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t e = httpd_resp_send(req, (const char *)s_shot_bmp, SHOT_BMP_SIZE);
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
    s_tlm_lock = xSemaphoreCreateMutex();

    /* NVS is required by the Wi-Fi driver */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    s_boot_id = ((uint64_t)esp_random() << 32) | esp_random();
    tlm_queue_init();

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

    /* --- ST7789 viewfinder panel (SPI3), driven by the camera frames --- */
    lcd_init();
    ESP_LOGI(TAG, "LCD ready (%dx%d)", CAM_LCD_W, CAM_LCD_H);

    /* --- BMP snapshot buffer that backs /shot --- */
    s_shot_bmp = heap_caps_malloc(SHOT_BMP_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_shot_bmp == NULL) {
        ESP_LOGW(TAG, "snapshot buffer alloc failed; /shot disabled");
    } else {
        shot_bmp_write_header(s_shot_bmp);
        ESP_LOGI(TAG, "snapshot ready: %ux%u BMP, %u bytes", CAM_LCD_W, CAM_LCD_H, (unsigned)SHOT_BMP_SIZE);
    }
    s_photo_bmp = heap_caps_malloc(SHOT_BMP_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_photo_bmp == NULL) {
        ESP_LOGW(TAG, "remote photo buffer alloc failed; capture_photo tasks will fail");
    } else {
        shot_bmp_write_header(s_photo_bmp);
        ESP_LOGI(TAG, "remote photo buffer ready: %u bytes", (unsigned)SHOT_BMP_SIZE);
    }

    /* --- camera + shared I2C bus --- */
    esp_cam_ctlr_handle_t cam_handle = NULL;
    i2c_master_bus_handle_t shared_i2c_bus = NULL;
    bool cam_ok = (camera_init(&cam_handle, &shared_i2c_bus) == ESP_OK);
    ESP_LOGI(TAG, "camera %s", cam_ok ? "ready" : "unavailable - fps will be 0");
    if (cam_ok) {
        /* Check the result: a task that never started leaves lcd_frames at 0
         * forever with no other symptom - the same silent-failure shape as the
         * bounce-buffer bug this counter was added to catch. */
        if (xTaskCreate(lcd_refresh_task, "lcd_refresh", 4096, NULL, 4, NULL) != pdPASS) {
            ESP_LOGE(TAG, "lcd_refresh task creation failed - the panel will stay blank");
        }
    }
    if (shared_i2c_bus != NULL) {
        qma_init(shared_i2c_bus);
    }
    ESP_LOGI(TAG, "free PSRAM: %u bytes", (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    ESP_LOGI(TAG, "embedded dashboard.html is %u bytes", (unsigned)DASHBOARD_HTML_LEN);

    /* --- Wi-Fi + web server --- */
    wifi_init_sta();
    tlm_start_sntp();
    start_webserver();
    start_mdns();
    if (xTaskCreate(telemetry_upload_task, "telemetry_upload", 8192, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "telemetry_upload task creation failed - nothing will reach the collector");
    }
    if (xTaskCreate(live_telemetry_task, "live_telemetry", 6144, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "live_telemetry task creation failed - web live updates unavailable");
    }
    if (xTaskCreate(capture_task_poll_task, "capture_task_poll", 8192, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "capture_task_poll task creation failed - on-demand capture unavailable");
    }

    const TickType_t interval = pdMS_TO_TICKS(CONFIG_SENSOR_DASH_INTERVAL_MS);

    /* discard the first window: it accumulates everything captured during boot */
    vTaskDelay(interval);
    s_cam_ctx.frame_count = 0;
    s_lcd_frames_last = s_lcd_frames;   /* same for the panel: no bogus first lcd_fps */

    /* fps is derived from the time that actually elapsed, not from the nominal
     * interval: vTaskDelay only guarantees "at least", so a window stretched by
     * a slow upload would otherwise report a frame rate the camera cannot hit. */
    int64_t last_sample_us = esp_timer_get_time();

    while (1) {
        vTaskDelay(interval);

        const int64_t now_us = esp_timer_get_time();
        float interval_s = (float)(now_us - last_sample_us) / 1000000.0f;
        last_sample_us = now_us;
        if (interval_s <= 0.0f) {
            interval_s = CONFIG_SENSOR_DASH_INTERVAL_MS / 1000.0f;
        }

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

        /* Frames the panel IO reports as fully shifted out.  Counted with a
         * plain subtraction so a wrap is harmless. */
        const uint32_t lcd_total = s_lcd_frames;
        const uint32_t lcd_delta = lcd_total - s_lcd_frames_last;
        s_lcd_frames_last = lcd_total;
        const float lcd_fps = lcd_delta / interval_s;

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
                 "\"uptime_s\":%lu,\"cpu_mhz\":%lu,\"rssi\":%d,\"ip\":\"%s\","
                 "\"lcd_frames\":%lu,\"lcd_fps\":%.1f,\"lcd_err\":%lu,"
                 "\"telemetry_queue\":%u,\"telemetry_dropped\":%lu,"
                 "\"upload_ok\":%s,\"last_upload_ms\":%lld,"
                 "\"live_upload_ok\":%s,\"last_live_upload_ms\":%lld}",
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
                 s_ip,
                 (unsigned long)lcd_total, lcd_fps, (unsigned long)s_lcd_errors,
                 (unsigned)s_queue_depth,
                 (unsigned long)s_queue_dropped,
                 s_upload_ok ? "true" : "false",
                 (long long)s_last_upload_ms,
                 s_live_upload_ok ? "true" : "false",
                 (long long)s_last_live_upload_ms);
        xSemaphoreGive(s_json_lock);

        telemetry_snapshot_t sample = {
            .temp_c = temp_c,
            .heap_int = heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
            .heap_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
            .fps = fps,
            .jpeg = (uint32_t)(shot_idx >= 0 ? s_cam_ctx.len[shot_idx] : 0),
            .accel_ok = accel_ok,
            .accel_x_g = accel_x_g, .accel_y_g = accel_y_g, .accel_z_g = accel_z_g,
            .accel_mag_g = accel_mag_g, .accel_id = s_qma_id,
            .uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL),
            .cpu_mhz = (uint32_t)(esp_clk_cpu_freq() / 1000000UL),
            .rssi = s_rssi, .captured_at_ms = tlm_now_ms(),
            .lcd_frames = lcd_total, .lcd_fps = lcd_fps, .lcd_err = s_lcd_errors,
        };
        strncpy(sample.ip, s_ip, sizeof(sample.ip) - 1);
        xSemaphoreTake(s_tlm_lock, portMAX_DELAY);
        s_latest_sample = sample;
        xSemaphoreGive(s_tlm_lock);

        /* same sample on the serial console, handy for debugging */
        printf("%s\n", s_json);
        fflush(stdout);
    }
}
