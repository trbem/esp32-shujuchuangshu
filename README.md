# ESP32-S3-EYE 传感器数据面板（传感器数据传输）

ESP32-S3-EYE 开发板自己跑一个 HTTP 服务，浏览器直接打开板子 IP 就能看到实时数据：
芯片温度、内部 SRAM / PSRAM、OV2640 实时画面、**QMA6100P 三轴加速度**（含姿态仪）。
数据同时以 JSON 形式从 `GET /data` 输出，方便上位机采集。

## 功能

| 接口 | 说明 |
|---|---|
| `GET /` | 面板页面（`main/dashboard.html`，用 `EMBED_TXTFILES` 嵌进固件） |
| `GET /data` | 一路 JSON 采样，页面每 500 ms 轮询一次 |
| `GET /shot` | OV2640 的 320×240 JPEG 单帧 |
| mDNS | `http://s3eye.local/`（Windows 默认不解析 `.local`，手机 / Linux 可用） |

`/data` 返回示例：

```json
{
  "temp_c": 42.6, "heap_int": 258707, "heap_psram": 8230836,
  "fps": 50.0, "jpeg": 5014,
  "accel_ok": true,
  "accel_x_g": 0.094, "accel_y_g": 0.170, "accel_z_g": 0.996,
  "accel_mag_g": 1.015, "accel_range_g": 4,
  "accel_id": "0x90", "accel_chip": "QMA6100P",
  "uptime_s": 4, "cpu_mhz": 240, "rssi": -43, "ip": "10.1.41.103"
}
```

面板里的姿态仪由重力矢量算得：`Roll = atan2(Y, Z)`，`Pitch = atan2(−X, √(Y²+Z²))`。
板子水平静止时合加速度 `accel_mag_g` 应接近 **1.0 g**。

## 硬件

- **ESP32-S3-EYE**（ESP32-S3-WROOM-1-N8R8：8 MB flash + 8 MB Octal PSRAM）
- OV2640 摄像头（DVP）
- 板载三轴加速度计，与摄像头 **共用 I2C0**：SDA = GPIO4，SCL = GPIO5，地址 `0x12`（或 `0x13`）

## 关于 IMU 型号（重要）

官方原理图把 IMU 标成 **QMA7981**，但实测读 WHO_AM_I（寄存器 `0x00`）得到 **`0x90`**，
即板上实际是 **QMA6100P**（QMA7981 的 ID 是 `0xE7`）。本项目因此使用官方组件
`espressif/qma6100p`，并在初始化时读 ID 自动判定型号：

```c
0x90 -> "QMA6100P"
0xE7 -> "QMA7981"
```

### 官方驱动的一个 2× 换算 bug

`espressif/qma6100p` 的 `qma6100p_get_raw_acce()` 用 `(int16_t)(H<<8 | L) / 4`，
对这颗料多右移了 1 位，读数只有真实值的一半（静止时 `|a|` 只有 ~0.50 g）。

实测证据（重力轴 raw，板子静止）：

| 量程 | 驱动灵敏度表 | 理论 1g | 实测 raw |
|---|---|---|---|
| ±2g | 4096 | 4096 | 2023 |
| ±4g | 2048 | 2048 | 1041 |

本项目**不使用** `qma6100p_get_acce()`，而是自己换算：

```c
qma6100p_get_acce_sensitivity(h, &sens);
qma6100p_get_raw_acce(h, &raw);
g = raw * QMA_RAW_SCALE_CORRECTION / sens;   /* QMA_RAW_SCALE_CORRECTION = 2.0f */
```

修正后静止 `accel_mag_g ≈ 1.015 g`（残余约 5% 为 MEMS 出厂灵敏度误差）。
如果你换了一颗料发现数值翻倍，把这个宏改回 `1.0f` 即可。

## 构建与烧录

环境：**ESP-IDF v6.0.3**

```bash
idf.py set-target esp32s3
idf.py menuconfig          # 设置 WiFi：Sensor Dashboard Configuration
idf.py build
idf.py -p COM5 flash
```

`idf.py menuconfig` 里需要填：

- `Wi-Fi SSID (2.4 GHz only)` — 必须是 2.4 GHz（ESP32-S3 没有 5 GHz 射频）
- `Wi-Fi password`

> **安全**：WiFi 密码只存在 git-ignore 的 `sdkconfig` 里，不要提交。
> 仓库中 `Kconfig.projbuild` 的默认值已替换成占位符。

`managed_components/` 由 `idf.py reconfigure` 依据 `dependencies.lock` 自动拉取，无需提交。
首次 clone 后直接 `idf.py build` 即可（会自动下载 `espressif/mdns` 和 `espressif/qma6100p`）。

## 目录结构

```
firmware/
├── CMakeLists.txt
├── sdkconfig.defaults            # 公共默认配置（已提交）
├── sdkconfig.defaults.esp32s3    # S3 专用：Octal PSRAM / 8MB flash（已提交）
├── dependencies.lock             # 托管组件版本锁（已提交）
├── main/
│   ├── CMakeLists.txt
│   ├── Kconfig.projbuild         # WiFi / 采样周期配置项
│   ├── idf_component.yml         # 依赖：sensor_init / mdns / qma6100p
│   ├── dashboard.html            # 面板页面（嵌入固件）
│   └── sensor_dashboard.c        # 主程序
└── tools/
    ├── http_probe.py             # 原始 socket 探测 HTTP 接口
    └── serial_capture.py         # 抓 N 秒串口日志
```

## 已知问题

- **IDF v6.0.3 的 DVP JPEG**：`trans->received_size` 有时恒为 0。处理办法是在完成回调里
  `esp_cache_msync(..., ESP_CACHE_MSYNC_FLAG_DIR_M2C)` 重新同步，再从后向前扫描 JPEG 结束标记
  `FF D9` 恢复真实帧长度。
- Windows 默认不解析 mDNS 的 `.local` 域名，直接用板子 IP 访问。
