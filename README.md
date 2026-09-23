# ESP32-S3-EYE 传感器数据面板（传感器数据传输）

ESP32-S3-EYE 开发板自己跑一个 HTTP 服务，浏览器直接打开板子 IP 就能看到实时数据：
芯片温度、内部 SRAM / PSRAM、OV2640 实时画面、**QMA6100P 三轴加速度**（含姿态仪）。
数据同时以 JSON 形式从 `GET /data` 输出，方便上位机采集。

板子还会把采样写入 Flash 队列并上传到**电脑端遥测服务**（`server/`），断网时缓存、
恢复后补传，由电脑保存到 SQLite 并提供历史查询页面。见「电脑端遥测存储」一节。

## 功能

| 接口 | 说明 |
|---|---|
| `GET /` | 面板页面（`main/dashboard.html`，用 `EMBED_TXTFILES` 嵌进固件） |
| `GET /data` | 一路 JSON 采样，页面每 500 ms 轮询一次 |
| `GET /shot` | 取景帧的 240×240 BMP 快照（`image/bmp`，172854 字节） |
| mDNS | `http://s3eye.local/`（Windows 默认不解析 `.local`，手机 / Linux 可用） |

`/data` 返回示例：

```json
{
  "temp_c": 39.6, "heap_int": 116067, "heap_psram": 7977908,
  "fps": 25.8, "jpeg": 115200,
  "accel_ok": true,
  "accel_x_g": 0.061, "accel_y_g": 0.194, "accel_z_g": 1.082,
  "accel_mag_g": 1.101, "accel_range_g": 4,
  "accel_id": "0x90", "accel_chip": "QMA6100P",
  "uptime_s": 15, "cpu_mhz": 240, "rssi": -40, "ip": "10.1.41.103",
  "lcd_frames": 296, "lcd_fps": 23.8, "lcd_err": 0,
  "telemetry_queue": 0, "telemetry_dropped": 0,
  "upload_ok": true, "last_upload_ms": 1789367752021
}
```

`jpeg` 这个字段名是历史遗留，现在表示**取景帧的字节数**（RGB565 240×240 下恒为 115200）。
`lcd_frames` / `lcd_fps` 统计的是**真正推给 ST7789 面板的帧**，来自 SPI panel IO 的
`on_color_trans_done` 回调 —— 看不到屏幕时，就看这两个值是否跟着 `fps` 一起涨。
`lcd_err` 是推屏失败次数，**正常情况下应恒为 0**；它非 0 就说明取景有问题（见「已知问题」）。

面板里的姿态仪由重力矢量算得：`Roll = atan2(Y, Z)`，`Pitch = atan2(−X, √(Y²+Z²))`。
板子水平静止时合加速度 `accel_mag_g` 应接近 **1.0 g**。

## 硬件

- **ESP32-S3-EYE**（ESP32-S3-WROOM-1-N8R8：8 MB flash + 8 MB Octal PSRAM）
- OV2640 摄像头（DVP），引脚见 `main/sensor_dashboard.c` 顶部 `CAM_*` 宏
- 板载三轴加速度计，与摄像头 **共用 I2C0**：SDA = GPIO4，SCL = GPIO5，地址 `0x12`（或 `0x13`）
- 板载 **ST7789** 屏，走 SPI3：MOSI = GPIO47，CLK = GPIO21，CS = GPIO44，DC = GPIO43，
  RST = 未接，背光 = GPIO48（LEDC timer 1，`output_invert = true`），像素时钟 80 MHz，原生 240×240。
  摄像头 XCLK 由 DVP 外设自己的时钟分频输出，不占 LEDC，所以背光用 timer 1 不会冲突。

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

## 电脑端遥测存储

固件每 500 ms 更新板载面板，并每 **5 秒**把当前传感器快照写入 Flash 队列并上传到电脑。
电脑未开机、服务未运行或网络中断时，数据会留在 6080 KB 的 `telemetry` 分区；恢复连接后，
设备以每批最多 20 条的方式补传。服务器以 `(device_id, boot_id, sequence)` 去重，因此重传不会
产生重复记录。队列写满时会淘汰最旧尚未上传的数据，并在 `/data` 中报告 `telemetry_dropped`。

### 实时数据传输

除可靠历史队列外，固件还每 **1 秒**向 `/api/v1/live-telemetry` 发送一条当前快照。服务器只在内存中
保留每台设备的最新一条，历史页的「实时数据」区域会每秒自动更新温度和三轴加速度的最近 120 个点。
因此实时查看不会把 SQLite 的 30 天历史和 Flash 队列放大为高频数据；实时链路短暂断开时只丢失该瞬时
更新，5 秒可靠队列仍会保存并补传历史观测。它不传输相机画面。

在 `idf.py menuconfig` 的 **Sensor Dashboard Configuration** 中，可用 `Live telemetry upload interval (ms)`
调整实时上报周期（默认 1000 ms，范围 500–60000 ms）。实时页面显示的「传输延迟」是浏览器当前时间与
服务器收到该快照的时间差；服务刚重启或开发板离线时会显示等待实时上报。

### 手机网页配网

Wi-Fi 凭据首次仍从 git-ignore 的 `sdkconfig` 读取；成功连接后会保存在板载 NVS。若该网络连续 3 次
连接失败（例如把开发板带离原网络），设备会启动一个 WPA2 配置热点：

- SSID：`S3EYE-Setup-<MAC后六位>`
- 地址：`http://192.168.4.1/`
- 密码：`Local Wi-Fi setup hotspot password`，在 menuconfig 的 **Sensor Dashboard Configuration** 设置；
  真实密码只在 git-ignore 的 `sdkconfig` 中保存。

手机连接这个热点后打开上述地址，选择扫描到的网络或手工填写 SSID 和密码。ESP32-S3 仅支持 **2.4 GHz**，
因此手机热点也必须开启 2.4 GHz。新凭据只有取得 DHCP 地址后才会写入 NVS；密码错误时旧凭据保留，热点继续
等待重新填写。配网完成后热点会关闭。切网时实时点暂时中断，但 Flash 里的可靠遥测仍会在重新联网后补传。

### 跨网远程访问（Cloudflare Tunnel）

不要将电脑的 8080 端口映射到公网。使用一个已经托管到 Cloudflare 的域名，并在一个 Tunnel 内配置两个
Public Hostname，均转发到 `http://127.0.0.1:8080`：

| 主机名 | 用途 | Cloudflare 访问策略 |
|---|---|---|
| `dashboard.<你的域名>` | 网页、历史、任务、照片 | Cloudflare Access：只允许你的邮箱一次性验证码登录 |
| `ingest.<你的域名>` | ESP32 上传、任务轮询与结果 | 不启用 Access；服务端仅放行携带 `X-Api-Key` 的设备接口 |

在 `server/.env` 设置两个主机名（不要提交）：

```ini
REMOTE_DASHBOARD_HOST=dashboard.example.com
REMOTE_INGEST_HOST=ingest.example.com
```

然后在 ESP-IDF 的 menuconfig 将 `Telemetry collector URL` 设置为
`https://ingest.<你的域名>/api/v1/telemetry`。固件会强制 HTTPS 并用 ESP x509 证书包验证服务端证书；
不再使用 `http://` 局域网地址。ESP32 可以连接任意可上网的 Wi-Fi 或手机热点，电脑与开发板不必在同一网络。

首次安装完 Python 依赖后，以管理员 PowerShell 执行：

```powershell
# 已在 Cloudflare 创建 Tunnel、两个 Public Hostname 和 Access 策略后
.\server\install-cloudflare-tunnel.ps1 -TunnelToken '<Cloudflare Tunnel token>'
.\server\install-autostart.ps1
```

前者安装 cloudflared 的 Windows 服务，后者创建开机启动的本机 FastAPI 任务。生产服务只监听
`127.0.0.1:8080`，Tunnel 是唯一公网入口。Cloudflare Token、API 密钥、域名和 Wi-Fi 密码均不得提交 Git。

### 启动服务

```powershell
cd server
Copy-Item .env.example .env
# 编辑 .env，设置一个长且随机的 TELEMETRY_API_KEY
.\start-server.ps1
```

首次执行会创建 Python 虚拟环境、安装依赖，并在 `http://<电脑局域网IP>:8080/` 提供历史页面。
数据库保存在 `server/data/telemetry.sqlite3`，使用 SQLite WAL，自动保留 30 天；页面支持时间筛选、
温度/加速度/内存曲线和 CSV 导出。

改了 `server/app.py` 后跑一遍单元测试（在 `firmware/` 目录下，用服务端自己的 venv）：

```bash
./server/.venv/Scripts/python.exe -m unittest server.test_server -v
```

**改动数据库列之后必须重启服务进程**：补列迁移 `add_missing_columns()` 只在
`Store.__init__`（即启动时）执行一次，正在跑的旧进程不会自动认出新列。
重启前上传仍然成功（Pydantic 会忽略多余字段），只是新列存不进去。

若 Windows 防火墙阻止局域网访问，请以管理员 PowerShell 执行：

```powershell
New-NetFirewallRule -DisplayName "ESP32 Telemetry 8080" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 8080 -Profile Any
```

> 用 `-Profile Any` 而不是 `Private`：很多有线/无线网络会被 Windows 判定为「公用」，
> 此时 `Private` 规则不会生效，板子会连接超时。先确认当前网络类别：
> `Get-NetConnectionProfile | Select-Object Name, NetworkCategory`

### 配置 ESP32 上传

在 `idf.py menuconfig` 的 **Sensor Dashboard Configuration** 中设置：

- `Telemetry collector URL`：例如 `http://192.168.1.50:8080/api/v1/telemetry`；电脑应使用固定或 DHCP 保留 IP。
- `Telemetry collector API key`：必须与 `server/.env` 中的 `TELEMETRY_API_KEY` 完全相同。
- `Telemetry device ID`：多块板子时为每块板子设不同值。

真实密钥只保存在 git-ignore 的 `sdkconfig` 和 `server/.env`，不要提交。分区表已改为 2 MB 应用
和 6080 KB 遥测队列，烧录本版本会覆盖板上原有分区表与 Flash 数据。

### 刷新历史与按需采集

电脑端页面的两个操作含义不同：

| 操作 | 是否访问设备 | 数据含义 |
|---|---:|---|
| 刷新已存数据 | 否 | 只查询 SQLite，可能是旧观测 |
| 采集一次最新数据 | 是 | 创建 UUID `request_id` 任务，板端回执后重新生成完整快照 |

按需任务目标是页面选择的 `device_id`（默认 `s3eye-001`），传感源为温度、内存、IMU、Wi-Fi、
摄像头和 LCD 统计。状态依次为 `submitted`、`received`、`completed`；板端报告错误为 `failed`，
30 秒内未完成为 `timeout`。结果写入 SQLite 后会带 `source=on_demand` 和对应的 `request_id`，可追踪
到唯一任务。

服务端的“暂停周期遥测入库”仅丢弃周期遥测记录，**不会**停止板端每秒任务轮询、回执或按需结果入库，
因此可用于课堂验证按钮触发而非周期上报。任务轮询间隔可在 menuconfig 中通过
`On-demand capture task poll interval` 调整。

### 局域网远程拍照

电脑端历史页的 **远程拍照** 只创建一个 `capture_photo` 任务。ESP32-S3-EYE 收到任务后回执、等待一帧
新的相机画面，并上传一张 240×240 BMP；它不会启动录像、定时拍摄或连续图片上传。上传使用与遥测相同的
`X-Api-Key`，任务会在 `submitted`、`received`、`completed`、`failed`、`timeout` 之间流转。照片保存在
服务器被 Git 忽略的 `server/data/photos/`，画廊按设备显示并在 7 天后自动清理；任务追踪记录仍会保留。

上传中的网络故障会重试同一张缓冲照片。设备重启后，服务器仍会下发处于 `received` 的任务，板端会重新拍摄
并补传；服务端以 `request_id` 去重，因此图库只保留一张照片。现有板载 `/shot` 与 LCD 取景仍可用，但它们
仅服务本地预览，不会将画面上传到服务器。

### 第 2 周：任务和新观测验收

局域网课堂验收时，先用“暂停周期遥测入库”停止周期记录，再点击“采集一次最新数据”或“远程拍照”。页面中
同一行任务会依次显示 `submitted`、`received`、`completed`，并带唯一 `request_id`；快照任务关联
`source=on_demand` 的一条新遥测，拍照任务关联图库中的一张 BMP。反复点击会创建不同任务；关闭设备后新任务在
30 秒后显示 `timeout`，旧记录不会被伪装成本次完成。

### 第 3 周：板载求助、测试、回应与取消

ESP32-S3-EYE 不需要额外接线：使用 ADC 按键梯形网络的前两个功能键（不使用 GPIO0 的 BOOT 键）和 GPIO3
板载绿灯。按键 1 短按创建 `help_request`（教学求助），按键 2 短按创建 `test_message`；任一键长按 1.5 秒
取消当前未结束事件。

| 板载 LED | 含义 |
|---|---|
| 快速闪烁 | 本地已按下，事件正等待上传或取消上传；不是远端已收到 |
| 慢速心跳 | 服务器已事务保存，等待查看者回应 |
| 常亮 1 秒 | 查看者回应已被开发板轮询到，并已发送开发板回执 |
| 三次短闪 | 本地或远端取消已完成 |

电脑网页“板载求助与测试事件”区分服务器接收、查看者回应、取消和“开发板已确认”。服务器对
`(device_id,event_id)` 去重；唯一的活动事件写入 NVS，所以断网或重启后会继续重试同一事件，而不会生成重复记录。
断网期间，板端 `/data` 只显示 `pending_upload` 与错误，不会声称服务器或查看者已经收到。

默认生产固件仅允许 HTTPS 采集地址。为完成私有局域网、无 TLS 的课堂联调，可在**不提交的** `sdkconfig` 中启用
`CONFIG_SENSOR_DASH_ALLOW_INSECURE_LAN=y` 并使用私网 HTTP 地址；该开关不得用于公网，完成联调后应关闭并恢复
HTTPS。Flash 备份写入 Git 忽略的 `flash-backups/`。

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
├── partitions.csv                 # 2 MB 固件 + 6080 KB 持久遥测队列
├── server/
│   ├── app.py                     # FastAPI + SQLite 接收、查询与导出
│   ├── static/index.html          # 历史曲线页面
│   └── start-server.ps1           # Windows 一键启动
└── tools/
    ├── _serial_port.py           # 无复位打开 USB Serial/JTAG（pyserial open 会复位板子）
    ├── http_probe.py             # 原始 socket 探测 HTTP 接口
    ├── serial_capture.py         # 抓 N 秒串口日志（被动，不复位板子）
    ├── serial_reset_capture.py   # 拉 DTR/RTS 复位板子并抓完整启动日志
    ├── soak.py                   # 长时间轮询 /data，查堆漂移 / 推屏停摆 / 重启
    └── bmp2png.py                # 纯标准库把 /shot 的 BMP 转 PNG（无需 Pillow）
```

## 稳定性自查

```bash
python tools/soak.py http://<板子IP> 600 30
```

每 30 秒拉一次 `/data`，跑 10 分钟，最后给结论：是否重启过、内部 SRAM 是否漂移、
`lcd_frames` 是否停摆、`lcd_err` 是否增长、遥测队列是否排空。

**`lcd_frames` 停摆 / `lcd_err` 增长 = 取景坏了**，短时间抓日志看不出来（要几分钟才触发）。
改了推屏相关代码后请跑一遍。

实测（`max_transfer_sz = 4096` / `trans_queue_depth = 4` 的版本）：
连续开机 20 分钟，`lcd_frames` 平均值始终 **24.9 → 25.0 fps**，`lcd_err` 恒为 0，无重启。
`heap_int` 会在 96–154 KB 之间来回振荡，那是 WiFi 动态缓冲池的涨落，**不是泄漏**
（泄漏会单调下降）。

## 已知问题

- **ESP32-S3 没有硬件 JPEG 编码器**。`esp_driver_jpeg` 只在带 `SOC_JPEG_CODEC_SUPPORTED`
  的芯片（如 P4）上编译出东西；S3 的 ROM 里只有 tjpgd **解码**（`jd_prepare` / `jd_decomp`）。
  所以摄像头一旦切到 RGB565 给屏幕取景，网页端就拿不到现成的 JPEG 了 —— `/shot` 改用
  未压缩的 24 位 BMP（浏览器 `<img>` 原生支持，前端零改动）。代价是每帧 173 KB。
- **`/shot` 的带宽**：`sdkconfig` 里 `CONFIG_LWIP_TCP_SND_BUF_DEFAULT` / `TCP_WND_DEFAULT`
  已从默认 5760 提到 32768，并关掉了 WiFi modem sleep（`esp_wifi_set_ps(WIFI_PS_NONE)`）。
  默认值下单个 TCP 流的吞吐被压到 `snd_buf / RTT ≈ 5760 / 46 ms ≈ 125 KB/s`，
  一帧 173 KB 要 1.4 s 以上。若把这两个值改回去，网页取景会明显变慢。
  实测调优后典型 **0.3–0.5 s / 帧（400–580 KB/s）**。
- **`/shot` 的丢包长尾**：约每 10 次会有一次 ~2.6 s。原因是 2.4 GHz 空口丢了一个段，
  LWIP 走一次 RTO（`CONFIG_LWIP_TCP_RTO_TIME=1500`，重传后翻倍）。
  这是环境因素，相关配置已经调到位（`LWIP_TCP_HIGH_SPEED_RETRANSMISSION=y`、
  `LWIP_TCP_QUEUE_OOSEQ=y`、`LWIP_TCP_OVERSIZE_MSS=y`），不用再重复调。
  想彻底消除就压缩帧——但 S3 没有 JPEG 编码器，见上一条。
- **面板页会把板子的套接字池打爆**。一帧 173 KB，如果浏览器按链路极限连续拉 `/shot`，
  板上 `CONFIG_LWIP_MAX_SOCKETS` 会被占满，症状是串口出现
  `httpd_accept_conn: error in accept (23)`（errno 23 = ENFILE）与
  `esp-tls: Failed to create socket`，然后 `/data` 开始超时、遥测也传不出去。
  两道防线：前端 `CAM_MIN_GAP_MS = 1000`（1 秒最多取一帧，取景够用），
  以及 `CONFIG_LWIP_MAX_SOCKETS=16`（默认 10 偏紧）。两项都在 `sdkconfig.defaults` 里。
  改 `CAM_MIN_GAP_MS` 能换更流畅的预览，代价是板子响应变迟钝。
- **抓串口日志默认会复位板子**：pyserial 的 `serial.Serial(port, ...)` 在 open 时断言
  DTR/RTS，而这在 ESP32-S3 内置 USB Serial/JTAG 上就是复位时序（实测 `uptime_s` 18 → 6）。
  `tools/serial_capture.py` 已改用 `tools/_serial_port.py::open_port()`
  （open 前先设 `rts = dtr = False`）绕开。自己写脚本时注意这一点。
- **`vTaskDelay` 的 tick 陷阱**：`CONFIG_FREERTOS_HZ = 100`，所以 `pdMS_TO_TICKS(2)` 等于 **0**，
  而 `vTaskDelay(0)` 只让给同优先级或更高的任务 —— 会把 idle 任务饿死并触发任务看门狗
  （`task_wdt: CPU 0: lcd_refresh`）。LCD 刷新任务必须用 `vTaskDelay(1)`。
- **`tlm_queue_init()` 仍要线性扫完整个 `telemetry` 分区**（6 MB / 128 B = 48640 槽位，
  约 2.8 s），所以启动到 Wi-Fi 可用会多花这几秒。这是**每次开机一次性**的代价，
  换来的好处是队列不需要任何 NVS 状态就能自愈（每个记录带 CRC + ACK 位，掉电可恢复）。
  想缩短就把 `partitions.csv` 里 `telemetry` 的 size 调小（代价是离线缓存时长等比下降：
  1 MB ≈ 8192 条 ≈ 11 小时）。
- `tlm_load_batch()` 原本**每个上传周期**也要扫完整个分区（+2.8 s，把 5 s 周期拖成 9 s）。
  已改成从写游标往前一个「待上传长度」开始扫：PENDING 记录是连续的、且紧邻写游标
  （记录顺序追加、按最旧优先 ACK），所以正常情况只需读几条。循环仍以 `s_queue_slots`
  为上限、并以「真的找齐 `want` 条」为退出条件，假设不成立时只是变慢、不会丢数据。
- **`lcd_frames` 是唯一能证明「屏幕在收帧」的信号**。SPI 推屏是单向的，面板不会回读；
  接上 `on_color_trans_done` 后，一帧 115200 B 会被 panel IO 按 `max_transfer_sz` 切成多块
  逐块发送，回调在**整帧发完后**触发一次，所以这个计数正好等于「完整推出去的帧数」。
  实测它与 `fps` 同步（24–26 /s），说明每一帧都推到了屏上。
- **取景会「卡死」的真正原因（已修）**：帧缓冲在 PSRAM，而 SPI 驱动对**每一次传输**
  都要申请一块**内部 RAM** 的临时 bounce buffer、memcpy 过去再发。
  原来 `max_transfer_sz = CAM_FRAME_MAX`，所以这块缓冲是**整帧 115200 字节**；
  内部 RAM 一紧张（WiFi 也在抢），分配失败 →
  `spi_common: Failed to allocate priv TX buffer` → `esp_lcd_panel_draw_bitmap()` 返回
  `ESP_ERR_NO_MEM` → **取景永久停止**（实测开机约 4 分钟后必现）。
  而且应用层完全静默，只有驱动层报错。
  现在 `max_transfer_sz = 4096`、`trans_queue_depth = 4`，峰值 bounce 约 **16 KB**，不再可能分配失败。
  一帧拆成 29 块发，总 memcpy 量与之前相同。
  - 排查时注意：**真正的内存约束是 `max_transfer_sz × trans_queue_depth`**
    （每个在飞的传输各持一块 bounce buffer）。只调小前者、把后者留在 10，
    峰值反而是 10 × 16 KB = 160 KB，比原来更糟 —— 这个坑踩过。
  - **不要**改用 `esp_lcd_panel_io_spi_config_t.flags.psram_dma_direct = 1` 来省掉 bounce buffer：
    SPI DMA 直接从 PSRAM 取数喂不上 80 MHz 像素时钟，会报
    `spi_master: DMA TX underflow detected`，之后每次调用都返回 `ESP_ERR_INVALID_STATE`
    （事务卡住无法回收）。文档里那句「has speed limit」就是这个意思。
- **`/data` 的 `lcd_err`** 统计推屏失败次数。加它的理由就是上面那个 bug：
  失败时应用层没有任何可见信号，`lcd_frames` 只是悄悄不再增长，很容易漏掉。
- Windows 默认不解析 mDNS 的 `.local` 域名，直接用板子 IP 访问。
