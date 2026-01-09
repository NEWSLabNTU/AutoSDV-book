<!--
Translation Metadata:
- Source file: gnss.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# GNSS 感測器

支援的 GNSS 接收器配置詳情。

## 支援的接收器

| 型號 | 類型 | 精度 | RTK 支援 | 介面 | 配置 |
|-------|------|----------|-------------|-----------|--------|
| **u-blox ZED-F9R** | 多頻 GNSS | ~2cm（RTK） | 是 | USB | `gnss_receiver:=ublox` |
| **Garmin GPS 18x** | 標準 GPS | ~3-5m | 否 | USB（FTDI） | `gnss_receiver:=garmin` |
| **Septentrio** | 多頻 GNSS | ~2cm（RTK） | 是 | USB/串列 | `gnss_receiver:=septentrio` |

**建議**：使用 u-blox ZED-F9R 進行具有 RTK 的戶外精密定位。

## u-blox ZED-F9R (RTK)

### 硬體

**板子**：SimpleRTK2B Fusion（u-blox ZED-F9R 模組）
**天線**：必須有清晰的天空視野（安裝在屋頂/最高點）
**連接**：USB 至 Jetson

### USB 裝置設定

建立 udev 規則以實現一致的裝置命名：

**檔案**：`/etc/udev/rules.d/99-ublox-gps.rules`
```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="ublox-gps", MODE="0666"
```

套用規則：
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger

# Verify
ls -l /dev/ublox-gps  # Should link to /dev/ttyACMx
```

### 驅動程式套件

**套件**：`ublox_gps`（ROS 2 Humble）
**安裝**：`sudo apt install ros-humble-ublox-gps`

### 使用 NTRIP 的 RTK

RTK（即時動態）使用來自基地台的修正資料實現約 2 公分的精度。

**NTRIP** 透過網際網路串流修正：
```
Internet → NTRIP Client → RTCM Corrections → u-blox Driver → Receiver
```

**預設 NTRIP 服務**：e-GNSS Taiwan VRS
- 伺服器：210.241.63.193:81
- 掛載點：Taiwan
- 覆蓋範圍：台灣地區

**配置**：`ntrip.launch.xml`
```xml
<node pkg="ntrip_client" exec="ntrip_client_node">
  <param name="host" value="210.241.63.193"/>
  <param name="port" value="81"/>
  <param name="mountpoint" value="Taiwan"/>
  <param name="username" value="[username]"/>
  <param name="password" value="[password]"/>
  <remap from="/ntrip/rtcm" to="/sensing/gnss/ntrip/rtcm"/>
</node>
```

### 啟用 RTK

```bash
# Launch with NTRIP corrections
make launch ARGS="gnss_receiver:=ublox use_ntrip:=true"
```

### 監控 RTK 狀態

**檢查 RTCM 修正**：
```bash
ros2 topic hz /sensing/gnss/ntrip/rtcm  # ~1 Hz
```

**檢查定位品質**：
```bash
ros2 topic echo /sensing/gnss/ublox/nav_sat_fix
# Look for status.status: 2 (RTK fix) or 1 (GNSS fix)
```

**定位狀態值**：
- `-1`：無定位
- `0`：GPS 定位
- `1`：DGPS 定位
- `2`：RTK 定位（最佳，約 2 公分精度）

### 疑難排解

**無 GNSS 定位**：
- 確保有清晰的天空視野（戶外、無障礙物）
- 等待 1-5 分鐘以取得衛星
- 檢查天線連接

**無 RTK 定位**：
- 驗證 RTCM 主題：`ros2 topic hz /sensing/gnss/ntrip/rtcm`
- 檢查網際網路連接（NTRIP 需要網路）
- 驗證 `ntrip.launch.xml` 中的 NTRIP 憑證
- 等待 1-10 分鐘以收斂 RTK

**精度不佳**：
- 改善天線位置（更高、更少障礙物）
- 減少多路徑（避免反射表面）
- 在開放區域測試（建議在台大思源廣場）

## Garmin GPS 18x

### 硬體

**接收器**：Garmin GPS 18x（標準 GPS，約 3-5 公尺精度）
**更新速率**：5 Hz
**介面**：USB（FTDI 串列轉換器）
**輸出**：NMEA 0183 語句

### USB 偵測

```bash
# Check device detected
sudo dmesg | grep tty
# Should show: FTDI USB Serial Device converter attached to ttyUSB0
```

### 驅動程式：gpsd

**安裝**：
```bash
sudo apt install gpsd gpsd-clients
```

**啟動守護程式**：
```bash
sudo /usr/sbin/gpsd -n -G -b /dev/ttyUSB0
```

**驗證訊號**：
```bash
# Terminal GPS viewer
cgps

# Graphical GPS viewer
xgps

# Wait for "3D Fix" (takes 1-5 minutes on cold start)
```

### ROS 2 整合

**配置**：`gnss.launch.xml`
```xml
<include file="$(find-pkg-share gpsd_client)/launch/gpsd_client-launch.py">
  <arg name="config_file" value="$(find-pkg-share gpsd_client)/config/gpsd_client.yaml"/>
</include>
```

**gpsd 客戶端配置**：
```yaml
/**:
  ros__parameters:
    host: "localhost"
    port: 2947
    frame_id: "gnss_link"
```

### 測試

```bash
# Start gpsd
sudo /usr/sbin/gpsd -n -G -b /dev/ttyUSB0

# Launch ROS 2 client
ros2 launch gpsd_client gpsd_client-launch.py

# Verify
ros2 topic echo /fix
ros2 topic hz /fix  # ~5 Hz
```

### 疑難排解

**無 GPS 訊號**：
- 移至開放區域（清晰的天空視野）
- 等待 5-10 分鐘進行冷啟動
- 檢查天線纜線連接

**找不到裝置**：
- 檢查 `/dev/ttyUSB0` 是否存在
- 嘗試不同的 USB 埠
- 驗證 USB 纜線提供電源

## 室內操作（無 GNSS）

對於無衛星訊號的室內環境：

```bash
# Disable GNSS, use NDT localization only
make launch ARGS="use_gnss:=false"
```

使用 RViz「2D Pose Estimate」工具手動設定初始位置。

## 快速參考

```bash
# Launch commands
make launch ARGS="gnss_receiver:=ublox use_ntrip:=true"   # u-blox RTK
make launch ARGS="gnss_receiver:=garmin"                  # Garmin
make launch ARGS="use_gnss:=false"                        # Indoor (no GNSS)

# u-blox verification
ls -l /dev/ublox-gps
ros2 topic hz /sensing/gnss/ublox/nav_sat_fix
ros2 topic hz /sensing/gnss/ntrip/rtcm  # RTK corrections

# Garmin verification
cgps                                     # Check GPS signal
ros2 topic hz /fix

# Check fix quality
ros2 topic echo /sensing/gnss/ublox/nav_sat_fix --once
# status.status: -1=no fix, 0=GPS, 1=DGPS, 2=RTK
```
