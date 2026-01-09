<!--
Translation Metadata:
- Source file: using-sensors.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 使用感測器

本指南說明如何使用不同的感測器配置啟動 AutoSDV。

## 感測器套件

AutoSDV 支援稱為**感測器套件**的預定義感測器組合。每個套件都是經過測試且能良好協同運作的感測器組合。

| 感測器套件 | 光達 | 相機 | IMU | GNSS | 使用情境 |
|--------------|-------|--------|-----|------|----------|
| `robin_zed` | Robin-W | ZED X Mini | ZED IMU | u-blox ZED-F9R | **建議** - 完整戶外套件 |
| `vlp32c_zed` | Velodyne VLP-32C | ZED X Mini | ZED IMU | u-blox ZED-F9R | 替代光達選項 |
| `cube1_zed` | Blickfeld Cube1 | ZED X Mini | ZED IMU | u-blox ZED-F9R | 固態光達選項 |

## 快速入門

### 使用感測器套件啟動

```bash
# Launch with Robin-W LiDAR + ZED camera (recommended)
make launch ARGS="sensor_suite:=robin_zed"

# Launch with Velodyne LiDAR + ZED camera
make launch ARGS="sensor_suite:=vlp32c_zed"

# Launch with Blickfeld LiDAR + ZED camera
make launch ARGS="sensor_suite:=cube1_zed"
```

### 驗證感測器運作

啟動後，檢查感測器是否正在發布資料：

```bash
# Check all sensor topics
ros2 topic list | grep /sensing

# Check LiDAR data rate (should be ~10 Hz)
ros2 topic hz /sensing/lidar/robin_lidar/points_raw

# Check camera images (should be ~15 Hz)
ros2 topic hz /sensing/camera/zedxm/rgb/image_rect_color

# Check IMU data (should be ~400 Hz)
ros2 topic hz /sensing/imu/zed/imu_raw

# Check GNSS position (should be ~5 Hz)
ros2 topic hz /sensing/gnss/ublox/nav_sat_fix
```

### 在 RViz 中視覺化

```bash
# Open RViz with AutoSDV configuration
make run-rviz
```

您應該會看到：
- 來自光達的點雲（白色/彩色點）
- 相機影像疊加
- 地圖中的車輛姿態
- 顯示感測器位置的 TF 框架

## 常見情境

### 使用 RTK GPS 的戶外環境

對於使用 RTK 修正的高精度戶外定位：

```bash
# Enable NTRIP for RTK positioning
make launch ARGS="sensor_suite:=robin_zed use_ntrip:=true"
```

這會連接到 e-GNSS Taiwan VRS 以獲得約 2 公分精度的 GPS。

### 室內（無 GPS）

對於無 GPS 訊號的室內操作：

```bash
# Disable GPS, use NDT localization only
make launch ARGS="sensor_suite:=robin_zed use_gnss:=false"
```

您需要在 RViz 中使用「2D Pose Estimate」工具手動設定初始姿態。

### 使用 Isaac Visual SLAM

使用基於相機的定位而非光達 NDT：

```bash
# Use Isaac Visual SLAM for pose estimation
make launch ARGS="sensor_suite:=robin_zed pose_source:=isaac"
```

## 自訂感測器選擇

除了使用預定義套件，您還可以選擇個別感測器：

```bash
# Custom combination
make launch ARGS="lidar_model:=robin-w camera_model:=zedxm imu_source:=zed gnss_receiver:=ublox"

# Minimal setup (LiDAR only)
make launch ARGS="lidar_model:=robin-w camera_model:=none imu_source:=mpu9250 use_gnss:=false"
```

### 可用選項

**光達型號：**
- `robin-w` - Robin-W 360° 光達（預設）
- `vlp32c` - Velodyne VLP-32C
- `cube1` - Blickfeld Cube1

**相機型號：**
- `zedxm` - ZED X Mini 立體相機（預設）
- `usb` - USB 網路攝影機
- `none` - 無相機

**IMU 來源：**
- `zed` - ZED 內建 IMU（預設）
- `mpu9250` - 外部 MPU9250 IMU

**GNSS 接收器：**
- `ublox` - u-blox ZED-F9R（支援 RTK，預設）
- `garmin` - Garmin GPS 18x（標準 GPS）
- `septentrio` - Septentrio 接收器

## 疑難排解

### 無感測器資料

**檢查主題是否存在：**
```bash
ros2 topic list | grep /sensing
```

如果主題缺失，感測器驅動程式可能無法啟動。檢查日誌：
```bash
# View latest logs
tail -f play_log/latest/*.log
```

### 未偵測到感測器

**檢查硬體連接：**
```bash
# For LiDAR (check network)
ping 172.168.1.10  # Robin-W
ping 192.168.1.201  # Velodyne

# For camera (check ZED SDK)
ZED_Explorer

# For GNSS (check USB device)
ls -l /dev/ublox-gps
```

### RViz 中的轉換錯誤

如果 RViz 顯示「No transform from X to Y」，請重新建置專案：
```bash
make build
```

## 下一步

- **了解感測器如何整合**：[整合範例](./integration-walkthrough.md)
- **配置特定感測器**：[光達](./lidar.md)、[相機](./camera.md)、[IMU](./imu.md)、[GNSS](./gnss.md)
- **新增感測器**：[新增感測器指南](./adding-sensor.md)
