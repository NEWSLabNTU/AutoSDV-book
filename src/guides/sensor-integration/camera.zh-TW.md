<!--
Translation Metadata:
- Source file: camera.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 相機感測器

支援的相機型號配置詳情。

## 支援的型號

| 型號 | 類型 | 解析度 | 畫面速率 | 功能 | 配置 |
|-------|------|------------|------------|----------|--------|
| **ZED X Mini** | 立體相機 | 1920×1200 | 15-60 FPS | 深度、IMU、AI 偵測 | `camera_model:=zedxm` |
| **USB Camera** | 單眼相機 | 不定 | 30 FPS | 基本影像 | `camera_model:=usb` |

## ZED X Mini

### 先決條件

**硬體**：配備 ZED Link Duo 擷取卡的 NVIDIA Jetson（GMSL 連接）
**軟體**：ZED SDK 5.1.2（參閱安裝指南）

### 連接

```
ZED X Mini → GMSL Cable → ZED Link Duo → PCIe → Jetson
```

驗證偵測：
```bash
lspci | grep NVIDIA        # Check ZED Link card
ZED_Explorer               # Test camera with GUI
```

### 驅動程式套件

**位置**：`src/sensor_component/external/zed-ros2-wrapper/`
**套件**：`zed_components`（可組合節點架構）
**外掛程式**：`stereolabs::ZedCamera`

<span id="special-namespace-workaround"></span>
### 特殊：命名空間解決方案

ZED Python 啟動檔案不遵守 XML 命名空間。**解決方案**：直接在 XML 中載入可組合節點。

**正確配置**：
```xml
<!-- Container with ABSOLUTE namespace -->
<node_container
  pkg="rclcpp_components"
  exec="component_container_isolated"
  name="zed_container"
  namespace="/sensing/camera/zedxm">  <!-- Starts with / -->
</node_container>

<!-- Load composable node -->
<load_composable_node target="/sensing/camera/zedxm/zed_container">
  <composable_node pkg="zed_components" plugin="stereolabs::ZedCamera" ...>
</load_composable_node>
```

這會正確地在 `/sensing/camera/zedxm/*` 建立主題。

### 內建 IMU

ZED 具有整合的 6 軸 IMU（400 Hz，出廠校正）。

**在 ZED 配置中啟用**：
```yaml
sensors.sensors_image_sync: true
sensors.publish_imu_tf: true
```

**IMU 中繼模式**：
```
ZED Driver → /sensing/camera/zedxm/imu/data
               ↓ (relay node)
          /sensing/imu/zed/imu_raw → Autoware EKF
```

中繼配置：
```xml
<node pkg="topic_tools" exec="relay" name="zed_imu_relay">
  <remap from="/sensing/camera/zedxm/imu/data" to="/sensing/imu/zed/imu_raw"/>
</node>
```

### 物件偵測

ZED SDK 包含基於 AI 的物件偵測（人員、車輛、動物）。

**啟用**：
```xml
<param name="object_detection.od_enabled" value="true"/>
<param name="object_detection.model" value="MULTI_CLASS_BOX_MEDIUM"/>
<param name="object_detection.confidence_threshold" value="50.0"/>
```

**輸出**：`/sensing/camera/zedxm/obj_det/objects`（ZED 格式）

**停用以提升效能**：
```bash
make launch ARGS="enable_zed_object_detection:=false"
```

### 配置選項

**深度模式**：
```yaml
depth.depth_mode:
  NEURAL        # Best quality (GPU-intensive)
  ULTRA         # High quality
  QUALITY       # Standard
  PERFORMANCE   # Fast
```

**畫面速率**：
```yaml
general.grab_frame_rate: 15  # or 30, 60
```

**解析度**：
```yaml
general.grab_resolution:
  HD2K    # 2208×1242 (highest)
  HD1080  # 1920×1080
  HD720   # 1280×720
  VGA     # 672×376 (lowest latency)
```

### 獨立測試

```bash
# ZED SDK test
ZED_Explorer

# ROS 2 wrapper test
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxm

# Verify topics
ros2 topic list | grep zedxm
ros2 topic hz /zedxm/zed_node/rgb/image_rect_color  # ~15-30 Hz
```

### 疑難排解

**未偵測到相機**：
- 檢查 `lspci | grep NVIDIA` 顯示 ZED Link
- 檢查 `/usr/local/zed/settings/version.txt` 顯示 5.1.2
- 將相機和 Jetson 重新啟動

**命名空間錯誤**：
- 使用可組合節點載入（不要使用 Python 啟動檔案）
- 使用絕對命名空間 `/sensing/camera/zedxm`

**VNC 問題**：
- 使用 TurboVNC + VirtualGL 進行硬體加速
- 標準 VNC 不支援 GPU

## USB 相機

### 硬體

- 標準 USB 網路攝影機（UVC 相容）
- 透過 USB 連接到 Jetson
- 偵測為 `/dev/video*`

### 驅動程式

**套件**：`v4l2_camera`
**安裝**：`sudo apt install ros-humble-v4l2-camera`

### 配置

```xml
<node pkg="v4l2_camera" exec="v4l2_camera_node" name="usb_camera">
  <param name="video_device" value="/dev/video0"/>
  <param name="image_size" value="[640, 480]"/>
  <param name="pixel_format" value="YUYV"/>
  <param name="camera_frame_id" value="usb_camera_link"/>
</node>
```

### 獨立測試

```bash
# List available cameras
v4l2-ctl --list-devices

# Test camera
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
```

## 快速參考

```bash
# ZED SDK tools
ZED_Explorer                                      # Live view
ZED_Calibration                                   # Calibration tool
cat /usr/local/zed/settings/version.txt           # Check SDK version

# Topic verification
ros2 topic list | grep /sensing/camera
ros2 topic hz /sensing/camera/zedxm/rgb/image_rect_color
ros2 topic echo /sensing/camera/zedxm/imu/data --once

# USB camera
v4l2-ctl --list-devices
ros2 topic echo /image_raw --once
```
