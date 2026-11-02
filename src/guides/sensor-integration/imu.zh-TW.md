<!--
Translation Metadata:
- Source file: imu.md
- Last synced: 2026-11-02
- Translator: Claude (Anthropic)
- Status: Complete
-->

# IMU 感測器

車輛 IMU 來源的配置詳情。

## 支援的 IMU

車輛的 IMU 是 ZED X Mini 相機內建的那一顆：6 軸（加速度計與陀螺儀）感測器，
與相機同步，以 400 Hz 發布。以 `imu_source:=zed` 選用，每個感測器套件都是如此。

## ZED 內建 IMU

### 硬體

**整合**於 ZED X Mini 相機中
**規格**：BMI088（或類似）、400 Hz、±16g 加速度計、±2000°/s 陀螺儀

### 運作方式

```
ZED Camera Driver → /sensing/camera/zedxm/imu/data
                    ↓ (relay node)
                 /sensing/imu/zed/imu_raw → Autoware EKF
```

### 在 ZED 配置中啟用

```yaml
sensors.sensors_image_sync: true    # Sync with camera frames
sensors.publish_imu_tf: true         # Publish TF transform
```

### IMU 中繼配置

```xml
<node pkg="topic_tools" exec="relay" name="zed_imu_relay">
  <remap from="/sensing/camera/zedxm/imu/data" to="/sensing/imu/zed/imu_raw"/>
</node>
```

### 優點

- **出廠校正**：無需手動校正
- **同步**：與相機畫面硬體同步（精確時間戳記）
- **高速率**：400 Hz
- **穩定**：專業級感測器、漂移少

### 測試

```bash
# Launch ZED camera
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxm

# Verify IMU topic
ros2 topic list | grep imu
# Expected: /zedxm/zed_node/imu/data

# Check rate
ros2 topic hz /zedxm/zed_node/imu/data  # ~400 Hz
```

### 疑難排解

**無 IMU 資料**：
- 檢查 ZED 配置中的 `sensors.sensors_image_sync: true`
- 驗證 ZED SDK 版本支援 IMU（5.1.2+）
- 並非所有 ZED 型號都有內建 IMU

**中繼不運作**：
- 檢查中繼節點是否執行：`ros2 node list | grep relay`
- 驗證主題是否存在：`ros2 topic list | grep imu`

## IMU 資料格式

IMU 發布標準的 `sensor_msgs/Imu`：

```yaml
header:
  stamp: {sec, nanosec}
  frame_id: "zed_imu_link"

orientation: {x, y, z, w}           # Quaternion
angular_velocity: {x, y, z}         # rad/s
linear_acceleration: {x, y, z}      # m/s²
```

## 快速參考

```bash
# ZED IMU
ros2 topic hz /sensing/imu/zed/imu_raw
ros2 topic echo /sensing/camera/zedxm/imu/data --once

# Verify relay
ros2 node list | grep relay
```
