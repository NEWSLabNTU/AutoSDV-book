<!--
Translation Metadata:
- Source file: imu.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# IMU 感測器

支援的 IMU 來源配置詳情。

## 支援的 IMU

| 型號 | 類型 | 更新速率 | 功能 | 配置 |
|-------|------|-------------|----------|--------|
| **MPU9250** | 9 軸（外部） | ~100 Hz | 加速度計、陀螺儀、磁力計 | `imu_source:=mpu9250` |
| **ZED Built-in** | 6 軸（整合） | 400 Hz | 加速度計、陀螺儀、相機同步 | `imu_source:=zed` |

**建議**：使用 ZED 相機時建議使用 ZED 內建 IMU（出廠校正、同步、更穩定）。

## MPU9250

### 硬體連接

**介面**：Jetson 上的 I2C Bus 1

```
MPU9250          Jetson AGX Orin
───────          ───────────────
VCC (3.3V) ───── Pin 1 (3.3V)
GND ──────────── Pin 6 (GND)
SDA ──────────── Pin 3 (I2C SDA)
SCL ──────────── Pin 5 (I2C SCL)
```

**I2C 位址**：0x68（預設）或 0x69（AD0 腳位為高電位時）

### 驅動程式套件

**位置**：`src/sensor_component/external/ros2_mpu9250_driver/`

<span id="configuration"></span>
### 配置

**I2C 設定**：
```yaml
/**:
  ros__parameters:
    i2c_bus: 1
    i2c_address: 0x68
    frame_id: "mpu9250_link"
```

**校正**（用於精確測量）：
```yaml
/**:
  ros__parameters:
    accel_bias: [0.0, 0.0, 0.0]
    gyro_bias: [0.0, 0.0, 0.0]
    mag_bias: [0.0, 0.0, 0.0]
```

### 獨立測試

```bash
# Check I2C device detected
sudo i2cdetect -y -r 1
# Should show 68 at address 0x68

# Build and launch driver
colcon build --packages-select mpu9250driver
source install/setup.bash
ros2 run mpu9250driver mpu9250driver

# Verify IMU data
ros2 topic hz /imu/data  # ~100 Hz
ros2 topic echo /imu/data --once
```

### 疑難排解

**未偵測到裝置**：
- 檢查配線（SDA、SCL、VCC、GND）
- 驗證已啟用 I2C：`ls /dev/i2c-*`
- 嘗試替代位址 0x69

**資料雜訊**：
- 校正 IMU（記錄偏差值）
- 固定安裝（將振動降至最低）
- 在軟體中加入低通濾波器

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
- **更高速率**：400 Hz vs 100 Hz（MPU9250）
- **更穩定**：專業級感測器、漂移更少

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

兩個 IMU 都發布標準的 `sensor_msgs/Imu`：

```yaml
header:
  stamp: {sec, nanosec}
  frame_id: "mpu9250_link" or "zed_imu_link"

orientation: {x, y, z, w}           # Quaternion
angular_velocity: {x, y, z}         # rad/s
linear_acceleration: {x, y, z}      # m/s²
```

## 快速參考

```bash
# MPU9250
sudo i2cdetect -y -r 1                    # Check I2C device
ros2 topic hz /sensing/imu/mpu9250/imu_raw

# ZED IMU
ros2 topic hz /sensing/imu/zed/imu_raw
ros2 topic echo /sensing/camera/zedxm/imu/data --once

# Verify relay
ros2 node list | grep relay
```
