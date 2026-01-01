# IMU Sensors

Configuration details for supported IMU sources.

## Supported IMUs

| Model | Type | Update Rate | Features | Config |
|-------|------|-------------|----------|--------|
| **MPU9250** | 9-axis (external) | ~100 Hz | Accel, Gyro, Magnetometer | `imu_source:=mpu9250` |
| **ZED Built-in** | 6-axis (integrated) | 400 Hz | Accel, Gyro, Camera-synced | `imu_source:=zed` |

**Recommendation**: Use ZED built-in IMU when using ZED camera (factory-calibrated, synchronized, more stable).

## MPU9250

### Hardware Connection

**Interface**: I2C Bus 1 on Jetson

```
MPU9250          Jetson AGX Orin
───────          ───────────────
VCC (3.3V) ───── Pin 1 (3.3V)
GND ──────────── Pin 6 (GND)
SDA ──────────── Pin 3 (I2C SDA)
SCL ──────────── Pin 5 (I2C SCL)
```

**I2C Address**: 0x68 (default) or 0x69 (if AD0 pin high)

### Driver Package

**Location**: `src/sensor_component/external/ros2_mpu9250_driver/`

### Configuration

**I2C settings**:
```yaml
/**:
  ros__parameters:
    i2c_bus: 1
    i2c_address: 0x68
    frame_id: "mpu9250_link"
```

**Calibration** (for accurate measurements):
```yaml
/**:
  ros__parameters:
    accel_bias: [0.0, 0.0, 0.0]
    gyro_bias: [0.0, 0.0, 0.0]
    mag_bias: [0.0, 0.0, 0.0]
```

### Test Standalone

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

### Troubleshooting

**Device not detected**:
- Check wiring (SDA, SCL, VCC, GND)
- Verify I2C enabled: `ls /dev/i2c-*`
- Try alternate address 0x69

**Noisy data**:
- Calibrate IMU (record bias values)
- Secure mounting (minimize vibration)
- Add low-pass filter in software

## ZED Built-in IMU

### Hardware

**Integrated** in ZED X Mini camera
**Specifications**: BMI088 (or similar), 400 Hz, ±16g accelerometer, ±2000°/s gyroscope

### How It Works

```
ZED Camera Driver → /sensing/camera/zedxm/imu/data
                    ↓ (relay node)
                 /sensing/imu/zed/imu_raw → Autoware EKF
```

### Enable in ZED Config

```yaml
sensors.sensors_image_sync: true    # Sync with camera frames
sensors.publish_imu_tf: true         # Publish TF transform
```

### IMU Relay Configuration

```xml
<node pkg="topic_tools" exec="relay" name="zed_imu_relay">
  <remap from="/sensing/camera/zedxm/imu/data" to="/sensing/imu/zed/imu_raw"/>
</node>
```

### Advantages

- **Factory-calibrated**: No manual calibration needed
- **Synchronized**: Hardware-synced with camera frames (precise timestamps)
- **Higher rate**: 400 Hz vs 100 Hz (MPU9250)
- **More stable**: Professional-grade sensor, lower drift

### Test

```bash
# Launch ZED camera
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxm

# Verify IMU topic
ros2 topic list | grep imu
# Expected: /zedxm/zed_node/imu/data

# Check rate
ros2 topic hz /zedxm/zed_node/imu/data  # ~400 Hz
```

### Troubleshooting

**No IMU data**:
- Check `sensors.sensors_image_sync: true` in ZED config
- Verify ZED SDK version supports IMU (5.1.2+)
- Not all ZED models have built-in IMU

**Relay not working**:
- Check relay node running: `ros2 node list | grep relay`
- Verify topic exists: `ros2 topic list | grep imu`

## IMU Data Format

Both IMUs publish standard `sensor_msgs/Imu`:

```yaml
header:
  stamp: {sec, nanosec}
  frame_id: "mpu9250_link" or "zed_imu_link"

orientation: {x, y, z, w}           # Quaternion
angular_velocity: {x, y, z}         # rad/s
linear_acceleration: {x, y, z}      # m/s²
```

## Quick Reference

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
