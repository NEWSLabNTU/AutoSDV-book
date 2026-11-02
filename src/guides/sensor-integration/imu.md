# IMU Sensors

Configuration details for the vehicle's IMU source.

## Supported IMU

The vehicle's IMU is the one built into the ZED X Mini camera: a 6-axis
(accelerometer and gyroscope) sensor, camera-synced, publishing at 400 Hz.
It is selected with `imu_source:=zed`, which every sensor suite does.

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
- **High rate**: 400 Hz
- **Stable**: Professional-grade sensor, low drift

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

The IMU publishes standard `sensor_msgs/Imu`:

```yaml
header:
  stamp: {sec, nanosec}
  frame_id: "zed_imu_link"

orientation: {x, y, z, w}           # Quaternion
angular_velocity: {x, y, z}         # rad/s
linear_acceleration: {x, y, z}      # m/s²
```

## Quick Reference

```bash
# ZED IMU
ros2 topic hz /sensing/imu/zed/imu_raw
ros2 topic echo /sensing/camera/zedxm/imu/data --once

# Verify relay
ros2 node list | grep relay
```
