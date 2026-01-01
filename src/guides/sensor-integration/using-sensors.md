# Using Sensors

This guide shows you how to launch AutoSDV with different sensor configurations.

## Sensor Suites

AutoSDV supports predefined sensor combinations called **sensor suites**. Each suite is a tested combination of sensors that work well together.

| Sensor Suite | LiDAR | Camera | IMU | GNSS | Use Case |
|--------------|-------|--------|-----|------|----------|
| `robin_zed` | Robin-W | ZED X Mini | ZED IMU | u-blox ZED-F9R | **Recommended** - Complete outdoor suite |
| `vlp32c_zed` | Velodyne VLP-32C | ZED X Mini | ZED IMU | u-blox ZED-F9R | Alternative LiDAR option |
| `cube1_zed` | Blickfeld Cube1 | ZED X Mini | ZED IMU | u-blox ZED-F9R | Solid-state LiDAR option |

## Quick Start

### Launch with Sensor Suite

```bash
# Launch with Robin-W LiDAR + ZED camera (recommended)
make launch ARGS="sensor_suite:=robin_zed"

# Launch with Velodyne LiDAR + ZED camera
make launch ARGS="sensor_suite:=vlp32c_zed"

# Launch with Blickfeld LiDAR + ZED camera
make launch ARGS="sensor_suite:=cube1_zed"
```

### Verify Sensors are Working

After launching, check that sensors are publishing data:

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

### Visualize in RViz

```bash
# Open RViz with AutoSDV configuration
make run-rviz
```

You should see:
- Point cloud from LiDAR (white/colored points)
- Camera image overlay
- Vehicle pose in the map
- TF frames showing sensor positions

## Common Scenarios

### Outdoor with RTK GPS

For high-precision outdoor localization with RTK corrections:

```bash
# Enable NTRIP for RTK positioning
make launch ARGS="sensor_suite:=robin_zed use_ntrip:=true"
```

This connects to the e-GNSS Taiwan VRS for ~2cm accuracy GPS.

### Indoor (No GPS)

For indoor operation without GPS signal:

```bash
# Disable GPS, use NDT localization only
make launch ARGS="sensor_suite:=robin_zed use_gnss:=false"
```

You'll need to manually set the initial pose in RViz using "2D Pose Estimate" tool.

### Using Isaac Visual SLAM

For camera-based localization instead of LiDAR NDT:

```bash
# Use Isaac Visual SLAM for pose estimation
make launch ARGS="sensor_suite:=robin_zed pose_source:=isaac"
```

## Custom Sensor Selection

Instead of using predefined suites, you can select individual sensors:

```bash
# Custom combination
make launch ARGS="lidar_model:=robin-w camera_model:=zedxm imu_source:=zed gnss_receiver:=ublox"

# Minimal setup (LiDAR only)
make launch ARGS="lidar_model:=robin-w camera_model:=none imu_source:=mpu9250 use_gnss:=false"
```

### Available Options

**LiDAR Models:**
- `robin-w` - Robin-W 360° LiDAR (default)
- `vlp32c` - Velodyne VLP-32C
- `cube1` - Blickfeld Cube1

**Camera Models:**
- `zedxm` - ZED X Mini stereo camera (default)
- `usb` - USB webcam
- `none` - No camera

**IMU Sources:**
- `zed` - ZED built-in IMU (default)
- `mpu9250` - External MPU9250 IMU

**GNSS Receivers:**
- `ublox` - u-blox ZED-F9R (RTK-capable, default)
- `garmin` - Garmin GPS 18x (standard GPS)
- `septentrio` - Septentrio receiver

## Troubleshooting

### No sensor data

**Check topics exist:**
```bash
ros2 topic list | grep /sensing
```

If topics are missing, the sensor driver may have failed to start. Check logs:
```bash
# View latest logs
tail -f play_log/latest/*.log
```

### Sensor not detected

**Check hardware connection:**
```bash
# For LiDAR (check network)
ping 172.168.1.10  # Robin-W
ping 192.168.1.201  # Velodyne

# For camera (check ZED SDK)
ZED_Explorer

# For GNSS (check USB device)
ls -l /dev/ublox-gps
```

### Transform errors in RViz

If RViz shows "No transform from X to Y", rebuild the project:
```bash
make build
```

## Next Steps

- **Learn how sensors are integrated**: [Integration Walkthrough](./integration-walkthrough.md)
- **Configure specific sensors**: [LiDAR](./lidar.md), [Camera](./camera.md), [IMU](./imu.md), [GNSS](./gnss.md)
- **Add a new sensor**: [Adding Sensors Guide](./adding-sensor.md)
