# LiDAR Sensors

Configuration details for supported LiDAR models.

## Supported Models

| Model | Type | FOV | Range | Network | Config |
|-------|------|-----|-------|---------|--------|
| **Seyond Robin-W** | Solid-state | 120° × 25° | 200m | 172.168.1.10 | `lidar_model:=robin-w` |
| **Velodyne VLP-32C** | Spinning | 360° × 40° | 200m | 192.168.1.201 | `lidar_model:=vlp32c` |
| **Blickfeld Cube1** | Solid-state | 70° × 30° | 150m | 192.168.26.26 | `lidar_model:=cube1` |

## Robin-W

### Network Setup

**LiDAR IP**: 172.168.1.10 (fixed)
**Jetson IP**: 172.168.1.100/24 (configure on same subnet)

```bash
# Configure Jetson network interface
sudo ip addr add 172.168.1.100/24 dev eth0
sudo ip link set eth0 up

# Test connectivity
ping 172.168.1.10
```

### Coordinate Transformation

**Important**: Robin-W requires rotation to match ROS standard coordinates.

**Native coordinates**: X=up, Y=right, Z=forward
**ROS standard**: X=forward, Y=left, Z=up

**Required calibration**:
```yaml
robin_lidar_link:
  roll: 3.14159    # 180° flip
  pitch: -1.5708   # -90° rotation
  yaw: 0.0
```

See [Integration Walkthrough](./integration-walkthrough.md) for detailed explanation.

### Driver Package

**Location**: `src/sensor_component/external/seyond_ros_driver/`
**Point Format**: PointXYZIRC (Autoware-compatible)

### Test Standalone

```bash
colcon build --packages-select seyond_ros_driver
source install/setup.bash
ros2 launch seyond_ros_driver robin_w.launch.py

# Verify
ros2 topic hz /robin_lidar/points_raw  # ~10 Hz
```

### Troubleshooting

**No data**: Check `ping 172.168.1.10` succeeds
**Wrong orientation**: Verify roll=3.14159, pitch=-1.5708 in calibration

## Velodyne VLP-32C

### Network Setup

**LiDAR IP**: 192.168.1.201 (factory default)
**Jetson IP**: 192.168.1.100/24

```bash
# Configure Jetson
sudo ip addr add 192.168.1.100/24 dev eth0

# Test connectivity
ping 192.168.1.201
```

### Coordinate System

**Standard ROS coordinates** - No rotation needed:
```yaml
velodyne_link:
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

### Driver Package

**Location**: `src/sensor_component/external/velodyne/`
**Installation**: `sudo apt install ros-humble-velodyne`
**Point Format**: PointXYZIR (standard Velodyne)

### Advanced: Dual Return Mode

For better performance in rain/fog:
```yaml
/**:
  ros__parameters:
    rpm: 600
    return_type: "dual"  # Both strongest and last returns
```

### Test Standalone

```bash
ros2 launch velodyne_pointcloud velodyne_driver_node-VLP32C-launch.py device_ip:=192.168.1.201

# Verify
ros2 topic hz /velodyne_points  # ~10-20 Hz
```

### Troubleshooting

**Packet loss**: Increase UDP buffer size:
```bash
sudo sysctl -w net.core.rmem_max=26214400
```

**Gaps in scan**: Configure CycloneDDS buffers (see Installation Guide)

## Blickfeld Cube1

### Network Setup

**LiDAR IP**: 192.168.26.26 (fixed)
**Jetson IP**: 192.168.26.1/24

```bash
# Configure Jetson
sudo ip addr add 192.168.26.1/24 dev eth0
sudo ip link set eth0 up

# Test connectivity
ping 192.168.26.26
```

### Coordinate System

**Standard ROS coordinates** - No rotation needed:
```yaml
bf_lidar_link:
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

### Driver Package

**Location**: `src/sensor_component/external/ros2_blickfeld_driver_src-v1.5.5/`
**Requires**: Blickfeld Scanner Library 2.20.6-newslab1 (install via `just blickfeld`)

### Test Standalone

```bash
colcon build --packages-select blickfeld_driver
source install/setup.bash
ros2 launch blickfeld_driver live_scanner_node.launch.py

# Verify
ros2 topic hz /bf_lidar/points_raw
```

### Troubleshooting

**Connection failed**: Verify Scanner Library installed:
```bash
dpkg -l | grep blickfeld
```

**EULA error**: Run `just blickfeld` to accept license

## Multi-LiDAR Setup

Combine multiple LiDARs for increased coverage:

```yaml
# sensor_kit_calibration.yaml
sensor_kit_base_link:
  robin_lidar_link:    # Front-facing
    x: 0.15
    z: 0.15
    roll: 3.14159
    pitch: -1.5708

  velodyne_link:       # Top 360°
    x: 0.0
    z: 0.30
    roll: 0.0
    pitch: 0.0
```

Configure fusion in `pointcloud_preprocessor.launch.py`:
```python
"input_topics": [
    "/sensing/lidar/robin_lidar/points_raw",
    "/sensing/lidar/velodyne/points_raw",
]
```

## Quick Reference

```bash
# Network tests
ping 172.168.1.10    # Robin-W
ping 192.168.1.201   # Velodyne
ping 192.168.26.26   # Blickfeld

# Topic verification
ros2 topic list | grep /sensing/lidar
ros2 topic hz /sensing/lidar/robin_lidar/points_raw

# TF verification
ros2 run tf2_ros tf2_echo sensor_kit_base_link robin_lidar_link

# Network monitoring
sudo iftop -i eth0
```
