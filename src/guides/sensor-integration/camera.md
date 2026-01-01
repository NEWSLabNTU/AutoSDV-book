# Camera Sensors

Configuration details for supported camera models.

## Supported Models

| Model | Type | Resolution | Frame Rate | Features | Config |
|-------|------|------------|------------|----------|--------|
| **ZED X Mini** | Stereo | 1920×1200 | 15-60 FPS | Depth, IMU, AI Detection | `camera_model:=zedxm` |
| **USB Camera** | Monocular | Varies | 30 FPS | Basic imaging | `camera_model:=usb` |

## ZED X Mini

### Prerequisites

**Hardware**: NVIDIA Jetson with ZED Link Duo capture card (GMSL connection)
**Software**: ZED SDK 5.1.2 (see Installation Guide)

### Connection

```
ZED X Mini → GMSL Cable → ZED Link Duo → PCIe → Jetson
```

Verify detection:
```bash
lspci | grep NVIDIA        # Check ZED Link card
ZED_Explorer               # Test camera with GUI
```

### Driver Package

**Location**: `src/sensor_component/external/zed-ros2-wrapper/`
**Package**: `zed_components` (composable node architecture)
**Plugin**: `stereolabs::ZedCamera`

### Special: Namespace Workaround

ZED Python launch file doesn't respect XML namespaces. **Solution**: Load composable node directly in XML.

**Correct configuration**:
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

This creates topics at `/sensing/camera/zedxm/*` correctly.

### Built-in IMU

ZED has integrated 6-axis IMU (400 Hz, factory-calibrated).

**Enable in ZED config**:
```yaml
sensors.sensors_image_sync: true
sensors.publish_imu_tf: true
```

**IMU relay pattern**:
```
ZED Driver → /sensing/camera/zedxm/imu/data
               ↓ (relay node)
          /sensing/imu/zed/imu_raw → Autoware EKF
```

Relay configuration:
```xml
<node pkg="topic_tools" exec="relay" name="zed_imu_relay">
  <remap from="/sensing/camera/zedxm/imu/data" to="/sensing/imu/zed/imu_raw"/>
</node>
```

### Object Detection

ZED SDK includes AI-based object detection (persons, vehicles, animals).

**Enable**:
```xml
<param name="object_detection.od_enabled" value="true"/>
<param name="object_detection.model" value="MULTI_CLASS_BOX_MEDIUM"/>
<param name="object_detection.confidence_threshold" value="50.0"/>
```

**Output**: `/sensing/camera/zedxm/obj_det/objects` (ZED format)

**Disable for performance**:
```bash
make launch ARGS="enable_zed_object_detection:=false"
```

### Configuration Options

**Depth Mode**:
```yaml
depth.depth_mode:
  NEURAL        # Best quality (GPU-intensive)
  ULTRA         # High quality
  QUALITY       # Standard
  PERFORMANCE   # Fast
```

**Frame Rate**:
```yaml
general.grab_frame_rate: 15  # or 30, 60
```

**Resolution**:
```yaml
general.grab_resolution:
  HD2K    # 2208×1242 (highest)
  HD1080  # 1920×1080
  HD720   # 1280×720
  VGA     # 672×376 (lowest latency)
```

### Test Standalone

```bash
# ZED SDK test
ZED_Explorer

# ROS 2 wrapper test
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxm

# Verify topics
ros2 topic list | grep zedxm
ros2 topic hz /zedxm/zed_node/rgb/image_rect_color  # ~15-30 Hz
```

### Troubleshooting

**Camera not detected**:
- Check `lspci | grep NVIDIA` shows ZED Link
- Check `/usr/local/zed/settings/version.txt` shows 5.1.2
- Power cycle camera and Jetson

**Wrong namespace**:
- Use composable node loading (not Python launch file)
- Use ABSOLUTE namespace `/sensing/camera/zedxm`

**VNC issues**:
- Use TurboVNC + VirtualGL for hardware acceleration
- Standard VNC doesn't support GPU

## USB Cameras

### Hardware

- Standard USB webcams (UVC-compatible)
- Connected via USB to Jetson
- Detected as `/dev/video*`

### Driver

**Package**: `v4l2_camera`
**Installation**: `sudo apt install ros-humble-v4l2-camera`

### Configuration

```xml
<node pkg="v4l2_camera" exec="v4l2_camera_node" name="usb_camera">
  <param name="video_device" value="/dev/video0"/>
  <param name="image_size" value="[640, 480]"/>
  <param name="pixel_format" value="YUYV"/>
  <param name="camera_frame_id" value="usb_camera_link"/>
</node>
```

### Test Standalone

```bash
# List available cameras
v4l2-ctl --list-devices

# Test camera
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
```

## Quick Reference

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
