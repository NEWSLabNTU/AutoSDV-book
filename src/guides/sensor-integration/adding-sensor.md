# Adding a New Sensor

Quick checklist for integrating a new sensor into AutoSDV.

## Before You Start

**Understand the system first**: Read [Integration Walkthrough](./integration-walkthrough.md) to see how Robin-W LiDAR is integrated. This gives you the complete picture of how sensors work in AutoSDV.

**Prerequisites**:
- Physical sensor hardware
- Sensor specs (power, interface, data format)
- ROS 2 driver package (vendor-provided or custom)

## Integration Checklist

### 1. Get the Driver Working

```bash
# Add driver package to src/sensor_component/external/
cd src/sensor_component/external/
git submodule add <driver_repository_url>

# Build standalone
colcon build --packages-select <driver_package>
source install/setup.bash

# Test independently
ros2 launch <driver_package> <launch_file>

# Verify topic publishes data
ros2 topic hz /<sensor_topic>
ros2 topic echo /<sensor_topic> --once
```

✅ **Checkpoint**: Driver runs and publishes data

### 2. Measure Physical Position

Measure sensor position relative to `sensor_kit_base_link`:
- X: forward/backward (meters)
- Y: left/right (meters)
- Z: up/down (meters)

Check if sensor uses non-standard coordinates (like Robin-W):
- Calculate rotation: roll, pitch, yaw (radians)
- See [Integration Walkthrough](./integration-walkthrough.md#why-the-strange-rotation) for example

### 3. Add to Calibration

**File**: `src/sensor_kit/autosdv_sensor_kit_description/config/sensor_kit_calibration.yaml`

```yaml
sensor_kit_base_link:
  your_sensor_link:
    x: 0.15       # Your measured values
    y: 0.0
    z: 0.20
    roll: 0.0     # Rotation if needed
    pitch: 0.0
    yaw: 0.0
```

### 4. Add to URDF

**File**: `src/sensor_kit/autosdv_sensor_kit_description/urdf/sensor_kit.xacro`

```xml
<!-- Your Sensor -->
<link name="your_sensor_link"/>

<joint name="your_sensor_joint" type="fixed">
  <origin
    xyz="${calibration['sensor_kit_base_link']['your_sensor_link']['x']} ..."
    rpy="${calibration['sensor_kit_base_link']['your_sensor_link']['roll']} ..."
  />
  <parent link="sensor_kit_base_link"/>
  <child link="your_sensor_link"/>
</joint>
```

### 5. Add to Launch File

**File**: `src/sensor_kit/autosdv_sensor_kit_launch/launch/<sensor_type>.launch.xml`

```xml
<group if="$(eval &quot;'$(var sensor_model)' == 'your_sensor'&quot;)">

  <!-- Launch driver -->
  <include file="$(find-pkg-share your_driver)/launch/your_sensor.launch.py">
    <arg name="config" value="$(var sensor_model_param_path)/your_sensor.param.yaml"/>
  </include>

  <!-- Remap to Autoware standard -->
  <remap from="/<driver_topic>" to="/sensing/<type>/<name>/<data>"/>

</group>
```

### 6. Create Parameter File

**File**: `src/param/autoware_individual_params/.../your_sensor.param.yaml`

```yaml
/**:
  ros__parameters:
    frame_id: "your_sensor_link"
    # Add sensor-specific parameters
```

### 7. Add Launch Argument

**File**: `src/launcher/autosdv_launch/launch/autosdv.launch.yaml`

```python
sensor_model_param = DeclareLaunchArgument(
    'sensor_model',
    choices=['existing', 'models', 'your_sensor'],  # Add your sensor
)
```

### 8. Test Integration

```bash
# Rebuild
make build

# Launch with your sensor
make launch ARGS="sensor_model:=your_sensor"

# Verify topic in Autoware namespace
ros2 topic list | grep /sensing
ros2 topic hz /sensing/<type>/<name>/<data>

# Verify TF transform
ros2 run tf2_tools view_frames
evince frames.pdf  # Check your_sensor_link exists

# Check transform values
ros2 run tf2_ros tf2_echo sensor_kit_base_link your_sensor_link
```

✅ **Checkpoint**: Sensor works in full AutoSDV system, data flows correctly

## Common Issues

**Wrong orientation in RViz**:
- Verify calibration rotation values (roll, pitch, yaw)
- Check sensor coordinate system vs ROS standard
- See Robin-W example for coordinate transformation

**Topic not found**:
- Check `<remap>` in launch file
- Verify driver topic name matches
- Use absolute paths: `/sensing/...`

**TF transform missing**:
- Rebuild after URDF changes: `make build`
- Check URDF syntax: `check_urdf sensor_kit.urdf`

**No data published**:
- Test driver standalone first
- Check network/USB connection
- Verify driver parameters

## Next Steps

- **Sensor-specific guides**: [LiDAR](./lidar.md), [Camera](./camera.md), [IMU](./imu.md), [GNSS](./gnss.md)
- **Troubleshooting**: Check logs in `play_log/latest/`, enable debug logging
- **Documentation**: Document your sensor configuration for future reference
