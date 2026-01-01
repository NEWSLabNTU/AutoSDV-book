# Integration Walkthrough: Robin-W LiDAR

This guide walks you through how a sensor is integrated into AutoSDV, using the **Seyond Robin-W LiDAR** as a concrete example. By the end, you'll understand how any sensor is integrated from hardware to software.

## The Physical Sensor

**Seyond Robin-W** is a solid-state LiDAR sensor:
- Forward-facing (120° × 25° field of view)
- Range: up to 200m
- Mounted on the front dock of the vehicle
- Connects via Ethernet (IP: 172.168.1.10)
- Powered by 12-24V DC from vehicle power supply

When the sensor is running, it streams 3D point cloud data over the network at ~10 Hz.

## The Driver Package

The driver converts the sensor's raw data into ROS 2 messages.

**Location**: `src/sensor_component/external/seyond_ros_driver/`

This is a git submodule (external package) that:
1. Connects to Robin-W over network (172.168.1.10)
2. Receives point cloud data
3. Publishes to ROS topic: `/robin_lidar/points_raw`

The driver outputs **PointXYZIRC** format (compatible with Autoware):
- `x, y, z` - 3D position
- `intensity` - Reflection strength
- `return_type` - First/last return
- `ring` - Laser channel ID

## How It's Configured

To integrate Robin-W into the vehicle, we need to tell AutoSDV:
1. Where the sensor is physically mounted
2. How to start the driver
3. What network settings to use

### Physical Position

**File**: `src/sensor_kit/autosdv_sensor_kit_description/config/sensor_kit_calibration.yaml`

```yaml
sensor_kit_base_link:
  robin_lidar_link:
    x: 0.0        # Position: at front center
    y: 0.0
    z: 0.15       # 15cm above sensor kit base
    roll: 3.14159   # 180° rotation (see explanation below)
    pitch: -1.5708  # -90° rotation
    yaw: 0.0
```

This defines a **coordinate frame** called `robin_lidar_link` that represents the sensor's position and orientation on the vehicle.

#### Why the Strange Rotation?

Robin-W uses different coordinate axes than ROS standard:

**Robin-W native**:
```
  X = Up
  Y = Right
  Z = Forward
```

**ROS standard (REP-103)**:
```
  X = Forward
  Y = Left
  Z = Up
```

To convert Robin-W coordinates → ROS standard:
1. **Roll 180°**: Flip the sensor upside down
2. **Pitch -90°**: Rotate so forward axis aligns

Without this transformation, the point cloud would appear upside down and rotated in RViz!

### Robot Description (URDF)

**File**: `src/sensor_kit/autosdv_sensor_kit_description/urdf/sensor_kit.xacro`

```xml
<!-- Robin-W LiDAR -->
<link name="robin_lidar_link"/>

<joint name="robin_lidar_joint" type="fixed">
  <origin
    xyz="${calibration['sensor_kit_base_link']['robin_lidar_link']['x']} ..."
    rpy="${calibration['sensor_kit_base_link']['robin_lidar_link']['roll']} ..."
  />
  <parent link="sensor_kit_base_link"/>
  <child link="robin_lidar_link"/>
</joint>
```

This creates the `robin_lidar_link` frame in the robot's TF tree using the calibration values above.

### Driver Launch Configuration

**File**: `src/sensor_kit/autosdv_sensor_kit_launch/launch/lidar.launch.xml`

```xml
<group if="$(eval &quot;'$(var lidar_model)' == 'robin-w'&quot;)">

  <!-- Launch Robin-W driver -->
  <include file="$(find-pkg-share seyond_ros_driver)/launch/robin_w.launch.py">
    <arg name="config" value="$(var sensor_model_param_path)/robin_lidar.param.yaml"/>
  </include>

  <!-- Remap topic to Autoware standard -->
  <remap from="/robin_lidar/points_raw"
         to="/sensing/lidar/robin_lidar/points_raw"/>

</group>
```

This says: "When user selects `lidar_model:=robin-w`, start the Robin-W driver and remap its topic to the standard Autoware namespace."

### Driver Parameters

**File**: `src/param/autoware_individual_params/.../robin_lidar.param.yaml`

```yaml
/**:
  ros__parameters:
    ip_address: "172.168.1.10"    # Robin-W network address
    port: 2368                     # UDP port
    frame_id: "robin_lidar_link"   # TF frame name
    min_range: 0.5                 # Filter points closer than 0.5m
    max_range: 200.0               # Filter points farther than 200m
```

These are runtime parameters passed to the driver when it starts.

## How Data Flows

When you launch AutoSDV with `lidar_model:=robin-w`:

```
1. Robin-W Hardware
   │ Streams point cloud over UDP
   ▼
2. seyond_ros_driver Node
   │ Converts to ROS PointCloud2 message
   │ Publishes to /robin_lidar/points_raw
   ▼
3. Topic Remapping
   │ Remaps to /sensing/lidar/robin_lidar/points_raw
   ▼
4. TF Transform
   │ Applies rotation (roll=180°, pitch=-90°)
   │ Transforms points: Robin-W coords → ROS coords
   ▼
5. Autoware Localization/Perception
   │ Uses point cloud for NDT localization
   │ Detects obstacles with CenterPoint
   ▼
6. Visualization (RViz)
   │ Shows correctly oriented point cloud
```

## How It Appears in the System

### Topic Namespace

```bash
$ ros2 topic list | grep lidar
/sensing/lidar/robin_lidar/points_raw
```

All sensor data follows the pattern: `/sensing/[type]/[name]/[data]`

### TF Tree

```bash
$ ros2 run tf2_tools view_frames
```

This generates a PDF showing:
```
base_link
  └─ sensor_kit_base_link
       └─ robin_lidar_link  ← Robin-W sensor frame
```

The transform from `sensor_kit_base_link` → `robin_lidar_link` uses the calibration values (position + rotation).

### Verification

Check that Robin-W is working:

```bash
# Topic publishing at ~10 Hz?
ros2 topic hz /sensing/lidar/robin_lidar/points_raw

# Point cloud has data?
ros2 topic echo /sensing/lidar/robin_lidar/points_raw --once

# Transform exists?
ros2 run tf2_ros tf2_echo sensor_kit_base_link robin_lidar_link
```

## The Big Picture

Every sensor integration follows this pattern:

1. **Physical Layer**
   - Mount sensor on vehicle
   - Connect power and communication (USB/Ethernet/etc.)

2. **Driver Layer** (`src/sensor_component/`)
   - ROS 2 package that talks to sensor hardware
   - Publishes raw sensor data to topics

3. **Sensor Kit Layer** (`src/sensor_kit/`)
   - Calibration: Where is the sensor? (position + rotation)
   - URDF: Add sensor to robot description
   - Launch: When to start the driver
   - Parameters: Runtime configuration

4. **Data Flow**
   - Driver publishes → Topics remapped → TF transforms applied → Autoware uses data

## Key Takeaways

- **Coordinate transformation matters**: Robin-W's non-standard coordinates require roll=180°, pitch=-90°
- **Everything is a coordinate frame**: `robin_lidar_link` represents the sensor in 3D space
- **Topic remapping standardizes naming**: Driver outputs → `/sensing/` namespace
- **Two-layer system**: Drivers (sensor component) + Integration (sensor kit)

## Next Steps

Now that you understand how Robin-W is integrated:

- **Try other sensors**: [LiDAR](./lidar.md), [Camera](./camera.md), [IMU](./imu.md), [GNSS](./gnss.md)
- **Add a new sensor**: [Adding Sensors Guide](./adding-sensor.md)
- **Troubleshoot issues**: Check network connectivity, verify TF transforms, monitor topics
