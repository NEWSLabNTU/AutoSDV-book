# Sensor Troubleshooting

This guide provides solutions to common sensor integration issues in AutoSDV.

## Quick Diagnostic Commands

Before diving into specific issues, run these commands to gather information:

```bash
# List all active topics
ros2 topic list

# Check sensor topics specifically
ros2 topic list | grep /sensing

# View TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Check node status
ros2 node list

# View logs with debug level
ros2 launch autosdv_launch autosdv.launch.yaml --log-level DEBUG
```

---

## General Issues

### No Sensor Data

**Symptoms**:
- Topics exist but `ros2 topic echo` shows nothing
- `ros2 topic hz` shows 0 Hz
- Driver launches but no data published

**Diagnostic Steps**:

1. **Check physical connections**:
   ```bash
   # USB devices
   lsusb
   # Should list your sensor

   # Serial devices
   ls -l /dev/tty*
   # Should show device (e.g., /dev/ttyUSB0)

   # Network devices
   ip addr show
   # Verify Ethernet interface up

   # Test network sensors
   ping <sensor_ip>
   # Should respond with <5ms latency
   ```

2. **Verify device permissions**:
   ```bash
   # Add user to dialout group (for serial)
   sudo usermod -aG dialout $USER

   # Add user to video group (for cameras)
   sudo usermod -aG video $USER

   # Log out and back in for changes to take effect
   ```

3. **Check driver logs**:
   ```bash
   # Launch with debug logging
   ros2 launch autosdv_launch autosdv.launch.yaml --log-level DEBUG

   # Or check specific node
   ros2 node info /sensing/lidar/robin_lidar/driver_node
   ```

4. **Verify power**:
   - Check sensor LED indicators
   - Measure voltage with multimeter
   - Ensure power supply has sufficient current

**Solutions by Sensor Type**:
- **LiDAR**: Check network settings, IP configuration
- **Camera**: Verify ZED SDK installed, GMSL connection
- **IMU**: Check I2C bus, device address
- **GNSS**: Ensure clear sky view, wait for satellite acquisition

---

### Wrong Coordinate Frame / Sensor in Wrong Location

**Symptoms**:
- Sensor data visible in RViz but in wrong position
- Point cloud upside down or rotated incorrectly
- Transform errors in RViz

**Diagnostic Steps**:

1. **Verify TF transforms**:
   ```bash
   # Generate TF tree
   ros2 run tf2_tools view_frames
   evince frames.pdf

   # Check specific transform
   ros2 run tf2_ros tf2_echo base_link sensor_link
   # Compare output with calibration values
   ```

2. **Check calibration file**:
   ```bash
   # View current calibration
   cat src/sensor_kit/autosdv_sensor_kit_description/config/sensor_kit_calibration.yaml

   # Look for your sensor's x, y, z, roll, pitch, yaw values
   ```

**Solutions**:

1. **Fix calibration values**:
   ```yaml
   # Edit: sensor_kit_calibration.yaml
   sensor_kit_base_link:
     your_sensor_link:
       x: 0.15      # Forward/backward (meters)
       y: 0.02      # Left/right (meters)
       z: 0.20      # Up/down (meters)
       roll: 0.0    # Rotation around X-axis (radians)
       pitch: 0.0   # Rotation around Y-axis (radians)
       yaw: 0.0     # Rotation around Z-axis (radians)
   ```

2. **Understand coordinate systems**:
   - **ROS Standard (REP-103)**: X=forward, Y=left, Z=up
   - **Some sensors use different conventions** (see sensor guides)
   - Calculate rotation needed to align sensor → ROS standard

3. **Iteratively adjust in RViz**:
   - Launch system and view in RViz
   - Adjust roll/pitch/yaw in calibration file
   - Rebuild and relaunch to see changes
   - Repeat until correct

**Common Rotations**:
- **Robin-W LiDAR**: roll=3.14159, pitch=-1.5708, yaw=0.0
- **ZED Camera**: roll=0.0, pitch=0.0, yaw=0.0 (standard)

---

### Topic Namespace Issues

**Symptoms**:
- Topics appear in unexpected namespaces
- Cannot find sensor topics in `/sensing/`
- Topic remapping not working

**Diagnostic Steps**:

1. **Check actual topic names**:
   ```bash
   ros2 topic list | grep -E '(lidar|camera|imu|gnss)'
   # Compare with expected: /sensing/[type]/[name]/[data]
   ```

2. **Verify launch file remapping**:
   ```bash
   # Check sensor kit launch files
   cat src/sensor_kit/autosdv_sensor_kit_launch/launch/lidar.launch.xml
   # Look for <remap> tags
   ```

**Solutions**:

1. **Add or fix topic remapping**:
   ```xml
   <!-- In sensor kit launch file -->
   <remap from="/driver_topic_name" to="/sensing/lidar/sensor_name/points_raw"/>
   ```

2. **Use absolute topic paths**:
   ```xml
   <!-- Bad (relative) -->
   <remap from="points" to="points_raw"/>

   <!-- Good (absolute) -->
   <remap from="/sensor/points" to="/sensing/lidar/sensor/points_raw"/>
   ```

3. **For ZED camera namespace issues**: Use composable node loading (see [Camera Sensors Guide](./camera.md#special-namespace-workaround))

---

### Performance Issues / High CPU Usage

**Symptoms**:
- System lag or slow response
- High CPU usage (check with `htop`)
- Low frame rate from sensors
- Dropped messages

**Diagnostic Steps**:

1. **Check topic bandwidth**:
   ```bash
   # Monitor data rate
   ros2 topic bw /sensing/lidar/robin_lidar/points_raw

   # Check all sensor topics
   ros2 topic bw /sensing/*
   ```

2. **Check CPU usage**:
   ```bash
   # Real-time process monitor
   htop

   # GPU usage (NVIDIA)
   nvidia-smi
   ```

3. **Verify CycloneDDS configuration**:
   ```bash
   # Check kernel buffers
   sysctl net.core.rmem_max
   sysctl net.ipv4.ipfrag_time
   sysctl net.ipv4.ipfrag_high_thresh

   # Should be configured per Installation Guide
   ```

**Solutions**:

1. **Configure CycloneDDS buffers** (if not done):
   ```bash
   cd setup
   just cyclonedds-sysctl
   ```

2. **Reduce sensor data rate**:
   ```yaml
   # Edit sensor parameters
   # For cameras:
   general.grab_frame_rate: 15  # Reduce from 30

   # For LiDAR:
   downsample_factor: 2  # Reduce points by half
   ```

3. **Disable heavy features**:
   ```bash
   # Disable ZED object detection
   ros2 launch autosdv_launch autosdv.launch.yaml enable_zed_object_detection:=false
   ```

4. **Use lightweight sensor suite**:
   ```bash
   # VLP-32C + ZED IMU only (no camera processing)
   ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=vlp32c_zed_imu
   ```

---

## Sensor-Specific Issues

### LiDAR

#### No Point Cloud

**Symptoms**: LiDAR driver launches but no points published

**Solutions**:
1. **Check network connectivity** (Ethernet LiDARs):
   ```bash
   ping <lidar_ip>
   # Robin-W: 172.168.1.10
   # Velodyne: 192.168.1.201
   # Blickfeld: 192.168.26.26
   ```

2. **Configure Jetson IP** on same subnet:
   ```bash
   # For Robin-W (172.168.1.x)
   sudo ip addr add 172.168.1.100/24 dev eth0
   sudo ip link set eth0 up

   # For Blickfeld (192.168.26.x)
   sudo ip addr add 192.168.26.1/24 dev eth0
   sudo ip link set eth0 up
   ```

3. **Check firewall**:
   ```bash
   # Disable firewall temporarily for testing
   sudo ufw disable

   # Or allow specific ports
   sudo ufw allow from <lidar_ip>
   ```

#### TensorRT Compilation Takes Forever

**Symptoms**: First launch takes 10-30 minutes with TensorRT messages

**This is NORMAL** for first launch. TensorRT compiles models for your specific GPU.

**Solutions**:
- Be patient - only happens once
- Models cached in `~/.autoware/models/`
- Subsequent launches are fast

#### Point Cloud in Wrong Orientation

**Solution**: Check coordinate frame rotation (especially Robin-W, which requires roll=180°, pitch=-90°)

See [LiDAR Sensors - Coordinate Transformation](./lidar.md#coordinate-transformation)

---

### Camera (ZED)

#### Camera Not Detected

**Symptoms**: ZED_Explorer shows "No camera detected"

**Solutions**:
1. **Check GMSL connection**:
   ```bash
   # Verify ZED Link card
   lspci | grep NVIDIA
   # Should show ZED Link Duo
   ```

2. **Power cycle camera**:
   - Unplug camera
   - Wait 10 seconds
   - Reconnect
   - Reboot Jetson

3. **Verify ZED SDK installation**:
   ```bash
   cat /usr/local/zed/settings/version.txt
   # Should show 5.1.2
   ```

4. **Check permissions**:
   ```bash
   sudo usermod -aG video $USER
   # Log out and back in
   ```

#### VNC/Remote Access Issues

**Problem**: ZED camera doesn't work over standard VNC

**Solution**: Use TurboVNC + VirtualGL for hardware acceleration

```bash
# Install TurboVNC
sudo apt install turbovnc virtualgl

# Start VNC with VirtualGL
vglrun /opt/TurboVNC/bin/vncserver

# Launch apps with vglrun
vglrun ZED_Explorer
vglrun rviz2
```

#### Namespace Problems

**Problem**: Topics in wrong namespace, can't remap

**Solution**: Use composable node loading instead of Python launch file

See [Camera Sensors - Namespace Workaround](./camera.md#special-namespace-workaround)

---

### IMU

#### MPU9250 Not Detected

**Symptoms**: i2cdetect doesn't show device

**Solutions**:
1. **Check I2C bus**:
   ```bash
   # List I2C buses
   ls /dev/i2c-*

   # Scan bus 1
   sudo i2cdetect -y -r 1
   # Should show device at 0x68 or 0x69
   ```

2. **Verify wiring**:
   - VCC → 3.3V (not 5V!)
   - GND → GND
   - SDA → Pin 3 (I2C1 SDA)
   - SCL → Pin 5 (I2C1 SCL)

3. **Check I2C address**:
   ```bash
   # Try alternate address
   sudo i2cdetect -y -r 1
   # Look for 0x68 or 0x69
   ```

#### Noisy IMU Data

**Symptoms**: Large fluctuations, unstable orientation

**Solutions**:
1. **Check IMU configuration**: See [IMU Sensors - Configuration](./imu.md#configuration)
2. **Secure mounting**: Minimize vibration
3. **Add software filtering**: Low-pass filter in driver config

#### ZED IMU Not Publishing

**Symptoms**: ZED camera works but no IMU topic

**Solutions**:
1. **Enable IMU in ZED config**:
   ```yaml
   sensors.sensors_image_sync: true
   sensors.publish_imu_tf: true
   ```

2. **Check IMU relay node**:
   ```bash
   ros2 node list | grep relay
   # Should show IMU relay node running
   ```

---

### GNSS

#### No GNSS Fix

**Symptoms**: Fix status remains -1 (no fix)

**Solutions**:
1. **Ensure clear sky view**: Move to open area (no buildings/trees)
2. **Wait for satellite acquisition**: Cold start takes 1-10 minutes
3. **Check antenna connection**: Verify cable secure
4. **Monitor satellite count**: Need minimum 4 satellites for 3D fix

#### No RTK Fix (u-blox)

**Symptoms**: GPS fix but no RTK (status != 2)

**Solutions**:
1. **Check NTRIP connection**:
   ```bash
   ros2 topic hz /sensing/gnss/ntrip/rtcm
   # Should show ~1 Hz
   ```

2. **Verify internet connection**: NTRIP requires network
3. **Check credentials**: Edit `ntrip.launch.xml` with correct username/password
4. **Wait for convergence**: RTK can take 1-10 minutes
5. **Improve antenna placement**: Higher, less obstruction

#### Garmin GPS Not Working

**Symptoms**: No data from Garmin GPS 18x

**Solutions**:
1. **Start gpsd**:
   ```bash
   sudo /usr/sbin/gpsd -n -G -b /dev/ttyUSB0
   ```

2. **Check device**:
   ```bash
   ls -l /dev/ttyUSB*
   sudo cat /dev/ttyUSB0
   # Should see NMEA sentences
   ```

3. **Set baud rate**:
   ```bash
   sudo stty -F /dev/ttyUSB0 9600
   ```

4. **Test with cgps**:
   ```bash
   cgps
   # Should show GPS data
   ```

---

## Build Errors

### Package Not Found

**Symptoms**: `colcon build` fails with "package 'X' not found"

**Solutions**:
1. **Update rosdep**:
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **Check package.xml dependencies**: Ensure all `<depend>` tags correct

3. **Install missing ROS packages**:
   ```bash
   sudo apt install ros-humble-<package-name>
   ```

### CMake Errors

**Symptoms**: CMake configuration fails

**Solutions**:
1. **Clean build**:
   ```bash
   rm -rf build install log
   colcon build
   ```

2. **Check CMakeLists.txt**: Verify syntax, dependencies

3. **Source ROS 2**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

---

## Getting Help

If issues persist after trying these solutions:

### 1. Enable Debug Logging

```bash
ros2 launch autosdv_launch autosdv.launch.yaml --log-level DEBUG 2>&1 | tee debug.log
```

### 2. Gather Diagnostic Information

```bash
# System info
lsb_release -a
uname -a

# ROS info
ros2 doctor
ros2 wtf

# Topic info
ros2 topic list
ros2 topic hz /sensing/lidar/*/points_raw

# TF tree
ros2 run tf2_tools view_frames
```

### 3. Check GitHub Issues

Search [AutoSDV Issues](https://github.com/NEWSLabNTU/AutoSDV/issues) for similar problems

### 4. Create Issue

When reporting issues, include:
- Hardware setup (sensor models, connections)
- Launch command used
- Full error messages/logs
- Output of diagnostic commands
- Steps to reproduce

---

## Useful Diagnostic Tools

### ROS 2 Tools

```bash
# List nodes
ros2 node list

# Node info
ros2 node info /node_name

# Topic info
ros2 topic info /topic_name
ros2 topic hz /topic_name
ros2 topic bw /topic_name
ros2 topic echo /topic_name

# Parameter inspection
ros2 param list /node_name
ros2 param get /node_name parameter_name

# Service calls
ros2 service list
ros2 service call /service_name service_type "{args}"

# TF debugging
ros2 run tf2_ros tf2_echo frame1 frame2
ros2 run tf2_tools view_frames
```

### System Tools

```bash
# Process monitoring
htop
top

# GPU monitoring
nvidia-smi
watch -n 1 nvidia-smi

# Network
ip addr show
ping <ip>
iftop -i eth0

# USB devices
lsusb
lsusb -t

# Serial devices
ls -l /dev/tty*
sudo dmesg | grep tty

# I2C devices
ls /dev/i2c-*
sudo i2cdetect -y -r 1

# Disk usage
df -h
du -sh *
```

---

## Related Guides

For sensor-specific troubleshooting, see:
- [LiDAR Sensors](./lidar.md) - Robin-W, Velodyne, Blickfeld issues
- [Camera Sensors](./camera.md) - ZED camera troubleshooting
- [IMU Sensors](./imu.md) - MPU9250 and ZED IMU issues
- [GNSS Sensors](./gnss.md) - GNSS and RTK troubleshooting

For integration help:
- [Adding a New Sensor](./adding-sensor.md) - Step-by-step integration guide
- [Using Sensors](./using-sensors.md) - Getting started with sensors
