# GNSS Sensors

Configuration details for supported GNSS receivers.

## Supported Receivers

| Model | Type | Accuracy | RTK Support | Interface | Config |
|-------|------|----------|-------------|-----------|--------|
| **u-blox ZED-F9R** | Multi-band GNSS | ~2cm (RTK) | Yes | USB | `gnss_receiver:=ublox` |
| **Garmin GPS 18x** | Standard GPS | ~3-5m | No | USB (FTDI) | `gnss_receiver:=garmin` |
| **Septentrio** | Multi-band GNSS | ~2cm (RTK) | Yes | USB/Serial | `gnss_receiver:=septentrio` |

**Recommendation**: Use u-blox ZED-F9R for outdoor precision positioning with RTK.

## u-blox ZED-F9R (RTK)

### Hardware

**Board**: SimpleRTK2B Fusion (u-blox ZED-F9R module)
**Antenna**: Must have clear sky view (mount on roof/highest point)
**Connection**: USB to Jetson

### USB Device Setup

Create udev rule for consistent device naming:

**File**: `/etc/udev/rules.d/99-ublox-gps.rules`
```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="ublox-gps", MODE="0666"
```

Apply rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger

# Verify
ls -l /dev/ublox-gps  # Should link to /dev/ttyACMx
```

### Driver Package

**Package**: `ublox_gps` (ROS 2 Humble)
**Installation**: `sudo apt install ros-humble-ublox-gps`

### RTK with NTRIP

RTK (Real-Time Kinematic) achieves ~2cm accuracy using correction data from a base station.

**NTRIP** streams corrections over internet:
```
Internet → NTRIP Client → RTCM Corrections → u-blox Driver → Receiver
```

**Default NTRIP Service**: e-GNSS Taiwan VRS
- Server: 210.241.63.193:81
- Mountpoint: Taiwan
- Coverage: Taiwan region

**Configuration**: `ntrip.launch.xml`
```xml
<node pkg="ntrip_client" exec="ntrip_client_node">
  <param name="host" value="210.241.63.193"/>
  <param name="port" value="81"/>
  <param name="mountpoint" value="Taiwan"/>
  <param name="username" value="[username]"/>
  <param name="password" value="[password]"/>
  <remap from="/ntrip/rtcm" to="/sensing/gnss/ntrip/rtcm"/>
</node>
```

### Enable RTK

```bash
# Launch with NTRIP corrections
make launch ARGS="gnss_receiver:=ublox use_ntrip:=true"
```

### Monitor RTK Status

**Check RTCM corrections**:
```bash
ros2 topic hz /sensing/gnss/ntrip/rtcm  # ~1 Hz
```

**Check fix quality**:
```bash
ros2 topic echo /sensing/gnss/ublox/nav_sat_fix
# Look for status.status: 2 (RTK fix) or 1 (GNSS fix)
```

**Fix status values**:
- `-1`: No fix
- `0`: GPS fix
- `1`: DGPS fix
- `2`: RTK fix (best, ~2cm accuracy)

### Troubleshooting

**No GNSS fix**:
- Ensure clear sky view (outdoor, no obstructions)
- Wait 1-5 minutes for satellite acquisition
- Check antenna connection

**No RTK fix**:
- Verify RTCM topic: `ros2 topic hz /sensing/gnss/ntrip/rtcm`
- Check internet connection (NTRIP requires network)
- Verify NTRIP credentials in `ntrip.launch.xml`
- Wait 1-10 minutes for RTK convergence

**Poor accuracy**:
- Improve antenna placement (higher, less obstruction)
- Reduce multipath (avoid reflective surfaces)
- Test in open area (NTU COSS courtyard recommended)

## Garmin GPS 18x

### Hardware

**Receiver**: Garmin GPS 18x (standard GPS, ~3-5m accuracy)
**Update Rate**: 5 Hz
**Interface**: USB (FTDI serial converter)
**Output**: NMEA 0183 sentences

### USB Detection

```bash
# Check device detected
sudo dmesg | grep tty
# Should show: FTDI USB Serial Device converter attached to ttyUSB0
```

### Driver: gpsd

**Installation**:
```bash
sudo apt install gpsd gpsd-clients
```

**Start daemon**:
```bash
sudo /usr/sbin/gpsd -n -G -b /dev/ttyUSB0
```

**Verify signal**:
```bash
# Terminal GPS viewer
cgps

# Graphical GPS viewer
xgps

# Wait for "3D Fix" (takes 1-5 minutes on cold start)
```

### ROS 2 Integration

**Configuration**: `gnss.launch.xml`
```xml
<include file="$(find-pkg-share gpsd_client)/launch/gpsd_client-launch.py">
  <arg name="config_file" value="$(find-pkg-share gpsd_client)/config/gpsd_client.yaml"/>
</include>
```

**gpsd client config**:
```yaml
/**:
  ros__parameters:
    host: "localhost"
    port: 2947
    frame_id: "gnss_link"
```

### Test

```bash
# Start gpsd
sudo /usr/sbin/gpsd -n -G -b /dev/ttyUSB0

# Launch ROS 2 client
ros2 launch gpsd_client gpsd_client-launch.py

# Verify
ros2 topic echo /fix
ros2 topic hz /fix  # ~5 Hz
```

### Troubleshooting

**No GPS signal**:
- Move to open area (clear sky view)
- Wait 5-10 minutes for cold start
- Check antenna cable connection

**Device not found**:
- Check `/dev/ttyUSB0` exists
- Try different USB port
- Verify USB cable provides power

## Indoor Operation (No GNSS)

For indoor environments without satellite signal:

```bash
# Disable GNSS, use NDT localization only
make launch ARGS="use_gnss:=false"
```

Use RViz "2D Pose Estimate" tool to manually set initial position.

## Quick Reference

```bash
# Launch commands
make launch ARGS="gnss_receiver:=ublox use_ntrip:=true"   # u-blox RTK
make launch ARGS="gnss_receiver:=garmin"                  # Garmin
make launch ARGS="use_gnss:=false"                        # Indoor (no GNSS)

# u-blox verification
ls -l /dev/ublox-gps
ros2 topic hz /sensing/gnss/ublox/nav_sat_fix
ros2 topic hz /sensing/gnss/ntrip/rtcm  # RTK corrections

# Garmin verification
cgps                                     # Check GPS signal
ros2 topic hz /fix

# Check fix quality
ros2 topic echo /sensing/gnss/ublox/nav_sat_fix --once
# status.status: -1=no fix, 0=GPS, 1=DGPS, 2=RTK
```
