# Datasets and Rosbags

AutoSDV provides recorded sensor data for testing and development without a physical vehicle.

## Available Datasets

### COSS Park Outdoor Recording

| Field | Value |
|-------|-------|
| Name | `outdoor_20251226_153115` |
| Location | `data/rosbags/outdoor_20251226_153115/` |
| Size | 2.8 GB |
| Duration | 157 seconds |
| Date | 2025-12-26 |
| Map | `data/COSS-map-planning/` |

Recorded at NTU COSS Park with the following sensors:

| Topic | Type | Rate |
|-------|------|------|
| `/sensing/lidar/velodyne_points` | PointCloud2 | 10 Hz |
| `/sensing/lidar/velodyne_packets` | VelodyneScan | 10 Hz |
| `/sensing/camera/zedxm/imu/data` | Imu | 100 Hz |
| `/sensing/gnss/ublox/nav_sat_fix` | NavSatFix | 4 Hz |
| `/sensing/gnss/ublox/fix_velocity` | TwistWithCovarianceStamped | 4 Hz |
| `/vehicle/status/velocity_status` | VelocityReport | 20 Hz |
| `/vehicle/status/steering_status` | SteeringReport | 30 Hz |

### Leo Drive Bus-ODD Dataset

The [Leo Drive Bus-ODD dataset](https://autowarefoundation.github.io/autoware-documentation/main/datasets/) is an Autoware-compatible dataset with camera streams, useful for testing visual localization. Tools for downloading and converting this dataset are in the `scripts/leodrive-bus-launch` submodule.

```bash
cd scripts/leodrive-bus-launch
just setup       # Download (~10.9 GB) and migrate to Autoware 1.5.0 format
just play data/all-sensors-bag1_migrated
```

## Downloading Data

Download the COSS Park recording:

```bash
just download-data
```

This runs `scripts/rosbag/download-test-rosbag.sh`, which:

1. Checks if the rosbag already exists with the correct checksum
2. Installs `synology-dl` via `cargo install` if not found
3. Downloads from Synology Drive
4. Extracts and verifies the SHA256 checksum

## Recording Your Own Data

Record outdoor sensor topics from a live vehicle:

```bash
just bag-record
```

This saves a timestamped rosbag to the `rosbags/` directory with all sensor and vehicle status topics listed in `scripts/rosbag/outdoor_topics.txt`.

Play back the most recent recording:

```bash
just bag-play
```

## Rosbag Format

AutoSDV uses ROS 2 bag format (SQLite3 storage). Each rosbag directory contains:

- `<name>_0.db3` — the message database
- `metadata.yaml` — topic list, message counts, duration, and storage format

Inspect a rosbag:

```bash
ros2 bag info data/rosbags/outdoor_20251226_153115/
```

## Using External Datasets

To use an Autoware-compatible rosbag with AutoSDV:

1. Ensure the rosbag contains at least LiDAR point cloud and vehicle status topics
2. Check that topic names match the AutoSDV sensor kit configuration (see `src/sensor_kit/`)
3. Provide a matching point cloud map and Lanelet2 map
4. Launch the logging simulation with the appropriate sensor suite:

```bash
just launch-sim-logging sensor_suite:=custom lidar_model:=vlp32c
```
