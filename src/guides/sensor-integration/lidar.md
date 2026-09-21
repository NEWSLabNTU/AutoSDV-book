# LiDAR Sensors

Configuration for the three supported LiDAR models, and what choosing a
solid-state one costs elsewhere in the stack.

One LiDAR runs at a time. `lidar_model` selects which:

```bash
play_launch launch autosdv_launch autosdv.launch.yaml lidar_model:=vlp32c
```

```bash
just launch lidar_model:=vlp32c
```

## Supported Models

| Model | Type | Field of view | Range | Address | `lidar_model:=` | Topic in the kit |
|-------|------|---------------|-------|---------|-----------------|------------------|
| **Seyond Robin-W** | Solid-state | 120° horizontal; vertical **unverified** | 200 m | 172.168.1.10 | `robin-w` | `/sensing/lidar/iv_points` |
| **Velodyne VLP-32C** | Spinning, 32 channels | 360° × 40° | 300 m configured | 192.168.7.10 | `vlp32c` | `/sensing/lidar/velodyne_points` |
| **Blickfeld Cube1** | Solid-state | 70° × 30° | 150 m | 192.168.26.26 | `cube1` | `/sensing/lidar/bf_lidar/points_raw` |

The topic column is the authority in `pointcloud_preprocessor.launch.py`, which
is where `lidar_model` is resolved to a raw topic. An unrecognised value is
rejected there with the list of valid ones.

The consolidated view — per-point time, CUDA deskew, whether the sensor can feed
2-D MCL — is the
[Sensor Capability Matrix](../../reference/hardware/sensor-capability-matrix.md).

!!! warning "The Robin-W vertical field of view is unverified"

    The horizontal figure is settled, and by the driver rather than by a
    datasheet: the Seyond SDK discards any point outside ±60° in
    `is_robin_inside_fov_point()`. 120° horizontal is what the driver enforces.

    The vertical figure is not settled. This book has said **25°**;
    `docs/research/robin_w_fov.md` in the repository says **70°**. Nothing in
    the tree decides between them. The SDK compiles in no vertical limit for
    Robin-W at all — the elevation table is fetched from the device at runtime
    via `inno_lidar_get_anglehv_table` — and no Robin-W recording exists in the
    repository to measure one from.

    It is one command to settle, the next time a Robin-W is on a bench. The
    driver publishes a per-point `elevation` field, so:

    ```bash
    python3 scripts/sensor/inspect_rings.py <bag> --topic /sensing/lidar/iv_points
    ```

    The span between the lowest and the highest channel elevation is the
    answer. Until someone runs it, treat the vertical extent as unknown rather
    than as either published number.

## What a solid-state LiDAR costs this stack

Robin-W and Cube1 have no moving parts, which buys reliability and a smaller
package. Three things in this stack are built around a spinning sensor's
geometry, and this is where the trade shows up.

### A restricted field of view, and scan matching

NDT — `pose_source:=ndt` or the default `cuda_ndt` — matches each scan against a
prebuilt point cloud map. It is map matching, not SLAM, so there is no loop
closure to lose; a narrow field of view does not cost drift the way the SLAM
literature describes. What it costs is **constraint**.

A scan constrains the pose only in the directions where it contains structure,
and a 120° forward cone contains structure only ahead. Where the cone is full of
geometry — a corridor, a row of parked cars, a building face — the estimate is
as well constrained as a spinner's. Where it is pointed across open ground the
match has little to bite on, and the estimate leans on the EKF's prediction from
the IMU and wheel odometry instead. Turns are the moment this is most visible,
because the cone swings onto structure that was not in the previous scan.

Two consequences that are actually shipped:

- **Tuning matters more, and so does watching it.** The repository carries the
  process in `docs/guides/ndt-tuning.md`, and the live diagnostics in
  `scripts/testing/localization/` — `ndt_quality_report.py` for pose quality,
  `ndt_alignment_report.py` for the scan-to-map residual.
- **Building the map is the harder half.** A PCD produced by a SLAM pipeline
  *does* need loop closure, and that is where a 240° blind sector genuinely
  hurts. See [Maps](../maps.md#where-the-point-cloud-map-comes-from) for the
  mapping workflow, and
  [Maps](../maps.md) for what each localization method consumes.

The repository's `docs/research/robin_w_fov.md` goes further and sketches
architectures — UWB beacons, AprilTag pose graphs, RTAB-Map fusion — that this
project has never built. Read it as research, not as configuration.

### No constant-elevation ring, so no 2-D scan

[2-D MCL](../localization-methods.md) consumes one `sensor_msgs/LaserScan`. The
sensor kit produces that scan from a 3-D cloud by keeping a narrow group of
channels:

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  pose_source:=mcl publish_scan:=true scan_ring:=<measured>
```

That works because a spinning LiDAR's channel index *is* an elevation: channel N
traces a cone of constant pitch, and a narrow group of channels is very nearly a
plane. On the VLP-32C it holds.

On Robin-W it does not. The driver's `channel` field carries the SDK's
`scan_id` — the position in a galvo sweep that moves in elevation — not a fixed
pitch, so "channel N" is not a horizontal plane and no choice of ring makes it
one. Cube1 is the same, and adds a 70° horizontal field of view, which
constrains a particle filter poorly against a 360° occupancy grid whichever
plane you extract.

Nothing refuses `publish_scan:=true` with a solid-state sensor. The launch file
dispatches the ring extractor for any model, and what comes out is a scan-shaped
message that is not a plane. Use these sensors on the 3-D NDT path, or put a
native 2-D LiDAR alongside.

`scan_input_topic` defaults to the Autoware sample bag's cloud, so on this kit
set it to what the driver actually publishes — for the VLP-32C,
`/sensing/lidar/velodyne_points`.

### Deskewing, and the CUDA sensing backend

Distortion correction shifts each point by the motion that occurred between the
start of the sweep and the moment that point was measured. It therefore needs a
**per-point time offset**, and not every driver publishes one.

| `lidar_model` | Driver | Per-point time | `pointcloud_backend:=cuda` |
|---------------|--------|----------------|----------------------------|
| `vlp32c` | Nebula | yes — `PointXYZIRCAEDT` | supported |
| `robin-w` | `seyond`, at this repository's pin | yes — `PointXYZIRCAEDT` | supported |
| `cube1` | Blickfeld | no | **refused at launch** |

`cube1` with `pointcloud_backend:=cuda` raises a `ValueError` before any node
starts, naming the reason and telling you to use `pointcloud_backend:=cpu`. This
is deliberate: the alternative is a cloud that is silently not deskewed.

!!! note "Two ways to lose Robin-W deskewing"

    The Seyond driver's point layout is a build-time choice, `POINT_TYPE` in its
    `CMakeLists.txt`, defaulting to `PointXYZIRCAEDT`. A driver rebuilt as
    `PointXYZIRC` publishes a cloud the CUDA preprocessor rejects at runtime
    rather than deskews — and the launch-time check will not catch it, because
    that check keys on `lidar_model`, not on the cloud.

    Symmetrically, the Blickfeld driver does have a `publish_point_time_offset`
    parameter, set `false` in the kit's `cube1.param.yaml`. Turning it on does
    not make `cube1` deskewable: it adds a field of its own, not the
    `PointXYZIRCAEDT` layout the CUDA node consumes, and the refusal is keyed on
    the model regardless.

The rest of the CUDA story — the three backend switches, why every stage must
load into one container — is [The CUDA Point Cloud
Pipeline](../cuda-pipeline.md).

### Why there is no concatenator

Autoware's sensing chain normally ends in a concatenator that merges several
LiDARs into one cloud. This kit does not use it, and cannot: both the CPU and
the CUDA concatenator refuse a single input topic.

```
Component constructor threw an exception:
Only one topic given. Need at least two topics to continue.
```

Listing the same topic twice does load, and then loses roughly 80 % of frames —
each message fills one slot, the collector waits out `timeout_sec` for a second
that never arrives, and the collector limit thrashes.

So the kit ends in a passthrough node instead, which also performs the transform
into `base_link`. That last part matters if you go looking for the CUDA
preprocessor's output: it leaves its cloud in the *sensor* frame, and the
passthrough is what moves it.

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

**Required calibration**, in
`src/param/autoware_individual_params/individual_params/config/default/autosdv_sensor_kit/sensor_kit_calibration.yaml`:

```yaml
sensor_kit_base_link:
  robin_w:           # the frame is robin_w, not robin_lidar_link
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 3.14159    # 180° flip
    pitch: -1.5708   # -90° rotation
    yaw: 0.0
```

See [Integration Walkthrough](./integration-walkthrough.md) for detailed explanation.

### Driver Package

**Submodule**: `src/sensor_component/external/seyond_ros_driver/`
**ROS package name**: `seyond`
**Point format**: `PointXYZIRCAEDT` by default — see the deskewing note above
before changing `POINT_TYPE`.

### Test Standalone

```bash
colcon build --base-paths src --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select seyond
source install/setup.bash
ros2 launch autosdv_sensor_kit_launch seyond_robin_w.launch.xml

# Verify — outside the kit's namespace the topic is /iv_points
ros2 topic hz /iv_points   # ~10 Hz
```

Inside the full launch the same cloud appears as `/sensing/lidar/iv_points`,
because the kit pushes the `sensing/lidar` namespace around the driver.

### Troubleshooting

**No data**: Check `ping 172.168.1.10` succeeds
**Wrong orientation**: Verify roll=3.14159, pitch=-1.5708 in calibration
**`pointcloud_backend:=cuda` rejects the cloud at runtime**: the driver was
built as `PointXYZIRC`; rebuild it with the default `POINT_TYPE`

## Velodyne VLP-32C

### Network Setup

The address the driver actually uses is `sensor_ip` in the kit's
`VLP32.param.yaml`, which is **192.168.7.10**, not the factory default. Put the
host on the same subnet:

```bash
# Configure Jetson
sudo ip addr add 192.168.7.100/24 dev eth0
sudo ip link set eth0 up

# Test connectivity
ping 192.168.7.10
```

If your unit is still on its factory address, either re-address the sensor or
edit `sensor_ip` in the parameter file — those are the only two places the value
exists.

### Coordinate System

Standard ROS coordinates — no roll or pitch. The shipped calibration does carry
a **yaw**, and it is not an arbitrary value:

```yaml
sensor_kit_base_link:
  vlp32c:            # the frame is vlp32c, not velodyne_link
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: -0.2210     # -12.66°, derived from replay, not measured
```

That yaw was recovered from a replay: with `yaw: 0.0` the localized heading sat
12.66° off the vehicle's own direction of travel on straight segments, which a
car cannot do. Correcting it drops the discrepancy to 0.07° and cuts NDT's
per-frame correction from 0.127 m to 0.049 m. The x/y/z are still unmeasured —
replace all four with measured extrinsics when the vehicle is available. The
working is in `docs/reports/cuda-ndt-coss-replay.md` in the repository.

### Driver Package

**Driver**: Nebula (`nebula_ros`), which ships with Autoware — there is no
Velodyne driver submodule in this repository and nothing to `apt install`.
**Configuration**:
`src/sensor_kit/autosdv_sensor_kit_launch/autosdv_sensor_kit_launch/config/VLP32.param.yaml`
**Point format**: `PointXYZIRCAEDT`, which carries the per-point time that makes
`pointcloud_backend:=cuda` possible.

### Dual Return Mode

Dual return — both the strongest and the last echo, which helps in rain and
fog — is already the configured default, together with 600 rpm:

```yaml
/**:
  ros__parameters:
    sensor_model: VLP32
    rotation_speed: 600
    return_mode: Dual
    min_range: 0.3
    max_range: 300.0
    frame_id: vlp32c
```

### Test Standalone

```bash
ros2 launch nebula_ros velodyne_launch_all_hw.xml \
  sensor_model:=VLP32 \
  config_file:=$(ros2 pkg prefix --share autosdv_sensor_kit_launch)/config/VLP32.param.yaml

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
sensor_kit_base_link:
  cube1:             # the frame is cube1, not bf_lidar_link
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
```

### Driver Package

**Location**: `src/sensor_component/external/ros2_blickfeld_driver_src-v1.5.5/`
**Requires**: Blickfeld Scanner Library 2.20.6-newslab1 (install via `./setup.sh blickfeld`)
**Configuration**:
`src/sensor_kit/autosdv_sensor_kit_launch/autosdv_sensor_kit_launch/config/cube1.param.yaml`

This is the one model that cannot use `pointcloud_backend:=cuda`; see
[Deskewing, and the CUDA sensing backend](#deskewing-and-the-cuda-sensing-backend).

### Test Standalone

```bash
colcon build --base-paths src --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select blickfeld_driver
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

**EULA error**: Run `./setup.sh blickfeld` to accept license

## Multi-LiDAR Setup

**This kit runs one LiDAR.** A second sensor can be mounted and calibrated —
`sensor_kit_calibration.yaml` takes as many links as you give it:

```yaml
# sensor_kit_calibration.yaml
sensor_kit_base_link:
  robin_w:             # Front-facing
    x: 0.15
    z: 0.15
    roll: 3.14159
    pitch: -1.5708

  vlp32c:              # Top 360°
    x: 0.0
    z: 0.30
    roll: 0.0
    pitch: 0.0
```

— but the preprocessing chain will not merge the two clouds. `lidar_model`
selects exactly one raw topic, and the chain ends in a single-input passthrough
rather than a concatenator, for the reason given above. Fusing a second LiDAR
means re-introducing the concatenator, which is work this kit has not done.

The gain, if someone does it, is more than coverage: with two inputs the CUDA
concatenator becomes usable, and the sensing chain could stay on the GPU
end to end instead of paying a device-to-host copy at the passthrough.

## Quick Reference

```bash
# Network tests
ping 172.168.1.10    # Robin-W
ping 192.168.7.10    # Velodyne, at the configured sensor_ip
ping 192.168.26.26   # Blickfeld

# Topic verification
ros2 topic list | grep /sensing/lidar
ros2 topic hz /sensing/lidar/iv_points            # Robin-W
ros2 topic hz /sensing/lidar/velodyne_points      # Velodyne
ros2 topic hz /sensing/lidar/bf_lidar/points_raw  # Blickfeld

# What the preprocessing chain publishes, whichever sensor is fitted
ros2 topic hz /sensing/lidar/concatenated/pointcloud

# TF verification -- the frames are robin_w, vlp32c, cube1
ros2 run tf2_ros tf2_echo sensor_kit_base_link robin_w

# Network monitoring
sudo iftop -i eth0
```

## Related

- [Sensor Capability Matrix](../../reference/hardware/sensor-capability-matrix.md) — every sensor fact in one table
- [The CUDA Point Cloud Pipeline](../cuda-pipeline.md) — the three backend switches
- [Localization Methods](../localization-methods.md) — which sensor suits which `pose_source`
- [Sensor Troubleshooting](./troubleshooting.md)

<!--
RECONCILE:
- nav: no new entry needed for this page; it is already navigated.
- cross-link FROM guides/localization-methods.md section "Which sensors this
  applies to" TO this page's "No constant-elevation ring, so no 2-D scan"
  (guides/sensor-integration/lidar.md#no-constant-elevation-ring-so-no-2-d-scan),
  and consider replacing that section's prose with the link, since this page now
  gives the mechanism (channel = scan_id, not elevation).
- cross-link FROM guides/cuda-pipeline.md section "Which LiDARs qualify" TO
  "Deskewing, and the CUDA sensing backend" here; the two POINT_TYPE /
  publish_point_time_offset failure modes are documented only here.
- DE-DUPLICATE with guides/cuda-pipeline.md: both pages now carry a per-model
  deskew table. Phase 2 should keep one (suggest: cuda-pipeline keeps the
  backend-switch table, this page keeps the per-sensor one) and link the other.
- When W3's guides/ndt-tuning.md lands, link it from "A restricted field of
  view, and scan matching", which currently cites the repository path instead.
- CORRECTIONS this page made that other pages still carry the old value of:
  - Robin-W topic is /sensing/lidar/iv_points, NOT /robin_lidar/points_raw
    (stale in guides/sensor-integration/troubleshooting.md).
  - Velodyne sensor_ip is 192.168.7.10, NOT 192.168.1.201 (stale in
    reference/hardware/core-components.md power table context and elsewhere).
  - There is no Velodyne driver submodule and no ros-humble-velodyne package;
    the driver is Nebula.
  - The calibration frames are robin_w / vlp32c / cube1. The names
    robin_lidar_link, velodyne_link and bf_lidar_link appear in several pages
    and in NO file in the tree; they should be corrected wherever they occur
    (guides/sensor-integration/integration-walkthrough.md,
    guides/sensor-integration/troubleshooting.md).
  - guides/sensor-integration/using-sensors.md calls robin-w a "360° LiDAR",
    which is wrong in both axes, and calls it the default. The default
    sensor_suite is vlp32c_zed_imu, so the default lidar_model is vlp32c.
  - platform-models.md repeats the unverified 120° × 25° figure; it should
    carry the same "vertical unverified" caveat as this page.
-->
