# Sensor Capability Matrix

What each supported sensor is, what its driver publishes, and which parts of the
stack can consume it. This page is the lookup table; the reasoning behind the
rows is in [LiDAR Sensors](../../guides/sensor-integration/lidar.md).

Every row here was checked against the repository rather than against a
datasheet, and the file that decides each answer is named. Where the tree does
not settle a value, the cell says so instead of guessing.

## LiDAR: what it is

| | Seyond Robin-W | Velodyne VLP-32C | Blickfeld Cube1 |
|---|---|---|---|
| `lidar_model:=` | `robin-w` | `vlp32c` | `cube1` |
| Type | solid-state | spinning, 32 channels | solid-state |
| Horizontal FOV | 120° (±60°, enforced by the SDK) | 360° | 70° |
| Vertical FOV | **unverified** — 25° or 70° | 40° | 30° |
| Scan lines | 192 (`kRobinWScanlines_`) | 32 | — |
| Range | 200 m | 300 m as configured | 150 m |
| Address | 172.168.1.10 | 192.168.7.10 (`sensor_ip`) | 192.168.26.26 |
| TF frame | `robin_w` | `vlp32c` | `cube1` |

!!! warning "The Robin-W vertical field of view"

    The horizontal 120° is enforced in the driver: the Seyond SDK's
    `is_robin_inside_fov_point()` discards points outside ±60°. **The vertical
    figure is not settled anywhere in the tree.** This book's LiDAR page has
    said 25°, `docs/research/robin_w_fov.md` in the repository says 70°, the SDK
    compiles in no vertical limit (the elevation table is fetched from the
    device at runtime), and no Robin-W recording exists in the repository to
    measure one from.

    What would settle it: one Robin-W bag, and
    `scripts/sensor/inspect_rings.py`, which prints each channel's elevation
    from the cloud's own `elevation` field. Do not propagate either number until
    someone has run it.

    Sources: `inno_lidar_packet_utils.h` in the `seyond_ros_driver` submodule;
    `sensor_kit_calibration.yaml`; `VLP32.param.yaml`; `cube1.param.yaml`. The
    Cube1 and VLP-32C FOV figures are vendor specifications carried forward from
    earlier documentation, not values the tree states.

## LiDAR: what the driver publishes

| | Seyond Robin-W | Velodyne VLP-32C | Blickfeld Cube1 |
|---|---|---|---|
| Driver | `seyond` (submodule `seyond_ros_driver`) | Nebula (`nebula_ros`, from Autoware) | `blickfeld_driver` (submodule) |
| Point layout | `PointXYZIRCAEDT` | `PointXYZIRCAEDT` | Blickfeld's own |
| Per-point time | yes (`time_stamp`, ns after the header) | yes | no, in the shipped configuration |
| `channel` means | `scan_id` — a galvo sweep position | ring index, a constant elevation | — |
| Raw topic in the kit | `/sensing/lidar/iv_points` | `/sensing/lidar/velodyne_points` | `/sensing/lidar/bf_lidar/points_raw` |
| Configuration | `seyond_robin_w.launch.xml` | `config/VLP32.param.yaml` | `config/cube1.param.yaml` |

The raw topics are the values of `LIDAR_TOPICS` in
`pointcloud_preprocessor.launch.py`, which is where `lidar_model` is resolved.
Whichever sensor is fitted, the preprocessing chain publishes
`/sensing/lidar/concatenated/pointcloud` in `base_link`.

## LiDAR: what the stack can do with it

| | Seyond Robin-W | Velodyne VLP-32C | Blickfeld Cube1 |
|---|---|---|---|
| 3-D NDT (`ndt`, `cuda_ndt`) | yes | yes | yes |
| `pointcloud_backend:=cuda` | yes | yes | **refused at launch** |
| Deskewing possible at all | yes | yes | no |
| 2-D MCL scan source (`publish_scan`) | no — no constant-elevation ring | yes | no — no ring, and 70° is too narrow |

Multi-LiDAR fusion is not available for any of them: the kit runs one LiDAR and
ends its chain in a single-input passthrough rather than a concatenator.

`cube1` with `pointcloud_backend:=cuda` raises a `ValueError` before any node
starts. The check is in `pointcloud_preprocessor.launch.py` and keys on
`lidar_model`, against a `DESKEWABLE` tuple of `vlp32c` and `robin-w`.

Two failure modes the launch-time check cannot see:

- A Seyond driver rebuilt with `POINT_TYPE=PointXYZIRC` publishes a cloud the
  CUDA preprocessor rejects **at runtime**. The default is `PointXYZIRCAEDT`;
  leave it alone unless you know why you are changing it.
- The Blickfeld driver has a `publish_point_time_offset` parameter, `false` in
  the kit config. Enabling it adds a field of its own, not the
  `PointXYZIRCAEDT` layout the CUDA node consumes, so it does not make `cube1`
  deskewable.

Nothing refuses `publish_scan:=true` on a solid-state sensor either. The ring
extractor runs for any model and produces a scan-shaped message that is not a
plane, because the `channel` field of a solid-state cloud is not an elevation.

## Sensor suites

`sensor_suite` sets all five selectors at once. The individual arguments
override whatever the suite chose. From `sensing.launch.xml`:

| `sensor_suite:=` | `lidar_model` | `camera_model` | `imu_source` | `gnss_receiver` | `use_gnss` | ZED object detection |
|---|---|---|---|---|---|---|
| `vlp32c_zed_imu` *(default)* | `vlp32c` | `none` | `zed` | `ublox` | true | false |
| `vlp32c_zed` | `vlp32c` | `zedxm` | `zed` | `ublox` | true | true |
| `vlp32c_zed_mpu` | `vlp32c` | `zedxm` | `mpu9250` | `ublox` | true | true |
| `robin_zed` | `robin-w` | `zedxm` | `zed` | `ublox` | true | true |
| `robin_zed_mpu` | `robin-w` | `zedxm` | `mpu9250` | `ublox` | true | true |
| `cube1_usb` | `cube1` | `usb` | `mpu9250` | `ublox` | true | false |
| `custom` | `vlp32c` | `zedxm` | `mpu9250` | `garmin` | true | — |

The default suite is `vlp32c_zed_imu`, so the default LiDAR is the VLP-32C —
the only one of the three that can feed 2-D MCL, and one of the two that can be
deskewed.

```bash
play_launch launch autosdv_launch autosdv.launch.yaml sensor_suite:=robin_zed
```

```bash
just launch sensor_suite:=robin_zed
```

Individual selectors, for `sensor_suite:=custom` or to override one value:

| Argument | Values |
|----------|--------|
| `lidar_model` | `robin-w`, `vlp32c`, `cube1` |
| `camera_model` | `zedxm`, `usb`, `none` |
| `imu_source` | `mpu9250`, `zed` |
| `gnss_receiver` | `ublox`, `septentrio`, `garmin` |

## Choosing by what you need

- **CUDA sensing and localization end to end**: `vlp32c` or `robin-w`.
- **2-D MCL against an occupancy grid**: `vlp32c`, or a native 2-D LiDAR
  alongside a solid-state one.
- **A 360° view for mapping**, where loop closure has to work: `vlp32c`.
- **Cube1**: the CPU sensing path and 3-D NDT only.

## Related

- [LiDAR Sensors](../../guides/sensor-integration/lidar.md) — why these rows are what they are
- [The CUDA Point Cloud Pipeline](../../guides/cuda-pipeline.md)
- [Localization Methods](../../guides/localization-methods.md)
- [Core Components](./core-components.md)

<!--
RECONCILE:
- nav: add "Sensor Capability Matrix" under Technical Reference > Hardware,
  between "Core Components" and "Wiring Diagrams":
      - Sensor Capability Matrix: reference/hardware/sensor-capability-matrix.md
  and a nav_translations entry: "Sensor Capability Matrix: 感測器能力對照表".
- cross-link from reference/overview.md "Core Components" list.
- cross-link from guides/sensor-integration/using-sensors.md, which lists the
  suites in prose and gets robin-w's defaults wrong.
- cross-link from guides/localization-methods.md section "Which sensors this
  applies to" -> this page's "LiDAR: what the stack can do with it".
- DE-DUPLICATE: this page and guides/cuda-pipeline.md both carry a deskew
  support table. Phase 2 should pick one home; the suggestion is that
  cuda-pipeline links here.
- platform-models.md repeats the unverified Robin-W 120° x 25° figure in three
  places and should be made consistent with the warning on this page.
-->
