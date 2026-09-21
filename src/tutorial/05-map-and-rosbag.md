# 5. The Map and the Rosbag

You have now used the same two data sets three times. This page says what they
actually are — partly because it is interesting, and partly because one of them
has a misleading name that will confuse you later if nobody explains it.

## `data/COSS-map-planning` is not a planning map

The name is a historical accident. The directory holds **the complete map of the
site**, and three different components each take a different file out of it:

```
data/COSS-map-planning/
├── lanelet2_map.osm              590 KB   the road network      → planning
├── pointcloud_map.pcd             78 MB   the 3-D point cloud   → NDT localization
├── occupancy_grid.pgm + .yaml    3.9 MB   the 2-D grid          → MCL localization
├── map_projector_info.yaml                where on Earth this is
├── pointcloud_map_metadata.yaml           how the PCD is tiled
└── autosdv_map.yaml                       provenance
```

So when you passed `map_path:=$PWD/data/COSS-map-planning` to the planning
simulation and then met the same directory in the logging simulation, it was not
a coincidence and it was not the same file being used. The planning simulator
read the lanelet2 map and ignored the point cloud entirely. NDT did the reverse.

### `lanelet2_map.osm` — the road network

Lanes, their boundaries, stop lines and traffic rules, in OpenStreetMap's XML
with lanelet2's extensions. Nodes carry both global coordinates and local ones:

```xml
<node id="10300004601" lat="25.0202319785" lon="121.54273096174">
  <tag k="local_x" v="304774.8603999999"/>
  <tag k="local_y" v="2768128.0634000003"/>
  <tag k="ele" v="8.9764"/>
</node>
```

This is what routing runs on. When a `2D Goal Pose` produced no route in the
planning simulation, it was because the point you clicked was not inside any
lanelet — the map, not the planner, decided that.

### `map_projector_info.yaml` — where on Earth this is

```yaml
projector_type: TransverseMercator
vertical_datum: WGS84
map_origin:
  latitude: 25.0201
  longitude: 121.5423
  altitude: 25.0
```

This fixes the `map` frame's origin to a real place — the COSS site at National
Taiwan University. Every coordinate you have seen, including the pose seeded at
`(-1.839, -8.280)`, is metres from that point.

It is also what lets GNSS be used for initialisation: a latitude and longitude
can be turned into map-frame metres only because this file exists.

### `pointcloud_map.pcd` — what NDT matches against

78 MB, about 4.9 million points, tiled 300 m × 300 m from `[-150, -150]`
according to `pointcloud_map_metadata.yaml`.

This is the file whose loading you waited 25 seconds for. NDT does not match
against raw points: it divides the map into voxels and fits a Gaussian to the
points in each, then finds the vehicle pose that makes the live scan most likely
under that model. That is where the name comes from — Normal Distributions
Transform.

### `occupancy_grid.pgm` + `.yaml` — the 2-D grid

```yaml
image: occupancy_grid.pgm
resolution: 0.05
origin: [-65.000, -25.000, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
```

A greyscale image where each pixel is 5 cm, with the metadata that places it in
the map frame. Used only by `pose_source:=mcl`, which matches a single-plane
`LaserScan` against it instead of a 3-D cloud against the PCD.

It was produced by slicing a height band out of the PCD — see
[Maps](../guides/maps.md) if you need one for your own site. The band is the one
judgement that matters, and getting it wrong produces a grid that looks fine and
localizes badly.

### Check a map before trusting it

```bash
just map check data/COSS-map-planning cuda_ndt
just map check data/COSS-map-planning mcl
```

This verifies the artefacts that method needs are present, and — the reason the
tool exists — that the occupancy grid is in the **same frame** as the lanelet2
map. A grid built in the wrong frame gives you a system that starts cleanly,
localizes confidently, and is wrong by a constant offset.

## The recording

```bash
ros2 bag info data/rosbags/outdoor_20251226_153115
```

```
Bag size:   2.8 GiB
Duration:   157.005 s
Start:      Dec 26 2025 15:31:16
Messages:   43538
```

16 topics. The ones that matter:

| Topic | Type | Count | Rate |
|---|---|---|---|
| `/sensing/lidar/velodyne_points` | `PointCloud2` | 1570 | 10 Hz |
| `/sensing/lidar/velodyne_packets` | `VelodyneScan` | 1571 | 10 Hz |
| `/sensing/camera/zedxm/imu/data` | `Imu` | 15463 | ~98 Hz |
| `/vehicle/status/steering_status` | `SteeringReport` | 4710 | ~30 Hz |
| `/vehicle/status/gear_status` | `GearReport` | 4702 | ~30 Hz |
| `/vehicle/status/velocity_status` | `VelocityReport` | 3134 | ~20 Hz |
| `/sensing/gnss/ublox/nav_sat_fix` | `NavSatFix` | 627 | ~4 Hz |

The vehicle was the Velodyne VLP-32C configuration with a ZED X Mini, whose IMU
is the one being recorded. The topic names are
[the Autoware conventions](../concepts/autoware-conventions.md) — `/sensing/` for
inputs, `/vehicle/status/` for what the vehicle reports back.

### What is *not* in it

**There is no `/tf` or `/tf_static`.** That surprises people, because a replay
plainly needs transforms. They come from the stack rather than the recording:
the sensor kit description publishes the fixed sensor-to-`base_link` transforms,
and localization publishes `map → odom`. Replaying this bag into a differently
configured vehicle would therefore use *that* vehicle's geometry — which is a
feature when the calibration improves, and a trap if you assume the bag is
self-contained.

There is also no camera image; only the ZED's IMU and health topics were kept.
The `/sensing/gnss/ntrip/rtcm` topic exists with **zero** messages — RTK
corrections were not available on the day.

### The drive

- **0 – 116 s**: parked, sensors running
- **116.3 s**: first motion above 0.2 m/s
- **116 – 157 s**: a 41-second drive, up to 1.58 m/s

That long stationary prefix is genuinely useful. It gives localization an easy
period to converge in before anything moves, and it is why the metrics split
`init` from `track` — the two halves are different problems.

### The GNSS is not good enough to initialise from

Which is why every demo passes `use_gnss:=false`. The fix is single-point with
roughly 20 m of scatter and disagrees with the direction of travel, so seeding
localization from it lands the vehicle somewhere different on every run. A
recorded pose is used instead, and that is what makes the demo reproducible.

This is worth seeing once, because it is the normal situation outdoors without
RTK — not a defect in this dataset.

## Using your own data

```bash
just bag record    # records the outdoor sensor topic set
```

Then replay against your own map:

```bash
play_launch launch autosdv_launch logging_simulation.launch.yaml \
  map_path:=/path/to/your/map
```

Validate the map first with `just map check`. More in
[Datasets & Rosbags](../running/datasets.md) and [Maps](../guides/maps.md).

**Next:** [6. play_launch](./06-play-launch.md) — the last layer, and the one
whose caveats you should know before you need them.
