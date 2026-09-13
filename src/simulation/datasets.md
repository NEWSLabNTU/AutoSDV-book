# Datasets & Rosbags

What to replay, how to get it, and how to record your own.

## The COSS Park test bag

The recording every simulation page uses.

```bash
just bag download
```

A 1.6 GiB download that unpacks to 2.8 GiB in
`data/rosbags/outdoor_20251226_153115`, with about 4.4 GiB needed in between.
It installs `synology-dl` with `cargo` if that is missing, so Rust has to be
present; it verifies the SHA-256 of the `.db3` afterwards and deletes the
download if it does not match. Re-running it checks the checksum and exits, so
it is safe in a script and safe after an interrupted download.

The matching map needs no download at all — `data/COSS-map-planning` is
committed to the repository.

The recording is 157 seconds: parked for the first 115.7 s, then a 41 s drive at
up to 1.58 m/s.

`just demo run` fetches it automatically if it is missing, so you rarely need to
call this directly.

## The Leo Drive Bus-ODD dataset

An Autoware Foundation dataset with camera streams, useful for testing visual
localization. Tools live in the `scripts/leodrive-bus-launch` submodule.

| Sensor | Model | Count |
|--------|-------|-------|
| LiDAR | Velodyne VLP16 | 1 (front) |
| LiDAR | Velodyne VLP32C | 2 (left, right) |
| Camera | Lucid Vision Triton 5.4 MP | 3 |
| GNSS/INS | Applanix POS LV 120 | 1 |

```bash
cd scripts/leodrive-bus-launch
just setup          # download ~10.9 GB and migrate to Autoware 1.5.0
just play data/all-sensors-bag1_migrated
```

The migration step is not optional: the dataset was recorded against
`autoware_auto_*` message types, which Autoware 1.5.0 no longer defines. `just
migrate-all` rewrites the bags to `autoware_*`.

## ROS 2 bags, in general

A bag is a recording of topic traffic. Four uses, all of them relevant here:

- **Record** sensor data during a drive, for analysis afterwards
- **Replay** it to test and tune algorithms against unchanging input
- **Share** a recording, so a colleague debugs the same data you saw
- **Diagnose** a fault that only happened once

### Recording

```bash
ros2 bag record <topic> <topic> ...   # named topics
ros2 bag record -a                    # everything
ros2 bag record -a -o <directory>     # choose the output directory
```

AutoSDV's own recorder selects the outdoor sensor topic set for you:

```bash
just bag record
```

Recording `-a` on a running stack captures a great deal — several GB per minute
with a 3-D LiDAR. Recording a named subset is usually what you want, except when
you intend to visualise the result in RViz later, which needs the transforms and
metadata topics too.

### Inspecting

```bash
ros2 bag info <bag>
```

Topics, message counts, duration and the message types — check this first when
a replay does nothing, because a bag recorded against different message
definitions will play without error and satisfy no subscriber.

### Playing

```bash
ros2 bag play <bag>
ros2 bag play <bag> --clock              # publish /clock — required for use_sim_time
ros2 bag play <bag> -r 2.0               # double speed
ros2 bag play <bag> --loop               # or -l
ros2 bag play <bag> --topics /a /b       # only these topics
ros2 bag play <bag> --start-offset 30    # skip the first 30 s
```

`--clock` is the one to remember. The logging simulation runs with
`use_sim_time:=true`, so without it the stack's clock never advances and nothing
happens at all.

`just bag play` plays the most recent recording in `rosbags/` with `--clock`
already set.

## Recording your own

For a recording that will drive a logging simulation, you need at minimum:

- the LiDAR point cloud
- the IMU
- the vehicle velocity report
- `/tf` and `/tf_static`
- GNSS, if you intend to initialise from it

`just bag record` selects this set. If you record by hand, `/tf_static` is the
one most often forgotten, and its absence produces a replay where every frame
transform is missing and nothing localizes.

To replay your own recording, point the launch at your own map:

```bash
play_launch launch autosdv_launch logging_simulation.launch.yaml \
  map_path:=/path/to/your/map
```

and validate the map first:

```bash
just map check /path/to/your/map cuda_ndt
```

See [Maps](../guides/maps.md).

## Next steps

- [Logging Simulation](../tutorial/03-logging-simulation.md)
- [COSS Park Scenario](./coss-park-scenario.md)
