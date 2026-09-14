# Operating the Vehicle

This page teaches you to launch AutoSDV yourself, rather than to run one command
that launches it for you. The difference matters: everything you will want to
change — which LiDAR, which localization method, whether perception runs at all
— is a **launch argument**, and arguments are something you pass, not something
a wrapper passes for you.

Before starting, complete [Software Installation](./installation/overview.md)
and build the workspace.

## The launch command

```bash
source install/setup.bash
play_launch launch autosdv_launch autosdv.launch.yaml
```

Three positions, and it is worth naming them:

| Position | Value | Meaning |
|----------|-------|---------|
| the tool | `play_launch launch` | the launch orchestrator |
| the package | `autosdv_launch` | an installed ROS 2 package |
| the launch file | `autosdv.launch.yaml` | a file inside that package's `launch/` directory |

`autosdv.launch.yaml` declares the whole driving system: the vehicle interface,
sensing, localization, perception, planning, control, and the arguments that
select between their variants.

## Passing arguments

Arguments follow the launch file, as `name:=value` — note the colon-equals, not
a plain `=`:

```bash
play_launch launch autosdv_launch autosdv.launch.yaml pose_source:=ndt
```

Several at once, in any order:

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  sensor_suite:=robin_zed \
  pose_source:=ndt \
  launch_perception:=false
```

Four rules cover nearly every mistake:

- **Every value is a string.** There are no typed arguments; `launch_perception`
  takes the *text* `true` or `false`.
- **Booleans are lowercase** `true` / `false`. `True` and `1` are not accepted.
- **An empty value is meaningful.** Several sensor arguments default to `""`,
  which means "take whatever the sensor suite chose". Passing
  `lidar_model:=""` explicitly is the same as not passing it.
- **Quote anything with spaces or shell metacharacters**, because your shell
  parses the line before ROS does.

### Seeing what your arguments actually did

An argument you spelled wrong is not always an error — it may simply be a
different argument that nothing reads. To see the launch as it resolves, with
every node and parameter it will start:

```bash
play_launch resolve autosdv_launch autosdv.launch.yaml \
  pose_source:=mcl -o ./tmp/resolved.yaml
```

This answers "did my argument take effect" definitively, and it costs nothing —
it starts no nodes.

## Why `play_launch` and not `ros2 launch`

`play_launch launch` is a drop-in replacement for `ros2 launch`, taking the same
package, file and arguments. Use it. What it adds:

- **It shuts the system down properly.** AutoSDV runs most of its nodes as
  composable nodes inside container processes. Killing `ros2 launch` kills the
  parent and leaves those containers running as orphans, still holding the
  GPU, still publishing. `play_launch` escalates SIGINT → SIGTERM → SIGKILL
  across the whole process group.
- **A web UI**, by default at `http://127.0.0.1:8080`, listing every node with
  its state and its logs.
- **Resource monitoring** — per-process CPU, memory, I/O and GPU — and the
  `/diagnostics` topic.
- **A resolve/replay workflow**: `play_launch dump launch …` writes a resolved
  system model once, and `play_launch up system_model.yaml` starts it as often
  as you like without re-resolving.

It installs from PyPI, and the setup program installs it for you:

```bash
pip install play_launch
play_launch setcap      # optional: per-process I/O monitoring, non-root RT scheduling
```

### If you only have `ros2 launch`

The same line works:

```bash
ros2 launch autosdv_launch autosdv.launch.yaml pose_source:=ndt
```

But **do not kill it with `Ctrl-C` alone and assume the system is gone**, and
never `kill -9` its PID — that is exactly what orphans the containers. Kill the
process group:

```bash
ros2 launch autosdv_launch autosdv.launch.yaml &
LAUNCH_PID=$!
# ... testing ...
kill -- -$(ps -o pgid= -p $LAUNCH_PID | tr -d ' ')
```

To check for orphans afterwards, `ros2 node list` should be empty.

### The `just launch` shortcut

The repository provides a convenience recipe:

```bash
just launch
just launch "pose_source:=ndt launch_perception:=false"
```

It is a thin wrapper, and it is worth knowing exactly what it adds, because the
additions are invisible from the command line:

```bash
play_launch launch --web-addr 0.0.0.0:8081 autosdv_launch autosdv.launch.yaml [ARGS]
# and, when $DISPLAY is unset, also: rviz:=false
```

Three consequences:

- the web UI is on **port 8081**, not play_launch's own default of 8080
- it binds `0.0.0.0`, so the UI is reachable from other machines on the network
- over SSH without X forwarding, RViz is disabled for you — convenient, but it
  means the same command behaves differently depending on your environment

All arguments go inside one quoted `ARGS=` string, which is a `just` requirement
rather than a ROS one.

## The arguments

Defaults are from `autosdv.launch.yaml`. This is the full set; the ones you will
actually reach for are in the first three tables.

### Choosing sensors

| Argument | Default | Meaning |
|----------|---------|---------|
| `sensor_suite` | `vlp32c_zed_imu` | Predefined combination: `robin_zed`, `robin_zed_mpu`, `vlp32c_zed`, `vlp32c_zed_mpu`, `vlp32c_zed_imu`, `cube1_usb`, `custom` |
| `lidar_model` | *(suite)* | `cube1`, `robin-w`, `vlp32c` |
| `camera_model` | *(suite)* | `zedxm`, `usb`, `none` |
| `imu_source` | *(suite)* | `mpu9250`, `zed` |
| `gnss_receiver` | *(suite)* | `ublox`, `septentrio`, `garmin` |
| `use_gnss` | *(suite)* | GNSS for outdoor operation |
| `use_ntrip` | `true` | NTRIP client for RTK corrections (u-blox only) |
| `enable_zed_object_detection` | *(suite)* | ZED camera object detection |

*(suite)* means the argument defaults to `""` and the sensor suite supplies the
value. Set one explicitly to override just that sensor.

### Choosing localization

| Argument | Default | Meaning |
|----------|---------|---------|
| `pose_source` | `cuda_ndt` | `cuda_ndt`, `ndt`, `mcl` — see [Localization Methods](../guides/localization-methods.md) |
| `pose_source_package` | `auto` | Derived from `pose_source`. Set explicitly only to plug in a third-party estimator |
| `localization_preset` | `default` | `default` (gyro odometry) or `eagleye` (GNSS odometry) |
| `use_mapless_mode` | `false` | Indoor operation with no localization at all |
| `map_path` | `./data/COSS-map-planning` | Map directory |
| `occupancy_grid_file` | `occupancy_grid.yaml` | Grid metadata filename inside `map_path` (`pose_source:=mcl` only) |
| `mcl_random_seed` | `-1` | Particle filter RNG seed; `-1` is nondeterministic (`mcl` only) |

### Choosing perception

| Argument | Default | Meaning |
|----------|---------|---------|
| `perception_preset` | `lidar_only` | `lidar_only`, `camera_lidar_fusion`, `minimal` — see [Presets](../guides/presets.md) |
| `launch_perception` | `true` | When `false`, empty object lists are published and no model is loaded |
| `perception_input_pointcloud` | `/sensing/lidar/concatenated/pointcloud` | Cloud feeding ground/obstacle segmentation |

### Point cloud backends

| Argument | Default | Meaning |
|----------|---------|---------|
| `pointcloud_backend` | `cpu` | `cpu` or `cuda`: sensing-side preprocessing (crop-self, deskew, ring outlier) |
| `localization_pointcloud_backend` | `cpu` | `cpu` or `cuda`: the NDT input chain (crop box, voxel grid, random downsample) |
| `input_pointcloud` | `/sensing/lidar/concatenated/pointcloud` | Cloud feeding the localization chain |

Both backends are whole-stage switches and neither can be half-applied. See
[the CUDA pipeline](../guides/cuda-pipeline.md).

### Turning modules off

Each defaults to `true`. Useful for isolating a problem, and for running on a
machine that cannot support everything.

| Argument | Turns off |
|----------|-----------|
| `launch_vehicle` | the vehicle interface |
| `launch_system` | system monitoring and MRM |
| `launch_map` | map loading |
| `launch_sensing` | sensing |
| `launch_sensing_driver` | the sensor drivers alone, keeping the rest of sensing |
| `launch_localization` | localization |
| `launch_planning` | planning |
| `launch_control` | control |
| `launch_perception` | perception |
| `launch_system_monitor` | the AutoSDV system monitor (up to 25 % of a core on Jetson) |

### Everything else

| Argument | Default | Meaning |
|----------|---------|---------|
| `is_simulation` | `false` | Simulation mode: disables PWM output to the hardware |
| `use_sim_time` | `false` | Use the simulated clock |
| `data_path` | `$AUTOSDV_DATA_PATH`, else `./data/autoware_data` | Autoware model directory. **Must be writable**, or TensorRT cannot cache its engines |
| `vehicle_model` | `autosdv_vehicle` | Vehicle description package prefix |
| `sensor_model` | `autosdv_sensor_kit` | Sensor kit description package prefix |
| `rviz_config` | the AutoSDV layout | RViz layout to load |


### Worked examples

Indoor, no GNSS, no map:

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  use_gnss:=false use_mapless_mode:=true
```

Robin-W with the CUDA pipeline end to end:

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  sensor_suite:=robin_zed \
  pointcloud_backend:=cuda \
  localization_pointcloud_backend:=cuda \
  pose_source:=cuda_ndt
```

Camera-LiDAR fusion with traffic lights:

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  sensor_suite:=robin_zed \
  perception_preset:=camera_lidar_fusion
```

Localization only, to debug a pose problem without perception in the way:

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  launch_perception:=false launch_planning:=false launch_control:=false
```

## Editing the launch file instead

You can also change defaults at the source:

```
src/launcher/autosdv_launch/launch/autosdv.launch.yaml
```

Because the workspace is built with `--symlink-install`, edits to `.yaml`,
`.xml` and `.py` files take effect on the next launch with no rebuild. **New**
files still need `just build`, to create their symlinks.

Prefer arguments for anything you might want to change back.

## Everyday commands

The repository's `just` recipes are grouped into modules. `just` on its own
lists everything; `just <module>` lists one module.

```bash
just              # every recipe
just tool         # just this module's recipes
```

Both spellings work: `just tool rviz` and `just tool::rviz`.

### Tools

```bash
just tool rviz          # RViz with the AutoSDV layout
just tool plotjuggler   # PlotJuggler
just tool tui           # drive monitor TUI: pose, speed, component states
just tool controller    # keyboard manual control
just tool zed           # the ZED camera node alone, for camera testing
```

### Control testing

```bash
just control basic      # the vehicle control test
just control straight   # a 10 m straight trajectory
just control circle     # a circular trajectory
```

### Recording and playback

```bash
just bag record         # record the outdoor sensor topics
just bag play           # play the most recent recording
just coss download-rosbag       # fetch the test rosbag (~2.8 GB)
```

### Simulation

```bash
just coss planning-sim       # planning simulator, no sensors needed
just coss logging-sim        # rosbag replay
just sim coss-park      # the full COSS Park scenario
```

See the [Simulation Guide](../tutorial/02-planning-simulation.md).

### Maps

```bash
just map check <map_dir> [pose_source]
just map grid-from-pcd <map_dir>
just map grid-from-bag <bag> <map_dir>
```

See [Maps](../guides/maps.md).

## Stopping the system

`Ctrl-C` in the `play_launch` terminal. It escalates through the process group,
so composable nodes go down with it.

If something is left behind:

```bash
ros2 node list      # should be empty
```

Logs are written to `play_log/latest/`.

## Next steps

- [Planning Simulation](../tutorial/02-planning-simulation.md) — the first thing
  to run, and it needs no sensors
- [Localization Methods](../guides/localization-methods.md) — what `pose_source`
  selects between
- [Presets](../guides/presets.md) — how `perception_preset` and
  `localization_preset` work, and how to add one
