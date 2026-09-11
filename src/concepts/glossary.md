# Glossary

Terms used throughout this book, in the sense AutoSDV uses them.

## Software and environment

**Workspace** — a directory whose `src/` holds ROS 2 packages, built with colcon
into `build/` and `install/`. AutoSDV's repository root is one.

**Package** — the unit ROS 2 installs and finds things by. A launch file is
addressed as `<package> <file>`.

**Overlay / underlay** — sourced environments stack. Each `source …/setup.bash`
adds an overlay over what came before; the layers beneath are the underlay.
AutoSDV's workspace is an overlay over Autoware, which is an overlay over ROS 2.
See [The Environment](./environment.md).

**colcon** — the build tool. `just build` wraps it with the flags this project
needs.

**rosdep** — resolves the system and ROS dependencies that packages declare,
which is why AutoSDV has no per-driver `apt install` steps.

**direnv** — the tool that reads `.envrc` and sources the environment when you
enter the directory. A convenience over the two `source` lines, not a
replacement for understanding them.

## Launch

**Launch file** — a file that starts nodes with parameters and remappings, and
includes other launch files. Written in YAML, XML or Python.

**Launch argument** — an input to a launch file, passed as `name:=value`. Exists
only while the launch file is evaluated. Always a string.

**Parameter** — a named value belonging to a running node, readable with
`ros2 param get`.

**Substitution** — a value computed during launch evaluation: `$(var x)`,
`$(find-pkg-share pkg)`, `$(env NAME default)`, `$(eval "...")`.

**Preset** — a launch file that declares only argument defaults, selected by
name. AutoSDV has perception and localization presets.

**Composable node** — a node that loads into a shared process rather than
running as its own, so messages pass in-process.

**Component container** — the process that hosts composable nodes. Killing a
launcher by PID rather than by process group leaves these running as orphans.

**`play_launch`** — AutoSDV's launch runner, a drop-in for `ros2 launch` that
adds process-group shutdown, a web UI and monitoring. Our own software; see
[play_launch](../tutorial/06-play-launch.md) for when to fall back.

## Messaging

**Node** — a process (or composable node) that participates in the ROS graph.

**Topic** — a named channel. Publishers write, subscribers read.

**QoS** — Quality of Service, the per-endpoint policy that decides whether a
publisher and subscriber connect at all. A `RELIABLE` subscriber will not
receive from a `BEST_EFFORT` publisher. See
[Inspecting a Running System](./inspecting.md).

**TF** — the transform system: the tree of coordinate frames and the
relationships between them. `/tf` carries changing transforms, `/tf_static` the
fixed ones.

**rosbag** — a recording of topic traffic, replayed with `ros2 bag play`.
`--clock` is required when the stack runs on simulated time.

## Vehicle state

**Pose** — position and orientation. In this book, usually the vehicle's pose in
the `map` frame.

**Twist** — linear and angular velocity. Estimated separately from pose;
`localization_preset` selects the twist source.

**Odometry** — motion estimated by integrating wheel rotation and IMU. Smooth
and locally accurate, drifts without bound.

**`base_link`** — the vehicle body frame, at the centre of the rear axle. Sensor
frames are defined relative to it.

## Localization

**NDT** — Normal Distributions Transform. Matches a 3-D scan against a point
cloud map by representing the map as a grid of Gaussians. AutoSDV's default
(`cuda_ndt`, GPU) with a CPU fallback (`ndt`).

**MCL** — Monte Carlo Localization. A particle filter matching a 2-D `LaserScan`
against an occupancy grid.

**PCD** — Point Cloud Data, the file format of the 3-D map NDT matches against.

**Occupancy grid** — a 2-D map of free, occupied and unknown cells, as a `.pgm`
image plus a `.yaml` describing its resolution and origin. What MCL needs
instead of a PCD.

**lanelet2** — the road-network map format: lanes, stop lines, traffic rules.
Used by planning, and by perception for filtering. Stored as `lanelet2_map.osm`.

**NVTL** — Nearest Voxel Transformation Likelihood, a score NDT reports for how
well a scan matched the map. Low values mean a poor match.

**TP** — Transformation Probability, the other NDT match score.

**cuVSLAM / cuVGL** — NVIDIA Isaac's visual odometry and global localization,
used by `pose_source:=isaac` and `pose_source:=visual`.

## Sensing and perception

**Deskew** — correcting a scan for the vehicle's motion during the sweep.
Requires a per-point time offset, which not every LiDAR driver provides.

**Ring** — one constant-elevation scan line of a spinning LiDAR. Ring extraction
is how a 2-D `LaserScan` is produced from a 3-D sensor.

**Voxel grid** — downsampling by keeping one point per cell of a 3-D grid.

**CenterPoint** — the LiDAR 3-D object detection model AutoSDV runs.

**TensorRT** — NVIDIA's inference runtime. It compiles an `.onnx` model into an
`.engine` tied to both the TensorRT version and the specific GPU, which is why
engines cannot be built elsewhere and shipped.

**`cuda_blackboard`** — the mechanism CUDA pipeline stages use to pass GPU
pointers. Process-local, which is why all CUDA stages must share one container.

## Operation

**MRM** — Minimum Risk Manoeuvre. What the system does when it must stop safely.

**ODD** — Operational Design Domain: the conditions a system is designed to
operate in.

**Planning simulator** — simulation with a kinematic vehicle model and no
sensors. Exercises planning and control.

**Logging simulation** — replay of recorded sensor data through the real stack.
Exercises localization and perception as well.

**Mapless mode** — operating with no map and no localization, for indoor use.

## Next

- [The Environment](./environment.md)
- [The Autoware Pipeline](./autoware-conventions.md)
