# Verifying the Installation

Four checks, in order of how much they prove. Run them after
[Software Installation](./overview.md). Each one tells you something the
previous one could not.

## 1. What does setup think is installed?

```bash
./setup.sh --status
```

Every step is listed with its state. Remember that this command **reads the
machine** where it can, so its answer can differ from what you remember running
— and where they differ, it is usually right.

What matters here is that nothing you selected reports as missing. A step you
deliberately skipped (`zed-sdk` without a ZED camera, say) reporting as not
installed is the correct result, not a failure.

## 2. Does the environment activate?

Open a **new** shell and enter the repository:

```bash
cd ~/AutoSDV
ros2 --help
```

If `ros2` is not found, direnv is not hooked into your shell, or you have not
run `direnv allow`. Source the environment by hand to confirm that is the only
problem:

```bash
source /opt/autoware/1.5.0/setup.bash
source install/setup.bash
ros2 --help
```

## 3. Does the workspace build?

```bash
just build
```

Expected: colcon finishes with a `Summary:` line and no failed packages.
Warnings on stderr are normal — several vendored packages produce them.

Confirm the Rust package in particular, because it is the one that fails
quietly:

```bash
ls install/cuda_ndt_matcher
```

If that directory does not exist, colcon skipped the package. See
[the troubleshooting section](./recommended.md#the-build-succeeds-but-cuda_ndt_matcher-is-absent).

## 4. Does a system actually come up?

This is the check that matters. Two simulations, in increasing order of what
they exercise.

### The planning simulation — no sensors, no GPU, no map download

```bash
source install/setup.bash
play_launch launch autoware_launch planning_simulator.launch.xml \
  map_path:=$PWD/data/COSS-map-planning \
  vehicle_model:=autosdv_vehicle \
  sensor_model:=autosdv_sensor_kit
```

RViz opens with the COSS Park map. Set an initial pose, set a goal, and the
vehicle plans a route and drives it. If that works, your ROS 2 installation,
your Autoware installation, your build and the AutoSDV vehicle model are all
sound.

Full walkthrough: [Planning Simulation](../../tutorial/02-planning-simulation.md).

### The logging simulation — the full pipeline, on recorded sensor data

```bash
just coss download-rosbag          # ~2.8 GB, once
```

```bash
source install/setup.bash
play_launch launch autosdv_launch logging_simulation.launch.yaml
```

and in a second terminal:

```bash
ros2 bag play data/rosbags/outdoor_20251226_153115 --clock
```

This exercises localization and perception, which the planning simulation does
not. It is also the first check that needs a GPU under the default settings —
see [Logging Simulation](../../tutorial/03-logging-simulation.md) for the CPU-only
arguments.

## A checklist

- [ ] `./setup.sh --status` shows nothing missing that you selected
- [ ] `ros2 --help` works in a fresh shell inside the repository
- [ ] `just build` completes with no failed packages
- [ ] `install/cuda_ndt_matcher` exists
- [ ] the planning simulation opens RViz and drives to a goal
- [ ] the logging simulation localizes against the recorded bag

## If something failed

The [troubleshooting section](./recommended.md#troubleshooting) of the installation
page covers the failures that have actual causes worth naming — a silently
skipped Rust package, kernel socket buffers too small for any node to start, a
loopback interface that lost multicast across a reboot, and models that
recompile on every launch.
