# Verifying the Installation

Four quick checks that the software is on disk and visible to ROS. None of them
starts a vehicle stack — that is what
[the tutorial](../../tutorial/00-what-you-will-build.md) is for, and it is the
real proof.

## 1. What does setup think is installed?

```bash
./setup.sh --status
```

Nothing you selected should report as missing. A step you skipped on purpose —
`zed-sdk` with no ZED camera — reporting as not installed is the correct result.

This command reads the machine where it can, so its answer can differ from what
you remember running. Where they differ, it is usually right.

## 2. Is Autoware where it should be?

```bash
ls /opt/autoware/1.5.0/setup.bash      # the environment script
ls data/autoware_data | head           # the writable model tree
```

Both must exist. The second is a farm of symlinks the setup program creates; if
it is missing, perception rebuilds its models on every launch.

## 3. Can ROS see the packages?

Source the environment, then ask ROS what it has:

```bash
source /opt/autoware/1.5.0/setup.bash  # includes ROS 2
source install/setup.bash              # the workspace you just built

ros2 pkg list | wc -l                  # hundreds, not zero
ros2 pkg list | grep autosdv           # this project's packages
ros2 pkg prefix autoware_launch        # Autoware, from the Debian install
```

`grep autosdv` should name the launcher, the sensor kit and the vehicle
packages. If `ros2` itself is not found, the environment is not sourced — which
is the whole subject of [The Environment](../../concepts/environment.md).

## 4. Did the Rust package build?

```bash
ls install/cuda_ndt_matcher
```

Worth its own check because it is the one that fails quietly: without the Rust
toolchain or the colcon plugin, colcon skips it and the build still reports
success. See
[the troubleshooting section](./recommended.md#the-build-succeeds-but-cuda_ndt_matcher-is-absent).

## Then run something

```bash
just demo check
```

Reports the rosbag, the map, the build, `play_launch`, the CUDA toolkit against
your GPU, and whether a display is available.

After that, go to [the tutorial](../../tutorial/00-what-you-will-build.md). It
starts the planning simulation — no sensors, no GPU, no downloads — and if that
drives to a goal, every layer underneath it is sound.

## If something failed

[Troubleshooting](./recommended.md#troubleshooting) on the installation page
covers the failures with causes worth naming: a silently skipped Rust package,
kernel socket buffers too small for any node to start, a loopback interface that
lost multicast across a reboot, and models that recompile on every launch.
