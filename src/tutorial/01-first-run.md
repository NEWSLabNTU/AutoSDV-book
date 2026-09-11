# 1. First Run

One command. It fetches the data, brings the whole stack up, localizes against a
real map using real recorded LiDAR, replays a drive, and prints how well it did.

Run it before you understand it. Seeing the finished thing work is worth more
than reading about it, and everything after this page takes it apart.

## Check first

```bash
cd ~/AutoSDV
just demo check
```

Expected:

```
COSS NDT demo prerequisites:
  ok   rosbag  …/data/rosbags/outdoor_20251226_153115
  ok   map     …/data/COSS-map-planning
  ok   workspace built
  ok   cuda_ndt_matcher is a release build (15 MB)
  ok   play_launch
```

A `warn no DISPLAY; RViz disabled (headless is fine)` line is not a problem —
it means you are on a machine without a graphical session and the run will
proceed without the visualisation.

If the rosbag is missing, the next command downloads it (about 2.8 GB). If the
workspace is not built, go back to
[the installation](../getting-started/installation/overview.md).

## Run it

```bash
just demo run
```

This takes a few minutes, most of it the 157-second replay. What it does, in
order:

1. fetches the rosbag if missing
2. stops any stack left from a previous run — two would fight over the same
   topics
3. launches the logging simulation with `pose_source:=cuda_ndt` and
   `use_gnss:=false`
4. waits for `ndt_scan_matcher` to exist, then gives the 4.9-million-point map
   25 seconds to load
5. starts the wheel-speed scaler
6. records every localization diagnostic
7. replays the bag, seeding a known starting pose 8 seconds in
8. prints the metrics and **leaves the stack running** so you can look at it

## What you will see

First the setup lines:

```
[demo] output   …/tmp/demo-runs/coss-ndt_20260912_054957
[demo] pose_source=cuda_ndt  use_gpu=1  rviz=false  scale=0.5  seed_pose=true  rate=1.0
[demo] stack pgid=1184896  (just demo stop)
[demo] initial pose will be seeded 8s into playback
[demo] replaying 157s: parked ~115s, then a 41s drive
```

!!! warning "Nothing moves for the first two minutes, and that is correct"

    The recording is 157 seconds and the vehicle **does not move until 116
    seconds in**. It was parked while the sensors ran. Two minutes of a
    stationary vehicle is the recording, not a failure.

Then the report. The numbers below are from a real run on a desktop machine, so
yours will differ, but the shape should not:

```
bag sim-time span: 1766734277.2 .. 1766734433.9
first motion (>0.2 m/s) at t=1766734393.5 (+116.3s)

nvtl:
  all     n=1412 mean=4.595 min=4.343 p50=4.589 p95=4.725 max=4.808

exe_ms:
  all     n=1412 mean=10.355 min=1.241 p50=5.470 p95=32.823 max=147.804
  init    n=1010 mean=7.814
  track   n=402  mean=16.739 p95=44.272

published ndt poses: 1395   ekf: 5222
ndt publish gaps [s]: n=1394 mean=0.100 p50=0.100 p95=0.101 max=0.200
```

Three numbers are worth reading, because they are the ones that tell you it
worked:

| | What it means |
|---|---|
| **`nvtl` mean 4.6** | how well each scan matched the map. Higher is better; consistently low means the vehicle does not know where it is |
| **`exe_ms` p95** | how long a scan match took. The budget is 100 ms, because the LiDAR runs at 10 Hz. Exceed it consistently and localization falls behind the vehicle |
| **`ndt publish gaps` mean 0.100** | poses came out at 10 Hz, matching the sensor. This is the headline result: localization kept up |

Note `init` versus `track`. While the vehicle is parked, matching is easy and
fast (mean 7.8 ms). Once it moves, the same work takes twice as long (16.7 ms),
because the scan no longer resembles the last one. That split is the single most
informative thing in the report.

## Stop it

The stack is still running so you can inspect it. When you are done:

```bash
just demo stop
```

Use this rather than `Ctrl-C` or `kill`. It kills the whole **process group** —
the launcher, the containers, and the composable nodes inside them. Killing the
launcher alone leaves orphans running, still holding memory and the GPU. You can
confirm afterwards:

```bash
ros2 node list    # should be empty
```

## Look at the run afterwards

```bash
just demo report        # the metrics again, for the most recent run
just demo list-runs
```

Each run writes to `tmp/demo-runs/<label>_<timestamp>/`, containing the recorded
diagnostics bag, the matcher metrics as CSV, the launch log and the node list.
They are several gigabytes each; `just demo clean` removes them.

## If it did not work

**Nothing started at all.** Kernel socket buffers — this stops every ROS 2 node,
not just this demo:

```bash
./setup.sh --rerun cyclonedds-sysctl
```

**`command not found`.** Your terminal has no environment. See
[The Environment](../concepts/environment.md).

**Perception spent minutes starting.** TensorRT compiling engines inside node
constructors. It caches, so it happens once — unless the model directory is not
writable, in which case it happens every single launch:

```bash
just setup-autoware-data
just build-engines
```

**It ran but NVTL was low and the pose wandered.** The seeded pose did not
converge. `just demo run-manual-init` lets you set it yourself in RViz.

## What just happened

You ran the **logging simulation**: the real Autoware stack, fed recorded sensor
data instead of live hardware, estimating its own position the whole way.

`just demo run` is a wrapper. It hid the launch command, the arguments, the
second terminal for the rosbag, and the pose seeding. The rest of this tutorial
un-hides them, starting with the simpler simulation.

**Next:** [2. Planning Simulation](./02-planning-simulation.md) — the same stack
with sensing and localization removed, so you can see the planner on its own.
