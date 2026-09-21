# Inspecting a Running System

When something does not work, the question is almost always one of two:

1. **Is it publishing, and at the rate it should be?**
2. **Why is my subscriber getting nothing, when the publisher is clearly there?**

This page is the small set of commands that answer those, which is what you will
actually use day to day on AutoSDV.

Every command here needs a terminal with the environment sourced — see
[The Environment](./environment.md).

## What is running

```bash
ros2 node list
ros2 topic list
```

`ros2 node list` is also the orphan check. After stopping the system it should
be empty; anything still listed is a process that outlived its launcher.

For one node, what it publishes, subscribes to, and offers:

```bash
ros2 node info /localization/pose_estimator/ndt_scan_matcher
```

## Is it publishing at the right rate?

```bash
ros2 topic hz /sensing/lidar/concatenated/pointcloud
```

This prints the measured rate until you stop it. It is the single most useful
command in this list, because on a vehicle most faults are *rate* faults rather
than absence faults — the data is there, but slower or burstier than the
consumer needs.

Some known-good numbers for AutoSDV:

| Topic | Expected |
|---|---|
| LiDAR point cloud | the sensor's rate — 10 Hz for the VLS128 and Robin-W |
| `/localization/pose_estimator/pose_with_covariance` | matches the LiDAR, since NDT runs per scan |
| IMU | far faster, typically 100–400 Hz depending on source |

`ros2 topic hz` also reports the standard deviation and min/max interval. A rate
that averages correctly with a large spread is a different problem from a rate
that is simply low, and usually points at CPU contention.

Two related commands:

```bash
ros2 topic bw /sensing/lidar/concatenated/pointcloud   # bandwidth
ros2 topic delay /some/topic                            # header stamp vs now
```

`ros2 topic delay` only works on messages with a header, and it measures
*latency*, which is what you want when the rate looks fine but the system behaves
as though data is stale.

## Why is nothing arriving?

First, is anyone actually connected:

```bash
ros2 topic info -v /sensing/lidar/concatenated/pointcloud
```

The `-v` is the important part. Without it you get counts; with it you get every
publisher and subscriber **and the QoS profile of each**. That output is the
answer to most "the topic exists but my node sees nothing" problems.

### QoS, and the one incompatibility that bites

ROS 2 lets each publisher and subscriber declare a Quality of Service profile.
Two of its settings cause nearly all the trouble here.

**Reliability.** `RELIABLE` retransmits until delivery; `BEST_EFFORT` does not.
The rule is asymmetric and worth memorising:

| Publisher | Subscriber | Connects? |
|---|---|---|
| RELIABLE | RELIABLE | yes |
| RELIABLE | BEST_EFFORT | yes |
| **BEST_EFFORT** | **RELIABLE** | **no** |
| BEST_EFFORT | BEST_EFFORT | yes |

A reliable subscriber will not connect to a best-effort publisher. Sensor data
is usually published best-effort — a dropped LiDAR scan is better than a delayed
one — so a tool or node that asks for reliability silently receives nothing.
There is no error. The topic lists, the publisher is visible, and no message
arrives.

**Durability.** `TRANSIENT_LOCAL` keeps the last message for subscribers that
join later; `VOLATILE` does not. Latched data uses it — the map, `/tf_static`.
A subscriber that asks for `VOLATILE` on a transient-local topic connects, and
then waits forever for a message that was already sent.

So when `ros2 topic info -v` shows a publisher and a subscriber that are not
exchanging anything, compare their two QoS blocks before looking anywhere else.

!!! note "QoS is not only a debugging concern here"

    Reliability on a high-rate topic has a real cost. In this project a LiDAR
    driver publishing its cloud as RELIABLE was measured discarding roughly 60 %
    of the sensor's points, because the reliable writer blocked the same thread
    that parses incoming packets. The fix was to offer best-effort. If you are
    choosing a QoS for a sensor topic, that is the precedent.

## Looking at the data

```bash
ros2 topic echo /localization/pose_estimator/pose_with_covariance
ros2 topic echo /vehicle/status/velocity_status --once
ros2 topic echo /some/topic --field header.stamp
```

`--once` prints one message and exits, which is what you want for anything large.
Echoing a point cloud without it will fill your terminal with several megabytes
per second.

If `echo` prints nothing, that is the QoS question above — `echo` picks a
profile, and it can be the incompatible one. `ros2 topic echo --qos-reliability
best_effort <topic>` is the quick test.

## Parameters

```bash
ros2 param list /node_name
ros2 param get /node_name parameter_name
ros2 param set /node_name parameter_name value
```

`ros2 param get` is how you confirm that a launch argument actually reached the
node — the end of the chain described in [Launch Files](./launch-files.md). A
value you passed that does not appear here did not arrive.

## Transforms

```bash
ros2 run tf2_tools view_frames        # writes a PDF of the whole TF tree
ros2 run tf2_ros tf2_echo base_link map
```

Missing transforms are a common cause of a system that runs without error and
produces nothing useful. `/tf_static` in particular is easy to omit when
recording a bag, and its absence leaves every frame relationship unresolved.

## Diagnostics

Autoware nodes publish health on `/diagnostics`:

```bash
ros2 topic echo /diagnostics
```

This is more informative than it first looks — it is where a node says it is
alive but not converged, which is exactly the distinction between "localization
is running" and "localization is working".

AutoSDV ships purpose-built checks for the localization case:

```bash
python3 scripts/testing/localization/check_ndt_activated.py    # activated, or merely alive?
python3 scripts/testing/localization/ndt_quality_report.py     # pose quality
python3 scripts/testing/localization/check_imu_velocity.py     # the EKF's two inputs
```

## The web UI

When the system was started with `play_launch`, a browser page lists every node
with its state, its logs, and per-process CPU, memory and GPU:

- `http://127.0.0.1:8080` — `play_launch` default
- `http://localhost:8081` — what `just launch` uses, because it passes
  `--web-addr 0.0.0.0:8081`

Worth knowing both, because opening the wrong one shows nothing and looks like a
failure.

## A short debugging order

When something does not work, in this order:

1. `ros2 node list` — is the node even running?
2. `ros2 topic list` — does the topic exist?
3. `ros2 topic info -v <topic>` — publisher and subscriber, and their QoS
4. `ros2 topic hz <topic>` — arriving, and fast enough?
5. `ros2 topic echo <topic> --once` — is the content sane?
6. `ros2 param get <node> <param>` — did the configuration arrive?
7. `/diagnostics` — does the node itself say it is unhappy?

Most problems are answered by step 3.

## Beyond the generic tools

Everything above is plain ROS 2, and works on any system. This project also
ships tools of its own:

- [Localization Diagnostics](../guides/localization-diagnostics.md) — purpose-built
  scripts that answer localization questions `ros2 topic` cannot: is NDT
  *activated* or merely alive, how good is the pose, what is the scan-to-map
  residual.
- [Command Reference](../reference/commands.md#tool--development-and-monitoring) — `just tool tui` is a
  drive monitor and control surface (pose, speed, component states, and the
  initialise/route/engage sequence without RViz); `just tool plotjuggler` plots
  any topic over time.
- `/system/fail_safe/mrm_state` and `/system/operation_mode/availability` are
  the two topics to echo when the vehicle stops on its own — see
  [Minimum Risk Manoeuvre](../guides/mrm.md).

## Next

- [Autoware Conventions](./autoware-conventions.md) — what topic names tell you
- [The Environment](./environment.md)
