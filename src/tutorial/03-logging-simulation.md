# 3. Logging Simulation

The same stack as the last page, with sensing, localization and perception put
back — fed from a recording of a real drive instead of from hardware.

One thing changes, and it changes everything: **the vehicle is no longer told
where it is.** It has to work that out, ten times a second, by matching LiDAR
returns against a map. That estimate can be slow, wrong, or lost, and watching it
succeed or fail is what this page is for.

This is what [step 1](./01-first-run.md) ran for you.

## What you need

```bash
just bag download    # ~2.8 GB, once
```

The recording lands in `data/rosbags/outdoor_20251226_153115`. `just demo run`
fetches it automatically, so you may already have it.

## Launch it — two terminals

Both terminals need [the environment](../concepts/environment.md).

**Terminal 1 — the stack:**

```bash
source /opt/autoware/1.5.0/setup.bash
source install/setup.bash

play_launch launch autosdv_launch logging_simulation.launch.yaml
```

Wait for it. The point cloud map is 4.9 million points and takes tens of seconds
to load; the scan matcher cannot do anything until it has finished. Starting the
recording early simply wastes the beginning of it.

**Terminal 2 — the data:**

```bash
source /opt/autoware/1.5.0/setup.bash
source install/setup.bash

ros2 bag play data/rosbags/outdoor_20251226_153115 --clock
```

!!! danger "`--clock` is not optional"

    `logging_simulation.launch.yaml` sets `use_sim_time:=true`, so the entire
    stack runs on simulated time taken from the `/clock` topic. Without
    `--clock`, nothing publishes that topic, the clock never advances, and the
    system sits there doing nothing at all — with no error.

    This is the single most common failure on this page. If nothing happens,
    check this first:

    ```bash
    ros2 topic echo /clock --once
    ```

??? note "The `just` shortcut"

    ```bash
    just sim logging
    just sim logging ARGS="pose_source:=ndt"
    ```

## If you have no NVIDIA GPU

The defaults assume one: `pose_source:=cuda_ndt` needs CUDA, and perception
loads TensorRT models. Two arguments remove both requirements:

```bash
play_launch launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=ndt \
  launch_perception:=false
```

- `pose_source:=ndt` selects Autoware's OpenMP **CPU** scan matcher. Slower per
  scan, and correct.
- `launch_perception:=false` publishes empty object lists instead of loading any
  model. Localization does not depend on perception, so if localization is what
  you are here to watch, this costs you nothing.

Measured with `pose_source:=ndt` on a desktop CPU, the matcher held the sensor's
full 10 Hz with a driving-phase p95 of 44 ms against a 100 ms budget. On a
laptop expect worse; if the pose starts lagging behind the vehicle, that is what
running out of budget looks like.

## Seeding the initial pose

NDT needs somewhere to start. It refines an estimate; it does not search the
whole map.

- **In RViz** — `2D Pose Estimate`, as in the planning simulation, clicked near
  where the recording starts.
- **Automatically** — `just demo run` publishes a known pose 8 seconds into
  playback, which is what makes its results reproducible. Without that seed, two
  runs of the same recording can differ.

The recording's own GNSS is *not* used (`use_gnss:=false`): it is single-point
with about 20 m of scatter and disagrees with the direction of travel, so
letting it seed localization puts the vehicle somewhere different every time.
That is a realistic problem, not a quirk of this dataset.

## Watch it

!!! warning "The vehicle is parked for the first 116 seconds"

    The recording is 157 seconds long. First motion above 0.2 m/s is at
    **+116.3 s**, then it drives for 41 seconds at up to 1.58 m/s. Two minutes
    of a stationary vehicle is the recording.

**In RViz**, the thing to look at is the live point cloud against the map. If
localization is working, the cloud sits *on* the map — walls line up with walls.
If it is failing, the cloud slides relative to the map, or sits beside it at a
constant offset. That visual is more reliable than any node's self-reported
state.

**In a third terminal**, the numbers:

```bash
ros2 topic hz /localization/pose_estimator/pose_with_covariance
```

Expect ~10 Hz, matching the LiDAR, because NDT runs once per scan. A rate well
below that means the matcher is not keeping up.

AutoSDV ships checks for the distinction between "the node is running" and
"localization is working", which are not the same claim:

```bash
python3 scripts/testing/localization/check_ndt_activated.py
python3 scripts/testing/localization/ndt_quality_report.py
python3 scripts/testing/localization/ndt_alignment_report.py
```

## Reading the result

The metrics `just demo run` prints — and that these scripts produce — are worth
understanding once.

| Metric | Meaning | Healthy |
|---|---|---|
| **NVTL** | how well the scan matched the map | steady, no collapses |
| **TP** | the other match score | steady |
| **`exe_ms`** | time for one scan match | p95 well under 100 ms |
| **iterations** | optimiser steps per match | low single digits |
| **publish gap** | time between poses | ~0.100 s |

The most informative split is **`init` versus `track`**: while parked, matching
is easy, because each scan resembles the last. Once the vehicle moves, the same
work costs roughly twice as much. In a measured run: 7.8 ms parked, 16.7 ms
driving, p95 44 ms. If a machine is marginal, that is where it fails — not at
startup.

## Troubleshooting

**Nothing happens when the bag plays.** `--clock`. See above.

**The pose never appears, or jumps to the origin.** No initial pose was set, or
it was set too far from the truth for NDT to converge. Set it again, closer.

**The cloud drifts away from the map partway through.** The match was lost.
Watch NVTL — it will have collapsed before the visual drift was obvious.

**Topics exist but nothing subscribes.** Compare QoS on both ends:

```bash
ros2 topic info -v /sensing/lidar/concatenated/pointcloud
```

A `RELIABLE` subscriber will not receive from a `BEST_EFFORT` publisher. See
[Inspecting a Running System](../concepts/inspecting.md).

**Perception takes minutes to start.** TensorRT compiling engines. Run
`just setup-autoware-data` then `just build-engines` once — without the first,
the engines cannot be cached and it happens on *every* launch.

## Compare the localization methods

The replay is the right place to compare them, because the input is byte for
byte identical every run:

```bash
play_launch launch autosdv_launch logging_simulation.launch.yaml pose_source:=cuda_ndt
play_launch launch autosdv_launch logging_simulation.launch.yaml pose_source:=ndt
play_launch launch autosdv_launch logging_simulation.launch.yaml pose_source:=mcl
```

`mcl` needs an occupancy grid rather than a point cloud map, and a 2-D
`LaserScan` rather than a 3-D cloud. See
[Localization Methods](../guides/localization-methods.md).

## What you have now seen

| | Planning simulation | Logging simulation |
|---|---|---|
| Pose | given | **estimated, and it can fail** |
| Objects | placed by you | detected from real returns |
| Sensor data | none | a real recording |
| Hardest part | choosing a good goal | staying localized |

**Next:** [4. Behind the Recipe](./04-behind-the-recipe.md) — what `just demo run`
was actually doing all along.
