# Localization Diagnostics

`ros2 topic hz` tells you a topic is publishing. It does not tell you the pose
on it is right, and localization's characteristic failure is a stack that keeps
publishing a confident pose that is wrong.

AutoSDV ships nineteen scripts in `scripts/testing/localization/`, plus two more
under its `mapcheck/` subdirectory, that answer the questions `ros2` cannot.
This page says what each one answers and which question it is for. For what to
do with the answers, see [Tuning NDT](./ndt-tuning.md).

## Running them

Every script needs the ROS environment and the workspace on the path:

```bash
source scripts/env.sh
python3 scripts/testing/localization/check_ndt_activated.py
```

The live tools subscribe to a running stack, so start the stack first and run
them in a second or third terminal. The offline tools read a recorded run
directory and need nothing running.

!!! note "Not run while this page was written"

    The scripts' behaviour below is read from their sources, not from a live
    session. The file names, the options and the topics are verified against the
    tree.

A few of the numerical scripts re-exec themselves with `PYTHONNOUSERSITE=1`
before importing numpy. That is deliberate: a `~/.local` numpy shadows the apt
numpy that the apt scipy and matplotlib were built against, and the resulting
import error reads like a scipy bug rather than a shadowing problem. Nothing is
required of you; it is worth knowing when you see the process replace itself.

## Start here: is the matcher activated?

```bash
python3 scripts/testing/localization/check_ndt_activated.py --timeout 20
```

Exit 0 when `ndt_scan_matcher` is ACTIVATED, 1 when it is not, and it prints why
when it can — so a harness can gate on it.

This is the first check because of how the failure hides. `is_activated_` is
written *only* by the trigger service; the node never activates itself. If
initialization throws before that service is reached — a missing `map` to
`pose.frame_id` TF, a map that has not loaded, or no scan accepted yet — NDT is
latched off and stays off. Nothing retries it. The EKF then dead-reckons on IMU
and wheel speed, `/localization/kinematic_state` keeps running at 40 Hz, and the
vehicle keeps moving across the map. Nothing in that picture announces a failure.

The "no scan accepted yet" case has its own trap: the scan callback returns
before registering the cloud when the sensor-to-`base_link` transform fails, or
when the scan's maximum point distance is under `sensor_points.required_distance`
(10 m). A short-range or untransformable scan silently starves initialization.

## While a replay or a drive is running

| Script | The question it answers |
|---|---|
| `check_ndt_activated.py` | Is the matcher activated, not merely alive? |
| `ndt_quality_report.py` | Is the pose good — scatter, yaw step, init-to-result, execution time? |
| `ndt_alignment_report.py` | Does the scan actually sit on the map, point by point? |
| `ndt_timeseries.py` | Which signal moved first, when localization walked into a wall? |
| `check_imu_velocity.py` | Are the two signals the prior is built from trustworthy? |
| `monitor_localization.py` | How far apart are the GNSS pose and the NDT pose right now? |
| `monitor_gps.py` | Does GNSS have a usable fix, and where does it put us on the map? |
| `check_map_bounds.py` | Are we inside the mapped area at all? |
| `log_localization_data.py` | Record GNSS and localization to CSV for later. |
| `launch_monitors.sh` | All three monitors at once, in a tmux session. |

### `ndt_quality_report.py` — pose quality, deliberately not NVTL

```bash
python3 scripts/testing/localization/ndt_quality_report.py --seconds 120 --label res2.0
```

Records for a window and prints metrics NDT does *not* gate on:

- **scatter** — per-frame deviation from a locally smoothed path. A pose that
  jitters against a static map is wrong even when it scores well.
- **yaw step** — frame-to-frame heading change. Large steps are the flip and
  slip failures a mean score averages away.
- **init to result** — how far NDT moves the prior each frame. Small and steady
  means the prior is good; large or growing means the prior is stale, which is a
  fusion problem, not a parameter to tune.
- **execution time** — so a configuration that is accurate but too slow for the
  Orin is visible as such.

It omits NVTL on purpose. NVTL rises with `ndt.resolution` and rises again when
imperfect far returns are cropped away, so maximising it selects for coarse
voxels and narrow crop boxes whether or not the pose improves. This project's
own tuning study had both of its headline conclusions reverse when it was
re-measured against pose quality instead. See
[Tuning NDT](./ndt-tuning.md#nvtl-gates-convergence-it-does-not-rank-quality).

### `ndt_alignment_report.py` — the residual, independent of NDT

```bash
python3 scripts/testing/localization/ndt_alignment_report.py --seconds 60
```

Takes each incoming scan, transforms it into the map frame using the pose the
stack is currently publishing, and measures the distance from every scan point
to the nearest map point. That residual is what "is it aligned" actually means,
and unlike NVTL it is not produced by the estimator whose pose is in question.

NVTL cannot distinguish "the scan is on the map" from "the scan is confidently
on the wrong part of the map". A nearest-neighbour residual can.

Results are reported separately for stationary and moving frames, because they
answer different questions: the stationary ones say whether initial convergence
succeeded, the moving ones whether tracking is holding.

Options: `--map` (defaults to the COSS map), `--topic`, `--max-radius`,
`--label`.

### `ndt_timeseries.py` — which signal moved first

```bash
python3 scripts/testing/localization/ndt_timeseries.py --seconds 300 -o tmp/ndt
```

Writes `tmp/ndt.csv` and `tmp/ndt.png`. Built for one question: when
localization fails, *which* signal moved first? It plots

- `iteration_num` — hitting `max_iterations` means the optimiser ran out of
  budget before converging. This is the earliest honest warning available.
- NVTL and TP — as a trend against their own gate, never as an absolute quality.
- `initial_to_result_distance` — a jump means the prior and the scan disagreed,
  which is what a mis-tracked turn looks like from inside.
- `skipping_publish_num` — consecutive rejected results. NDT deactivates when
  this passes its limit, and the EKF then dead-reckons in silence.
- yaw rate and speed underneath, so a divergence can be lined up against the
  manoeuvre that caused it.

Everything is stamped on the simulated clock, so the x axis lines up with bag
time and with anything else recorded from the same replay.

### `check_imu_velocity.py` — auditing the prior's two inputs

```bash
python3 scripts/testing/localization/check_imu_velocity.py --seconds 200
```

The EKF prior is gyro odometry over IMU and `VelocityReport`. When
`initial_to_result_distance` is large while moving and small parked, the problem
is here, not in the matcher. This checks the three ways those signals go wrong:

- **Coordinates.** Is the IMU in REP-103 body axes (x forward, y left, z up)?
  Read off gravity while parked — a level REP-103 IMU reports az near +9.81, and
  a sign or axis swap shows up immediately. Yaw rate must also share a sign
  convention with the pose, or every turn is integrated backwards.
- **Scale.** Integrate each signal over a window where NDT is still tracking and
  compare against the pose the map produced. Reported as a ratio, so 1.00 is
  correct and 0.90 means the vehicle really moved 11 % more than the wheels said.
  A few percent is invisible per scan and is exactly what drags a prior a metre
  behind over a turn.
- **Quality.** Gyro bias and noise while stationary, publication rate, and gaps.
  A rate that sags makes the EKF extrapolate.

Comparisons use the NDT pose while it is still converged, not the EKF output —
the EKF is downstream of the very signals under test, so scoring them against it
would hide a shared error. `--raw-topic` follows `imu_source`.

### The GNSS monitors

```bash
python3 scripts/testing/localization/monitor_gps.py           # fix quality, map-frame coordinates
python3 scripts/testing/localization/monitor_localization.py  # GNSS pose vs NDT pose vs fused state
python3 scripts/testing/localization/check_map_bounds.py      # are we inside the map?
./scripts/testing/localization/launch_monitors.sh             # all three, in tmux
```

`monitor_gps.py` shows raw latitude, longitude and altitude, reported accuracy,
and the converted map-frame coordinates. `monitor_localization.py` shows the 2-D
distance, height difference and heading difference between GNSS and NDT.
`check_map_bounds.py` prints the point cloud map's extent and how far the
current position is from its edges.

Establish GNSS quality *before* using GNSS as a reference for anything.
Consumer GNSS without RTK is a rough initialization seed and nothing more.

`log_localization_data.py` records the same signals to CSV for later analysis.

## Seeding a run without a human

Tuning means running the same recording many times and comparing. A hand-placed
initial pose makes every run start somewhere slightly different, and that
difference shows up in the results looking like a parameter difference.

```bash
# once: place the pose in RViz, let NDT settle, then capture it
python3 scripts/testing/localization/capture_initial_pose.py COSS

# every run after that
python3 scripts/testing/localization/set_initial_pose.py COSS
```

The pose is written to `data/initial_poses/<name>.yaml`, taken from
`/localization/kinematic_state` — after NDT converged and the EKF settled, not
from the raw click. The file records *which* source it came from, because the
two look identical once saved and are worth very different things: under the
planning simulator the pose is the click unchanged, reported back as ground
truth; under a replay it is NDT agreeing with the map. Both work as a seed, only
the second is a measurement.

`set_initial_pose.py` calls `/localization/initialize`, the same service RViz's
2D Pose Estimate reaches through the ADAPI adaptor. Publishing `/initialpose3d`
directly does **not** initialize Autoware — `pose_initializer` publishes that
topic rather than listening to it. The method is AUTO rather than DIRECT, so the
captured pose is handed to the estimator as a starting guess and NDT refines it
against the map, which is the difference between a seed and an assertion.

Options: `--settle` and `--max-drift` on the capture, `--timeout` on the replay.

## Recording a run, and reading it afterwards

```bash
scripts/testing/localization/run-ndt-replay.sh <label>
POSE_SOURCE=ndt scripts/testing/localization/run-ndt-replay.sh builtin
```

Replays the COSS recording through a pose estimator and records everything
needed to tell an initialization failure from a tracking failure: every NDT
diagnostic topic into a rosbag, the full launch output, and `/diagnostics`,
GNSS, EKF and kinematic state for cross-checking. Output lands in
`tmp/ndt-replay/<label>_<timestamp>/`.

The recording is parked for its first 115.7 s and drives for the last 41.3 s, so
one run covers both phases. Per-node output is in `play_log/latest/node/<name>/`
as usual — read the *other* nodes' logs, not just the matcher's.

`just demo run` wraps the same idea with the data fetch, the pose seed and a
metrics report, and writes its runs to `tmp/demo-runs/`. `just demo report`,
`just demo compare` and `just demo list-runs` operate on those.

| Script | The question it answers |
|---|---|
| `summarize_ndt_run.py` | One run's diagnostics, split into initialization and tracking phase. |
| `compare_ndt_runs.py` | Several runs side by side: scatter, yaw jitter, prediction error, score. |
| `ndt_yaw_bias.py` | Is there a constant heading offset — that is, a mounting calibration error? |
| `ndt_benchmark_report.py` | Did two matchers do the same work, and which was faster and cheaper? |
| `export_ndt_frames.py` | Dump the exact (scan, prior) pairs, for an offline matcher comparison. |
| `tegrastats_summary.py` | GPU busy, rail power and CPU load, on a Jetson. |
| `mapcheck/map_coverage.py` | Does the map even extend as far as the crop box? |
| `mapcheck/map_agreement.py` | Where it does cover, does it agree — and is the disagreement the map or the foliage? |

### `summarize_ndt_run.py` and `compare_ndt_runs.py`

```bash
python3 scripts/testing/localization/summarize_ndt_run.py <run_dir>
python3 scripts/testing/localization/compare_ndt_runs.py a=<run_dir> b=<run_dir>
python3 scripts/testing/localization/compare_ndt_runs.py --row a=<run_dir>   # one TSV line
```

The init-versus-track split is the most informative thing in the summary. While
parked, matching is easy, because each scan resembles the last; once the vehicle
moves, the same work costs roughly twice as much. A marginal machine fails
there, not at startup.

Reading a run means scanning a multi-gigabyte rosbag, so several runs are worth
extracting concurrently — which is what `--row` is for, and what `just demo
compare` fans out.

### `ndt_yaw_bias.py` — heading minus course over ground

```bash
python3 scripts/testing/localization/ndt_yaw_bias.py label=<run_bag_dir>
```

A vehicle driving straight has heading equal to course. A constant difference is
a yaw offset between `base_link` and the sensor NDT actually localizes — a
mounting calibration error, not a localization bug. It survives every score
check, because the match itself is good.

Turns are excluded, and that exclusion is the point: a finite chord
systematically lags the instantaneous heading through a curve, which
manufactures a bias that is not there. On the COSS recording, restricting to
straight segments moved the estimate from a confounded -11.5 degrees to a clean
-12.66.

### `ndt_benchmark_report.py` and `export_ndt_frames.py`

A live replay cannot fairly compare two matchers. The slower one misses the scan
budget, drops scans, and every dropped scan leaves a staler prior, which scores
worse, which fails the convergence gate — so what gets measured is a collapse
rather than a speed ratio.

`export_ndt_frames.py` writes the exact (scan, initial guess) pairs the stack
used, with the map, into one flat file. Feeding that identical sequence to each
arm offline makes "the same work" structural rather than something to verify
afterwards.

```bash
python3 scripts/testing/localization/export_ndt_frames.py --run tmp/demo-runs/<run> --out tmp/ndt-frames.bin
python3 scripts/testing/localization/ndt_benchmark_report.py gpu=<dir> cpu=<dir>
```

`ndt_benchmark_report.py` reports speed, cost and **equivalence** — iterations
and score — because a speed comparison is meaningless if the two configurations
did not converge to the same place.

### On a Jetson

```bash
python3 scripts/testing/localization/tegrastats_summary.py <run_dir>/tegrastats.log --skip-seconds 60
```

`play_launch`'s per-node GPU utilisation and power are `nan` on Jetson: they come
from NVML, which Tegra does not implement. `tegrastats` is the only source, and
on Orin power is a first-class number — the case for moving NDT to the GPU is
that it frees CPU inside a fixed power envelope, which cannot be checked without
the rails. `--skip-seconds` excludes map loading and TensorRT warm-up from the
steady-state figure.

### Checking the map itself

```bash
python3 scripts/testing/localization/mapcheck/map_coverage.py --run <run_dir>
python3 scripts/testing/localization/mapcheck/map_agreement.py --run <run_dir>
```

`map_coverage.py` is the first question to ask before widening the
measurement-range crop box: does the map extend that far at all? A scan point
beyond the mapped area cannot constrain the pose, and because the downsample
filter keeps a fixed number of points regardless of range, unmapped far returns
*dilute* the ones that carry the constraint. It is cheap — a footprint test on a
2 m grid, no nearest-neighbour search.

`map_agreement.py` answers the follow-up from a nearest-neighbour pass over
several observer poses:

- **Ground or vegetation?** Ground cannot move between mapping and recording, so
  a displaced ground return means the map is wrong there. Foliage moves with the
  wind and the season and disagrees at every range.
- **Warped map or moving foliage?** If several vehicle poses see the same map
  cell displaced the same way, the displacement belongs to the map — SLAM drift
  while mapping. If each observer sees something different, it is noise.

## Which script for which symptom

| Symptom | Start with |
|---|---|
| No pose on `/localization/pose_estimator/pose_with_covariance` at all | `check_ndt_activated.py` |
| The pose exists but the cloud slides against the map | `ndt_alignment_report.py` |
| The pose jitters while the vehicle is parked | `ndt_quality_report.py` |
| It tracked, then diverged partway through | `ndt_timeseries.py` |
| `initial_to_result_distance` is large while moving, small parked | `check_imu_velocity.py` |
| Heading is off by a constant on straight roads | `ndt_yaw_bias.py` |
| Localization degrades away from the map's centre | `mapcheck/map_coverage.py`, then `map_agreement.py` |
| It works here and not on the vehicle | `tegrastats_summary.py`, then `ndt_benchmark_report.py` |
| Successive runs disagree for no reason you changed | `capture_initial_pose.py` / `set_initial_pose.py` |

## Related

- [Tuning NDT](./ndt-tuning.md) — what to do with the answers
- [Localization Methods](./localization-methods.md) — `ndt`, `cuda_ndt`, `mcl`
- [Maps](./maps.md) — validating a map before blaming the matcher
- [Inspecting a Running System](../concepts/inspecting.md) — the generic `ros2`
  tools these sit on top of
- The scripts' own reference is
  [`scripts/testing/localization/README.md`](https://github.com/NEWSLabNTU/AutoSDV/blob/main/scripts/testing/localization/README.md)
  in the repository

<!--
RECONCILE (phase 2):
- nav: "Localization Diagnostics" under Building the Stack > 4.2 Localization,
  last in that group (after Maps); nav_translations entry "定位診斷".
- concepts/inspecting.md: add a pointer at the end — "these are the generic
  tools; localization has purpose-built ones" -> link here. That page currently
  teaches only generic ros2 commands.
- reference/: roadmap 13 also allocates a "Diagnostic Scripts" reference page.
  This page is the guide (what each answers, which to reach for); the reference
  page should be the flat table of every script with its options, and should
  link here rather than restate the reasoning. If phase 2 decides one page is
  enough, this one is the keeper and the reference entry becomes a link.
- glossary terms: NVTL, TP, prior, dead reckoning, ADAPI, REP-103.
- cross-link FROM tutorial/03-logging-simulation.md, which already names
  check_ndt_activated.py, ndt_quality_report.py and ndt_alignment_report.py
  without saying what they report — link here from that paragraph.
- cross-link FROM guides/localization-methods.md §mcl: the scan normaliser
  diagnostics described there are MCL's equivalent of this page; a "see also"
  in both directions.
- W6 owns the command reference: `just demo run/report/compare/list-runs` and
  `just coss logging-sim/play-rosbag` are named here and should exist there.
-->
