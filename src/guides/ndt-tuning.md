# Tuning NDT

NDT is the scan matcher behind `pose_source:=ndt` and `pose_source:=cuda_ndt`.
It takes a pose prior, a LiDAR scan and a point cloud map, and returns a
corrected pose. This page is about what to change when that pose is wrong, in
what order to look, and — more often — how to find out that nothing in NDT
needs changing at all.

## NDT is usually the last thing wrong

The matcher sits at the end of a chain: TF, IMU, twist, EKF, point cloud
preprocessing, map. It fails *loudly* when something earlier in the chain fails
*quietly*, which is why it collects the blame.

The failure that prompted this page looked exactly like a tuning problem. NDT
scored badly, rejected most frames and eventually stopped publishing. The cause
was a missing TF: nothing published the IMU transform in logging simulation, so
the EKF never propagated and NDT was handed the same stale prior on every scan.
Every parameter a person reaches for first — resolution, the convergence
threshold, the crop box — would have been tuned to compensate for that, and each
would have made the stack worse once the TF was fixed.

So the rule is: **prove the chain, then tune the matcher.** The sections below
are in that order deliberately.

## Get a repeatable replay first

A parameter study is worthless if the runs are not comparable. Fix the
recording, the map, the initial pose and the set of nodes, then change exactly
one thing per run.

The COSS recording is the fixture this project tunes against. It needs two
terminals, which is the point: the stack runs on simulated time and does
nothing until a bag drives its clock.

```bash
# terminal 1 — the stack, waiting on a clock
play_launch launch --web-addr 0.0.0.0:8081 \
  autosdv_launch logging_simulation.launch.yaml \
  map_path:="$PWD/data/COSS-map-planning" \
  pose_source:=ndt launch_perception:=false
```

```bash
# terminal 2 — the recording, driving that clock
ros2 bag play data/rosbags/outdoor_20251226_153115 --clock
```

Shortcuts for the same two commands:

```bash
just coss logging-sim        # cpu path (pose_source:=ndt); `gpu` for cuda_ndt
just coss play-rosbag        # --clock is not optional, and the recipe knows it
```

If the recording is not there yet, `just coss download-rosbag` fetches it
(about 1.6 GB downloaded, 2.8 GB unpacked).

Three things make successive runs comparable, and all three are easy to skip:

- **Seed the initial pose deliberately.** Do not let GNSS do it. On this
  recording, GNSS auto-init lands about 5 m apart from run to run, and that
  scatter shows up in the results looking like a parameter difference. Capture a
  pose once and replay it — see
  [Localization Diagnostics](./localization-diagnostics.md#seeding-a-run-without-a-human).
- **Turn off what you are not studying.** `launch_perception:=false` removes the
  TensorRT engine build and the GPU contention along with it.
- **Build in release.** `-DCMAKE_BUILD_TYPE=Release` does not reach Rust
  packages, and `cuda_ndt_matcher` is Rust. `just build` passes `--cargo-args
  --release` for exactly this reason; a debug matcher runs roughly an order of
  magnitude slower and drops scans, which looks like a tuning problem and is not.

!!! note "These commands need a built workspace and the recording"

    They were not run while this page was written. The parameter names, defaults
    and file paths below *were* read out of the tree.

## Which file you are editing

The two pose sources do not share a parameter file, and the values differ.
Editing the wrong one is the most common way to change nothing at all.

| `pose_source` | Parameter file |
|---|---|
| `ndt` | `src/launcher/autosdv_launch/config/localization/ndt_scan_matcher/ndt_scan_matcher.param.yaml` |
| `cuda_ndt` | `src/localization/cuda_ndt_matcher/src/cuda_ndt_matcher_launch/config/cuda_scan_matcher.param.yaml` |

The input preprocessing chain — crop box, voxel grid, random downsample — is
split the same way, under a `pointcloud_preprocessor/` directory beside each of
those files.

With `--symlink-install`, editing one of these YAML files takes effect on the
next launch with no rebuild. That also means the file you edited *is* the file
that loads, which is worth confirming rather than assuming, because several
files with the same basename exist in this repository:

```bash
ros2 param get /localization/pose_estimator/ndt_scan_matcher ndt.resolution
ros2 param get /localization/util/crop_box_filter_measurement_range max_x
```

Ask the running node. It is the only answer that is about the stack you are
looking at.

## What to look at, and in what order

### 1. Is the matcher activated?

Not "is the process alive" — activated. `ndt_scan_matcher` has an
`is_activated_` latch that only the trigger service sets, and if initialization
throws before that service is reached, NDT stays off permanently. Nothing
retries it. Meanwhile the EKF keeps dead-reckoning on IMU and wheel speed, so
`/localization/kinematic_state` runs at 40 Hz and the vehicle still moves across
the map. Nothing in that picture says "broken".

```bash
python3 scripts/testing/localization/check_ndt_activated.py
```

Exit 0 means activated. If it is not, stop here: no parameter changes anything.

### 2. Is the prior chain alive?

NDT does not estimate a pose from nothing — it refines the prior the EKF hands
it. A stale prior degrades the match no matter what the matcher does.

```bash
ros2 topic hz /sensing/imu/imu_data
ros2 topic hz /localization/twist_estimator/twist_with_covariance
ros2 topic hz /localization/kinematic_state
ros2 run tf2_ros tf2_echo base_link <sensor_frame>
```

A zero-rate topic here is the bug. So is a TF that does not resolve.

Read the *other* nodes' logs, not just the matcher's — `play_log/latest/node/*/err`.
The COSS failure announced itself as thousands of copies of
`Please publish TF base_link to zedxm_imu_link` from `imu_corrector`, a node
nobody was watching, while `ndt_scan_matcher` merely reported low scores.

The two signals the prior is actually built from have their own check:

```bash
python3 scripts/testing/localization/check_imu_velocity.py
```

It tests them for the three ways they go wrong: wrong axes or signs, a scale
error, and gaps or a sagging publication rate.

### 3. Is the pose good, independently of NDT's opinion?

NDT's own scores cannot tell you whether NDT is right. Use references it does
not depend on:

| Question | Independent reference |
|---|---|
| Is yaw *rotating* correctly? | integrate the gyro over the same window |
| Is yaw *pointing* correctly? | course over ground, on straight segments only |
| Is the distance right? | wheel odometry, and the map's own scale |
| Does the pose fit the world? | scan-to-map nearest-neighbour distance |
| Is the map trustworthy far out? | the same map cell seen from several poses |

Two of those are scripted:

```bash
python3 scripts/testing/localization/ndt_quality_report.py    # scatter, yaw step, init-to-result
python3 scripts/testing/localization/ndt_alignment_report.py  # scan-to-map residual
```

### 4. Only now, the parameters

Change one variable per run. When two candidate causes are confounded, a 2x2
costs two extra runs and settles it — on this map, crossing resolution against
crop range reversed the conclusion that either one alone supported.

## What the numbers mean

| Number | What it is | Healthy |
|---|---|---|
| published poses | frames that passed the convergence gate | one per scan, so ~10 Hz |
| worst publish gap | the longest silence from the pose estimator | around 0.1 s |
| `initial_to_result_distance` | how far NDT had to move the prior | near zero parked; small and *steady* moving |
| position scatter | per-frame deviation from a locally smoothed path | a pose that jitters against a static map is wrong even when it scores well |
| yaw step | frame-to-frame heading change | no isolated large steps |
| `iteration_num` | optimiser steps per scan | low single digits; pinned at the cap means no pose at all |
| `exe_time_ms` | one scan match | comfortably inside the 100 ms scan period |
| NVTL | mean per-point fit score | above its gate with margin — and nothing more (see below) |

`initial_to_result_distance` is the most informative single number, and it must
be split by motion. Near zero parked and much larger while moving means the
problem is in the prediction path — twist scale, IMU, or an extrinsic — not in
the matcher. On the COSS recording that split ran 0.819 m moving against
0.024 m parked; correcting the wheel-speed scale and the LiDAR mounting yaw took
the moving figure to 0.049 m without touching a single NDT parameter.

## NVTL gates convergence; it does not rank quality

`nearest_voxel_transformation_likelihood` is compared against
`converged_param_nearest_voxel_transformation_likelihood` to decide whether to
publish a frame. It is a *gate*. It is not an accuracy metric, and tuning to
maximise it inverts the answer:

- **It scales with `ndt.resolution`.** Coarser voxels raise NVTL while lowering
  accuracy. On this map, resolution 4.0 scored substantially higher than 2.0 and
  had five times the per-frame position scatter.
- **Honest but imperfect far returns lower the mean while improving the pose.**
  Widening the crop box dropped NVTL and halved the yaw error.

So maximising NVTL selects for coarse voxels and narrow crop boxes whether or
not the pose gets better. Rank on position scatter, yaw step,
`initial_to_result_distance` and publish continuity instead.

Two further things about it:

**A low NVTL means a bad prior at least as often as a bad matcher.** If it sits
near its threshold, look upstream before touching the matcher — and note that
the failure is self-reinforcing. Rejected frames stop publishing, the prior gets
staler, the score falls further. A cliff in the published-pose rate is the
signature.

**An absolute score is only comparable within one build.** NVTL is not a
physical quantity: it depends on the resolution, on the point count, and on the
scoring code being right. Two defects in this project's CUDA matcher inflated it
by about 1.45x until they were fixed in August 2026, so any threshold recorded
before then is calibrated against a different scale. **Re-derive a threshold
from the distribution of the build you are running** — never carry one across a
version, a map, a resolution or a downsample count.

## The parameters, and when to touch them

| Parameter | Reach for it when | Do not use it to |
|---|---|---|
| `ndt.resolution` | the map's point density genuinely changed | buy NVTL margin |
| `converged_param_nearest_voxel_transformation_likelihood` | you have re-measured the score distribution of *this* build | silence a symptom of a bad prior |
| crop box `min/max_x`, `min/max_y` | the map's far field is measured good and yaw is under-constrained | compensate for a mis-scaled twist |
| `random_downsample_filter.sample_num` | `exe_time_ms` has headroom, or does not | fix an accuracy problem |
| `ndt.max_iterations` | `exe_time_ms` fits the scan period with room to spare | fix non-convergence caused by a stale prior |
| `initial_pose_estimation.particles_num` | Monte Carlo initialization is unreliable | improve tracking |

### Defaults in the tree

Read from the two parameter files, 2026-09-21. Where they differ, they differ on
purpose, and the CUDA file carries the measurements in comments beside each
value.

| Parameter | `ndt` | `cuda_ndt` |
|---|---|---|
| `ndt.resolution` | 4.0 | 2.0 |
| `ndt.max_iterations` | 30 | 30 |
| `ndt.trans_epsilon` | 0.01 | 0.01 |
| `ndt.step_size` | 0.1 | 0.1 |
| `ndt.num_threads` | 4 | 4 |
| `converged_param_nearest_voxel_transformation_likelihood` | 2.2 | 2.0 |
| `validation.skipping_publish_num` | 5 | 5 |
| `validation.critical_upper_bound_exe_time_ms` | 100.0 | 100.0 |
| `sensor_points.required_distance` | 10.0 | 10.0 |
| `initial_pose_estimation.particles_num` | 200 | 200 |
| crop box `min/max_x`, `min/max_y` | ±40.0 | ±60.0 |
| voxel grid `voxel_size_*` | 0.5 | 0.5 |
| `random_downsample_filter.sample_num` | 2000 | 5000 |

!!! warning "The two resolutions disagree, and the measurement favours 2.0"

    The repository's own measurement on the COSS map says resolution 2.0 beats
    4.0 on every pose-quality metric while scoring *lower* NVTL. `cuda_ndt` ships
    2.0. The built-in `ndt` path still ships 4.0, and its crop box comment
    records measurements taken at 2.0. If you are tuning on the `ndt` path, that
    is the first thing to try, and the difference is not a deliberate CPU-versus-
    GPU trade as far as anything in the tree says.

`cuda_ndt` has four parameters the CPU path does not:

- `ndt.use_line_search` (`true`) — batched parallel line search, which can cut
  iterations by about a quarter.
- `score_estimation.compute_before_scores` (`false`) — scores the scan at the
  initial pose as well, to fill two diagnostic fields nothing else consumes. On
  an AGX Orin the pair costs about 28 ms against a 32 ms alignment. Turn it on
  while tuning to see how far NDT moved the pose, and off again afterwards.
  While off, both fields report NaN rather than 0.0, so a disabled score cannot
  be misread as a genuinely zero one.
- `initial_pose_estimation.yaw_weight_sigma` (`30.0`, degrees) — how strongly
  initialization is biased toward the initial heading.
- `batch.*` (`enabled: false`) — queue several scans for GPU-parallel alignment.

### `max_iterations`: the cap that published nothing

Worth keeping because the failure looks like success. AutoSDV once shipped
`max_iterations: 15` against Autoware's 30, and `pose_source:=ndt` published no
pose at all on the COSS recording.

Autoware's matcher treats a frame that reaches the cap as **not converged** and
discards the result, so nothing reaches
`/localization/pose_estimator/pose_with_covariance`:

```
The number of iterations has reached its upper limit.
The number of iterations: 15, Limit: 15.
```

The stack still *looks* localized, because the EKF keeps publishing
`/localization/kinematic_state` from wheel odometry and IMU. The telling pair is
that topic carrying thousands of messages while the pose estimator's carries
zero.

Restoring 30 was both correct and *faster*. The matcher converges in about four
iterations and needs more only on the first few frames, so the low cap kept it
grinding through all 15 forever instead of accepting the one frame that would
have made the rest easy. Both values are now 30; the measured comparison is in
[the repository's tuning guide](https://github.com/NEWSLabNTU/AutoSDV/blob/main/docs/guides/ndt-tuning.md#incident-a-max_iterations-cap-that-published-nothing).

## What is not a tuning knob

These are measurements. Get them right rather than searching over them, because
a parameter tuned to compensate for one of them is a parameter that will be
wrong everywhere else:

- sensor extrinsics, in `sensor_kit_calibration.yaml`
- wheel diameter and encoder counts
- the IMU frame and its axis convention
- the map itself

## Traps that look like a tuning problem

**An uncalibrated mounting rotation.** If the LiDAR is yawed relative to
`base_link` and the calibration says 0, NDT places the *sensor* correctly and the
trajectory looks right, but the reported heading is off by the mounting error,
constantly. It survives every score check, because the match itself is good. The
symptoms are: heading differs from course over ground by a constant on straight
segments, yaw *changes* still agree with the gyro, and
`initial_to_result_distance` is elevated while moving but fine at rest. Check
`sensor_kit_calibration.yaml` early, and be suspicious of a file full of zeros.

**Frame-to-frame statistics hiding slow drift.** A yaw error that accumulates
30 degrees over 10 seconds is only 0.3 degrees per frame, and looks healthy in
any p95-of-step metric. Plot the absolute quantity against time, alongside an
independent reference.

**Course over ground through a turn.** Comparing heading with the direction of
travel is the way to catch a yaw offset, but a finite chord lags the
instantaneous heading through a curve and manufactures a bias that is not there.
Restrict the comparison to roughly |yaw rate| < 2 deg/s. On the COSS recording
that changed the estimate from a confounded -11.5 degrees to a clean -12.66.

**A shared GPU.** CUDA NDT measured 67-83 ms per scan through an entire
investigation, against the few milliseconds it should take. The matcher was
fine; another process held 26 GB of the 32 GB card. The tell is that *only* the
wall clock moves — iterations and NVTL were identical, because contention costs
time, not convergence. A timing regression with unchanged iteration counts
points outside the algorithm.

```bash
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv
```

**Consumer GNSS treated as ground truth.** Before trusting `/sensing/gnss/pose`,
check `nav_sat_fix.status` and whether RTCM is flowing. The COSS recording's fix
is `status: 0` with no NTRIP: about 20 m of horizontal scatter, altitude wrong by
tens of metres, and it disagrees with the actual direction of travel. Fine as a
rough initialization seed, useless as a reference.

**A recording bakes in sensor errors.** No parameter change can undo a
wheel-speed scale error that is already in
`/vehicle/status/velocity_status`. You can republish a rescaled copy to test the
hypothesis, but the fix is on the vehicle. Watch for dead signals too: COSS
recordings carry `steering_status` identically zero, because this vehicle has no
steering angle sensor.

**The map's far field is not as good as its near field.** Scan-to-map residual
grows with range, and past some distance the map simply ends — on the COSS map,
around 60 m, beyond which very few returns land on any mapped cell. Before
widening the crop box, measure whether the map deserves it:

```bash
python3 scripts/testing/localization/mapcheck/map_coverage.py --run <run_dir>
python3 scripts/testing/localization/mapcheck/map_agreement.py --run <run_dir>
```

To tell a warped map from moving vegetation: compare the residual for the same
map cell seen from several vehicle poses. Coherent across observers means the
map is displaced there; random means foliage.

## Checklist

Before tuning anything:

- [ ] The replay runs end to end and records diagnostics
- [ ] Initial pose seeded deliberately, identical across runs
- [ ] Release build — `just build`, not a hand-rolled colcon without `--cargo-args --release`
- [ ] `check_ndt_activated.py` exits 0
- [ ] IMU topic publishing, and `imu_corrector`'s log is clean
- [ ] Twist publishing, and EKF displacement matches reality over a drive
- [ ] Every sensor TF resolves from `base_link`
- [ ] Parameters read back from the running node, not from the file you edited
- [ ] GNSS quality established before using GNSS for anything
- [ ] GPU not shared with another process, if any timing is to be believed

Per run, record:

- [ ] published poses, aligned frames, and the worst publish gap
- [ ] `initial_to_result_distance`, split parked versus moving
- [ ] per-frame position scatter and yaw step
- [ ] NVTL as a gate check only, with its margin over the threshold
- [ ] iterations and `exe_time_ms` against the scan period

Validate before believing:

- [ ] yaw change against integrated gyro, bucketed over time
- [ ] heading against course over ground, on straight segments only
- [ ] travelled distance against wheel odometry and against the map
- [ ] one changed variable per run, or a full cross of the confounded pair

## Related

- [Localization Diagnostics](./localization-diagnostics.md) — every script named
  on this page, and the question each one answers
- [Localization Methods](./localization-methods.md) — choosing between `ndt`,
  `cuda_ndt` and `mcl`
- [Maps](./maps.md) — what each method needs, and how to build it
- [Logging Simulation](../tutorial/03-logging-simulation.md) — the replay this
  page tunes against, from the beginning
- The measured results behind this page live in the repository:
  [`docs/reports/cuda-ndt-coss-replay.md`](https://github.com/NEWSLabNTU/AutoSDV/blob/main/docs/reports/cuda-ndt-coss-replay.md)
  and
  [`docs/reports/localization-open-questions.md`](https://github.com/NEWSLabNTU/AutoSDV/blob/main/docs/reports/localization-open-questions.md)

<!--
RECONCILE (phase 2):
- nav: "NDT Tuning" under Building the Stack > 4.2 Localization, after
  "Localization Methods"; nav_translations entry "NDT 調校".
- cross-link FROM guides/localization-methods.md: in the `cuda_ndt` section and
  in the `ndt` section, link here for "what to change when the pose is wrong".
- cross-link FROM tutorial/03-logging-simulation.md ("Reading the result"
  section, which already covers NVTL/TP/exe_ms/iterations at tutorial depth):
  add a "going deeper" pointer to this page. Do NOT delete that section — it is
  the right depth for the tutorial; this page is the depth after it.
- glossary terms to add: NVTL (nearest voxel transformation likelihood), TP
  (transform probability), convergence gate, initial_to_result_distance, prior.
- DE-DUPLICATION: this page and tutorial/03 both explain the max_iterations
  cap. Keep both — tutorial 03 states the symptom, this page states the
  incident and the fix — but check the wording does not drift apart.
- OPEN ITEM for a maintainer, not for the book: the `ndt` path ships
  ndt.resolution 4.0 while cuda_ndt ships 2.0, and the repository's own
  measurement favours 2.0. The page flags it as a divergence rather than
  asserting which is right. If that is settled in the tree before phase 3, the
  warning block under "Defaults in the tree" should be updated or removed.
-->
