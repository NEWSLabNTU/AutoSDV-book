# Command Reference

Nearly every page in this book reaches for a `just` recipe at some point. This
page is the list: what the repository ships, what each recipe actually runs, and
which spellings have been superseded.

The recipes live in `justfile` at the repository root, in the module files under
`just/`, and in `demo/justfile`. `just` on its own prints all of them:

```bash
just              # every recipe, modules expanded
just --list       # the same list
just tool         # just one module's recipes
```

Throughout this page the underlying command comes first and the recipe second.
That is deliberate: the recipe is a way to avoid typing the command forty times,
not a replacement for knowing it. When something goes wrong, the command is what
you will be debugging.

## How the recipes are organised

Six verbs you use every day stay at the root. Everything entered deliberately is
grouped into a module.

| Module | Covers |
|--------|--------|
| `bag` | rosbag recording and playback |
| `control` | control-system test launches and canned trajectories |
| `coss` | the COSS Park scenario: its data, and its two simulations |
| `demo` | end-to-end scenarios, run reports, localization benchmarks |
| `map` | map validation and occupancy-grid construction |
| `sim` | the full COSS Park scenario, plus two deprecated spellings |
| `tool` | RViz, PlotJuggler, the drive TUI, manual control, ZED |

Both spellings work, and they mean the same thing:

```bash
just bag play
just bag::play
```

### Two mechanics worth knowing

These bite when you add a module of your own, and they explain why the justfiles
look the way they do.

**A module may not share a name with a recipe.** `mod launch` beside a `launch:`
recipe is not a shadowing warning — it is a hard error that kills the whole
justfile, so no recipe runs at all. That is one reason the daily verbs stay at
the root rather than moving into a module.

**`just <module>` runs that module's *first* recipe.** It does not list the
module; listing is what the first recipe happens to do. Every module file here
therefore opens with a private `default` that runs `just --list <module>` and
nothing else. Without it, `just bag` would start recording.

## Root recipes

| Recipe | Does |
|--------|------|
| `just build` | build every package in `src/` |
| `just test` | run the tests and print the results |
| `just clean` | delete `build/`, `install/` and `log/`, after a prompt |
| `just launch ARGS` | launch the full system, web UI on port 8081 |
| `just checkout` | initialise and update every git submodule |
| `just setup` | run the interactive setup program |
| `just setup-autoware-data` | mirror the Autoware model tree somewhere writable |
| `just engines` | fetch a matching TensorRT engine set if one exists, else build |
| `just build-engines` | compile the TensorRT engines ahead of the first launch |
| `just export-engines` | package this board's engines for a release |

### `just build`

```bash
source /opt/ros/humble/setup.bash
colcon build \
    --base-paths src \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --cargo-args --release
```

`just build` is that command, preceded by a check that the Python environment
colcon needs is intact.

If you build a single package by hand, carry all of these flags. `--base-paths
src` keeps colcon out of `data/` and `docker/`; `--symlink-install` is what makes
an edit to a YAML or launch file take effect without rebuilding.

`--cargo-args --release` is the one that is easy to drop and expensive to drop.
`CMAKE_BUILD_TYPE=Release` covers only the C++ packages; without the cargo flag,
colcon-cargo builds `cuda_ndt_matcher` unoptimised, and `pose_source:=cuda_ndt`
then runs at roughly 80 ms per scan instead of 5.

### `just test`

```bash
colcon test --base-paths src --return-code-on-test-failure
colcon test-result --verbose
```

The recipe runs both and exits with the test run's status, so it is usable in a
script.

### `just clean`

Removes `build/`, `install/` and `log/`. It asks for a literal `yes` first;
anything else leaves the workspace alone.

### `just launch`

```bash
play_launch launch --web-addr 0.0.0.0:8081 \
    autosdv_launch autosdv.launch.yaml [ARGS]
# and, when $DISPLAY is unset, also: rviz:=false
```

Arguments are passed as one quoted positional string:

```bash
just launch
just launch "pose_source:=ndt launch_perception:=false"
```

!!! warning "Do not write `ARGS=` in front of them"

    `just launch ARGS="pose_source:=ndt"` looks like it sets the recipe's
    parameter, but in `just` a `NAME=value` *after* a recipe name is a
    positional value, not an assignment. The literal text `ARGS=pose_source:=ndt`
    would be handed to `play_launch`, which cannot use it, and the launch would
    run with its defaults — silently. The recipe now detects this spelling and
    refuses with exit code 2 rather than launching the wrong thing.

The full argument list is in [Operating the
Vehicle](../running/on-the-vehicle.md#the-arguments), and
[`play_launch`](../tutorial/06-play-launch.md) explains the launcher itself.

### `just checkout`

```bash
git submodule update --init --recursive --checkout
```

`--recursive` matters: several submodules contain submodules of their own.

### `just setup` and `just setup-autoware-data`

`just setup` runs `./setup.sh`, the interactive installer. See
[Software Installation](../getting-started/installation/overview.md).

`just setup-autoware-data` runs `./scripts/setup_autoware_data.sh`, which mirrors
Autoware's model tree into `data/autoware_data` as symlinks. Autoware writes each
compiled `.engine` next to the `.onnx` it was built from, and the packaged tree
under `/opt/autoware/` is root-owned — so without this the engine is discarded
the moment it is built and every launch recompiles it.

### TensorRT engines

The first launch of the perception stack compiles each ONNX model into a
TensorRT engine **inside the node's constructor**, which on an Orin is minutes
per model with perception unavailable throughout. These three recipes turn that
into a provisioning step.

```bash
just engines          # try the published cache for this board, then build
just build-engines    # build locally, always
just export-engines   # package what this board built, for a release
```

`just build-engines` compiles the five models the perception presets resolve to
(CenterPoint tiny, the YOLOX camera detector, and the three traffic-light models
that `camera_lidar_fusion` adds), then prints what landed.

Engines are specific to **the GPU and the TensorRT version**, so this has to run
on the target board and has to be re-run after an Autoware or JetPack upgrade.
They cannot be baked into an image built somewhere else.

On an amd64 workstation there is a trap worth knowing about: Autoware discards
any engine whose recorded TensorRT version differs from the one its own libraries
were compiled against, so a host loading a different patch of TensorRT rebuilds
everything on every launch no matter how often you run this recipe.
`just build-engines` compares the two versions and says so at the end rather than
letting you discover it later.

`just engines` is what the setup program runs. It looks for a published engine
set matching this board's fingerprint, downloads and verifies it if one exists,
and falls through to a local build if not — so it is safe to run on any machine,
and safe to interrupt.

## `bag` — recording and playback

```bash
just bag record    # record the outdoor sensor topics to rosbags/
just bag play      # play the most recent outdoor recording
```

`record` runs `./scripts/rosbag/record_outdoor.sh`. `play` finds the newest
`rosbags/outdoor_*` directory and runs:

```bash
ros2 bag play <that directory> --clock
```

`--clock` is not decorative. The stack runs on simulated time during a replay,
and without a clock source nothing advances — with no error anywhere.

`just bag download` is **deprecated**; it prints a pointer and forwards to
`just coss download-rosbag`, which names the recording it actually fetches.

## `control` — control-system testing

```bash
just control basic      # launch the vehicle control test
just control straight   # drive a 10 m straight trajectory
just control circle     # drive a circular trajectory
```

Underneath:

```bash
play_launch launch control_test basic_control.launch.xml
ros2 run control_test trajectory_player --ros-args -p trajectory_file:=straight_10m.yaml
ros2 run control_test trajectory_player --ros-args -p trajectory_file:=circle.yaml
```

`basic` brings up the control chain; the two trajectory recipes feed it a canned
path. They need the vehicle, or at least the vehicle interface, running. See
[Tuning and Testing](../guides/vehicle-control/tuning-and-testing.md).

## `coss` — the COSS Park scenario

The scenario the tutorial is built on: one map, one recording, two simulations.

| Recipe | Does |
|--------|------|
| `just coss download-rosbag` | fetch the drive recording (~1.6 GB down, 2.8 GB unpacked) |
| `just coss planning-sim` | the planning component alone |
| `just coss logging-sim [backend]` | the full stack, fed from the recording |
| `just coss play-rosbag [rate]` | play the recording into a running `logging-sim` |
| `just coss demo` | forwards to `just demo run` |

Each of the launch recipes echoes the `play_launch` command it is about to run,
in dim text, before running it.

**The planning simulator** needs no sensors, no localization, no GPU and no
rosbag:

```bash
play_launch launch --web-addr 0.0.0.0:8081 \
    autoware_launch planning_simulator.launch.xml \
    map_path:="$PWD/data/COSS-map-planning" \
    vehicle_model:=autosdv_vehicle \
    sensor_model:=autosdv_sensor_kit
```

**The logging simulation takes two terminals**, and that is the lesson rather
than an inconvenience:

```bash
# terminal 1 — the stack, waiting on a clock
just coss logging-sim

# terminal 2 — the recording, driving that clock
just coss play-rosbag
```

Without the second, nothing happens and nothing reports why.

`logging-sim` takes a backend argument, defaulting to `cpu`:

| Backend | Adds |
|---------|------|
| `cpu` *(default)* | `pose_source:=ndt launch_perception:=false` — runs on any laptop |
| `gpu` | `pose_source:=cuda_ndt`, with perception |

The CPU default is chosen so a machine that has never built a TensorRT engine
does not spend its first half hour doing so. `play-rosbag` takes a playback rate,
defaulting to `1.0`, and refuses with a pointer to `download-rosbag` if the
recording is not there.

See [The COSS Park Scenario](../running/coss-park-scenario.md) and
[Logging Simulation](../tutorial/03-logging-simulation.md).

## `demo` — scenarios that run end to end

Each demo owns its own data preparation, so a fresh machine needs nothing beyond
`just build` and the recipe.

**Before and after a run**

```bash
just demo check         # are the prerequisites present?
just demo fetch-data    # download the COSS rosbag if it is missing
just demo prepare       # fetch-data, then just build
just demo stop          # stop the stack a run left up
just demo list-runs     # recorded runs, newest first
just demo clean         # delete recorded runs (several GB each), after a prompt
```

`just demo check` verifies the rosbag, the map, that the workspace is built,
that `cuda_ndt_matcher` is a release build rather than a debug one, that
`play_launch` is on `PATH`, and that the CUDA toolkit knows this GPU's
architecture. It exits non-zero if anything is missing, so it works as a gate in
a script.

**Running**

```bash
just demo run               # the COSS NDT replay, end to end
just demo run-headless      # the same, no RViz, stack not left up
just demo run-manual-init   # no pose seed; set it yourself in RViz
just demo run-raw-speed     # with the uncorrected wheel speed
just demo exhibition        # live perception, no map and no localization
```

`run` fetches the data if needed, launches the stack, seeds an initial pose,
replays the recording, writes a run directory under `tmp/demo-runs/`, prints a
summary, and leaves the stack up for inspection — which is why `just demo stop`
exists. Each takes an optional label that names the run directory.

`exhibition` is the odd one out: it runs the perception stack against live
sensors in mapless mode, with no localization and no map, for demonstrations
where visitors walk past the vehicle and watch their own detection boxes appear.
It takes a model argument (`centerpoint` by default, `centerpoint_tiny` if RViz
and the detector are contending for the GPU). It needs the vehicle's sensors.

**Reading a run**

```bash
just demo report [run_dir]        # metrics (default: the most recent run)
just demo yaw-bias [run_dir]      # heading-versus-course yaw bias
just demo map-quality [run_dir]   # does the map cover the scan, and agree with it?
just demo compare a=<dir> b=<dir> # runs side by side
```

**Benchmarks**

```bash
just demo bench [configs] [repeats]   # cuda_ndt on GPU, the same code on CPU, Autoware's
just demo bench-offline [run_dir] [frames]  # GPU versus CPU on identical recorded input
just demo bench-nvtl-probe [frames]   # scoring parity between the arms
just demo bench-report <runs.tsv>     # rebuild a report from runs already recorded
```

`bench-offline` is the comparison to trust: the two arms are fed byte-identical
recorded input rather than two live runs that saw different scans.

## `map` — validation and grid construction

```bash
just map check MAP_DIR [POSE_SOURCE] [FLAGS...]
just map grid-from-pcd MAP_DIR [FLAGS...]
just map grid-from-bag BAG MAP_DIR [FLAGS...]
```

Underneath these are three Python tools:

```bash
python3 ./scripts/map/check_map.py <map_dir> --pose-source cuda_ndt
python3 ./scripts/map/pcd_to_pgm.py <map_dir>/pointcloud_map.pcd <map_dir>/occupancy_grid --sidecar
python3 ./scripts/2dlidar/scan_accumulate_grid.py <bag> <map_dir>/occupancy_grid --sidecar
```

`check` defaults to `cuda_ndt`. It verifies the lanelet2 map, the projector
info, the PCD, the occupancy grid — and, the point of the whole exercise, that
the grid is in the same frame as the lanelet2 map. That is the failure class a
grid built in the wrong frame produces: a valid-looking map that localizes
badly, rather than an error.

Both `grid-from-*` recipes write `occupancy_grid.pgm` and `occupancy_grid.yaml`,
record how the grid was built in `autosdv_map.yaml`, and then run
`just map check <map_dir> mcl` on the result.

`grid-from-pcd` run without flags prints the height distribution and a suggested
z band instead of guessing one:

```bash
just map grid-from-pcd data/COSS-map-planning
just map grid-from-pcd data/COSS-map-planning --z-min 9.1 --z-max 9.4
```

Further flags pass straight through — `--resolution`, `--min-points` for the PCD
path, `--min-hits` for the bag path. See [Maps](../guides/maps.md).

## `sim` — simulation

```bash
just sim coss-park   # logging sim + rosbag feed + localization recording, in parallel
```

`coss-park` is the only recipe here that is not deprecated. It runs the logging
simulation, the rosbag playback and a localization recording together, staggered
so each has started before the next depends on it. It needs the COSS recording.

The other two recipes in this module are **deprecated spellings**, kept as
pointers. With `just bag download` they are the whole list of deprecated
spellings in the repository:

| Deprecated | Use |
|------------|-----|
| `just sim planning` | `just coss planning-sim` |
| `just sim logging` | `just coss logging-sim` |
| `just bag download` | `just coss download-rosbag` |

They still work and still do the right thing; each prints the new spelling
before forwarding. They were renamed because the old names said what kind of
simulation it was and not which scenario, and every one of them is COSS Park —
the map is COSS, and the recording the logging simulation replays is the COSS
drive. Anything you write that outlives today should use the right-hand column.

## `tool` — development and monitoring

Five tools, and it is worth knowing what each is for rather than only that it
exists.

```bash
just tool rviz          # rviz2 -d ./src/launcher/autosdv_launch/rviz/autosdv.rviz
just tool plotjuggler   # ros2 run plotjuggler plotjuggler
just tool tui           # python3 ./scripts/testing/drive/run.py
just tool controller    # ros2 run control_test keyboard_control
just tool zed           # play_launch launch zed_wrapper zed_camera.launch.py camera_model:=zedxm
```

**`rviz`** opens RViz with this project's layout already loaded — the map, the
point cloud, the trajectory and the vehicle model, configured. Use it when the
stack is already running and you started it without RViz, which is what happens
over SSH.

**`plotjuggler`** plots any topic against time. It is the tool for questions
with a shape rather than a value: whether the velocity report is noisy, whether
the steering command oscillates, whether a pose jumps. It reads live topics and
recorded bags alike.

**`tui`** is the drive monitor, and it is more than a monitor. It shows
localization state, route state, operation mode, position, velocity and the NDT
debug statistics (score, point count, iterations, execution time) in a terminal,
and it drives the system from the same keyboard: initialise localization at a
named pose, set a route to a named goal, engage autonomy, stop, limit speed,
return to manual. The poses come from `scripts/testing/drive/poses.json`, and
they are site-specific — the ones shipped are COSS Park.

It is the way to run a scenario without RViz, over SSH, and it is the only
non-graphical way to do the "2D Pose Estimate, 2D Goal Pose, Engage" sequence
the tutorial does with the mouse. The poses it offers must lie on connected
lanes in the lanelet2 map, exactly as RViz's do.

**`controller`** is keyboard manual control: drive the vehicle directly, without
planning. Useful for positioning the vehicle before a run, and for checking that
the actuators respond at all. Note the known steering inversion — see
[Vehicle Interface](./software/vehicle-interface.md).

**`zed`** brings up the ZED camera node alone, on port 8081, with nothing else
running. When a camera problem might be the camera or might be the rest of the
stack, this is how you find out.

## Related

- [Operating the Vehicle](../running/on-the-vehicle.md) — the launch arguments
- [`play_launch`](../tutorial/06-play-launch.md) — the launcher the recipes wrap
- [Maps](../guides/maps.md) — what the `map` recipes produce and check
- [The COSS Park Scenario](../running/coss-park-scenario.md)

<!--
RECONCILE:
- nav "Command Reference" under Reference (Technical Reference), directly after
  Overview; nav_translations entry "指令參考".
- src/concepts/inspecting.md gains a "Tools" section pointing at
  reference/commands.md#tool-development-and-monitoring, in particular
  `just tool tui` and `just tool plotjuggler` as the non-`ros2 topic` ways to
  watch a running system.
- src/reference/overview.md: the "Vehicle Interface - CAN bus and actuator
  control (coming soon)" bullet is stale; the page exists at
  software/vehicle-interface.md. Add a Command Reference bullet to Contents.
- src/running/on-the-vehicle.md, "Everyday commands": now duplicated here.
  Reduce to the handful an operator uses daily plus a link to this page.
- src/running/on-the-vehicle.md, "The `just launch` shortcut": the sentence
  "All arguments go inside one quoted `ARGS=` string" teaches the spelling the
  recipe now rejects with exit code 2. It should read "one quoted positional
  string, without `ARGS=`".
- src/running/on-the-vehicle.md, "Simulation": `just sim coss-park` is correct,
  but the surrounding block should note the two deprecated `just sim` spellings
  rather than leaving them unmentioned.
- glossary: `play_launch`; `just` (the command runner); "engine" (a compiled
  TensorRT plan, GPU- and version-specific).
-->
