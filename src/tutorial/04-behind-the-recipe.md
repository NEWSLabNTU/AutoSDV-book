# 4. Behind the Recipe

`just demo run` did a lot and showed you none of it. This page opens it, one
layer at a time, downwards — because the day something fails, you will need to
run the layer underneath the one that broke.

There are four:

```
just demo run                  ← a recipe
  play_launch launch …         ← a launch runner (ours)
    ros2 launch …              ← the reference launch runner
      source …/setup.bash      ← the environment everything needs
```

## Layer 4 — the environment

At the bottom, always, two lines:

```bash
source /opt/autoware/1.5.0/setup.bash   # ROS 2 Humble AND Autoware
source install/setup.bash               # the AutoSDV workspace
```

`just` recipes do this internally, which is why `just demo run` works in a
terminal where `ros2` is not even on the `PATH`. That convenience is also why
people get stranded the first time they run something by hand.

If any command on this page fails with "command not found" or "package not
found", it is this layer. See
[The Environment](../concepts/environment.md).

## Layer 3 — `ros2 launch`

The standard way to start a ROS 2 system:

```bash
ros2 launch <package> <launch_file> [name:=value ...]
```

For the logging simulation:

```bash
ros2 launch autosdv_launch logging_simulation.launch.yaml
```

Three positions: the installed package, the launch file inside it, then
arguments as `name:=value`. (See [Launch Files](../concepts/launch-files.md) for
what the launch file is doing with them.)

This works, and it is the reference implementation. It has one practical problem
for a system like this, which is the reason for the next layer up.

### The orphan problem

AutoSDV runs most of its nodes as **composable nodes** inside container
processes. `ros2 launch` starts those containers; killing `ros2 launch` by PID
kills the launcher and leaves the containers running — still publishing, still
holding memory and the GPU. They do not stop when your terminal closes.

If you use `ros2 launch`, kill the **process group**:

```bash
ros2 launch autosdv_launch logging_simulation.launch.yaml &
LAUNCH_PID=$!
# ... testing ...
kill -- -$(ps -o pgid= -p $LAUNCH_PID | tr -d ' ')
```

and check afterwards:

```bash
ros2 node list    # should be empty
```

## Layer 2 — `play_launch`

```bash
play_launch launch autosdv_launch logging_simulation.launch.yaml
```

Same three positions, same arguments — a drop-in for `ros2 launch`. It does the
process-group shutdown above for you, and adds a web UI at
`http://127.0.0.1:8080`, per-process CPU/memory/GPU monitoring, and
`/diagnostics` collection.

It is **our own software**, which matters enough to have
[its own page](./06-play-launch.md) covering when it can differ from
`ros2 launch` and how to fall back.

### The command that answers "did my argument work"

```bash
play_launch resolve autosdv_launch autosdv.launch.yaml \
  pose_source:=mcl -o ./tmp/resolved.yaml
```

This evaluates the launch file and writes out every node it *would* start with
every parameter it *would* get, without starting anything. When an argument
seems to have no effect, this is how you find out whether it reached a node —
and it is faster than launching to find out.

## Layer 1 — the recipe

```bash
just demo run
```

The recipe is `demo/justfile`, which calls `demo/scripts/run-coss-ndt.sh`. Its
core is this:

```bash
play_launch launch --web-addr 0.0.0.0:8081 \
    autosdv_launch logging_simulation.launch.yaml \
    pose_source:=$POSE_SOURCE \
    map_path:=$MAP \
    use_gnss:=false \
    rviz:=$RVIZ
```

plus the orchestration you did not see:

1. **Fetch the rosbag** if missing
2. **Stop any previous stack** — two would fight over the same topics
3. **Wait for `ndt_scan_matcher` to exist**, then allow 25 s for the map to load.
   This is why the recipe is more reliable than doing it by hand: it waits for a
   *condition*, not for a guessed number of seconds
4. **Start the wheel-speed scaler** and remap the bag's raw velocity topic
   through it
5. **Record diagnostics** to `tmp/demo-runs/<label>_<stamp>/bag`
6. **Play the bag** and seed a known pose 8 s in
7. **Print metrics** and leave the stack up

Steps 3 and 6 are the ones that make the result reproducible, and they are
exactly what a person doing this by hand gets wrong.

### Reproducing it manually

```bash
# terminal 1
source /opt/autoware/1.5.0/setup.bash && source install/setup.bash
play_launch launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=cuda_ndt use_gnss:=false

# terminal 2 — after the map has loaded
source /opt/autoware/1.5.0/setup.bash && source install/setup.bash
ros2 bag play data/rosbags/outdoor_20251226_153115 --clock

# terminal 3 — seed the pose, or click 2D Pose Estimate in RViz
python3 demo/scripts/seed_initialpose.py
```

Same system. What you lose is the waiting, the recording and the metrics.

## `just` recipes and what they wrap

Throughout this book a recipe is shown next to its underlying command. The ones
worth knowing:

| Recipe | Wraps |
|---|---|
| `just build` | `colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --cargo-args --release` |
| `just launch` | `play_launch launch --web-addr 0.0.0.0:8081 autosdv_launch autosdv.launch.yaml` (plus `rviz:=false` with no `$DISPLAY`) |
| `just sim logging` | the same, with `logging_simulation.launch.yaml` |
| `just sim planning` | `play_launch launch autoware_launch planning_simulator.launch.xml` with the COSS map and AutoSDV models |
| `just demo stop` | kills the stack's process group |

Two details in `just launch` worth carrying: the web UI moves to **8081**, not
`play_launch`'s own 8080, and RViz is silently disabled when `$DISPLAY` is unset.
Both are invisible from the command line, which is why this book teaches the
`play_launch` form first.

Arguments to a recipe go in one quoted string — a `just` requirement, not a ROS
one:

```bash
just launch ARGS="pose_source:=ndt launch_perception:=false"
```

## Which layer to use

- **Daily work**: the `just` recipes. They are shorter and carry flags that
  matter.
- **Anything unusual** — a new argument combination, a different launch file,
  debugging: `play_launch launch` directly.
- **When you suspect the launcher itself**: `ros2 launch`. See
  [the next page](./06-play-launch.md).

**Next:** [5. The Map and the Rosbag](./05-map-and-rosbag.md) — what the data you
have been replaying actually is.
