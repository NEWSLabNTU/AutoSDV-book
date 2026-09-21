# 2. Planning Simulation

The planning component, on its own.

There are no sensors here, no localization and no perception. You tell the
vehicle where it is and where to go; it works out a route, turns that into a
trajectory, and follows it. Because the pose is *given*, nothing about position
can be wrong — so anything that goes wrong is a planning decision, which is
exactly what makes this the right place to learn planning.

It needs no GPU, no rosbag and no sensors.

<figure style="text-align: center; margin: 1.5em auto; max-width: 960px;">
  <video autoplay loop muted playsinline controls style="width: 100%; border-radius: 8px;">
    <source src="../../figures/planning_sim_video/planning-sim.webm" type="video/webm">
  </video>
  <figcaption>The whole page, end to end: launch it from the repository root, place the vehicle, give it a goal, engage — and watch it drive there.</figcaption>
</figure>

## Launch it

```bash
source /opt/autoware/1.5.0/setup.bash
source install/setup.bash

play_launch launch autoware_launch planning_simulator.launch.xml \
  map_path:=$PWD/data/COSS-map-planning \
  vehicle_model:=autosdv_vehicle \
  sensor_model:=autosdv_sensor_kit
```

Read that command as four parts, because every launch in this book has the same
shape:

```
play_launch launch   <package>         <launch file>                  <arg>:=<value> ...
             ^                          ^                              ^
             the verb                   a file inside that package     launch arguments
```

`play_launch launch` takes a package name and a launch file in it, then any
number of `name:=value` arguments — the same grammar as `ros2 launch`, which you
can substitute here if you prefer ([why this book uses
`play_launch`](../running/on-the-vehicle.md)).

The three arguments are the whole configuration: which map, which vehicle
description, which sensor kit description.

Note the package is `autoware_launch` — this launch file comes from Autoware
itself. What makes it an *AutoSDV* simulation is `vehicle_model:=autosdv_vehicle`,
which supplies the real vehicle's dimensions, wheelbase and steering limits, so
the trajectories you see are ones this vehicle could actually follow.

??? note "The `just` shortcut"

    ```bash
    just coss planning-sim
    ```

    The same command with `--web-addr 0.0.0.0:8081` added. Once you are past
    this tutorial, use it.

On a desktop machine the launcher reports all nodes ready in a second or two —
34 nodes, 15 containers, 70 composable nodes — and the RViz window is usable
within about a minute.

## What you are looking at

![RViz at startup, with the COSS Park map loaded and no vehicle placed](../figures/simulation/planning-01-startup.png)

Four regions matter.

**The toolbar**, along the top. `Interact`, `Move Camera`, `Select`,
`Focus Camera`, `Measure`, then the ones you will use:
**`2D Pose Estimate`**, **`2D Goal Pose`**, `2D Rough Goal Pose`,
`2D Dummy Pedestrian`, **`2D Dummy Car`**, `2D Dummy Bus`, `2D Checkpoint Pose`.

**The AutowareStatePanel**, on the left. This is the thing to watch, and at
startup it reads:

| Field | At startup |
|---|---|
| Autoware Control | on |
| *(mode buttons)* | `Auto` `Local` `Remote` `Stop` — `Stop` selected |
| Routing | **Unknown** |
| Localization | **Uninitialized** |
| Motion | Moving |
| MRM State / Behavior | Inactive |

**The 3-D view**, in the middle, showing the COSS Park lanelet map from above.
The white lines are lane boundaries. There is no vehicle yet, because
localization is uninitialized.

**The Displays panel**, on the right, with one group per pipeline component —
System, Map, Sensing, Localization, Perception, Planning, Control. Those are
[the same component names the topics use](../concepts/autoware-conventions.md).

## Step 1 — Place the vehicle

Click **`2D Pose Estimate`** in the toolbar, then **click and drag** on the map:
the click sets the position, and the direction you drag sets the heading. Release.

The panel changes:

![After setting the initial pose: Localization Initialized, gear D](../figures/simulation/planning-02-pose-set.png)

| Field | Before | After |
|---|---|---|
| Localization | Uninitialized | **Initialized** ✓ |
| Routing | Unknown | **Unset** |
| Motion | Moving | **Stopped** |
| gear | `P` | **`D`** |

The vehicle now exists, is stationary, is in drive, and has nowhere to go.

Watch that panel rather than the map. Every step in this tutorial shows up there
first, and when something does not work it is nearly always because a field you
expected to change did not.

!!! note "There is also an `Initialize with GNSS` button"

    Next to the Localization row. It is for a real vehicle outdoors with a fix.
    In simulation there is no GNSS, so place the pose by hand.

## Step 2 — Give it a goal

Click **`2D Goal Pose`**, then click and drag somewhere else on the road — again,
drag sets which way the vehicle should be facing when it arrives.

Two things appear, and the difference between them is the heart of planning:

- **The route** — the sequence of lanes from here to there, computed once from
  the lanelet map. It does not change unless you change the goal.
- **The trajectory** — the actual path with speeds along it, recomputed
  continuously. This is what the controller follows.

`Routing` in the panel becomes **Set**.

### When nothing appears

A goal that is refused says nothing at all — no message, no marker, `Routing`
simply stays `Unset`. There are two reasons, and they need different responses.

**The goal was not inside a lanelet.** Being on something that looks like road
is not enough: the goal has to fall within a lane the map defines, and on this
map that is a narrower target than it appears. Heading is forgiving — roughly
±45° of the lane direction is accepted — so it is nearly always position that
failed. Click closer to the middle of a lane, not on the line that bounds it.

**The stack was not ready yet.** The mission planner needs about a minute after
launch before it will accept anything, and until then a goal is refused exactly
as if it were off-road. Wait and click again; the same goal that failed will
take.

!!! tip "A goal that is known to work"

    If you would rather see the rest of the tutorial before hunting for a
    routable spot, use this pair. It was driven end to end while writing this
    page: 28.4 m, up to 3.12 m/s, arriving within 0.1 m.

    | | x | y | heading |
    |---|---|---|---|
    | initial pose | −1.84 | −8.28 | ≈ 175° |
    | goal | −27.9 | −4.4 | ≈ 135° |

    The coordinates are metres in the `map` frame, whose origin is fixed by
    `map_projector_info.yaml` — see [The Map and the
    Rosbag](./05-map-and-rosbag.md). You are placing these by eye in RViz, so
    getting within a metre or so is enough.

## Step 3 — Drive

Click **`Auto`** in the AutowareStatePanel.

The vehicle follows the trajectory. Watch the speedometer at the top of the view
and the `Motion` field, which becomes `Moving`.

**If the first click does nothing, click it again.** Engaging is refused while
the operation mode is still changing — which it is for a second or two after the
route is set — and the refusal is silent. A second click a moment later takes.

If it still will not move, the answer is in the panel. `Routing | Unset` means no
goal took. `Localization | Uninitialized` means step 1 did not take. Both are
more common than a real planning failure.

## Now experiment

This is the part worth your time. The planner is the only thing running, so
everything you see is the planner.

### Put an obstacle in the way

Select **`2D Dummy Car`** and click on the road ahead of the vehicle.

Watch what happens: the **route does not change** — the lanes are still the
lanes — but the **trajectory** does. Depending on how much room there is, the
vehicle slows, stops, or steers around it. That is the distinction from step 2,
made visible.

`2D Dummy Pedestrian` and `2D Dummy Bus` do the same with different footprints
and different behaviour rules.

### Move the goal while it is driving

Set a new `2D Goal Pose` mid-drive. The route replans from the current position.

### Watch the pipeline in a second terminal

```bash
source /opt/autoware/1.5.0/setup.bash && source install/setup.bash

ros2 topic echo /planning/scenario_planning/trajectory --once
ros2 topic echo /control/command/control_cmd --once
ros2 topic hz /control/command/control_cmd
```

The trajectory is the planner's output; the control command is what the
controller derived from it. In a real vehicle that command reaches the actuators.

This is also the moment the naming convention pays off — `/planning/…` and
`/control/…` tell you which component produced what, without knowing anything
else. See [Autoware Conventions](../concepts/autoware-conventions.md).

## Stop

`Ctrl-C` in the `play_launch` terminal, then confirm nothing is left:

```bash
ros2 node list    # should be empty
```

## What this did not test

Everything to do with the real world:

- **Localization** — you asserted the pose; no sensor had to find it
- **Perception** — obstacles appeared because you clicked, perfectly known
- **Sensing** — no driver ran, no point cloud existed

Which is the point of the next page: the same stack, with those three put back,
and a recording of a real drive to feed them.

**Next:** [3. Logging Simulation](./03-logging-simulation.md)
