# Minimum Risk Manoeuvre

When Autoware decides it can no longer drive safely it does not merely stop
publishing commands. It runs a **minimum risk manoeuvre** — a deliberate,
parameterised stop, with hazard lights on. MRM is live by default on every
AutoSDV launch, and an unasked-for hard brake during a first outdoor run is the
most common way people meet it.

This page covers what fires an MRM, how to watch one, what the vehicle actually
does, and — the part that has caught people out — where the parameters that
govern it really live.

## MRM is one of four ways this vehicle stops

They are separate mechanisms with separate triggers, and telling them apart is
the first step in diagnosing any unexplained stop.

| Mechanism | Decided by | Reacts to |
|---|---|---|
| **MRM** | `mrm_handler`, from the system diagnostic graph | the stack becoming unfit to drive: a dead topic, a failed module, lost localization |
| **AEB** | `autonomous_emergency_braking`, in the control container | an obstacle in the predicted path |
| `vehicle_cmd_gate` emergency | the command gate | an emergency flag or a stale heartbeat on the command path |
| Actuator watchdog | AutoSDV's own vehicle interface | one second with no control command at all — see [Control Details](./vehicle-control/control-details.md) |

Only the first is MRM. The rest are described elsewhere; this page does not
repeat them.

## The trigger chain

Nothing watches for "danger" directly. MRM is driven by whether *autonomous
mode is still available*, which is a boolean computed from a tree of
diagnostics:

```
every module's diagnostics
    -> diagnostic_graph_aggregator
         evaluates /autoware/modes/autonomous
         = map AND localization AND planning AND perception
           AND control AND vehicle AND system
    -> converter_node
    -> /system/operation_mode/availability   (tier4_system_msgs/OperationModeAvailability)
    -> mrm_handler
         autonomous unavailable for longer than
         timeout_operation_mode_availability (0.5 s)
    -> the selected MRM operator
    -> /system/emergency/control_cmd
    -> vehicle_cmd_gate -> /control/command/control_cmd -> the vehicle
```

Two consequences worth holding on to:

- **Any branch can stop the vehicle.** A perception node that stops publishing
  is as much an MRM trigger as a localization failure. When a stop is
  unexplained, read the whole graph, not the branch you suspect.
- **The stop command travels the ordinary control path.** The MRM operator
  publishes to `/system/emergency/control_cmd` and the command gate forwards it,
  so the vehicle interface sees a normal braking command. There is no separate
  wire to the actuators.

The graph itself is a set of YAML files. On the default `pose_source` it is the
installed Autoware's:

```bash
ls /opt/autoware/1.5.0/share/autoware_launch/config/system/diagnostics/
```

With `pose_source:=mcl` the launch swaps in AutoSDV's variant,
`src/launcher/autosdv_launch/config/system/diagnostics/autosdv-mcl-main.yaml`,
because the stock graph requires a PCD map and an `ndt_scan_matcher` that
`mcl` deliberately does not have. The selection is automatic; see
`autosdv_autoware.launch.xml`.

## Watching it

MRM state is published, not drawn. **Neither RViz nor `just tool tui` shows
it** — the RViz overlay plugin AutoSDV's configs used to ask for does not exist
in Autoware 1.5.0, so that display has never worked on this machine
(`docs/known-config-defects.md`, item 1). Use the topics.

```bash
# the state machine
ros2 topic echo /system/fail_safe/mrm_state

# why it is in that state: which modes are available
ros2 topic echo /system/operation_mode/availability

# which diagnostic actually failed
ros2 topic echo /diagnostics_agg
```

A GUI over the same data, if you would rather click:

```bash
ros2 run rqt_robot_monitor rqt_robot_monitor
```

!!! note "These need a running stack"

    Every command on this page reads a live ROS graph. They were checked
    against the message and launch definitions in `/opt/autoware/1.5.0`, not
    executed against a driving vehicle.

## The states

`/system/fail_safe/mrm_state` is an `autoware_adapi_v1_msgs/msg/MrmState` with
two numeric fields. `ros2 topic echo` prints the numbers, so here is the
mapping:

| `state` | Meaning |
|---|---|
| `1` `NORMAL` | autonomous mode available, nothing happening |
| `2` `MRM_OPERATING` | a manoeuvre is being executed right now |
| `3` `MRM_SUCCEEDED` | the manoeuvre completed; the vehicle is stopped |
| `4` `MRM_FAILED` | the manoeuvre could not be carried out |

| `behavior` | Meaning |
|---|---|
| `1` `NONE` | no manoeuvre selected |
| `2` `EMERGENCY_STOP` | hard braking |
| `3` `COMFORTABLE_STOP` | gentle deceleration |
| `4` `PULL_OVER` | leave the lane and stop |

**Rapid oscillation between `NORMAL` and `MRM_OPERATING`, with the hazard
lights flashing in time, is a symptom, not a mode.** It means a diagnostic is
flickering in and out of ERROR — almost always a topic that is intermittently
late rather than genuinely dead. Go and find that topic; do not tune the MRM
parameters.

## What the vehicle actually does

Three behaviours exist. Which ones the handler may choose from is configuration:

| Behaviour | Enabled here | Deceleration |
|---|---|---|
| Pull over | no (`use_pull_over: false`) | — |
| Comfortable stop | **yes** (`use_comfortable_stop: true`) | `min_acceleration: -1.0` m/s², jerk limited to ±0.3 m/s³ |
| Emergency stop | always available | `target_acceleration: -2.5` m/s², `target_jerk: -1.5` m/s³ |

Hazard lights come on for the emergency case (`turning_hazard_on.emergency:
true`). `use_emergency_holding` is `false`, so the handler is not configured to
latch the emergency indefinitely once the cause clears;
`timeout_emergency_recovery` is `5.0` s. For the precise recovery semantics,
Autoware's [MRM handler
documentation](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/system/)
is the authority — this page does not restate behaviour it cannot read out of a
config file.

## Where the parameters actually live

This is the trap. **The MRM configuration files in this repository are not
loaded.** `src/launcher/autosdv_launch/config/system/mrm_handler/`,
`mrm_emergency_stop_operator/` and `mrm_comfortable_stop_operator/` exist, but
the launch file passes the *installed Autoware's* paths, as fixed values that no
launch argument overrides:

```bash
grep -n mrm_ src/launcher/autosdv_launch/launch/components/autosdv_system_component.launch.xml
```

Every one of those lines reads `$(find-pkg-share autoware_launch)/…`, and no
package in `src/` overlays `autoware_launch`. So the files that take effect are:

```
/opt/autoware/1.5.0/share/autoware_launch/config/system/mrm_handler/mrm_handler.param.yaml
/opt/autoware/1.5.0/share/autoware_launch/config/system/mrm_emergency_stop_operator/mrm_emergency_stop_operator.param.yaml
/opt/autoware/1.5.0/share/autoware_launch/config/system/mrm_comfortable_stop_operator/mrm_comfortable_stop_operator.param.yaml
```

The in-repo copies are not merely unused — they *disagree*, which is what makes
editing them actively misleading:

| Parameter | In effect | In-repo copy (dead) |
|---|---|---|
| `use_comfortable_stop` | `true` | `false` |
| `target_acceleration` | `-2.5` m/s² | `-3.0` m/s² |
| `target_jerk` | `-1.5` m/s³ | `-3.0` m/s³ |

An operator who softened the braking by editing the repository copy, ran the
vehicle and saw −2.5 m/s² would conclude the parameter does nothing. It does;
the file does not.

!!! warning "Changing MRM braking today means editing the installed Autoware"

    There is no launch argument for these paths, and `/opt/autoware/1.5.0` is
    root-owned. Editing it breaks the property that makes the install useful —
    that `/opt/autoware/<version>` is unmodified upstream Autoware, so a bug
    found there is reportable. Treat a change to MRM braking as a change that
    needs the launch file taught to forward the path first.

## The localization accuracy check is on

Older notes in this repository state that AutoSDV disables the localization
error-ellipse check to stop false emergency stops. **The tree does not support
that claim.** A modified `localization.yaml` with the check commented out does
exist at
`src/launcher/autosdv_launch/config/system/diagnostics/localization.yaml`, but
no launch path selects the graph that includes it: the default resolves to the
installed `autoware-main.yaml`, and `pose_source:=mcl` resolves to
`autosdv-mcl-main.yaml`, which pulls in `localization-mcl.yaml` — and that file
*keeps* the accuracy check. On every pose source, `/autoware/localization/accuracy`
is an active branch of the graph.

The thresholds that therefore apply, from
`/opt/autoware/1.5.0/share/autoware_localization_error_monitor/config/localization_error_monitor.param.yaml`:

| Parameter | Value |
|---|---|
| `error_ellipse_size` | 1.5 m |
| `warn_ellipse_size` | 1.2 m |
| `error_ellipse_size_lateral_direction` | 0.3 m |
| `warn_ellipse_size_lateral_direction` | 0.25 m |

If localization uncertainty crosses those, autonomous mode goes unavailable and
an MRM follows. Watch the ellipse during a drive before blaming anything else:

```bash
ros2 topic echo /localization/localization_error_monitor/debug/ellipse_marker
```

!!! note "Under `mcl` there is no estimator-health check at all"

    `localization-mcl.yaml` drops `/autoware/localization/scan_matching_status`,
    because it is sourced from `ndt_scan_matcher`, which does not run under MCL,
    and the particle filter publishes no ROS diagnostic to put in its place. The
    rate checks, the accuracy check and the EKF status all still apply, but a
    silently diverging particle filter will not raise a diagnostic here. This is
    a known gap, recorded in the config file itself.

## Diagnosing a stop

`play_launch` writes per-node logs under `play_log/latest/node/`, which is where
the answer usually is. Start at the handler and work outwards.

```bash
# what the handler decided, and when
grep -E "MRM State|EMERGENCY" play_log/latest/node/mrm_handler/err

# every topic monitor that reported a timeout
for m in play_log/latest/node/topic_state_monitor_*/err; do
  echo "== $(basename $(dirname $m))"
  grep -E "timeout|ERROR" "$m" | tail -5
done
```

Three causes account for most stops on this vehicle.

**The fused pose times out.** `topic_state_monitor_pose_twist_fusion_filter_pose`
logs `topic is timeout. Set ERROR in diagnostics`, and the MRM oscillates. The
pose is not wrong, it is *late* — usually NDT taking too long, or the CPU
saturated. It cascades: the trajectory follower reports a too-large tracking
error and the planning validator rejects the trajectory, which produces a great
deal of alarming log output downstream of a single root cause.

```bash
ros2 topic hz /localization/pose_twist_fusion_filter/pose   # expect ~50 Hz
```

**No route.** MRM fires the instant you request autonomous mode, and
`topic_state_monitor_mission_planning_route` says the route `has not received`.
Autonomous mode was requested before a route existed. Set the initial pose, wait
for localization to converge, set the goal, wait for a trajectory, *then*
engage.

**No control command.** `/control/command/control_cmd` is silent because
planning produced no trajectory, or the trajectory starts too far from the
vehicle for the follower to accept.

```bash
ros2 topic hz /planning/scenario_planning/trajectory
ros2 topic hz /control/command/control_cmd
```

The repository's `docs/guides/mrm_troubleshooting.md` carries the longer log
walkthrough, including the exact messages each failure emits.

## Turning MRM off

There is no MRM-only switch. The nearest thing disables the whole system
component:

```bash
play_launch launch autosdv_launch autosdv.launch.yaml launch_system:=false
```

```bash
just launch "launch_system:=false"
```

That removes the system monitor, the component state monitor, the duplicated
node checker — and the diagnostic graph aggregator itself, which is what
publishes `/system/operation_mode/availability`. It is a bench configuration for
running perception or sensing alone. It is not a way to make a moving vehicle
stop braking.

## Related

- [Vehicle Control: Overview](./vehicle-control/overview.md) — the vehicle-side
  watchdog and the actuator's own stop behaviour
- [Localization Methods](./localization-methods.md) — `pose_source`, and what
  `mcl` changes about the diagnostic graph
- [Operating the Vehicle](../running/on-the-vehicle.md) — `launch_system` and
  the other subsystem toggles
- `docs/guides/mrm_troubleshooting.md` and `docs/known-config-defects.md` in the
  repository
- Autoware's [fail-safe
  design](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/fail-safe/)

<!--
RECONCILE (phase 2):
- nav entry "Minimum Risk Manoeuvre" -> guides/mrm.md, under Building the Stack >
  4.5 Control and Safety, after the four Vehicle Control pages.
- nav_translations entry: "Minimum Risk Manoeuvre" -> "最小風險操作".
- glossary (src/concepts/glossary.md): MRM is already defined. Add:
  "comfortable stop" (the -1.0 m/s2 MRM behaviour, enabled on this install),
  "diagnostic graph" (the YAML tree the aggregator evaluates into
  /autoware/modes/autonomous), and "operation mode availability" (the boolean
  set that gates engaging autonomous). Cross-reference MRM to this page.
- cross-link from guides/vehicle-control/overview.md, in its "Emergency Stop"
  section: MRM is a distinct mechanism, link here.
- cross-link from guides/localization-methods.md, in the mcl section: under mcl
  the diagnostic graph carries no estimator-health check, so a diverging filter
  raises no diagnostic. Link here.
- concepts/inspecting.md: add /system/fail_safe/mrm_state and
  /system/operation_mode/availability to the topics worth echoing.
- DEFECT for phase 3 (not mine to fix): guides/vehicle-control/overview.md
  publishes to /control/command/control_cmd with type
  autoware_auto_control_msgs/AckermannControlCommand. That package does not
  exist in Autoware 1.5.0; the type is autoware_control_msgs/msg/Control. The
  command as written cannot run.
-->
