# Vehicle Interface

The vehicle interface bridges the Autoware control and vehicle actuators. It is served by the `autosdv_vehicle_launch` repository located at `src/vehicle/autosdv_vehicle_launch`. It includes the following packages.

- `autosdv_vehicle_description`

  It provides vehicle appearance parameters.

- `autosdv_vehicle_launch`

  It provides a launch file that runs necessary nodes to drive the vehicle.

- `autosdv_vehicle_interface`

  The package provides the node that converts the Autoware control commands to motor power and provides vehicle status reporting nodes for cruise control.

To launch the vehicle interface on its own:

```sh
source install/setup.bash
play_launch launch autosdv_vehicle_launch vehicle_interface.launch.xml
```

`play_launch` is the launch orchestrator this project uses in place of
`ros2 launch`; see [Operating the Vehicle](../../running/on-the-vehicle.md#why-play_launch-and-not-ros2-launch).

Normally the interface is started as part of the whole system, and is turned off
with `launch_vehicle:=false` rather than started separately.

## The Velocity Reporting Node

The node is implemented in `velocity_report.py`. It periodically reads the Hall effect sensor and counts the magnet markers embedded on the wheel in each period. In this way, the rotation speed of the wheel can be measured, and the instantaneous speed can be calculated by multiplying by the wheel radius.

## The Actuator Node

The node implemented in `actuator.py` reads a target speed and controls the motor power to reach to that speed. It uses a PID controller to compute PWM values and applies them on DC motors.

## The Steering Status Node

The node is implemented in `steering_status.py`. It publishes
`/vehicle/status/steering_status` (a `SteeringReport`) at 30 Hz, and it is what
the rest of Autoware reads to learn where the front wheels are pointing.

!!! warning "`steering_status` is the command, not a measurement"

    **The vehicle has no steering angle sensor.** The node subscribes to
    `/control/command/control_cmd`, clamps the commanded tyre angle to
    `max_steering_angle` from `actuator.yaml`, applies a first-order lag to
    imitate servo response, and publishes the result. Nothing in that path
    observes the wheels.

    As a stand-in for a missing sensor that is a defensible choice. The problem
    is who believes it.

Three consequences, all of which have bitten this project:

- **The lateral controller has never had feedback.**
  `tier4_control_launch` remaps `~/input/current_steering` to this topic, so
  `autoware_mpc_lateral_controller` takes the echo as its controller state. The
  lateral loop is therefore closed on its own output: servo lag, deadband,
  saturation and linkage play are invisible to MPC, which models the actuator
  as ideal. If you are tuning lateral gains, you are tuning against a model
  that cannot be wrong — so a gain that looks stable in the controller's own
  state may not be stable on the vehicle.
- **The engage check cannot fail.**
  `autoware_operation_mode_transition_manager` gates engagement on the command
  and the status agreeing. With an echo they cannot disagree, so that check
  passes unconditionally.
- **It reads exactly 0 in any bag recorded while disengaged**, because there is
  no control command to echo. No manually driven recording can be used to
  validate steering geometry or the lateral controller, however long it is.

To see this for yourself on a running vehicle, compare the two topics — the
report is the command, delayed:

```sh
ros2 topic echo /control/command/control_cmd --field lateral.steering_tire_angle
ros2 topic echo /vehicle/status/steering_status --field steering_tire_angle
```

A real measurement is available from sensors the vehicle already carries: the
kinematic bicycle relation gives the tyre angle from yaw rate and speed. It is
not wired in, and it should not be wired in casually — it is undefined at low
speed, it inherits the wheel-speed scale error, and it closes a loop through the
IMU. The measurements, the options and their limits are in
[`docs/reports/steering-status-has-no-feedback.md`](https://github.com/NEWSLabNTU/AutoSDV/blob/main/docs/reports/steering-status-has-no-feedback.md).

<!--
RECONCILE:
- nav: no change needed; this page is already under Reference > Software.
- src/guides/vehicle-control/control-details.md and tuning-and-testing.md:
  both discuss lateral tuning without saying the controller has no feedback.
  Add a cross-link to
  reference/software/vehicle-interface.md#the-steering-status-node.
- src/guides/vehicle-control/control-details.md line ~329 gives
  `max_steering_angle: 0.349` and overview.md gives "±0.5 rad"; the tree's
  `autosdv_vehicle_interface/params/actuator.yaml` says `0.5`. One of the three
  is wrong and the book states both. Needs an owner to settle it against the
  hardware.
- glossary: `SteeringReport`; "lateral controller" / MPC.
-->
