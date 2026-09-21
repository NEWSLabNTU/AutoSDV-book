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
