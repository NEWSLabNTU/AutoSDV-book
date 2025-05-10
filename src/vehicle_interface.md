# Vehicle Interface

The vehicle interface bridges the Autoware control and vehicle actuators. It is served by the `autosdv_vehicle_launch` repository located at `src/autoware/vehicle/autosdv_vehicle_launch`. It includes the following packages.

- `autosdv_vehicle_description`

  It provides vehicle appearance parameters.

- `autosdv_vehicle_launch`

  It provides a launch file that runs necessary nodes to drive the vehicle.

- `autosdv_vehicle_interface`

  The package provides the node that converts the Autoware control commands to motor power and provides vehicle status reporting nodes for cruise control.

To launch the vehicle interface for the vehicle,

```sh
ros2 launch autosdv_vehicle_launch vehicle_interface.launch.xml
```

## The Velocity Reporting Node

The node is implemented in `velocity_report.py`. It periodically reads the Hall effect sensor and counts the magnet markers embedded on the wheel in each period. In this way, the rotation speed of the wheel can be measured, and the instant speed can be calculated by multiplying the wheel radius.

## The Actuator Node

The node implemented in `actuator.py` reads a target speed and controls the motor power to reach to that speed. It uses a PID controller to compute PWM values and applies them on DC motors.
