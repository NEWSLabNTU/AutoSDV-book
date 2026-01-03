# Vehicle Control Overview

Overview of the vehicle control system architecture and integration with Autoware.

## System Architecture

The vehicle control system bridges Autoware's high-level planning with low-level actuator control.

**Data Flow**:
```
Autoware Control → Control Commands → Actuator Node → PCA9685 (I2C) → Motor/Steering
                                           ↓
                        Hall Effect Sensor → Velocity Report Node → Velocity Status
                                           ↓
                        IMU Sensor → Yaw Rate Feedback (for steering)
```

### Software Components

**Autoware Control Stack** (`src/universe/autoware.universe/control/`):
```
trajectory_follower_node       # Computes control commands from trajectory
       ↓
vehicle_cmd_gate               # Safety gate
       ↓
/control/command/control_cmd   # Control command topic
```

**Vehicle Interface** (`src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/`):
```
Actuator Node (actuator.py)
  ├─ Subscribes: /control/command/control_cmd
  ├─ Subscribes: /vehicle/status/velocity_status (for speed feedback)
  ├─ Subscribes: /sensing/imu/imu_data (for yaw rate feedback)
  ├─ Multi-mode longitudinal controller (4 modes)
  ├─ Dual-mode lateral controller (2 modes based on speed)
  └─ Outputs PWM signals to PCA9685 (I2C)

Velocity Report Node (velocity_report.py)
  ├─ Reads: Hall effect sensor (KY-003) on GPIO pin
  ├─ Calculates velocity from wheel rotation
  └─ Publishes: /vehicle/status/velocity_status
```

## Component Roles

### Actuator Node

Converts Autoware control commands to PWM signals for motor and steering.

**Responsibilities**:
- Subscribe to control commands from Autoware
- Implement multi-mode longitudinal controller (speed control)
- Implement dual-mode lateral controller (steering control)
- Send PWM signals to PCA9685 via I2C
- Monitor velocity and IMU feedback
- Ensure safe operation with watchdog timers

### Velocity Report Node

Measures vehicle speed using hall effect sensor.

**Responsibilities**:
- Read hall effect sensor pulses from GPIO
- Calculate velocity from wheel rotation
- Publish velocity reports for feedback control
- Provide velocity data to Autoware stack

### PCA9685 PWM Driver

Hardware interface to ESC and servo.

**Responsibilities**:
- Receive I2C commands from actuator.py
- Generate PWM signals at 60 Hz
- Drive motor ESC (channel 0) and steering servo (channel 1)

## Topic Architecture

### Key Topics

| Topic                             | Type                      | Description                        | Rate  |
|-----------------------------------|---------------------------|------------------------------------|-------|
| `/control/command/control_cmd`    | `AckermannControlCommand` | Throttle, brake, steering commands | 10 Hz |
| `/vehicle/status/velocity_status` | `VelocityReport`          | Current velocity from sensor       | 20 Hz |
| `/vehicle/status/control_mode`    | `ControlModeReport`       | Manual/Auto mode                   | 1 Hz  |
| `/vehicle/command/actuation_cmd`  | Custom                    | PWM values to PCA9685              | 10 Hz |
| `/sensing/imu/imu_data`           | `Imu`                     | IMU data for yaw rate feedback     | 100 Hz|

### Subscriptions (Actuator Node)

- `~/input/control_cmd` - Control commands from Autoware
- `~/input/velocity_status` - Current vehicle velocity (for speed control feedback)
- `~/input/imu` - IMU data for yaw rate feedback (steering control)

### Publications (Actuator Node)

- `~/debug/control_values` - Debug control values
- `~/debug/pwm_values` - Debug PWM outputs
- `~/debug/pid_values` - Debug PID state

## Autoware Integration

### Control Command Flow

1. **Planning**: Autoware's planning stack generates target trajectory
2. **Control**: `trajectory_follower_node` converts trajectory to control commands
3. **Safety Gate**: `vehicle_cmd_gate` applies safety checks and mode switching
4. **Vehicle Interface**: Actuator node converts commands to PWM
5. **Hardware**: PCA9685 drives motor ESC and steering servo
6. **Feedback**: Velocity and IMU data flow back to controllers

### Control Modes

**MANUAL (0)**: RC controller or keyboard control
- Control commands from external sources
- Direct PWM control via controller GUI

**AUTO (1)**: Autoware autonomous control
- Control commands from trajectory follower
- Safety limits enforced by vehicle_cmd_gate

**Switching**:
```bash
# Switch to AUTO mode
ros2 service call /vehicle/set_control_mode autoware_auto_vehicle_msgs/srv/ControlModeCommand "{mode: 1}"

# Switch to MANUAL mode
ros2 service call /vehicle/set_control_mode autoware_auto_vehicle_msgs/srv/ControlModeCommand "{mode: 0}"
```

## Safety Features

### Control Mode Gate

- Enforces velocity limits (0-3 m/s)
- Enforces steering angle limits (±0.5 rad)
- Enforces acceleration limits (±2 m/s²)
- Rejects commands outside safe ranges

### Watchdog Timer

**Actuator Node** includes watchdog timer:

```python
# actuator.py watchdog implementation
last_command_time = time.time()
TIMEOUT_SECONDS = 1.0

def check_watchdog():
    if time.time() - last_command_time > TIMEOUT_SECONDS:
        # No command for 1 second → EMERGENCY STOP
        pwm.set_pwm(0, 0, 370)   # Motor: dead zone
        pwm.set_pwm(1, 0, 400)   # Steering: center
```

**Additional timeout**: Vehicle interface monitors velocity feedback
- If no feedback for >2 seconds → publish STOP command
- Ensures vehicle stops even if communication fails

### Emergency Stop

**Trigger**:
```bash
# Publish zero velocity
ros2 topic pub /control/command/control_cmd autoware_auto_control_msgs/AckermannControlCommand "{
  longitudinal: {speed: 0.0, acceleration: -5.0}
}" -1

# Or switch to MANUAL mode
ros2 service call /vehicle/set_control_mode ... "{mode: 0}"
```

**Behavior**:
- Maximum braking applied (PWM 340)
- Steering held at current angle
- Control mode switches to MANUAL

## Hardware Summary

| Component         | Interface        | Details                            |
|-------------------|------------------|------------------------------------|
| PCA9685           | I2C (0x40, bus 1)| 16-channel PWM, 60 Hz              |
| Motor ESC         | PWM (channel 0)  | Range: 280-460, Init: 370          |
| Steering Servo    | PWM (channel 1)  | Range: 350-450, Init: 400          |
| Hall Effect Sensor| GPIO             | KY-003, velocity measurement       |
| IMU               | ROS topic        | Yaw rate feedback for steering     |

For detailed hardware specifications, see [Hardware](./hardware.md).

## Configuration Files

- **Actuator parameters**: `src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/params/actuator.yaml`
- **Velocity report parameters**: `src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/params/velocity_report.yaml`
- **Vehicle information**: `src/param/autoware_individual_params/.../vehicle_info.param.yaml`

## Next Steps

- **Understand the hardware**: [Hardware Details](./hardware.md)
- **Learn control algorithms**: [Control Details](./control-details.md)
- **Test and tune**: [Tuning & Testing](./tuning-and-testing.md)
