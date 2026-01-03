# Tuning & Testing

Practical guide to test, tune, and calibrate the vehicle control system.

## Quick Test Procedure

### Prerequisites

- Vehicle powered on
- PCA9685 PWM board connected (I2C interface)
- ROS 2 environment sourced (`source install/setup.bash`)

### 1. Launch Basic Control

```bash
# Launch minimal control system (no planning, no localization)
make play-basic-control
```

This starts:
- Vehicle interface (PCA9685 PWM communication via I2C)
- Control nodes (lateral + longitudinal control)
- Basic subscribers/publishers

**Expected output**:
```
[vehicle_interface_node]: Connected to PCA9685 on I2C bus
[trajectory_follower_node]: Lateral controller started
[longitudinal_controller_node]: Longitudinal controller started
```

### 2. Launch PlotJuggler

In a new terminal:
```bash
# Launch PlotJuggler for real-time visualization
make plot-test
```

**Topics to monitor**:
- `/vehicle/status/control_mode` - Control mode (manual/auto)
- `/vehicle/status/velocity_status` - Current velocity
- `/control/command/control_cmd` - Control commands (throttle, brake, steering)
- `/control/current_gate_mode` - Control gate mode

**PlotJuggler setup**:
1. Window opens automatically
2. Select topics from left panel
3. Drag topics to plot area
4. Observe real-time data

### 3. Verify System Status

```bash
# Check control mode
ros2 topic echo /vehicle/status/control_mode
# Should show: mode: AUTO

# Verify control commands
ros2 topic echo /control/command/control_cmd
# Should show incoming commands

# Check PCA9685 connection
sudo i2cdetect -y 1
# Should show device at address 0x40 (default PCA9685 address)

# Verify vehicle interface node running
ros2 node list | grep vehicle_interface
```

## Manual Control Testing

Test manual control with controller GUI (safe method).

### Launch Controller GUI

```bash
# Launch controller GUI
make run-controller

# Controls:
# W - Forward
# S - Backward
# A - Left
# D - Right
# Space - Brake/Stop
# Q - Quit
```

**Why use the controller GUI?**
- Safe: Commands stop when you release keys
- Prevents runaway: Vehicle stops when GUI closes
- Visual feedback: See current control state

**Verify**:
- Vehicle responds to commands immediately
- Steering centers when A/D released
- Brake (Space) stops vehicle smoothly
- No unexpected movements

**⚠️ Warning**: Do NOT use `ros2 topic pub` to send control commands directly. The actuator remembers the last command and the vehicle will keep moving. Always use the controller GUI.

## Automated Test Scenarios

The `control_test` package provides automated test scenarios that safely verify control system performance.

### Test 1: Velocity Tracking

**Objective**: Verify velocity controller tracks target velocity.

```bash
# Terminal 1: Launch basic control
make play-basic-control

# Terminal 2: Launch PlotJuggler
make plot-test

# Terminal 3: Run velocity tracking test
ros2 launch control_test velocity_tracking.launch.py
```

This test automatically:
- Ramps velocity from 0 to 2 m/s
- Holds at 2 m/s for 5 seconds
- Ramps down to 0

**Expected in PlotJuggler**: Smooth velocity curve matching commanded trajectory, settling within ±0.2 m/s.

### Test 2: Steering Response

**Objective**: Verify steering servo responds to commands.

```bash
# Terminal 1: Basic control running
# Terminal 2: PlotJuggler running

# Terminal 3: Run steering response test
ros2 launch control_test steering_response.launch.py
```

This test cycles through:
- Center (0°) → Left (15°) → Center → Right (15°) → Center

**Expected in PlotJuggler**: Steering angle tracking commanded angles within 100ms.

### Test 3: Emergency Stop

**Objective**: Verify brake response and safety.

```bash
# Use controller GUI for this test
make run-controller

# Steps:
# 1. Press W to accelerate to ~1 m/s
# 2. Press Space (brake) to stop
# 3. Observe stopping distance
```

**Expected**: Vehicle stops smoothly within 1 meter.

**Monitor in PlotJuggler**: Deceleration should be smooth, no jerking.

## PID Tuning

### Longitudinal Controller (Speed Control)

**Test procedure**:
1. Launch basic control: `make play-basic-control`
2. Launch PlotJuggler: `make plot-test`
3. Run step response test: `ros2 launch control_test step_response.launch.py`
4. Observe velocity response in PlotJuggler

**⚠️ Safety**: Use the `control_test` package for PID tuning, not manual `ros2 topic pub` commands. The test package includes safety timeouts and watchdogs.

**Tuning effects**:

**Kp (Proportional)**:
- **Too high**: Overshooting, oscillation
- **Too low**: Slow response, large steady-state error
- **Typical range**: 30-70 (default: 50.0)

**Ki (Integral)**:
- **Purpose**: Eliminates steady-state error
- **Too high**: Integral windup, overshooting
- **Too low**: Permanent offset from target
- **Typical range**: 2-10 (default: 5.0)

**Kd (Derivative)**:
- **Purpose**: Reduces overshooting, dampens oscillations
- **Too high**: Amplifies noise, jerky motion
- **Too low**: Overshooting
- **Typical range**: 0.5-5 (default: 2.0)

**Tuning process**:
1. Start with Kp only (Ki=0, Kd=0)
2. Increase Kp until response is fast but oscillates slightly
3. Add Ki to eliminate steady-state error
4. Add Kd to reduce overshooting

### Lateral Controller (Steering Control)

**Parameters to tune**:
- `kp_steer`: Proportional gain for yaw rate control
- `ki_steer`: Integral gain
- `kd_steer`: Derivative gain

**Effects are similar** to longitudinal controller:
- Higher Kp → faster steering response, but may oscillate
- Ki → eliminates steady-state yaw rate error
- Kd → dampens oscillations

**Default values**:
- `kp_steer: 10.0`
- `ki_steer: 1.0`
- `kd_steer: 0.5`

### How to Edit Parameters

```bash
# Edit PID gains for longitudinal control
vi src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/params/actuator.yaml
# Update: kp_speed, ki_speed, kd_speed

# Edit PID gains for lateral control
vi src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/params/actuator.yaml
# Update: kp_steer, ki_steer, kd_steer

# Rebuild
make build

# Test
make play-basic-control
```

## Calibration Procedures

### Calibrate PWM Values

**Objective**: Determine PWM-to-velocity mapping for your specific motor/ESC.

**⚠️ Safety Warning**: PWM calibration requires careful testing. The actuator remembers the last command, so always have an emergency stop method ready.

**Recommended approach**: Use the `control_test` package calibration mode:

```bash
# Launch PWM calibration tool (with safety features)
ros2 launch control_test pwm_calibration.launch.py
```

This tool:
- Incrementally tests PWM values with confirmation
- Includes automatic timeout/watchdog
- Records velocity at each PWM value
- Generates calibration parameters

**Alternative (Advanced)**: Use controller GUI with monitoring:

```bash
# Terminal 1: Launch vehicle interface
ros2 run autosdv_vehicle_interface vehicle_interface_node

# Terminal 2: Monitor velocity
ros2 topic echo /vehicle/status/velocity_status

# Terminal 3: Use controller GUI
make run-controller
# Gradually increase throttle, record velocity at each level
```

**Update parameters**:
```yaml
# File: params/actuator.yaml
min_pwm: 280           # Minimum PWM (maximum reverse)
init_pwm: 370          # Neutral/dead zone
max_pwm: 460           # Maximum PWM (maximum forward)
brake_pwm: 340         # Emergency brake PWM
```

### Calibrate Steering

**Objective**: Measure steering angle vs PWM.

Measure steering angle vs PWM using controller GUI:

```bash
# Terminal 1: Launch vehicle interface
ros2 run autosdv_vehicle_interface vehicle_interface_node

# Terminal 2: Launch controller GUI
make run-controller

# Terminal 3: Monitor steering commands
ros2 topic echo /vehicle/command/actuation_cmd
```

**Procedure**:
1. Press A (left) gradually → Observe steering PWM and measure wheel angle
2. Center steering → Verify PWM returns to ~400, wheel at 0°
3. Press D (right) gradually → Observe steering PWM and measure wheel angle
4. Record max left angle (PWM ~350) and max right angle (PWM ~450)
5. Calculate: `angle_per_pwm = (max_angle - min_angle) / (450 - 350)`

**Safety**: Controller GUI automatically centers steering when key is released.

### Calibrate Hall Effect Sensor

**Objective**: Verify velocity measurement accuracy.

Measure sensor accuracy using controller GUI:

```bash
# Terminal 1: Launch vehicle interface
ros2 run autosdv_vehicle_interface velocity_report_node

# Terminal 2: Monitor velocity feedback
ros2 topic echo /vehicle/status/velocity_status

# Terminal 3: Controller GUI
make run-controller
```

**Procedure**:
1. Mark start position on ground
2. Use controller GUI (press W) to drive vehicle forward slowly
3. Stop at exactly 1 meter mark (press Space to brake)
4. Record total pulses (if monitoring raw sensor) or verify velocity matches expected
5. Calculate: `pulses_per_meter = total_pulses / 1.0`

**Update**:
```yaml
# File: params/velocity_report.yaml
wheel_diameter: 0.1            # Measured in meters
markers_per_rotation: 4        # Number of magnets on wheel
gpio_pin: 17                   # GPIO pin number
```

## Understanding the Plots

### Velocity Plot

```
Target velocity (commanded)   ----
Actual velocity (measured)    ____

Good tracking:
  Target and actual overlap closely

Poor tracking:
  Large gap between target and actual
  Oscillations around target
```

**What to look for**:
- **Rise time**: How quickly velocity reaches target (should be < 2 seconds)
- **Overshoot**: Velocity should not exceed target by >10%
- **Steady-state error**: Final error should be < 0.1 m/s
- **Oscillation**: Should settle within 1-2 cycles

### Control Command Plot

```
Throttle (positive)    - Acceleration command
Brake (negative)       - Deceleration command
Steering (±rad)        - Steering angle command

Good response:
  Commands change smoothly
  No sudden jumps

Poor response:
  Oscillating commands
  Saturated commands (stuck at limits)
```

**What to look for**:
- **Smooth transitions**: Commands should ramp, not jump
- **No saturation**: Commands should stay within limits (not constantly at MAX/MIN)
- **Correlation**: Velocity error should correlate with throttle/brake commands

## Troubleshooting

### No response from vehicle

**Symptoms**: Commands published but vehicle doesn't move.

**Diagnosis**:
```bash
# 1. Check PCA9685 connection
sudo i2cdetect -y 1
# Should show device at address 0x40

# 2. Check control mode
ros2 topic echo /vehicle/status/control_mode
# Should show: mode: AUTO (1)

# 3. Check PWM commands
ros2 topic echo /vehicle/command/actuation_cmd
# Should show motor_pwm != 370

# 4. Check vehicle interface node
ros2 node info /vehicle_interface_node

# 5. Check I2C permissions
groups $USER | grep i2c
# Should show i2c group
```

**Solutions**:
- Check I2C wiring to PCA9685 (SDA, SCL, VCC, GND)
- Add user to i2c group: `sudo usermod -a -G i2c $USER`
- Switch to AUTO mode: `ros2 service call /vehicle/set_control_mode autoware_auto_vehicle_msgs/srv/ControlModeCommand "{mode: 1}"`
- Restart vehicle interface: `ros2 run autosdv_vehicle_interface vehicle_interface_node`

### Velocity oscillates

**Symptoms**: Vehicle speed oscillates around target.

**Diagnosis**:
- Plot velocity in PlotJuggler
- Check control commands for oscillation
- Monitor hall effect sensor for noise

**Solutions**:
1. **Reduce PID Kp**: Decrease proportional gain
2. **Increase PID Kd**: Add damping
3. **Filter sensor**: Increase `velocity_measurement_filter_alpha` (0.3 → 0.5)
4. **Check mechanical**: Loose sensor mount causes noise

### Steering lag or jitter

**Symptoms**: Steering response delayed or erratic.

**Diagnosis**:
```bash
# Monitor steering commands and servo response
ros2 topic echo /control/command/control_cmd
ros2 topic echo /vehicle/command/actuation_cmd
```

**Solutions**:
1. **Power supply**: Check servo gets clean 5V with sufficient current (>1A)
2. **PWM range**: Verify PWM values 350-450
3. **Mechanical binding**: Check steering linkage moves freely
4. **PCA9685 test**: Use `control_test` calibration mode to test servo independently

### Hall effect sensor not working

**Symptoms**: Velocity always reads 0.

**Diagnosis**:
```bash
# Monitor velocity feedback
ros2 topic echo /vehicle/status/velocity_status

# Check if velocity updates when vehicle moves
# (Use controller GUI to drive vehicle slowly)

# Check velocity_report node is running
ros2 node list | grep velocity_report
```

**Solutions**:
1. **Wiring**: Check sensor power (VCC, GND) and signal connections
2. **Magnet alignment**: Ensure magnets on wheel pass close to sensor (< 5mm gap)
3. **GPIO pin**: Verify correct GPIO pin configured in `velocity_report.yaml`
4. **Pull-up resistor**: KY-003 may need pull-up on signal pin
5. **LED indicator**: KY-003 has LED that blinks when detecting magnets - check if blinking when wheel rotates

## Quick Reference

### Launch commands
```bash
make play-basic-control        # Launch control system
make plot-test                 # Launch PlotJuggler
make run-controller            # Controller GUI (safe manual control)
```

### Monitor topics
```bash
ros2 topic echo /vehicle/status/control_mode
ros2 topic echo /vehicle/status/velocity_status
ros2 topic echo /control/command/control_cmd
ros2 topic echo /vehicle/command/actuation_cmd
ros2 topic hz /vehicle/status/velocity_status
```

### PCA9685 connection
```bash
sudo i2cdetect -y 1            # Check I2C device at 0x40
groups $USER | grep i2c        # Verify I2C permissions
```

### Control mode
```bash
# AUTO mode
ros2 service call /vehicle/set_control_mode autoware_auto_vehicle_msgs/srv/ControlModeCommand "{mode: 1}"

# MANUAL mode
ros2 service call /vehicle/set_control_mode autoware_auto_vehicle_msgs/srv/ControlModeCommand "{mode: 0}"
```

### Parameter files
```
Actuator control:    src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/params/actuator.yaml
Velocity reporting:  src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/params/velocity_report.yaml
Vehicle info:        src/param/autoware_individual_params/.../vehicle_info.param.yaml
```

### Common test commands

```bash
# Record test session
ros2 bag record -o control_test_session /vehicle/status/velocity_status /control/command/control_cmd /vehicle/command/actuation_cmd

# Playback and analyze
ros2 bag play control_test_session.db3

# Check topics publishing rate
ros2 topic hz /vehicle/status/velocity_status
ros2 topic hz /control/command/control_cmd
```

## Advanced Testing

### Record Data for Analysis

```bash
# Record test session
ros2 bag record -o control_test_session \
  /vehicle/status/velocity_status \
  /control/command/control_cmd \
  /vehicle/command/actuation_cmd

# Playback and analyze
ros2 bag play control_test_session.db3
```

### Profile Control Performance

```bash
# Use control_test package for metrics
ros2 run control_test performance_analysis

# Outputs:
# - Rise time (0-90% of target velocity)
# - Settling time (within ±5% of target)
# - Overshoot percentage
# - Steady-state error
```

### Custom Test Trajectories

**Note**: For safety, custom trajectory tests should be implemented in the `control_test` package rather than ad-hoc scripts.

The `control_test` package includes:
- `velocity_tracking.launch.py` - Ramp velocity profiles
- `steering_response.launch.py` - Steering angle sweeps
- `step_response.launch.py` - Step input tests for PID tuning

**To add custom tests**: Modify the `control_test` package and use proper safety checks (watchdog timers, velocity limits, etc.).

## Next Steps

- **Understand control details**: [Control Details](./control-details.md)
- **Review hardware**: [Hardware](./hardware.md)
- **Back to overview**: [Overview](./overview.md)
