# Control Details

Technical details of the control algorithms and PWM implementation.

## Important Note

The vehicle interface implements **low-level control** that converts Autoware's control commands to PWM signals. It does **NOT** implement trajectory following (that's handled by Autoware's `trajectory_follower_node`).

## Longitudinal Control (Speed Control)

### Algorithm: Multi-Mode PID Controller

The actuator node implements a sophisticated **4-mode controller** that adapts to different driving conditions:

```
Control Command → Mode Selection → Mode-Specific Logic → PWM Output
```

### Mode 1: Emergency Brake Mode

**Trigger Conditions**:
- Target velocity ≈ 0 (within `full_stop_threshold`)
- Measured velocity > `brake_threshold`

**Behavior**:
- Output: `BRAKE_PWM` (340)
- Purpose: Rapid deceleration when commanded to stop

**Example**:
```
Target: 0.0 m/s
Measured: 0.5 m/s
→ Emergency Brake Mode
→ PWM = 340 (brake)
```

### Mode 2: Full Stop Mode

**Trigger Conditions**:
- Target velocity ≈ 0 (within `full_stop_threshold`)
- Measured velocity ≈ 0 (within `full_stop_threshold`)

**Behavior**:
- Output: `INIT_PWM` (370)
- Purpose: Maintain stopped state, prevent drift

**Example**:
```
Target: 0.0 m/s
Measured: 0.05 m/s
→ Full Stop Mode
→ PWM = 370 (neutral)
```

### Mode 3: Deadband Hold Mode

**Trigger Conditions**:
- Velocity error < `velocity_deadband`

**Behavior**:
- Hold last PWM value
- Purpose: Prevent jitter from small velocity errors

**Example**:
```
Target: 1.0 m/s
Measured: 1.03 m/s
Error: 0.03 m/s (< deadband of 0.05 m/s)
→ Deadband Hold Mode
→ PWM = [last PWM value]
```

### Mode 4: Active Control Mode

**Trigger Conditions**:
- All other cases (normal driving)

**Algorithm**: PID controller with low-pass filtering and anti-windup

**Detailed Steps**:

1. **Low-pass filter on target velocity**:
   ```
   filtered_target = α × target + (1-α) × filtered_target_prev
   where α = velocity_command_filter_alpha (0.5)
   ```

2. **Low-pass filter on measured velocity**:
   ```
   filtered_measured = α × measured + (1-α) × filtered_measured_prev
   where α = velocity_measurement_filter_alpha (0.3)
   ```

3. **Calculate velocity error**:
   ```
   error = filtered_target - filtered_measured
   ```

4. **PID control**:
   ```
   P = Kp × error

   I = I_prev + Ki × error × dt
   I = clamp(I, -integral_limit, +integral_limit)  # Anti-windup

   D = -Kd × (filtered_measured - filtered_measured_prev) / dt

   pwm_offset = P + I + D
   ```

5. **Conditional integration** (if enabled):
   ```
   if output is saturated (at MIN_PWM or MAX_PWM):
       stop integrating (I remains constant)
   else:
       continue integrating normally
   ```

6. **Direction-specific PWM mapping**:
   ```
   if in_reverse:
       pwm = INIT_PWM - pwm_offset
   else:
       pwm = INIT_PWM + pwm_offset
   ```

7. **Low-pass filter on output PWM**:
   ```
   filtered_pwm = 0.25 × pwm + 0.75 × filtered_pwm_prev
   ```

8. **Clamp to hardware limits**:
   ```
   final_pwm = clamp(filtered_pwm, MIN_PWM, MAX_PWM)
   final_pwm = clamp(final_pwm, 280, 460)
   ```

### Parameters (from `actuator.yaml`)

```yaml
# PID gains
kp_speed: 50.0                              # Proportional gain
ki_speed: 5.0                               # Integral gain
kd_speed: 2.0                               # Derivative gain

# Anti-windup
integral_limit: 50.0                        # Maximum integral value
enable_conditional_integration: true        # Stop integration when saturated

# Mode thresholds
velocity_deadband: 0.05                     # m/s - deadband hold threshold
full_stop_threshold: 0.1                    # m/s - full stop detection
brake_threshold: 0.2                        # m/s - emergency brake activation

# Filtering
velocity_measurement_filter_alpha: 0.3      # Low-pass filter for measured velocity
velocity_command_filter_alpha: 0.5          # Low-pass filter for target velocity

# PWM limits
min_pwm: 280                                # Minimum PWM (max reverse)
init_pwm: 370                               # Neutral/dead zone
max_pwm: 460                                # Maximum PWM (max forward)
brake_pwm: 340                              # Emergency brake PWM
```

### Control Flow Diagram

```
Target velocity → Filter →
                            ↓
                        Calculate error ← Filter ← Measured velocity
                            ↓
                        Mode check?
                            ↓
        ┌───────────────────┼───────────────────┐
        ↓                   ↓                   ↓
   Emergency Brake     Full Stop          Deadband Hold    Active Control
   (PWM = 340)        (PWM = 370)         (PWM = last)     (PID)
        ↓                   ↓                   ↓              ↓
        └───────────────────┴───────────────────┴──────────────┘
                                  ↓
                            Output PWM
```

## Lateral Control (Steering Control)

### Algorithm: Dual-Mode Yaw Rate Controller

The actuator node implements a **2-mode controller** that switches based on vehicle speed:

```
Vehicle speed → Mode Selection → Mode-Specific Logic → PWM Output
```

### Fallback Mode (v < 0.3 m/s)

**When**: Low speed or standstill

**Why**: Yaw rate measurements are unreliable at low speeds

**Algorithm**: Open-loop feedforward control

**Steps**:

1. **Clamp steering angle to limits**:
   ```
   clamped_angle = clamp(steering_angle, -max_steering_angle, +max_steering_angle)
   clamped_angle = clamp(clamped_angle, -0.349, +0.349)  # ±20°
   ```

2. **Direct angle-to-PWM mapping**:
   ```
   pwm = INIT_STEER + clamped_angle × tire_angle_to_steer_ratio
   pwm = 400 + clamped_angle × 143.24
   ```

3. **Clamp PWM to limits**:
   ```
   final_pwm = clamp(pwm, MIN_STEER, MAX_STEER)
   final_pwm = clamp(pwm, 350, 450)
   ```

**Example**:
```
Speed: 0.1 m/s (< 0.3 m/s)
Steering angle: 0.2 rad
→ Fallback Mode
→ PWM = 400 + 0.2 × 143.24 = 428.65 ≈ 429
```

**Characteristics**:
- Simple and predictable
- No feedback required
- Fast response
- Adequate for parking and low-speed maneuvers

### Normal Mode (v ≥ 0.3 m/s)

**When**: Normal driving speeds

**Algorithm**: Closed-loop yaw rate feedback control with feedforward

**Steps**:

1. **Clamp steering angle to limits**:
   ```
   clamped_angle = clamp(steering_angle, -max_steering_angle, +max_steering_angle)
   ```

2. **Ackermann kinematic model** (target yaw rate):
   ```
   target_yaw_rate = (vehicle_speed / wheelbase) × tan(clamped_angle)
   where wheelbase ≈ vehicle length
   ```

3. **Low-pass filter on target yaw rate**:
   ```
   filtered_target = α × target_yaw_rate + (1-α) × filtered_target_prev
   where α = 0.3
   ```

4. **Low-pass filter on measured yaw rate** (from IMU):
   ```
   filtered_measured = α × measured_yaw_rate + (1-α) × filtered_measured_prev
   where α = 0.2
   ```

5. **Calculate yaw rate error**:
   ```
   error = filtered_target - filtered_measured
   ```

6. **PID controller**:
   ```
   P = Kp × error

   I = I_prev + Ki × error × dt
   I = clamp(I, -integral_limit, +integral_limit)  # Anti-windup

   D = -Kd × (filtered_measured - filtered_measured_prev) / dt

   pwm_correction = P + I + D
   ```

7. **Feedforward term**:
   ```
   pwm_ff = INIT_STEER + clamped_angle × tire_angle_to_steer_ratio
   ```

8. **Combine feedforward + feedback**:
   ```
   pwm = pwm_ff + pwm_correction
   ```

9. **Clamp PWM to limits**:
   ```
   final_pwm = clamp(pwm, MIN_STEER, MAX_STEER)
   final_pwm = clamp(pwm, 350, 450)
   ```

**Example**:
```
Speed: 1.5 m/s (≥ 0.3 m/s)
Steering angle: 0.2 rad
Target yaw rate: (1.5 / 0.5) × tan(0.2) ≈ 0.6 rad/s
Measured yaw rate (IMU): 0.5 rad/s
Error: 0.1 rad/s
→ Normal Mode
→ PID correction: +5 PWM units
→ Feedforward: 400 + 0.2 × 143.24 = 429
→ Final PWM: 429 + 5 = 434
```

**Benefits of yaw rate feedback**:
- **Disturbance rejection**: Compensates for tire slip, wind, road camber
- **Improved tracking**: Actual yaw rate matches desired yaw rate
- **Faster response**: Feedback accelerates settling
- **Robustness**: Less sensitive to tire/road variations

### Parameters (from `actuator.yaml`)

```yaml
# PID gains
kp_steer: 10.0                              # Proportional gain
ki_steer: 1.0                               # Integral gain
kd_steer: 0.5                               # Derivative gain

# Physical limits
max_steering_angle: 0.349                   # rad ≈ 20° (from vehicle_info.param.yaml)
tire_angle_to_steer_ratio: 143.24           # Conversion from radians to PWM units
steering_speed: 0.5                         # Maximum steering rate (rad/s)

# PWM limits
min_steer: 350                              # Minimum PWM (max left)
init_steer: 400                             # Center/straight
max_steer: 450                              # Maximum PWM (max right)
```

### Control Flow Diagram

```
Steering angle → Clamp →
                          ↓
                   Speed check (v < 0.3?)
                          ↓
        ┌─────────────────┴─────────────────┐
        ↓                                   ↓
   Fallback Mode                      Normal Mode
   (Feedforward only)                 (Feedforward + Feedback)
        │                                   │
        │                              Ackermann model
        │                                   ↓
        │                              Target yaw rate → Filter
        │                                   ↓
        │                              Error ← Filter ← IMU yaw rate
        │                                   ↓
        │                              PID controller
        │                                   ↓
        │                              Correction
        │                                   ↓
   PWM_ff ← Angle mapping            PWM = PWM_ff + Correction
        ↓                                   ↓
        └───────────────────┬───────────────┘
                            ↓
                       Clamp to limits
                            ↓
                       Output PWM
```

## Velocity Calculation

### Hall Effect Sensor Method

The velocity report node calculates velocity from hall effect sensor pulses.

**Algorithm**:

1. **Count pulses over time period**:
   ```python
   delta_pulses = current_pulses - previous_pulses
   delta_time = current_time - previous_time
   ```

2. **Calculate wheel rotations**:
   ```python
   rotations = delta_pulses / markers_per_rotation
   ```

3. **Calculate distance traveled**:
   ```python
   distance = rotations × π × wheel_diameter
   ```

4. **Calculate velocity**:
   ```python
   velocity = distance / delta_time  # m/s
   ```

**Parameters** (from `velocity_report.yaml`):

```yaml
gpio_pin: 17                       # GPIO pin number (BCM numbering)
wheel_diameter: 0.1                # meters (measure your wheel)
markers_per_rotation: 4            # Number of magnets on wheel
publication_rate: 20.0             # Hz
```

**Example Calculation**:
```
Wheel diameter: 0.1 m
Markers per rotation: 4
Time period: 0.1 s
Pulses counted: 2

Rotations: 2 / 4 = 0.5 rotations
Distance: 0.5 × π × 0.1 = 0.157 m
Velocity: 0.157 / 0.1 = 1.57 m/s
```

## I2C Communication Protocol

### PCA9685 Interface

**Python Driver** (`actuator.py`):

```python
from Adafruit_PCA9685 import PCA9685

# Initialize PCA9685
pwm = PCA9685(address=0x40, busnum=1)
pwm.set_pwm_freq(60)  # 60 Hz for RC servos/ESC

# Set motor PWM (channel 0)
# Arguments: channel, on_tick, off_tick
# For standard PWM: on_tick=0, off_tick=PWM value
pwm.set_pwm(0, 0, motor_pwm)

# Set steering PWM (channel 1)
pwm.set_pwm(1, 0, steering_pwm)
```

**PWM Command Structure**:

- **Channel**: 0-15 (we use 0 for motor, 1 for steering)
- **ON tick**: When to turn signal ON (usually 0)
- **OFF tick**: When to turn signal OFF (this sets pulse width)
- **Frequency**: 60 Hz (20ms period)
- **Resolution**: 12-bit (4096 steps)

**Pulse Width Calculation**:

```
Pulse width (μs) = (off_tick / 4096) × (1000000 / frequency)
Pulse width (μs) = (off_tick / 4096) × (1000000 / 60)
Pulse width (μs) = off_tick × 4.07

Example:
PWM value 400 → Pulse width = 400 × 4.07 = 1628 μs
```

**Communication Flow**:

```
actuator.py → I2C command → PCA9685 → PWM signal → ESC/Servo
     ↑                                                   ↓
     └──────── Velocity feedback ←──── Hall sensor ← Wheel rotation
```

## Debug Topics

The actuator node publishes debug information:

**Topics**:
- `~/debug/control_values` - Control command values
- `~/debug/pwm_values` - PWM output values
- `~/debug/pid_values` - PID state (P, I, D terms)

**Monitoring**:
```bash
# Monitor PWM values
ros2 topic echo /debug/pwm_values

# Monitor PID state
ros2 topic echo /debug/pid_values
```

## Next Steps

- **Test the system**: [Tuning & Testing](./tuning-and-testing.md)
- **Understand hardware**: [Hardware](./hardware.md)
- **Back to overview**: [Overview](./overview.md)
