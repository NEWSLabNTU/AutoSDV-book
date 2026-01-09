<!--
Translation Metadata:
- Source file: control-details.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 控制詳情

控制演算法與 PWM 實作的技術細節。

## 重要說明

車輛介面實作**低階控制**，將 Autoware 的控制指令轉換為 PWM 訊號。它**不**實作軌跡追蹤（由 Autoware 的 `trajectory_follower_node` 處理）。

## 縱向控制（速度控制）

### 演算法：多模式 PID 控制器

Actuator 節點實作一個精密的**四模式控制器**，可適應不同的駕駛條件：

```
Control Command → Mode Selection → Mode-Specific Logic → PWM Output
```

### 模式 1：緊急煞車模式

**觸發條件**：
- 目標速度 ≈ 0（在 `full_stop_threshold` 範圍內）
- 測量速度 > `brake_threshold`

**行為**：
- 輸出：`BRAKE_PWM` (340)
- 目的：接到停止指令時快速減速

**範例**：
```
Target: 0.0 m/s
Measured: 0.5 m/s
→ Emergency Brake Mode
→ PWM = 340 (brake)
```

### 模式 2：完全停止模式

**觸發條件**：
- 目標速度 ≈ 0（在 `full_stop_threshold` 範圍內）
- 測量速度 ≈ 0（在 `full_stop_threshold` 範圍內）

**行為**：
- 輸出：`INIT_PWM` (370)
- 目的：維持停止狀態，防止漂移

**範例**：
```
Target: 0.0 m/s
Measured: 0.05 m/s
→ Full Stop Mode
→ PWM = 370 (neutral)
```

### 模式 3：死區保持模式

**觸發條件**：
- 速度誤差 < `velocity_deadband`

**行為**：
- 保持最後的 PWM 值
- 目的：防止因小速度誤差造成的抖動

**範例**：
```
Target: 1.0 m/s
Measured: 1.03 m/s
Error: 0.03 m/s (< deadband of 0.05 m/s)
→ Deadband Hold Mode
→ PWM = [last PWM value]
```

### 模式 4：主動控制模式

**觸發條件**：
- 所有其他情況（正常駕駛）

**演算法**：具有低通濾波與防積分飽和的 PID 控制器

**詳細步驟**：

1. **目標速度的低通濾波**：
   ```
   filtered_target = α × target + (1-α) × filtered_target_prev
   where α = velocity_command_filter_alpha (0.5)
   ```

2. **測量速度的低通濾波**：
   ```
   filtered_measured = α × measured + (1-α) × filtered_measured_prev
   where α = velocity_measurement_filter_alpha (0.3)
   ```

3. **計算速度誤差**：
   ```
   error = filtered_target - filtered_measured
   ```

4. **PID 控制**：
   ```
   P = Kp × error

   I = I_prev + Ki × error × dt
   I = clamp(I, -integral_limit, +integral_limit)  # Anti-windup

   D = -Kd × (filtered_measured - filtered_measured_prev) / dt

   pwm_offset = P + I + D
   ```

5. **條件積分**（若啟用）：
   ```
   if output is saturated (at MIN_PWM or MAX_PWM):
       stop integrating (I remains constant)
   else:
       continue integrating normally
   ```

6. **方向特定的 PWM 映射**：
   ```
   if in_reverse:
       pwm = INIT_PWM - pwm_offset
   else:
       pwm = INIT_PWM + pwm_offset
   ```

7. **輸出 PWM 的低通濾波**：
   ```
   filtered_pwm = 0.25 × pwm + 0.75 × filtered_pwm_prev
   ```

8. **限制在硬體範圍內**：
   ```
   final_pwm = clamp(filtered_pwm, MIN_PWM, MAX_PWM)
   final_pwm = clamp(final_pwm, 280, 460)
   ```

### 參數（來自 `actuator.yaml`）

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

### 控制流程圖

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

## 橫向控制（轉向控制）

### 演算法：雙模式偏航率控制器

Actuator 節點實作**雙模式控制器**，根據車輛速度切換：

```
Vehicle speed → Mode Selection → Mode-Specific Logic → PWM Output
```

### 後備模式 (v < 0.3 m/s)

**時機**：低速或靜止時

**原因**：低速時偏航率測量不可靠

**演算法**：開迴路前饋控制

**步驟**：

1. **將轉向角限制在範圍內**：
   ```
   clamped_angle = clamp(steering_angle, -max_steering_angle, +max_steering_angle)
   clamped_angle = clamp(clamped_angle, -0.349, +0.349)  # ±20°
   ```

2. **直接角度到 PWM 的映射**：
   ```
   pwm = INIT_STEER + clamped_angle × tire_angle_to_steer_ratio
   pwm = 400 + clamped_angle × 143.24
   ```

3. **限制 PWM 在範圍內**：
   ```
   final_pwm = clamp(pwm, MIN_STEER, MAX_STEER)
   final_pwm = clamp(pwm, 350, 450)
   ```

**範例**：
```
Speed: 0.1 m/s (< 0.3 m/s)
Steering angle: 0.2 rad
→ Fallback Mode
→ PWM = 400 + 0.2 × 143.24 = 428.65 ≈ 429
```

**特性**：
- 簡單且可預測
- 不需要回饋
- 快速回應
- 適合停車與低速操作

### 正常模式 (v ≥ 0.3 m/s)

**時機**：正常駕駛速度

**演算法**：具有前饋的閉迴路偏航率回饋控制

**步驟**：

1. **將轉向角限制在範圍內**：
   ```
   clamped_angle = clamp(steering_angle, -max_steering_angle, +max_steering_angle)
   ```

2. **Ackermann 運動學模型**（目標偏航率）：
   ```
   target_yaw_rate = (vehicle_speed / wheelbase) × tan(clamped_angle)
   where wheelbase ≈ vehicle length
   ```

3. **目標偏航率的低通濾波**：
   ```
   filtered_target = α × target_yaw_rate + (1-α) × filtered_target_prev
   where α = 0.3
   ```

4. **測量偏航率的低通濾波**（來自 IMU）：
   ```
   filtered_measured = α × measured_yaw_rate + (1-α) × filtered_measured_prev
   where α = 0.2
   ```

5. **計算偏航率誤差**：
   ```
   error = filtered_target - filtered_measured
   ```

6. **PID 控制器**：
   ```
   P = Kp × error

   I = I_prev + Ki × error × dt
   I = clamp(I, -integral_limit, +integral_limit)  # Anti-windup

   D = -Kd × (filtered_measured - filtered_measured_prev) / dt

   pwm_correction = P + I + D
   ```

7. **前饋項**：
   ```
   pwm_ff = INIT_STEER + clamped_angle × tire_angle_to_steer_ratio
   ```

8. **結合前饋 + 回饋**：
   ```
   pwm = pwm_ff + pwm_correction
   ```

9. **限制 PWM 在範圍內**：
   ```
   final_pwm = clamp(pwm, MIN_STEER, MAX_STEER)
   final_pwm = clamp(pwm, 350, 450)
   ```

**範例**：
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

**偏航率回饋的優點**：
- **干擾抑制**：補償輪胎滑移、風力、路面傾斜
- **改善追蹤**：實際偏航率匹配期望偏航率
- **更快回應**：回饋加速收斂
- **強健性**：對輪胎/路面變化較不敏感

### 參數（來自 `actuator.yaml`）

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

### 控制流程圖

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

## 速度計算

### 霍爾效應感測器方法

Velocity report 節點根據霍爾效應感測器脈衝計算速度。

**演算法**：

1. **計算時間段內的脈衝數**：
   ```python
   delta_pulses = current_pulses - previous_pulses
   delta_time = current_time - previous_time
   ```

2. **計算輪胎旋轉數**：
   ```python
   rotations = delta_pulses / markers_per_rotation
   ```

3. **計算移動距離**：
   ```python
   distance = rotations × π × wheel_diameter
   ```

4. **計算速度**：
   ```python
   velocity = distance / delta_time  # m/s
   ```

**參數**（來自 `velocity_report.yaml`）：

```yaml
gpio_pin: 17                       # GPIO pin number (BCM numbering)
wheel_diameter: 0.1                # meters (measure your wheel)
markers_per_rotation: 4            # Number of magnets on wheel
publication_rate: 20.0             # Hz
```

**計算範例**：
```
Wheel diameter: 0.1 m
Markers per rotation: 4
Time period: 0.1 s
Pulses counted: 2

Rotations: 2 / 4 = 0.5 rotations
Distance: 0.5 × π × 0.1 = 0.157 m
Velocity: 0.157 / 0.1 = 1.57 m/s
```

## I2C 通訊協定

### PCA9685 介面

**Python 驅動程式**（`actuator.py`）：

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

**PWM 指令結構**：

- **通道**：0-15（我們使用 0 給電機，1 給轉向）
- **ON tick**：何時將訊號轉為 ON（通常為 0）
- **OFF tick**：何時將訊號轉為 OFF（這設定脈衝寬度）
- **頻率**：60 Hz（20ms 週期）
- **解析度**：12 位元（4096 步進）

**脈衝寬度計算**：

```
Pulse width (μs) = (off_tick / 4096) × (1000000 / frequency)
Pulse width (μs) = (off_tick / 4096) × (1000000 / 60)
Pulse width (μs) = off_tick × 4.07

Example:
PWM value 400 → Pulse width = 400 × 4.07 = 1628 μs
```

**通訊流程**：

```
actuator.py → I2C command → PCA9685 → PWM signal → ESC/Servo
     ↑                                                   ↓
     └──────── Velocity feedback ←──── Hall sensor ← Wheel rotation
```

## 除錯主題

Actuator 節點發布除錯資訊：

**主題**：
- `~/debug/control_values` - 控制指令值
- `~/debug/pwm_values` - PWM 輸出值
- `~/debug/pid_values` - PID 狀態（P、I、D 項）

**監控**：
```bash
# Monitor PWM values
ros2 topic echo /debug/pwm_values

# Monitor PID state
ros2 topic echo /debug/pid_values
```

## 下一步

- **測試系統**：[調校與測試](./tuning-and-testing.md)
- **了解硬體**：[硬體](./hardware.md)
- **返回總覽**：[總覽](./overview.md)
