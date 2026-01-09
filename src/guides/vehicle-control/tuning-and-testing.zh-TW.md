<!--
Translation Metadata:
- Source file: tuning-and-testing.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 調校與測試

測試、調校與校正車輛控制系統的實用指南。

## 快速測試程序

### 先決條件

- 車輛已上電
- PCA9685 PWM 板已連接（I2C 介面）
- ROS 2 環境已來源（`source install/setup.bash`）

### 1. 啟動基本控制

```bash
# Launch minimal control system (no planning, no localization)
make play-basic-control
```

這會啟動：
- 車輛介面（透過 I2C 的 PCA9685 PWM 通訊）
- 控制節點（橫向 + 縱向控制）
- 基本訂閱者/發布者

**預期輸出**：
```
[vehicle_interface_node]: Connected to PCA9685 on I2C bus
[trajectory_follower_node]: Lateral controller started
[longitudinal_controller_node]: Longitudinal controller started
```

### 2. 啟動 PlotJuggler

在新終端機中：
```bash
# Launch PlotJuggler for real-time visualization
make plot-test
```

**要監控的主題**：
- `/vehicle/status/control_mode` - 控制模式（手動/自動）
- `/vehicle/status/velocity_status` - 當前速度
- `/control/command/control_cmd` - 控制指令（油門、煞車、轉向）
- `/control/current_gate_mode` - 控制閘門模式

**PlotJuggler 設定**：
1. 視窗自動開啟
2. 從左側面板選擇主題
3. 拖曳主題至繪圖區域
4. 觀察即時資料

### 3. 驗證系統狀態

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

## 手動控制測試

使用控制器 GUI 測試手動控制（安全方法）。

### 啟動控制器 GUI

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

**為什麼要使用控制器 GUI？**
- 安全：放開按鍵時指令停止
- 防止失控：GUI 關閉時車輛停止
- 視覺回饋：查看當前控制狀態

**驗證**：
- 車輛立即回應指令
- 放開 A/D 時轉向回中
- 煞車（Space）平順停止車輛
- 無意外移動

**⚠️ 警告**：請勿使用 `ros2 topic pub` 直接發送控制指令。Actuator 會記住最後的指令，車輛會持續移動。務必使用控制器 GUI。

## 自動化測試情境

`control_test` 套件提供自動化測試情境，可安全驗證控制系統效能。

### 測試 1：速度追蹤

**目標**：驗證速度控制器追蹤目標速度。

```bash
# Terminal 1: Launch basic control
make play-basic-control

# Terminal 2: Launch PlotJuggler
make plot-test

# Terminal 3: Run velocity tracking test
ros2 launch control_test velocity_tracking.launch.py
```

此測試會自動：
- 將速度從 0 斜升至 2 m/s
- 保持在 2 m/s 五秒
- 斜降至 0

**PlotJuggler 中的預期結果**：平順的速度曲線匹配指令軌跡，穩定在 ±0.2 m/s 內。

### 測試 2：轉向回應

**目標**：驗證轉向伺服馬達回應指令。

```bash
# Terminal 1: Basic control running
# Terminal 2: PlotJuggler running

# Terminal 3: Run steering response test
ros2 launch control_test steering_response.launch.py
```

此測試循環執行：
- 中心（0°）→ 左轉（15°）→ 中心 → 右轉（15°）→ 中心

**PlotJuggler 中的預期結果**：轉向角在 100ms 內追蹤指令角度。

### 測試 3：緊急停止

**目標**：驗證煞車回應與安全性。

```bash
# Use controller GUI for this test
make run-controller

# Steps:
# 1. Press W to accelerate to ~1 m/s
# 2. Press Space (brake) to stop
# 3. Observe stopping distance
```

**預期**：車輛在 1 公尺內平順停止。

**在 PlotJuggler 中監控**：減速應平順，無抖動。

## PID 調校

### 縱向控制器（速度控制）

**測試程序**：
1. 啟動基本控制：`make play-basic-control`
2. 啟動 PlotJuggler：`make plot-test`
3. 執行階躍回應測試：`ros2 launch control_test step_response.launch.py`
4. 在 PlotJuggler 中觀察速度回應

**⚠️ 安全**：使用 `control_test` 套件進行 PID 調校，不要使用手動 `ros2 topic pub` 指令。測試套件包含安全逾時與看門狗機制。

**調校效果**：

**Kp（比例）**：
- **過高**：超調、震盪
- **過低**：回應慢、穩態誤差大
- **典型範圍**：30-70（預設：50.0）

**Ki（積分）**：
- **目的**：消除穩態誤差
- **過高**：積分飽和、超調
- **過低**：與目標永久偏移
- **典型範圍**：2-10（預設：5.0）

**Kd（微分）**：
- **目的**：減少超調、阻尼震盪
- **過高**：放大雜訊、運動抖動
- **過低**：超調
- **典型範圍**：0.5-5（預設：2.0）

**調校過程**：
1. 僅從 Kp 開始（Ki=0、Kd=0）
2. 增加 Kp 直到回應快速但略微震盪
3. 加入 Ki 以消除穩態誤差
4. 加入 Kd 以減少超調

### 橫向控制器（轉向控制）

**要調校的參數**：
- `kp_steer`：偏航率控制的比例增益
- `ki_steer`：積分增益
- `kd_steer`：微分增益

**效果與縱向控制器類似**：
- 較高的 Kp → 更快的轉向回應，但可能震盪
- Ki → 消除穩態偏航率誤差
- Kd → 阻尼震盪

**預設值**：
- `kp_steer: 10.0`
- `ki_steer: 1.0`
- `kd_steer: 0.5`

### 如何編輯參數

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

## 校正程序

### 校正 PWM 值

**目標**：為您的特定電機/ESC 確定 PWM 到速度的映射。

**⚠️ 安全警告**：PWM 校正需要仔細測試。Actuator 會記住最後的指令，因此務必準備好緊急停止方法。

**建議方法**：使用 `control_test` 套件的校正模式：

```bash
# Launch PWM calibration tool (with safety features)
ros2 launch control_test pwm_calibration.launch.py
```

此工具：
- 逐步測試 PWM 值並確認
- 包含自動逾時/看門狗
- 記錄每個 PWM 值的速度
- 產生校正參數

**替代方案（進階）**：使用控制器 GUI 並監控：

```bash
# Terminal 1: Launch vehicle interface
ros2 run autosdv_vehicle_interface vehicle_interface_node

# Terminal 2: Monitor velocity
ros2 topic echo /vehicle/status/velocity_status

# Terminal 3: Use controller GUI
make run-controller
# Gradually increase throttle, record velocity at each level
```

**更新參數**：
```yaml
# File: params/actuator.yaml
min_pwm: 280           # Minimum PWM (maximum reverse)
init_pwm: 370          # Neutral/dead zone
max_pwm: 460           # Maximum PWM (maximum forward)
brake_pwm: 340         # Emergency brake PWM
```

### 校正轉向

**目標**：測量轉向角與 PWM 的關係。

使用控制器 GUI 測量轉向角與 PWM：

```bash
# Terminal 1: Launch vehicle interface
ros2 run autosdv_vehicle_interface vehicle_interface_node

# Terminal 2: Launch controller GUI
make run-controller

# Terminal 3: Monitor steering commands
ros2 topic echo /vehicle/command/actuation_cmd
```

**程序**：
1. 逐漸按下 A（左轉）→ 觀察轉向 PWM 並測量輪胎角度
2. 轉向回中 → 驗證 PWM 回到約 400，輪胎在 0°
3. 逐漸按下 D（右轉）→ 觀察轉向 PWM 並測量輪胎角度
4. 記錄最大左轉角度（PWM 約 350）與最大右轉角度（PWM 約 450）
5. 計算：`angle_per_pwm = (max_angle - min_angle) / (450 - 350)`

**安全**：控制器 GUI 會在放開按鍵時自動將轉向回中。

### 校正霍爾效應感測器

**目標**：驗證速度測量準確度。

使用控制器 GUI 測量感測器準確度：

```bash
# Terminal 1: Launch vehicle interface
ros2 run autosdv_vehicle_interface velocity_report_node

# Terminal 2: Monitor velocity feedback
ros2 topic echo /vehicle/status/velocity_status

# Terminal 3: Controller GUI
make run-controller
```

**程序**：
1. 在地面標記起始位置
2. 使用控制器 GUI（按 W）讓車輛慢慢前進
3. 在正好 1 公尺標記處停止（按 Space 煞車）
4. 記錄總脈衝數（若監控原始感測器）或驗證速度符合預期
5. 計算：`pulses_per_meter = total_pulses / 1.0`

**更新**：
```yaml
# File: params/velocity_report.yaml
wheel_diameter: 0.1            # Measured in meters
markers_per_rotation: 4        # Number of magnets on wheel
gpio_pin: 17                   # GPIO pin number
```

## 理解圖表

### 速度圖

```
Target velocity (commanded)   ----
Actual velocity (measured)    ____

Good tracking:
  Target and actual overlap closely

Poor tracking:
  Large gap between target and actual
  Oscillations around target
```

**要注意的事項**：
- **上升時間**：速度達到目標的速度（應 < 2 秒）
- **超調**：速度不應超過目標 >10%
- **穩態誤差**：最終誤差應 < 0.1 m/s
- **震盪**：應在 1-2 個週期內穩定

### 控制指令圖

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

**要注意的事項**：
- **平順轉換**：指令應斜升，不應跳躍
- **無飽和**：指令應保持在限制內（不會持續在 MAX/MIN）
- **相關性**：速度誤差應與油門/煞車指令相關

## 疑難排解

### 車輛無回應

**症狀**：指令已發布但車輛不移動。

**診斷**：
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

**解決方案**：
- 檢查 PCA9685 的 I2C 配線（SDA、SCL、VCC、GND）
- 將使用者加入 i2c 群組：`sudo usermod -a -G i2c $USER`
- 切換至 AUTO 模式：`ros2 service call /vehicle/set_control_mode autoware_auto_vehicle_msgs/srv/ControlModeCommand "{mode: 1}"`
- 重新啟動車輛介面：`ros2 run autosdv_vehicle_interface vehicle_interface_node`

### 速度震盪

**症狀**：車輛速度在目標值附近震盪。

**診斷**：
- 在 PlotJuggler 中繪製速度圖
- 檢查控制指令是否震盪
- 監控霍爾效應感測器是否有雜訊

**解決方案**：
1. **降低 PID Kp**：減少比例增益
2. **增加 PID Kd**：加入阻尼
3. **濾波感測器**：增加 `velocity_measurement_filter_alpha`（0.3 → 0.5）
4. **檢查機械**：鬆動的感測器固定座會造成雜訊

### 轉向延遲或抖動

**症狀**：轉向回應延遲或不穩定。

**診斷**：
```bash
# Monitor steering commands and servo response
ros2 topic echo /control/command/control_cmd
ros2 topic echo /vehicle/command/actuation_cmd
```

**解決方案**：
1. **電源供應**：檢查伺服馬達獲得乾淨的 5V 且電流充足（>1A）
2. **PWM 範圍**：驗證 PWM 值為 350-450
3. **機械卡住**：檢查轉向連桿移動順暢
4. **PCA9685 測試**：使用 `control_test` 校正模式獨立測試伺服馬達

### 霍爾效應感測器無作用

**症狀**：速度總是讀取為 0。

**診斷**：
```bash
# Monitor velocity feedback
ros2 topic echo /vehicle/status/velocity_status

# Check if velocity updates when vehicle moves
# (Use controller GUI to drive vehicle slowly)

# Check velocity_report node is running
ros2 node list | grep velocity_report
```

**解決方案**：
1. **配線**：檢查感測器電源（VCC、GND）與訊號連接
2. **磁鐵對齊**：確保輪胎上的磁鐵靠近感測器（< 5mm 間隙）
3. **GPIO 腳位**：驗證 `velocity_report.yaml` 中配置的 GPIO 腳位正確
4. **上拉電阻**：KY-003 可能需要在訊號腳位上使用上拉電阻
5. **LED 指示燈**：KY-003 有 LED 在偵測到磁鐵時會閃爍 - 檢查輪胎旋轉時是否閃爍

## 快速參考

### 啟動指令
```bash
make play-basic-control        # Launch control system
make plot-test                 # Launch PlotJuggler
make run-controller            # Controller GUI (safe manual control)
```

### 監控主題
```bash
ros2 topic echo /vehicle/status/control_mode
ros2 topic echo /vehicle/status/velocity_status
ros2 topic echo /control/command/control_cmd
ros2 topic echo /vehicle/command/actuation_cmd
ros2 topic hz /vehicle/status/velocity_status
```

### PCA9685 連接
```bash
sudo i2cdetect -y 1            # Check I2C device at 0x40
groups $USER | grep i2c        # Verify I2C permissions
```

### 控制模式
```bash
# AUTO mode
ros2 service call /vehicle/set_control_mode autoware_auto_vehicle_msgs/srv/ControlModeCommand "{mode: 1}"

# MANUAL mode
ros2 service call /vehicle/set_control_mode autoware_auto_vehicle_msgs/srv/ControlModeCommand "{mode: 0}"
```

### 參數檔案
```
Actuator control:    src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/params/actuator.yaml
Velocity reporting:  src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/params/velocity_report.yaml
Vehicle info:        src/param/autoware_individual_params/.../vehicle_info.param.yaml
```

### 常見測試指令

```bash
# Record test session
ros2 bag record -o control_test_session /vehicle/status/velocity_status /control/command/control_cmd /vehicle/command/actuation_cmd

# Playback and analyze
ros2 bag play control_test_session.db3

# Check topics publishing rate
ros2 topic hz /vehicle/status/velocity_status
ros2 topic hz /control/command/control_cmd
```

## 進階測試

### 記錄資料以供分析

```bash
# Record test session
ros2 bag record -o control_test_session \
  /vehicle/status/velocity_status \
  /control/command/control_cmd \
  /vehicle/command/actuation_cmd

# Playback and analyze
ros2 bag play control_test_session.db3
```

### 分析控制效能

```bash
# Use control_test package for metrics
ros2 run control_test performance_analysis

# Outputs:
# - Rise time (0-90% of target velocity)
# - Settling time (within ±5% of target)
# - Overshoot percentage
# - Steady-state error
```

### 自訂測試軌跡

**注意**：為了安全，自訂軌跡測試應在 `control_test` 套件中實作，而非臨時腳本。

`control_test` 套件包含：
- `velocity_tracking.launch.py` - 斜升速度剖面
- `steering_response.launch.py` - 轉向角掃描
- `step_response.launch.py` - PID 調校的階躍輸入測試

**加入自訂測試**：修改 `control_test` 套件並使用適當的安全檢查（看門狗計時器、速度限制等）。

## 下一步

- **了解控制詳情**：[控制詳情](./control-details.md)
- **檢視硬體**：[硬體](./hardware.md)
- **返回總覽**：[總覽](./overview.md)
