<!--
Translation Metadata:
- Source file: overview.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 車輛控制總覽

車輛控制系統架構及與 Autoware 整合的總覽。

## 系統架構

車輛控制系統橋接 Autoware 的高階規劃與低階致動器控制。

**資料流**：
```
Autoware Control → Control Commands → Actuator Node → PCA9685 (I2C) → Motor/Steering
                                           ↓
                        Hall Effect Sensor → Velocity Report Node → Velocity Status
                                           ↓
                        IMU Sensor → Yaw Rate Feedback (for steering)
```

### 軟體元件

**Autoware 控制堆疊** (`src/universe/autoware.universe/control/`)：
```
trajectory_follower_node       # Computes control commands from trajectory
       ↓
vehicle_cmd_gate               # Safety gate
       ↓
/control/command/control_cmd   # Control command topic
```

**車輛介面** (`src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/`)：
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

## 元件角色

### Actuator Node

將 Autoware 控制指令轉換為電機與轉向的 PWM 訊號。

**職責**：
- 訂閱來自 Autoware 的控制指令
- 實作多模式縱向控制器（速度控制）
- 實作雙模式橫向控制器（轉向控制）
- 透過 I2C 發送 PWM 訊號至 PCA9685
- 監控速度與 IMU 回饋
- 透過看門狗計時器確保安全運作

### Velocity Report Node

使用霍爾效應感測器測量車輛速度。

**職責**：
- 從 GPIO 讀取霍爾效應感測器脈衝
- 根據輪胎旋轉計算速度
- 發布速度報告以進行回饋控制
- 提供速度資料給 Autoware 堆疊

### PCA9685 PWM Driver

ESC 和伺服馬達的硬體介面。

**職責**：
- 從 actuator.py 接收 I2C 指令
- 產生 60 Hz 的 PWM 訊號
- 驅動電機 ESC（通道 0）與轉向伺服馬達（通道 1）

## 主題架構

### 關鍵主題

| 主題                              | 類型                      | 說明                        | 頻率  |
|-----------------------------------|---------------------------|-----------------------------|-------|
| `/control/command/control_cmd`    | `AckermannControlCommand` | 油門、煞車、轉向指令        | 10 Hz |
| `/vehicle/status/velocity_status` | `VelocityReport`          | 來自感測器的當前速度        | 20 Hz |
| `/vehicle/status/control_mode`    | `ControlModeReport`       | 手動/自動模式               | 1 Hz  |
| `/vehicle/command/actuation_cmd`  | Custom                    | 發送至 PCA9685 的 PWM 值    | 10 Hz |
| `/sensing/imu/imu_data`           | `Imu`                     | 用於偏航率回饋的 IMU 資料   | 100 Hz|

### 訂閱主題 (Actuator Node)

- `~/input/control_cmd` - 來自 Autoware 的控制指令
- `~/input/velocity_status` - 當前車輛速度（用於速度控制回饋）
- `~/input/imu` - 用於偏航率回饋的 IMU 資料（轉向控制）

### 發布主題 (Actuator Node)

- `~/debug/control_values` - 除錯控制值
- `~/debug/pwm_values` - 除錯 PWM 輸出
- `~/debug/pid_values` - 除錯 PID 狀態

## Autoware 整合

### 控制指令流程

1. **規劃**：Autoware 的規劃堆疊產生目標軌跡
2. **控制**：`trajectory_follower_node` 將軌跡轉換為控制指令
3. **安全閘門**：`vehicle_cmd_gate` 執行安全檢查與模式切換
4. **車輛介面**：Actuator 節點將指令轉換為 PWM
5. **硬體**：PCA9685 驅動電機 ESC 與轉向伺服馬達
6. **回饋**：速度與 IMU 資料回流至控制器

### 控制模式

**MANUAL (0)**：RC 遙控器或鍵盤控制
- 來自外部來源的控制指令
- 透過控制器 GUI 直接控制 PWM

**AUTO (1)**：Autoware 自動駕駛控制
- 來自軌跡追蹤器的控制指令
- 由 vehicle_cmd_gate 強制執行安全限制

**切換**：
```bash
# Switch to AUTO mode
ros2 service call /vehicle/set_control_mode autoware_auto_vehicle_msgs/srv/ControlModeCommand "{mode: 1}"

# Switch to MANUAL mode
ros2 service call /vehicle/set_control_mode autoware_auto_vehicle_msgs/srv/ControlModeCommand "{mode: 0}"
```

## 安全功能

### 控制模式閘門

- 強制執行速度限制（0-3 m/s）
- 強制執行轉向角度限制（±0.5 rad）
- 強制執行加速度限制（±2 m/s²）
- 拒絕超出安全範圍的指令

### 看門狗計時器

**Actuator Node** 包含看門狗計時器：

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

**額外逾時機制**：車輛介面監控速度回饋
- 若超過 2 秒無回饋 → 發布 STOP 指令
- 確保即使通訊故障車輛仍能停止

### 緊急停止

**觸發**：
```bash
# Publish zero velocity
ros2 topic pub /control/command/control_cmd autoware_auto_control_msgs/AckermannControlCommand "{
  longitudinal: {speed: 0.0, acceleration: -5.0}
}" -1

# Or switch to MANUAL mode
ros2 service call /vehicle/set_control_mode ... "{mode: 0}"
```

**行為**：
- 套用最大煞車（PWM 340）
- 轉向保持在當前角度
- 控制模式切換至 MANUAL

## 硬體摘要

| 元件                | 介面              | 詳細資訊                          |
|---------------------|-------------------|-----------------------------------|
| PCA9685             | I2C (0x40, bus 1) | 16 通道 PWM，60 Hz                |
| 電機 ESC            | PWM (channel 0)   | 範圍：280-460，初始值：370        |
| 轉向伺服馬達        | PWM (channel 1)   | 範圍：350-450，初始值：400        |
| 霍爾效應感測器      | GPIO              | KY-003，速度測量                  |
| IMU                 | ROS topic         | 用於轉向的偏航率回饋              |

詳細硬體規格請參閱[硬體](./hardware.md)。

## 配置檔案

- **Actuator 參數**：`src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/params/actuator.yaml`
- **Velocity report 參數**：`src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/params/velocity_report.yaml`
- **車輛資訊**：`src/param/autoware_individual_params/.../vehicle_info.param.yaml`

## 下一步

- **了解硬體**：[硬體詳情](./hardware.md)
- **學習控制演算法**：[控制詳情](./control-details.md)
- **測試與調校**：[調校與測試](./tuning-and-testing.md)
