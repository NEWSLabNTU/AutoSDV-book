<!--
Translation Metadata:
- Source file: mrm.md
- Last synced: 2026-09-21
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 最小風險操作

當 Autoware 判斷自己已無法安全行駛時，它並不只是停止發佈控制命令，而是執行一次
**最小風險操作（minimum risk manoeuvre, MRM）**——一次有參數、有意圖的停車，並
亮起危險警示燈。MRM 在每一次 AutoSDV 啟動時都是啟用的，而多數人第一次戶外測試時
遇到的那個沒人要求的急煞，就是它。

本頁說明什麼會觸發 MRM、如何觀察它、車輛實際上會做什麼，以及——最常讓人踩坑的
部分——決定這些行為的參數究竟放在哪裡。

## MRM 是這台車四種停車機制之一

它們是觸發條件各不相同的獨立機制，而分辨它們是診斷任何不明停車的第一步。

| 機制 | 由誰決定 | 對什麼作出反應 |
|---|---|---|
| **MRM** | `mrm_handler`，依據系統診斷圖 | 整個堆疊已不適合行駛：主題中斷、模組失效、定位遺失 |
| **AEB** | control container 內的 `autonomous_emergency_braking` | 預測路徑上的障礙物 |
| `vehicle_cmd_gate` 緊急處理 | 命令閘門 | 命令路徑上的緊急旗標或過期心跳 |
| 致動器看門狗 | AutoSDV 自己的車輛介面 | 整整一秒沒有收到任何控制命令——見[控制細節](./vehicle-control/control-details.md) |

只有第一項是 MRM。其餘三項在別處說明，本頁不重複。

## 觸發鏈

沒有任何東西直接偵測「危險」。MRM 是由*自駕模式是否仍然可用*驅動的，而那是從一棵
診斷樹計算出來的布林值：

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

有兩個推論值得記住：

- **任何一個分支都能讓車停下。** 一個停止發佈的感知節點，和定位失效一樣是 MRM 的
  觸發來源。當停車原因不明時，要讀整張圖，而不是只讀你懷疑的那一支。
- **停車命令走的是一般控制路徑。** MRM operator 發佈到
  `/system/emergency/control_cmd`，命令閘門再轉送出去，所以車輛介面看到的是一個
  正常的煞車命令。並沒有一條通往致動器的獨立線路。

診斷圖本身是一組 YAML 檔。在預設的 `pose_source` 下，用的是已安裝 Autoware 的版本：

```bash
ls /opt/autoware/1.5.0/share/autoware_launch/config/system/diagnostics/
```

使用 `pose_source:=mcl` 時，launch 會換成 AutoSDV 的變體
`src/launcher/autosdv_launch/config/system/diagnostics/autosdv-mcl-main.yaml`，
因為原廠的圖要求一份 PCD 地圖與一個 `ndt_scan_matcher`，而 `mcl` 刻意兩者都沒有。
這個切換是自動的；見 `autosdv_autoware.launch.xml`。

## 觀察它

MRM 狀態是發佈出來的，而不是畫出來的。**RViz 和 `just tool tui` 都不會顯示它**
——AutoSDV 的設定檔過去索求的那個 RViz overlay 外掛在 Autoware 1.5.0 中並不存在，
所以那個顯示在這台機器上從來沒有運作過（`docs/known-config-defects.md` 第 1 項）。
請直接看主題。

```bash
# the state machine
ros2 topic echo /system/fail_safe/mrm_state

# why it is in that state: which modes are available
ros2 topic echo /system/operation_mode/availability

# which diagnostic actually failed
ros2 topic echo /diagnostics_agg
```

如果你偏好圖形介面，同一份資料也有 GUI：

```bash
ros2 run rqt_robot_monitor rqt_robot_monitor
```

!!! note "這些指令需要執行中的堆疊"

    本頁的每一個指令都在讀取一個運行中的 ROS 圖。它們是對照
    `/opt/autoware/1.5.0` 中的訊息定義與 launch 定義驗證的，並未在行駛中的車輛上
    實際執行過。

## 狀態

`/system/fail_safe/mrm_state` 的型別是 `autoware_adapi_v1_msgs/msg/MrmState`，
含兩個數值欄位。`ros2 topic echo` 印出的是數字，對照如下：

| `state` | 意義 |
|---|---|
| `1` `NORMAL` | 自駕模式可用，沒有任何動作 |
| `2` `MRM_OPERATING` | 正在執行一次操作 |
| `3` `MRM_SUCCEEDED` | 操作已完成，車輛已停止 |
| `4` `MRM_FAILED` | 操作無法完成 |

| `behavior` | 意義 |
|---|---|
| `1` `NONE` | 未選定任何操作 |
| `2` `EMERGENCY_STOP` | 急煞 |
| `3` `COMFORTABLE_STOP` | 緩和減速 |
| `4` `PULL_OVER` | 離開車道並停車 |

**在 `NORMAL` 與 `MRM_OPERATING` 之間快速來回震盪、危險警示燈隨之閃爍，是症狀而
不是一種模式。** 它代表某個診斷正在 ERROR 與正常之間跳動——幾乎總是某個主題間歇
性延遲，而不是真的中斷。去找出那個主題；不要去調 MRM 的參數。

## 車輛實際上會做什麼

共有三種行為。處理器可以從哪些行為中選擇，是由設定決定的：

| 行為 | 此處是否啟用 | 減速度 |
|---|---|---|
| 靠邊停車 | 否（`use_pull_over: false`） | — |
| 緩和停止 | **是**（`use_comfortable_stop: true`） | `min_acceleration: -1.0` m/s²，jerk 限制在 ±0.3 m/s³ |
| 緊急停止 | 永遠可用 | `target_acceleration: -2.5` m/s²，`target_jerk: -1.5` m/s³ |

緊急情況下會亮起危險警示燈（`turning_hazard_on.emergency: true`）。
`use_emergency_holding` 為 `false`，所以處理器並未被設定成在原因排除後仍無限期
鎖住緊急狀態；`timeout_emergency_recovery` 是 `5.0` 秒。關於精確的恢復語意，
Autoware 的 [MRM handler
文件](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/system/)
才是依據——本頁不會複述它無法從設定檔讀出來的行為。

## 參數究竟放在哪裡

這就是那個坑。**本儲存庫裡的 MRM 設定檔並沒有被載入。**
`src/launcher/autosdv_launch/config/system/mrm_handler/`、
`mrm_emergency_stop_operator/` 與 `mrm_comfortable_stop_operator/` 確實存在，但
launch 檔傳入的是*已安裝的 Autoware* 的路徑，而且是寫死的值，沒有任何 launch
參數可以覆寫：

```bash
grep -n mrm_ src/launcher/autosdv_launch/launch/components/autosdv_system_component.launch.xml
```

其中每一行都是 `$(find-pkg-share autoware_launch)/…`，而 `src/` 中沒有任何套件
覆蓋 `autoware_launch`。所以實際生效的檔案是：

```
/opt/autoware/1.5.0/share/autoware_launch/config/system/mrm_handler/mrm_handler.param.yaml
/opt/autoware/1.5.0/share/autoware_launch/config/system/mrm_emergency_stop_operator/mrm_emergency_stop_operator.param.yaml
/opt/autoware/1.5.0/share/autoware_launch/config/system/mrm_comfortable_stop_operator/mrm_comfortable_stop_operator.param.yaml
```

儲存庫裡的副本不只是沒被用到——它們的值還*不一致*，這才是讓編輯它們變成一種誤導的
原因：

| 參數 | 實際生效 | 儲存庫副本（未生效） |
|---|---|---|
| `use_comfortable_stop` | `true` | `false` |
| `target_acceleration` | `-2.5` m/s² | `-3.0` m/s² |
| `target_jerk` | `-1.5` m/s³ | `-3.0` m/s³ |

一位操作員若修改儲存庫裡的副本來減緩煞車力道，實車上卻仍看到 −2.5 m/s²，會下
結論說這個參數沒有作用。參數是有作用的；只是那個檔案沒有。

!!! warning "今天要改 MRM 煞車力道，等於要去改已安裝的 Autoware"

    這些路徑沒有對應的 launch 參數，而 `/opt/autoware/1.5.0` 屬於 root。去改它會
    破壞這個安裝之所以有價值的性質——`/opt/autoware/<version>` 是未經修改的上游
    Autoware，因此在那裡發現的缺陷是可以回報的。要更動 MRM 煞車力道，應視為一項
    必須先讓 launch 檔學會轉送該路徑的變更。

## 定位精度檢查是開著的

本儲存庫中較舊的筆記聲稱 AutoSDV 停用了定位誤差橢圓檢查，以避免誤觸緊急停止。
**但樹狀原始碼並不支持這個說法。** 一份把該檢查註解掉的
`src/launcher/autosdv_launch/config/system/diagnostics/localization.yaml` 確實
存在，但沒有任何 launch 路徑會選到包含它的那張圖：預設會解析到已安裝的
`autoware-main.yaml`，而 `pose_source:=mcl` 會解析到 `autosdv-mcl-main.yaml`，
後者引入的是 `localization-mcl.yaml`——而那個檔案*保留*了精度檢查。在每一種
pose source 下，`/autoware/localization/accuracy` 都是圖中一個有效的分支。

因此實際適用的閾值，來自
`/opt/autoware/1.5.0/share/autoware_localization_error_monitor/config/localization_error_monitor.param.yaml`：

| 參數 | 值 |
|---|---|
| `error_ellipse_size` | 1.5 m |
| `warn_ellipse_size` | 1.2 m |
| `error_ellipse_size_lateral_direction` | 0.3 m |
| `warn_ellipse_size_lateral_direction` | 0.25 m |

定位不確定度一旦越過這些值，自駕模式就會變成不可用，接著就是一次 MRM。在怪罪其他
東西之前，先在行駛中盯著這個橢圓：

```bash
ros2 topic echo /localization/localization_error_monitor/debug/ellipse_marker
```

!!! note "在 `mcl` 之下完全沒有估測器健康檢查"

    `localization-mcl.yaml` 拿掉了 `/autoware/localization/scan_matching_status`，
    因為它來自 `ndt_scan_matcher`，而該節點在 MCL 下不會運行，粒子濾波器也沒有可
    替代的 ROS 診斷可發佈。頻率檢查、精度檢查與 EKF 狀態都仍然適用，但一個正在
    無聲發散的粒子濾波器不會在這裡提出任何診斷。這是一個已知的缺口，記錄在該設定
    檔本身之中。

## 診斷一次停車

`play_launch` 會把各節點的紀錄寫到 `play_log/latest/node/` 之下，答案通常就在那裡。
從處理器開始，再往外找。

```bash
# what the handler decided, and when
grep -E "MRM State|EMERGENCY" play_log/latest/node/mrm_handler/err

# every topic monitor that reported a timeout
for m in play_log/latest/node/topic_state_monitor_*/err; do
  echo "== $(basename $(dirname $m))"
  grep -E "timeout|ERROR" "$m" | tail -5
done
```

這台車上的停車，多數出於三個原因。

**融合後的位姿逾時。** `topic_state_monitor_pose_twist_fusion_filter_pose` 會記下
`topic is timeout. Set ERROR in diagnostics`，而 MRM 開始震盪。位姿並不是錯的，
而是*太晚*——通常是 NDT 耗時過長，或 CPU 已經飽和。它會連鎖：軌跡跟隨器回報追蹤
誤差過大，規劃驗證器拒絕該軌跡，於是在單一根因的下游產生大量令人不安的紀錄輸出。

```bash
ros2 topic hz /localization/pose_twist_fusion_filter/pose   # expect ~50 Hz
```

**沒有路徑。** 你一要求進入自駕模式，MRM 立刻觸發，而
`topic_state_monitor_mission_planning_route` 說路徑 `has not received`。這是在
路徑存在之前就請求了自駕模式。先設定初始位姿，等定位收斂，設定目標點，等軌跡產生，
*然後*才切換。

**沒有控制命令。** `/control/command/control_cmd` 沒有訊息，因為規劃沒有產生軌跡，
或軌跡的起點離車太遠，跟隨器不接受。

```bash
ros2 topic hz /planning/scenario_planning/trajectory
ros2 topic hz /control/command/control_cmd
```

儲存庫中的 `docs/guides/mrm_troubleshooting.md` 收錄了更長的紀錄檔逐步分析，包含
每種失效各自會印出的確切訊息。

## 關閉 MRM

沒有單獨關閉 MRM 的開關。最接近的做法是停用整個 system 元件：

```bash
play_launch launch autosdv_launch autosdv.launch.yaml launch_system:=false
```

```bash
just launch "launch_system:=false"
```

這會拿掉系統監控、元件狀態監控、重複節點檢查器——以及診斷圖聚合器本身，而後者正是
發佈 `/system/operation_mode/availability` 的節點。這是用來單獨跑感知或感測的實驗
台設定，不是讓行駛中的車輛不再煞車的辦法。

## 相關

- [車輛控制：總覽](./vehicle-control/overview.md)——車端看門狗與致動器自身的停車行為
- [定位方法](./localization-methods.md)——`pose_source`，以及 `mcl` 會如何改變診斷圖
- [在車上運行](../running/on-the-vehicle.md)——`launch_system` 與其他子系統開關
- 儲存庫中的 `docs/guides/mrm_troubleshooting.md` 與 `docs/known-config-defects.md`
- Autoware 的[失效安全設計](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/fail-safe/)
