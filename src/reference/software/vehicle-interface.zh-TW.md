<!--
Translation Metadata:
- Source file: vehicle-interface.md
- Last synced: 2026-09-21
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 車輛介面

車輛介面銜接 Autoware 的控制與車輛致動器。它由位於
`src/vehicle/autosdv_vehicle_launch` 的 `autosdv_vehicle_launch` 儲存庫提供，
包含以下套件。

- `autosdv_vehicle_description`

  提供車輛外觀參數。

- `autosdv_vehicle_launch`

  提供一個啟動檔，執行驅動車輛所需的節點。

- `autosdv_vehicle_interface`

  此套件提供將 Autoware 控制命令轉換為馬達動力的節點，並提供供定速控制使用的
  車輛狀態回報節點。

若要單獨啟動車輛介面：

```sh
source install/setup.bash
play_launch launch autosdv_vehicle_launch vehicle_interface.launch.xml
```

`play_launch` 是本專案用來取代 `ros2 launch` 的啟動協調器；參閱
[操作車輛](../../running/on-the-vehicle.md#為什麼用-play_launch-而不是-ros2-launch)。

一般情況下這個介面會作為整個系統的一部分啟動，並以 `launch_vehicle:=false`
關閉，而不是單獨啟動。

## 速度回報節點

此節點實作於 `velocity_report.py`。它週期性讀取霍爾效應感測器，並在每個週期內
計算嵌在輪子上的磁鐵標記數量。如此便可量得輪子的轉速，再乘上輪半徑即可算出
瞬時速度。

## 致動器節點

實作於 `actuator.py` 的節點會讀取目標速度，並控制馬達動力以達到該速度。它使用
PID 控制器計算 PWM 值，並施加於直流馬達上。

## 轉向狀態節點

此節點實作於 `steering_status.py`。它以 30 Hz 發佈
`/vehicle/status/steering_status`（一則 `SteeringReport`），而 Autoware 其餘部分
就是靠它來得知前輪指向何處。

!!! warning "`steering_status` 是命令，不是量測"

    **這台車沒有轉向角感測器。** 該節點訂閱
    `/control/command/control_cmd`，把命令的輪胎角度夾限在 `actuator.yaml` 的
    `max_steering_angle` 之內，套上一階延遲以模仿伺服機反應，然後把結果發佈
    出去。這條路徑上沒有任何東西在觀測輪子。

    作為缺席感測器的替代品，這是個說得過去的選擇。問題在於誰相信了它。

三個後果，而且每一個都咬過本專案：

- **橫向控制器從來沒有過回授。**
  `tier4_control_launch` 把 `~/input/current_steering` 重新對應到這個主題，
  於是 `autoware_mpc_lateral_controller` 把這個回音當成自己的控制器狀態。
  橫向迴路因此是以自己的輸出閉合的：伺服延遲、死區、飽和與連桿間隙對 MPC 全然
  不可見，而它把致動器視為理想。如果你在調校橫向增益，你是在對著一個不可能出錯
  的模型調校——所以在控制器自己的狀態裡看起來穩定的增益，在車上未必穩定。
- **啟用檢查不可能失敗。**
  `autoware_operation_mode_transition_manager` 以命令與狀態是否一致來把關啟用。
  面對一個回音，兩者不可能不一致，於是這項檢查無條件通過。
- **在任何未啟用自駕時錄下的 bag 裡，它都恰好讀到 0**，因為根本沒有控制命令可以
  回音。不論錄多久，任何人工駕駛的錄製都不能用來驗證轉向幾何或橫向控制器。

想在跑動中的車上親眼確認，就比對這兩個主題——回報值就是延遲後的命令：

```sh
ros2 topic echo /control/command/control_cmd --field lateral.steering_tire_angle
ros2 topic echo /vehicle/status/steering_status --field steering_tire_angle
```

真正的量測其實可以由車上既有的感測器取得：運動學自行車模型可由橫擺率與速度
推出輪胎角度。它並未被接上，而且也不該草率接上——它在低速時未定義、會繼承輪速
的比例誤差，而且會經由 IMU 閉合一條迴路。相關量測、選項與其限制詳見
[`docs/reports/steering-status-has-no-feedback.md`](https://github.com/NEWSLabNTU/AutoSDV/blob/main/docs/reports/steering-status-has-no-feedback.md)。
