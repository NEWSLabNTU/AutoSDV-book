<!--
Translation Metadata:
- Source file: vehicle-interface.md
- Last synced: 2026-09-12
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
