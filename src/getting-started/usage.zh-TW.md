<!--
Translation Metadata:
- Source file: usage.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 操作車輛

在閱讀本文前，請確保您已完成[軟體安裝](./installation/overview.md)並建置專案。專案儲存庫有一個啟動檔案 `autosdv.launch.yaml`，定義了啟動整個駕駛系統所需執行的節點集合及其參數。

## 簡單的方式

Makefile 提供了啟動整個系統的指令。

```sh
make launch
```

## 客製化啟動

您可以直接修改啟動檔案，位於：

```
AutoSDV/src/launcher/autosdv_launch/launch/autosdv.launch.yaml
```

或在啟動指令中指定參數值。例如，使用 Isaac Visual SLAM 進行定位：

```sh
source install/setup.bash
ros2 launch autosdv_launch autosdv.launch.yaml pose_source:=isaac
```

或在沒有硬體的情況下以模擬模式執行：

```sh
ros2 launch autosdv_launch autosdv.launch.yaml is_simulation:=true
```

### 常用參數

| 參數                            | 說明                                                    | 預設值              |
|--------------------------------|--------------------------------------------------------|---------------------|
| `is_simulation`                | 啟用模擬模式（停用對硬體的 PWM 輸出）                      | `false`             |
| `sensor_suite`                 | 預定義感測器套件（robin_zed、vlp32c_zed_imu 等）         | `vlp32c_zed_imu`    |
| `lidar_model`                  | 光達型號（cube1、robin-w、vlp32c）                       | （來自套件）         |
| `camera_model`                 | 相機型號（zedxm、usb、none）                             | （來自套件）         |
| `imu_source`                   | IMU 來源（mpu9250、zed）                                 | （來自套件）         |
| `gnss_receiver`                | GNSS 接收器類型（ublox、septentrio、garmin）             | （來自套件）         |
| `use_gnss`                     | 啟用 GNSS 以進行戶外操作                                  | （來自套件）         |
| `use_ntrip`                    | 啟用 NTRIP 客戶端以獲取 RTK 修正                          | `true`              |
| `use_mapless_mode`             | 啟用無地圖模式以進行室內操作                               | `false`             |
| `pose_source`                  | 姿態估計來源（ndt、isaac）                                | `ndt`               |
| `enable_zed_object_detection`  | 啟用 ZED 相機物件偵測                                     | （來自套件）         |
| `launch_perception`            | 啟動感知模組（物件偵測）                                   | `true`              |

完整的參數列表請參閱[主要啟動檔案](https://github.com/NEWSLabNTU/AutoSDV/blob/main/src/launcher/autosdv_launch/launch/autosdv.launch.yaml)。

## 常用操作

### 自動駕駛

執行具有航點導航的自動駕駛：

```sh
make run-drive
```

此指令啟動完整系統，並根據 `scripts/testing/drive/poses.json` 中定義的姿態執行自動駕駛。車輛將沿定義的航點導航。

### 視覺化和監控

啟動 RViz 以進行 3D 視覺化：

```sh
make run-rviz
```

啟動 PlotJuggler 以進行即時資料繪圖：

```sh
make run-plotjuggler
```

### 手動控制

啟動基於鍵盤的手動控制：

```sh
make run-controller
```

### 測試控制系統

測試基本控制系統：

```sh
make play-basic-control
```

執行預定義軌跡：

```sh
make run-straight-10m  # Drive 10m straight
make run-circle        # Drive in a circle
```

### 錄製和回放

在戶外操作期間錄製感測器資料：

```sh
make record-outdoor
```

回放最近的錄製內容：

```sh
make play-outdoor
```
