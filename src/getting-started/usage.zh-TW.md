<!--
Translation Metadata:
- Source file: usage.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 操作車輛

本頁教你**自己**啟動 AutoSDV，而不是執行某一條替你啟動它的指令。這個差別很重要：
你將來會想改的每一件事——用哪顆光達、用哪種定位方法、感知模組要不要跑——都是一個
**啟動參數**，而參數是你自己傳的，不是包裝腳本替你傳的。

開始之前，請先完成[軟體安裝](./installation/overview.md)並建置工作空間。

## 啟動指令

```bash
source install/setup.bash
play_launch launch autosdv_launch autosdv.launch.yaml
```

三個位置，值得逐一指名：

| 位置 | 值 | 意義 |
|------|-----|------|
| 工具 | `play_launch launch` | 啟動協調器 |
| 套件 | `autosdv_launch` | 一個已安裝的 ROS 2 套件 |
| 啟動檔 | `autosdv.launch.yaml` | 該套件 `launch/` 目錄中的一個檔案 |

`autosdv.launch.yaml` 宣告了整個駕駛系統：車輛介面、感測、定位、感知、規劃、
控制，以及在它們各種變體之間做選擇的參數。

## 傳遞參數

參數接在啟動檔之後，形式為 `name:=value`——注意是冒號等號，不是單純的 `=`：

```bash
play_launch launch autosdv_launch autosdv.launch.yaml pose_source:=ndt
```

可以一次傳多個，順序不拘：

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  sensor_suite:=robin_zed \
  pose_source:=ndt \
  launch_perception:=false
```

四條規則涵蓋幾乎所有會犯的錯：

- **每個值都是字串。** 沒有型別化的參數；`launch_perception` 接受的是*文字*
  `true` 或 `false`。
- **布林值是小寫**的 `true` / `false`。`True` 與 `1` 不被接受。
- **空值是有意義的。** 有幾個感測器參數預設為 `""`，意思是「採用感測器組合選定的
  值」。明確傳入 `lidar_model:=""` 與完全不傳是一樣的。
- **含空白或 shell 特殊字元的值要加引號**，因為你的 shell 會在 ROS 之前先解析
  這一行。

### 確認你的參數到底起了什麼作用

拼錯的參數不一定會報錯——它可能只是變成另一個沒有人讀的參數。若要看到啟動解析
後的結果，包含它將啟動的每個節點與參數：

```bash
play_launch resolve autosdv_launch autosdv.launch.yaml \
  pose_source:=mcl -o ./tmp/resolved.yaml
```

這能明確回答「我的參數有沒有生效」，而且不花任何代價——它不會啟動任何節點。

## 為什麼用 `play_launch` 而不是 `ros2 launch`

`play_launch launch` 是 `ros2 launch` 的直接替代品，接受相同的套件、檔案與參數。
請使用它。它額外提供：

- **它會正確地關閉系統。** AutoSDV 的多數節點是以可組合節點的形式跑在容器行程
  內。殺掉 `ros2 launch` 只會殺掉父行程，留下那些容器成為孤兒行程，繼續佔著
  GPU、繼續發佈訊息。`play_launch` 會對整個行程群組逐級升高
  SIGINT → SIGTERM → SIGKILL。
- **一個網頁介面**，預設在 `http://127.0.0.1:8080`，列出每個節點的狀態與紀錄。
- **資源監控**——每個行程的 CPU、記憶體、I/O 與 GPU——以及 `/diagnostics` 主題。
- **解析／重放工作流程**：`play_launch dump launch …` 解析一次並寫出系統模型，
  `play_launch up system_model.yaml` 之後可重複啟動而不必重新解析。

它從 PyPI 安裝，而設定程式會替你裝好：

```bash
pip install play_launch
play_launch setcap      # 選用：每行程 I/O 監控、非 root 的即時排程
```

### 如果你手上只有 `ros2 launch`

同樣的一行也可以用：

```bash
ros2 launch autosdv_launch autosdv.launch.yaml pose_source:=ndt
```

但**不要只按 `Ctrl-C` 就假設系統已經結束**，也絕對不要對它的 PID 下 `kill -9`
——那正是造成容器變成孤兒的原因。請殺掉整個行程群組：

```bash
ros2 launch autosdv_launch autosdv.launch.yaml &
LAUNCH_PID=$!
# ... 測試中 ...
kill -- -$(ps -o pgid= -p $LAUNCH_PID | tr -d ' ')
```

事後要檢查有沒有孤兒行程，`ros2 node list` 應為空。

### `just launch` 捷徑

儲存庫提供一個方便的 recipe：

```bash
just launch
just launch "pose_source:=ndt launch_perception:=false"
```

它只是一層薄薄的包裝，而值得確切知道它加了什麼，因為這些附加內容從命令列上
看不出來：

```bash
play_launch launch --web-addr 0.0.0.0:8081 autosdv_launch autosdv.launch.yaml [ARGS]
# 且當 $DISPLAY 未設定時，另外加上：rviz:=false
```

三個後果：

- 網頁介面在**通訊埠 8081**，而非 play_launch 自身的預設 8080
- 它綁定 `0.0.0.0`，所以網路上的其他機器也連得到該介面
- 在沒有 X forwarding 的 SSH 下，RViz 會被自動關閉——方便，但也表示同一條指令
  會因環境不同而行為不同

所有參數要放進單一個加引號的 `ARGS=` 字串裡，這是 `just` 的要求，不是 ROS 的。

## 參數

預設值取自 `autosdv.launch.yaml`。這是完整清單；你實際上最常用到的在前三張表。

### 選擇感測器

| 參數 | 預設 | 意義 |
|------|------|------|
| `sensor_suite` | `vlp32c_zed_imu` | 預先定義的組合：`robin_zed`、`robin_zed_mpu`、`vlp32c_zed`、`vlp32c_zed_mpu`、`vlp32c_zed_imu`、`cube1_usb`、`custom` |
| `lidar_model` | *(組合)* | `cube1`、`robin-w`、`vlp32c` |
| `camera_model` | *(組合)* | `zedxm`、`usb`、`none` |
| `imu_source` | *(組合)* | `mpu9250`、`zed` |
| `gnss_receiver` | *(組合)* | `ublox`、`septentrio`、`garmin` |
| `use_gnss` | *(組合)* | 戶外運作時啟用 GNSS |
| `use_ntrip` | `true` | RTK 修正用的 NTRIP 用戶端（僅 u-blox） |
| `enable_zed_object_detection` | *(組合)* | ZED 相機物件偵測 |

*(組合)* 表示該參數預設為 `""`，由感測器組合提供實際值。明確設定某一項即可只
覆寫該感測器。

### 選擇定位方法

| 參數 | 預設 | 意義 |
|------|------|------|
| `pose_source` | `cuda_ndt` | `cuda_ndt`、`ndt`、`mcl`——參閱[定位方法](../guides/localization-methods.md) |
| `pose_source_package` | `auto` | 由 `pose_source` 推導而來。只有在要接入第三方估測器時才明確設定 |
| `localization_preset` | `default` | `default`（陀螺儀里程計）或 `eagleye`（GNSS 里程計） |
| `use_mapless_mode` | `false` | 室內運作，完全不做定位 |
| `map_path` | `./data/COSS-map-planning` | 地圖目錄 |
| `occupancy_grid_file` | `occupancy_grid.yaml` | `map_path` 內的佔據網格描述檔名（僅 `pose_source:=mcl`） |
| `mcl_random_seed` | `-1` | 粒子濾波器的亂數種子；`-1` 為不可重現（僅 `mcl`） |

### 選擇感知設定

| 參數 | 預設 | 意義 |
|------|------|------|
| `perception_preset` | `lidar_only` | `lidar_only`、`camera_lidar_fusion`、`minimal`——參閱[預設組態](../guides/presets.md) |
| `launch_perception` | `true` | 設為 `false` 時發佈空的物件清單，且不載入任何模型 |
| `perception_input_pointcloud` | `/sensing/lidar/concatenated/pointcloud` | 供地面／障礙物分割使用的點雲 |

### 點雲後端

| 參數 | 預設 | 意義 |
|------|------|------|
| `pointcloud_backend` | `cpu` | `cpu` 或 `cuda`：感測端前處理（自身裁切、去畸變、環狀離群濾除） |
| `localization_pointcloud_backend` | `cpu` | `cpu` 或 `cuda`：NDT 輸入鏈（裁切盒、體素網格、隨機降採樣） |
| `input_pointcloud` | `/sensing/lidar/concatenated/pointcloud` | 供定位鏈使用的點雲 |

兩個後端都是整段式開關，且都不能只套用一半。參閱
[CUDA 點雲管線](../guides/cuda-pipeline.md)。

### 關閉模組

以下每一項都預設為 `true`。適合用來隔離問題，也適合在無法支撐全部功能的機器上
執行。

| 參數 | 關閉的對象 |
|------|-----------|
| `launch_vehicle` | 車輛介面 |
| `launch_system` | 系統監控與 MRM |
| `launch_map` | 地圖載入 |
| `launch_sensing` | 感測 |
| `launch_sensing_driver` | 只關閉感測器驅動程式，保留感測的其餘部分 |
| `launch_localization` | 定位 |
| `launch_planning` | 規劃 |
| `launch_control` | 控制 |
| `launch_perception` | 感知 |
| `launch_system_monitor` | AutoSDV 系統監控（在 Jetson 上最多佔一個核心的 25%） |

### 其餘參數

| 參數 | 預設 | 意義 |
|------|------|------|
| `is_simulation` | `false` | 模擬模式：停止對硬體輸出 PWM |
| `use_sim_time` | `false` | 使用模擬時鐘 |
| `data_path` | `$AUTOSDV_DATA_PATH`，否則 `./data/autoware_data` | Autoware 模型目錄。**必須可寫入**，否則 TensorRT 無法快取引擎 |
| `vehicle_model` | `autosdv_vehicle` | 車輛描述套件前綴 |
| `sensor_model` | `autosdv_sensor_kit` | 感測器套件描述前綴 |
| `rviz_config` | AutoSDV 的版面 | 要載入的 RViz 版面 |


### 實例

室內，無 GNSS，無地圖：

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  use_gnss:=false use_mapless_mode:=true
```

Robin-W 搭配端到端的 CUDA 管線：

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  sensor_suite:=robin_zed \
  pointcloud_backend:=cuda \
  localization_pointcloud_backend:=cuda \
  pose_source:=cuda_ndt
```

相機與光達融合，含交通號誌辨識：

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  sensor_suite:=robin_zed \
  perception_preset:=camera_lidar_fusion
```

只跑定位，以便在沒有感知模組干擾的情況下除錯姿態問題：

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  launch_perception:=false launch_planning:=false launch_control:=false
```

## 或者直接修改啟動檔

你也可以直接在原始檔改預設值：

```
src/launcher/autosdv_launch/launch/autosdv.launch.yaml
```

由於工作空間以 `--symlink-install` 建置，對 `.yaml`、`.xml` 與 `.py` 檔案的修改
在下次啟動時即生效，不需重新建置。但**新增**檔案仍需 `just build` 來建立符號
連結。

任何你可能會想改回來的設定，都優先用參數。

## 日常指令

儲存庫的 `just` recipes 依模組分組。單獨執行 `just` 會列出全部；
`just <模組>` 只列出該模組。

```bash
just              # 全部 recipes
just tool         # 只列出此模組的 recipes
```

兩種寫法都可以：`just tool rviz` 與 `just tool::rviz`。

### 工具

```bash
just tool rviz          # 載入 AutoSDV 版面的 RViz
just tool plotjuggler   # PlotJuggler
just tool tui           # 行駛監看 TUI：姿態、速度、組件狀態
just tool controller    # 鍵盤手動控制
just tool zed           # 只啟動 ZED 相機節點，用於測試相機
```

### 控制測試

```bash
just control basic      # 車輛控制測試
just control straight   # 10 公尺直線軌跡
just control circle     # 圓形軌跡
```

### 錄製與回放

```bash
just bag record         # 錄製戶外感測器主題
just bag play           # 回放最近一次的錄製
just coss download-rosbag       # 取得測試 rosbag（約 2.8 GB）
```

### 模擬

```bash
just coss planning-sim       # 路徑規劃模擬器，不需感測器
just coss logging-sim        # rosbag 回放
just sim coss-park      # 完整的 COSS Park 情境
```

參閱[模擬指南](../tutorial/02-planning-simulation.md)。

### 地圖

```bash
just map check <map_dir> [pose_source]
just map grid-from-pcd <map_dir>
just map grid-from-bag <bag> <map_dir>
```

參閱[地圖](../guides/maps.md)。

## 停止系統

在 `play_launch` 的終端機按 `Ctrl-C`。它會對整個行程群組逐級升高訊號，因此
可組合節點會一起結束。

如果有東西殘留：

```bash
ros2 node list      # 應為空
```

紀錄寫在 `play_log/latest/`。

## 後續步驟

- [路徑規劃模擬](../tutorial/02-planning-simulation.md) —— 最值得先跑的東西，
  而且不需要任何感測器
- [定位方法](../guides/localization-methods.md) —— `pose_source` 在選擇什麼
- [預設組態](../guides/presets.md) —— `perception_preset` 與
  `localization_preset` 如何運作，以及如何新增一個
