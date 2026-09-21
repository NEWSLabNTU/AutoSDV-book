<!--
Translation Metadata:
- Source file: commands.md
- Last synced: 2026-09-21
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 指令參考

本書幾乎每一頁遲早都會用到某個 `just` recipe。這一頁就是那份清單：儲存庫提供了
哪些指令、每個 recipe 實際執行什麼，以及哪些寫法已經被取代。

這些 recipe 位於儲存庫根目錄的 `justfile`、`just/` 底下的模組檔案，以及
`demo/justfile`。單獨執行 `just` 會把全部印出來：

```bash
just              # every recipe, modules expanded
just --list       # the same list
just tool         # just one module's recipes
```

本頁一律先寫底層指令、後寫 recipe。這是刻意的：recipe 只是讓你不必把同一行指令
打四十次，而不是取代對那行指令的理解。出問題的時候，你要除錯的是那行指令。

## 各項 recipe 如何組織

每天都會用到的六個動詞留在根層級。其餘刻意進入的主題都歸到模組裡。

| 模組 | 涵蓋 |
|------|------|
| `bag` | rosbag 錄製與播放 |
| `control` | 控制系統測試啟動與預錄軌跡 |
| `coss` | COSS Park 場景：它的資料與兩種模擬 |
| `demo` | 端到端場景、執行報告、定位基準測試 |
| `map` | 地圖驗證與佔據網格建構 |
| `sim` | 完整的 COSS Park 場景，以及兩個已棄用的寫法 |
| `tool` | RViz、PlotJuggler、駕駛 TUI、手動控制、ZED |

兩種寫法都可以，意思相同：

```bash
just bag play
just bag::play
```

### 兩個值得知道的機制

當你自己新增模組時，這兩點會咬你一口；它們也解釋了這些 justfile 為什麼長成
這樣。

**模組不能和 recipe 同名。** `mod launch` 與 `launch:` recipe 並存並不是一個
遮蔽警告——它是一個直接讓整個 justfile 失效的硬錯誤，於是所有 recipe 都跑不了。
這是日常動詞留在根層級、沒有搬進模組的原因之一。

**`just <module>` 會執行該模組的*第一個* recipe。** 它並不是「列出模組」；列出
只是第一個 recipe 碰巧做的事。因此這裡每個模組檔案都以一個 private 的 `default`
開頭，它只執行 `just --list <module>`，不做別的。少了它，`just bag` 會直接開始
錄製。

## 根層級 recipes

| Recipe | 作用 |
|--------|------|
| `just build` | 建置 `src/` 中的每個套件 |
| `just test` | 執行測試並印出結果 |
| `just clean` | 詢問後刪除 `build/`、`install/` 與 `log/` |
| `just launch ARGS` | 啟動完整系統，Web UI 在 8081 埠 |
| `just checkout` | 初始化並更新每個 git 子模組 |
| `just setup` | 執行互動式安裝程式 |
| `just setup-autoware-data` | 把 Autoware 模型樹鏡射到可寫入的位置 |
| `just engines` | 若有相符的 TensorRT 引擎集就取回，否則建置 |
| `just build-engines` | 在首次啟動前先編譯 TensorRT 引擎 |
| `just export-engines` | 把這塊板子的引擎打包，供發佈使用 |

### `just build`

```bash
source /opt/ros/humble/setup.bash
colcon build \
    --base-paths src \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --cargo-args --release
```

`just build` 就是這行指令，前面再加上一個檢查：確認 colcon 需要的 Python 環境
完好。

如果你要手動建置單一套件，請把這些旗標全部帶上。`--base-paths src` 讓 colcon
不會跑進 `data/` 和 `docker/`；`--symlink-install` 則是讓 YAML 或啟動檔的修改
不必重新建置就生效的關鍵。

`--cargo-args --release` 是最容易漏掉、而漏掉代價最高的一個。
`CMAKE_BUILD_TYPE=Release` 只涵蓋 C++ 套件；少了 cargo 這個旗標，colcon-cargo
會以未最佳化的方式建置 `cuda_ndt_matcher`，於是 `pose_source:=cuda_ndt` 每幀掃描
大約要 80 ms，而不是 5 ms。

### `just test`

```bash
colcon test --base-paths src --return-code-on-test-failure
colcon test-result --verbose
```

這個 recipe 兩者都跑，並以測試的結果碼結束，因此可以放進腳本裡。

### `just clean`

移除 `build/`、`install/` 與 `log/`。它會先要求你輸入字面上的 `yes`；輸入其他
任何內容都不會動到工作區。

### `just launch`

```bash
play_launch launch --web-addr 0.0.0.0:8081 \
    autosdv_launch autosdv.launch.yaml [ARGS]
# and, when $DISPLAY is unset, also: rviz:=false
```

參數以單一個加引號的位置字串傳入：

```bash
just launch
just launch "pose_source:=ndt launch_perception:=false"
```

!!! warning "不要在前面寫 `ARGS=`"

    `just launch ARGS="pose_source:=ndt"` 看起來像是在設定 recipe 的參數，但在
    `just` 裡，recipe 名稱*之後*的 `NAME=value` 是一個位置值，不是指派。字面上的
    `ARGS=pose_source:=ndt` 會被交給 `play_launch`，而它無法使用，於是這次啟動
    會用預設值執行——而且不聲不響。這個 recipe 現在會偵測這種寫法，並以結束碼 2
    拒絕執行，而不是啟動錯誤的東西。

完整的參數清單在[操作車輛](../running/on-the-vehicle.md#參數)，
而 [`play_launch`](../tutorial/06-play-launch.md) 說明啟動器本身。

### `just checkout`

```bash
git submodule update --init --recursive --checkout
```

`--recursive` 很重要：有好幾個子模組自己又包含子模組。

### `just setup` 與 `just setup-autoware-data`

`just setup` 執行 `./setup.sh`，也就是互動式安裝程式。參閱
[軟體安裝](../getting-started/installation/overview.md)。

`just setup-autoware-data` 執行 `./scripts/setup_autoware_data.sh`，它以符號連結
把 Autoware 的模型樹鏡射到 `data/autoware_data`。Autoware 會把編譯好的 `.engine`
寫在它所依據的 `.onnx` 旁邊，而 `/opt/autoware/` 底下的套件樹屬於 root——因此少了
這一步，引擎一建好就被丟棄，每次啟動都要重新編譯。

### TensorRT 引擎

感知堆疊第一次啟動時，會**在節點的建構子裡**把每個 ONNX 模型編譯成 TensorRT
引擎；在 Orin 上這是每個模型數分鐘，期間感知完全不可用。下面三個 recipe 把它
變成一個佈建步驟。

```bash
just engines          # try the published cache for this board, then build
just build-engines    # build locally, always
just export-engines   # package what this board built, for a release
```

`just build-engines` 會編譯感知 preset 實際解析到的五個模型（CenterPoint tiny、
YOLOX 相機偵測器，以及 `camera_lidar_fusion` 額外加入的三個號誌燈模型），然後
印出實際產生了什麼。

引擎綁定**特定 GPU 與特定 TensorRT 版本**，所以這必須在目標板子上執行，而且在
Autoware 或 JetPack 升級之後必須重跑。它們無法被烘焙進別處建置的映像檔。

在 amd64 工作站上有個值得知道的陷阱：只要引擎記錄的 TensorRT 版本與 Autoware
自身函式庫編譯時所用的版本不同，Autoware 就會丟棄它；因此載入了不同修補版
TensorRT 的主機，不論你跑幾次這個 recipe，每次啟動都會重建全部引擎。
`just build-engines` 會比對這兩個版本並在結尾說明，而不是讓你稍後才發現。

`just engines` 是安裝程式所執行的那一個。它會尋找與這塊板子指紋相符的已發佈
引擎集，存在就下載並驗證，不存在就落到本機建置——因此在任何機器上執行都安全，
中途被中斷也安全。

## `bag` — 錄製與播放

```bash
just bag record    # record the outdoor sensor topics to rosbags/
just bag play      # play the most recent outdoor recording
```

`record` 執行 `./scripts/rosbag/record_outdoor.sh`。`play` 會找出最新的
`rosbags/outdoor_*` 目錄並執行：

```bash
ros2 bag play <that directory> --clock
```

`--clock` 不是裝飾。重播期間整個堆疊跑在模擬時間上，沒有時鐘來源就什麼都不會
前進——而且任何地方都不會有錯誤訊息。

`just bag download` 已**棄用**；它會印出指引並轉送到
`just coss download-rosbag`，後者明確說出它取回的是哪一份錄製。

## `control` — 控制系統測試

```bash
just control basic      # launch the vehicle control test
just control straight   # drive a 10 m straight trajectory
just control circle     # drive a circular trajectory
```

底層是：

```bash
play_launch launch control_test basic_control.launch.xml
ros2 run control_test trajectory_player --ros-args -p trajectory_file:=straight_10m.yaml
ros2 run control_test trajectory_player --ros-args -p trajectory_file:=circle.yaml
```

`basic` 把控制鏈啟動起來；兩個軌跡 recipe 餵給它一條預錄路徑。它們需要車輛，
至少需要車輛介面在跑。參閱
[調校與測試](../guides/vehicle-control/tuning-and-testing.md)。

## `coss` — COSS Park 場景

教學所建立於其上的場景：一張地圖、一份錄製、兩種模擬。

| Recipe | 作用 |
|--------|------|
| `just coss download-rosbag` | 取回行駛錄製（下載約 1.6 GB，解開 2.8 GB） |
| `just coss planning-sim` | 只跑規劃元件 |
| `just coss logging-sim [backend]` | 由錄製餵入的完整堆疊 |
| `just coss play-rosbag [rate]` | 把錄製播放給執行中的 `logging-sim` |
| `just coss demo` | 轉送到 `just demo run` |

其中的啟動 recipe 在執行之前，都會以暗色文字回顯它即將執行的 `play_launch`
指令。

**規劃模擬器**不需要感測器、不需要定位、不需要 GPU、也不需要 rosbag：

```bash
play_launch launch --web-addr 0.0.0.0:8081 \
    autoware_launch planning_simulator.launch.xml \
    map_path:="$PWD/data/COSS-map-planning" \
    vehicle_model:=autosdv_vehicle \
    sensor_model:=autosdv_sensor_kit
```

**記錄模擬需要兩個終端機**，而這是重點，不是不便：

```bash
# terminal 1 — the stack, waiting on a clock
just coss logging-sim

# terminal 2 — the recording, driving that clock
just coss play-rosbag
```

少了第二個，什麼都不會發生，也沒有任何東西會說明原因。

`logging-sim` 接受一個 backend 參數，預設為 `cpu`：

| Backend | 額外加上 |
|---------|----------|
| `cpu`（預設） | `pose_source:=ndt launch_perception:=false`——任何筆電都跑得動 |
| `gpu` | `pose_source:=cuda_ndt`，含感知 |

預設走 CPU 是刻意的：讓一台從未建過 TensorRT 引擎的機器，不必把最初半小時花在
編譯上。`play-rosbag` 接受播放倍率，預設 `1.0`；若錄製不存在，它會拒絕執行並
指向 `download-rosbag`。

參閱 [COSS Park 場景](../running/coss-park-scenario.md)與
[記錄模擬](../tutorial/03-logging-simulation.md)。

## `demo` — 端到端執行的場景

每個 demo 自己負責資料準備，所以一台全新的機器除了 `just build` 與該 recipe
之外不需要別的。

**執行前後**

```bash
just demo check         # are the prerequisites present?
just demo fetch-data    # download the COSS rosbag if it is missing
just demo prepare       # fetch-data, then just build
just demo stop          # stop the stack a run left up
just demo list-runs     # recorded runs, newest first
just demo clean         # delete recorded runs (several GB each), after a prompt
```

`just demo check` 會檢查 rosbag、地圖、工作區是否已建置、`cuda_ndt_matcher` 是
release 建置而非 debug 建置、`play_launch` 是否在 `PATH` 上，以及 CUDA 工具鏈是否
認得這顆 GPU 的架構。任何一項缺失它都會以非零碼結束，因此可以當成腳本裡的閘門。

**執行**

```bash
just demo run               # the COSS NDT replay, end to end
just demo run-headless      # the same, no RViz, stack not left up
just demo run-manual-init   # no pose seed; set it yourself in RViz
just demo run-raw-speed     # with the uncorrected wheel speed
just demo exhibition        # live perception, no map and no localization
```

`run` 會在需要時取回資料、啟動堆疊、種入初始位姿、重播錄製、在 `tmp/demo-runs/`
底下寫出一個執行目錄、印出摘要，並把堆疊留著讓你檢視——這也是
`just demo stop` 存在的原因。每一個都接受一個可選的標籤，用來命名執行目錄。

`exhibition` 是其中的異類：它以無地圖模式對著即時感測器跑感知堆疊，不做定位、
不載地圖，用於展示場合——參觀者走過車旁，看著屬於自己的偵測框出現。它接受一個
模型參數（預設 `centerpoint`，若 RViz 與偵測器在搶 GPU 則用
`centerpoint_tiny`）。它需要車上的感測器。

**判讀一次執行**

```bash
just demo report [run_dir]        # metrics (default: the most recent run)
just demo yaw-bias [run_dir]      # heading-versus-course yaw bias
just demo map-quality [run_dir]   # does the map cover the scan, and agree with it?
just demo compare a=<dir> b=<dir> # runs side by side
```

**基準測試**

```bash
just demo bench [configs] [repeats]   # cuda_ndt on GPU, the same code on CPU, Autoware's
just demo bench-offline [run_dir] [frames]  # GPU versus CPU on identical recorded input
just demo bench-nvtl-probe [frames]   # scoring parity between the arms
just demo bench-report <runs.tsv>     # rebuild a report from runs already recorded
```

`bench-offline` 是值得信任的那個比較：兩條路徑吃到的是逐位元組相同的錄製輸入，
而不是看到不同掃描的兩次即時執行。

## `map` — 驗證與網格建構

```bash
just map check MAP_DIR [POSE_SOURCE] [FLAGS...]
just map grid-from-pcd MAP_DIR [FLAGS...]
just map grid-from-bag BAG MAP_DIR [FLAGS...]
```

底層是三個 Python 工具：

```bash
python3 ./scripts/map/check_map.py <map_dir> --pose-source cuda_ndt
python3 ./scripts/map/pcd_to_pgm.py <map_dir>/pointcloud_map.pcd <map_dir>/occupancy_grid --sidecar
python3 ./scripts/2dlidar/scan_accumulate_grid.py <bag> <map_dir>/occupancy_grid --sidecar
```

`check` 預設為 `cuda_ndt`。它會驗證 lanelet2 地圖、投影資訊、PCD、佔據網格——
以及整件事的重點：網格與 lanelet2 地圖是否位於同一個座標框。這正是「網格建在
錯誤座標框」所造成的失敗類型：一張看起來有效、卻定位得很差的地圖，而不是一個
錯誤訊息。

兩個 `grid-from-*` recipe 都會寫出 `occupancy_grid.pgm` 與
`occupancy_grid.yaml`，把網格的建構方式記錄在 `autosdv_map.yaml`，然後對結果
執行 `just map check <map_dir> mcl`。

`grid-from-pcd` 不帶旗標執行時，會印出高度分佈與建議的 z 區間，而不是替你猜：

```bash
just map grid-from-pcd data/COSS-map-planning
just map grid-from-pcd data/COSS-map-planning --z-min 9.1 --z-max 9.4
```

其餘旗標會直接傳遞下去——PCD 路徑的 `--resolution`、`--min-points`，以及 bag
路徑的 `--min-hits`。參閱[地圖](../guides/maps.md)。

## `sim` — 模擬

```bash
just sim coss-park   # logging sim + rosbag feed + localization recording, in parallel
```

`coss-park` 是這個模組裡唯一沒有被棄用的 recipe。它同時執行記錄模擬、rosbag
播放與一份定位錄製，並錯開啟動時間，好讓每一項在下一項依賴它之前已經起來。
它需要 COSS 錄製。

這個模組裡另外兩個 recipe 是**已棄用的寫法**，僅作為指引保留。加上
`just bag download`，它們就是整個儲存庫已棄用寫法的完整清單：

| 已棄用 | 改用 |
|--------|------|
| `just sim planning` | `just coss planning-sim` |
| `just sim logging` | `just coss logging-sim` |
| `just bag download` | `just coss download-rosbag` |

它們仍然可用，也仍然做對的事；每一個都會先印出新的寫法再轉送。之所以改名，是
因為舊名說了它是哪一種模擬、卻沒說是哪一個場景，而它們全都是 COSS Park——地圖
是 COSS，記錄模擬所重播的錄製也是 COSS 的那趟行駛。任何你寫下、而且會活過今天
的東西，都應該使用右欄。

## `tool` — 開發與監控

五個工具；值得知道的是每一個「用來做什麼」，而不只是它存在。

```bash
just tool rviz          # rviz2 -d ./src/launcher/autosdv_launch/rviz/autosdv.rviz
just tool plotjuggler   # ros2 run plotjuggler plotjuggler
just tool tui           # python3 ./scripts/testing/drive/run.py
just tool controller    # ros2 run control_test keyboard_control
just tool zed           # play_launch launch zed_wrapper zed_camera.launch.py camera_model:=zedxm
```

**`rviz`** 以本專案的版面開啟 RViz——地圖、點雲、軌跡與車輛模型都已設定好。當
堆疊已經在跑、而你啟動時沒有帶 RViz 時使用；透過 SSH 操作時就是這種情況。

**`plotjuggler`** 把任何主題對時間作圖。它適合回答「形狀」而非「數值」的問題：
速度回報是否雜訊很大、轉向命令是否震盪、位姿是否跳動。它同樣能讀即時主題與
錄製的 bag。

**`tui`** 是駕駛監控器，而且不只是監控。它在終端機裡顯示定位狀態、路線狀態、
操作模式、位置、速度，以及 NDT 除錯統計（分數、點數、迭代次數、執行時間），
並且用同一個鍵盤驅動系統：在具名位姿上初始化定位、設定到具名目標的路線、啟用
自動駕駛、停車、限速、回到手動。這些位姿來自
`scripts/testing/drive/poses.json`，並且是場地專屬的——隨附的那組是 COSS Park。

它是不用 RViz、透過 SSH 跑一個情境的方式，也是唯一非圖形化的途徑來完成教學裡
用滑鼠做的那串「2D Pose Estimate、2D Goal Pose、Engage」。它提供的位姿必須位於
lanelet2 地圖中相連的車道上，這點和 RViz 的完全一樣。

**`controller`** 是鍵盤手動控制：不經規劃直接駕駛車輛。用來在一次執行之前把車
擺到位，以及確認致動器到底有沒有反應。請注意已知的轉向左右相反問題——參閱
[車輛介面](./software/vehicle-interface.md)。

**`zed`** 單獨啟動 ZED 相機節點，使用 8081 埠，其餘什麼都不跑。當一個相機問題
可能出在相機、也可能出在堆疊其餘部分時，用這個把它分開。

## 相關頁面

- [操作車輛](../running/on-the-vehicle.md)——啟動參數
- [`play_launch`](../tutorial/06-play-launch.md)——這些 recipe 所包裝的啟動器
- [地圖](../guides/maps.md)——`map` 這組 recipe 產生與檢查的東西
- [COSS Park 場景](../running/coss-park-scenario.md)
