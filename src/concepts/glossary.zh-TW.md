<!--
Translation Metadata:
- Source file: glossary.md
- Last synced: 2026-09-21
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 詞彙表

本書中出現的術語，以 AutoSDV 使用它們的意義為準。

## 軟體與環境

**Workspace（工作空間）** —— 一個 `src/` 底下放著 ROS 2 套件的目錄，由 colcon
建置成 `build/` 與 `install/`。AutoSDV 的儲存庫根目錄就是一個。

**Package（套件）** —— ROS 2 安裝與尋找東西的單位。啟動檔的定址方式是
`<package> <file>`。

**Overlay / underlay（覆蓋層／底層）** —— 載入的環境會堆疊。每個
`source …/setup.bash` 都在先前的內容之上加一個 overlay；底下各層則是 underlay。
AutoSDV 的工作空間是 Autoware 的 overlay，而 Autoware 是 ROS 2 的 overlay。參閱
[環境與相依套件](./environment.md)。

**colcon** —— 建置工具。`just build` 以本專案所需的旗標包裝它。

**rosdep** —— 解析套件所宣告的系統與 ROS 相依項，這也是 AutoSDV 沒有個別驅動程式
`apt install` 步驟的原因。

**direnv** —— 讀取 `.envrc` 並在你進入目錄時載入環境的工具。它是那兩行 `source`
的便利包裝，不是理解它們的替代品。

## 啟動

**Launch file（啟動檔）** —— 以參數與重新對應啟動節點、並 include 其他啟動檔的
檔案。以 YAML、XML 或 Python 撰寫。

**Launch argument（啟動參數）** —— 啟動檔的輸入，以 `name:=value` 傳入。只在啟動
檔被求值期間存在。永遠是字串。

**Parameter（節點參數）** —— 屬於某個執行中節點的具名值，可用 `ros2 param get`
讀取。

**Substitution（替換）** —— 在啟動求值期間計算出的值：`$(var x)`、
`$(find-pkg-share pkg)`、`$(env NAME default)`、`$(eval "...")`。

**Preset（預設組態）** —— 只宣告參數預設值、以名稱選取的啟動檔。AutoSDV 有感知與
定位兩種預設組態。

**Composable node（可組合節點）** —— 載入到共用行程而非各自獨立執行的節點，因此
訊息在行程內傳遞。

**Component container（組件容器）** —— 承載可組合節點的行程。以 PID 而非行程群組
殺掉啟動器，會讓這些容器變成孤兒行程。

**`play_launch`** —— AutoSDV 的啟動執行器，`ros2 launch` 的直接替代品，額外提供
行程群組關閉、網頁介面與監控。它是我們自己的軟體；何時該退回官方作法，參閱
[play_launch](../tutorial/06-play-launch.md)。

## 訊息傳遞

**Node（節點）** —— 參與 ROS 圖的行程（或可組合節點）。

**Topic（主題）** —— 具名的通道。發佈者寫入，訂閱者讀取。

**QoS** —— Quality of Service，決定發佈者與訂閱者是否會連上的端點政策。
`RELIABLE` 的訂閱者不會從 `BEST_EFFORT` 的發佈者收到東西。參閱
[檢視執行中的系統](./inspecting.md)。

**TF** —— 轉換系統：座標系的樹與它們之間的關係。`/tf` 承載會變動的轉換，
`/tf_static` 承載固定的。

**rosbag** —— 主題流量的錄製，以 `ros2 bag play` 重播。當堆疊跑在模擬時間上時
必須加 `--clock`。

## 車輛狀態

**Pose（姿態）** —— 位置與朝向。本書中通常指車輛在 `map` 座標系中的姿態。

**Twist** —— 線速度與角速度。與姿態分開估計；`localization_preset` 選擇 twist
的來源。

**Odometry（里程計）** —— 透過積分輪子轉動與 IMU 估計出的運動。平滑且在局部準確，
但會無界漂移。

**`base_link`** —— 車體座標系，位於後軸中心。感測器座標系都相對它定義。

## 定位

**NDT** —— Normal Distributions Transform。把地圖表示為高斯分佈的網格，藉此將 3D
掃描與點雲地圖匹配。AutoSDV 的預設是 `cuda_ndt`（GPU），並有 CPU 備援（`ndt`）。

**MCL** —— Monte Carlo Localization。以粒子濾波器將 2D `LaserScan` 與佔據網格匹配。

**PCD** —— Point Cloud Data，NDT 所匹配的 3D 地圖檔案格式。

**Occupancy grid（佔據網格）** —— 由自由／被佔據／未知格子構成的 2D 地圖，形式是
一張 `.pgm` 影像加上描述解析度與原點的 `.yaml`。MCL 需要的是它，而不是 PCD。

**lanelet2** —— 道路網格式：車道、停止線、交通規則。供規劃使用，感知也用它過濾。
儲存為 `lanelet2_map.osm`。

**NVTL** —— Nearest Voxel Transformation Likelihood，NDT 回報的掃描與地圖匹配程度
分數。數值低表示匹配不良。

**TP** —— Transformation Probability，NDT 的另一個匹配分數。

**收斂門檻（convergence gate）** —— NVTL 的門檻值；低於它，匹配器就把該幀判為未
收斂並丟棄結果。碰到 `max_iterations` 上限的幀同樣算未收斂，於是位姿根本不會發布。

**GLIM** —— 用來建立點雲地圖的光達慣性 SLAM 套件。它保留可再編輯的 dump（因子圖、
submap、軌跡），而不只是一團點雲。

**迴路閉合（loop closure）** —— 認出曾經到過的地點，並修正兩次造訪之間累積的漂移。
受限的視野會讓這件事變難，而它屬於建圖的問題，不是定位的問題。


## 感測與感知

**Deskew（去畸變）** —— 修正掃描期間車輛運動造成的失真。需要每個點的時間偏移量，
而不是每個 LiDAR 驅動程式都提供。

**Ring（環）** —— 旋轉式 LiDAR 中一條固定仰角的掃描線。環的抽取就是從 3D 感測器
產生 2D `LaserScan` 的方式。

**Voxel grid（體素網格）** —— 每個 3D 網格單元只保留一個點的降採樣方式。

**CenterPoint** —— AutoSDV 所執行的 LiDAR 3D 物件偵測模型。

**TensorRT** —— NVIDIA 的推論執行期。它把 `.onnx` 模型編譯成同時綁定 TensorRT
版本與特定 GPU 的 `.engine`，這就是引擎無法在別處建好再搬過來的原因。

**`cuda_blackboard`** —— CUDA 管線各階段用來傳遞 GPU 指標的機制。它是行程內的，
這就是所有 CUDA 階段必須共用一個容器的原因。

## 操作

**MRM** —— Minimum Risk Manoeuvre，最小風險操作。系統必須安全停下時所做的事。

**舒適停車（comfortable stop）** —— 兩種 MRM 行為中較溫和的那一種：受控的減速，
相對於緊急停車所用的最大可用煞車力。

**診斷圖（diagnostic graph）** —— Autoware 把各項健康檢查匯聚成單一系統判定的那棵
樹，也是決定要不要啟動 MRM 的依據。

**ODD** —— Operational Design Domain，系統被設計來運作的條件範圍。

**Planning simulator（路徑規劃模擬器）** —— 使用運動學車輛模型、沒有感測器的模擬。
運作到規劃與控制。

**Logging simulation（記錄回放模擬）** —— 將錄製的感測器資料重播通過真實堆疊。
同時運作到定位與感知。

**Mapless mode（無地圖模式）** —— 在沒有地圖也沒有定位的情況下運作，供室內使用。

## 接下來

- [環境與相依套件](./environment.md)
- [Autoware 管線](./autoware-conventions.md)
