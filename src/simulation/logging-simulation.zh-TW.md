<!--
Translation Metadata:
- Source file: logging-simulation.md
- Last synced: 2026-03-30
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 記錄模擬

記錄模擬將錄製的感測器資料重播通過完整的 Autoware 堆疊 — 定位、感知、規劃與控制。這會測試與實體車輛上運行的相同程式碼，使用真實的 LiDAR、IMU 和 GNSS 資料。

**先決條件**：

- 完成[推薦安裝方式](../getting-started/installation/recommended.md)
- NVIDIA GPU（感知和定位所需）

## 下載測試資料

下載 COSS 公園戶外錄製：

```bash
just download-data
```

這會將 `outdoor_20251226_153115` rosbag（約 2.8 GB）下載到 `data/rosbags/`。下載工具（`synology-dl`）會在未安裝時自動安裝。

詳細資訊請參閱[資料集與 Rosbag](./datasets.md)。

## 啟動

在一個終端機中啟動記錄模擬：

```bash
just launch-sim-logging
```

在第二個終端機中播放錄製的資料：

```bash
ros2 bag play data/rosbags/outdoor_20251226_153115/ --clock
```

開啟 Web UI：[http://localhost:8081](http://localhost:8081)。

## 您看到的內容

rosbag 提供原始感測器資料，如同車輛正在行駛。Autoware 堆疊透過以下模組處理：

- **感測** — LiDAR 點雲前處理、IMU 校正
- **定位** — NDT 掃描匹配將車輛對齊到點雲地圖
- **感知** — 基於 LiDAR 的物體偵測（CenterPoint）與追蹤
- **規劃與控制** — 軌跡產生與跟隨

在 RViz 中您應該會看到 LiDAR 點雲、偵測到的物體，以及車輛在地圖上的估計位置。

## 監控工具

開啟額外的終端機監控系統：

```bash
just tool-plotjuggler   # 繪製任何 ROS 主題的時間序列
just tool-tui           # 終端機儀表板，顯示位置姿態、速度與元件狀態
just tool-rviz          # 額外的 RViz 實例
```

## 定位模式

預設定位使用 NDT 掃描匹配。您可以選擇不同的模式：

```bash
just launch-sim-logging pose_source:=ndt     # 預設：LiDAR NDT
just launch-sim-logging pose_source:=isaac    # Isaac cuVSLAM（需要視覺地圖）
```

## 播放選項

控制 rosbag 播放：

```bash
# 以半速播放
ros2 bag play data/rosbags/outdoor_20251226_153115/ --clock -r 0.5

# 循環播放
ros2 bag play data/rosbags/outdoor_20251226_153115/ --clock -l
```

## 疑難排解

**LiDAR 點出現但定位未初始化**

NDT 需要初始位置姿態估計。若自動初始化未觸發，請在 RViz 中使用 **2D Pose Estimate** 手動設定。

**RViz 中沒有點雲**

- 確認 rosbag 正在播放：檢查 `ros2 bag play` 終端機的輸出。
- 確認 rosbag 存在：`ls data/rosbags/outdoor_20251226_153115/`

**感知節點載入失敗（CUDA 記憶體不足）**

CenterPoint 和 occupancy grid 節點需要 GPU 記憶體。若載入失敗，感知將無法運作，但定位和規劃不受影響。

## 下一步

- [COSS 公園場景](./coss-park-scenario.md) — 含錄製的完整自動化場景
- [資料集與 Rosbag](./datasets.md) — 可用的錄製資料與建立方式
- [感測器整合](../guides/sensor-integration/using-sensors.md) — 配置感測器以取得即時資料
