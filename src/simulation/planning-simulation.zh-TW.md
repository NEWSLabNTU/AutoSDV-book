<!--
Translation Metadata:
- Source file: planning-simulation.md
- Last synced: 2026-03-30
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 規劃模擬

規劃模擬器在不使用任何感測器資料的情況下運行 Autoware 規劃與控制堆疊。您在地圖上設定目的地，車輛會規劃路線並行駛到該處。這是探索 AutoSDV 最簡單的方式，不需要 GPU 或任何硬體。

**先決條件**：請先完成[推薦安裝方式](../getting-started/installation/recommended.md)。

## 啟動

```bash
just launch-sim-planning
```

這會以 AutoSDV 車輛模型和 COSS 公園地圖啟動 Autoware 規劃模擬器。開啟 Web UI：[http://localhost:8081](http://localhost:8081)。

## 設定目的地

1. 在 RViz 工具列中點擊 **2D Pose Estimate**，在地圖上點擊以設定車輛的初始位置。
2. 點擊 **2D Goal Pose**，在地圖上點擊一個位置。
3. 規劃器計算車道級路線，車輛開始行駛。

## 您看到的內容

規劃模擬器運行以下 Autoware 模組：

- **任務規劃器** — 沿車道圖計算全域路線
- **行為規劃器** — 產生局部軌跡（車道跟隨、變換車道、轉彎）
- **運動規劃器** — 為控制器平滑軌跡
- **控制** — 使用模型預測或 PID 控制器跟隨軌跡

不涉及感知和定位。車輛的位置由模擬器直接提供。

## COSS 公園地圖

預設地圖位於 `data/COSS-map-planning/`。這是台大 COSS 公園區域的點雲地圖和 Lanelet2 向量地圖。若要使用不同的地圖，請將路徑作為參數傳入：

```bash
play_launch launch \
    --web-addr 0.0.0.0:8081 \
    autoware_launch planning_simulator.launch.xml \
    map_path:=/path/to/your/map \
    vehicle_model:=autosdv_vehicle \
    sensor_model:=autosdv_sensor_kit
```

## 新增障礙物

在 RViz 工具列中，使用 **Dummy Object** 工具在地圖上放置靜態或移動的障礙物。規劃器會繞過它們或等待它們通過。

## 常見場景

- **車道行駛** — 沿道路設定目標，觀察車道跟隨行為
- **迴轉** — 在車輛後方設定目標以觸發迴轉操作
- **障礙物迴避** — 在車道上放置虛擬物件，觀看規劃器重新規劃路線

## 疑難排解

**設定目標後車輛不移動**

- 確認已先設定初始位姿（2D Pose Estimate）。
- 確認目標在可行駛的車道上（不在人行道或道路外區域）。

**地圖未載入**

- 確認地圖目錄存在：`ls data/COSS-map-planning/`
- 確認其中包含 `pointcloud_map.pcd` 和 `lanelet2_map.osm`。

**Web UI 無法存取**

- 確認 URL 為 `http://localhost:8081`。
- 若在遠端機器上執行，請使用機器的 IP 位址替代 `localhost`。

## 下一步

- [記錄模擬](./logging-simulation.md) — 將真實感測器資料重播通過完整堆疊
- [操作車輛](../getting-started/usage.md) — 啟動參數與工具
