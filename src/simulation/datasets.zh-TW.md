<!--
Translation Metadata:
- Source file: datasets.md
- Last synced: 2026-03-30
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 資料集與 Rosbag

AutoSDV 提供錄製的感測器資料，供在沒有實體車輛的情況下進行測試和開發。

## 可用的資料集

### COSS 公園戶外錄製

| 欄位 | 值 |
|------|-----|
| 名稱 | `outdoor_20251226_153115` |
| 位置 | `data/rosbags/outdoor_20251226_153115/` |
| 大小 | 2.8 GB |
| 時長 | 157 秒 |
| 日期 | 2025-12-26 |
| 地圖 | `data/COSS-map-planning/` |

在台大 COSS 公園錄製，使用以下感測器：

| 主題 | 類型 | 頻率 |
|------|------|------|
| `/sensing/lidar/velodyne_points` | PointCloud2 | 10 Hz |
| `/sensing/lidar/velodyne_packets` | VelodyneScan | 10 Hz |
| `/sensing/camera/zedxm/imu/data` | Imu | 100 Hz |
| `/sensing/gnss/ublox/nav_sat_fix` | NavSatFix | 4 Hz |
| `/sensing/gnss/ublox/fix_velocity` | TwistWithCovarianceStamped | 4 Hz |
| `/vehicle/status/velocity_status` | VelocityReport | 20 Hz |
| `/vehicle/status/steering_status` | SteeringReport | 30 Hz |

### Leo Drive Bus-ODD 資料集

[Leo Drive Bus-ODD 資料集](https://autowarefoundation.github.io/autoware-documentation/main/datasets/)是一個與 Autoware 相容的資料集，包含相機串流，適用於測試視覺定位。下載和轉換此資料集的工具位於 `scripts/leodrive-bus-launch` 子模組中。

```bash
cd scripts/leodrive-bus-launch
just setup       # 下載（約 10.9 GB）並遷移至 Autoware 1.5.0 格式
just play data/all-sensors-bag1_migrated
```

## 下載資料

下載 COSS 公園錄製：

```bash
just download-data
```

這會執行 `scripts/rosbag/download-test-rosbag.sh`，其功能為：

1. 檢查 rosbag 是否已存在且檢查碼正確
2. 若未安裝 `synology-dl`，透過 `cargo install` 安裝
3. 從 Synology Drive 下載
4. 解壓縮並驗證 SHA256 檢查碼

## 錄製您自己的資料

從執行中的車輛錄製戶外感測器主題：

```bash
just bag-record
```

這會將帶時間戳記的 rosbag 儲存到 `rosbags/` 目錄，包含 `scripts/rosbag/outdoor_topics.txt` 中列出的所有感測器和車輛狀態主題。

播放最近的錄製：

```bash
just bag-play
```

## Rosbag 格式

AutoSDV 使用 ROS 2 bag 格式（SQLite3 儲存）。每個 rosbag 目錄包含：

- `<name>_0.db3` — 訊息資料庫
- `metadata.yaml` — 主題列表、訊息數量、時長與儲存格式

檢查 rosbag：

```bash
ros2 bag info data/rosbags/outdoor_20251226_153115/
```

## 使用外部資料集

若要在 AutoSDV 中使用與 Autoware 相容的 rosbag：

1. 確保 rosbag 至少包含 LiDAR 點雲和車輛狀態主題
2. 確認主題名稱與 AutoSDV 感測器套件配置相符（參閱 `src/sensor_kit/`）
3. 提供對應的點雲地圖和 Lanelet2 地圖
4. 以適當的感測器套件啟動記錄模擬：

```bash
just launch-sim-logging sensor_suite:=custom lidar_model:=vlp32c
```
