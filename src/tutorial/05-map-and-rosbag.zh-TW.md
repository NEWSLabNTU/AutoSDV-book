<!--
Translation Metadata:
- Source file: 05-map-and-rosbag.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 5. 地圖與 Rosbag

你現在已經用了同樣兩份資料三次。本頁說明它們究竟是什麼——一部分是因為有趣，一部分
是因為其中一份的名稱會誤導人，若沒人解釋，之後會讓你困惑。

## `data/COSS-map-planning` 不是一張規劃地圖

這個名稱是歷史遺留。這個目錄裝的是**該場地的完整地圖**，而三個不同的組件各自從中
取用不同的檔案：

```
data/COSS-map-planning/
├── lanelet2_map.osm              590 KB   道路網          → 規劃
├── pointcloud_map.pcd             78 MB   3D 點雲         → NDT 定位
├── occupancy_grid.pgm + .yaml    3.9 MB   2D 網格         → MCL 定位
├── map_projector_info.yaml                這在地球上的哪裡
├── pointcloud_map_metadata.yaml           PCD 如何分塊
└── autosdv_map.yaml                       來源資訊
```

所以當你把 `map_path:=$PWD/data/COSS-map-planning` 傳給路徑規劃模擬，接著又在記錄
回放模擬中遇到同一個目錄時，那不是巧合，而且用到的並不是同一個檔案。路徑規劃模擬器
讀的是 lanelet2 地圖，完全忽略點雲。NDT 則相反。

### `lanelet2_map.osm` —— 道路網

車道、它們的邊界、停止線與交通規則，以 OpenStreetMap 的 XML 加上 lanelet2 的擴充
表示。節點同時帶有全球座標與本地座標：

```xml
<node id="10300004601" lat="25.0202319785" lon="121.54273096174">
  <tag k="local_x" v="304774.8603999999"/>
  <tag k="local_y" v="2768128.0634000003"/>
  <tag k="ele" v="8.9764"/>
</node>
```

路線規劃就是在這上面跑的。當路徑規劃模擬中某個 `2D Goal Pose` 沒有產生路線時，那是
因為你點的位置不在任何 lanelet 內——決定這件事的是地圖，不是規劃器。

### `map_projector_info.yaml` —— 這在地球上的哪裡

```yaml
projector_type: TransverseMercator
vertical_datum: WGS84
map_origin:
  latitude: 25.0201
  longitude: 121.5423
  altitude: 25.0
```

它把 `map` 座標系的原點固定在一個真實地點——國立臺灣大學的 COSS 場地。你看到的每一個
座標，包括植入於 `(-1.839, -8.280)` 的那個姿態，都是距離該點的公尺數。

它也是 GNSS 能被用來初始化的原因：一組經緯度之所以能換算成 map 座標系的公尺，正是
因為有這個檔案。

### `pointcloud_map.pcd` —— NDT 匹配的對象

78 MB，約 490 萬個點，依 `pointcloud_map_metadata.yaml` 從 `[-150, -150]` 起以
300 m × 300 m 分塊。

這就是你等了 25 秒載入的那個檔案。NDT 並不是直接對原始點做匹配：它把地圖切成體素，
對每個體素內的點擬合一個高斯分佈，然後找出讓即時掃描在該模型下最可能的車輛姿態。
名稱就是這麼來的——Normal Distributions Transform。

### `occupancy_grid.pgm` + `.yaml` —— 2D 網格

```yaml
image: occupancy_grid.pgm
resolution: 0.05
origin: [-65.000, -25.000, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
```

一張灰階影像，每個像素 5 公分，加上把它放進 map 座標系的描述資料。只有
`pose_source:=mcl` 會用它，MCL 用單一平面的 `LaserScan` 與它匹配，而不是用 3D 點雲
與 PCD 匹配。

它是從 PCD 切出一個高度帶產生的——如果你需要為自己的場地做一張，參閱
[地圖](../guides/maps.md)。那個高度帶是唯一需要你判斷的地方，選錯會產生一張看起來
正常、定位卻很差的網格。

### 在信任地圖之前先檢查它

```bash
just map check data/COSS-map-planning cuda_ndt
just map check data/COSS-map-planning mcl
```

這會驗證該方法所需的產物是否齊備，以及——這工具存在的理由——佔據網格是否與 lanelet2
地圖位於**同一個座標系**。建在錯誤座標系的網格會給你一個乾淨啟動、自信定位、卻整體
偏移一個固定量的系統。

## 那段錄製

```bash
ros2 bag info data/rosbags/outdoor_20251226_153115
```

```
Bag size:   2.8 GiB
Duration:   157.005 s
Start:      Dec 26 2025 15:31:16
Messages:   43538
```

16 個主題。重要的那些：

| 主題 | 型別 | 數量 | 頻率 |
|---|---|---|---|
| `/sensing/lidar/velodyne_points` | `PointCloud2` | 1570 | 10 Hz |
| `/sensing/lidar/velodyne_packets` | `VelodyneScan` | 1571 | 10 Hz |
| `/sensing/camera/zedxm/imu/data` | `Imu` | 15463 | 約 98 Hz |
| `/vehicle/status/steering_status` | `SteeringReport` | 4710 | 約 30 Hz |
| `/vehicle/status/gear_status` | `GearReport` | 4702 | 約 30 Hz |
| `/vehicle/status/velocity_status` | `VelocityReport` | 3134 | 約 20 Hz |
| `/sensing/gnss/ublox/nav_sat_fix` | `NavSatFix` | 627 | 約 4 Hz |

當時的車輛是 Velodyne VLP-32C 配置搭配 ZED X Mini，被錄下的是後者的 IMU。主題名稱
依循 [Autoware 慣例](../concepts/autoware-conventions.md)——`/sensing/` 是輸入，
`/vehicle/status/` 是車輛回報的內容。

### 裡面*沒有*什麼

**裡面沒有 `/tf` 或 `/tf_static`。** 這會讓人意外，因為回放顯然需要座標轉換。它們
來自堆疊而非錄製：感測器套件描述發佈固定的 sensor-到-`base_link` 轉換，而定位發佈
`map → odom`。因此把這個 bag 重播進一台設定不同的車輛，會使用*那台*車的幾何——當校正
改善時這是優點，若你假設 bag 是自足的則是陷阱。

裡面也沒有相機影像；只保留了 ZED 的 IMU 與健康狀態主題。`/sensing/gnss/ntrip/rtcm`
這個主題存在但有**零**則訊息——當天沒有 RTK 修正可用。

### 那段行駛

- **0 – 116 秒**：停著，感測器運作中
- **116.3 秒**：第一次超過 0.2 m/s 的移動
- **116 – 157 秒**：41 秒的行駛，最高 1.58 m/s

那段長長的靜止前綴其實很有用。它讓定位在任何東西移動之前有一段容易收斂的時間，也是
量測數據要把 `init` 與 `track` 分開的原因——這兩半是不同的問題。

### GNSS 不夠好，不能拿來初始化

這就是為什麼每個 demo 都傳 `use_gnss:=false`。那個定位是單點的、約有 20 公尺散佈，
而且與行進方向不一致，所以用它來初始化定位會讓車輛每次落在不同地方。改用一個錄下來
的姿態，那才是 demo 可重現的原因。

這件事值得看一次，因為在沒有 RTK 的戶外，這是常態——不是這份資料集的缺陷。

## 使用你自己的資料

```bash
just bag record    # 錄製戶外感測器主題集合
```

接著對著你自己的地圖重播：

```bash
play_launch launch autosdv_launch logging_simulation.launch.yaml \
  map_path:=/path/to/your/map
```

先用 `just map check` 驗證地圖。更多內容參閱
[資料集與 Rosbag](../simulation/datasets.md)與[地圖](../guides/maps.md)。

**接下來：** [6. play_launch](./06-play-launch.md) —— 最後一層，也是你在需要之前就
該知道其注意事項的那一層。
