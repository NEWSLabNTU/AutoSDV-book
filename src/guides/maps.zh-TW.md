<!--
Translation Metadata:
- Source file: maps.md
- Last synced: 2026-09-21
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 地圖

不同的定位方法消費不同的地圖產物。給錯產物通常會*安靜地*失敗——它會定位得很差，
而不是拒絕啟動——這就是為什麼本頁大部分在談檢查。

## 各方法需要什麼

| `pose_source` | 需要 | 地圖目錄中的檔案 |
|---------------|------|-----------------|
| `cuda_ndt`、`ndt` | 3D 點雲地圖 | `pointcloud_map.pcd` |
| `mcl` | 2D 佔據網格 | `occupancy_grid.yaml` + `occupancy_grid.pgm` |
| 全部 | 供規劃使用的 lanelet2 地圖 | `lanelet2_map.osm` 與投影資訊 |

地圖目錄以 `map_path` 傳入：

```bash
play_launch launch autosdv_launch autosdv.launch.yaml map_path:=data/my_site
```

### 三種產物之間的關係

它們並不是三個各自獨立、要用三種獨立方式取得的東西。其中一個是實地測繪出來的，
另外兩個則建立在它之上：

```text
a recorded drive
      |
      | SLAM (GLIM)
      v
pointcloud_map.pcd  ------ slice a height band ----->  occupancy_grid.pgm/.yaml
      |                                                  (what mcl matches)
      | (what ndt / cuda_ndt match)
      |
      +-- drawn on top of, by hand ----> lanelet2_map.osm  (what planning follows)
```

點雲就是那份測繪成果。佔據網格是穿過它的一個水平切片，這也是為什麼下面的
[建立佔據網格](#建立佔據網格)是從 PCD 開始的。lanelet2 地圖則是由人在點雲之上
畫出來的——決定車道與停止線在哪裡——而 `map_projector_info.yaml` 是把這些檔案綁在
同一個座標系上的東西：可以是有地理參考的（`TransverseMercator`、`MGRS` 等），也
可以是局部的（`Local`），但整個目錄裡的每個檔案都必須是同一個。

這也是為什麼一個目錄裝的東西比它的名字所暗示的還多。
`data/COSS-map-planning` 聽起來像是一張規劃用地圖，但它四樣都有：

```text
data/COSS-map-planning/
├── pointcloud_map.pcd            74.7 MB   the survey
├── pointcloud_map_metadata.yaml            which PCD covers which tile
├── lanelet2_map.osm                        lanes, for planning
├── map_projector_info.yaml                 TransverseMercator @ 25.0201, 121.5423
├── occupancy_grid.pgm / .yaml              derived from the PCD, for mcl
└── autosdv_map.yaml                        provenance, optional
```

所以一個場地只測繪一次，各個 `pose_source` 所需的產物都由那一份測繪成果產生。

## 點雲地圖從哪裡來

`pointcloud_map.pcd` 是其他一切所依賴的輸入，而這個儲存庫裡沒有任何東西能無中生有
地產生它：它來自**開著車、讓光達與 IMU 持續運轉，把過程錄下來，再把這份錄製送進
SLAM 建圖器**。建圖器負責的正是關鍵的那一段——它估計軌跡，在路線重訪自身之處閉合
迴路，然後才把各次掃描疊成一片一致的點雲。用里程計姿態直接把掃描串起來是行不通
的；漂移會以牆壁重影的形式顯現。

!!! warning "本節並未實際執行過"

    產生一張地圖需要一台車、一個場地、以及一趟實際行駛。以下指令整理自本儲存庫
    改寫自
    一個姊妹專案的教學——它們**並未**在此經過一次實際建圖驗證。從
    [在信任地圖之前先檢查它](#在信任地圖之前先檢查它)以下的內容，都有對
    `data/COSS-map-planning` 實際執行過。

### 錄製行駛資料

錄下串接後的點雲、IMU，以及（在戶外）GNSS，再加上座標轉換：

```bash
ros2 bag record -o mapping_run \
  /sensing/lidar/concatenated/pointcloud \
  /sensing/imu/imu_data \
  /sensing/gnss/ublox/nav_sat_fix \
  /tf /tf_static
```

`just bag record` **不是**這件事的捷徑。它錄的是 `scripts/rosbag/outdoor_topics.txt`
裡那份固定清單，目的是把一趟行駛重播進整個系統：原始的 `velodyne_points` 與 ZED
的 IMU，而不是串接後的點雲，也不是 `/sensing/imu/imu_data`。建圖資料要明確地錄。

你怎麼開車，決定了這張地圖最好能到什麼程度，而且事後再多的處理都救不回來：

- **開慢一點**——時速 5 到 10 公里。快速運動會放大每一個時間同步與標定的誤差。
- **回到出發點**，而且用相近的視角，讓迴路閉合有重疊可用。一條從不重訪自身的路線
  是無法被修正的。
- **從一個以上的方向涵蓋每個區域**，讓相鄰的來回之間有重疊。
- **一開始先靜止幾秒**，讓 IMU 穩定下來。
- **盡量避開人群、車流與茂密樹葉**。移動物體會在點雲裡變成拖影，之後得靠人工刪掉。
- **離開場地前先檢查錄製結果**：`ros2 bag info mapping_run`——訊息數量與時間長度
  應該合理。一份掉了點雲訊息的 bag 是修不回來的。

### 用 GLIM 建立點雲

GLIM 執行光達慣性的圖優化 SLAM，而它之所以是這裡採用的建圖器，是因為它的結果
**可以再編輯**：它會保留一個 dump 目錄——因子圖、各個 submap、軌跡，以及它所使用
的設定——並附帶一個離線檢視器，讓你能手動補上一條迴路閉合、重新優化、刪掉移動
物體，之後才匯出。壓平後的點雲只能從 bag 重跑；dump 則是可以修的。

它讀的是 AutoSDV 的主題：

| GLIM 設定項目 | AutoSDV 主題 |
|---|---|
| `points_topic` | `/sensing/lidar/concatenated/pointcloud` |
| `imu_topic` | `/sensing/imu/imu_data` |

!!! note "尚未在 AutoSDV 車上執行過"

    GLIM 是姊妹專案（金車專案）所使用的建圖器，本流程來自它的教學文件。GLIM 不由
    `setup.sh` 安裝，在本原始碼樹中也沒有任何引用，所以要自行安裝；以下指令都是
    GLIM 自己的，只有主題名稱是 AutoSDV 的。

離線對著 bag 執行它，並使用一份 GLIM `config` 目錄的副本：其中 `config_ros.json`
要指定 `/sensing/lidar/concatenated/pointcloud` 與 `/sensing/imu/imu_data`，而
`config_sensors.json` 要帶有真實的光達對 IMU 外參：

```bash
ros2 run glim_ros glim_rosbag ./mapping_run \
  --ros-args -p config_path:=$(realpath ./config)
```

正常結束時，它會把 dump 寫到 `/tmp/dump`。請立刻把它搬到能長期保存的地方——`/tmp`
是會被清掉的：

```bash
mkdir -p ~/glim-results
mv /tmp/dump ~/glim-results/my_site_raw
```

然後在匯出任何東西之前，先檢視與修正它：

```bash
ros2 run glim_ros offline_viewer   # 開啟 dump、檢查對位、補上迴路閉合
ros2 run glim_ros map_editor       # 在姿態正確之後，刪掉停放車輛與其他殘影
```

順序很重要：先修軌跡，再刪點。地圖編輯器會凍結 submap 的姿態，所以在圖優化正確
之前刪掉的東西，之後都得再刪一次。

GLIM 匯出的是二進位 PLY，不是 PCD。只有在非轉不可時才轉，而且要保留 PLY：

```bash
pcl_ply2pcd ~/glim-results/my_site_final.ply data/my_site/pointcloud_map.pcd
```

轉檔之後要檢查點數、單位（公尺）與欄位。轉檔工具會安靜地丟掉純量欄位，尤其是
intensity。

### 保留錄製資料與建圖器自己的輸出

要留的是三樣東西，不是一樣：

1. **那份 bag。** 它是唯一能用不同建圖器或不同參數重新處理的產物。
2. **建圖器自己的輸出**——GLIM 的 dump 目錄，或其他建圖器儲存的關鍵影格與圖。這是
   之後還能再做迴路閉合、與後續測繪合併、或重新優化的東西。
3. **匯出的點雲**，也就是那個 `.ply` 或 `.pcd`。

只有第三樣會放進地圖目錄，而它偏偏是修不了的那一個。PCD 就是一袋被壓平的點，背後
沒有圖結構：半年後在裡面發現一道接縫，是無法原地修好的，而少了前兩樣產物，答案就
只剩「再跑一趟場地」。請把設定、標定與測繪日期一起保存在它們旁邊。

### 組裝地圖目錄

Autoware 的地圖載入器要求特定檔名，所以點雲要以它期望的名字複製進去，並附上一個
metadata 檔案說明哪個檔案覆蓋哪一塊：

```bash
mkdir -p data/my_site
cp ~/glim-results/my_site_final.pcd data/my_site/pointcloud_map.pcd
```

對於單一、未分割的點雲，`pointcloud_map_metadata.yaml` 就是一格、大到足以容納它。
COSS 地圖的是 300 公尺見方，左下角在 (−150, −150)：

```yaml
x_resolution: 300.0
y_resolution: 300.0
pointcloud_map.pcd: [-150, -150]
```

`map_projector_info.yaml` 宣告座標系。有地理參考的版本，如同 COSS：

```yaml
projector_type: TransverseMercator
vertical_datum: WGS84
map_origin:
  latitude: 25.0201
  longitude: 121.5423
  altitude: 25.0
```

或者，對於沒有大地基準的室內場地：

```yaml
projector_type: Local
vertical_datum: WGS84
```

`Local` 是唯一一種情況：lanelet2 地圖自己的節點必須帶 `local_x` / `local_y` 標籤，
而不是經緯度——少了的話 `just map check` 會明確指出來，而且這種情況下無法使用以
GNSS 為基礎的姿態初始化。

lanelet2 地圖是剩下的那一塊，而它是被「畫」出來的、不是算出來的：由人在向量地圖
編輯器裡，對著點雲畫出車道與停止線。那項工作發生在本儲存庫之外，使用的工具也不是
本頁能夠代言的。

接著就檢查你建出來的東西，也就是下一節。

## 在信任地圖之前先檢查它

```bash
just map check <map_dir> [pose_source]
just map check data/COSS-map-planning cuda_ndt
just map check data/COSS-map-planning mcl
```

這會驗證指定方法所需的產物是否齊備，以及——這工具存在的理由——**佔據網格是否
位於 lanelet2 地圖的座標系中**。建在錯誤座標系的網格會產生一個乾淨啟動、自信
定位、卻整體偏移一個固定量的系統。這類失敗曾耗掉數天；這項檢查只要一秒。

額外的旗標會直接傳遞下去，例如用 `--grid-yaml NAME` 檢查
`occupancy_grid.yaml` 以外的網格變體。

## 建立佔據網格

MCL 需要網格，而多數場地只有 PCD。有兩種取得方式。

### 從既有的 PCD 地圖

```bash
just map grid-from-pcd data/COSS-map-planning
```

**請先不加任何旗標執行一次。** 它會印出點雲的高度分布、估計的地面高度與建議的
高度帶——然後就拒絕自行猜測，因為 z 帶是唯一需要你判斷的地方，而且它不寬容。
錯誤的高度帶會產生一個看起來正常、卻定位很差的網格，而不是一個錯誤訊息。

接著帶著高度帶重新執行：

```bash
just map grid-from-pcd data/COSS-map-planning --z-min 9.1 --z-max 9.4
```

這會寫出 `occupancy_grid.pgm` 與 `occupancy_grid.yaml`，把網格的建立方式記錄在
`autosdv_map.yaml` 中，並針對 `pose_source:=mcl` 驗證結果。

其他旗標：`--resolution`（預設 0.05 公尺/像素）與 `--min-points`，即一個格子要
被視為被佔據時、該高度帶內所需的點數。

### 從錄製的行駛資料，適用於沒有 PCD 的場地

```bash
just map grid-from-bag <bag> <map_dir>
```

以各筆掃描的真實姿態累積 2D 掃描。這裡的高度帶是相對於*掃描平面*而非場地地面，
所以它的預設值（−0.15 … 0.15 公尺）是有意義的，而且通常就是對的。

兩種方式都會把來源資訊記錄在 `autosdv_map.yaml`，因此任何一張網格都能追溯它是
怎麼做出來的。

## 如何選擇 z 帶

這個高度帶是穿過點雲的一個水平切片。你會希望它：

- **在地面之上**，否則每個格子都會是被佔據的
- **在懸空物之下**——樹冠、遮雨棚、天花板——否則你會把車上光達看不到的東西畫進
  地圖
- **大致位於掃描平面將處的高度**，因為那正是粒子濾波器要比對的對象

從估計地面高度稍微往上、取 0.3 公尺的帶，是個合理的初次嘗試。接著看看那張
`.pgm`：牆面與建築立面應該是連續的線條，開闊地面應該是空的。

## 預設地圖

`data/COSS-map-planning`，即 COSS Park 地圖，是 `map_path` 的預設值，也是
[模擬指南](../tutorial/02-planning-simulation.md)所使用的地圖。

## 相關頁面

- [定位方法](./localization-methods.md)
- [NDT 調校](./ndt-tuning.md)——在地圖已經存在、NDT 已經跑在上面之後
- 儲存庫中的 `docs/design/map-handling-per-localization-method.md`
