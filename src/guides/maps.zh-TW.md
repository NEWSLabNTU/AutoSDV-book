<!--
Translation Metadata:
- Source file: maps.md
- Last synced: 2026-09-12
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
| `visual` | 視覺地圖 | `cuvgl_map/`、`cuvslam_map/` |
| 全部 | 供規劃使用的 lanelet2 地圖 | `lanelet2_map.osm` 與投影資訊 |

地圖目錄以 `map_path` 傳入：

```bash
play_launch launch autosdv_launch autosdv.launch.yaml map_path:=data/my_site
```

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

## 視覺地圖

供 `pose_source:=visual` 使用：

```bash
# 1. 以 ZED 立體相機 + IMU 錄製 rosbag
./scripts/visual-map/record.sh ./data/visual_maps/my_location

# 2. 建立地圖
./scripts/visual-map/create-map.sh ./data/visual_maps/my_location_recording
```

這會產生 `cuvgl_map/`、`cuvslam_map/` 與 `occupancy_map/`，並以
`visual_map_dir:=` 而非 `map_path:=` 傳入。

## 預設地圖

`data/COSS-map-planning`，即 COSS Park 地圖，是 `map_path` 的預設值，也是
[模擬指南](../tutorial/02-planning-simulation.md)所使用的地圖。

## 相關頁面

- [定位方法](./localization-methods.md)
- 儲存庫中的 `docs/design/map-handling-per-localization-method.md`
