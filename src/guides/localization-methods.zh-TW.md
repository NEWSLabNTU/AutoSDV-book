<!--
Translation Metadata:
- Source file: localization-methods.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 定位方法

`pose_source` 決定 AutoSDV 如何估測自己的位置。共有三個選項，需求差異很大——
不同的感測器、不同的地圖產物、不同的硬體。

```bash
play_launch launch autosdv_launch autosdv.launch.yaml pose_source:=ndt
```

## 選項一覽

| `pose_source` | 估測依據 | 需要的地圖產物 | 需要 GPU |
|---------------|---------|---------------|----------|
| `cuda_ndt`（預設） | 3D 光達掃描對點雲地圖做匹配 | PCD | 是 |
| `ndt` | 同上，但在 CPU 上 | PCD | 否 |
| `mcl` | 單一 2D `LaserScan` 對佔據網格做匹配 | 佔據網格 | 否 |

## `cuda_ndt` —— 預設

CUDA 加速的 NDT 掃描匹配。比 CPU 匹配器快 1.3–1.6 倍，在 Jetson 平台上少用
57% 的 CPU——這正是它在一台同時要跑感知與控制的車輛上成為預設值的原因。

套件是 `cuda_ndt_matcher`，由 AutoSDV 維護而非上游 Autoware，並提供與標準 NDT
相同的輸入與輸出介面。

它以 Rust 撰寫。若建置時缺少 Rust 工具鏈或 `colcon-cargo-ros2`，colcon 會
**靜默地**略過該套件，這個姿態來源便無物可啟動——參閱
[安裝疑難排解](../getting-started/installation/overview.md#the-build-succeeds-but-cuda_ndt_matcher-is-absent)。

它也必須以 release 模式建置。`just build` 之所以傳入 `--cargo-args --release`
正是為此；少了它，匹配器每個掃描約需 80 毫秒而非 5 毫秒。

## `ndt` —— CPU 備援

Autoware 內建的 OpenMP NDT。相同演算法、相同地圖，不需 GPU。在沒有 CUDA 的
機器上使用，或用來釐清某個問題是否屬於 CUDA 路徑。

!!! warning "`pose_source_package` 與空值"

    `pose_source_package` 預設為 `auto`：對 `cuda_ndt` 解析為
    `cuda_ndt_matcher_launch`，其餘情況解析為內建的 Autoware NDT。只有在要接入
    第三方估測器時才明確設定它。若要使用內建 NDT，請傳
    `pose_source:=ndt`——而不是把 `pose_source_package` 設為空值，那會被
    `ros2 launch` 拒絕。

## `mcl` —— 2D 蒙地卡羅定位

以單一平面的 `LaserScan` 對 2D 佔據網格做定位，而非以 3D 點雲對 PCD。在 Autoware
範例場地上以 NDT 作為基準、跑五個亂數種子的量測結果：**平均 0.789 公尺、
p95 2.075 公尺、平均 |yaw| 0.0159 弧度**，使用三環的掃描來源。

### 掃描的介面約定

MCL 只消費**一個** `sensor_msgs/LaserScan`，其座標系只要 TF 能連到 `base_link`
即可。它本身不持有任何掃描幾何資訊。掃描的產生由感測器套件負責，因為要使用哪個
實體平面取決於感測器本身與它的安裝方式。

```bash
# 由感測器套件發佈掃描（正式運作）
play_launch launch autosdv_launch autosdv.launch.yaml \
  pose_source:=mcl map_path:=data/my_site

# 由 MCL 從 3D 點雲合成一份（僅供測試）
just coss logging-sim   # then, for MCL: pass pose_source:=mcl scan_source:=test_pointcloud
```

`mcl_scan_normalizer` 會自行處理安裝位移。這不是細節：粒子濾波器把掃描視為
*源自粒子姿態*，因此一個安裝在 `base_link` 前方 0.5 公尺的雷射座標系，會讓每一
筆距離都偏差 0.5 公尺。

### 由 3D 光達產生掃描

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  publish_scan:=true   # 另外加上 scan_ring，或 ring_min / ring_max
```

**使用少數幾環，而不是單一環。** 量測結果：

| 掃描來源 | 平均 | 種子間離散 | 通過率 | 平均 \|yaw\| |
|---|---|---|---|---|
| 厚片，0.30 公尺帶 | 0.821 m | 0.072 | 5/5 | 0.0339 rad |
| 單環 | 0.992 m | 0.317 | 3/5 | 0.0321 rad |
| **三環（70–72）** | **0.789 m** | **0.037** | **5/5** | **0.0159 rad** |

單環在幾何上是完美的平面，但在 128 環的旋轉式光達上太稀疏。三個相鄰的 VLS128
通道跨越 0.22 度——在 60 公尺處為 0.23 公尺——這比它們勝過的厚片*更緊*，所以這
並不是拿精確度換密度。

### 適用哪些感測器

**僅限旋轉式光達。** 環的抽取假設各環具有固定仰角，因此它適用於 `vlp32c`，
而不適用於套件中的固態光達。Robin-W 與 Cube1 的視野受限，且通道索引並不對應
固定仰角，所以它們沒有任何一環是水平面——而且狹窄的視野本來就難以對著 360 度
的網格做良好約束。這些感測器請走 3D NDT 路徑，或另外搭配原生的 2D 光達。

### 環是感測器專屬的，必須實測

0.11 度的通道間距是某一顆特定 VLS128 的性質，不是常數。請不要照抄本頁的環號：

```bash
python3 scripts/sensor/inspect_rings.py <bag> --topic <cloud> --height <mounting_h>
```

它會回報每個通道的仰角、指出水平的那一環，並在某一環朝上太多、打不到地面時
提出警告——低底盤車輛就是這樣被設定成把某一環對準天空的。

### 地圖與診斷

MCL 需要 `occupancy_grid.yaml` 與其 `.pgm`，而不是 PCD。請以
`just map grid-from-pcd` 或 `just map grid-from-bag` 建立，並以 `just map check`
驗證；參閱[地圖](./maps.md)。

正規化節點會回報：掃描缺失、TF 缺失、整份掃描皆為非有限值，以及安裝偏離平面。
在本專案的歷史中，每一個掃描端的失敗過去都是靜默的。

## 無地圖模式

這不是一種 `pose_source`，而是一個逃生出口。用於沒有地圖、也無法定位的室內場合：

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  use_mapless_mode:=true use_gnss:=false
```

無地圖模式意味著不使用點雲地圖。

## 如何選擇

- **在車上、戶外、有 3D 光達**：`cuda_ndt`。
- **在沒有 CUDA 的筆電上**：`ndt`。
- **只有 2D 光達，或有旋轉式 3D 光達但只有平面圖**：`mcl`。

[記錄回放模擬](../tutorial/03-logging-simulation.md)是比較它們的正確場合，因為
每次執行的錄製輸入都完全相同。

## 相關頁面

- [地圖](./maps.md) —— 每種方法需要什麼，以及如何建立
- [CUDA 點雲管線](./cuda-pipeline.md)
- [操作車輛](../getting-started/usage.md)
