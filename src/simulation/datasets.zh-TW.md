<!--
Translation Metadata:
- Source file: datasets.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 資料集與 Rosbag

有哪些可以回放、如何取得，以及如何錄製自己的。

## COSS Park 測試 bag

所有模擬頁面使用的那份錄製。

```bash
just coss download-rosbag
```

下載 1.6 GiB，解開後在 `data/rosbags/outdoor_20251226_153115` 佔 2.8 GiB，過程中
約需 4.4 GiB。缺少 `synology-dl` 時會用 `cargo` 安裝，所以必須先有 Rust；完成後會
驗證 `.db3` 的 SHA-256，不符就刪除。再跑一次只檢查 checksum 就結束，因此放進腳本
安全，下載中斷後重跑也安全。

對應的地圖完全不用下載——`data/COSS-map-planning` 已納入版本控制。

錄製長 157 秒：前 115.7 秒停著不動，接著是 41 秒、最高 1.58 m/s 的行駛。

`just demo run` 會在缺少時自動取得它，所以你很少需要直接呼叫這條指令。

## Leo Drive Bus-ODD 資料集

Autoware Foundation 的資料集，含相機串流，適合用來測試視覺定位。工具位於
`scripts/leodrive-bus-launch` 子模組。

| 感測器 | 型號 | 數量 |
|--------|------|------|
| LiDAR | Velodyne VLP16 | 1（前方） |
| LiDAR | Velodyne VLP32C | 2（左、右） |
| 相機 | Lucid Vision Triton 5.4 MP | 3 |
| GNSS/INS | Applanix POS LV 120 | 1 |

```bash
cd scripts/leodrive-bus-launch
just setup          # 下載約 10.9 GB 並遷移至 Autoware 1.5.0
just play data/all-sensors-bag1_migrated
```

遷移步驟不是選用的：該資料集是對著 `autoware_auto_*` 訊息型別錄製的，而
Autoware 1.5.0 已不再定義它們。`just migrate-all` 會把 bag 改寫為 `autoware_*`。

## ROS 2 bag 的通用知識

bag 是主題流量的錄製。四種用途，在這裡都用得上：

- **錄製**行駛過程中的感測器資料，供事後分析
- **回放**它，以在不變的輸入上測試與調校演算法
- **分享**錄製，讓同事對著你看到的同一份資料除錯
- **診斷**只發生過一次的故障

### 錄製

```bash
ros2 bag record <topic> <topic> ...   # 指定的主題
ros2 bag record -a                    # 全部
ros2 bag record -a -o <directory>     # 選擇輸出目錄
```

AutoSDV 自己的錄製器會替你選好戶外感測器主題集合：

```bash
just bag record
```

在執行中的堆疊上用 `-a` 錄製會抓到非常大量的資料——使用 3D 光達時每分鐘數 GB。
通常你會想錄製指定的子集合，除非你打算之後在 RViz 中檢視結果，那就還需要
transform 與 metadata 主題。

### 檢視

```bash
ros2 bag info <bag>
```

主題、訊息數量、長度與訊息型別——回放沒有反應時先看這個，因為對著不同訊息定義
錄製的 bag 會毫無錯誤地播放完畢，卻滿足不了任何訂閱者。

### 播放

```bash
ros2 bag play <bag>
ros2 bag play <bag> --clock              # 發佈 /clock——use_sim_time 必須
ros2 bag play <bag> -r 2.0               # 兩倍速
ros2 bag play <bag> --loop               # 或 -l
ros2 bag play <bag> --topics /a /b       # 只播這些主題
ros2 bag play <bag> --start-offset 30    # 跳過前 30 秒
```

`--clock` 是最該記住的一個。記錄回放模擬以 `use_sim_time:=true` 執行，少了它
堆疊的時鐘永遠不會前進，什麼事都不會發生。

`just bag play` 會以已設定好的 `--clock` 播放 `rosbags/` 中最近一次的錄製。

## 錄製自己的資料

若這份錄製是要用來驅動記錄回放模擬，至少需要：

- 光達點雲
- IMU
- 車輛速度回報
- `/tf` 與 `/tf_static`
- 若你打算用 GNSS 做初始化，還需要 GNSS

`just bag record` 會選好這組主題。如果你手動錄製，`/tf_static` 是最常被忘記的
一個，而它的缺席會產生一份所有座標轉換都缺失、完全無法定位的回放。

要回放自己的錄製，請把啟動指向你自己的地圖：

```bash
play_launch launch autosdv_launch logging_simulation.launch.yaml \
  map_path:=/path/to/your/map
```

並先驗證地圖：

```bash
just map check /path/to/your/map cuda_ndt
```

參閱[地圖](../guides/maps.md)。

## 後續步驟

- [記錄回放模擬](../tutorial/03-logging-simulation.md)
- [COSS Park 情境](./coss-park-scenario.md)
