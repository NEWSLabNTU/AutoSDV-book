<!--
Translation Metadata:
- Source file: localization-diagnostics.md
- Last synced: 2026-09-21
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 定位診斷

`ros2 topic hz` 告訴你某個主題正在發布。它不會告訴你上面那個位姿是對的，而定位典型的
失敗，正是一套持續發布著自信卻錯誤位姿的系統。

AutoSDV 在 `scripts/testing/localization/` 中提供十九個腳本，另有兩個在它的 `mapcheck/`
子目錄下，回答 `ros2` 無法回答的問題。
本頁說明每個腳本回答什麼、以及它是為哪個問題而存在。至於拿到答案後要做什麼，見
[NDT 調校](./ndt-tuning.md)。

## 執行方式

每個腳本都需要 ROS 環境與工作空間在路徑上：

```bash
source scripts/env.sh
python3 scripts/testing/localization/check_ndt_activated.py
```

即時工具訂閱執行中的系統，所以先啟動系統，再在第二或第三個終端機執行它們。離線工具讀取
已記錄的執行目錄，不需要任何東西在跑。

!!! note "撰寫本頁時並未實際執行"

    以下對腳本行為的描述讀自它們的原始碼，而非實際工作階段。檔名、選項與主題名稱都已對照
    程式樹驗證。

其中幾個數值腳本會在 import numpy 之前以 `PYTHONNOUSERSITE=1` 重新 exec 自己。這是刻意的：
`~/.local` 中的 numpy 會遮蔽 apt 的 numpy，而 apt 的 scipy 與 matplotlib 是對著後者建置的，
產生的 import 錯誤看起來像 scipy 的 bug 而不是遮蔽問題。你不需要做任何事；只是當你看到
行程自我替換時，知道是怎麼回事。

## 從這裡開始：匹配器有被啟用嗎？

```bash
python3 scripts/testing/localization/check_ndt_activated.py --timeout 20
```

`ndt_scan_matcher` 處於 ACTIVATED 時離開碼 0，否則為 1，而且在能判斷時會印出原因——所以
測試框架可以用它當閘門。

這是第一項檢查，原因在於這個失敗藏得很好。`is_activated_` *只*由 trigger service 寫入；
節點不會自行啟用。若初始化在抵達那個服務之前丟出例外——缺少 `map` 到 `pose.frame_id` 的 TF、
地圖尚未載入、或還沒有掃描被接受——NDT 就會被鎖在關閉狀態並維持下去。沒有任何東西會重試。
此時 EKF 以 IMU 與輪速推算，`/localization/kinematic_state` 照樣以 40 Hz 運行，車輛也照樣
在地圖上移動。這幅畫面裡沒有任何東西宣告失敗。

「還沒有掃描被接受」這種情況有它自己的陷阱：當感測器到 `base_link` 的轉換失敗，或掃描的
最大點距離小於 `sensor_points.required_distance`（10 m）時，掃描回呼會在登錄點雲之前就返回。
一份短距或無法轉換的掃描會安靜地餓死初始化。

## 重播或行駛進行中時

| 腳本 | 它回答的問題 |
|---|---|
| `check_ndt_activated.py` | 匹配器有被啟用嗎，而不只是活著？ |
| `ndt_quality_report.py` | 位姿好嗎——散布、yaw 步進、init-to-result、執行時間？ |
| `ndt_alignment_report.py` | 掃描真的貼在地圖上嗎，逐點來看？ |
| `ndt_timeseries.py` | 定位撞牆時，是哪個訊號先動的？ |
| `check_imu_velocity.py` | 建構先驗的那兩個訊號值得信任嗎？ |
| `monitor_localization.py` | 此刻 GNSS 位姿與 NDT 位姿差多遠？ |
| `monitor_gps.py` | GNSS 有可用的定位嗎，它把我們放在地圖的哪裡？ |
| `check_map_bounds.py` | 我們到底在不在已建圖的區域內？ |
| `log_localization_data.py` | 把 GNSS 與定位記成 CSV 供日後分析。 |
| `launch_monitors.sh` | 三個監看工具一次啟動，開在 tmux 工作階段裡。 |

### `ndt_quality_report.py` —— 位姿品質，刻意不看 NVTL

```bash
python3 scripts/testing/localization/ndt_quality_report.py --seconds 120 --label res2.0
```

記錄一段時間窗，然後印出 NDT *不會*拿來把關的指標：

- **散布** —— 每幀相對於局部平滑路徑的偏離。對著靜態地圖抖動的位姿，即使分數好也是錯的。
- **yaw 步進** —— 相鄰影格間的航向變化。大步進就是那些會被平均分數抹掉的翻轉與滑移失敗。
- **init to result** —— NDT 每幀把先驗搬多遠。小而穩定代表先驗好；大或持續增長代表先驗
  過期，那是融合問題，不是拿來調的參數。
- **執行時間** —— 這樣一個準確但對 Orin 而言太慢的組態才會被看見。

它刻意省略 NVTL。NVTL 隨 `ndt.resolution` 上升，也會在不完美的遠距回波被裁掉時再次上升，
所以最大化它是在挑選粗體素與窄 crop box，無論位姿是否改善。本專案自己的調校研究在改以
位姿品質重新量測後，兩個主要結論都被推翻。見
[NDT 調校](./ndt-tuning.md#nvtl-把關收斂不排序品質)。

### `ndt_alignment_report.py` —— 獨立於 NDT 的殘差

```bash
python3 scripts/testing/localization/ndt_alignment_report.py --seconds 60
```

取每一份進來的掃描，用系統當下發布的位姿把它轉到地圖座標系，然後量測每個掃描點到最近地圖
點的距離。那個殘差才是「有沒有對上」真正的意思，而且不像 NVTL，它不是由那個位姿受到質疑
的估計器自己算出來的。

NVTL 無法分辨「掃描在地圖上」與「掃描很自信地在地圖的錯誤位置上」。最近鄰殘差可以。

結果分開回報靜止與移動影格，因為它們回答不同問題：靜止的說初始收斂是否成功，移動的說追蹤
是否維持得住。

選項：`--map`（預設為 COSS 地圖）、`--topic`、`--max-radius`、`--label`。

### `ndt_timeseries.py` —— 哪個訊號先動

```bash
python3 scripts/testing/localization/ndt_timeseries.py --seconds 300 -o tmp/ndt
```

寫出 `tmp/ndt.csv` 與 `tmp/ndt.png`。為一個問題而生：定位失敗時，是*哪個*訊號先動？它繪製

- `iteration_num` —— 碰到 `max_iterations` 代表最佳化器在收斂前就用光預算。這是能取得的
  最早的誠實警告。
- NVTL 與 TP —— 當作相對於各自門檻的趨勢來讀，絕不要當成絕對品質。
- `initial_to_result_distance` —— 跳變代表先驗與掃描意見不合，那就是從內部看起來的
  「轉彎沒追好」。
- `skipping_publish_num` —— 連續被拒絕的結果數。超過上限時 NDT 會停用，而 EKF 接著在沉默中
  推算。
- 下方一併畫出 yaw rate 與速度，好讓發散能對上造成它的那個操控動作。

所有資料都蓋上模擬時鐘的時戳，所以 x 軸與 bag 時間、以及同一次重播記錄的任何其他東西都對得上。

### `check_imu_velocity.py` —— 稽核先驗的兩個輸入

```bash
python3 scripts/testing/localization/check_imu_velocity.py --seconds 200
```

EKF 先驗是 IMU 與 `VelocityReport` 的陀螺里程融合。當 `initial_to_result_distance` 行進時
很大、停車時很小，問題就在這裡，不在匹配器。這個腳本檢查那兩個訊號會出錯的三種方式：

- **座標。** IMU 是 REP-103 車體軸（x 前、y 左、z 上）嗎？停車時從重力讀出來——水平的
  REP-103 IMU 回報 az 接近 +9.81，正負號或軸向交換會立刻顯現。yaw rate 也必須與位姿共用
  同一個正負號約定，否則每個轉彎都會被反向積分。
- **尺度。** 在 NDT 仍在追蹤的時間窗內積分各訊號，與地圖產生的位姿比較。以比值回報，所以
  1.00 是正確，0.90 代表車輛實際移動了比輪速所述多 11 %。差幾個百分點在單幀上看不出來，
  而那正是讓先驗在一個轉彎中落後一公尺的原因。
- **品質。** 靜止時的陀螺偏差與雜訊、發布率與缺口。發布率下陷會讓 EKF 去外插。

比較時用的是仍然收斂時的 NDT 位姿，而不是 EKF 輸出——EKF 位於受測訊號的下游，用它來評分
會把共同誤差藏起來。`--raw-topic` 跟隨 `imu_source`。

### GNSS 監看工具

```bash
python3 scripts/testing/localization/monitor_gps.py           # 定位品質、地圖座標
python3 scripts/testing/localization/monitor_localization.py  # GNSS 位姿 vs NDT 位姿 vs 融合狀態
python3 scripts/testing/localization/check_map_bounds.py      # 我們在地圖內嗎？
./scripts/testing/localization/launch_monitors.sh             # 三個一起，開在 tmux
```

`monitor_gps.py` 顯示原始經緯度與高度、回報的精度，以及換算後的地圖座標。
`monitor_localization.py` 顯示 GNSS 與 NDT 之間的 2D 距離、高度差與航向差。
`check_map_bounds.py` 印出點雲地圖的範圍，以及目前位置距離邊界多遠。

在把 GNSS 當成任何東西的參照*之前*，先確立 GNSS 品質。沒有 RTK 的消費級 GNSS 只是一個粗略的
初始化種子，僅此而已。

`log_localization_data.py` 把同樣的訊號記成 CSV 供日後分析。

## 讓執行不需要人介入的位姿種子

調校意味著把同一份錄製檔跑很多次再比較。手動點的初始位姿會讓每次執行的起點稍有不同，而
那個差異會出現在結果裡，看起來像參數差異。

```bash
# 一次就好：在 RViz 中放置位姿，等 NDT 穩定，然後擷取它
python3 scripts/testing/localization/capture_initial_pose.py COSS

# 之後每次執行
python3 scripts/testing/localization/set_initial_pose.py COSS
```

位姿會寫到 `data/initial_poses/<name>.yaml`，取自 `/localization/kinematic_state`——也就是
NDT 收斂且 EKF 穩定之後，而不是那一下原始點擊。檔案會記錄它*來自哪個*來源，因為兩者存檔後
看起來一模一樣，價值卻非常不同：在 planning simulator 下，位姿就是那一下點擊原封不動被當成
真值回報；在重播下，它是 NDT 與地圖取得一致的結果。兩者都能當種子，只有後者是量測。

`set_initial_pose.py` 呼叫 `/localization/initialize`，也就是 RViz 的 2D Pose Estimate 透過
ADAPI 轉接層所觸及的同一個服務。直接發布 `/initialpose3d` **不會**初始化 Autoware——
`pose_initializer` 是發布那個主題，而不是監聽它。方法是 AUTO 而非 DIRECT，所以擷取到的位姿
會當作起始猜測交給估計器，由 NDT 對著地圖去修正它——那是種子與斷言之間的差別。

選項：擷取時的 `--settle` 與 `--max-drift`，重放時的 `--timeout`。

## 記錄一次執行，以及事後如何讀它

```bash
scripts/testing/localization/run-ndt-replay.sh <label>
POSE_SOURCE=ndt scripts/testing/localization/run-ndt-replay.sh builtin
```

把 COSS 錄製檔透過某個位姿估計器重播，並記錄下所有能分辨初始化失敗與追蹤失敗所需的東西：
每個 NDT 診斷主題錄成 rosbag、完整的啟動輸出，以及 `/diagnostics`、GNSS、EKF 與 kinematic
state 供交叉比對。輸出落在 `tmp/ndt-replay/<label>_<timestamp>/`。

這份錄製檔前 115.7 s 是停車、最後 41.3 s 在行駛，所以一次執行涵蓋兩個階段。各節點的輸出一如
往常在 `play_log/latest/node/<name>/`——去讀*其他*節點的 log，不只是匹配器的。

`just demo run` 把同樣的想法連同資料取得、位姿種子與指標報告包在一起，並把執行結果寫到
`tmp/demo-runs/`。`just demo report`、`just demo compare` 與 `just demo list-runs` 都針對那些
結果操作。

| 腳本 | 它回答的問題 |
|---|---|
| `summarize_ndt_run.py` | 單次執行的診斷，拆成初始化與追蹤兩階段。 |
| `compare_ndt_runs.py` | 多次執行並排：散布、yaw 抖動、預測誤差、分數。 |
| `ndt_yaw_bias.py` | 有沒有一個恆定的航向偏移——也就是安裝校正誤差？ |
| `ndt_benchmark_report.py` | 兩個匹配器做的是同樣的工作嗎，誰比較快、比較省？ |
| `export_ndt_frames.py` | 匯出確切的（掃描、先驗）對，供離線比較匹配器。 |
| `tegrastats_summary.py` | 在 Jetson 上的 GPU 忙碌度、電軌功率與 CPU 負載。 |
| `mapcheck/map_coverage.py` | 地圖到底有沒有延伸到 crop box 那麼遠？ |
| `mapcheck/map_agreement.py` | 有覆蓋的地方吻合嗎——而不吻合是地圖的問題還是樹葉的問題？ |

### `summarize_ndt_run.py` 與 `compare_ndt_runs.py`

```bash
python3 scripts/testing/localization/summarize_ndt_run.py <run_dir>
python3 scripts/testing/localization/compare_ndt_runs.py a=<run_dir> b=<run_dir>
python3 scripts/testing/localization/compare_ndt_runs.py --row a=<run_dir>   # 一行 TSV
```

摘要中資訊量最大的就是 init 與 track 的拆分。停車時匹配很容易，因為每一幀掃描都像上一幀；
一旦車輛移動，同樣的工作大約要花兩倍成本。條件邊緣的機器會在那裡失敗，而不是在啟動時。

讀一次執行意味著掃過數 GB 的 rosbag，所以多次執行值得並行萃取——那就是 `--row` 的用途，也是
`just demo compare` 所扇出的東西。

### `ndt_yaw_bias.py` —— 航向減去對地航跡角

```bash
python3 scripts/testing/localization/ndt_yaw_bias.py label=<run_bag_dir>
```

直行的車輛，航向等於航跡角。恆定的差值代表 `base_link` 與 NDT 實際定位的那個感測器之間有
yaw 偏移——那是安裝校正誤差，不是定位 bug。它能通過每一項分數檢查，因為匹配本身是好的。

轉彎被排除，而那個排除正是重點：有限弦長在曲線中會系統性地落後瞬時航向，製造出一個根本不
存在的偏差。在 COSS 錄製檔上，限制在直線路段後，估計值從混淆的 -11.5 度變成乾淨的 -12.66 度。

### `ndt_benchmark_report.py` 與 `export_ndt_frames.py`

即時重播無法公平比較兩個匹配器。較慢的那個會錯過掃描預算、丟掉掃描，而每一個被丟掉的掃描都
留下更舊的先驗，分數更差，於是未通過收斂門檻——所以被量到的是一場崩潰，而不是速度比。

`export_ndt_frames.py` 寫出系統實際使用的（掃描、初始猜測）對，連同地圖，放進一個扁平檔案。
把那組完全相同的序列離線餵給每一方，「做同樣的工作」就成了結構上的保證，而不是事後才要驗證
的東西。

```bash
python3 scripts/testing/localization/export_ndt_frames.py --run tmp/demo-runs/<run> --out tmp/ndt-frames.bin
python3 scripts/testing/localization/ndt_benchmark_report.py gpu=<dir> cpu=<dir>
```

`ndt_benchmark_report.py` 回報速度、成本與**等價性**——迭代次數與分數——因為如果兩個組態並未
收斂到同一個地方，速度比較就毫無意義。

### 在 Jetson 上

```bash
python3 scripts/testing/localization/tegrastats_summary.py <run_dir>/tegrastats.log --skip-seconds 60
```

`play_launch` 的每節點 GPU 使用率與功率在 Jetson 上是 `nan`：它們來自 NVML，而 Tegra 沒有
實作它。`tegrastats` 是唯一來源，而在 Orin 上功率是第一類數字——把 NDT 移到 GPU 的理由是它在
固定功耗預算內釋放 CPU，沒有電軌數據就無法檢驗這一點。`--skip-seconds` 把地圖載入與 TensorRT
暖機排除在穩態數字之外。

### 檢查地圖本身

```bash
python3 scripts/testing/localization/mapcheck/map_coverage.py --run <run_dir>
python3 scripts/testing/localization/mapcheck/map_agreement.py --run <run_dir>
```

在放寬量測範圍 crop box 之前，`map_coverage.py` 是該問的第一個問題：地圖到底有沒有延伸那麼遠？
超出已建圖區域的掃描點無法約束位姿，而且因為降採樣濾波器不論距離都保留固定點數，未建圖的遠距
回波會*稀釋*那些真正帶著約束的點。它很便宜——2 m 網格上的覆蓋測試，不做最近鄰搜尋。

`map_agreement.py` 則從跨多個觀測位姿的一次最近鄰掃描中回答後續問題：

- **地面還是植被？** 地面不會在建圖與錄製之間移動，所以位移的地面回波代表地圖在那裡是錯的。
  樹葉會隨風與季節移動，而且在每個距離上都不吻合。
- **地圖變形還是樹葉在動？** 若多個車輛位姿看到同一個地圖網格以相同方式位移，那個位移屬於
  地圖——建圖時的 SLAM 漂移。若每個觀測者看到的都不一樣，那是雜訊。

## 哪個症狀用哪個腳本

| 症狀 | 從這個開始 |
|---|---|
| `/localization/pose_estimator/pose_with_covariance` 上完全沒有位姿 | `check_ndt_activated.py` |
| 位姿存在，但點雲相對地圖在滑動 | `ndt_alignment_report.py` |
| 車輛停著時位姿在抖 | `ndt_quality_report.py` |
| 本來追得好，中途發散 | `ndt_timeseries.py` |
| `initial_to_result_distance` 行進時大、停車時小 | `check_imu_velocity.py` |
| 直路上航向偏掉一個常數 | `ndt_yaw_bias.py` |
| 離開地圖中心後定位變差 | `mapcheck/map_coverage.py`，接著 `map_agreement.py` |
| 這裡可以、上車就不行 | `tegrastats_summary.py`，接著 `ndt_benchmark_report.py` |
| 你什麼都沒改，連續多次執行卻不一致 | `capture_initial_pose.py` / `set_initial_pose.py` |

## 相關資料

- [NDT 調校](./ndt-tuning.md) —— 拿到答案之後要做什麼
- [定位方法](./localization-methods.md) —— `ndt`、`cuda_ndt`、`mcl`
- [地圖](./maps.md) —— 在怪罪匹配器之前先驗證地圖
- [檢視執行中的系統](../concepts/inspecting.md) —— 這些工具所建立於其上的通用 `ros2` 工具
- 這些腳本自己的參考文件是儲存庫中的
  [`scripts/testing/localization/README.md`](https://github.com/NEWSLabNTU/AutoSDV/blob/main/scripts/testing/localization/README.md)
