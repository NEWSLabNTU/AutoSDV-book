<!--
Translation Metadata:
- Source file: 03-logging-simulation.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 3. 記錄回放模擬

與上一頁相同的堆疊，但把感測、定位與感知放了回去——由一段真實行駛的錄製餵入，而不是
硬體。

有一件事改變了，而它改變了一切：**車輛不再被告知自己在哪裡。** 它必須每秒十次，靠
把 LiDAR 回波與地圖匹配來推算。那個估計可能變慢、出錯或完全失去，而看著它成功或失敗
正是這一頁的用意。

這就是[步驟 1](./01-first-run.md) 替你執行的東西。

## 你需要什麼

```bash
just bag download    # 約 2.8 GB，只需一次
```

錄製會落在 `data/rosbags/outdoor_20251226_153115`。`just demo run` 會自動取得它，
所以你可能已經有了。

## 啟動 —— 兩個終端機

兩個終端機都需要[環境](../concepts/environment.md)。

**終端機 1 —— 堆疊：**

```bash
source /opt/autoware/1.5.0/setup.bash
source install/setup.bash

play_launch launch autosdv_launch logging_simulation.launch.yaml
```

等它起來。點雲地圖有 490 萬個點，需要數十秒載入；掃描匹配器在它載完之前什麼都做不了。
太早開始播放錄製只是浪費它的開頭。

**終端機 2 —— 資料：**

```bash
source /opt/autoware/1.5.0/setup.bash
source install/setup.bash

ros2 bag play data/rosbags/outdoor_20251226_153115 --clock
```

!!! danger "`--clock` 不是選用的"

    `logging_simulation.launch.yaml` 設定了 `use_sim_time:=true`，所以整個堆疊跑在
    取自 `/clock` 主題的模擬時間上。沒有 `--clock`，就沒有東西發佈那個主題，時鐘
    永遠不會前進，系統就只是坐在那裡什麼都不做——而且沒有任何錯誤。

    這是本頁最常見的失敗。如果什麼都沒發生，先檢查這個：

    ```bash
    ros2 topic echo /clock --once
    ```

??? note "`just` 捷徑"

    ```bash
    just sim logging
    just sim logging ARGS="pose_source:=ndt"
    ```

## 如果你沒有 NVIDIA GPU

預設值假設你有：`pose_source:=cuda_ndt` 需要 CUDA，而感知會載入 TensorRT 模型。
兩個參數就能移除這兩個需求：

```bash
play_launch launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=ndt \
  launch_perception:=false
```

- `pose_source:=ndt` 選用 Autoware 以 OpenMP 實作的 **CPU** 掃描匹配器。每個掃描
  較慢，但正確。
- `launch_perception:=false` 發佈空的物件清單，而不載入任何模型。定位並不依賴感知，
  所以如果你是來看定位的，這不會有任何損失。

在一台桌上型 CPU 上以 `pose_source:=ndt` 量測，匹配器維持了感測器完整的 10 Hz，
行駛階段的 p95 為 44 毫秒，預算是 100 毫秒。在筆電上預期會更差；如果姿態開始落後
車輛，那就是預算用盡的樣子。

## 植入初始姿態

NDT 需要一個起點。它會精修一個估計值，而不是搜尋整張地圖。

- **在 RViz 中** —— 用 `2D Pose Estimate`，就像路徑規劃模擬那樣，點在錄製起始位置
  附近。
- **自動** —— `just demo run` 會在回放開始 8 秒時發佈一個已知姿態，那正是它的結果
  可重現的原因。沒有那個植入，同一段錄製跑兩次可能不同。

錄製本身的 GNSS *不會*被使用（`use_gnss:=false`）：它是單點定位、約有 20 公尺的
散佈，而且與行進方向不一致，所以讓它來初始化定位會讓車輛每次落在不同地方。那是一個
真實的問題，不是這份資料集的怪癖。

## 觀看

!!! warning "車輛在前 116 秒是停著的"

    錄製長 157 秒。第一次超過 0.2 m/s 的移動在 **+116.3 秒**，接著行駛 41 秒，最高
    1.58 m/s。兩分鐘的靜止車輛就是這段錄製。

**在 RViz 中**，要看的是即時點雲與地圖的關係。如果定位正常運作，點雲會*貼在*地圖
上——牆對齊牆。如果失敗，點雲會相對地圖滑動，或以固定偏移停在旁邊。那個視覺判斷比
任何節點自己回報的狀態都可靠。

**在第三個終端機中**，看數字：

```bash
ros2 topic hz /localization/pose_estimator/pose_with_covariance
```

預期約 10 Hz，與 LiDAR 相符，因為 NDT 每個掃描跑一次。明顯低於它就表示匹配器跟不上。

AutoSDV 附了用來分辨「節點在跑」與「定位正常運作」的檢查，這兩者並不是同一個主張：

```bash
python3 scripts/testing/localization/check_ndt_activated.py
python3 scripts/testing/localization/ndt_quality_report.py
python3 scripts/testing/localization/ndt_alignment_report.py
```

## 讀懂結果

`just demo run` 印出的量測數據——以及這些腳本產生的——值得理解一次。

| 指標 | 意義 | 健康的樣子 |
|---|---|---|
| **NVTL** | 掃描與地圖匹配得多好 | 穩定，沒有崩落 |
| **TP** | 另一個匹配分數 | 穩定 |
| **`exe_ms`** | 一次掃描匹配的時間 | p95 遠低於 100 毫秒 |
| **iterations** | 每次匹配的最佳化步數 | 個位數低值 |
| **publish gap** | 兩次姿態之間的時間 | 約 0.100 秒 |

最有資訊量的分割是 **`init` 與 `track`**：停著的時候匹配容易，因為每個掃描都像上
一個。一旦車輛移動，同樣的工作大約要花兩倍。實測的一次執行：停著 7.8 毫秒、行駛
16.7 毫秒、p95 44 毫秒。如果一台機器處於邊緣，它會在那裡失敗——而不是在啟動時。

## 疑難排解

**播放 bag 時什麼都沒發生。** `--clock`。見上文。

**姿態始終沒出現，或跳到原點。** 沒有設定初始姿態，或設定的位置離真實位置太遠，使
NDT 無法收斂。請重新設定，並靠近一些。

**點雲在中途漂離地圖。** 匹配失去了。注意 NVTL——在視覺上的漂移變明顯之前，它就已經
崩落了。

**主題存在但沒有人訂閱。** 比對兩端的 QoS：

```bash
ros2 topic info -v /sensing/lidar/concatenated/pointcloud
```

`RELIABLE` 的訂閱者不會從 `BEST_EFFORT` 的發佈者收到東西。參閱
[檢視執行中的系統](../concepts/inspecting.md)。

**感知要花好幾分鐘啟動。** TensorRT 正在編譯引擎。執行一次
`just setup-autoware-data`，接著 `just build-engines`——少了前者，引擎無法被快取，
它會在*每一次*啟動時發生。

## 比較各種定位方法

回放是比較它們的正確場合，因為每次執行的輸入逐位元組完全相同：

```bash
play_launch launch autosdv_launch logging_simulation.launch.yaml pose_source:=cuda_ndt
play_launch launch autosdv_launch logging_simulation.launch.yaml pose_source:=ndt
play_launch launch autosdv_launch logging_simulation.launch.yaml pose_source:=mcl
```

`mcl` 需要的是佔據網格而非點雲地圖，以及 2D `LaserScan` 而非 3D 點雲。參閱
[定位方法](../guides/localization-methods.md)。

## 你現在看過了什麼

| | 路徑規劃模擬 | 記錄回放模擬 |
|---|---|---|
| 姿態 | 給定 | **估計，而且可能失敗** |
| 物件 | 你放置的 | 由真實回波偵測 |
| 感測器資料 | 沒有 | 一段真實錄製 |
| 最難的部分 | 挑一個好目標 | 維持定位 |

**接下來：** [4. 拆解那條指令](./04-behind-the-recipe.md) —— `just demo run` 一直
以來到底在做什麼。
