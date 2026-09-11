<!--
Translation Metadata:
- Source file: 01-first-run.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 1. 第一次執行

一條指令。它會取得資料、把整個堆疊帶起來、用真實錄製的 LiDAR 對著真實地圖定位、
重播一段行駛，並印出它做得多好。

先執行它，再理解它。親眼看到成品運作，比讀關於它的說明更有價值，而本頁之後的一切
都在拆解它。

## 先檢查

```bash
cd ~/AutoSDV
just demo check
```

預期輸出：

```
COSS NDT demo prerequisites:
  ok   rosbag  …/data/rosbags/outdoor_20251226_153115
  ok   map     …/data/COSS-map-planning
  ok   workspace built
  ok   cuda_ndt_matcher is a release build (15 MB)
  ok   play_launch
```

出現 `warn no DISPLAY; RViz disabled (headless is fine)` 不是問題——它表示你在一台
沒有圖形工作階段的機器上，執行會在沒有視覺化的情況下繼續。

如果 rosbag 不存在，下一個指令會下載它（約 2.8 GB）。如果工作空間尚未建置，請回到
[安裝](../getting-started/installation/overview.md)。

## 執行

```bash
just demo run
```

這需要幾分鐘，其中大部分是 157 秒的回放。它依序做以下事情：

1. 若 rosbag 不存在則取得它
2. 停掉上一次執行殘留的堆疊——兩個堆疊會爭搶同樣的主題
3. 以 `pose_source:=cuda_ndt` 與 `use_gnss:=false` 啟動記錄回放模擬
4. 等待 `ndt_scan_matcher` 出現，再給那張 490 萬點的地圖 25 秒載入
5. 啟動輪速縮放器
6. 錄製所有定位診斷
7. 回放 bag，並在第 8 秒植入一個已知的起始姿態
8. 印出量測數據，並**讓堆疊繼續執行**，以便你檢視

## 你會看到什麼

先是設定訊息：

```
[demo] output   …/tmp/demo-runs/coss-ndt_20260912_054957
[demo] pose_source=cuda_ndt  use_gpu=1  rviz=false  scale=0.5  seed_pose=true  rate=1.0
[demo] stack pgid=1184896  (just demo stop)
[demo] initial pose will be seeded 8s into playback
[demo] replaying 157s: parked ~115s, then a 41s drive
```

!!! warning "前兩分鐘什麼都不會動，而那是正確的"

    錄製長 157 秒，而車輛**直到第 116 秒才開始移動**。它在感測器運作時停著。兩分鐘
    的靜止車輛是這段錄製本身，不是失敗。

接著是報告。以下數字來自一台桌機上的真實執行，所以你的會不同，但形狀不該不同：

```
bag sim-time span: 1766734277.2 .. 1766734433.9
first motion (>0.2 m/s) at t=1766734393.5 (+116.3s)

nvtl:
  all     n=1412 mean=4.595 min=4.343 p50=4.589 p95=4.725 max=4.808

exe_ms:
  all     n=1412 mean=10.355 min=1.241 p50=5.470 p95=32.823 max=147.804
  init    n=1010 mean=7.814
  track   n=402  mean=16.739 p95=44.272

published ndt poses: 1395   ekf: 5222
ndt publish gaps [s]: n=1394 mean=0.100 p50=0.100 p95=0.101 max=0.200
```

有三個數字值得讀，因為它們告訴你它成功了：

| | 意義 |
|---|---|
| **`nvtl` 平均 4.6** | 每個掃描與地圖匹配得多好。越高越好；持續偏低表示車輛不知道自己在哪 |
| **`exe_ms` p95** | 一次掃描匹配花多久。預算是 100 毫秒，因為 LiDAR 是 10 Hz。持續超過就表示定位跟不上車輛 |
| **`ndt publish gaps` 平均 0.100** | 姿態以 10 Hz 輸出，與感測器相符。這是最關鍵的結果：定位跟上了 |

注意 `init` 與 `track` 的差別。車輛停著時匹配容易也快（平均 7.8 毫秒）。一旦開始
移動，同樣的工作要花兩倍時間（16.7 毫秒），因為掃描不再像上一個。這個分割是整份
報告中最有資訊量的東西。

## 停止

堆疊還在執行，好讓你檢視。完成後：

```bash
just demo stop
```

請用這個，而不是 `Ctrl-C` 或 `kill`。它會殺掉整個**行程群組**——啟動器、容器，以及
裡面的可組合節點。只殺啟動器會留下仍佔著記憶體與 GPU 的孤兒行程。之後可以確認：

```bash
ros2 node list    # 應為空
```

## 事後查看那次執行

```bash
just demo report        # 再次顯示最近一次的量測數據
just demo list-runs
```

每次執行都寫到 `tmp/demo-runs/<label>_<timestamp>/`，內含錄製的診斷 bag、匹配器
量測 CSV、啟動紀錄與節點清單。每次都有數 GB；`just demo clean` 會刪除它們。

## 如果它沒成功

**完全沒有東西啟動。** 核心 socket 緩衝區——這會擋住每一個 ROS 2 節點，不只這個
demo：

```bash
./setup.sh --rerun cyclonedds-sysctl
```

**`command not found`。** 你的終端機沒有環境。參閱
[環境與相依套件](../concepts/environment.md)。

**感知模組花了好幾分鐘啟動。** TensorRT 正在節點建構子內編譯引擎。它會被快取，所以
只會發生一次——除非模型目錄不可寫入，那樣它會在每一次啟動時發生：

```bash
just setup-autoware-data
just build-engines
```

**它跑起來了，但 NVTL 很低而且姿態亂飄。** 植入的姿態沒有收斂。
`just demo run-manual-init` 讓你自己在 RViz 中設定。

## 剛才發生了什麼

你執行的是**記錄回放模擬**：真實的 Autoware 堆疊，以錄製的感測器資料取代即時硬體，
全程自行估計位置。

`just demo run` 是一層包裝。它藏起了啟動指令、參數、播放 rosbag 的第二個終端機，
以及姿態植入。這份教學接下來會把它們一一揭開，從比較簡單的那種模擬開始。

**接下來：** [2. 路徑規劃模擬](./02-planning-simulation.md) —— 同樣的堆疊，但拿掉
感測與定位，讓你單獨看規劃器。
