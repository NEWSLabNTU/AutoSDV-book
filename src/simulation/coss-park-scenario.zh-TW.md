<!--
Translation Metadata:
- Source file: coss-park-scenario.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# COSS Park 情境

完整的定位堆疊，對著真實錄製資料回放，全部由一條指令完成——連取得資料都包含
在內。這是 AutoSDV 中最接近「可以親眼看著跑」的回歸測試。

## 這份錄製

`data/rosbags/outdoor_20251226_153115`，約 2.8 GB，在 COSS Park 錄製的 157 秒：
前 115.7 秒停著不動，接著是 41 秒、最高 1.58 m/s 的行駛。完全不需要車輛——整件
事都從錄製資料跑起來。

## 執行

```bash
just demo check        # 先決條件是否齊備？
just demo run          # 全部
```

`just demo run` 會依序做以下事情：

1. 若 rosbag 不存在則取得它（約 2.8 GB），並檢查地圖是否存在
2. 停掉上一次執行殘留的堆疊——兩個堆疊會爭搶同樣的主題
3. 以 `pose_source:=cuda_ndt`、`use_gnss:=false` 啟動
   `logging_simulation.launch.yaml`，並在 `$DISPLAY` 有設定時開啟 RViz
4. 等待 `ndt_scan_matcher`，再給那張 490 萬點的地圖 25 秒載入
5. 啟動輪速縮放器，並將 bag 的原始速度主題重新導向經過它
6. 將所有定位診斷錄製到 `tmp/demo-runs/<label>_<stamp>/bag`
7. 回放 bag，並在第 8 秒植入初始姿態
8. 印出量測數據，並**讓堆疊繼續執行**，以便你檢視結果

結束後停止它：

```bash
just demo stop
```

它會殺掉整個行程群組。這比聽起來更重要：只用 PID 殺掉啟動器會讓可組合節點的
容器變成孤兒，而 `play_launch` 自己的包裝行程又經常能撐過群組訊號——這正是為何
有這個 recipe，而不是在文件裡寫一行 `kill`。

## 變體

```bash
just demo run-headless      # 不開 RViz，且不讓堆疊留著
just demo run-manual-init   # 不植入姿態——自己在 RViz 裡設
just demo run-raw-speed     # 使用未修正的原始輪速，看看它造成的頓挫
```

`run-manual-init` 是比較誠實的版本：`demo run` 中植入的姿態正是它可重現的原因，
拿掉它就能看出結果有多少倚賴一個好的初始猜測。

`run-raw-speed` 的存在是為了*展示*一個真實的缺陷而非掩蓋它——錄製的輪速需要
縮放，沒有縮放時姿態會頓挫。

## 讀取結果

```bash
just demo report              # 最近一次執行的量測數據
just demo report run_dir=...  # 指定某次執行
just demo list-runs
just demo compare a=<dir> b=<dir>
```

`compare` 就是回歸測試的工作流程：在改動前錄一次、改動後再錄一次，然後並排
比較。因為輸入是固定的錄製資料，輸出的差異就是程式碼的差異。

另外兩個較專門的報告：

```bash
just demo map-quality    # 地圖有涵蓋到掃描嗎？有涵蓋之處兩者吻合嗎？
just demo yaw-bias       # 某次執行中，車頭朝向與行進方向的偏航偏差
```

每次執行都有數 GB。`just demo clean` 會刪除它們。

## 比較掃描匹配器

```bash
just demo bench                 # GPU 上的 cuda_ndt、同一份程式碼跑 CPU，以及 Autoware 的版本
just demo bench-offline         # 以完全相同的錄製輸入做離線 GPU-CPU 比較
just demo bench-nvtl-probe      # 兩組在相同姿態下的 NVTL 評分一致性
just demo bench-report <tsv>    # 從已錄製的執行重新產生報告
```

儲存庫把 `bench-offline` 描述為「公平的那一個」——它餵給兩邊完全相同的錄製輸入，
而不是讓它們各自即時執行，而這是讓比較有意義的唯一方式。

## 較低階的版本

`just sim coss-park` 以較粗糙的方式執行同一個情境——記錄回放模擬、bag，以及一個
定位錄製器，透過 GNU `parallel` 一起啟動並以固定的 sleep 等待：

```bash
parallel --line-buffer ::: \
    "just sim logging" \
    "sleep 40 && ros2 bag play data/rosbags/outdoor_20251226_153115/ --clock -l -r 1.0" \
    "sleep 45 && ./scripts/rosbag/record_localization.sh"
```

它需要 GNU `parallel`、寫死了那個 bag 路徑，而且是靠時鐘而非就緒狀態等待——因此
在較慢的機器上，bag 可能在地圖載入完成前就開始播放。`just demo run` 改為等待
掃描匹配器，這也是為什麼該用它。

## 後續步驟

- [資料集與 Rosbag](./datasets.md)
- [定位方法](../guides/localization-methods.md)
