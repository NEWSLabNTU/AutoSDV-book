<!--
Translation Metadata:
- Source file: coss-park-scenario.md
- Last synced: 2026-03-30
- Translator: Claude (Anthropic)
- Status: Complete
-->

# COSS 公園場景

COSS 公園場景使用來自台大 COSS 公園的錄製感測器資料執行全堆疊模擬。它會啟動 Autoware 堆疊、播放 rosbag 並錄製定位輸出 — 全部只需一個指令。

**先決條件**：

- 完成[推薦安裝方式](../getting-started/installation/recommended.md)
- 下載測試 rosbag：`just download-data`
- 安裝 GNU parallel：`sudo apt install parallel`

## 啟動

```bash
just sim-coss-park
```

這會平行執行三個程序：

1. **記錄模擬** — 啟動完整的 Autoware 堆疊（`just launch-sim-logging`）
2. **Rosbag 播放** — 以 1 倍速循環播放 COSS 公園錄製（40 秒後開始）
3. **定位錄製** — 錄製 `/localization/pose_estimator/*` 主題 60 秒（45 秒後開始）

延遲確保 Autoware 堆疊在資料開始流入前完全啟動。

## 測試內容

與手動啟動和播放的[記錄模擬](./logging-simulation.md)不同，此場景自動化整個流程並擷取定位輸出供分析。適用於：

- 驗證定位堆疊的端對端運作
- 比較不同程式碼變更的定位效能
- 修改啟動檔案或參數後的回歸測試

## 檢查結果

定位錄製儲存在 `rosbags/localization_test_<timestamp>/`。使用以下指令檢查：

```bash
ros2 bag info rosbags/localization_test_*/
```

視覺化錄製的位置姿態軌跡：

```bash
just tool-plotjuggler
```

在 PlotJuggler 中載入錄製的 bag，繪製 `/localization/pose_estimator/pose_with_covariance` 以查看估計軌跡。

## 範圍檢查

使用 `play_launch dump` 檢查啟動圖而不執行任何節點：

```bash
play_launch dump -o tmp/scope.json \
    launch autosdv_launch logging_simulation.launch.yaml \
    pose_source:=ndt

play_launch context tmp/scope.json --tree
```

這會顯示完整的 include 樹和節點層級結構，有助於驗證啟動檔案的變更是否產生預期的結構。

## 疑難排解

**`parallel: command not found`**

安裝 GNU parallel：

```bash
sudo apt install parallel
```

**找不到 Rosbag**

先下載測試資料：

```bash
just download-data
```

**定位錄製為空（0 則訊息）**

若 NDT 在錄製期間未初始化，可能會發生此情況。檢查 NDT 日誌：

```bash
cat play_log/latest/node/ndt_scan_matcher/err
```

若 NDT 報告「No InputSource」，點雲前處理節點可能載入失敗。嘗試重新啟動場景。

## 下一步

- [資料集與 Rosbag](./datasets.md) — 可用的錄製資料與建立方式
- [操作車輛](../getting-started/usage.md) — 即時駕駛的啟動參數
