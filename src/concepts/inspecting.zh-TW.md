<!--
Translation Metadata:
- Source file: inspecting.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 檢視執行中的系統

當某個東西不動時，問題幾乎總是兩者之一：

1. **它有在發佈嗎？頻率對嗎？**
2. **發佈者明明在，為什麼我的訂閱者什麼都收不到？**

本頁就是回答這兩個問題的那一小組指令，也是你在 AutoSDV 上真正天天會用的。

這裡每個指令都需要一個已載入環境的終端機——參閱[環境與相依套件](./environment.md)。

## 有什麼在執行

```bash
ros2 node list
ros2 topic list
```

`ros2 node list` 也是孤兒行程檢查。停止系統後它應該是空的；任何仍列出的項目，都是
比它的啟動器活得更久的行程。

若只看某個節點發佈什麼、訂閱什麼、提供什麼：

```bash
ros2 node info /localization/pose_estimator/ndt_scan_matcher
```

## 它有在以正確頻率發佈嗎？

```bash
ros2 topic hz /sensing/lidar/concatenated/pointcloud
```

這會持續印出量測到的頻率直到你停止它。它是這份清單中最有用的一個指令，因為在車上
多數故障是*頻率*故障而不是缺失故障——資料在，但比消費端需要的更慢或更不規則。

AutoSDV 的一些已知正常數值：

| 主題 | 預期 |
|---|---|
| LiDAR 點雲 | 感測器的頻率——VLS128 與 Robin-W 為 10 Hz |
| `/localization/pose_estimator/pose_with_covariance` | 與 LiDAR 相同，因為 NDT 每個掃描跑一次 |
| IMU | 快得多，依來源通常是 100–400 Hz |

`ros2 topic hz` 也會回報標準差與最小／最大間隔。平均正確但離散度很大的頻率，和單純
偏低的頻率是不同的問題，前者通常指向 CPU 競爭。

兩個相關指令：

```bash
ros2 topic bw /sensing/lidar/concatenated/pointcloud   # 頻寬
ros2 topic delay /some/topic                            # header 時間戳 vs 現在
```

`ros2 topic delay` 只對帶 header 的訊息有效，它量的是*延遲*——當頻率看起來正常但
系統表現得像資料過時時，那正是你要的。

## 為什麼什麼都沒收到？

首先，到底有沒有人連上：

```bash
ros2 topic info -v /sensing/lidar/concatenated/pointcloud
```

`-v` 是關鍵。沒有它你只得到數量；有它你會得到每個發佈者與訂閱者，**以及各自的 QoS
設定**。那份輸出就是多數「主題存在但我的節點看不到」問題的答案。

### QoS，以及那個會咬人的不相容

ROS 2 讓每個發佈者與訂閱者各自宣告一組 Quality of Service。其中兩個設定造成了幾乎
所有的麻煩。

**可靠性（Reliability）。** `RELIABLE` 會重傳直到送達；`BEST_EFFORT` 不會。這條
規則是不對稱的，值得背下來：

| 發佈者 | 訂閱者 | 會連上嗎？ |
|---|---|---|
| RELIABLE | RELIABLE | 會 |
| RELIABLE | BEST_EFFORT | 會 |
| **BEST_EFFORT** | **RELIABLE** | **不會** |
| BEST_EFFORT | BEST_EFFORT | 會 |

reliable 的訂閱者不會連上 best-effort 的發佈者。感測器資料通常以 best-effort 發佈
——丟掉一個 LiDAR 掃描比延遲一個好——所以要求可靠性的工具或節點會靜默地收不到東西。
沒有錯誤訊息。主題列得出來、發佈者看得到，就是沒有訊息送達。

**耐久性（Durability）。** `TRANSIENT_LOCAL` 會為稍後加入的訂閱者保留最後一則
訊息；`VOLATILE` 不會。鎖存的資料會用它——地圖、`/tf_static`。對一個
transient-local 主題要求 `VOLATILE` 的訂閱者會連上，然後永遠等待一則早已送出的
訊息。

所以當 `ros2 topic info -v` 顯示有發佈者也有訂閱者卻沒有任何交換時，先比較它們的
兩段 QoS，再去看別的地方。

!!! note "QoS 在這裡不只是除錯問題"

    在高頻率主題上要求可靠性是有實際代價的。本專案中曾量測到一個把點雲以 RELIABLE
    發佈的 LiDAR 驅動程式丟棄了大約 60% 的感測器點數，因為那個可靠的 writer 阻塞
    了同一條負責解析進入封包的執行緒。解法是改為 best-effort。如果你要為某個感測器
    主題選 QoS，那就是先例。

## 看資料本身

```bash
ros2 topic echo /localization/pose_estimator/pose_with_covariance
ros2 topic echo /vehicle/status/velocity_status --once
ros2 topic echo /some/topic --field header.stamp
```

`--once` 印一則就結束，那是你面對任何大型訊息時想要的。沒有它去 echo 一個點雲會
讓你的終端機每秒被灌進好幾 MB。

如果 `echo` 什麼都沒印，那就是上面的 QoS 問題——`echo` 會挑一組設定，而它可能挑到
不相容的那組。`ros2 topic echo --qos-reliability best_effort <topic>` 是快速測試。

## 節點參數

```bash
ros2 param list /node_name
ros2 param get /node_name parameter_name
ros2 param set /node_name parameter_name value
```

`ros2 param get` 是你確認某個啟動參數真的到達節點的方法——也就是
[啟動檔](./launch-files.md)所描述那條鏈的終點。你傳了但在這裡看不到的值，就是沒有
送達。

## 座標轉換

```bash
ros2 run tf2_tools view_frames        # 產生整棵 TF 樹的 PDF
ros2 run tf2_ros tf2_echo base_link map
```

缺少座標轉換是「系統沒報錯卻產不出有用結果」的常見成因。尤其 `/tf_static` 在錄製
bag 時很容易漏掉，而它的缺席會讓所有 frame 關係無法解析。

## 診斷

Autoware 的節點在 `/diagnostics` 上發佈健康狀態：

```bash
ros2 topic echo /diagnostics
```

它比乍看之下更有用——那是節點說出「我活著但還沒收斂」的地方，而那正是「定位在跑」
與「定位正常運作」之間的差別。

AutoSDV 為定位這個情況附了專用檢查：

```bash
python3 scripts/testing/localization/check_ndt_activated.py    # 已啟用，還是只是活著？
python3 scripts/testing/localization/ndt_quality_report.py     # 姿態品質
python3 scripts/testing/localization/check_imu_velocity.py     # EKF 的兩個輸入
```

## 網頁介面

當系統是以 `play_launch` 啟動時，會有一個網頁列出每個節點的狀態、紀錄，以及每個
行程的 CPU、記憶體與 GPU：

- `http://127.0.0.1:8080` —— `play_launch` 預設
- `http://localhost:8081` —— `just launch` 所用，因為它傳了
  `--web-addr 0.0.0.0:8081`

兩個都值得知道，因為開錯那個會什麼都看不到，看起來就像失敗。

## 一個簡短的除錯順序

當某個東西不動時，依此順序：

1. `ros2 node list` —— 節點到底有沒有在跑？
2. `ros2 topic list` —— 主題存在嗎？
3. `ros2 topic info -v <topic>` —— 發佈者與訂閱者，以及它們的 QoS
4. `ros2 topic hz <topic>` —— 有在送嗎？夠快嗎？
5. `ros2 topic echo <topic> --once` —— 內容合理嗎？
6. `ros2 param get <node> <param>` —— 設定有送達嗎？
7. `/diagnostics` —— 節點自己說它不開心嗎？

多數問題在第 3 步就有答案。

## 接下來

- [Autoware 管線](./autoware-conventions.md) —— 主題名稱告訴你什麼
- [環境與相依套件](./environment.md)
