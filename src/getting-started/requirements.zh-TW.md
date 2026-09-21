<!--
Translation Metadata:
- Source file: requirements.md
- Last synced: 2026-09-14
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 你需要什麼樣的機器

**如果你要裝在自己的筆電或桌機上，請先讀這一頁**——它是
[推薦安裝方式](./installation/recommended.md)的前置條件，而答案大概是「你現在用的
那台就可以」：兩種模擬在四核心、8 GB、完全沒有 GPU 的機器上都跑得動。

**這不是車上那台電腦。** 車上跑的是接了 LiDAR、相機、GNSS 與車輛介面的
Jetson AGX Orin，它的數字由模擬永遠不會產生的感測器流量主導。那台機器記載於
[車輛](../platform-models.md) → [硬體設定](./hardware-assembly.md)；這一頁的內容
不構成對它的限制。

以下全部是實測值，在一台桌機上量的，方法寫在每張表下面。把它當成採購時的下限，
而不是保證。

## 簡短的答案

| | 最低 | 從容 |
|---|---|---|
| 作業系統 | **Ubuntu 22.04 LTS**——沒有商量餘地，見下 | Ubuntu 22.04 LTS |
| CPU | 4 核 | 8 核以上 |
| 記憶體 | 8 GB | 16 GB |
| 可用磁碟 | 20 GB | 40 GB |
| GPU | **不需要** | NVIDIA，CUDA 12.x |

最低規格的機器能跑完兩種模擬、也能建置工作空間。「從容」那一欄讓你感覺不到它在
工作，並且留下跑 GPU 路徑與感知的餘裕。

### Ubuntu 22.04 是硬性要求

AutoSDV 建立在 ROS 2 Humble 上，而 Humble 的目標是 Ubuntu 22.04，沒有 24.04 的
套件。這是唯一沒有彈性的一列：不是更新的 Ubuntu、不是 Debian，也不建議一開始就用
WSL。虛擬機在核心數與記憶體夠的情況下可以跑模擬，但拿不到 GPU。

## 磁碟

全部加起來約 **11 GB**：Autoware 4.8 GB、專案約 1 GB、建置好的工作空間約 1 GB，
再加上只有記錄回放模擬才需要的 2.8 GB rosbag。

請保留 **20 GB** 可用空間——rosbag 的壓縮檔與解開後的副本會並存一段時間，而
`play_log/` 下的執行紀錄也會累積。如果你打算自己錄製 rosbag，請準備 40 GB。

地圖不需要下載：`data/COSS-map-planning` 就在專案裡。唯一要下載的大檔是 rosbag，
見[資料集與 Rosbag](../running/datasets.md)。

## GPU

**兩種模擬都不需要 GPU。** 這是最常被誤解的一點。路徑規劃模擬沒有感測器，完全
不會碰到 GPU；記錄回放模擬加上 `pose_source:=ndt launch_perception:=false` 就是
純 CPU，而且跟得上錄製的速度。

有 GPU 可以換到兩件事：

| | 需要 | 代價 |
|---|---|---|
| `pose_source:=cuda_ndt` | CUDA，且 toolkit 認得你的 GPU | 約 1 GiB VRAM |
| 感知（TensorRT 模型） | NVIDIA GPU | VRAM，外加首次啟動 **10–30 分鐘**的 engine 編譯 |

engine 編譯每台機器、每個 Autoware 版本只需一次，而 `just build-engines` 是刻意
先做掉它，而不是讓它發生在你第一次啟動時。在那之前，`launch_perception:=false`
才是筆電上誠實的跑法。

### 如果你的 GPU 很新

`cuda_ndt` 在執行期才編譯 kernel，所以 CUDA toolkit 必須認得你顯卡的架構。比
toolkit 新的顯卡，失敗方式跟沒有 GPU 一模一樣，只是更吵也更晚——launcher 回報每個
節點都就緒，然後什麼都不發布。Blackwell（RTX 50 系列，sm_120）需要 **CUDA 12.8**
或更新。

```bash
scripts/check-cuda-arch.sh     # 也包含在 `just demo check` 裡
```

它會比對你的 GPU 與選定的 toolkit，不合時會列出機器上哪些 toolkit 可用。要選一個
只是一個變數的事——見[環境](../concepts/environment.md)。

### RViz 需要能用的 OpenGL

RViz 是唯一需要真正圖形堆疊的部分。在只有軟體算繪的 VNC 上，它大約每秒一張
畫面——足以確認畫面上有東西，但完全不適合看車子行駛。請用本機顯示器，或啟用
GPU 加速的 VNC。

## 在信任一台機器之前先檢查

```bash
just demo check
```

它會回報 rosbag、地圖、建置結果、`play_launch`、CUDA toolkit 與你的 GPU 是否相容，
以及有沒有可用的顯示。它報缺的每一項，都能在它指向的頁面找到解法。

---

這一頁背後的實測數字——各工作負載的記憶體與 CPU、建置尖峰與如何限制它、磁碟的
逐項明細——放在程式碼庫裡：`docs/reports/host-resource-measurements.md`。
