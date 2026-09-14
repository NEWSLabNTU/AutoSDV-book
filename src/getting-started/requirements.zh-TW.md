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

## 每個數字從哪來

量測環境：Ubuntu 22.04、Intel Core Ultra 7 270K Plus（24 核）、125 GB 記憶體、
RTX 5090，工具是 `scripts/profiling/host_resource_sampler.py`。記憶體以「相對於
閒置基線，整台機器多用了多少」表示，那正是小機器必須擠出來的量。CPU 以「核數」
表示，因為那才是能換算到不同核心數機器上的數字。

| 工作負載 | 記憶體尖峰 | 啟動時 CPU | 穩態 CPU |
|---|---|---|---|
| `just build`，乾淨工作空間 | **11.3 GiB** | 有幾核用幾核 | — |
| 規劃模擬 | **2.5 GiB** | 短暫 19 核 | 約 2 核 |
| 日誌模擬，CPU 路徑 | **3.4 GiB** | 短暫 21 核 | 約 3 核 |
| 日誌模擬，GPU 路徑 | 3.4 GiB 加約 1 GiB VRAM | 同上 | 約 3 核 |
| 任一項再加上 RViz | 約多 1.4 GiB | — | 約 1 核 |

其中三個數字各值得一句說明。

**建置尖峰最大，而且可以調。** 11.3 GiB 是 colcon 以 24 個工作並行編譯 31 個套件
時達到的；整段軌跡在那裡停留約十二秒，然後就掉下來。尖峰隨同時跑幾個編譯器而變，
所以四核筆電不用特別設定就大約只會用到四分之一。如果機器記憶體吃緊，明確設上限，
不要等 OOM killer 出手：

```bash
colcon build --base-paths src --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
```

**「啟動時 CPU」是突波，不是需求。** 啟動一個 stack 會一次拉起三十幾個節點，它們
會把在場的核心全部用上——這裡是 24 核用掉 21 核。在四核機器上同樣的工作只是比較
久，不會失敗。機器必須長時間撐住的是穩態那一欄，也就是兩到三核。

**建置很快，是因為 Autoware 大部分早就建好了。** 在這台桌機上 31 個套件花了
**89 秒**。AutoSDV 是疊在二進位 Autoware 安裝*之上*的工作空間，所以 `just build`
編譯的是車輛自己的套件，而不是底下那 30 GB 的 Autoware。預算以分鐘計，不是小時；
四核機器上也仍然以分鐘計。

## 磁碟，逐項

| | |
|---|---|
| Autoware Debian，下載 | 1.9 GB |
| Autoware，安裝於 `/opt/autoware/1.5.0` | 4.8 GB |
| 本專案 clone 後（含 COSS 地圖） | 約 1 GB |
| `just build` 後的工作空間（`build/` + `install/`） | 0.9 GB |
| COSS rosbag，下載 | 1.6 GB |
| COSS rosbag，解開後 | 2.8 GB |
| **全部裝完、清理後合計** | **約 11 GB** |

最低要 20 GB，是因為 rosbag 的壓縮檔與解開後的副本會並存一段時間，也因為
`play_log/` 下的 ROS 紀錄會隨每次執行增加。如果你打算自己錄 bag，就抓 40 GB——那
是這裡唯一沒有自然大小上限的東西。

地圖不用下載：`data/COSS-map-planning` 已納入版本控制。rosbag 是唯一一個大檔，而且
只有日誌模擬需要它——見[資料集與 Rosbag](../simulation/datasets.md)。

## GPU，細談

**兩種模擬都不需要 GPU。** 這是最常被想錯的一點。

- **規劃模擬**沒有感測器、沒有感知、沒有定位，從頭到尾不碰 GPU。
- **日誌模擬**回放真實 LiDAR 並對地圖定位。用
  `pose_source:=ndt launch_perception:=false` 時純靠 CPU，而且維持感測器完整的
  10 Hz——對著 10 Hz 的錄製實測為 10.06 Hz。

GPU 買到的是兩件事：

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

RViz 是唯一真正需要圖形堆疊的部分。在只有軟體算繪的 VNC 上，本機實測是 **1 fps**
——足以確認畫面上有東西，但完全不足以看車子開。要讓教學裡視覺的部分值得做，需要
本機顯示，或是有 GPU 加速的 VNC。

## 在信任一台機器之前先檢查

```bash
just demo check
```

它會回報 rosbag、地圖、建置結果、`play_launch`、CUDA toolkit 與你的 GPU 是否相容，
以及有沒有可用的顯示。它報缺的每一項，都能在它指向的頁面找到解法。
