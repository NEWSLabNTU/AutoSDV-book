<!--
Translation Metadata:
- Source file: container.md
- Last synced: 2026-09-16
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 以容器執行

在你現有的筆電上執行模擬，不需要安裝 ROS 2、Autoware，也不需要編譯
AutoSDV。你只要打開瀏覽器，就會看到一個含有 RViz 與終端機的桌面。

這是為工作坊、課堂，以及任何想在投入一台機器之前先試用 AutoSDV 的人所準備的
路徑。Windows、macOS（Intel **與** Apple Silicon）以及 Linux 都執行同一個指令。

!!! info "這不是已停止維護的 `docker/` 映像檔"

    那個是 [Docker 設定（已停止維護）](../getting-started/installation/docker.md)，是另一個已經無法建置的
    東西。本頁說明的是 desktop 映像檔，它已建置、已發布，並在兩種架構上通過
    測試。

## 你需要什麼

| | |
|---|---|
| **Docker Desktop**（Linux 上為 Docker Engine） | [docs.docker.com/desktop](https://docs.docker.com/desktop/) |
| **磁碟空間** | 30 GB 可用空間 |
| **配給 Docker 的記憶體** | 最低 8 GB，**建議 12 GB** — 見[給 Docker 足夠的記憶體](#give-docker-enough-memory) |
| **下載量** | Apple Silicon 為 5.4 GB，其他平台為 14.3 GB |

不需要 GPU；在 macOS 上也不可能有 GPU：Hypervisor.framework 不會把 GPU 提供給
容器，任何選項都改變不了這一點。兩種模擬都是以這個底線為前提設計的，沒有 GPU
也能執行。

## 取得

你仍然需要 clone 儲存庫，因為地圖與 rosbag **不在**映像檔裡——它們很大、獨立
更新，而且已經有這些檔案的學生不該再下載一次。

```bash
git clone https://github.com/NEWSLabNTU/AutoSDV.git
cd AutoSDV
```

接著啟動容器。每個平台都有對應的啟動腳本，它會一次完成 pull、run 與開啟
shell：

=== "Linux / macOS"

    ```bash
    ./docker/desktop/autosdv.sh
    ```

=== "Windows（PowerShell）"

    ```powershell
    .\docker\desktop\autosdv.ps1
    ```

    若 PowerShell 拒絕執行——顯示 *"running scripts is disabled"*——請為你自己的
    帳號放行本機腳本，只需一次：

    ```powershell
    Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
    ```

第一次執行會下載數 GB，而且只會發生一次。完成後你會進入容器**內部**的 shell，
ROS 2、Autoware 與工作空間都已經 source 完成。

在瀏覽器中開啟桌面：

```
http://localhost:6080/vnc.html?autoconnect=1&resize=remote
```

!!! tip "一個 tag，適用每一台機器"

    `jerry73204/autosdv:desktop` 是一個多架構 manifest。Docker 會依你所在的平台
    索取對應的映像檔——Windows、Linux 與 Intel Mac 取得 amd64 版，Apple Silicon
    取得 arm64 版。沒有人需要挑選版本。

### 手動執行

啟動腳本只是便利工具，不是必要條件。直接執行相同的動作：

```bash
docker pull jerry73204/autosdv:desktop

docker run -dit --name autosdv \
  -p 6080:6080 \
  -v "$PWD/data:/opt/AutoSDV/data" \
  --shm-size=2gb --cap-add=NET_ADMIN \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  jerry73204/autosdv:desktop

docker exec -it autosdv bash
```

!!! warning "`data` 掛載不是選用的"

    沒有它，地圖目錄會是空的，而 `lanelet2_map_loader` 面對缺少的地圖會直接
    **segfault**，而不是回報問題——所以你看到的第一行是當機訊息，而不是原因：

    ```
    PCD load failed: /opt/AutoSDV/data/COSS-map-planning/pointcloud_map.pcd
    Composable node '/map/lanelet2_map_loader' crashed: killed by signal 11
    ```

## 執行模擬

在容器的 shell 中：

```bash
just sim planning
```

然後在瀏覽器分頁中觀看。關於你所看到的內容，教學本身的說明請繼續閱讀
[1. 第一次執行](../tutorial/01-first-run.md)；那裡的每一步在容器中同樣適用。

!!! note "RViz 大約 90 秒內不會畫出任何東西"

    地圖載入期間畫面是黑的。這是載入時間，不是故障——在地圖出現之前不要開始
    除錯。

### 直接執行 Autoware 的 launch 檔

如果你依照的教學會自己呼叫 `play_launch`，請加上
`--container-mode observable`：

```bash
play_launch launch --container-mode observable \
  autoware_launch planning_simulator.launch.xml \
  map_path:=$PWD/data/COSS-map-planning \
  vehicle_model:=autosdv_vehicle \
  sensor_model:=autosdv_sensor_kit
```

預設值（`isolated`）會為每個 composable node 各 fork 一個行程。在筆電上，這就是
能否正常運作的差別——在限制為 8 核心與 8 GB 的容器中實測：

| | isolated（預設） | observable |
|---|---|---|
| 所有節點就緒 | 約 90 秒 | **7 秒** |
| 記憶體 | 5.71 GiB | **3.09 GiB** |
| 行程數 | 119 | **49** |

代價是失去個別節點的故障隔離與 OOM 記帳，而這是教學用不到的。

### 兩個終端機

logging simulation 需要一個 shell 執行整個系統，另一個執行 rosbag 重播。再執行
一次啟動腳本——在任何目錄都可以——它會在**同一個容器中開啟第二個 shell**，而不是
啟動新的容器：

=== "Linux / macOS"

    ```bash
    ./docker/desktop/autosdv.sh
    ```

=== "Windows（PowerShell）"

    ```powershell
    .\docker\desktop\autosdv.ps1
    ```

<a id="give-docker-enough-memory"></a>

## 給 Docker 足夠的記憶體

這是最容易毀掉第一次嘗試的設定，而它的症狀完全不會提到記憶體。

**Docker Desktop → Settings → Resources → Memory。** 機器允許的話請給
**12 GB**；8 GB 是底線。Linux 搭配 Docker Engine 沒有這個限制，不需要調整。

當上限太低時，節點會在啟動過程中被殺掉，而 log 看起來會像是系統中彼此無關的
部分零星當機：

```
Composable node '/map/lanelet2_map_visualization' crashed: killed by signal 6 (Aborted)
Composable node '/adapi/node/vehicle_metrics' crashed: killed by signal 6 (Aborted)
```

其中沒有任何一行提到記憶體不足。如果你看到數個不相關的節點出現 abort，請先
提高記憶體並加上 `--container-mode observable`，再去找 bug。

## 停止容器

=== "Linux / macOS"

    ```bash
    ./docker/desktop/autosdv.sh --stop
    ```

=== "Windows（PowerShell）"

    ```powershell
    .\docker\desktop\autosdv.ps1 -Stop
    ```

容器會刻意在多個 shell 之間持續執行，所以關閉一個終端機並不會中斷模擬。

## 選項

| | Linux / macOS | Windows |
|---|---|---|
| 先檢查較新的映像檔 | `--pull` | `-Pull` |
| 將 NVIDIA GPU 傳遞進容器 | `--gpu` | `-Gpu` |
| 停止並移除 | `--stop` | `-Stop` |

`--gpu` 可用於安裝了 NVIDIA Container Toolkit 的 Linux，以及透過 Docker Desktop
的 WSL2 GPU 支援的 Windows。在 macOS 上**無法**運作。它只在容器建立時生效，所以
若已有容器在執行，請先停止它。

有三個環境變數可以改變預設值：`AUTOSDV_PORT`（當 6080 已被占用時）、
`AUTOSDV_IMAGE` 與 `AUTOSDV_CONTAINER`。

## 離線發放

五十台筆電透過同一個無線基地台下載 14 GB，會浪費掉一整個早上。請改為把映像檔
匯出成檔案並在本地端提供：

```bash
./docker/desktop/export-images.sh          # 兩種架構
cd /srv/autosdv && ./docker/desktop/serve-images.sh
```

它會印出一個可以寫在白板上的網址。每位學生下載自己筆電對應的檔案——檔名寫的是
*"Apple Silicon Mac"* 而不是 *"arm64"*——然後載入：

```bash
docker load < autosdv-desktop-arm64.tar.gz
```

啟動腳本接著會在本機找到映像檔，不會再下載。

## 這個容器不適用於什麼

- **車輛本體。** 容器沒有可以溝通的感測器驅動程式，而 Jetson 使用的是真正的
  安裝方式。請見[軟體安裝](../getting-started/installation/overview.md)。
- **感知（Perception）。** TensorRT engine 沒有預先建置，也沒有 GPU 可以執行；
  工作坊的模擬不會啟動感知模組。
- **`pose_source:=cuda_ndt`**，那是真實機器上的預設值，執行時需要 NVIDIA GPU。
  在容器中請使用 `pose_source:=ndt`。

## 下一步

- [1. 第一次執行](../tutorial/01-first-run.md) — 教學，在容器中完全適用
- [你需要什麼樣的機器](../getting-started/requirements.md) — 如果你更想直接安裝在主機上
- [執行環境](../concepts/environment.md) — 那兩行 `source` 為何存在；啟動
  腳本開啟的每個 shell 都已經幫你做好了
