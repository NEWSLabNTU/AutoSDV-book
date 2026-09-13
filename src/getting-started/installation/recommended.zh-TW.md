<!--
Translation Metadata:
- Source file: recommended.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 推薦安裝方式

本指南以自動化設定程式安裝 AutoSDV。開始之前，請先完成
[作業系統準備](./overview.md#prepare-operating-system)。

## 複製儲存庫

工作空間大部分由子模組組成，請以遞迴方式複製：

```bash
cd ~
git clone --recursive https://github.com/NEWSLabNTU/AutoSDV.git
cd AutoSDV
```

如果你已經在沒有 `--recursive` 的情況下複製了：

```bash
just checkout    # git submodule update --init --recursive --checkout
```

## 執行設定程式

```bash
./setup.sh
```

這會開啟一個選單：先選一個**設定檔（profile）**，視需要調整個別步驟，然後確認。
在你確認之前不會安裝任何東西。

### 設定檔

設定檔是一組步驟的預設選取，而不是限制——你隨時可以在選單中切換任何步驟。

| 設定檔 | 適用對象 |
|--------|----------|
| `dev` | 開發工作站。建置、模擬與執行所需的一切。 |
| `vehicle` | 車輛電腦。等同 `dev`，再加上 u-blox GNSS udev 規則。 |
| `ci` | 僅供建置的自動化環境。工具鏈與工作空間相依套件，**不含 Autoware**——它無法建置工作空間，也不是你在筆電上想要的選擇。 |
| `all` / `none` | 由程式計算得出：全部步驟，或不選任何步驟。 |

無人值守安裝：

```bash
./setup.sh --run --profile vehicle --yes
```

### 命令列

選單是選用的；每個操作都有對應的旗標：

```bash
./setup.sh --status                      # 已安裝什麼，並對照機器實際狀態檢查
./setup.sh --list                        # 每個步驟，以及它是否適用於此機器
./setup.sh --run --profile dev --yes     # 無人值守
./setup.sh --run --all --skip tensorrt-engines
./setup.sh --rerun opencv                # 重跑單一步驟
./setup.sh --run --dry-run --profile ci  # 印出將執行什麼，但不安裝任何東西
./setup.sh --plain                       # 數字選單，當 curses 無法驅動終端機時使用
```

### `--status` 實際回報的是什麼

`--status` 不是你執行過什麼的紀錄。當某個步驟帶有驗證指令時，**機器的答案會勝過
紀錄**——因此一個步驟可能顯示為未安裝，即使你安裝過它，而這是正確的：

- 重新開機會讓 loopback 失去 `MULTICAST` 旗標
- JetPack OTA 會替換掉 OpenCV 標頭檔
- Autoware 升級會讓鏡像的資料樹指向一個已不存在的版本

它同時會對所執行腳本的*內容*取指紋，因此能告訴你某個步驟成功了，但它的腳本自那
之後已經改變。

### 各個步驟

依選單的分組排列。「預設於」指的是不必你要求就會選取該步驟的設定檔。

#### 工具鏈（Toolchain）

| 步驟 | 預設於 | 存在的理由 |
|------|--------|-----------|
| `just` | 全部 | 本儲存庫的每個工作流程都是一個 `just` recipe |
| `ros2` | 全部 | ROS 2 Humble，其餘一切建置的基礎 |
| `ros2-dev-tools` | 全部 | colcon、rosdep、vcstool——沒有它們什麼都建不起來 |
| `rust` | 全部 | `cuda_ndt_matcher` 是 Rust 寫的；沒有工具鏈時 colcon 會略過它，`pose_source:=cuda_ndt` 便無物可啟動 |
| `colcon-cargo-ros2` | 全部 | 讓 colcon 會建置 Rust 套件；缺少它時這些套件會被*靜默*略過，建置改在稍後因找不到套件而失敗 |
| `play-launch` | 全部 | 本書全程使用的啟動協調器 |
| `python-deps` | 全部 | `Adafruit-PCA9685`、`simple-pid`——車輛介面在執行時匯入 |
| `geographiclib` | 全部 | Autoware 的地圖投影需要 egm2008-1 大地水準面來換算 GNSS 高度 |
| `gdown` | dev、vehicle | 範例資料下載腳本會用到 |
| `dev-tools` | dev、vehicle | git-lfs、pre-commit、clang-format、PlotJuggler |

#### Autoware

| 步驟 | 預設於 | 存在的理由 |
|------|--------|-----------|
| `autoware-debian` | dev、vehicle | Autoware 1.5.0 localrepo，2–3 GB。`src/` 中的一切都以它為基礎建置 |
| `autoware-data` | dev、vehicle | **見下文**——少了它，感知模組會在每一次啟動時失敗，永遠如此 |
| `ros-deps` | 全部 | rosdep 解析 `src/` 下套件宣告的每個相依鍵，這也是這裡沒有個別驅動程式 apt 步驟的原因 |
| `tensorrt-engines` | *選用* | 預先編譯引擎；每個模型數分鐘。略過它只是把成本移到你的第一次啟動 |

#### 函式庫（Libraries）

| 步驟 | 預設於 | 存在的理由 |
|------|--------|-----------|
| `opencv` | dev、vehicle | JetPack 會留下 4.8.0 的標頭檔搭配 4.5.4 的執行期，能編譯但行為異常。也提供 aruco/contrib |
| `zed-sdk` | *選用* | ZED X Mini，每個預設感測器組合都包含它。下載量大 |
| `blickfeld` | *選用* | Cube1 LiDAR 驅動程式。選取它即表示接受該函式庫的授權條款 |

#### 系統設定（System configuration）

| 步驟 | 預設於 | 存在的理由 |
|------|--------|-----------|
| `cyclonedds-sysctl` | dev、vehicle | `net.core.rmem_max` 與 IP 分片設定。**低於約 10 MB 時沒有任何 ROS 2 節點能啟動** |
| `multicast-lo` | dev、vehicle | `cyclonedds.xml` 指定 `lo`，而 `lo` 每次重開機都會失去 `MULTICAST` 旗標。安裝一個 unit 讓它撐過重開機 |
| `ublox-udev` | vehicle | 提供穩定的 `/dev/ublox-gps` 名稱，並把你加入 `dialout`。需登出再登入群組才生效 |
| `turbovnc-virtualgl` | *選用* | 透過 VNC 進行 GPU 加速繪圖，ZED 工具在 VNC 工作階段中需要它 |

### 選用步驟是你必須做的選擇

有三個步驟**不屬於任何**設定檔——沒有任何東西會替你選取它們：

- `zed-sdk` —— 有 ZED 相機時需要。每個預設感測器組合都包含一台，所以在車輛上你
  幾乎一定需要它。
- `blickfeld` —— 只有 Cube1 LiDAR 需要。
- `tensorrt-engines` —— 嚴格來說永遠不是必要的，但在車輛上永遠值得。

> **注意：** 除非你選取，否則不會安裝 ZED SDK。參閱
> [ZED SDK 安裝](./zed-sdk.md)。

## 安裝並設定 direnv

AutoSDV 附有一份 `.envrc`，會在你進入該目錄時啟用環境：

```bash
sudo apt install direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc   # 或你所用 shell 的對應寫法
source ~/.bashrc

cd ~/AutoSDV
direnv allow
```

若不使用 direnv，就必須在每個新 shell 中手動執行這兩行：

```bash
source /opt/autoware/1.5.0/setup.bash   # 已包含 ROS 2
source install/setup.bash               # 首次建置之後
```

**套件相依性正是在這兩行被解析的**，這件事值得在系統告訴你「package not found」
之前先弄懂——參閱[環境與相依套件](../../concepts/environment.md)。

## 建置

```bash
just build
```

它以本專案所需的旗標執行 colcon：

```bash
colcon build --base-paths src --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --cargo-args --release
```

`--cargo-args --release` 不是裝飾。`CMAKE_BUILD_TYPE=Release` 只涵蓋 C++ 套件；
少了 cargo 這個旗標，Rust 套件會以未最佳化的方式建置，`pose_source:=cuda_ndt`
每個掃描約需 80 毫秒，而非 5 毫秒。

## 讓 Autoware 模型樹可寫入

**即使你略過了所有選用步驟，這一步仍要做。**

Autoware 會把每個 `.onnx` 模型編譯成 `.engine`，並寫在*該 onnx 檔案旁邊*。
Debian 套件位於 `/opt/autoware/1.5.0/data` 的資料樹屬於 root，因此該寫入失敗、
引擎被丟棄，同一批模型便會在每一次啟動時重新建置——並再次失敗。

```bash
just setup-autoware-data
```

這會以符號連結把資料樹鏡像到 `data/autoware_data`（171 個檔案，不到 1 MB），
而啟動檔案本來就預設指向該處。Autoware 升級後請重新執行。

## 預先編譯 TensorRT 引擎（選用，建議）

```bash
just build-engines
```

少了這一步，第一次啟動會在各節點的建構子內編譯引擎——在 Orin 上需 10 到 30
分鐘，期間感知模組不可用。

引擎**同時**綁定 TensorRT 版本與 GPU，因此必須在將要使用它們的機器上執行。它
無法被烘進在別處建置的映像檔，且在 Autoware 或 JetPack 升級後必須重新執行。

## 驗證

參閱[驗證安裝](./verify.md)——四項檢查，依它們能證明多少事情排序。

## 疑難排解

### 建置因缺少套件而失敗

通常是 rosdep 尚未解析某項相依：

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 建置成功，但少了 `cuda_ndt_matcher`

缺少 Rust 工具鏈或 `colcon-cargo-ros2`，colcon 略過了該套件且未提示：

```bash
./setup.sh --rerun rust
./setup.sh --rerun colcon-cargo-ros2
just build
```

### 完全沒有任何 ROS 2 節點啟動

核心 socket 緩衝區。這不是細微的失敗——是什麼都起不來：

```bash
./setup.sh --rerun cyclonedds-sysctl
```

### 重開機後同一台機器上的節點彼此看不到

loopback 介面失去了 `MULTICAST` 旗標：

```bash
./setup.sh --status          # multicast-lo 會回報為未滿足
./setup.sh --rerun multicast-lo
```

### 感知模組在每次啟動時重新編譯模型

可寫入資料樹那一步被略過了，或 Autoware 升級使鏡像失效：

```bash
just setup-autoware-data
```

### CUDA 或 TensorRT 問題

請檢查你的 Autoware 安裝實際需要什麼，而不是你以為你裝了什麼：

```bash
ldd /opt/autoware/1.5.0/lib/libtensorrt_ops.so | grep nvinfer
# libnvinfer.so.10  -> 你需要 TensorRT 10.x，不是 8.x
nvidia-smi
```

### 某個步驟顯示未安裝，但你明明裝過

那是 `--status` 在讀取機器狀態而非它自己的紀錄，而它通常是對的。

## 後續步驟

- [驗證安裝](./verify.md)
- [教學](../../tutorial/00-what-you-will-build.md) —— 在模擬中駕駛
- [操作車輛](../usage.md) —— 完整的啟動參數參考

## 取得協助

- [AutoSDV GitHub Issues](https://github.com/NEWSLabNTU/AutoSDV/issues)
