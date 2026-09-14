<!--
Translation Metadata:
- Source file: recommended.md
- Last synced: 2026-09-14
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

選一個設定檔，確認，然後讓它跑完。在你確認之前不會安裝任何東西。

| 設定檔 | 適用對象 |
|--------|----------|
| `dev` | 開發工作站：建置、模擬與執行所需的一切 |
| `vehicle` | 車上電腦：`dev` 的內容，再加上 u-blox GNSS 的 udev 規則 |
| `ci` | 只做建置的自動化。**不含 Autoware**——不是你在筆電上要的 |

不進選單、直接跑完：

```bash
./setup.sh --run --profile dev --yes
```

選單會為每個步驟附上一行說明，所以請在那裡讀，而不是在這裡。其中有兩項要你做
選擇，而且兩項都有合理的預設值：

- **TensorRT 引擎**——下載為這台機器發佈的那一份（約 30 秒），或在本機建置
  （Orin 上約一小時）。預設是下載；找不到相符的版本時會自動改為建置。
- **ZED SDK**——只檢查、不安裝，因為 Stereolabs 沒有提供 apt 套件。如果你有 ZED
  相機，安裝流程結束時會告訴你要下載什麼，見
  [ZED SDK 安裝](./zed-sdk.md)。沒有相機就忽略它。

## 建置

工作空間用 colcon 建置，完整的指令是：

```bash
source /opt/autoware/1.5.0/setup.bash
colcon build --base-paths src --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --cargo-args --release
```

四個旗標，每個都有作用：

| 旗標 | 作用 |
|------|------|
| `--base-paths src` | 只建置 `src/` 底下的套件，而不是目錄裡的其他東西 |
| `--symlink-install` | 以符號連結安裝，因此改 YAML、XML 與 Python 不必重新建置 |
| `--cmake-args -DCMAKE_BUILD_TYPE=Release` | 最佳化的 C++ |
| `--cargo-args --release` | 最佳化的 Rust——`CMAKE_BUILD_TYPE` 傳不到 cargo，而未最佳化的 `cuda_ndt_matcher` 慢上大約十五倍 |

**捷徑：** `just build` 執行的就是上面那串，連 source 都包含在內。

```bash
just build
```

預期花幾分鐘，最後出現 `Summary:` 且沒有失敗的套件；stderr 上的警告是正常的。

## 確認它成功了

```bash
./setup.sh --status
```

接著看[驗證安裝](./verify.md)——幾個一分鐘就能做完的檢查。

## 下一步

[教學](../../tutorial/00-what-you-will-build.md)：在模擬中駕駛兩次，實際看到系統
運作。那才是安裝成功的真正證明。

---

## 進階內容

以下都不是「能用」所必需的。

### 用 direnv 自動進入環境

AutoSDV 附了一個 `.envrc`，你一進入該目錄就會啟用環境：

```bash
sudo apt install direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc   # 或你所用 shell 的對應寫法
source ~/.bashrc

cd ~/AutoSDV
direnv allow
```

不用 direnv 的話，每開一個新終端機就手動 source 兩行：

```bash
source /opt/autoware/1.5.0/setup.bash   # 包含 ROS 2
source install/setup.bash               # 第一次建置之後
```

相依套件就是在這兩行裡解析的——見
[環境與相依套件](../../concepts/environment.md)。

### 從命令列驅動 setup.sh

```bash
./setup.sh --status                      # 裝了什麼，會實際檢查機器
./setup.sh --list                        # 每個步驟，以及它是否適用於這台機器
./setup.sh --run --profile dev --yes     # 不互動
./setup.sh --rerun opencv                # 只重跑某一個步驟
./setup.sh --run --dry-run --profile ci  # 印出會做什麼，不安裝任何東西
./setup.sh --plain                       # 編號選單，當 curses 無法驅動終端機時
```

`--status` 不是「你跑過什麼」的紀錄。只要某個步驟能在機器上檢查，**以機器的答案
為準**——所以一個步驟可能顯示未安裝，而它是對的：重開機會讓 loopback 失去
`MULTICAST` 旗標、JetPack OTA 會替換掉 OpenCV 標頭檔、Autoware 升級會讓鏡像的
模型樹失效。

### TensorRT 引擎，事後補做

如果你跳過了引擎步驟，或想改變主意：

```bash
just engines          # 取用為這台機器發佈的引擎，沒有才自行建置
just build-engines    # 直接在本機建置，忽略已發佈的版本
```

可以安全地重複執行：第二次會發現快取已就位而什麼都不做，中斷的下載則會續傳。

引擎同時綁定 GPU **與** TensorRT 版本，所以無法預先烘焙進在別處建置的映像檔，
而且在 Autoware 或 JetPack 升級後都必須重做。

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

## 取得協助

- [AutoSDV GitHub Issues](https://github.com/NEWSLabNTU/AutoSDV/issues)
