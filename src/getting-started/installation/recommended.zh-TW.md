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

**你不需要把它們背下來。** 選一個設定檔即可；游標停在哪個步驟，選單就顯示那個
步驟的一行說明，包含跳過它的代價。那段文字寫在步驟旁邊，而不是寫在這裡——寫在
這裡遲早會和實際情況脫節。`./setup.sh --list` 印出的是同一份說明。

選單依工作性質分組：

| 分組 | 涵蓋內容 |
|------|----------|
| Toolchain | `just`、ROS 2 Humble、colcon/rosdep、Rust 工具鏈與其 colcon 外掛、`play_launch`，以及車輛介面在執行期匯入的 Python 套件 |
| Autoware | Autoware 1.5.0 Debian 套件（2–3 GB）、它們建置時所用的 TensorRT runtime、可寫入的模型樹、對 `src/` 執行的 rosdep，以及感知引擎 |
| Libraries | OpenCV 一致性、ZED SDK 檢查、Blickfeld Cube1 驅動 |
| System configuration | CycloneDDS 所需的核心 socket 緩衝區、loopback multicast、u-blox udev 規則、TurboVNC/VirtualGL |

其中四項過去在本頁是要你手動執行的步驟。最常讓全新安裝失敗的兩項——CycloneDDS
的 socket 緩衝區與 loopback 的 `MULTICAST` 旗標——正是「讓設定檔替你選」的理由：
`net.core.rmem_max` 低於約 10 MB 時**沒有任何 ROS 2 節點能啟動**，而 `lo` 每次
重開機都會失去 multicast 旗標。`dev` 與 `vehicle` 都會選取修正這兩者的步驟，不必
再照著手冊逐條輸入。

確認之前，有兩個選擇值得先了解——它們是少數「預設值出於判斷而非必要」的地方。

#### TensorRT 引擎：下載或自行建置

Autoware 會把五個感知模型編譯成 TensorRT 引擎。在機器上編譯，Orin 約一小時，
桌機 GPU 約九分鐘；若跳過，同樣的工作會發生在第一次啟動時的節點建構子裡，看起來
像當掉，而且在完成前感知功能都不可用。

選單把它呈現成一個決定、兩個答案：

```
    TensorRT engines (pick one, or neither)
   15   (o) Download the published set (build only if none matches)
   16   ( ) Build here, ignoring the published set
```

預設是下載，約 30 秒：AutoSDV 的 releases 上發佈了針對特定硬體建置的引擎，而下載
回來的引擎會先逐一載入驗證過才採用。我們實際使用的板子都有對應的發佈版本；其他
硬體則由同一個步驟在本機建置，也就是你本來就得付出的成本。當你自己在改模型、或
要產生一份要發佈的引擎時，才選第二個答案。

引擎同時綁定 GPU **與** 確切的 TensorRT 版本，所以兩種答案都無法預先烘焙進在別處
建置的映像檔，而且在 Autoware 或 JetPack 升級後都必須重做。

#### ZED SDK 由你安裝，不是由設定程式安裝

Stereolabs 沒有提供 apt 套件庫——唯一的官方安裝檔是互動式安裝程式，會要求你接受
專有授權。因此 `zed-sdk` 這個步驟只做**檢查**：回報你目前安裝的版本，若缺少或版本
不符，就印出適合這台機器的下載連結，並在整個安裝流程結束時以醒目顏色再印一次，
確保它是螢幕上最後看到的東西。

在沒有 ZED 相機的機器上保留這個步驟不會有任何代價：驅動套件會自行跳過，工作空間
其餘部分照常建置。當安裝流程提示你時，再依
[ZED SDK 安裝](./zed-sdk.md) 進行即可。

另一個需要自行選取的步驟是 `blickfeld`，供 Cube1 光達使用；選取它等同接受該函式庫
的授權條款。

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

## TensorRT 引擎：如果你在選單裡跳過了

設定程式會替你處理——見上面「TensorRT 引擎：下載或自行建置」一節。
事後要補做：

```bash
just engines          # 取用這台機器適用的已發佈引擎，沒有才自行建置（setup.sh 預設）
just build-engines    # 直接在本機建置，忽略已發佈的版本
```

當已發佈的引擎符合這台硬體時，`just engines` 約 30 秒完成，否則退回自行建置。
它可以安全地重複執行：第二次會發現快取已就位而什麼都不做，中斷的下載則會續傳
而不是從頭來過。

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
