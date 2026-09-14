<!--
Translation Metadata:
- Source file: environment.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 環境與相依套件的來源

請在第一次看到「package not found」之前讀這一頁。

AutoSDV 早期幾乎每一個失敗都源自同一個錯誤：某個終端機沒有被告知 ROS 2、
Autoware 與 AutoSDV 工作空間在哪裡。這個錯誤之所以容易犯，是因為本專案同時附了
一個會把它藏起來的工具，也因為錯誤訊息指向的方向是錯的。

## AutoSDV 安裝 Autoware 的方式與 Autoware 官方不同

如果你讀過 Autoware 的官方文件，你預期的會是：複製一個工作空間、對數十個儲存庫
執行 `vcs import`，再花上一小時以上的 `colcon build`。

**AutoSDV 不是這樣做的。** Autoware 以 **Debian 套件**的形式安裝到
`/opt/autoware/1.5.0`。你的磁碟上沒有 Autoware 的原始碼樹，機器上也不會編譯任何
Autoware 的程式碼。

理由是成本。在 Jetson AGX Orin 上從原始碼建置 Autoware 要花好幾個小時，而且每次
乾淨檢出都得再花一次。準備實驗課的學生，或佈署車輛的整合人員，不該付出這個代價。

!!! note "取捨"

    你用幾分鐘而不是幾小時完成安裝。代價是你接受別人替你建好的版本——`1.5.0`，
    鎖定在 `versions.yaml`——而且你無法在不
    [自行建置](../getting-started/installation/manual-environment.md)的情況下
    修改 Autoware 自己的原始碼。

**但 AutoSDV 本身仍然要編譯。** `src/` 底下的一切都是一個 colcon 工作空間，
由 `just build` 建置：啟動檔、感測器套件、車輛介面、CUDA NDT 匹配器。

所以你始終處於一種混合狀態：**底層是二進位的 Autoware，上層是原始碼工作空間。**
本頁其餘部分談的就是這個結構。

## 一行就找得到套件，但有一樣東西它帶不來

```bash
source install/setup.bash    # 在儲存庫根目錄，且要先 `just build`
```

這一行就足以*找到*所有東西。`install/setup.bash` 是 colcon 在建置時產生的，裡面
記下了建置當下已經載入的 prefix：

```sh
# install/setup.sh 內部
COLCON_CURRENT_PREFIX="/opt/autoware/1.5.0"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"
```

所以載入工作空間會把 ROS 2 與 Autoware 一起帶進來。在一個沒有載入其他東西的乾淨
shell 上實測：

```console
$ source install/setup.bash
$ ros2 pkg prefix autoware_launch
/opt/autoware/1.5.0
$ ros2 pkg prefix autosdv_launch
…/AutoSDV/install/autosdv_launch
```

**它帶不來的是 middleware。** 這條鏈載入的是 `local_setup.bash`，那是 colcon 產生
的檔案，只帶套件環境，別的都沒有。`RMW_IMPLEMENTATION` 與 `CYCLONEDDS_URI` 是寫在
`/opt/autoware/1.5.0/setup.bash` 裡的普通 export——那是包在產生檔外面的手寫
wrapper——所以不會被繼承：

```console
$ source install/setup.bash
$ echo "${RMW_IMPLEMENTATION:-unset}"
unset
```

未設定就是 ROS 2 的預設值 `rmw_fastrtps_cpp`，那和別處啟動的 stack 正在用的是
*另一套 middleware*。這就是下面描述的故障，也正是 `.envrc` 存在的理由：它替這個
目錄做一次選擇，於是一行 `source install/setup.bash` 就真的是終端機所需的全部。

如果你不想依賴 `.envrc`，就先載入 Autoware、再載入工作空間——這個順序兩者都拿得到：

```bash
source /opt/autoware/1.5.0/setup.bash
source install/setup.bash
```

### 第一行做的事比看起來多

你可能以為需要三個指令——一個給 ROS 2、一個給 Autoware、一個給工作空間。其實只
需要兩個，因為 `/opt/autoware/1.5.0/setup.bash` 會先自己載入 ROS 2，再加上
Autoware：

```bash
# /opt/autoware/1.5.0/setup.bash 內部
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
...
    source "$AUTOWARE_HOME/local_setup.bash"
```

一個指令，兩個層。這不是你能猜到的事，也是為什麼別處那些以
`source /opt/ros/humble/setup.bash` 開頭的說明，在這裡與其說是錯的，不如說是
不完整的。

它執行時也會印出幾行——說明 `ROS_LOCALHOST_ONLY` 已為 CycloneDDS 取消設定，以及
核心網路設定未調整時的警告。那些是資訊性的。真正重要的一行是
`Autoware 1.5.0 environment loaded.`

### 少載入 Autoware 這一層，連 middleware 都會換掉

這件事讓人損失過好幾天，值得單獨講。`/opt/autoware/1.5.0/setup.bash` 會設定兩個
別處都不會設的變數：

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///opt/autoware/1.5.0/config/cyclonedds.xml
```

只載入 ROS 2 與工作空間，`RMW_IMPLEMENTATION` 就維持未設定，也就是 ROS 2 的預設
值 `rmw_fastrtps_cpp`。那不是比較慢或比較差的選項，而是**另一套 middleware**；
跑在不同 middleware 上的兩個 ROS 2 process 彼此完全看不到。

沒有任何東西會回報這件事。兩邊都乾淨啟動、都列得出自己的節點、都對著空氣發布。
實際看起來像這樣：

- 你的終端機 `ros2 topic list` 空無一物，而 stack 自己的 web UI 顯示每個節點都
  就緒
- `ros2 bag record` 錄出一個零筆訊息的 bag
- 等 `/clock` 的腳本一路等到 timeout，而 bag 明明正在播放

這在本專案發生過不只一次，最近一次是 demo runner 只載入了 ROS 與工作空間，沒有
載入 Autoware。

### 用一行取代兩行

```bash
source scripts/env.sh
```

`scripts/env.sh` 會依照 `versions.yaml` 記錄的 Autoware 安裝路徑載入，再載入工作
空間，順序固定。每個 `just` recipe 都用它。想看清楚發生什麼事時用那兩行，不想看
時用這一行。

### 用哪套 middleware 由你決定，而它住在 `.envrc`

本專案不會替你決定 RMW。`.envrc` 是唯一做這個決定的檔案，預設值就是 Autoware 選的
那個——目前是 CycloneDDS。

```bash
# 只在這個目錄改用 Zenoh
RMW_IMPLEMENTATION=rmw_zenoh_cpp direnv reload
```

之所以要繞這一圈，是因為 Autoware 的 `setup.bash` 會**無條件** export
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`：你先 export 自己的選擇、再載入 Autoware，
選擇就被無聲蓋掉。`.envrc` 會在載入前記下你的選擇、載入後再放回去，
`scripts/env.sh` 也做同樣的事，所以 `just` recipe 也不會蓋掉它。

不論選哪一個，圖裡的每個 process 都必須一致。

!!! tip "沒有 direnv 時"

    [direnv](https://direnv.net/) 會在進入目錄時套用 `.envrc`，這是最不容易出錯的
    做法，值得花五分鐘安裝。沒有它的話，**每個**終端機都要自己載入：

    ```bash
    source .envrc      # PATH_add / watch_file 是 direnv 的指令，會回報找不到；
                       # 其餘都會生效
    ```

    上課時特別容易出事：只要有一位學生的終端機漏掉這一步，他的 stack 就會啟動得
    很漂亮，然後跟誰都講不上話。

## 每一層實際給你什麼

這是本頁的重點，所以以下是實測而非斷言。每一列都是一個完全乾淨、沒有繼承任何
變數的 shell：

| 已 source | PATH 上有 `ros2` | `autoware_launch` | `autosdv_launch` | `RMW_IMPLEMENTATION` |
|---|---|---|---|---|
| 什麼都沒有 | **找不到** | — | — | — |
| `/opt/ros/humble/setup.bash` | 有 | **找不到** | **找不到** | **未設定** |
| `/opt/autoware/1.5.0/setup.bash` | 有 | `/opt/autoware/1.5.0` | **找不到** | `rmw_cyclonedds_cpp` |
| **只有** `install/setup.bash` | 有 | `/opt/autoware/1.5.0` | `…/install/autosdv_launch` | **未設定** |
| 先 Autoware 再工作空間 | 有 | `/opt/autoware/1.5.0` | `…/install/autosdv_launch` | `rmw_cyclonedds_cpp` |

請把最後兩列一起讀。只載入工作空間就找得到**每一個**套件，因為建置當下已經把底下
那幾層記了下來——但 middleware 仍然是未設定的。套件能不能被找到，和要用哪一套
middleware，是兩個不同的問題，而只有第二個需要那行 Autoware。

**這就是你的相依套件的來源。** 不是來自建置——建置只產生了 `install/`。AutoSDV
的節點所連結的 ROS 2 與 Autoware 套件、它們發佈的訊息型別、它們 include 的啟動
檔，全都在*執行時*透過這條鏈解析。少一層就是少一個相依套件，每次都是。

## 簡述 overlay

ROS 2 把每個載入的層稱為底下各層的 **overlay**，底下的則是 underlay。載入的動作
是往一個搜尋路徑 `AMENT_PREFIX_PATH` 附加內容，而套件是沿著它尋找的。

```bash
echo $AMENT_PREFIX_PATH | tr ':' '\n'
```

在上面那些乾淨的 shell 中，只有 ROS 2 時該路徑有 1 個項目；加上 Autoware 後是 2
個——Autoware 是單一合併安裝，只貢獻一個前綴；載入工作空間後變成 33 個，因為
`--symlink-install` 的工作空間是每個套件貢獻一個項目，而底下那兩層是透過記錄下來
的鏈一起帶進來的，不另外計數。

兩個值得記住的後果：

- **順序有影響。** 先載入 underlay。在 Autoware 之前載入工作空間會讓層的順序
  錯誤，雖然它常常看起來能動，但套件解析已經不是你以為的那樣了。
- **這是每個終端機各自的事。** 載入只會改變*單一 shell* 的環境變數。新分頁、新
  的 SSH 連線、新的終端機分割——每一個都是乾淨的開始，都需要再執行那兩行。沒有
  任何東西寫進磁碟，也沒有任何東西會留存。

## 少了一層時看起來是什麼樣子

以下是那些錯誤，以及它們實際的意思。

### `ros2: command not found`

什麼都沒載入。你在一個全新的終端機裡。

### `Package 'autosdv_launch' not found`

Autoware 載入了，工作空間沒有。可能是你忘了第二行，或你不在儲存庫根目錄，或
`just build` 還沒執行過。

```bash
ls install/setup.bash    # 若不存在，請先建置
```

### 錯誤指向某個 Autoware 套件，而你在處理 AutoSDV

例如某個啟動檔找不到 `autoware_launch`，或某個節點因缺少訊息型別而無法啟動。
**這是最容易誤導人的一種。** 訊息指向一個 AutoSDV 的檔案，看起來像 AutoSDV 的
錯誤，但它不是——你載入了 `install/setup.bash`，卻沒有載入底下的 Autoware 層。

在你除錯任何其他東西之前先檢查：

```bash
ros2 pkg prefix autoware_launch   # 應為 /opt/autoware/1.5.0
ros2 pkg prefix autosdv_launch    # 應為 …/AutoSDV/install/autosdv_launch
```

如果第一個失敗，就是 underlay 不見了。你正在看的其他東西都不是真正的問題。

### 在某個終端機能動，另一個不能

第一個終端機載入了，第二個沒有。這也是「在我機器上可以」的成因——已載入的環境
會把變數傳給子行程，所以從一個已載入的 shell *啟動*的 shell 本來就有那些變數。

## 接著才談那些便利工具

上面所有內容都是真正的機制。以下的東西只是把它自動化，而你應該能認出它沒有做
別的事。

### direnv

儲存庫附有一份 `.envrc`，會在你進入該目錄時執行同樣那兩行：

```bash
sudo apt install direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc   # 或你所用 shell 的對應寫法
source ~/.bashrc

cd ~/AutoSDV
direnv allow
```

執行 `direnv allow` 之後，進入該目錄就會載入 Autoware，接著——若
`install/setup.bash` 存在——載入工作空間。該檔案的相關部分短到可以直接讀：

```bash
if [ -f /opt/autoware/1.5.0/setup.bash ]; then
    source /opt/autoware/1.5.0/setup.bash
...
if [ -f install/setup.bash ]; then
    watch_file install/setup.bash
    source install/setup.bash
fi
```

就只有這樣。它確實方便，但這也是為什麼一個新手可以工作一週而完全沒學到這些，
然後在第一次於別處開終端機時卡住——透過 SSH、在容器裡、在編輯器內建的 shell 中，
或在 `cron` 工作裡。

**如果你不用 direnv，也不會壞掉。** 把那兩行打出來就是了。

### `just`

同樣的關係。`just build` 執行一個帶特定旗標的 `colcon build`；`just launch` 執行
一個帶特定參數的 `play_launch` 指令。每個 recipe 內部會自行載入它需要的環境，這
也是為什麼 `just build` 在未載入環境的終端機中仍然可用。

本書全程都會把 `just` recipe 放在它所包裝的指令旁邊。請使用這些 recipe——它們比較
短，而且帶有重要的旗標——但要知道底下是什麼，因為某天出問題時，你會需要直接執行
底下那一條。

### 一行檢查你的環境

```bash
ros2 pkg prefix autoware_launch && ros2 pkg prefix autosdv_launch
```

印出兩個路徑就表示兩層都在，你可以不用再管這一頁了。

## 摘要

- Autoware 以 **Debian 套件**安裝；**AutoSDV 則是編譯的**。
- 兩行，依序：`/opt/autoware/1.5.0/setup.bash`，然後 `install/setup.bash`。
  第一行也會帶入 ROS 2。
- 那兩行就是**套件相依性被解析的地方**，而且是在執行時。
- 每一個新終端機都要再執行一次。
- `.envrc` 與 `just` 自動化的正是這件事，沒有別的。

## 接下來

- [啟動檔](./launch-files.md) —— 環境對了之後，你要執行的東西
- [檢視執行中的系統](./inspecting.md) —— 主題、QoS 與頻率
- [軟體安裝](../getting-started/installation/overview.md)
