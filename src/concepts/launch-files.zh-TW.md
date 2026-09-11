<!--
Translation Metadata:
- Source file: launch-files.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 啟動檔

AutoSDV 中幾乎所有東西都是啟動檔。整個駕駛系統是一個；每個感測器驅動程式、每個
組件群組、每個預設組態也都是。如果你要改變 AutoSDV 的執行方式，你會透過傳入啟動
參數或編輯啟動檔來做——所以花二十分鐘理解它們的運作方式是值得的。

本頁只涵蓋 AutoSDV 實際用到的部分。完整的啟動系統請參閱
[ROS 2 launch 文件](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)。

## 什麼是啟動檔

一個啟動其他程式的程式。它描述要執行哪些節點、給它們什麼參數、重新對應哪些主題，
以及要 include 哪些其他啟動檔。執行方式是：

```bash
ros2 launch <package> <launch_file> [name:=value ...]
```

package 是一個已安裝的 ROS 2 套件；launch file 是該套件 `launch/` 目錄中的一個
檔案。AutoSDV 最上層的是：

```bash
ros2 launch autosdv_launch autosdv.launch.yaml
```

!!! note "在本專案中你通常會打 `play_launch launch`"

    相同的三個位置、相同的參數。它是我們自己的啟動執行器，值得知道為什麼用它，
    以及什麼時候不要用——參閱
    [play_launch](../tutorial/06-play-launch.md)。本頁的所有內容對兩者都成立。

## 三種檔案格式，同一種意義

ROS 2 接受以 Python、XML 或 YAML 撰寫的啟動檔，而 AutoSDV 三種都用。它們表達的
是同樣的東西。

=== "YAML"

    ```yaml
    launch:
      - arg:
          name: pose_source
          default: "cuda_ndt"
      - node:
          pkg: my_package
          exec: my_node
    ```

=== "XML"

    ```xml
    <launch>
      <arg name="pose_source" default="cuda_ndt"/>
      <node pkg="my_package" exec="my_node"/>
    </launch>
    ```

需要邏輯的地方會用 Python 檔——迴圈，或難以宣告式表達的條件。其餘地方偏好 YAML
與 XML，因為它們比較好讀、也比較好做 diff。

## 參數（argument）與節點參數（parameter）

這是兩種不同的東西，而兩者的差別會造成真正的混淆。

**argument** 是*啟動檔*的輸入。它在啟動檔被求值期間存在，之後就消失了。

```yaml
- arg:
    name: pose_source
    default: "cuda_ndt"
    description: "Pose estimation source"
```

你在命令列上設定它：

```bash
ros2 launch autosdv_launch autosdv.launch.yaml pose_source:=ndt
```

注意是 `:=`，不是 `=`。另外注意**每個值都是字串**——沒有型別化的參數，所以布林值
就是小寫的文字 `true` 或 `false`。

**parameter** 屬於某個*節點*，在節點執行期間存在，可以被讀取、有時也可以在執行時
寫入：

```bash
ros2 param list /some_node
ros2 param get /some_node some_parameter
```

兩者的連結在於：啟動檔取用它的 argument，並把它們作為 parameter 交給節點：

```yaml
- node:
    pkg: my_package
    exec: my_node
    param:
      - name: source
        value: "$(var pose_source)"
```

於是命令列上的 `pose_source:=ndt` 變成節點參數 `source`，值為 `ndt`。這條鏈——
命令列 → 啟動參數 → 替換 → 節點參數——是要記住的那一條。

## 替換（substitution）

替換是在啟動檔求值期間計算出來的值。本專案中經常出現四種。

| 替換 | 意義 |
|---|---|
| `$(var name)` | 某個啟動參數的值 |
| `$(find-pkg-share pkg)` | 某個套件已安裝的 `share/` 目錄 |
| `$(env NAME default)` | 一個環境變數，帶預設值 |
| `$(eval "...")` | 一小段 Python 運算式 |

`find-pkg-share` 是啟動檔在不寫死路徑的情況下引用套件內檔案的方式：

```yaml
rviz_config: "$(find-pkg-share autosdv_launch)/rviz/autosdv.rviz"
```

它透過[環境那一頁](./environment.md)描述的同一個 `AMENT_PREFIX_PATH` 解析——換句
話說，少一行 `source` 也會讓啟動檔壞掉，而且錯誤訊息講的是某個套件，而不是你的
環境。

`$(env ...)` 是模型目錄取得覆寫值的方式：

```yaml
default: "$(env AUTOSDV_DATA_PATH ./data/autoware_data)"
```

## include，以及參數如何傳遞

一個啟動檔可以執行另一個，並把值往下傳：

```yaml
- include:
    file: "$(find-pkg-share autosdv_launch)/launch/components/tier4_localization_component.launch.xml"
    arg:
      - name: pose_source
        value: "$(var pose_source)"
```

AutoSDV 就是這樣組織的：`autosdv.launch.yaml` 宣告參數並 include 各組件的啟動檔，
而那些檔案再 include 個別套件的啟動檔。

**參數不會自動往下傳。** 如果某個 include 沒有把 `pose_source` 傳下去，被 include
的檔案就會使用它自己的預設值，而你在命令列上給的值會被靜默忽略。這類 bug——命令列
接受了一個值，但它沒有到達任何地方——正是下一節存在的原因。

## 看看啟動檔實際解析成什麼

不要用猜的。在不執行的情況下解析啟動：

```bash
play_launch resolve autosdv_launch autosdv.launch.yaml \
  pose_source:=mcl -o ./tmp/resolved.yaml
```

輸出會列出所有將被啟動的節點，以及它們將取得的所有參數。這回答了「我的參數有沒有
生效」，而且不花任何代價——不會啟動任何節點。

若只想看某個啟動檔接受哪些參數：

```bash
ros2 launch autosdv_launch autosdv.launch.yaml --show-args
```

## 預設組態也是啟動檔

一旦 argument 與 include 都理解了，AutoSDV 的預設組態系統就不再是一個獨立的功能。

預設組態就是一個只宣告參數預設值的啟動檔：

```yaml
# config/perception/preset/lidar_only_preset.yaml
launch:
  - arg:
      name: perception_mode
      default: "lidar"
  - arg:
      name: use_traffic_light_recognition
      default: "false"
```

而主啟動檔以名稱 include 其中一個：

```yaml
- include:
    file: "$(find-pkg-share autosdv_launch)/config/perception/preset/$(var perception_preset)_preset.yaml"
```

因為預設組態提供的是*預設值*，而預設值只在沒有其他來源提供值時才生效，所以你在
命令列上傳的參數會勝過預設組態：

```bash
ros2 launch autosdv_launch autosdv.launch.yaml \
  perception_preset:=camera_lidar_fusion \
  use_traffic_light_recognition:=false
```

整個機制就是這樣。參閱[預設組態](../guides/presets.md)。

## 編輯啟動檔

工作空間以 `--symlink-install` 建置，意思是 `install/` 內含的是指向你原始檔的
連結，而不是副本。所以：

- **編輯既有的** `.yaml`、`.xml` 或 `.py` 會在下次啟動時生效，不需重新建置。
- **新增檔案**需要 `just build` 來建立它的符號連結。在那之前，檔案存在於 `src/`，
  但對 ROS 而言並不存在。

這個不對稱會坑到那些新增了預設組態卻發現找不到的人。

## 簡述可組合節點

有些 ROS 2 節點可以載入到一個共用行程中——一個 **component container**——而不是各自
以獨立行程執行。它們接著在同一個行程內傳遞訊息，而不是透過網路，對點雲來說這是
很大的節省。

Autoware 大量使用這個機制，而它有兩個你會遇到的後果：

- 以 PID 殺掉啟動指令會讓那些容器變成孤兒行程，仍佔著記憶體與 GPU。請改殺整個
  行程群組。
- CUDA 點雲管線*要求*它的各階段位於同一個容器中，因為它們彼此傳遞的是 GPU 指標
  而不是資料。參閱 [CUDA 管線](../guides/cuda-pipeline.md)。

## 接下來

- [檢視執行中的系統](./inspecting.md) —— 既然它已經在跑了
- [Autoware 管線](./autoware-conventions.md) —— 那些主題名稱的意義
- [操作車輛](../getting-started/usage.md) —— 完整的參數清單
