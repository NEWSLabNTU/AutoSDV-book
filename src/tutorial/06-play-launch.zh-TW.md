<!--
Translation Metadata:
- Source file: 06-play-launch.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 6. play_launch

本書中每一條啟動指令都是 `play_launch launch`。本頁說明它是什麼、為什麼這裡用它，
以及——出問題時最重要的部分——**它是我們自己的軟體，以及如何繞過它。**

## 它是什麼

一個取代 `ros2 launch` 的啟動執行器：

```bash
play_launch launch <package> <launch_file> [name:=value ...]
```

相同的三個位置、相同的參數。設定程式會安裝它，也可以從 PyPI 安裝：

```bash
pip install play_launch
play_launch setcap      # 選用：每行程 I/O 監控、非 root 的即時排程
```

## 本專案為什麼用它

**它能正確關閉系統。** 這是主要理由。AutoSDV 把多數節點以可組合節點的形式跑在容器
行程內。殺掉 `ros2 launch` 只會殺掉啟動器，留下那些容器繼續執行——仍在發佈、仍佔著
記憶體與 GPU，並且比啟動它們的終端機活得更久。`play_launch` 會對整個行程群組逐級
升高 SIGINT → SIGTERM → SIGKILL。

**一個網頁介面**，預設在 `http://127.0.0.1:8080`，列出每個節點的狀態並串流其紀錄。
當 34 個節點同時啟動時，這比讀一份合併的主控台輸出快得多。

**監控** —— 每個行程的 CPU、記憶體、I/O 與 GPU，加上 `/diagnostics`。

**一套解析／重放流程：**

```bash
play_launch resolve <pkg> <file> args…        # 會啟動什麼，但不啟動
play_launch dump launch <pkg> <file> args…    # 寫出已解析的系統模型
play_launch up system_model.yaml              # 由該模型重複啟動
```

`resolve` 是你最常用的那一個。它在不啟動任何東西的情況下回答「我的參數有沒有生效」。

## 它是我們的軟體，而 `ros2 launch` 才是基準

`ros2 launch` 是 ROS 2 的一部分。生態系中每個啟動檔都是對著它撰寫與測試的。
`play_launch` 由本專案維護，所以當某個啟動檔在它底下表現異常時，第一個該問的問題不是
*這個啟動檔哪裡有問題*，而是**是不是我們的問題？**

這個工具自己的選項就告訴你它可能在哪裡不同。

### 解析器

`play_launch` 為了速度用 Rust 重新實作了啟動檔語言，並附上原本的 Python 實作作為
備援：

```bash
play_launch launch <pkg> <file> --parser python
```

這個旗標的文件寫的是「為求最大相容性」，那就是一個明白的承認：快速路徑是一個重新
實作，在不尋常的檔案上可能與基準實作不一致。如果某個啟動檔解析失敗，或解析出你沒
預期的結果，請先試這個。

### 容器模式

`play_launch` 預設會覆寫可組合節點的容器化方式，以取得行程隔離：

```bash
play_launch launch <pkg> <file> --container-mode isolated     # 預設：每個節點 fork+exec
play_launch launch <pkg> <file> --container-mode observable   # 共用行程，ComponentEvents
play_launch launch <pkg> <file> --container-mode stock        # 完全不覆寫
```

`stock` 會使用啟動檔原本要求的容器，不做任何改變。如果可組合節點行為怪異——某個節點
載入不了，或在 `ros2 launch` 下可以而在這裡不行——就該試這個設定。

## 退回的階梯

依序。每一步都放棄 `play_launch` 的一項行為：

```bash
play_launch launch <pkg> <file> …                        # 1. 預設
play_launch launch <pkg> <file> --parser python          # 2. 若解析有誤
play_launch launch <pkg> <file> --container-mode stock    # 3. 若可組合節點異常
ros2 launch <pkg> <file> …                                # 4. 基準；永遠可用
```

如果你走到第 4 步，請記得你也放棄了關閉處理，所以要殺行程群組：

```bash
kill -- -$(ps -o pgid= -p <pid> | tr -d ' ')
ros2 node list    # 確認為空
```

## 是誰的錯

一條簡單的規則：

> **如果它在 `ros2 launch` 下可以、在 `play_launch` 下不行，那就是 `play_launch`
> 的 bug。**

請回報到 [play_launch](https://github.com/NEWSLabNTU/play_launch)，而不是開成
AutoSDV 的 issue。附上啟動檔、參數，以及階梯上哪一階讓它能動——最後那個細節通常就
指出了該負責的組件。

反過來說，如果兩邊都失敗，那問題出在啟動檔或設定，而 `play_launch` 是無辜的。

這個測試只花一個指令，卻能省下大量方向錯誤的除錯，這也是本頁存在的唯一理由。

## 通訊埠的混淆

值得知道，因為它看起來像失敗：

| 啟動方式 | 網頁介面 |
|---|---|
| `play_launch launch …` | `http://127.0.0.1:8080` |
| `just launch`、`just sim …`、`just demo run` | `http://localhost:8081` |

那些 `just` recipe 會傳 `--web-addr 0.0.0.0:8081`。開錯通訊埠會什麼都看不到，看起來
就跟系統啟動失敗一模一樣。

`0.0.0.0` 也表示網路上的其他機器連得到這個介面，在車上很方便，而在共用機器上則值得
知道。

## 升級

`play_launch` 有過破壞性變更——0.9.0 把 `replay` 改成 `up`，還有其他幾項，而它的
changelog 指出其中有些是改變行為而不是報錯。設定程式會安裝一個已知能與本儲存庫搭配
的版本。如果你自行升級它，而啟動開始表現怪異，那是一個可能的原因。

## 你完成教學了

你現在可以：

- 用一條指令執行整個堆疊，並在不留下孤兒行程的情況下停止它
- 駕駛路徑規劃模擬器，並讀懂狀態面板
- 重播真實感測器資料，並判斷定位是否真的在運作
- 傳入參數，並檢查它們是否生效
- 在出問題時往下降一層

接下來可以去：

- **[定位方法](../guides/localization-methods.md)** —— `pose_source` 還能是什麼
- **[預設組態](../guides/presets.md)** —— 設定感知與定位
- **[操作車輛](../running/on-the-vehicle.md)** —— 完整的參數參考
- **[地圖](../guides/maps.md)** —— 為你自己的場地建一張地圖
