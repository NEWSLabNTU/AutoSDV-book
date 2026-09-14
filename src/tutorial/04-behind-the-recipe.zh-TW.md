<!--
Translation Metadata:
- Source file: 04-behind-the-recipe.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 4. 拆解那條指令

`just demo run` 做了很多事，卻什麼都沒給你看。本頁一層一層把它打開，由上往下——
因為某天出問題時，你會需要執行壞掉那層底下的那一層。

總共四層：

```
just demo run                  ← 一個 recipe
  play_launch launch …         ← 一個啟動執行器（我們的）
    ros2 launch …              ← 官方的啟動執行器
      source …/setup.bash      ← 一切都需要的環境
```

## 第 4 層 —— 環境

在最底下，永遠是那兩行：

```bash
source /opt/autoware/1.5.0/setup.bash   # ROS 2 Humble 與 Autoware
source install/setup.bash               # AutoSDV 工作空間
```

`just` recipe 在內部會做這件事，這也是為什麼 `just demo run` 在一個連 `ros2` 都不在
`PATH` 上的終端機裡仍然可用。那份便利，也正是為什麼人們第一次手動執行東西時會卡住。

如果本頁任何指令以「command not found」或「package not found」失敗，就是這一層。
參閱[環境與相依套件](../concepts/environment.md)。

## 第 3 層 —— `ros2 launch`

啟動 ROS 2 系統的標準方式：

```bash
ros2 launch <package> <launch_file> [name:=value ...]
```

對記錄回放模擬而言：

```bash
ros2 launch autosdv_launch logging_simulation.launch.yaml
```

三個位置：已安裝的套件、其中的啟動檔，然後是 `name:=value` 形式的參數。（啟動檔拿
這些參數做什麼，參閱[啟動檔](../concepts/launch-files.md)。）

它可以用，而且它是官方實作。對這樣的系統它有一個實際問題，那就是上面還有一層的原因。

### 孤兒行程問題

AutoSDV 把多數節點以**可組合節點**的形式跑在容器行程內。`ros2 launch` 啟動那些容器；
以 PID 殺掉 `ros2 launch` 只會殺掉啟動器，留下那些容器繼續執行——仍在發佈，仍佔著
記憶體與 GPU。它們不會因為你關掉終端機而停止。

如果你用 `ros2 launch`，請殺掉**行程群組**：

```bash
ros2 launch autosdv_launch logging_simulation.launch.yaml &
LAUNCH_PID=$!
# ... 測試中 ...
kill -- -$(ps -o pgid= -p $LAUNCH_PID | tr -d ' ')
```

然後確認：

```bash
ros2 node list    # 應為空
```

## 第 2 層 —— `play_launch`

```bash
play_launch launch autosdv_launch logging_simulation.launch.yaml
```

相同的三個位置、相同的參數——`ros2 launch` 的直接替代品。它替你做上面那個行程群組
關閉，並加上 `http://127.0.0.1:8080` 的網頁介面、每個行程的 CPU／記憶體／GPU 監控，
以及 `/diagnostics` 收集。

它是**我們自己的軟體**，這件事重要到值得有
[專屬的一頁](./06-play-launch.md)，說明它何時可能與 `ros2 launch` 不同，以及如何
退回去。

### 回答「我的參數有沒有生效」的那個指令

```bash
play_launch resolve autosdv_launch autosdv.launch.yaml \
  pose_source:=mcl -o ./tmp/resolved.yaml
```

它會求值啟動檔，並寫出它*將會*啟動的每個節點與它們*將會*取得的每個參數，而不啟動
任何東西。當某個參數似乎沒有作用時，這就是你查出它有沒有到達節點的方法——而且比
啟動起來看還快。

## 第 1 層 —— 那個 recipe

```bash
just demo run
```

recipe 位於 `demo/justfile`，它呼叫 `demo/scripts/run-coss-ndt.sh`。它的核心是：

```bash
play_launch launch --web-addr 0.0.0.0:8081 \
    autosdv_launch logging_simulation.launch.yaml \
    pose_source:=$POSE_SOURCE \
    map_path:=$MAP \
    use_gnss:=false \
    rviz:=$RVIZ
```

加上你沒看到的編排：

1. **取得 rosbag**（若不存在）
2. **停掉先前的堆疊**——兩個會爭搶同樣的主題
3. **等待 `ndt_scan_matcher` 出現**，再給地圖 25 秒載入。這就是為什麼 recipe 比手動
   做更可靠：它等的是一個*條件*，而不是一個猜出來的秒數
4. **啟動輪速縮放器**，並把 bag 的原始速度主題重新導向經過它
5. **錄製診斷**到 `tmp/demo-runs/<label>_<stamp>/bag`
6. **播放 bag**，並在第 8 秒植入一個已知姿態
7. **印出量測數據**並讓堆疊繼續執行

第 3 與第 6 步是讓結果可重現的關鍵，也正是手動操作的人最容易弄錯的地方。

### 手動重現它

```bash
# 終端機 1
source /opt/autoware/1.5.0/setup.bash && source install/setup.bash
play_launch launch autosdv_launch logging_simulation.launch.yaml \
  pose_source:=cuda_ndt use_gnss:=false

# 終端機 2 —— 等地圖載入完成後
source /opt/autoware/1.5.0/setup.bash && source install/setup.bash
ros2 bag play data/rosbags/outdoor_20251226_153115 --clock

# 終端機 3 —— 植入姿態，或在 RViz 中點 2D Pose Estimate
python3 demo/scripts/seed_initialpose.py
```

同樣的系統。你失去的是等待、錄製與量測。

## `just` recipe 與它們包裝的東西

本書全程都會把 recipe 放在它底層的指令旁邊。值得知道的有：

| Recipe | 包裝的是 |
|---|---|
| `just build` | `colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --cargo-args --release` |
| `just launch` | `play_launch launch --web-addr 0.0.0.0:8081 autosdv_launch autosdv.launch.yaml`（在沒有 `$DISPLAY` 時另加 `rviz:=false`） |
| `just sim logging` | 同上，但用 `logging_simulation.launch.yaml` |
| `just sim planning` | `play_launch launch autoware_launch planning_simulator.launch.xml`，帶 COSS 地圖與 AutoSDV 模型 |
| `just demo stop` | 殺掉堆疊的行程群組 |

`just launch` 有兩個值得記住的細節：網頁介面移到 **8081**，而不是 `play_launch`
自己的 8080；而在 `$DISPLAY` 未設定時 RViz 會被靜默關閉。兩者從命令列上都看不出來，
這也是為什麼本書先教 `play_launch` 的形式。

給 recipe 的參數要放進單一個加引號的字串——這是 `just` 的要求，不是 ROS 的：

```bash
just launch "pose_source:=ndt launch_perception:=false"
```

## 該用哪一層

- **日常工作**：用 `just` recipe。它們比較短，而且帶有重要的旗標。
- **任何不尋常的事**——新的參數組合、不同的啟動檔、除錯：直接用
  `play_launch launch`。
- **當你懷疑啟動器本身時**：`ros2 launch`。參閱[下一頁](./06-play-launch.md)。

**接下來：** [5. 地圖與 Rosbag](./05-map-and-rosbag.md) —— 你一直在回放的資料究竟
是什麼。
