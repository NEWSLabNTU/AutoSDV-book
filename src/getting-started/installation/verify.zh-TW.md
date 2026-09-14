<!--
Translation Metadata:
- Source file: verify.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 驗證安裝

四項檢查，依它們能證明多少事情排序。請在[軟體安裝](./overview.md)之後執行。
每一項都會告訴你前一項無法告訴你的事。

## 1. 設定程式認為裝了什麼？

```bash
./setup.sh --status
```

每個步驟都會列出其狀態。請記得這個指令在可能的情況下會**讀取機器實際狀態**，
因此它的答案可能與你記得執行過的內容不同——而兩者不一致時，通常是它對。

這裡重要的是：你選取過的步驟都沒有顯示為缺少。至於你刻意略過的步驟（例如沒有
ZED 相機時的 `zed-sdk`）顯示為未安裝，那是正確的結果，不是失敗。

## 2. 環境會啟用嗎？

開啟一個**新的** shell 並進入儲存庫：

```bash
cd ~/AutoSDV
ros2 --help
```

如果找不到 `ros2`，表示 direnv 沒有掛進你的 shell，或你還沒執行
`direnv allow`。手動載入環境以確認問題僅止於此：

```bash
source /opt/autoware/1.5.0/setup.bash
source install/setup.bash
ros2 --help
```

## 3. 工作空間建置得起來嗎？

```bash
just build
```

預期結果：colcon 以一行 `Summary:` 結束，且沒有失敗的套件。stderr 上的警告是
正常的——數個第三方套件都會產生警告。

請特別確認那個 Rust 套件，因為它是會安靜失敗的那一個：

```bash
ls install/cuda_ndt_matcher
```

如果該目錄不存在，表示 colcon 略過了該套件。參閱
[疑難排解章節](./recommended.md#建置成功但少了-cuda_ndt_matcher)。

## 4. 系統真的起得來嗎？

這才是關鍵的檢查。兩個模擬，依其涵蓋範圍由小而大。

### 路徑規劃模擬 —— 不需感測器、GPU 或下載地圖

```bash
source install/setup.bash
play_launch launch autoware_launch planning_simulator.launch.xml \
  map_path:=$PWD/data/COSS-map-planning \
  vehicle_model:=autosdv_vehicle \
  sensor_model:=autosdv_sensor_kit
```

RViz 會開啟並載入 COSS Park 地圖。設定初始姿態、設定目標點，車輛便會規劃路線
並沿線行駛。若這能運作，代表你的 ROS 2 安裝、Autoware 安裝、建置結果以及
AutoSDV 車輛模型全都沒問題。

完整說明：[路徑規劃模擬](../../tutorial/02-planning-simulation.md)。

### 記錄回放模擬 —— 以錄製的感測器資料跑完整管線

```bash
just coss download-rosbag          # 約 2.8 GB，只需一次
```

```bash
source install/setup.bash
play_launch launch autosdv_launch logging_simulation.launch.yaml
```

並在第二個終端機中：

```bash
ros2 bag play data/rosbags/outdoor_20251226_153115 --clock
```

這會運作到定位與感知，而那是路徑規劃模擬不涵蓋的部分。在預設設定下，這也是
第一項需要 GPU 的檢查——純 CPU 的參數請參閱
[記錄回放模擬](../../tutorial/03-logging-simulation.md)。

## 檢查清單

- [ ] `./setup.sh --status` 中你選取過的項目都不缺少
- [ ] 在儲存庫內開新 shell 後 `ros2 --help` 可執行
- [ ] `just build` 完成且沒有失敗的套件
- [ ] `install/cuda_ndt_matcher` 存在
- [ ] 路徑規劃模擬會開啟 RViz 並駛向目標點
- [ ] 記錄回放模擬能對錄製的 bag 完成定位

## 如果有東西失敗了

安裝頁的[疑難排解章節](./recommended.md#疑難排解)涵蓋了那些有明確成因、
值得指名的失敗——被靜默略過的 Rust 套件、小到任何節點都無法啟動的核心 socket
緩衝區、重開機後失去多播的 loopback 介面，以及每次啟動都重新編譯的模型。
