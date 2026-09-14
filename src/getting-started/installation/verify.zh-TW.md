<!--
Translation Metadata:
- Source file: verify.md
- Last synced: 2026-09-14
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 驗證安裝

四個快速檢查，確認軟體確實在硬碟上、而且 ROS 看得見。這些檢查都不會啟動整套
車輛系統——那是[教學](../../tutorial/00-what-you-will-build.md)的工作，也是真正
的證明。

## 1. setup 認為裝了什麼？

```bash
./setup.sh --status
```

你選取過的步驟都不應該顯示為缺少。刻意跳過的步驟——例如沒有 ZED 相機時的
`zed-sdk`——顯示為未安裝才是正確結果。

這道指令會盡可能直接讀取機器狀態，所以它的答案可能和你記得執行過的內容不同。
兩者不一致時，通常是它對。

## 2. Autoware 在該在的位置嗎？

```bash
ls /opt/autoware/1.5.0/setup.bash      # 環境腳本
ls data/autoware_data | head           # 可寫入的模型樹
```

兩者都必須存在。第二個是設定程式建立的符號連結農場；缺少它，感知模組會在每次
啟動時重新編譯模型。

## 3. ROS 看得到這些套件嗎？

先 source 環境，再問 ROS 手上有什麼：

```bash
source /opt/autoware/1.5.0/setup.bash  # 包含 ROS 2
source install/setup.bash              # 你剛建置好的工作空間

ros2 pkg list | wc -l                  # 應該是數百，不是零
ros2 pkg list | grep autosdv           # 本專案的套件
ros2 pkg prefix autoware_launch        # Autoware，來自 Debian 安裝
```

`grep autosdv` 應該列出啟動器、感測器套件與車輛套件。如果連 `ros2` 都找不到，
就是環境沒有 source——那正是[環境與相依套件](../../concepts/environment.md)整頁
在講的事。

## 4. Rust 套件建置起來了嗎？

```bash
ls install/cuda_ndt_matcher
```

它值得單獨檢查，因為它是會安靜失敗的那一個：少了 Rust 工具鏈或 colcon 外掛，
colcon 會跳過它，而建置仍然回報成功。見
[疑難排解](./recommended.md#建置成功但少了-cuda_ndt_matcher)。

## 然後跑點東西

```bash
just demo check
```

會回報 rosbag、地圖、建置結果、`play_launch`、CUDA 工具鏈與你的 GPU 是否相符，
以及是否有可用的顯示器。

接著就去看[教學](../../tutorial/00-what-you-will-build.md)。它會啟動路徑規劃
模擬——不需要感測器、不需要 GPU、不需要下載——只要車子能開到目標點，底下每一層
就都是好的。

## 如果有東西失敗了

安裝頁面的[疑難排解](./recommended.md#疑難排解)涵蓋了那些有明確原因的失敗：
被安靜跳過的 Rust 套件、小到沒有任何節點能啟動的核心 socket 緩衝區、重開機後
失去 multicast 的 loopback 介面，以及每次啟動都重新編譯的模型。
