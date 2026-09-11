<!--
Translation Metadata:
- Source file: development.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 開發指南

給想要擴充、修改或貢獻 AutoSDV 的開發者。

## 處理程式碼

- **[原始碼說明](./source-code.md)** —— 儲存庫結構，以及各套件之間的關係
- **[版本控制](./version-control.md)** —— Git 超專案與其子模組，本工作空間的
  大部分都是子模組

## 設定系統

- **[預設組態](./presets.md)** —— `perception_preset` 與 `localization_preset`
  如何把參數分組，以及如何新增一個
- **[定位方法](./localization-methods.md)** —— `pose_source` 在選擇什麼，各個
  選項各自需要什麼
- **[地圖](./maps.md)** —— 每種方法消費哪種地圖產物，以及如何建立與驗證
- **[CUDA 點雲管線](./cuda-pipeline.md)** —— GPU 前處理，以及支配它的單一容器
  限制

## 感測器與車輛

- **[感測器整合](./sensor-integration/using-sensors.md)** —— 可用的感測器、
  它們的驅動程式與設定，以及如何新增一個
- **[車輛控制](./vehicle-control/overview.md)** —— 控制系統、其硬體與調校
- **[車輛介面](../reference/software/vehicle-interface.md)** —— 從 Autoware
  控制命令到致動器的橋接

## 在你動手改任何東西之前

有兩個習慣值得在這裡重述，因為兩者都能避免白做的工。

**從 git 讀取設定，而不是從子模組的工作樹。** 工作樹可能領先於、落後於、或
完全無關於超專案實際固定的那個 commit，因此從它得出的結論可能是關於一份別人
根本沒有的程式碼。

**先推送子模組，再推送 pin。** 超專案的 pin 是一個 commit hash，而只存在於你
本機檢出中的 hash，是別人無法解析的 pin——他們的 `git submodule update` 會失敗，
CI 也會跟著失敗。參閱[版本控制](./version-control.md)。
