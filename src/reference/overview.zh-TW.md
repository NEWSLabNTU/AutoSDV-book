<!--
Translation Metadata:
- Source file: overview.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 技術參考

本節提供 AutoSDV 平台的詳細技術規格、配線圖與組件細節。此資訊以深入的技術文件補充型號特定指南。

## 內容

### 核心組件
- [核心組件規格](./hardware/core-components.md) - 所有基礎平台組件的詳細規格
- [配線圖](./hardware/wiring-diagrams.md) - 電氣連接與電源分配

### 進階主題
- [5G/LTE 部署](./networking/5g-deployment.md) - 行動網路連線設定的詳細指南
- [感測器整合](../guides/sensor-integration/using-sensors.md) - 完整的感測器整合指南
- [車輛介面](./software/vehicle-interface.md) - CAN 匯流排與致動器控制

## 快速參考表

### 電源需求
| 組件                 | 電壓     | 電流（典型） | 電流（峰值） |
|---------------------|----------|-------------|-------------|
| Jetson AGX Orin     | 9-20V    | 2.5A @ 12V  | 4A @ 12V    |
| Velodyne VLP-32C    | 9-32V    | 1A @ 12V    | 1.5A @ 12V  |
| Robin-W LiDAR       | 12-24V   | 0.7A @ 12V  | 1A @ 12V    |
| Cube1 LiDAR         | 12-24V   | 0.7A @ 12V  | 1A @ 12V    |
| ZED X Mini          | 12V      | 0.5A        | 0.8A        |
| 5G Module           | 12V      | 1.2A        | 2A          |

### 通訊介面
| 介面             | 用途         | 協定      | 頻寬       |
|-----------------|-------------|----------|-----------|
| Ethernet (GbE)  | LiDAR 資料   | UDP      | 1 Gbps    |
| USB 3.0         | 相機資料     | USB      | 5 Gbps    |
| CAN Bus         | 車輛控制     | CAN 2.0B | 1 Mbps    |
| I2C             | 感測器       | I2C      | 400 kbps  |
| UART            | GPS/IMU     | Serial   | 115200 baud |

### 環境規格
| 參數             | 操作範圍         | 儲存範圍         |
|-----------------|-----------------|-----------------|
| 溫度             | -10°C 至 +50°C  | -20°C 至 +60°C  |
| 濕度             | 10% 至 90% RH   | 5% 至 95% RH    |
| 振動             | 2G RMS          | 5G RMS          |
| 衝擊             | 15G 峰值        | 30G 峰值        |
| 防護等級         | IP54（含外殼）  | -               |

## 系統架構

### 資料流
```
Sensors → Processing → Decision → Control → Actuators
   ↓          ↓           ↓         ↓          ↓
LiDAR    Perception   Planning   Commands   Motors
Camera   Localization  Safety    Validation  Steering
IMU/GPS  Fusion        Behavior  Monitoring  Brakes
```

### 軟體堆疊
```
Application Layer:     User Applications
     ↓
Autoware Layer:       Perception, Planning, Control
     ↓
ROS 2 Middleware:     DDS Communication
     ↓
Driver Layer:         Sensor/Actuator Drivers
     ↓
OS Layer:            Ubuntu 22.04 + RT Kernel
     ↓
Hardware Layer:       Jetson AGX Orin
```

## 維護排程

### 每日檢查
- 目視檢查組件
- 電池電壓檢查
- 感測器鏡頭清潔

### 每週維護
- 連接器檢查
- 輪軸承檢查
- 軟體更新

### 每月維修
- 完整系統診斷
- 校正驗證
- 效能基準測試

### 年度大修
- 完全拆解與檢查
- 軸承更換（若為機械式光達）
- 感測器重新校正
- 結構完整性檢查

## 疑難排解快速參考

### 常見問題
| 症狀           | 可能原因         | 解決方案           |
|---------------|-----------------|-------------------|
| 無 LiDAR 資料  | 網路配置         | 檢查 IP 設定       |
| 定位不良       | 地圖不匹配       | 重新產生地圖       |
| 運動不穩定     | IMU 校正         | 重新校正 IMU       |
| 電池壽命短     | 高負載           | 檢查 CPU 使用率    |
| 通訊中斷       | 干擾             | 檢查無線頻道       |

## 安全規格

### 緊急停止
- 硬體急停按鈕
- 軟體緊急煞車
- 遠端終止開關（5G 型號）
- 自動故障偵測

### 操作限制
- 最高速度：15 km/h
- 最大負載：5 kg
- 最大坡度：15°
- 最小照明：10 lux（含燈光）

## 合規與認證

### 標準
- ROS 2：符合 REP-2000
- 安全：符合 ISO 26262 ASIL-B
- EMC：FCC Part 15 Class B
- 環境：通過 MIL-STD-810G 測試

### 文件
- 提供完整原理圖
- 開源軟體
- 硬體設計檔案
- 測試報告

## 額外資源

- [硬體組裝指南](../getting-started/hardware-assembly.md) - 逐步組裝
- [軟體安裝](../getting-started/installation/overview.md) - 軟體設定
- [開發指南](../guides/development.md) - 自訂開發
