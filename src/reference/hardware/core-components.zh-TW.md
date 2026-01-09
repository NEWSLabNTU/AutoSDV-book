<!--
Translation Metadata:
- Source file: core-components.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 硬體組件與配線

## 硬體組件

車輛使用核心組件與選配的補充組件組裝而成。核心組件包括底盤與其他必要零件。補充組件，例如光達與 5G/LTE 模組，為選配項目，可根據您的特定需求選擇。

### 核心組件

| 項目                                                               |
|--------------------------------------------------------------------|
| **# 底盤**                                                         |
| Tekno TKR9500 Truck Kit 16×11×5 inch                               |
| ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯                           |
| **# 動力系統**                                                     |
| Brushless Motor4274 / 1500kv                                       |
| PCA9685 PWM Driver                                                 |
| DC-DC Converter IN 24V OUT 12V10A                                  |
| ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯                           |
| **# 電腦**                                                         |
| NVIDIA Jetson AGX ORIN Dev. kit 64GB / 32GB                        |
| Micron P3 PLUS 1000GB SSD                                          |
| ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯                           |
| **# 相機**                                                         |
| ZED X Mini Stereo Camera (Lens Focal Length 2.2mm, with polarizer) |
| ZED Link Capture Card                                              |
| GMSL2 Fakra Cable F-F(0.3m)                                        |
| ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯                           |
| **# 導航感測器**                                                   |
| KY-003 Hall Effect Sensor                                          |
| MPU9250 9-axis Motion Processing Unit                              |
| ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯                           |
| **# 電池與電源供應**                                               |
| Battery Gens ACE-5000mAh-45C-22.2V-6S1P-XT60                       |
| Battery Gens ACE-7200mAh-50C-7.4V-2S1P-21                          |
| Breaker 4230-T110-K0BE-8AB8                                        |

*表 1. 車輛的核心材料。*

### 補充：光達感測器

選擇光達感測器時，取決於特定的定位方法與期望的視覺品質。如果使用基於點雲的 NDT 定位，通常會選擇 Velodyne VLP-32C 光達，因其具有全景視野。相較之下，固態光達提供更高的點密度，使其更適合用於詳細的場景與物體特徵提取，以及與相機協作的視覺定位。

| 光達感測器            |                                           |
|-----------------------|-------------------------------------------|
| （從以下選擇一項）    |                                           |
| Seyond Robin-W        | Solid-State LiDAR with 120° FOV           |
| Blickfeld Cube 1      | Solid-State LiDAR with 70° FOV. (EOL)     |
| Velodyne VLP-32C      | Mechanical spinning LiDAR with 360° (EOL) |

*表 2. 車輛的建議光達感測器。*

### 補充：5G/LTE 通訊

Ataya 5G Harmony 套件已成功部署在車輛上，並經過國立台灣大學 NEWSLab 的檢驗。下表列出 5G 套件的關鍵組件。有關更詳細的規格與報價，請造訪 [Ataya 的網站](https://www.ataya.io/)。此外，請諮詢 [Spectrum Monitoring](https://www.spectrummonitoring.com/frequencies.php/) 的全球行動頻率資料庫以了解您所在地區的可用頻段。

| 5G/LTE 套件                       |                                                                |
|-----------------------------------|----------------------------------------------------------------|
| **# 5G/LTE**                      |                                                                |
| Ataya Harmony 5G Core Network Kit | Included within a 28-inch suitcase containing the core router. |
| Askey 5G Sub-6 Indoor Small Cell  | The base station connected to the core network.                |
| MOXA CCG-1500 Gateway             | Installed on the vehicle as the connector to 5G.               |

*表 3. 車輛的建議 5G/LTE 組件。*


## 採購資訊

車輛可透過 Hennes Co. 訂購，包括可客製化的選項以加購額外零件。請注意，由於運送限制，電池不包含在組裝中，應在本地採購。您可以透過他們的 [Robot Kingdom](https://robotkingdom.com.tw/contact/) 網站索取報價。

選配的補充零件，例如光達與 5G 模組，取決於您的特定需求。建議諮詢您當地的代理商以獲得採購協助。
