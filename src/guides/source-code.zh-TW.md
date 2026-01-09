<!--
Translation Metadata:
- Source file: source-code.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 原始碼說明

AutoSDV 遵循[超專案](https://en.wikibooks.org/wiki/Git/Submodules_and_Superprojects)慣例。它將數百個套件作為 Git 子模組依功能分類到目錄中。它建構在 Autoware 之上，加上專案特定的額外套件，組成一個大型的 ROS 工作空間。

您可以在此造訪 GitHub 儲存庫：

[https://github.com/NEWSLabNTU/AutoSDV](https://github.com/NEWSLabNTU/AutoSDV)。


| 目錄            | 功能                                               |
|-----------------|---------------------------------------------------|
| `AutoSDV/`      |                                                   |
| `├── book/`     | 本文件的原始文件。                                 |
| `├── data/`     | 包含執行時使用或載入的資料檔案。                    |
| `├── docker/`   | Docker 容器建置腳本。                              |
| `├── scripts/`  | 輔助腳本檔案。包含設定環境的 Ansible 腳本。         |
| `├── src/`      | 原始碼套件。                                       |
| `├── Makefile`  | 包含常用的指令。                                   |
| `└── README.md` | 介紹文件，提供專案的第一印象。                     |


## 原始碼套件類別

| 目錄                    | 功能                                   |
|-------------------------|---------------------------------------|
| `AutoSDV/src/`          | Autoware 原始碼樹的入口。              |
| `├── calibration/`      | 校正工具與公用程式。                   |
| `├── launcher/`         | 包含啟動整個駕駛系統的啟動檔案。       |
| `├── localization/`     | 定位套件（Isaac Visual SLAM 等）。     |
| `├── param/`            | 特定於車輛型號的參數。                 |
| `├── sensor_component/` | 感測器驅動程式與感測資料處理器。       |
| `├── sensor_kit/`       | 感測器相關參數與啟動檔案。             |
| `├── system/`           | 系統執行時與監控組件。                 |
| `└── vehicle/`          | 動力系統控制與運動學參數。             |


## 車輛介面套件

`AutoSDV/src/vehicle/autosdv_vehicle_launch/`

| 目錄                               | 功能                           |
|------------------------------------|-------------------------------|
| `.../autosdv_vehicle_launch/`      |                               |
| `├── autosdv_vehicle_interface/`   | 動力系統控制及其狀態測量。     |
| `├── autosdv_vehicle_description/` | 車輛形狀參數與網格檔案。       |
| `└── autosdv_vehicle_launch/`      | 啟動車輛介面的啟動檔案。       |


## 車輛特定參數的套件

`autoware_individual_params` 套件提供特定於不同車輛型號的參數。它位於

```
AutoSDV/src/param/autoware_individual_params
```

您可以在此套件中找到參數目錄。

```
.../autoware_individual_params/individual_params/default/
```

| 目錄                                  | 功能                         |
|---------------------------------------|------------------------------|
| `.../default/`                        |                              |
| `├── awsim_sensor_kit`                | AWSIM 車輛的參數。           |
| `└── autosdv_sensor_kit`              | AutoSDV 車輛的參數。         |
| `    ├── imu_corrector.param.yaml`    |                              |
| `    ├── sensor_kit_calibration.yaml` |                              |
| `    └── sensors_calibration.yaml`    |                              |


## 感測器相關套件

| 目錄                                          | 功能                       |
|-----------------------------------------------|----------------------------|
| `AutoSDV/src/`                                |                            |
| `├── sensor_component/`                       | 感測器驅動程式與預處理器。 |
| `└── sensor_kit/`                             |                            |
| `    └── autosdv_sensor_kit_launch/`          |                            |
| `        ├── autosdv_sensor_kit_description/` | 各感測器的座標。           |
| `        └── autosdv_sensor_kit_launch/`      | 感測器的額外啟動檔案。     |
