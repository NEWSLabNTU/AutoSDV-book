<!--
Translation Metadata:
- Source file: platform-models.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 平台型號

| 特性                 | 基礎型     | 360° 光達    | 固態型             | 聯網型             |
|----------------------|-----------|------------|--------------------|--------------------|
| **光達型號**         | -         | VLP-32C    | Robin-W            | Robin-W            |
| **光達視野**         | -         | 360° × 40° | 120° × 25°         | 120° × 25°         |
| **光達範圍**         | -         | 100m       | 150m               | 150m               |
| **定位**             | 純視覺    | NDT 就緒   | 開發中             | 開發中             |
| **遠端操作**         | 手動      | 否         | 否                 | 是（5G）           |
| **電池續航力**       | 40 分鐘   | 40 分鐘    | 40 分鐘            | 40 分鐘            |

<table align="center" border="0" style="margin: 2em 0;">
  <tr>
    <td align="center" valign="bottom">
      <a href="../../figures/model_robin-w.webp" target="_blank">
        <img src="../../figures/model_robin-w.webp" alt="Robin-W 固態光達套件" width="80%"/>
      </a>
    </td>
    <td align="center" valign="bottom">
      <a href="../../figures/model_velodyne_32c.webp" target="_blank">
        <img src="../../figures/model_velodyne_32c.webp" alt="Velodyne 32C 光達套件" width="80%"/>
      </a>
    </td>
    <td align="center" valign="bottom">
      <a href="../../figures/model_cube1_moxa-5g.webp" target="_blank">
        <img src="../../figures/model_cube1_moxa-5g.webp" alt="Cube1 光達 + MOXA 5G 套件" width="80%"/>
      </a>
    </td>
  </tr>
  <tr>
    <td align="center">
      <b>Robin-W 固態光達套件</b>
    </td>
    <td align="center">
      <b>Velodyne 32C 光達套件</b>
    </td>
    <td align="center">
      <b>Cube1 光達 + MOXA 5G 套件</b>
    </td>
  </tr>
</table>

- [基礎型號](#base-model) - 可客製化的核心平台
- [360° 光達型號](#360-lidar-model) - 配備 Velodyne VLP-32C 的生產就緒型號
- [固態型號](#solid-state-model) - 配備 Seyond Robin-W 的現代化光達
- [聯網型號](#connected-model) - 配備固態光達和 5G 連線功能

---

<span id="base-model"></span>
## 基礎型號

<figure style="width: 50%; text-align: center; margin: auto;">
	<img src="../../figures/vehicle_side_1.png" alt="Base Model">
	<figcaption>AutoSDV 基礎平台可供客製化</figcaption>
</figure>

具備運算、視覺和控制能力的 AutoSDV 核心平台。適合供應商整合自家感測器。

### 核心元件

| 元件                | 規格                         |
|---------------------|------------------------------|
| **底盤**            | Tekno TKR9500 16×11×5" 卡車  |
| **電腦**            | NVIDIA Jetson AGX Orin 64GB  |
| **儲存裝置**        | 1TB NVMe SSD                 |
| **相機**            | ZED X Mini 立體相機          |
| **ZED Link**        | ZED 擷取卡                   |
| **IMU**             | MPU9250 9 軸                 |
| **霍爾感測器**      | KY-003 輪速編碼器            |
| **上層電池**        | 22.2V 6S 5000mAh LiPo        |
| **下層電池**        | 7.4V 2S 7200mAh LiPo         |
| **馬達**            | 無刷 4274/1500kv             |
| **ESC**             | 60A 無刷控制器               |
| **伺服馬達**        | 高扭力數位型                 |
| **PWM 驅動器**      | PCA9685 16 通道              |
| **DC-DC 轉換器**    | 24V→12V 10A                  |
| **斷路器**          | 30A 安全開關                 |

### 可用介面

- **Ethernet**：10GbE 埠，用於高速光達連接
- **USB**：4x USB 3.2 Type-A、2x USB Type-C（USB 3.2）
- **GPIO**：40 針腳排針，支援 I2C、SPI、UART、PWM
- **PCIe**：M.2 Key M 和 Key E 插槽供擴充使用
- **顯示器**：DisplayPort

---

<span id="360-lidar-model"></span>
## 360° 光達型號

<figure style="width: 50%; text-align: center; margin: auto;">
	<img src="../../figures/model_velodyne_32c.webp" alt="360° LiDAR Model">
	<figcaption>配備 Velodyne VLP-32C 360° 光達的 AutoSDV</figcaption>
</figure>

生產就緒配置，已驗證可與 Autoware 整合。支援完整的 NDT 定位功能。

### 額外元件

| 元件            | 規格                 |
|-----------------|---------------------|
| **光達**        | Velodyne VLP-32C    |
| **光達支架**    | 頂部中央支架        |

### 光達規格
- **視野範圍**：360° × 40°
- **範圍**：100m
- **點數/秒**：600,000
- **通道數**：32
- **連接方式**：Ethernet


---

<span id="solid-state-model"></span>
## 固態型號

<figure style="width: 50%; text-align: center; margin: auto;">
	<img src="../../figures/model_robin-w.webp" alt="Solid-State Model">
	<figcaption>配備 Seyond Robin-W 固態光達的 AutoSDV</figcaption>
</figure>

採用現代化固態光達平台，具備高點雲密度。定位功能尚在開發中。

### 額外元件

| 元件            | 規格                 |
|-----------------|---------------------|
| **光達**        | Seyond Robin-W      |
| **光達支架**    | 前向支架            |


### 光達規格

- **視野範圍**：120° × 25°
- **範圍**：150m
- **點數/秒**：750,000
- **技術**：固態（無可動零件）
- **連接方式**：Ethernet


---

<span id="connected-model"></span>
## 聯網型號

<figure style="width: 50%; text-align: center; margin: auto;">
	<img src="../../figures/model_cube1_moxa-5g.webp" alt="Connected Model">
	<figcaption>配備固態光達和 5G 閘道器的 AutoSDV</figcaption>
</figure>

支援遠端操作和車隊管理的聯網平台。注意：5G 模組佔用頂部安裝位置。

### 額外元件

| 元件            | 規格                 |
|-----------------|---------------------|
| **光達**        | Seyond Robin-W      |
| **5G 模組**     | MOXA OnCell G4302   |
| **光達支架**    | 前向支架            |
| **5G 支架**     | 頂部中央支架        |

### 光達規格
- **視野範圍**：120° × 25°
- **範圍**：150m
- **點數/秒**：750,000
- **技術**：固態（無可動零件）
- **連接方式**：Ethernet

### 5G 規格

*注意：MOXA OnCell G4302 僅作為 5G 閘道器範例展示。實際的 5G 模組選擇由供應商決定，可根據部署需求進行客製化。*

- **網路**：5G/LTE 並具備降級功能
- **SIM 卡槽**：雙 SIM 卡支援電信商備援
- **功能**：遠端操作、遙測串流、OTA 更新

### 系統規格
- **重量**：11.3 kg（因 5G 套件增加 33% 重量）
- **電池續航力**：40 分鐘（上層）、2+ 小時（下層）
- **連線需求**：需要 5G/LTE 數據方案

## 下一步

- [硬體組裝指南](./getting-started/hardware-assembly.md) - 建置說明
- [軟體安裝](./getting-started/installation/overview.md) - 軟體設定
