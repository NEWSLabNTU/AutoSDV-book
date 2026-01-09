<!--
Translation Metadata:
- Source file: hardware-assembly.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 建置車輛

建議的車輛組建基於 16×11×5 英吋底盤，加上額外的感測器和通訊安裝架，可分為以下幾個部分。

**核心元件**包含執行 Autoware 所需的必要元件。

- 車載電腦
- 導航系統
- 電源供應系統
- 動力系統
- 底盤

車輛可根據您的選擇配備**額外安裝架**。

- 光達感測器
- 5G/LTE 通訊模組


## 核心元件

車輛由圖 1 所示的三個主要層次組成，由上至下為：

- <span style="color: Yellow">黃色</span>：車載電腦、導航感測器和額外安裝架。
- <span style="color: Red">紅色</span>：為黃色層的車載電腦和感測器供電的電源供應系統。
- <span style="color: DodgerBlue">藍色</span>：動力系統和動力系統的電源供應。

使用本車輛時，額外的感測器和 5G 安裝架安裝於<span style="color: Yellow">黃色</span>層，其電源來自<span style="color: Red">紅色</span>層。由於電壓需求不同，馬達在<span style="color: DodgerBlue">藍色</span>層有獨立的電池和電源供應。

<figure style="width: 80%; text-align: center;">
	<img src="../../../figures/vehicle_parts.jpg" alt="Vehicle parts.">
	<figcaption>圖 1. 車輛元件由上至下：車載電腦、電源供應和動力系統。</figcaption>
</figure>

### 電源供應系統

#### 電池

兩個電源供應分別使用兩顆電池，即*上層電源*和*下層電源*。電池如圖 2 所示。上層電源使用 22.2V 6S 電池 (1) 為車載電腦和感測器供電，下層電源使用 7.4V 2S 電池 (2) 為 DC 馬達和動力系統供電。

兩顆電池都配有黃色 XT60 電源插頭和白色 JST-XH 連接器，如圖 3 所示。JST-XH 連接器插入圖 4 所示的電壓監視器。當電壓過低時會發出嗶嗶聲。

<figure style="width: 80%; text-align: center; margin: auto;">
	<img src="../../../figures/lipo_batteries.webp" alt="Lithium polymer battery examples.">
	<figcaption>圖 2. 鋰聚合物電池範例。</figcaption>
</figure>

<figure style="width: 50%; text-align: center; margin: auto;">
	<img src="../../../figures/battery_connectors.webp" alt="Battery connectors.">
	<figcaption>圖 3. 電池 XT60 和 JST-XH 連接器。</figcaption>
</figure>

<figure style="width: 40%; text-align: center; margin: auto;">
	<img src="../../../figures/voltage_monitor.webp" alt="Battery voltage monitor.">
	<figcaption>圖 4. 電池電壓監視器。</figcaption>
</figure>

#### 上層電源供應

上層電源啟動程序如圖 4 所示。首先，將電池安裝在電池座上。其次，將電池連接至纜線。最後，開啟圖 5 所示的電源開關。

請注意，在安裝或拆卸電池前，必須關閉電源開關。這是保護系統免受電壓突波影響的必要措施。

<figure style="width: 80%; text-align: center; margin: auto;">
	<img src="../../../figures/battery_installation_steps.jpg" alt="Upper power start up procedure.">
	<figcaption>圖 4. 上層電源啟動程序。</figcaption>
</figure>


<figure style="width: 80%; text-align: center; margin: auto;">
	<img src="../../../figures/upper_power_switch_with_arrow.jpg" alt="Turn on power switch">
	<figcaption>圖 5. 開啟上層電源開關。</figcaption>
</figure>

#### 下層電源供應

下層電源啟動程序如圖 6 所示。電池安裝在車輛底層的電池座中 (1)。然後，開啟電源 (2)。

<figure style="width: 60%; text-align: center; margin: auto;">
	<img src="../../../figures/lower_power_installation.webp" alt="Lower power start up procedure">
	<figcaption>圖 6. 下層電源啟動程序</figcaption>
</figure>


## 安裝座

車輛有三個安裝座可供安裝您喜愛的感測器。下圖顯示兩種組建方式，圖上標示了三個安裝座：(1) 前方安裝座、(2) 頂部安裝座和 (3) 後方安裝座。

<table align="center" border="0">
  <tbody>
    <tr>
      <td align="center" valign="bottom">
        <img src="../../../figures/sensor_mounts-1.webp" alt="Sensor mounts example 1" width="80%"/>
      </td>
      <td align="center" valign="bottom">
        <img src="../../../figures/sensor_mounts-2.webp" alt="Sensor mounts example 2" width="80%"/>
      </td>
    </tr>
  </tbody>
</table>

下表說明兩種組建方式的詳細資訊。

<table align="center" border="0">
  <thead>
    <tr>
      <th align="center">編號</th>
      <th align="center">前方安裝座</th>
      <th align="center">頂部安裝座</th>
      <th align="center">後方安裝座</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="center" valign="middle" rowspan="2">1</td>
      <td align="center" valign="bottom">
        <img src="../../../figures/front_dock-1.webp" alt="Front dock example 1" width="80%"/>
      </td>
      <td align="center" valign="bottom">
        <img src="../../../figures/top_dock-1.webp" alt="Top dock example 1" width="80%"/>
      </td>
      <td align="center" valign="bottom">
        <img src="../../../figures/rear_dock-1.webp" alt="Rear dock example 1" width="80%"/>
      </td>
    </tr>
    <tr>
      <td align="center">Seyond Robin-W 光達</td>
      <td align="center">MOXA 5G 模組</td>
      <td align="center">光達 Ethernet 轉接器</td>
    </tr>
    <tr>
      <td align="center" valign="middle" rowspan="2">2</td>
      <td align="center" valign="bottom">
        <img src="../../../figures/front_dock-2.webp" alt="Front dock example 2" width="80%"/>
      </td>
      <td align="center" valign="bottom">
        <img src="../../../figures/top_dock-2.webp" alt="Top dock example 2" width="80%"/>
      </td>
      <td align="center" valign="bottom">
        <img src="../../../figures/rear_dock-2.webp" alt="Rear dock example 2" width="80%"/>
      </td>
    </tr>
    <tr>
      <td align="center">Velodyne 光達轉接器</td>
      <td align="center">Velodyne 32C 光達</td>
      <td align="center">導航感測器套件</td>
    </tr>
  </tbody>
</table>

## 元件與配線

車輛包含底盤、車身、車載電腦等必要元件，以及額外的光達和 5G 通訊模組。有關這些元件及其配線的詳細資訊，請參閱[配線圖](../reference/hardware/wiring-diagrams.md)中的完整指南。
