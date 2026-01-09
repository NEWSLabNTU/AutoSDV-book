<!--
Translation Metadata:
- Source file: 5g-deployment.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 5G/LTE 部署

## 戶外設定範例


圖 1 顯示基於 Ataya Harmony 系統的私有 5G 基礎設施戶外設定範例。該系統有幾個部分：

1. Ataya 5G 核心網路機箱（圖 1 中的「1」）
2. Askey 小型基地台（圖 1 中的「2」）
3. 安裝在車頂的 MOXA 5G 行動閘道器（圖 2）

<figure style="width: 80%; text-align: center; margin: auto;">
	<img src="../../../../figures/5G_setup_example.webp" alt="5G core network and antenna setup example">
	<figcaption>圖 1. 5G 核心網路與天線設定範例。</figcaption>
</figure>

<figure style="width: 40%; text-align: center; margin: auto;">
	<img src="../../../../figures/vehicle_top_mount.jpg" alt="5G cellular gateway installation">
	<figcaption>圖 2. 5G 行動閘道器安裝範例。</figcaption>
</figure>

5G 訊號範圍可延伸至 600 公尺。與 Wi-Fi 相比，它在該區域內保持穩定的延遲與頻寬，無論與基地台的距離如何。

設定 5G 設備時，有幾點需要考慮。首先，車輛中的 5G 接收器必須放置在車體外部而非內部，以避免接收不良。其次，基地台附近的建築物會影響訊號範圍。基地台天線通常具有方向性功能，因此請確保車輛的活動範圍在天線的覆蓋區域內。

## 網路架構

<figure style="width: 100%; text-align: center; margin: auto;">
	<img src="../../../../figures/5G_network_architecture.png" alt="5G network architecture">
	<figcaption>圖 3. Ataya 5G 網路架構（範例）</figcaption>
</figure>
