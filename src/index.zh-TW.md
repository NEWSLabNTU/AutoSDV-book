<!--
Translation Metadata:
- Source file: index.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

<figure style="text-align: center">
	<img src="../figures/logo/logo_brand_gray.png" alt="The AutoSDV Project logo">
</figure>

# AutoSDV 文件

歡迎來到 AutoSDV（自動駕駛軟體定義車輛）文件。

AutoSDV 專案，即 *Autoware 軟體定義車輛*，提供了一個經濟實惠的自動駕駛平台，配備實用的車輛設備，專為教育和研究機構設計。本專案讓您能夠在家中建置自動駕駛平台，並在真實的戶外道路環境中使用。採用領先的開源自動駕駛軟體專案 Autoware 驅動，為車輛軟體提供極大的靈活性和可擴展性。

AutoSDV 提供從硬體規格到軟體實作的完整堆疊，使用業界標準工具和實踐，為真實世界的自動駕駛系統提供易於接觸的切入點。

<table align="center" border="0">
  <tr>
    <td align="center" valign="bottom">
      <a href="../figures/model_robin-w.webp" target="_blank">
        <img src="../figures/model_robin-w.webp" alt="Robin-W Solid-State LiDAR Kit" width="80%"/>
      </a>
    </td>
    <td align="center" valign="bottom">
      <a href="../figures/model_velodyne_32c.webp" target="_blank">
        <img src="../figures/model_velodyne_32c.webp" alt="Velodyne 32C LiDAR Kit" width="80%"/>
      </a>
    </td>
    <td align="center" valign="bottom">
      <a href="../figures/model_cube1_moxa-5g.webp" target="_blank">
        <img src="../figures/model_cube1_moxa-5g.webp" alt="Blickfeld Cube1 + MOXA 5G Kit" width="80%"/>
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

## 主要特性

- **ROS 2 Humble** - 基於最新的長期支援 ROS 2 發行版
- **Autoware 整合** - 完整的自動駕駛軟體堆疊
- **彈性的感測器支援** - 支援多種光達、相機、IMU 和 GNSS 配置
- **多模式控制** - 具備 PID 和回饋機制的精密車輛控制
- **生產就緒** - 已在實際硬體平台上測試

## 開始使用

本指南將引導您設定和使用 AutoSDV 平台。請依序遵循以下步驟來建置功能完整的自動駕駛車輛：

1. **[硬體設定](getting-started/hardware-assembly.md)** - 設定和使用車輛硬體
2. **[軟體安裝](getting-started/installation/overview.md)** - 安裝 AutoSDV 軟體
   - **[ZED SDK 安裝](getting-started/installation/zed-sdk.md)** - ZED 相機驅動程式設定
   - **[自動安裝](getting-started/installation/overview.md)** - 自動化安裝（建議）
   - **[手動安裝](getting-started/installation/manual-environment.md)** - 進階客製化
   - **[Docker 安裝](getting-started/installation/docker.md)** - 容器化安裝
3. **[操作車輛](getting-started/usage.md)** - 啟動和控制系統

### 快速入門路徑

**車輛部署**：依序遵循步驟：硬體設定 → 安裝 → 操作

**模擬/開發**：跳至[軟體安裝](getting-started/installation/overview.md)或使用 [Docker 安裝](getting-started/installation/docker.md)進行快速測試

**客製化**：參閱[手動安裝](getting-started/installation/manual-environment.md)取得進階配置選項

## 快速連結

- [**平台型號**](platform-models.md) - 探索不同的硬體配置
- [**指南**](guides/development.md) - 開發者和操作者教學
- [**技術參考**](reference/overview.md) - 詳細技術規格

## 引用

如果您在研究或教育專案中使用 AutoSDV，請使用以下 BibTeX 條目引用我們的工作：

```latex
@misc{autosdv2025,
  author = {Hsiang-Jui Lin, Chi-Sheng Shih},
  title = {AutoSDV: A Software-Defined Vehicle Platform for Research and Education},
  year = {2025},
  institution = {National Taiwan University},
  url = {https://github.com/NEWSLabNTU/AutoSDV},
  note = {Accessed: 2025-04-28}
}
```

## 取得協助

- **文件**：您正在閱讀！
- **問題回報**：[GitHub Issues](https://github.com/NEWSLabNTU/AutoSDV-book/issues)
- **原始碼**：[GitHub Repository](https://github.com/NEWSLabNTU/AutoSDV-book)

---

*本文件由 AutoSDV 專案團隊維護。*
