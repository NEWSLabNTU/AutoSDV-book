<!--
Translation Metadata:
- Source file: recommended.md
- Last synced: 2026-03-29
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 推薦安裝方式

本指南說明如何使用自動安裝腳本安裝 AutoSDV。請先確認已[準備好作業系統](./overview.md#prepare-operating-system)。

## 複製儲存庫

```bash
cd ~
git clone -b develop --recursive https://github.com/NEWSLabNTU/AutoSDV.git
cd AutoSDV
```

## 執行安裝腳本

```bash
./setup.sh
```

腳本會先詢問是否安裝所有可選元件。若您選擇不安裝，會個別提示：

- **Autoware Debian 套件**：安裝預先建置的套件（約 2-3 GB）
- **Isaac ROS Visual Localization**：純相機定位（需要 NVIDIA GPU）
- **Blickfeld Scanner Library**：接受 Cube1 光達的授權條款

以下項目會固定安裝：

- ROS 2 Humble
- ROS 2 開發工具（colcon、rosdep、vcstool）
- GeographicLib 資料集
- 開發工具（git-lfs、golang、pre-commit、plotjuggler）
- Python 相依套件（Adafruit-PCA9685、simple-pid、play_launch）
- u-blox GPS udev 規則
- 所有 ROS 相依套件（透過 rosdep）

設定完成後，會建議可選的後續步驟：

- **CycloneDDS 核心緩衝區**：`./setup.sh cyclonedds-sysctl`
- **TurboVNC + VirtualGL**：`./setup.sh turbovnc-virtualgl`

> **注意**：此腳本不會安裝 ZED SDK。如果您使用 ZED 相機，請事先手動安裝。請參閱 [ZED SDK 安裝](./zed-sdk.md)。

## 安裝並配置 direnv

AutoSDV 使用 direnv 進行自動環境啟動：

```bash
# Install direnv
sudo apt install direnv

# Add to your shell (bash)
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
source ~/.bashrc

# Allow .envrc in AutoSDV directory
cd ~/AutoSDV
direnv allow
```

完成後，當您進入 AutoSDV 目錄時，ROS 2 和 Autoware 環境會自動啟動。

## 建置與驗證

建置專案：

```bash
cd ~/AutoSDV
just build
```

啟動系統以驗證安裝：

```bash
just launch
```

如果成功，您應該會看到系統啟動且沒有嚴重錯誤。

## 疑難排解

### 建置錯誤

如果遇到建置錯誤：

```bash
# 清除並重新建置
just clean
just build
```

### 缺少相依套件

```bash
# 更新 rosdep 資料庫
rosdep update

# 安裝缺少的相依套件
cd ~/AutoSDV
rosdep install --from-paths src --ignore-src -r -y
```

### CUDA/TensorRT 問題

驗證 CUDA 安裝：
```bash
nvcc --version  # 應顯示 12.3 或相容版本
nvidia-smi      # 應顯示驅動程式 550+
```

### ROS 2 環境問題

建置前務必先載入 ROS 2 環境：
```bash
source /opt/ros/humble/setup.bash
```

## 下一步

成功安裝後：

- [操作車輛](../usage.md) - 學習如何執行 AutoSDV
- [開發指南](../../guides/development.md) - 開始使用 AutoSDV 進行開發

## 取得協助

如果您遇到此處未涵蓋的問題：

- 檢查 [AutoSDV GitHub Issues](https://github.com/NEWSLabNTU/AutoSDV/issues)
- 查看各安裝方法頁面中的疑難排解章節
