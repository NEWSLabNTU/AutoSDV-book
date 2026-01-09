<!--
Translation Metadata:
- Source file: overview.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 軟體安裝

本指南提供 AutoSDV 的完整安裝流程。請依序執行以下步驟，以設定功能完整的自動駕駛車輛軟體堆疊。

## 系統需求

請選擇以下其中一個平台：

1. **NVIDIA Jetson AGX Orin 64GB**（建議用於車輛部署）
2. **Ubuntu 22.04 PC 配備 NVIDIA GPU**（用於開發和測試）
3. **Docker 環境**（用於模擬和開發）

### 儲存空間需求

- 最少 256GB SSD（Jetson 建議使用 NVMe）
- 至少 100GB 可用空間供軟體安裝使用

## 安裝方法

選擇最適合您需求的安裝方法：

| 方法              | 最適合                          | 難度   | 客製化程度 |
|-------------------|--------------------------------|--------|-----------|
| **自動安裝**      | 大多數使用者、生產部署          | 簡單   | 有限      |
| **手動安裝**      | 進階使用者、客製化配置          | 進階   | 完整      |
| **Docker 安裝**   | 開發、測試、模擬                | 簡單   | 有限      |

## 安裝流程

<span id="step-1-prepare-operating-system"></span>
### 步驟 1：準備作業系統

#### NVIDIA Jetson AGX Orin

1. 下載並安裝 [NVIDIA SDK Manager](https://developer.nvidia.com/sdk-manager)
2. 使用以下配置燒錄 Jetson：
   - **JetPack SDK 版本：6.0**（必須是 6.0，不是 6.1 或 6.2）
   - 安裝所有 CUDA 和 TensorRT 套件
   - 燒錄至外接 NVMe SSD（不要使用內建 eMMC）

#### Ubuntu 22.04 PC

1. 安裝 Ubuntu 22.04 LTS
2. 安裝 NVIDIA 驅動程式（版本 550 或更高）：
   ```bash
   sudo apt update
   sudo apt install nvidia-driver-550
   ```
3. 使用 deb（網路）安裝程式安裝 [CUDA 12.3](https://developer.nvidia.com/cuda-12-3-2-download-archive)
4. 安裝 [TensorRT 8.6 GA](https://developer.nvidia.com/nvidia-tensorrt-8x-download)

#### Docker

請參閱 [Docker 安裝](./docker.md) 以進行容器化安裝（跳過步驟 1-3）。

---

### 步驟 2：安裝 ZED SDK（如果使用 ZED 相機）

**ZED SDK 必須在繼續之前手動安裝。**

如果您使用 ZED 相機，需要安裝 ZED SDK 和 ZED Link 驅動程式。請參閱 [ZED SDK 安裝指南](./zed-sdk.md) 以取得詳細說明。

> **注意**：這是手動安裝步驟。自動安裝腳本不會安裝 ZED SDK。

---

<span id="step-3-install-autosdv-software"></span>
### 步驟 3：安裝 AutoSDV 軟體

選擇您的安裝方法：

#### 自動安裝（建議）

自動安裝腳本會安裝所有相依套件並配置您的系統。

1. **複製儲存庫：**
   ```bash
   cd ~
   git clone https://github.com/NEWSLabNTU/AutoSDV.git
   cd AutoSDV
   ```

2. **執行自動安裝：**
   ```bash
   ./setup.sh
   ```

此腳本會自動安裝：
- ROS 2 Humble
- Autoware 2025.02
- Blickfeld Scanner Library（用於 Cube1 光達，需接受授權條款）
- Velodyne 驅動程式（透過 rosdep）
- NMEA/串列驅動程式（透過 rosdep）
- 所有其他 ROS 相依套件

腳本會提示您：
- **Autoware Debian 套件**：安裝預先建置的套件或從原始碼建置
- **CycloneDDS 核心緩衝區**：配置系統級網路緩衝區（建議）
- **Blickfeld 授權**：接受 Cube1 光達支援的授權條款

**注意**：此腳本不會安裝 ZED SDK，必須事先手動安裝。

3. **安裝並配置 direnv：**

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

#### 手動環境安裝（進階）

手動安裝 ROS 2、Autoware 和相依套件以獲得完整控制：

**請參閱[手動環境安裝](./manual-environment.md)取得詳細說明**

#### Docker 安裝（替代方案）

在容器化環境中執行 AutoSDV：

**請參閱 [Docker 安裝](./docker.md)取得詳細說明**

---

### 步驟 4：建置與驗證

安裝 AutoSDV 軟體後，建置專案：

```bash
cd ~/AutoSDV
make prepare  # Install ROS dependencies
make build    # Build the project
```

啟動系統以驗證安裝：

```bash
source install/setup.bash
ros2 launch autosdv_launch autosdv.launch.yaml
```

如果成功，您應該會看到系統啟動且沒有嚴重錯誤。

## 疑難排解

### 建置錯誤

如果遇到建置錯誤：

```bash
# Clean and rebuild
./clean_build.sh clean
make build
```

### 缺少相依套件

```bash
# Update rosdep database
rosdep update

# Install missing dependencies
cd ~/AutoSDV
rosdep install --from-paths src --ignore-src -r -y
```

### CUDA/TensorRT 問題

驗證 CUDA 安裝：
```bash
nvcc --version  # Should show 12.3 or compatible
nvidia-smi      # Should show driver 550+
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
