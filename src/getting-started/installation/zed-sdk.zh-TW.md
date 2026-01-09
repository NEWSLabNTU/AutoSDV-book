<!--
Translation Metadata:
- Source file: zed-sdk.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# ZED SDK 安裝

ZED SDK 提供 ZED 立體相機和 ZED Link 擷取卡的驅動程式和 API。這是執行 AutoSDV 安裝腳本前**必須手動完成的安裝步驟**。

## 何時需要安裝？

如果您使用以下設備，請安裝 ZED SDK：
- ZED X Mini 相機（標準 AutoSDV 配置）
- ZED 2/2i 相機
- ZED Link 擷取卡（Mono/Dual/Quad）

如果您僅使用光達感測器（Velodyne、Blickfeld、Robin-W），可以跳過此步驟。

## 先決條件

- Ubuntu 22.04 已安裝 NVIDIA 驅動程式（550+），或
- Jetson AGX Orin 配備 JetPack 6.0

## 安裝步驟

### 步驟 1：下載 ZED SDK 5.1

前往 [ZED SDK Downloads](https://www.stereolabs.com/developers/release) 頁面。

下載適合的安裝程式：

**Jetson AGX Orin（JetPack 6.0）：**
- [ZED SDK 5.1 for JetPack 6.0](https://download.stereolabs.com/zedsdk/5.1/l4t36.3/jetsons)

**Ubuntu 22.04 PC：**
- [ZED SDK 5.1 for Ubuntu 22 + CUDA 12](https://download.stereolabs.com/zedsdk/5.1/cu124/ubuntu22)

### 步驟 2：安裝 ZED SDK

```bash
# Make the installer executable
chmod +x ZED_SDK_*.run

# Run the installer
sudo ./ZED_SDK_*.run
```

按照螢幕提示操作：
- 接受授權協議
- 安裝所有元件（SDK、工具、Python API、範例）
- 允許下載 AI 模型（物件偵測所需）

安裝需要 10-20 分鐘，視網路速度而定。

### 步驟 3：驗證安裝

```bash
# Run ZED diagnostic tool
/usr/local/zed/tools/ZED_Diagnostic

# Expected output should show:
# - ZED SDK Version: 5.1.x
# - CUDA version detected
# - Camera detection status (if connected)
```

## ZED Link 驅動程式安裝（選用）

如果您使用 ZED Link 擷取卡進行多相機設定，請安裝適當的驅動程式：

### 識別您的 ZED Link 型號

- **ZED Link Mono**：單相機輸入
- **ZED Link Dual**：雙相機輸入
- **ZED Link Quad**：四相機輸入

### 下載並安裝

1. 前往 [Stereolabs Download Center](https://www.stereolabs.com/developers/release)
2. 下載適合您型號和 Ubuntu 版本的 ZED Link 驅動程式
3. 安裝 debian 套件：

```bash
# For ZED Link Mono
sudo dpkg -i zed-link-mono_*.deb

# For ZED Link Dual
sudo dpkg -i zed-link-dual_*.deb

# For ZED Link Quad
sudo dpkg -i zed-link-quad_*.deb
```

4. 驗證安裝：

```bash
# Check if ZED Link is detected
lspci | grep -i stereolabs

# Should show PCIe device for your ZED Link model
```

## 疑難排解

### 找不到 CUDA

如果安裝程式無法找到 CUDA：

```bash
# Verify CUDA installation
nvcc --version
nvidia-smi

# CUDA should be 12.3 or compatible
# If not installed, return to Step 1 of the main installation guide
```

### Python 相依套件

安裝程式可能會安裝 Python 套件。為避免衝突：

```bash
# After installation, verify numpy location
python3 -c "import numpy; print(numpy.__file__)"

# Should be in /home/user/.local or /usr/lib
# NOT in /usr/local (which can cause conflicts)
```

### 未偵測到相機

如果安裝後未偵測到 ZED 相機：

```bash
# Check USB connection
lsusb | grep -i stereo

# Add user to video group
sudo usermod -aG video $USER

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Log out and back in for group changes to take effect
```

### 權限錯誤

```bash
# Fix ZED SDK directory permissions
sudo chown -R root:root /usr/local/zed
sudo chmod -R 755 /usr/local/zed

# Fix calibration directory permissions
sudo chmod 777 /usr/local/zed/settings
```

## 下一步

成功安裝 ZED SDK 後：

- 返回[軟體安裝總覽](./overview.md#step-3-install-autosdv-software)
- 繼續進行自動或手動安裝
- 繼續建置並驗證 AutoSDV
