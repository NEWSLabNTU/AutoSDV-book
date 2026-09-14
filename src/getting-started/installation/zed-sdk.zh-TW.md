<!--
Translation Metadata:
- Source file: zed-sdk.md
- Last synced: 2026-09-14
- Translator: Claude (Anthropic)
- Status: Complete
-->

# ZED SDK 安裝

ZED SDK 提供 ZED 立體相機與 ZED Link 擷取卡的驅動與 API。**這一步由你手動安裝。**
Stereolabs 並未提供 apt 套件庫，唯一的官方安裝檔是自解壓安裝程式，且會要求你接受
專有授權；因此 `setup.sh` 只檢查它是否存在、並告訴你該下載什麼，而不會代你回答
授權問題。

## 什麼時候需要它？

使用以下裝置時需要安裝 ZED SDK：

- ZED X Mini 相機（AutoSDV 的標準配置）
- ZED 2/2i 相機
- ZED Link 擷取卡（Mono/Dual/Quad）

只用光達（Velodyne、Blickfeld、Robin-W）時可以跳過。缺少它不會影響其他部分：
`zed_components` 會回報自行跳過，工作空間其餘部分照常建置。

## 版本不是偏好問題

| 元件 | 版本 |
|------|------|
| ZED SDK | 5.4.1 |
| ZED ROS 2 wrapper | 5.4.1（我們的 `ntust-workshop` 分支，rebase 到 `v5.4.1`） |

`zed_components` 是對著 SDK 自己的標頭檔編譯的，所以 SDK 與 wrapper 版本不一致
是建置或執行失敗，而不是功能降級。這一對版本記錄在 `versions.yaml` 的 `zed:`
之下；要升就兩個一起升。

## 先備條件

- Ubuntu 22.04，已安裝 NVIDIA 驅動與 CUDA 12（Autoware 鎖定的版本），或
- 達到本專案下限的 Jetson：**JetPack 6.2.2 或更新**，也就是 L4T 36.5

## 步驟 1 — 先問 setup.sh 這台機器需要什麼

```bash
./setup.sh --run --only zed-sdk --yes
```

若已安裝，它會印出版本；否則印出適合這台機器的下載連結。任何讓這個步驟仍未滿足的
安裝流程，結束時都會以醒目顏色再提示一次。

## 步驟 2 — 下載

| 機器 | 安裝檔 |
|------|--------|
| amd64、Ubuntu 22.04、CUDA 12 | <https://download.stereolabs.com/zedsdk/5.4/cu12/ubuntu22> |
| Jetson、L4T 36.5——JetPack 6.2.2 以上，本專案所訂的版本 | <https://download.stereolabs.com/zedsdk/5.4/l4t36.5/jetsons> |
| Jetson、L4T 36.4——JetPack 6.2 或 6.2.1，低於下限 | <https://download.stereolabs.com/zedsdk/5.4/l4t36.4/jetsons> |

這些是 Stereolabs 維持穩定的轉址連結，各自會轉到檔名帶有修訂版號的 CDN 檔案
（amd64 目前是 `ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v5.4.1.zstd.run`）。請
收藏轉址連結，不要收藏它轉到的檔案。

選擇前先確認 Jetson 上的 L4T 版本——看盒子上的 JetPack 版本不夠，因為 6.2.1 是
L4T 36.4，而 6.2.2 是 L4T 36.5：

```bash
head -1 /etc/nv_tegra_release     # "# R36 (release), REVISION: 5.0" -> L4T 36.5
```

## 步驟 3 — 安裝

```bash
curl -fsSL -o zed_sdk.run 'https://download.stereolabs.com/zedsdk/5.4/cu12/ubuntu22'
chmod +x zed_sdk.run
./zed_sdk.run
```

依提示回答：接受授權、安裝工具與 Python API；若要使用物件偵測，允許下載 AI 模型。
約需 10–20 分鐘，大部分時間在下載。

## 步驟 4 — 確認

```bash
./setup.sh --rerun zed-sdk     # "ZED SDK 5.4.1 is installed at /usr/local/zed."
just build                     # 這時才會建置 zed_components
```

`./setup.sh --status` 讀的是 SDK 自己的 cmake 版本檔——與建置時 `find_package(ZED)`
讀的是同一個檔案——所以版本不符會被回報為未安裝，這是誠實的答案。

## 在 amd64 上，這個安裝檔同時帶來 TensorRT 10.9

amd64 版安裝檔附帶 TensorRT 10.9，而 Autoware 的感知引擎需要**恰好** 10.8：快取的
引擎會記錄建置它的 TensorRT 版本，Autoware 會丟棄版本不同的引擎，於是每次啟動都
重建全部五個模型。

兩者可以並存，AutoSDV 已經替你安排好：`tensorrt-runtime` 步驟把 Autoware 的
TensorRT 裝在獨立前綴下，`scripts/env.sh` 只為 AutoSDV 的程序把它排在系統版本
前面。這也正是該步驟不去降級系統函式庫的原因——那樣會弄壞 ZED SDK。

```bash
./setup.sh --run --only tensorrt-runtime --yes
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
```

CUDA 12 是 Autoware 鎖定的版本（`versions.yaml` 的 `nvidia_amd64.cuda`），請對齊
它，而不是對齊某個修訂版號。Jetson 上它來自 JetPack；工作站上來自主機映像或
NVIDIA 的 apt 套件庫——`setup.sh` 刻意不安裝 CUDA 工具鏈，因為那會改動
`/usr/local/cuda`，影響這台機器上的每一個使用者。

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

- 繼續進行[推薦安裝方式](./recommended.md)
- 繼續進行自動或手動安裝
- 繼續建置並驗證 AutoSDV
