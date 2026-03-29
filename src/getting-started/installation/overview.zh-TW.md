<!--
Translation Metadata:
- Source file: overview.md
- Last synced: 2026-03-29
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

| 方法                                                  | 最適合                          | 難度   | 客製化程度 |
|-------------------------------------------------------|--------------------------------|--------|-----------|
| **[推薦安裝方式](./recommended.md)**                   | 大多數使用者、生產部署          | 簡單   | 有限      |
| **[手動環境安裝](./manual-environment.md)**            | 進階使用者、客製化配置          | 進階   | 完整      |
| **[Docker 安裝](./docker.md)**                        | 開發、測試、模擬                | 簡單   | 有限      |

## 準備作業系統

安裝 AutoSDV 之前，請先準備您的目標平台。

<span id="step-1-prepare-operating-system"></span>

### NVIDIA Jetson AGX Orin

1. 下載並安裝 [NVIDIA SDK Manager](https://developer.nvidia.com/sdk-manager)
2. 使用以下配置燒錄 Jetson：
   - **JetPack SDK 版本：6.2.1**
   - 安裝所有 CUDA 和 TensorRT 套件
   - 燒錄至外接 NVMe SSD（不要使用內建 eMMC）

### Ubuntu 22.04 PC

1. 安裝 Ubuntu 22.04 LTS
2. 安裝 NVIDIA 驅動程式（版本 550 或更高）：
   ```bash
   sudo apt update
   sudo apt install nvidia-driver-550
   ```
3. 使用 deb（網路）安裝程式安裝 [CUDA 12.3](https://developer.nvidia.com/cuda-12-3-2-download-archive)
4. 安裝 [TensorRT 8.6 GA](https://developer.nvidia.com/nvidia-tensorrt-8x-download)

### Docker

請參閱 [Docker 安裝](./docker.md) 以進行容器化安裝。這會跳過上述的作業系統準備步驟。

## 安裝 ZED SDK（如果使用 ZED 相機）

**ZED SDK 必須在繼續之前手動安裝。**

如果您使用 ZED 相機，需要安裝 ZED SDK 和 ZED Link 驅動程式。請參閱 [ZED SDK 安裝指南](./zed-sdk.md) 以取得詳細說明。

> **注意**：這是手動安裝步驟。自動安裝腳本不會安裝 ZED SDK。

## 下一步

作業系統準備完成後，請前往[推薦安裝方式](./recommended.md)指南。
