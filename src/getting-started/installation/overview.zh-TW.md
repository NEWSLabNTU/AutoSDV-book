<!--
Translation Metadata:
- Source file: overview.md
- Last synced: 2026-03-29
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 軟體安裝

本指南提供 AutoSDV 的完整安裝流程。請依序執行以下步驟，以設定功能完整的自動駕駛車輛軟體堆疊。

## 這套安裝方式與 Autoware 官方的差異

如果你讀過 Autoware 的官方文件，你預期的會是：複製一個工作空間、對數十個儲存庫
執行 `vcs import`，再花上一小時以上的 `colcon build`。

**AutoSDV 不是這樣做的。** Autoware 以 **Debian 套件**的形式安裝到
`/opt/autoware/1.5.0`。你的磁碟上沒有 Autoware 的原始碼樹，機器上也不會編譯任何
Autoware 的程式碼——在 Orin 上從原始碼建置要花掉數小時，那不該由學生或車輛整合
人員來付出。代價是你接受別人替你建好的版本，它鎖定在 `versions.yaml`。

**但 AutoSDV 本身仍然要編譯。** `src/` 底下的一切都是一個 colcon 工作空間，
由 `just build` 建置。

因此你始終處於一種混合狀態：底層是二進位的 Autoware，上層是原始碼工作空間。
這個結構正是[環境與相依套件](../../concepts/environment.md)中那兩行 `source`
的意義所在，也是套件相依性被解析的地方——而不是在建置時。之後若出現
「package not found」，請先讀那一頁。

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

<span id="prepare-operating-system"></span>
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
3. 使用 deb（網路）安裝程式安裝 **CUDA 12.x**
4. 安裝 **TensorRT 10.x**

!!! warning "TensorRT 必須是 10.x，不是 8.x"

    Autoware 1.5.0 自己的函式庫是對 `libnvinfer.so.10` 連結的。TensorRT 8.6
    提供的是 `libnvinfer.so.8`，因此感知模組會載入失敗，而錯誤訊息指向某個共享
    物件而非真正的原因。本頁較早的版本寫「TensorRT 8.6 GA」；那適用於較舊的
    Autoware 基底。

    你可以驗證你的安裝實際需要什麼：

    ```bash
    ldd /opt/autoware/1.5.0/lib/libtensorrt_ops.so | grep nvinfer
    # libnvinfer.so.10 => ...
    ```

    同樣的檢查可得出 CUDA 主版本：

    ```bash
    ldd /opt/autoware/1.5.0/lib/libautoware_lidar_centerpoint_cuda_lib.so \
      | grep -E 'cublas|cudart'
    # libcublas.so.12 => ...
    ```

    這些 soname 才是真正的需求。任何滿足它們的 CUDA 12.x 與 TensorRT 10.x 都
    可以；確切的修訂版本並不重要。

### Docker

請參閱 [Docker 安裝](./docker.md) 以進行容器化安裝。這會跳過上述的作業系統準備步驟。

## 安裝 ZED SDK（如果使用 ZED 相機）

**ZED SDK 必須在繼續之前手動安裝。**

如果您使用 ZED 相機，需要安裝 ZED SDK 和 ZED Link 驅動程式。請參閱 [ZED SDK 安裝指南](./zed-sdk.md) 以取得詳細說明。

> **注意**：這是手動安裝步驟。自動安裝腳本不會安裝 ZED SDK。

## 下一步

作業系統準備完成後，請前往[推薦安裝方式](./recommended.md)指南。
