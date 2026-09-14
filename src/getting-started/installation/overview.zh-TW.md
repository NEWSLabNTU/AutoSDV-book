<!--
Translation Metadata:
- Source file: overview.md
- Last synced: 2026-09-14
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

## 你要把它裝在什麼機器上

1. **一台 Ubuntu 22.04 筆電或桌機**——用於模擬與開發。有 GPU 有幫助，但
   **不是必要**；兩種模擬沒有 GPU 也能跑。請保留 **20 GB** 可用空間。核心數、
   記憶體、建置尖峰，以及 GPU 買到什麼、買不到什麼，實測數字在下一頁
   [你需要什麼樣的機器](../requirements.md)。
2. **NVIDIA Jetson AGX Orin 64GB**——車輛本身，接著各種感測器。見
   [硬體設定](../hardware-assembly.md)。
3. **Docker 環境**（未維護）

Ubuntu 22.04 是唯一沒有替代方案的要求：AutoSDV 建立在 ROS 2 Humble 上，而 Humble
沒有 24.04 的套件。

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
2. 燒錄 Jetson，設定如下：
   - **JetPack SDK 6.2.2 或更新**——這是下限，不是某個確切版本
   - 勾選所有 CUDA 與 TensorRT 套件
   - 目標選外接 NVMe SSD，不要用內建 eMMC

板子開起來後，**要看的是 L4T 版本，不是 JetPack 版本**：

```bash
head -1 /etc/nv_tegra_release     # "# R36 (release), REVISION: 5.0" -> L4T 36.5.0
```

JetPack 的修訂號在 6.2 系列內部跨越了一個 L4T **次版本**，這既是下限訂在這裡的
原因，也是後面好幾個下載連結問的是 L4T 而不是 JetPack 的原因（[NVIDIA 的對應
表](https://developer.nvidia.com/embedded/jetpack-archive)）：

| JetPack | L4T | |
|---------|-----|---|
| 6.2 | 36.4.3 | 低於下限 |
| 6.2.1 | 36.4.4 | 低於下限 |
| **6.2.2** | **36.5.0** | 下限 |
| 6.2.3 | 36.5.2 | 可以 |

有三件事看的是這個號碼而不是 JetPack 的號碼：[ZED SDK 安裝檔](./zed-sdk.md)是按
L4T 次版本發佈的、NVIDIA 的 Jetson apt pocket 是 `r36.5`，以及預先建置的 TensorRT
引擎集是以 L4T 為鍵——所以在下限版本的板子上，不會命中從 JetPack 6.2.1 發佈的那
份引擎集，第一次會自己建置，約一小時。那是慢，不是壞；見
[TensorRT 引擎，事後補做](./recommended.md#tensorrt-引擎事後補做)。

arm64 版 Autoware 套件在每個修訂版都保留 `jetpack62` 這個檔名後綴：它指的是建置
時所針對的 6.2 系列，而 6.2.2 就在這個系列裡。

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
