<!--
Translation Metadata:
- Source file: docker.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# Docker 設定

Docker 提供容器化環境以執行 AutoSDV，而不需修改您的主機系統。這對於開發、測試與模擬情境很理想。

## 使用情境

Docker 建議用於：
- **開發與測試**：在不同機器上的一致環境
- **模擬**：在沒有實體硬體的情況下執行 AutoSDV
- **CI/CD**：自動化測試與部署
- **快速評估**：在不完整安裝的情況下試用 AutoSDV

## 先決條件

### 主機系統需求

- Ubuntu 20.04 或 22.04（其他 Linux 發行版可能可用）
- NVIDIA GPU 配備驅動程式 470+（用於 GPU 加速）
- 至少 50GB 可用磁碟空間
- 建議 16GB+ RAM

### 軟體需求

1. **Docker Engine**（20.10 或更新版本）：
   ```bash
   # Install Docker
   curl -fsSL https://get.docker.com -o get-docker.sh
   sudo sh get-docker.sh

   # Add user to docker group
   sudo usermod -aG docker $USER
   # Log out and back in for group changes to take effect
   ```

2. **NVIDIA Container Toolkit**（用於 GPU 支援）：
   ```bash
   # Add NVIDIA repository
   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
   curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
   curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
     sudo tee /etc/apt/sources.list.d/nvidia-docker.list

   # Install nvidia-container-toolkit
   sudo apt update
   sudo apt install nvidia-container-toolkit
   sudo systemctl restart docker
   ```

3. **Docker Compose**（選配，用於多容器設定）：
   ```bash
   sudo apt install docker-compose
   ```

## 快速開始

### 步驟 1：複製 AutoSDV 儲存庫

```bash
git clone -b 2025.02 --recursive https://github.com/NEWSLabNTU/AutoSDV.git
cd AutoSDV/docker
```

### 步驟 2：啟動 Docker 環境

設定跨架構支援（在 x86_64 上進行 ARM64 模擬時需要）：

```bash
make bootstrap
```

### 步驟 3：建置 Docker 映像

建置 AutoSDV Docker 映像：

```bash
make build
```

這會建立一個包含以下內容的映像：
- Ubuntu 22.04 基底配備 ROS 2 Humble
- 預先安裝的 Autoware 2025.02
- 所有 AutoSDV 相依套件
- CUDA 與 TensorRT 支援
- 感測器驅動程式函式庫（專有的除外）

### 步驟 4：執行容器

啟動互動式容器會話：

```bash
make run
```

您將進入一個 shell，AutoSDV 在 `/home/developer/AutoSDV` 準備就緒可供使用。

## Docker 映像詳情

### 映像架構

AutoSDV Docker 映像是為 **ARM64 架構**建置的，以匹配 Jetson 平台。在 x86_64 主機上，QEMU 提供透明模擬。

### 預先安裝的軟體

- **ROS 2 Humble** 配備桌面工具
- **Autoware 2025.02** 二進位發行版
- **CUDA 12.3** 與 **TensorRT 8.6**
- **Cyclone DDS** 配置為預設值
- **開發工具**：git、vim、tmux、htop

### 磁碟區掛載

`make run` 指令會自動掛載：
- `/tmp/.X11-unix` 用於 GUI 應用程式
- NVIDIA GPU 裝置以進行 CUDA 存取

## 進階使用

### 自訂執行選項

使用額外的磁碟區或埠執行：

```bash
docker run -it --rm \
  --gpus all \
  --network host \
  -v /path/to/data:/data \
  -v /dev:/dev \
  --privileged \
  autosdv:latest
```

### 開發工作流程

對於主動開發，掛載您的本地程式碼：

```bash
docker run -it --rm \
  --gpus all \
  -v $(pwd):/workspace/AutoSDV \
  -w /workspace/AutoSDV \
  autosdv:latest
```

### GUI 應用程式

為視覺化工具啟用 X11 轉發：

```bash
xhost +local:docker
docker run -it --rm \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  autosdv:latest
```

然後執行 GUI 應用程式，例如 RViz：
```bash
# Inside container
rviz2
```

### 多容器設定

為複雜部署建立 `docker-compose.yml`：

```yaml
version: '3.8'

services:
  autosdv:
    image: autosdv:latest
    runtime: nvidia
    network_mode: host
    privileged: true
    volumes:
      - /dev:/dev
      - ./data:/data
    environment:
      - ROS_DOMAIN_ID=0
      - DISPLAY=${DISPLAY}
    command: ros2 launch autosdv_launch autosdv.launch.yaml

  monitoring:
    image: autosdv:latest
    runtime: nvidia
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
    command: python3 /home/developer/AutoSDV/src/launcher/autosdv_launch/autosdv_launch/autosdv_monitor.py
```

執行：
```bash
docker-compose up
```

## 容器管理

### 儲存與載入映像

匯出映像以供部署：

```bash
make save  # Creates autosdv_docker.tar.gz
```

在另一台機器上載入：

```bash
docker load < autosdv_docker.tar.gz
```

### 清理

移除容器與映像：

```bash
make clean
```

## 限制

### 硬體存取

Docker 容器的硬體存取有限：
- **無法直接存取光達**（USB/乙太網路感測器需要特殊配置）
- **無 CAN 匯流排**（除非使用 `--privileged` 旗標）
- **相機存取**需要裝置掛載

### 效能

- x86_64 上的 ARM64 模擬會降低效能
- GPU 穿透會增加開銷
- 網路效能可能因 Docker 網路模式而異

### 感測器驅動程式

Docker 映像透過 rosdep 包含大多數驅動程式，但：

- **ZED SDK**：由於硬體需求，無法在 Docker 中完全使用。實體部署需要原生安裝。
- **Blickfeld**：透過容器中的 rosdep 安裝
- **Velodyne**：透過容器中的 rosdep 安裝

對於完整的 ZED 相機支援，請使用原生安裝與 [ZED SDK 安裝指南](./zed-sdk.md)。

## 疑難排解

### GPU 無法存取

驗證 NVIDIA runtime：
```bash
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

### 網路問題

使用主機網路進行 ROS 2 通訊：
```bash
docker run --network host ...
```

### 權限被拒

對於裝置存取，使用權限執行：
```bash
docker run --privileged -v /dev:/dev ...
```

### 建置失敗

清除 Docker 快取並重新建置：
```bash
docker system prune -a
make bootstrap
make build
```

## 與 CI/CD 整合

### GitHub Actions 範例

```yaml
name: AutoSDV Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2

      - name: Set up Docker
        uses: docker/setup-buildx-action@v1

      - name: Build Docker image
        run: |
          cd docker
          make build

      - name: Run tests
        run: |
          docker run --rm autosdv:latest \
            bash -c "cd /home/developer/AutoSDV && colcon test"
```

### Jenkins Pipeline 範例

```groovy
pipeline {
    agent any

    stages {
        stage('Build') {
            steps {
                sh 'cd docker && make build'
            }
        }

        stage('Test') {
            steps {
                sh 'docker run --rm autosdv:latest make test'
            }
        }
    }
}
```

## 下一步

- [軟體安裝](./overview.md) - 原生安裝指南
- [使用指南](../usage.md) - 操作 AutoSDV
- [開發指南](../../guides/development.md) - 開發工作流程
- [手動設定](./manual-environment.md) - 客製化選項
