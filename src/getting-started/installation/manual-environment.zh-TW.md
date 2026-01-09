<!--
Translation Metadata:
- Source file: manual-environment.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 手動設定與客製化

本指南涵蓋進階配置選項與自動化安裝的替代方案。**大多數使用者應使用[自動設定](./overview.md)**。

## 何時使用手動設定

在以下情況使用手動設定：
- **自訂 Autoware**：從原始碼建置 Autoware 以修改核心組件
- **效能調校**：用於生產部署的即時最佳化
- **自訂 DDS 配置**：特定的網路需求
- **開發工具**：額外的除錯與效能分析公用程式
- **非標準安裝路徑**：自訂目錄結構

## 自動設定所做的事情

在繼續之前，請了解 `./setup.sh` 已安裝的內容：

- ROS 2 Humble (ros-humble-desktop)
- ROS 2 開發工具 (colcon, rosdep, vcstool)
- Autoware 2025.02 Debian 套件（選配，建議）
- Blickfeld Scanner Library（用於 Cube1 LiDAR）
- Velodyne、NMEA 與 serial 驅動程式（透過 rosdep）
- 開發工具：git-lfs、golang、pre-commit、clang-format、plotjuggler
- Python 相依套件：Adafruit-PCA9685、simple-pid、ros2systemd
- u-blox GPS udev 規則與使用者群組權限

以下的手動步驟提供此自動設定的替代方案或補充。

## 先決條件

1. **作業系統已準備好**（參閱[總覽中的步驟 1](./overview.md#step-1-prepare-operating-system)）
2. **已安裝 ZED SDK 5.1**（如果使用 ZED 相機，參閱 [ZED SDK 安裝](./zed-sdk.md)）

## 從原始碼建置 Autoware

自動設定會從 `/opt/autoware` 安裝 Autoware Debian 套件。若要修改 Autoware 核心組件，請改為從原始碼建置：

### 步驟 1：複製 Autoware 儲存庫

```bash
mkdir -p ~/autoware_ws/src
cd ~/autoware_ws
git clone https://github.com/autowarefoundation/autoware.git -b release/2025.02
```

### 步驟 2：安裝相依套件

```bash
cd autoware
./setup-dev-env.sh
```

此腳本會安裝：
- CUDA、cuDNN、TensorRT（如果偵測到 NVIDIA GPU）
- ROS 2 Humble 與開發工具
- GeographicLib 與其他 Autoware 相依套件

### 步驟 3：匯入儲存庫

```bash
mkdir -p src
vcs import src < autoware.repos
```

### 步驟 4：安裝 ROS 相依套件

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro humble
```

### 步驟 5：建置 Autoware

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

建置時間：30-60 分鐘，取決於 CPU 核心數與是否建置感知套件。

### 步驟 6：來源工作空間

```bash
echo "source ~/autoware_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 使用自訂 Autoware 路徑

如果您在自訂位置建置 Autoware，AutoSDV 會在您建置 AutoSDV 之前先來源 Autoware 工作空間時自動偵測到它。

## 系統最佳化

### 即時效能調校

對於生產部署，配置系統以達到即時效能：

**1. 配置 CPU 調節器：**
```bash
sudo apt install cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl restart cpufrequtils
```

**2. 增加系統限制：**

新增至 `/etc/security/limits.conf`：
```
* soft memlock unlimited
* hard memlock unlimited
* soft rtprio 99
* hard rtprio 99
```

**3. Jetson 特定最佳化：**
```bash
# Lock clocks to maximum performance
sudo jetson_clocks

# Set performance mode (0 = MAXN)
sudo nvpmodel -m 0
```

## ROS 2 中介軟體配置

### Cyclone DDS 配置（建議）

Cyclone DDS 是 Autoware 的建議中介軟體。請按照以下步驟進行最佳配置：

**1. 配置核心網路緩衝區：**

```bash
# Apply immediately
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728

# Make persistent across reboots
sudo tee /etc/sysctl.d/10-cyclone-max.conf > /dev/null << EOF
net.core.rmem_max=2147483647
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF
```

**2. 建立 CycloneDDS 配置檔案：**

儲存為 `~/cyclonedds.xml`：

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="https://cdds.io/config
  https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="false" name="lo"
          priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>default</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Internal>
      <SocketReceiveBufferSize min="10MB"/>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
  </Domain>
</CycloneDDS>
```

**注意：** `name="lo"` 配置使用迴路介面進行單機操作。對於多機設定，請變更為您的網路介面（例如 `eth0`、`wlan0` 或 `enp0s31f6`）。使用 `ip link show` 列出可用的介面。

**3. 設定環境變數：**

```bash
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml' >> ~/.bashrc
source ~/.bashrc
```

**4. 選配 - 設定 ROS Domain ID：**

對於多車輛設定，為每輛車使用不同的 domain ID（1-255）：

```bash
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
```

**重要：** 請勿設定 `ROS_LOCALHOST_ONLY=1` - 它與 Autoware 有已知的相容性問題。

### FastDDS 配置（替代方案）

FastDDS 是替代的中介軟體。注意：Autoware 文件主要支援 CycloneDDS。

```bash
sudo apt install ros-humble-rmw-fastrtps-cpp
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
```

## 自訂建置選項

### 除錯建置

使用除錯符號建置 AutoSDV 以進行開發：

```bash
cd ~/AutoSDV
colcon build \
  --base-paths src \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### 選擇性套件建置

僅建置特定套件以節省時間：

```bash
# Build only vehicle interface packages
colcon build --packages-select \
  autosdv_vehicle_interface \
  autosdv_vehicle_launch

# Build package and all its dependencies
colcon build --packages-up-to autosdv_launch
```

### 交叉編譯

在 x86_64 主機上交叉編譯至 Jetson：

```bash
colcon build \
  --cmake-args \
  -DCMAKE_TOOLCHAIN_FILE=/path/to/jetson-toolchain.cmake \
  -DCMAKE_BUILD_TYPE=Release
```

## 額外開發工具

自動設定會安裝基本開發工具。為進階開發新增更多工具：

```bash
# Code quality tools
sudo apt install python3-autopep8 cppcheck

# Debugging and profiling
sudo apt install gdb valgrind heaptrack

# Build acceleration (can reduce build time by 50%)
sudo apt install ccache
echo 'export CC="ccache gcc"' >> ~/.bashrc
echo 'export CXX="ccache g++"' >> ~/.bashrc

# Advanced visualization
sudo apt install ros-humble-rqt ros-humble-rqt-common-plugins
```

## 環境變數

對開發有用的環境變數：

```bash
# CUDA paths (if not using Debian Autoware)
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
```

將這些新增至 `~/.bashrc` 以保持持久性。

**注意：** `ROS_DOMAIN_ID` 與 `RMW_IMPLEMENTATION` 在上方的 ROS 2 中介軟體配置區段中配置。

## 手動安裝疑難排解

### 遺失 ROS 相依套件

如果在手動安裝後遇到套件遺失錯誤：

```bash
# Update rosdep database
rosdep update --rosdistro=humble

# Install missing dependencies
cd ~/AutoSDV
rosdep install --from-paths src --ignore-src -r -y
```

### 函式庫路徑問題

對於安裝在非標準路徑的函式庫：

```bash
# Add custom library path
echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/local.conf
sudo ldconfig

# Verify library is found
ldconfig -p | grep <library_name>
```

### 權限問題

對於裝置存取（USB 感測器、CAN 匯流排、串列埠）：

```bash
# Add user to required groups
sudo usermod -aG dialout $USER    # Serial ports, GPS
sudo usermod -aG plugdev $USER    # USB devices
sudo usermod -aG video $USER      # Cameras

# Log out and back in for changes to take effect
```

### 建置失敗

如果 colcon build 失敗：

```bash
# Clean build artifacts
rm -rf build install log

# Try building with verbose output
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

# Build with single thread to see errors clearly
colcon build --executor sequential
```

## 驗證

手動設定後，驗證您的安裝：

```bash
# Check ROS 2 installation
ros2 --version

# Check Autoware installation (Debian)
dpkg -l | grep autoware

# Check Autoware installation (source build)
source ~/autoware_ws/install/setup.bash
ros2 pkg list | grep autoware

# Check sensor drivers
ros2 pkg list | grep velodyne
ros2 pkg list | grep nmea

# Test AutoSDV build
cd ~/AutoSDV
colcon test --packages-select autosdv_launch
```

## 下一步

- 返回[軟體安裝總覽](./overview.md)以完成標準工作流程
- 參閱[使用指南](../usage.md)以啟動與操作系統
- 檢查[開發指南](../../guides/development.md)以了解開發工作流程
