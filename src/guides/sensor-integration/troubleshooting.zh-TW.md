<!--
Translation Metadata:
- Source file: troubleshooting.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 感測器疑難排解

本指南提供 AutoSDV 中常見感測器整合問題的解決方案。

## 快速診斷指令

在深入研究具體問題之前，執行這些指令以收集資訊：

```bash
# List all active topics
ros2 topic list

# Check sensor topics specifically
ros2 topic list | grep /sensing

# View TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Check node status
ros2 node list

# View logs with debug level
ros2 launch autosdv_launch autosdv.launch.yaml --log-level DEBUG
```

---

## 一般問題

### 無感測器資料

**症狀**：
- 主題存在但 `ros2 topic echo` 沒有顯示任何內容
- `ros2 topic hz` 顯示 0 Hz
- 驅動程式啟動但未發布資料

**診斷步驟**：

1. **檢查實體連接**：
   ```bash
   # USB devices
   lsusb
   # Should list your sensor

   # Serial devices
   ls -l /dev/tty*
   # Should show device (e.g., /dev/ttyUSB0)

   # Network devices
   ip addr show
   # Verify Ethernet interface up

   # Test network sensors
   ping <sensor_ip>
   # Should respond with <5ms latency
   ```

2. **驗證裝置權限**：
   ```bash
   # Add user to dialout group (for serial)
   sudo usermod -aG dialout $USER

   # Add user to video group (for cameras)
   sudo usermod -aG video $USER

   # Log out and back in for changes to take effect
   ```

3. **檢查驅動程式日誌**：
   ```bash
   # Launch with debug logging
   ros2 launch autosdv_launch autosdv.launch.yaml --log-level DEBUG

   # Or check specific node
   ros2 node info /sensing/lidar/robin_lidar/driver_node
   ```

4. **驗證電源**：
   - 檢查感測器 LED 指示燈
   - 使用三用電表測量電壓
   - 確保電源供應有足夠的電流

**按感測器類型的解決方案**：
- **光達**：檢查網路設定、IP 配置
- **相機**：驗證已安裝 ZED SDK、GMSL 連接
- **IMU**：檢查 I2C 匯流排、裝置位址
- **GNSS**：確保有清晰的天空視野，等待衛星取得

---

### 錯誤的座標框架 / 感測器位置錯誤

**症狀**：
- 在 RViz 中可看到感測器資料但位置錯誤
- 點雲上下顛倒或旋轉不正確
- RViz 中的轉換錯誤

**診斷步驟**：

1. **驗證 TF 轉換**：
   ```bash
   # Generate TF tree
   ros2 run tf2_tools view_frames
   evince frames.pdf

   # Check specific transform
   ros2 run tf2_ros tf2_echo base_link sensor_link
   # Compare output with calibration values
   ```

2. **檢查校正檔案**：
   ```bash
   # View current calibration
   cat src/sensor_kit/autosdv_sensor_kit_description/config/sensor_kit_calibration.yaml

   # Look for your sensor's x, y, z, roll, pitch, yaw values
   ```

**解決方案**：

1. **修正校正值**：
   ```yaml
   # Edit: sensor_kit_calibration.yaml
   sensor_kit_base_link:
     your_sensor_link:
       x: 0.15      # Forward/backward (meters)
       y: 0.02      # Left/right (meters)
       z: 0.20      # Up/down (meters)
       roll: 0.0    # Rotation around X-axis (radians)
       pitch: 0.0   # Rotation around Y-axis (radians)
       yaw: 0.0     # Rotation around Z-axis (radians)
   ```

2. **了解座標系統**：
   - **ROS 標準（REP-103）**：X=向前、Y=向左、Z=向上
   - **某些感測器使用不同的慣例**（請參閱感測器指南）
   - 計算將感測器對齊到 ROS 標準所需的旋轉

3. **在 RViz 中反覆調整**：
   - 啟動系統並在 RViz 中檢視
   - 調整校正檔案中的 roll/pitch/yaw
   - 重新建置並重新啟動以查看變更
   - 重複直到正確

**常見旋轉**：
- **Robin-W 光達**：roll=3.14159, pitch=-1.5708, yaw=0.0
- **ZED 相機**：roll=0.0, pitch=0.0, yaw=0.0（標準）

---

### 主題命名空間問題

**症狀**：
- 主題出現在意外的命名空間中
- 在 `/sensing/` 中找不到感測器主題
- 主題重新映射不運作

**診斷步驟**：

1. **檢查實際主題名稱**：
   ```bash
   ros2 topic list | grep -E '(lidar|camera|imu|gnss)'
   # Compare with expected: /sensing/[type]/[name]/[data]
   ```

2. **驗證啟動檔案重新映射**：
   ```bash
   # Check sensor kit launch files
   cat src/sensor_kit/autosdv_sensor_kit_launch/launch/lidar.launch.xml
   # Look for <remap> tags
   ```

**解決方案**：

1. **新增或修正主題重新映射**：
   ```xml
   <!-- In sensor kit launch file -->
   <remap from="/driver_topic_name" to="/sensing/lidar/sensor_name/points_raw"/>
   ```

2. **使用絕對主題路徑**：
   ```xml
   <!-- Bad (relative) -->
   <remap from="points" to="points_raw"/>

   <!-- Good (absolute) -->
   <remap from="/sensor/points" to="/sensing/lidar/sensor/points_raw"/>
   ```

3. **對於 ZED 相機命名空間問題**：使用可組合節點載入（請參閱[相機感測器指南](./camera.md#special-namespace-workaround)）

---

### 效能問題 / 高 CPU 使用率

**症狀**：
- 系統延遲或回應緩慢
- 高 CPU 使用率（使用 `htop` 檢查）
- 來自感測器的畫面速率低
- 訊息遺失

**診斷步驟**：

1. **檢查主題頻寬**：
   ```bash
   # Monitor data rate
   ros2 topic bw /sensing/lidar/robin_lidar/points_raw

   # Check all sensor topics
   ros2 topic bw /sensing/*
   ```

2. **檢查 CPU 使用率**：
   ```bash
   # Real-time process monitor
   htop

   # GPU usage (NVIDIA)
   nvidia-smi
   ```

3. **驗證 CycloneDDS 配置**：
   ```bash
   # Check kernel buffers
   sysctl net.core.rmem_max
   sysctl net.ipv4.ipfrag_time
   sysctl net.ipv4.ipfrag_high_thresh

   # Should be configured per Installation Guide
   ```

**解決方案**：

1. **配置 CycloneDDS 緩衝區**（如果尚未完成）：
   ```bash
   cd setup
   just cyclonedds-sysctl
   ```

2. **降低感測器資料速率**：
   ```yaml
   # Edit sensor parameters
   # For cameras:
   general.grab_frame_rate: 15  # Reduce from 30

   # For LiDAR:
   downsample_factor: 2  # Reduce points by half
   ```

3. **停用重型功能**：
   ```bash
   # Disable ZED object detection
   ros2 launch autosdv_launch autosdv.launch.yaml enable_zed_object_detection:=false
   ```

4. **使用輕量級感測器套件**：
   ```bash
   # VLP-32C + ZED IMU only (no camera processing)
   ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=vlp32c_zed_imu
   ```

---

## 感測器特定問題

### 光達

#### 無點雲

**症狀**：光達驅動程式啟動但未發布點

**解決方案**：
1. **檢查網路連接**（Ethernet 光達）：
   ```bash
   ping <lidar_ip>
   # Robin-W: 172.168.1.10
   # Velodyne: 192.168.1.201
   # Blickfeld: 192.168.26.26
   ```

2. **在相同子網路上配置 Jetson IP**：
   ```bash
   # For Robin-W (172.168.1.x)
   sudo ip addr add 172.168.1.100/24 dev eth0
   sudo ip link set eth0 up

   # For Blickfeld (192.168.26.x)
   sudo ip addr add 192.168.26.1/24 dev eth0
   sudo ip link set eth0 up
   ```

3. **檢查防火牆**：
   ```bash
   # Disable firewall temporarily for testing
   sudo ufw disable

   # Or allow specific ports
   sudo ufw allow from <lidar_ip>
   ```

#### TensorRT 編譯耗時過長

**症狀**：首次啟動需要 10-30 分鐘並顯示 TensorRT 訊息

**這是正常的**首次啟動。TensorRT 為您的特定 GPU 編譯模型。

**解決方案**：
- 耐心等待 - 僅發生一次
- 模型快取在 `~/.autoware/models/`
- 後續啟動速度快

#### 點雲方向錯誤

**解決方案**：檢查座標框架旋轉（特別是 Robin-W，需要 roll=180°, pitch=-90°）

請參閱[光達感測器 - 座標轉換](./lidar.md#coordinate-transformation)

---

### 相機（ZED）

#### 未偵測到相機

**症狀**：ZED_Explorer 顯示「No camera detected」

**解決方案**：
1. **檢查 GMSL 連接**：
   ```bash
   # Verify ZED Link card
   lspci | grep NVIDIA
   # Should show ZED Link Duo
   ```

2. **將相機重新啟動**：
   - 拔除相機電源
   - 等待 10 秒
   - 重新連接
   - 重新啟動 Jetson

3. **驗證 ZED SDK 安裝**：
   ```bash
   cat /usr/local/zed/settings/version.txt
   # Should show 5.1.2
   ```

4. **檢查權限**：
   ```bash
   sudo usermod -aG video $USER
   # Log out and back in
   ```

#### VNC/遠端存取問題

**問題**：ZED 相機在標準 VNC 上無法運作

**解決方案**：使用 TurboVNC + VirtualGL 進行硬體加速

```bash
# Install TurboVNC
sudo apt install turbovnc virtualgl

# Start VNC with VirtualGL
vglrun /opt/TurboVNC/bin/vncserver

# Launch apps with vglrun
vglrun ZED_Explorer
vglrun rviz2
```

#### 命名空間問題

**問題**：主題在錯誤的命名空間中，無法重新映射

**解決方案**：使用可組合節點載入而非 Python 啟動檔案

請參閱[相機感測器 - 命名空間解決方案](./camera.md#special-namespace-workaround)

---

### IMU

#### 未偵測到 MPU9250

**症狀**：i2cdetect 未顯示裝置

**解決方案**：
1. **檢查 I2C 匯流排**：
   ```bash
   # List I2C buses
   ls /dev/i2c-*

   # Scan bus 1
   sudo i2cdetect -y -r 1
   # Should show device at 0x68 or 0x69
   ```

2. **驗證配線**：
   - VCC → 3.3V（不是 5V！）
   - GND → GND
   - SDA → Pin 3（I2C1 SDA）
   - SCL → Pin 5（I2C1 SCL）

3. **檢查 I2C 位址**：
   ```bash
   # Try alternate address
   sudo i2cdetect -y -r 1
   # Look for 0x68 or 0x69
   ```

#### IMU 資料雜訊

**症狀**：大幅波動、方向不穩定

**解決方案**：
1. **檢查 IMU 配置**：請參閱 [IMU 感測器 - 配置](./imu.md#configuration)
2. **固定安裝**：將振動降至最低
3. **加入軟體濾波**：驅動程式配置中的低通濾波器

#### ZED IMU 未發布

**症狀**：ZED 相機運作但無 IMU 主題

**解決方案**：
1. **在 ZED 配置中啟用 IMU**：
   ```yaml
   sensors.sensors_image_sync: true
   sensors.publish_imu_tf: true
   ```

2. **檢查 IMU 中繼節點**：
   ```bash
   ros2 node list | grep relay
   # Should show IMU relay node running
   ```

---

### GNSS

#### 無 GNSS 定位

**症狀**：定位狀態保持為 -1（無定位）

**解決方案**：
1. **確保有清晰的天空視野**：移至開放區域（無建築物/樹木）
2. **等待衛星取得**：冷啟動需要 1-10 分鐘
3. **檢查天線連接**：驗證纜線固定
4. **監控衛星數量**：3D 定位至少需要 4 顆衛星

#### 無 RTK 定位（u-blox）

**症狀**：有 GPS 定位但無 RTK（status != 2）

**解決方案**：
1. **檢查 NTRIP 連接**：
   ```bash
   ros2 topic hz /sensing/gnss/ntrip/rtcm
   # Should show ~1 Hz
   ```

2. **驗證網際網路連接**：NTRIP 需要網路
3. **檢查憑證**：使用正確的使用者名稱/密碼編輯 `ntrip.launch.xml`
4. **等待收斂**：RTK 可能需要 1-10 分鐘
5. **改善天線位置**：更高、更少障礙物

#### Garmin GPS 無法運作

**症狀**：Garmin GPS 18x 無資料

**解決方案**：
1. **啟動 gpsd**：
   ```bash
   sudo /usr/sbin/gpsd -n -G -b /dev/ttyUSB0
   ```

2. **檢查裝置**：
   ```bash
   ls -l /dev/ttyUSB*
   sudo cat /dev/ttyUSB0
   # Should see NMEA sentences
   ```

3. **設定鮑率**：
   ```bash
   sudo stty -F /dev/ttyUSB0 9600
   ```

4. **使用 cgps 測試**：
   ```bash
   cgps
   # Should show GPS data
   ```

---

## 建置錯誤

### 找不到套件

**症狀**：`colcon build` 失敗並顯示「package 'X' not found」

**解決方案**：
1. **更新 rosdep**：
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **檢查 package.xml 相依性**：確保所有 `<depend>` 標籤正確

3. **安裝缺少的 ROS 套件**：
   ```bash
   sudo apt install ros-humble-<package-name>
   ```

### CMake 錯誤

**症狀**：CMake 配置失敗

**解決方案**：
1. **清除建置**：
   ```bash
   rm -rf build install log
   colcon build
   ```

2. **檢查 CMakeLists.txt**：驗證語法、相依性

3. **載入 ROS 2**：
   ```bash
   source /opt/ros/humble/setup.bash
   ```

---

## 取得協助

如果在嘗試這些解決方案後問題仍然存在：

### 1. 啟用除錯日誌記錄

```bash
ros2 launch autosdv_launch autosdv.launch.yaml --log-level DEBUG 2>&1 | tee debug.log
```

### 2. 收集診斷資訊

```bash
# System info
lsb_release -a
uname -a

# ROS info
ros2 doctor
ros2 wtf

# Topic info
ros2 topic list
ros2 topic hz /sensing/lidar/*/points_raw

# TF tree
ros2 run tf2_tools view_frames
```

### 3. 檢查 GitHub Issues

搜尋 [AutoSDV Issues](https://github.com/NEWSLabNTU/AutoSDV/issues) 以尋找類似問題

### 4. 建立 Issue

報告問題時，請包含：
- 硬體設定（感測器型號、連接）
- 使用的啟動指令
- 完整的錯誤訊息/日誌
- 診斷指令的輸出
- 重現步驟

---

## 有用的診斷工具

### ROS 2 工具

```bash
# List nodes
ros2 node list

# Node info
ros2 node info /node_name

# Topic info
ros2 topic info /topic_name
ros2 topic hz /topic_name
ros2 topic bw /topic_name
ros2 topic echo /topic_name

# Parameter inspection
ros2 param list /node_name
ros2 param get /node_name parameter_name

# Service calls
ros2 service list
ros2 service call /service_name service_type "{args}"

# TF debugging
ros2 run tf2_ros tf2_echo frame1 frame2
ros2 run tf2_tools view_frames
```

### 系統工具

```bash
# Process monitoring
htop
top

# GPU monitoring
nvidia-smi
watch -n 1 nvidia-smi

# Network
ip addr show
ping <ip>
iftop -i eth0

# USB devices
lsusb
lsusb -t

# Serial devices
ls -l /dev/tty*
sudo dmesg | grep tty

# I2C devices
ls /dev/i2c-*
sudo i2cdetect -y -r 1

# Disk usage
df -h
du -sh *
```

---

## 相關指南

有關感測器特定的疑難排解，請參閱：
- [光達感測器](./lidar.md) - Robin-W、Velodyne、Blickfeld 問題
- [相機感測器](./camera.md) - ZED 相機疑難排解
- [IMU 感測器](./imu.md) - MPU9250 和 ZED IMU 問題
- [GNSS 感測器](./gnss.md) - GNSS 和 RTK 疑難排解

有關整合協助：
- [新增感測器](./adding-sensor.md) - 逐步整合指南
- [使用感測器](./using-sensors.md) - 感測器入門
