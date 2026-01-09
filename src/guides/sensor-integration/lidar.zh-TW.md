<!--
Translation Metadata:
- Source file: lidar.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 光達感測器

支援的光達型號配置詳情。

## 支援的型號

| 型號 | 類型 | 視野 | 範圍 | 網路 | 配置 |
|-------|------|-----|-------|---------|--------|
| **Seyond Robin-W** | 固態 | 120° × 25° | 200m | 172.168.1.10 | `lidar_model:=robin-w` |
| **Velodyne VLP-32C** | 旋轉式 | 360° × 40° | 200m | 192.168.1.201 | `lidar_model:=vlp32c` |
| **Blickfeld Cube1** | 固態 | 70° × 30° | 150m | 192.168.26.26 | `lidar_model:=cube1` |

## Robin-W

### 網路設定

**光達 IP**：172.168.1.10（固定）
**Jetson IP**：172.168.1.100/24（配置在同一子網路）

```bash
# Configure Jetson network interface
sudo ip addr add 172.168.1.100/24 dev eth0
sudo ip link set eth0 up

# Test connectivity
ping 172.168.1.10
```

<span id="coordinate-transformation"></span>
### 座標轉換

**重要**：Robin-W 需要旋轉以符合 ROS 標準座標。

**原生座標**：X=向上, Y=向右, Z=向前
**ROS 標準**：X=向前, Y=向左, Z=向上

**必要的校正**：
```yaml
robin_lidar_link:
  roll: 3.14159    # 180° flip
  pitch: -1.5708   # -90° rotation
  yaw: 0.0
```

詳細說明請參閱[整合範例](./integration-walkthrough.md)。

### 驅動程式套件

**位置**：`src/sensor_component/external/seyond_ros_driver/`
**點格式**：PointXYZIRC（Autoware 相容）

### 獨立測試

```bash
colcon build --packages-select seyond_ros_driver
source install/setup.bash
ros2 launch seyond_ros_driver robin_w.launch.py

# Verify
ros2 topic hz /robin_lidar/points_raw  # ~10 Hz
```

### 疑難排解

**無資料**：檢查 `ping 172.168.1.10` 是否成功
**方向錯誤**：驗證校正中的 roll=3.14159, pitch=-1.5708

## Velodyne VLP-32C

### 網路設定

**光達 IP**：192.168.1.201（出廠預設）
**Jetson IP**：192.168.1.100/24

```bash
# Configure Jetson
sudo ip addr add 192.168.1.100/24 dev eth0

# Test connectivity
ping 192.168.1.201
```

### 座標系統

**標準 ROS 座標** - 不需要旋轉：
```yaml
velodyne_link:
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

### 驅動程式套件

**位置**：`src/sensor_component/external/velodyne/`
**安裝**：`sudo apt install ros-humble-velodyne`
**點格式**：PointXYZIR（標準 Velodyne）

### 進階：雙回波模式

在雨霧中獲得更好的效能：
```yaml
/**:
  ros__parameters:
    rpm: 600
    return_type: "dual"  # Both strongest and last returns
```

### 獨立測試

```bash
ros2 launch velodyne_pointcloud velodyne_driver_node-VLP32C-launch.py device_ip:=192.168.1.201

# Verify
ros2 topic hz /velodyne_points  # ~10-20 Hz
```

### 疑難排解

**封包遺失**：增加 UDP 緩衝區大小：
```bash
sudo sysctl -w net.core.rmem_max=26214400
```

**掃描間隙**：配置 CycloneDDS 緩衝區（參閱安裝指南）

## Blickfeld Cube1

### 網路設定

**光達 IP**：192.168.26.26（固定）
**Jetson IP**：192.168.26.1/24

```bash
# Configure Jetson
sudo ip addr add 192.168.26.1/24 dev eth0
sudo ip link set eth0 up

# Test connectivity
ping 192.168.26.26
```

### 座標系統

**標準 ROS 座標** - 不需要旋轉：
```yaml
bf_lidar_link:
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

### 驅動程式套件

**位置**：`src/sensor_component/external/ros2_blickfeld_driver_src-v1.5.5/`
**需要**：Blickfeld Scanner Library 2.20.6-newslab1（透過 `just blickfeld` 安裝）

### 獨立測試

```bash
colcon build --packages-select blickfeld_driver
source install/setup.bash
ros2 launch blickfeld_driver live_scanner_node.launch.py

# Verify
ros2 topic hz /bf_lidar/points_raw
```

### 疑難排解

**連接失敗**：驗證已安裝 Scanner Library：
```bash
dpkg -l | grep blickfeld
```

**EULA 錯誤**：執行 `just blickfeld` 以接受授權

## 多光達設定

結合多個光達以增加覆蓋範圍：

```yaml
# sensor_kit_calibration.yaml
sensor_kit_base_link:
  robin_lidar_link:    # Front-facing
    x: 0.15
    z: 0.15
    roll: 3.14159
    pitch: -1.5708

  velodyne_link:       # Top 360°
    x: 0.0
    z: 0.30
    roll: 0.0
    pitch: 0.0
```

在 `pointcloud_preprocessor.launch.py` 中配置融合：
```python
"input_topics": [
    "/sensing/lidar/robin_lidar/points_raw",
    "/sensing/lidar/velodyne/points_raw",
]
```

## 快速參考

```bash
# Network tests
ping 172.168.1.10    # Robin-W
ping 192.168.1.201   # Velodyne
ping 192.168.26.26   # Blickfeld

# Topic verification
ros2 topic list | grep /sensing/lidar
ros2 topic hz /sensing/lidar/robin_lidar/points_raw

# TF verification
ros2 run tf2_ros tf2_echo sensor_kit_base_link robin_lidar_link

# Network monitoring
sudo iftop -i eth0
```
