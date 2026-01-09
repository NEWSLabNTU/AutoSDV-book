<!--
Translation Metadata:
- Source file: adding-sensor.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 新增感測器

將新感測器整合到 AutoSDV 的快速檢查清單。

## 開始之前

**先了解系統**：閱讀[整合範例](./integration-walkthrough.md)以了解 Robin-W LiDAR 是如何整合的。這能讓您完整了解感測器在 AutoSDV 中的運作方式。

**先決條件**：
- 實體感測器硬體
- 感測器規格（電源、介面、資料格式）
- ROS 2 驅動程式套件（供應商提供或自訂）

## 整合檢查清單

### 1. 讓驅動程式運作

```bash
# Add driver package to src/sensor_component/external/
cd src/sensor_component/external/
git submodule add <driver_repository_url>

# Build standalone
colcon build --packages-select <driver_package>
source install/setup.bash

# Test independently
ros2 launch <driver_package> <launch_file>

# Verify topic publishes data
ros2 topic hz /<sensor_topic>
ros2 topic echo /<sensor_topic> --once
```

✅ **檢查點**：驅動程式執行並發布資料

### 2. 測量實體位置

測量感測器相對於 `sensor_kit_base_link` 的位置：
- X：前/後（公尺）
- Y：左/右（公尺）
- Z：上/下（公尺）

檢查感測器是否使用非標準座標（如 Robin-W）：
- 計算旋轉：roll、pitch、yaw（弧度）
- 參閱[整合範例](./integration-walkthrough.md#why-the-strange-rotation)的範例

### 3. 加入校正

**檔案**：`src/sensor_kit/autosdv_sensor_kit_description/config/sensor_kit_calibration.yaml`

```yaml
sensor_kit_base_link:
  your_sensor_link:
    x: 0.15       # Your measured values
    y: 0.0
    z: 0.20
    roll: 0.0     # Rotation if needed
    pitch: 0.0
    yaw: 0.0
```

### 4. 加入 URDF

**檔案**：`src/sensor_kit/autosdv_sensor_kit_description/urdf/sensor_kit.xacro`

```xml
<!-- Your Sensor -->
<link name="your_sensor_link"/>

<joint name="your_sensor_joint" type="fixed">
  <origin
    xyz="${calibration['sensor_kit_base_link']['your_sensor_link']['x']} ..."
    rpy="${calibration['sensor_kit_base_link']['your_sensor_link']['roll']} ..."
  />
  <parent link="sensor_kit_base_link"/>
  <child link="your_sensor_link"/>
</joint>
```

### 5. 加入啟動檔案

**檔案**：`src/sensor_kit/autosdv_sensor_kit_launch/launch/<sensor_type>.launch.xml`

```xml
<group if="$(eval &quot;'$(var sensor_model)' == 'your_sensor'&quot;)">

  <!-- Launch driver -->
  <include file="$(find-pkg-share your_driver)/launch/your_sensor.launch.py">
    <arg name="config" value="$(var sensor_model_param_path)/your_sensor.param.yaml"/>
  </include>

  <!-- Remap to Autoware standard -->
  <remap from="/<driver_topic>" to="/sensing/<type>/<name>/<data>"/>

</group>
```

### 6. 建立參數檔案

**檔案**：`src/param/autoware_individual_params/.../your_sensor.param.yaml`

```yaml
/**:
  ros__parameters:
    frame_id: "your_sensor_link"
    # Add sensor-specific parameters
```

### 7. 加入啟動引數

**檔案**：`src/launcher/autosdv_launch/launch/autosdv.launch.yaml`

```python
sensor_model_param = DeclareLaunchArgument(
    'sensor_model',
    choices=['existing', 'models', 'your_sensor'],  # Add your sensor
)
```

### 8. 測試整合

```bash
# Rebuild
make build

# Launch with your sensor
make launch ARGS="sensor_model:=your_sensor"

# Verify topic in Autoware namespace
ros2 topic list | grep /sensing
ros2 topic hz /sensing/<type>/<name>/<data>

# Verify TF transform
ros2 run tf2_tools view_frames
evince frames.pdf  # Check your_sensor_link exists

# Check transform values
ros2 run tf2_ros tf2_echo sensor_kit_base_link your_sensor_link
```

✅ **檢查點**：感測器在完整的 AutoSDV 系統中運作，資料流正確

## 常見問題

**RViz 中方向錯誤**：
- 驗證校正旋轉值（roll、pitch、yaw）
- 檢查感測器座標系統與 ROS 標準
- 參閱 Robin-W 範例的座標轉換

**找不到主題**：
- 檢查啟動檔案中的 `<remap>`
- 驗證驅動程式主題名稱匹配
- 使用絕對路徑：`/sensing/...`

**TF 轉換遺失**：
- URDF 變更後重新建置：`make build`
- 檢查 URDF 語法：`check_urdf sensor_kit.urdf`

**無資料發布**：
- 先獨立測試驅動程式
- 檢查網路/USB 連接
- 驗證驅動程式參數

## 下一步

- **感測器特定指南**：[LiDAR](./lidar.md)、[相機](./camera.md)、[IMU](./imu.md)、[GNSS](./gnss.md)
- **疑難排解**：檢查 `play_log/latest/` 中的日誌，啟用除錯記錄
- **文件**：為未來參考記錄您的感測器配置
