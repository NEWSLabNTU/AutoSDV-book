<!--
Translation Metadata:
- Source file: integration-walkthrough.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 整合範例：Robin-W 光達

本指南將使用 **Seyond Robin-W 光達**作為具體範例，逐步說明如何將感測器整合到 AutoSDV。完成後，您將了解任何感測器從硬體到軟體的整合方式。

## 實體感測器

**Seyond Robin-W** 是一款固態光達感測器：
- 前向（120° × 25° 視野）
- 範圍：最高 200 公尺
- 安裝於車輛前方安裝座
- 透過 Ethernet 連接（IP：172.168.1.10）
- 由車輛電源供應 12-24V DC

當感測器運作時，它會以約 10 Hz 的頻率透過網路串流 3D 點雲資料。

## 驅動程式套件

驅動程式將感測器的原始資料轉換為 ROS 2 訊息。

**位置**：`src/sensor_component/external/seyond_ros_driver/`

這是一個 git 子模組（外部套件），它會：
1. 透過網路連接到 Robin-W（172.168.1.10）
2. 接收點雲資料
3. 發布到 ROS 主題：`/robin_lidar/points_raw`

驅動程式輸出 **PointXYZIRC** 格式（與 Autoware 相容）：
- `x, y, z` - 3D 位置
- `intensity` - 反射強度
- `return_type` - 第一次/最後一次回波
- `ring` - 雷射通道 ID

## 如何配置

要將 Robin-W 整合到車輛中，我們需要告訴 AutoSDV：
1. 感測器實際安裝的位置
2. 如何啟動驅動程式
3. 要使用什麼網路設定

### 實體位置

**檔案**：`src/sensor_kit/autosdv_sensor_kit_description/config/sensor_kit_calibration.yaml`

```yaml
sensor_kit_base_link:
  robin_lidar_link:
    x: 0.0        # Position: at front center
    y: 0.0
    z: 0.15       # 15cm above sensor kit base
    roll: 3.14159   # 180° rotation (see explanation below)
    pitch: -1.5708  # -90° rotation
    yaw: 0.0
```

這定義了一個稱為 `robin_lidar_link` 的**座標框架**，代表感測器在車輛上的位置和方向。

<span id="why-the-strange-rotation"></span>
#### 為什麼會有奇怪的旋轉？

Robin-W 使用的座標軸與 ROS 標準不同：

**Robin-W 原生座標**：
```
  X = Up
  Y = Right
  Z = Forward
```

**ROS 標準（REP-103）**：
```
  X = Forward
  Y = Left
  Z = Up
```

要將 Robin-W 座標 → ROS 標準座標：
1. **Roll 180°**：將感測器上下翻轉
2. **Pitch -90°**：旋轉使前向軸對齊

沒有這個轉換，點雲在 RViz 中會顯示為上下顛倒並旋轉！

### 機器人描述（URDF）

**檔案**：`src/sensor_kit/autosdv_sensor_kit_description/urdf/sensor_kit.xacro`

```xml
<!-- Robin-W LiDAR -->
<link name="robin_lidar_link"/>

<joint name="robin_lidar_joint" type="fixed">
  <origin
    xyz="${calibration['sensor_kit_base_link']['robin_lidar_link']['x']} ..."
    rpy="${calibration['sensor_kit_base_link']['robin_lidar_link']['roll']} ..."
  />
  <parent link="sensor_kit_base_link"/>
  <child link="robin_lidar_link"/>
</joint>
```

這會使用上述校正值在機器人的 TF 樹中建立 `robin_lidar_link` 框架。

### 驅動程式啟動配置

**檔案**：`src/sensor_kit/autosdv_sensor_kit_launch/launch/lidar.launch.xml`

```xml
<group if="$(eval &quot;'$(var lidar_model)' == 'robin-w'&quot;)">

  <!-- Launch Robin-W driver -->
  <include file="$(find-pkg-share seyond_ros_driver)/launch/robin_w.launch.py">
    <arg name="config" value="$(var sensor_model_param_path)/robin_lidar.param.yaml"/>
  </include>

  <!-- Remap topic to Autoware standard -->
  <remap from="/robin_lidar/points_raw"
         to="/sensing/lidar/robin_lidar/points_raw"/>

</group>
```

這表示：「當使用者選擇 `lidar_model:=robin-w` 時，啟動 Robin-W 驅動程式並將其主題重新映射到標準 Autoware 命名空間。」

### 驅動程式參數

**檔案**：`src/param/autoware_individual_params/.../robin_lidar.param.yaml`

```yaml
/**:
  ros__parameters:
    ip_address: "172.168.1.10"    # Robin-W network address
    port: 2368                     # UDP port
    frame_id: "robin_lidar_link"   # TF frame name
    min_range: 0.5                 # Filter points closer than 0.5m
    max_range: 200.0               # Filter points farther than 200m
```

這些是啟動時傳遞給驅動程式的執行時參數。

## 資料如何流動

當您使用 `lidar_model:=robin-w` 啟動 AutoSDV 時：

```
1. Robin-W Hardware
   │ Streams point cloud over UDP
   ▼
2. seyond_ros_driver Node
   │ Converts to ROS PointCloud2 message
   │ Publishes to /robin_lidar/points_raw
   ▼
3. Topic Remapping
   │ Remaps to /sensing/lidar/robin_lidar/points_raw
   ▼
4. TF Transform
   │ Applies rotation (roll=180°, pitch=-90°)
   │ Transforms points: Robin-W coords → ROS coords
   ▼
5. Autoware Localization/Perception
   │ Uses point cloud for NDT localization
   │ Detects obstacles with CenterPoint
   ▼
6. Visualization (RViz)
   │ Shows correctly oriented point cloud
```

## 系統中的表現方式

### 主題命名空間

```bash
$ ros2 topic list | grep lidar
/sensing/lidar/robin_lidar/points_raw
```

所有感測器資料都遵循以下模式：`/sensing/[類型]/[名稱]/[資料]`

### TF 樹

```bash
$ ros2 run tf2_tools view_frames
```

這會產生一個 PDF 顯示：
```
base_link
  └─ sensor_kit_base_link
       └─ robin_lidar_link  ← Robin-W sensor frame
```

從 `sensor_kit_base_link` → `robin_lidar_link` 的轉換使用校正值（位置 + 旋轉）。

### 驗證

檢查 Robin-W 是否正常運作：

```bash
# Topic publishing at ~10 Hz?
ros2 topic hz /sensing/lidar/robin_lidar/points_raw

# Point cloud has data?
ros2 topic echo /sensing/lidar/robin_lidar/points_raw --once

# Transform exists?
ros2 run tf2_ros tf2_echo sensor_kit_base_link robin_lidar_link
```

## 整體架構

每個感測器整合都遵循此模式：

1. **實體層**
   - 將感測器安裝在車輛上
   - 連接電源和通訊（USB/Ethernet 等）

2. **驅動程式層**（`src/sensor_component/`）
   - 與感測器硬體通訊的 ROS 2 套件
   - 將原始感測器資料發布到主題

3. **感測器套件層**（`src/sensor_kit/`）
   - 校正：感測器在哪裡？（位置 + 旋轉）
   - URDF：將感測器加入機器人描述
   - 啟動：何時啟動驅動程式
   - 參數：執行時配置

4. **資料流**
   - 驅動程式發布 → 主題重新映射 → 套用 TF 轉換 → Autoware 使用資料

## 重點要點

- **座標轉換很重要**：Robin-W 的非標準座標需要 roll=180°, pitch=-90°
- **一切都是座標框架**：`robin_lidar_link` 代表 3D 空間中的感測器
- **主題重新映射標準化命名**：驅動程式輸出 → `/sensing/` 命名空間
- **雙層系統**：驅動程式（感測器元件）+ 整合（感測器套件）

## 下一步

現在您了解了 Robin-W 的整合方式：

- **嘗試其他感測器**：[光達](./lidar.md)、[相機](./camera.md)、[IMU](./imu.md)、[GNSS](./gnss.md)
- **新增感測器**：[新增感測器指南](./adding-sensor.md)
- **疑難排解**：檢查網路連接、驗證 TF 轉換、監控主題
