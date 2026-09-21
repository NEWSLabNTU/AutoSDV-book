<!--
Translation Metadata:
- Source file: lidar.md
- Last synced: 2026-09-21
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 光達感測器

三款支援光達的配置方式，以及選用固態光達會在這套系統的其他地方付出什麼代價。

同一時間只跑一顆光達。由 `lidar_model` 選擇：

```bash
play_launch launch autosdv_launch autosdv.launch.yaml lidar_model:=vlp32c
```

```bash
just launch lidar_model:=vlp32c
```

## 支援的型號

| 型號 | 類型 | 視野 | 範圍 | 位址 | `lidar_model:=` | 套件中的話題 |
|-------|------|-----|-------|---------|-----------------|------------------|
| **Seyond Robin-W** | 固態 | 水平 120°；垂直**未經查證** | 200 m | 172.168.1.10 | `robin-w` | `/sensing/lidar/iv_points` |
| **Velodyne VLP-32C** | 旋轉式，32 通道 | 360° × 40° | 配置為 300 m | 192.168.7.10 | `vlp32c` | `/sensing/lidar/velodyne_points` |
| **Blickfeld Cube1** | 固態 | 70° × 30° | 150 m | 192.168.26.26 | `cube1` | `/sensing/lidar/bf_lidar/points_raw` |

話題那一欄的權威來源是 `pointcloud_preprocessor.launch.py`，`lidar_model` 就是在
那裡被解析成原始話題的。無法辨識的值會在該處被拒絕，並列出有效值。

彙整後的檢視——每點時間戳、CUDA 去畸變、感測器能否餵給 2D
MCL——請見[感測器能力對照表](../../reference/hardware/sensor-capability-matrix.md)。

!!! warning "Robin-W 的垂直視野未經查證"

    水平數字是確定的，而且確定它的是驅動程式而非規格書：Seyond SDK 的
    `is_robin_inside_fov_point()` 會丟棄 ±60° 之外的點。水平 120° 就是驅動程式
    強制執行的範圍。

    垂直數字則未有定論。本書一直寫 **25°**；儲存庫中的
    `docs/research/robin_w_fov.md` 寫 **70°**。樹中沒有任何東西能判定何者為真。
    SDK 對 Robin-W 根本沒有編入垂直上限——仰角表是執行期透過
    `inno_lidar_get_anglehv_table` 從裝置取得的——而儲存庫裡也沒有任何 Robin-W
    的錄製資料可供量測。

    下次 Robin-W 上工作台時，一道指令就能定案。驅動程式會發布每點的 `elevation`
    欄位，因此：

    ```bash
    python3 scripts/sensor/inspect_rings.py <bag> --topic /sensing/lidar/iv_points
    ```

    最低與最高通道仰角之間的跨距就是答案。在有人跑過之前，請把垂直範圍當成未知，
    而不是當成上述兩個已公布數字之一。

## 固態光達對這套系統的代價

Robin-W 與 Cube1 沒有活動零件，換來的是可靠度與更小的體積。這套系統中有三處是
圍繞旋轉式感測器的幾何特性建立的，代價就顯現在那裡。

### 受限的視野與掃描匹配

NDT——`pose_source:=ndt` 或預設的 `cuda_ndt`——是把每一幀掃描與預先建立的點雲地圖
做匹配。它是地圖匹配而非 SLAM，所以沒有迴圈閉合可以失去；狹窄的視野不會像 SLAM
文獻描述的那樣造成漂移。它付出的代價是**約束**。

一幀掃描只能在它含有結構的方向上約束位姿，而 120° 的前向錐體只在正前方含有結構。
當錐體中滿是幾何特徵時——走廊、一排停放的車輛、建築立面——估計的約束程度和旋轉式
光達一樣好。當錐體指向空曠地面時，匹配沒什麼可以咬合，估計就改為倚賴 EKF 由 IMU
與輪速里程計所做的預測。轉彎是最容易看出這件事的時刻，因為錐體會擺向上一幀掃描中
並不存在的結構。

實際落地的兩個後果：

- **調校更重要，觀察調校結果也更重要。** 儲存庫中的 `docs/guides/ndt-tuning.md`
  記錄了流程，`scripts/testing/localization/` 則提供即時診斷——`ndt_quality_report.py`
  看位姿品質，`ndt_alignment_report.py` 看掃描對地圖的殘差。
- **建圖才是比較難的那一半。** 由 SLAM 流程產生的 PCD *確實*需要迴圈閉合，而 240°
  的盲區在那裡是真的會痛。建圖流程請見 [地圖](../maps.md#點雲地圖從哪裡來)，以及
  [地圖](../maps.md)說明各定位方法各自消費什麼。

儲存庫中的 `docs/research/robin_w_fov.md` 走得更遠，勾勒了 UWB 信標、AprilTag
位姿圖、RTAB-Map 融合等架構，而這個專案從未建置過它們。請把它當成研究，而不是配置。

### 沒有等仰角的環，因此沒有 2D 掃描

[2D MCL](../localization-methods.md) 消費一則 `sensor_msgs/LaserScan`。感測器套件
是從 3D 點雲中保留一小群通道來產生這則掃描：

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  pose_source:=mcl publish_scan:=true scan_ring:=<measured>
```

這之所以可行，是因為旋轉式光達的通道索引*就是*仰角：第 N 通道描出一個固定俯仰角
的圓錐，而一小群相鄰通道非常接近一個平面。在 VLP-32C 上這個前提成立。

在 Robin-W 上則不成立。驅動程式的 `channel` 欄位帶的是 SDK 的 `scan_id`——振鏡掃描
過程中的位置，而該過程本身就在仰角上移動——不是固定的俯仰角，所以「第 N 通道」不是
水平面，也沒有任何環的選法能讓它變成水平面。Cube1 亦然，而且還多了 70° 的水平視野，
無論取出哪個平面，都難以在 360° 佔據網格上約束粒子濾波器。

對固態感測器下 `publish_scan:=true` 並不會被拒絕。啟動檔對任何型號都會派出環擷取
節點，而出來的是一則形狀像掃描、但並非平面的訊息。請讓這些感測器走 3D NDT 路徑，
或在旁邊加裝一顆原生 2D 光達。

`scan_input_topic` 預設指向 Autoware 範例 bag 的點雲，所以在這套套件上請把它設成
驅動程式實際發布的話題——VLP-32C 是 `/sensing/lidar/velodyne_points`。

<span id="deskewing-and-the-cuda-sensing-backend"></span>
### 去畸變與 CUDA 感測後端

畸變校正會依照掃描起點到該點被量測之間發生的運動，逐點位移。因此它需要**每點時間
偏移量**，而不是每個驅動程式都會發布。

| `lidar_model` | 驅動程式 | 每點時間 | `pointcloud_backend:=cuda` |
|---------------|--------|----------------|----------------------------|
| `vlp32c` | Nebula | 有——`PointXYZIRCAEDT` | 支援 |
| `robin-w` | `seyond`，本儲存庫所固定的版本 | 有——`PointXYZIRCAEDT` | 支援 |
| `cube1` | Blickfeld | 沒有 | **啟動時拒絕** |

`cube1` 搭配 `pointcloud_backend:=cuda` 會在任何節點啟動前拋出 `ValueError`，說明
原因並要你改用 `pointcloud_backend:=cpu`。這是刻意的：另一個選項是一份安靜地沒有
被去畸變的點雲。

!!! note "會失去 Robin-W 去畸變的兩種方式"

    Seyond 驅動程式的點格式是建置期的選擇，也就是其 `CMakeLists.txt` 中的
    `POINT_TYPE`，預設為 `PointXYZIRCAEDT`。若重新以 `PointXYZIRC` 建置，驅動程式
    發布的點雲會在執行期被 CUDA 前處理器拒絕，而非被去畸變——而且啟動期的檢查抓不到
    它，因為那個檢查看的是 `lidar_model`，不是點雲本身。

    對稱地，Blickfeld 驅動程式確實有一個 `publish_point_time_offset` 參數，在套件的
    `cube1.param.yaml` 中設為 `false`。把它打開並不會讓 `cube1` 可被去畸變：它加上的
    是自己的欄位，不是 CUDA 節點所消費的 `PointXYZIRCAEDT` 格式，而且拒絕與否本來就
    只看型號。

CUDA 故事的其餘部分——三個後端開關、為何每個階段都必須載入同一個容器——請見
[CUDA 點雲流程](../cuda-pipeline.md)。

### 為什麼沒有串接器

Autoware 的感測鏈通常以一個串接器（concatenator）作結，把數顆光達合併成一份點雲。
這套套件沒有用它，也不能用：CPU 版與 CUDA 版的串接器都拒絕單一輸入話題。

```
Component constructor threw an exception:
Only one topic given. Need at least two topics to continue.
```

把同一個話題列兩次確實能載入，然後會遺失大約 80% 的幀——每則訊息填滿一個槽位，
收集器等滿 `timeout_sec` 也等不到第二則，收集器上限於是反覆翻騰。

因此套件改以一個 passthrough 節點作結，它同時會把點雲轉換到 `base_link`。如果你去
找 CUDA 前處理器的輸出，最後這點很重要：它留下的點雲位於*感測器*座標系，是
passthrough 把它搬過去的。

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

**必要的校正**，位於
`src/param/autoware_individual_params/individual_params/config/default/autosdv_sensor_kit/sensor_kit_calibration.yaml`：

```yaml
sensor_kit_base_link:
  robin_w:           # the frame is robin_w, not robin_lidar_link
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 3.14159    # 180° flip
    pitch: -1.5708   # -90° rotation
    yaw: 0.0
```

詳細說明請參閱[整合範例](./integration-walkthrough.md)。

### 驅動程式套件

**子模組**：`src/sensor_component/external/seyond_ros_driver/`
**ROS 套件名稱**：`seyond`
**點格式**：預設為 `PointXYZIRCAEDT`——變更 `POINT_TYPE` 前請先看上面的去畸變說明。

### 獨立測試

```bash
colcon build --base-paths src --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select seyond
source install/setup.bash
ros2 launch autosdv_sensor_kit_launch seyond_robin_w.launch.xml

# Verify -- outside the kit's namespace the topic is /iv_points
ros2 topic hz /iv_points   # ~10 Hz
```

在完整啟動流程中，同一份點雲會以 `/sensing/lidar/iv_points` 出現，因為套件在驅動
程式外圍推入了 `sensing/lidar` 命名空間。

### 疑難排解

**無資料**：檢查 `ping 172.168.1.10` 是否成功
**方向錯誤**：驗證校正中的 roll=3.14159, pitch=-1.5708
**`pointcloud_backend:=cuda` 在執行期拒絕點雲**：驅動程式是以 `PointXYZIRC` 建置的；
請以預設的 `POINT_TYPE` 重新建置

## Velodyne VLP-32C

### 網路設定

驅動程式實際使用的位址是套件 `VLP32.param.yaml` 中的 `sensor_ip`，也就是
**192.168.7.10**，而不是出廠預設值。請把主機放在同一子網路：

```bash
# Configure Jetson
sudo ip addr add 192.168.7.100/24 dev eth0
sudo ip link set eth0 up

# Test connectivity
ping 192.168.7.10
```

如果你的裝置仍在出廠位址上，請改感測器的位址，或改參數檔中的 `sensor_ip`——這個值
只存在於這兩個地方。

### 座標系統

標準 ROS 座標——沒有 roll 也沒有 pitch。出貨的校正確實帶有一個 **yaw**，而且它不是
隨便填的值：

```yaml
sensor_kit_base_link:
  vlp32c:            # the frame is vlp32c, not velodyne_link
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: -0.2210     # -12.66 deg, derived from replay, not measured
```

這個 yaw 是從回放中還原出來的：當 `yaw: 0.0` 時，定位出來的航向在直線路段上與車輛
自身的行進方向差了 12.66°，而汽車不可能這樣。修正之後差距降到 0.07°，NDT 的每幀
修正量也從 0.127 m 降到 0.049 m。x/y/z 仍未經量測——車輛可用時請把四者一併換成實測
外參。推導過程在儲存庫的 `docs/reports/cuda-ndt-coss-replay.md`。

### 驅動程式套件

**驅動程式**：Nebula（`nebula_ros`），隨 Autoware 一起安裝——本儲存庫中沒有 Velodyne
驅動子模組，也沒有需要 `apt install` 的東西。
**配置**：
`src/sensor_kit/autosdv_sensor_kit_launch/autosdv_sensor_kit_launch/config/VLP32.param.yaml`
**點格式**：`PointXYZIRCAEDT`，它帶有讓 `pointcloud_backend:=cuda` 得以成立的每點
時間戳。

### 雙回波模式

雙回波——同時取最強與最後一次回波，在雨霧中有幫助——已經是配置好的預設值，連同
600 rpm：

```yaml
/**:
  ros__parameters:
    sensor_model: VLP32
    rotation_speed: 600
    return_mode: Dual
    min_range: 0.3
    max_range: 300.0
    frame_id: vlp32c
```

### 獨立測試

```bash
ros2 launch nebula_ros velodyne_launch_all_hw.xml \
  sensor_model:=VLP32 \
  config_file:=$(ros2 pkg prefix --share autosdv_sensor_kit_launch)/config/VLP32.param.yaml

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
sensor_kit_base_link:
  cube1:             # the frame is cube1, not bf_lidar_link
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
```

### 驅動程式套件

**位置**：`src/sensor_component/external/ros2_blickfeld_driver_src-v1.5.5/`
**需要**：Blickfeld Scanner Library 2.20.6-newslab1（透過 `./setup.sh blickfeld` 安裝）
**配置**：
`src/sensor_kit/autosdv_sensor_kit_launch/autosdv_sensor_kit_launch/config/cube1.param.yaml`

這是唯一無法使用 `pointcloud_backend:=cuda` 的型號；請見[去畸變與 CUDA
感測後端](#deskewing-and-the-cuda-sensing-backend)。

### 獨立測試

```bash
colcon build --base-paths src --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select blickfeld_driver
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

**EULA 錯誤**：執行 `./setup.sh blickfeld` 以接受授權

## 多光達設定

**這套套件只跑一顆光達。** 第二顆感測器可以裝上並校正——`sensor_kit_calibration.yaml`
你給幾個連結它就收幾個：

```yaml
# sensor_kit_calibration.yaml
sensor_kit_base_link:
  robin_w:             # Front-facing
    x: 0.15
    z: 0.15
    roll: 3.14159
    pitch: -1.5708

  vlp32c:              # Top 360°
    x: 0.0
    z: 0.30
    roll: 0.0
    pitch: 0.0
```

——但前處理鏈不會把兩份點雲合併。`lidar_model` 只會選出唯一一個原始話題，而該鏈以
單一輸入的 passthrough 作結，而非串接器，理由如上。要融合第二顆光達，就必須重新引入
串接器，而這套套件還沒做這件事。

如果有人真的做了，收穫不只是覆蓋範圍：有了兩個輸入，CUDA 串接器就變成可用，感測鏈
可以從頭到尾留在 GPU 上，不必在 passthrough 處付出一次裝置到主機的複製。

## 快速參考

```bash
# Network tests
ping 172.168.1.10    # Robin-W
ping 192.168.7.10    # Velodyne, at the configured sensor_ip
ping 192.168.26.26   # Blickfeld

# Topic verification
ros2 topic list | grep /sensing/lidar
ros2 topic hz /sensing/lidar/iv_points            # Robin-W
ros2 topic hz /sensing/lidar/velodyne_points      # Velodyne
ros2 topic hz /sensing/lidar/bf_lidar/points_raw  # Blickfeld

# What the preprocessing chain publishes, whichever sensor is fitted
ros2 topic hz /sensing/lidar/concatenated/pointcloud

# TF verification -- the frames are robin_w, vlp32c, cube1
ros2 run tf2_ros tf2_echo sensor_kit_base_link robin_w

# Network monitoring
sudo iftop -i eth0
```

## 相關頁面

- [感測器能力對照表](../../reference/hardware/sensor-capability-matrix.md)——所有感測器事實集中於一張表
- [CUDA 點雲流程](../cuda-pipeline.md)——三個後端開關
- [定位方法](../localization-methods.md)——哪種感測器適合哪種 `pose_source`
- [感測器疑難排解](./troubleshooting.md)

<!--
RECONCILE:
- nav: no new entry needed for this page; it is already navigated.
- cross-link FROM guides/localization-methods.md section "Which sensors this
  applies to" TO this page's "No constant-elevation ring, so no 2-D scan"
  (guides/sensor-integration/lidar.md#no-constant-elevation-ring-so-no-2-d-scan),
  and consider replacing that section's prose with the link, since this page now
  gives the mechanism (channel = scan_id, not elevation).
- cross-link FROM guides/cuda-pipeline.md section "Which LiDARs qualify" TO
  "Deskewing, and the CUDA sensing backend" here; the two POINT_TYPE /
  publish_point_time_offset failure modes are documented only here.
- DE-DUPLICATE with guides/cuda-pipeline.md: both pages now carry a per-model
  deskew table. Phase 2 should keep one (suggest: cuda-pipeline keeps the
  backend-switch table, this page keeps the per-sensor one) and link the other.
- When W3's guides/ndt-tuning.md lands, link it from "A restricted field of
  view, and scan matching", which currently cites the repository path instead.
- CORRECTIONS this page made that other pages still carry the old value of:
  - Robin-W topic is /sensing/lidar/iv_points, NOT /robin_lidar/points_raw
    (stale in guides/sensor-integration/troubleshooting.md).
  - Velodyne sensor_ip is 192.168.7.10, NOT 192.168.1.201 (stale in
    reference/hardware/core-components.md power table context and elsewhere).
  - There is no Velodyne driver submodule and no ros-humble-velodyne package;
    the driver is Nebula.
  - The calibration frames are robin_w / vlp32c / cube1. The names
    robin_lidar_link, velodyne_link and bf_lidar_link appear in several pages
    and in NO file in the tree; they should be corrected wherever they occur
    (guides/sensor-integration/integration-walkthrough.md,
    guides/sensor-integration/troubleshooting.md).
  - guides/sensor-integration/using-sensors.md calls robin-w a "360° LiDAR",
    which is wrong in both axes, and calls it the default. The default
    sensor_suite is vlp32c_zed_imu, so the default lidar_model is vlp32c.
  - platform-models.md repeats the unverified 120° × 25° figure; it should
    carry the same "vertical unverified" caveat as this page.
-->
