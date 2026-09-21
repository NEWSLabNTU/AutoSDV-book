<!--
Translation Metadata:
- Source file: sensor-capability-matrix.md
- Last synced: 2026-09-21
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 感測器能力對照表

每款支援感測器是什麼、它的驅動程式發布什麼，以及系統的哪些部分能消費它。本頁是查表
用的；各列背後的推理在[光達感測器](../../guides/sensor-integration/lidar.md)。

這裡每一列都是對照儲存庫而非規格書查核過的，並且註明了決定該答案的檔案。凡是樹中
無法定案的值，該格就直說，而不是用猜的。

## 光達：它是什麼

| | Seyond Robin-W | Velodyne VLP-32C | Blickfeld Cube1 |
|---|---|---|---|
| `lidar_model:=` | `robin-w` | `vlp32c` | `cube1` |
| 類型 | 固態 | 旋轉式，32 通道 | 固態 |
| 水平視野 | 120°（±60°，由 SDK 強制） | 360° | 70° |
| 垂直視野 | **未經查證**——25° 或 70° | 40° | 30° |
| 掃描線 | 192（`kRobinWScanlines_`） | 32 | — |
| 範圍 | 200 m | 配置為 300 m | 150 m |
| 位址 | 172.168.1.10 | 192.168.7.10（`sensor_ip`） | 192.168.26.26 |
| TF 座標系 | `robin_w` | `vlp32c` | `cube1` |

!!! warning "Robin-W 的垂直視野"

    水平的 120° 是由驅動程式強制的：Seyond SDK 的 `is_robin_inside_fov_point()`
    會丟棄 ±60° 之外的點。**垂直的數字在樹中任何地方都沒有定論。** 本書的光達頁面
    一直寫 25°，儲存庫中的 `docs/research/robin_w_fov.md` 寫 70°，SDK 根本沒有編入
    垂直上限（仰角表是執行期從裝置取得的），而儲存庫中也沒有任何 Robin-W 錄製資料
    可供量測。

    什麼能定案：一份 Robin-W 的 bag，加上 `scripts/sensor/inspect_rings.py`——它會
    從點雲自身的 `elevation` 欄位印出每個通道的仰角。在有人跑過之前，請不要再傳播
    這兩個數字中的任何一個。

    來源：`seyond_ros_driver` 子模組中的 `inno_lidar_packet_utils.h`；
    `sensor_kit_calibration.yaml`；`VLP32.param.yaml`；`cube1.param.yaml`。Cube1
    與 VLP-32C 的視野數字是從早期文件沿用下來的原廠規格，而不是樹所陳述的值。

## 光達：驅動程式發布什麼

| | Seyond Robin-W | Velodyne VLP-32C | Blickfeld Cube1 |
|---|---|---|---|
| 驅動程式 | `seyond`（子模組 `seyond_ros_driver`） | Nebula（`nebula_ros`，來自 Autoware） | `blickfeld_driver`（子模組） |
| 點格式 | `PointXYZIRCAEDT` | `PointXYZIRCAEDT` | Blickfeld 自有格式 |
| 每點時間 | 有（`time_stamp`，標頭之後的奈秒） | 有 | 沒有，就出貨配置而言 |
| `channel` 的意義 | `scan_id`——振鏡掃描位置 | 環索引，固定仰角 | — |
| 套件中的原始話題 | `/sensing/lidar/iv_points` | `/sensing/lidar/velodyne_points` | `/sensing/lidar/bf_lidar/points_raw` |
| 配置 | `seyond_robin_w.launch.xml` | `config/VLP32.param.yaml` | `config/cube1.param.yaml` |

原始話題就是 `pointcloud_preprocessor.launch.py` 中 `LIDAR_TOPICS` 的值，`lidar_model`
也是在那裡被解析的。無論裝的是哪顆感測器，前處理鏈都會在 `base_link` 中發布
`/sensing/lidar/concatenated/pointcloud`。

## 光達：系統能拿它做什麼

| | Seyond Robin-W | Velodyne VLP-32C | Blickfeld Cube1 |
|---|---|---|---|
| 3D NDT（`ndt`、`cuda_ndt`） | 可以 | 可以 | 可以 |
| `pointcloud_backend:=cuda` | 可以 | 可以 | **啟動時拒絕** |
| 是否可能去畸變 | 可以 | 可以 | 不行 |
| 2D MCL 掃描來源（`publish_scan`） | 不行——沒有等仰角的環 | 可以 | 不行——沒有環，而且 70° 太窄 |

三者都無法做多光達融合：這套套件只跑一顆光達，而且其鏈路以單一輸入的 passthrough
作結，而非串接器。

`cube1` 搭配 `pointcloud_backend:=cuda` 會在任何節點啟動前拋出 `ValueError`。該檢查
位於 `pointcloud_preprocessor.launch.py`，比對的是 `lidar_model` 與由 `vlp32c` 和
`robin-w` 組成的 `DESKEWABLE` 元組。

啟動期檢查看不到的兩種失效方式：

- 以 `POINT_TYPE=PointXYZIRC` 重新建置的 Seyond 驅動程式，發布的點雲會在**執行期**
  被 CUDA 前處理器拒絕。預設是 `PointXYZIRCAEDT`；除非你清楚自己為何要改，否則別動它。
- Blickfeld 驅動程式有一個 `publish_point_time_offset` 參數，在套件配置中為 `false`。
  打開它加上的是它自己的欄位，不是 CUDA 節點所消費的 `PointXYZIRCAEDT` 格式，所以
  並不會讓 `cube1` 變得可去畸變。

對固態感測器下 `publish_scan:=true` 同樣不會被拒絕。環擷取節點對任何型號都會執行，
產生一則形狀像掃描、但並非平面的訊息，因為固態點雲的 `channel` 欄位不是仰角。

## 感測器套組

`sensor_suite` 會一次設定全部五個選擇器。個別參數可覆寫套組所選的值。出自
`sensing.launch.xml`：

| `sensor_suite:=` | `lidar_model` | `camera_model` | `imu_source` | `gnss_receiver` | `use_gnss` | ZED 物件偵測 |
|---|---|---|---|---|---|---|
| `vlp32c_zed_imu` *（預設）* | `vlp32c` | `none` | `zed` | `ublox` | true | false |
| `vlp32c_zed` | `vlp32c` | `zedxm` | `zed` | `ublox` | true | true |
| `vlp32c_zed_mpu` | `vlp32c` | `zedxm` | `mpu9250` | `ublox` | true | true |
| `robin_zed` | `robin-w` | `zedxm` | `zed` | `ublox` | true | true |
| `robin_zed_mpu` | `robin-w` | `zedxm` | `mpu9250` | `ublox` | true | true |
| `cube1_usb` | `cube1` | `usb` | `mpu9250` | `ublox` | true | false |
| `custom` | `vlp32c` | `zedxm` | `mpu9250` | `garmin` | true | — |

預設套組是 `vlp32c_zed_imu`，所以預設光達是 VLP-32C——三者之中唯一能餵給 2D MCL 的
一顆，也是能被去畸變的兩顆之一。

```bash
play_launch launch autosdv_launch autosdv.launch.yaml sensor_suite:=robin_zed
```

```bash
just launch sensor_suite:=robin_zed
```

個別選擇器，供 `sensor_suite:=custom` 或覆寫單一值使用：

| 參數 | 可用值 |
|----------|--------|
| `lidar_model` | `robin-w`、`vlp32c`、`cube1` |
| `camera_model` | `zedxm`、`usb`、`none` |
| `imu_source` | `mpu9250`、`zed` |
| `gnss_receiver` | `ublox`、`septentrio`、`garmin` |

## 依需求選擇

- **端到端的 CUDA 感測與定位**：`vlp32c` 或 `robin-w`。
- **對佔據網格做 2D MCL**：`vlp32c`，或在固態光達旁加裝原生 2D 光達。
- **建圖所需的 360° 視野**，也就是迴圈閉合必須能運作時：`vlp32c`。
- **Cube1**：只有 CPU 感測路徑與 3D NDT。

## 相關頁面

- [光達感測器](../../guides/sensor-integration/lidar.md)——這些列為何長這樣
- [CUDA 點雲流程](../../guides/cuda-pipeline.md)
- [定位方法](../../guides/localization-methods.md)
- [核心組件](./core-components.md)

<!--
RECONCILE:
- nav: add "Sensor Capability Matrix" under Technical Reference > Hardware,
  between "Core Components" and "Wiring Diagrams":
      - Sensor Capability Matrix: reference/hardware/sensor-capability-matrix.md
  and a nav_translations entry: "Sensor Capability Matrix: 感測器能力對照表".
- cross-link from reference/overview.md "Core Components" list.
- cross-link from guides/sensor-integration/using-sensors.md, which lists the
  suites in prose and gets robin-w's defaults wrong.
- cross-link from guides/localization-methods.md section "Which sensors this
  applies to" -> this page's "LiDAR: what the stack can do with it".
- DE-DUPLICATE: this page and guides/cuda-pipeline.md both carry a deskew
  support table. Phase 2 should pick one home; the suggestion is that
  cuda-pipeline links here.
- platform-models.md repeats the unverified Robin-W 120° x 25° figure in three
  places and should be made consistent with the warning on this page.
-->
