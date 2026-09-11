<!--
Translation Metadata:
- Source file: presets.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 預設組態

預設組態（preset）是一組具名的啟動參數預設值。AutoSDV 沿用 Autoware 的做法：
把總是一起變動的設定歸成一組，讓你選擇一個情境，而不必記住六個參數。

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  perception_preset:=camera_lidar_fusion
```

## 運作方式

預設組態檔就是一個只宣告參數與其預設值的啟動檔：

```yaml
# config/perception/preset/lidar_only_preset.yaml
launch:
  - arg:
      name: perception_mode
      default: "lidar"
  - arg:
      name: use_traffic_light_recognition
      default: "false"
  - arg:
      name: use_detection_by_tracker
      default: "false"
  - arg:
      name: use_image_segmentation_based_filter
      default: "false"
  - arg:
      name: use_pointcloud_map
      default: "true"
```

主啟動檔以名稱包含其中一個：

```yaml
- arg:
    name: perception_preset
    default: "lidar_only"

- include:
    file: "$(find-pkg-share autosdv_launch)/config/perception/preset/$(var perception_preset)_preset.yaml"
```

其機制在於：預設組態提供的是**預設值**，而預設值只在沒有其他來源提供該值時才
生效。因此你在命令列上傳的參數會勝過預設組態：

```bash
# 使用融合預設組態，但關閉交通號誌辨識
play_launch launch autosdv_launch autosdv.launch.yaml \
  perception_preset:=camera_lidar_fusion \
  use_traffic_light_recognition:=false
```

整個設計就是這樣：預設組態求方便，個別參數供實驗。

## 感知預設組態

`config/perception/preset/`

| 預設組態 | `perception_mode` | 交通號誌 | 追蹤器輔助偵測 | 影像分割濾除 |
|---------|-------------------|---------|---------------|-------------|
| `lidar_only`（預設） | `lidar` | 關 | 關 | 關 |
| `camera_lidar_fusion` | `camera_lidar_fusion` | 開 | 開 | 開 |
| `minimal` | — | 關 | 關 | 關 |

- **`lidar_only`** —— 預設值。不含任何相機功能，所以沒有相機也不會缺少相依項。
- **`camera_lidar_fusion`** —— 需要可運作的相機。它也會替 TensorRT 必須編譯的
  模型集合再加上三個交通號誌模型，這點值得在一台全新機器首次啟動前先知道。
- **`minimal`** —— 用於開發與除錯，當你要讓堆疊跑起來、但不想讓感知模組礙事時。

若要完全停用感知，別用預設組態——請用參數：

```bash
play_launch launch autosdv_launch autosdv.launch.yaml launch_perception:=false
```

這會發佈空的物件清單，且完全不載入任何模型。

## 定位預設組態

`config/localization/preset/`

| 預設組態 | `twist_source` | 需要 |
|---------|----------------|------|
| `default` | `gyro_odom` | 無額外需求 |
| `eagleye` | `eagleye` | GNSS |

這些選擇的是**twist**（速度）估測器，與 `pose_source` 是兩個不同的問題。
`default` 融合陀螺儀與輪速里程計；`eagleye` 由 GNSS 推導里程計，需要一台能取得
可用定位的接收器。

```bash
play_launch launch autosdv_launch autosdv.launch.yaml localization_preset:=eagleye
```

## 自訂預設組態

1. 複製一個既有的預設組態：

   ```bash
   cd src/launcher/autosdv_launch/config/perception/preset
   cp lidar_only_preset.yaml custom_preset.yaml
   ```

2. 在新檔案中修改預設值。

3. 使用它：

   ```bash
   play_launch launch autosdv_launch autosdv.launch.yaml perception_preset:=custom
   ```

兩個要求：

- **檔名必須是 `<name>_preset.yaml`。** include 會內插
  `$(var perception_preset)_preset.yaml`，所以這個後綴不是慣例，而是查找方式。
- **新檔案需要 `just build`** 來建立符號連結，即使工作空間是以
  `--symlink-install` 建置的。修改*既有*預設組態會立即生效；新增的則在安裝前
  並不存在。

## 檢查預設組態解析成了什麼

預設組態會讓實際生效的設定變得比較不明顯，所以請驗證而不要臆測：

```bash
play_launch resolve autosdv_launch autosdv.launch.yaml \
  perception_preset:=camera_lidar_fusion -o ./tmp/resolved.yaml
```

當某個預設組態改變了堆疊實際載入哪些模型時，這也是重新推導
`just build-engines` 所編譯模型集合的方法。

## 相關頁面

- [操作車輛](../getting-started/usage.md) —— 完整的參數清單
- 儲存庫中的 `config/{perception,localization}/preset/README.md`
