<!--
Translation Metadata:
- Source file: cuda-pipeline.md
- Last synced: 2026-09-12
- Translator: Claude (Anthropic)
- Status: Complete
-->

# CUDA 點雲管線

點雲前處理可以在 GPU 上執行，而不是 CPU。兩個整段式開關，兩者都預設為 `cpu`，
再加上掃描匹配器：

```bash
play_launch launch autosdv_launch autosdv.launch.yaml \
  pointcloud_backend:=cuda \
  localization_pointcloud_backend:=cuda \
  pose_source:=cuda_ndt
```

| 開關 | 決定哪一段的後端 |
|------|-----------------|
| `pointcloud_backend` | 感測端：自身裁切、去畸變、環狀離群濾除 |
| `localization_pointcloud_backend` | NDT 輸入鏈：裁切盒、體素網格、隨機降採樣 |
| `pose_source` | 掃描匹配器本身 |

它們彼此獨立。你可以只把感測端搬到 GPU 而讓定位留在 CPU，反之亦然。

## 哪些光達適用

**`pointcloud_backend:=cuda` 適用於 `vlp32c` 與 `robin-w`，不適用 `cube1`。**

原因在於去畸變。要移除掃描中的運動失真，需要**每個點的時間偏移量**，而不是每個
驅動程式都提供：

| 光達 | 驅動程式 | 每點時間 | CUDA 感測端 |
|------|---------|---------|------------|
| `vlp32c` | Nebula | 有——`PointXYZIRCAEDT` | 支援 |
| `robin-w` | `seyond_ros_driver`，自 `autosdv-1.5.0` 起 | 有 | 支援 |
| `cube1` | Blickfeld | **沒有** | 拒絕，並回報指出原因的錯誤 |

Robin-W 另有一個建置期的條件值得知道：驅動程式必須以預設的 `POINT_TYPE` 建置。
以 `PointXYZIRC` 建置的驅動程式所發佈的點雲會被 CUDA 前處理器拒絕，而這個失敗
發生在執行期而非建置期。

`cube1` 是被明確拒絕的，而不是靜默地產生錯誤結果。

## 所有東西都必須載入同一個容器

這是最容易讓人意外的限制。

`cuda_blackboard` **不是一種傳輸機制**。它是一個行程內的對映表，把一個 id 對應
到一個裝置指標。某一段會把那個*id* 往下游傳，而下一段會在它自己的行程中查找該
id。如果下一段位於另一個行程，它收到 id 之後會發現後面什麼都沒有。

因此每個 CUDA 階段都必須載入同一個 component container。這正是這些開關是整段式
的原因：只套用一半的後端不是比較慢的設定，而是壞掉的設定。

## AutoSDV 必須自行補上的部分

Autoware 提供了這條鏈大部分的 CUDA 實作，但不是全部。有兩個濾波器位於本儲存庫：

- 獨立的 CUDA 裁切盒
- CUDA 隨機降採樣

它們位於 `src/sensing/cuda_pointcloud_filters`。找不到 CUDA 工具鏈時，該套件會
自行略過，所以沒有 CUDA 的機器仍能建置整個工作空間。

## 它比較快嗎？

- `pointcloud_backend:=cuda` —— 量測結果在儲存庫的
  `docs/design/cuda-pipeline-data-flow.md`。
- `localization_pointcloud_backend:=cuda` —— 正確性已驗證，**速度尚未量測**。
  誠實的做法是把它當成未經證實，而不是當成一項最佳化。
- `pose_source:=cuda_ndt` —— 比 CPU NDT 快 1.3–1.6 倍，在 Jetson 上少用 57% 的
  CPU。這是效益最明確的一項，而且它已經是預設值。

在車輛上，節省 CPU 往往比降低延遲更重要：同一顆 CPU 還要跑感知、規劃與控制。

## 相關頁面

- [定位方法](./localization-methods.md)
- [操作車輛](../getting-started/usage.md)
- 儲存庫中的 `docs/design/cuda-pipeline-data-flow.md`
