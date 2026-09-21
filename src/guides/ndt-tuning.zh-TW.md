<!--
Translation Metadata:
- Source file: ndt-tuning.md
- Last synced: 2026-09-21
- Translator: Claude (Anthropic)
- Status: Complete
-->

# NDT 調校

NDT 是 `pose_source:=ndt` 與 `pose_source:=cuda_ndt` 背後的掃描匹配器。它接收一個
位姿先驗、一份 LiDAR 掃描與一張點雲地圖，回傳修正後的位姿。本頁談的是當那個位姿
錯了要改什麼、依什麼順序檢查——以及更常見的情況：如何確認 NDT 裡根本沒有東西需要改。

## NDT 通常是最後才出錯的那個

匹配器位於一條鏈的末端：TF、IMU、twist、EKF、點雲前處理、地圖。鏈上較早的環節
*安靜地*壞掉時，它會*大聲地*失敗，所以責任都算到它頭上。

促成本頁的那次失敗看起來完全像調校問題。NDT 分數很差、拒絕多數影格，最後停止發布。
真正的原因是一條缺失的 TF：logging simulation 中沒有任何節點發布 IMU 的轉換，所以
EKF 從未推進，NDT 每一幀拿到的都是同一個過期的先驗。任何人第一個會伸手去動的參數
——解析度、收斂門檻、crop box——都會被調成去補償那件事，而每一個在 TF 修好之後都會
讓系統更糟。

所以規則是：**先證明整條鏈，再調匹配器。** 以下章節刻意依此順序排列。

## 先取得可重複的重播

如果每次執行無法互相比較，參數研究就毫無價值。固定錄製檔、地圖、初始位姿與節點集合，
然後每次只改一件事。

COSS 錄製檔是本專案調校時所用的固定素材。它需要兩個終端機，而這正是重點：整個系統
跑在模擬時間上，在有 bag 驅動它的時鐘之前什麼都不會發生。

```bash
# 終端機 1 —— 系統本身，等待時鐘
play_launch launch --web-addr 0.0.0.0:8081 \
  autosdv_launch logging_simulation.launch.yaml \
  map_path:="$PWD/data/COSS-map-planning" \
  pose_source:=ndt launch_perception:=false
```

```bash
# 終端機 2 —— 錄製檔，驅動那個時鐘
ros2 bag play data/rosbags/outdoor_20251226_153115 --clock
```

同樣這兩道命令的捷徑：

```bash
just coss logging-sim        # cpu 路徑（pose_source:=ndt）；用 `gpu` 走 cuda_ndt
just coss play-rosbag        # --clock 不是選配，而這個 recipe 知道這件事
```

若錄製檔尚未下載，`just coss download-rosbag` 會取得它（下載約 1.6 GB，解開後 2.8 GB）。

有三件事讓連續多次執行可以互相比較，而三件都很容易被略過：

- **刻意設定初始位姿。** 不要交給 GNSS。在這份錄製檔上，GNSS 自動初始化每次落點相差
  約 5 m，那個散布會出現在結果裡，看起來像是參數差異。擷取一次位姿再重放它——見
  [定位診斷](./localization-diagnostics.md#讓執行不需要人介入的位姿種子)。
- **關掉你不在研究的東西。** `launch_perception:=false` 移除 TensorRT 引擎編譯，連帶
  移除 GPU 爭用。
- **用 release 建置。** `-DCMAKE_BUILD_TYPE=Release` 到不了 Rust 套件，而
  `cuda_ndt_matcher` 是 Rust 寫的。`just build` 正是為此傳入 `--cargo-args
  --release`；debug 版匹配器大約慢一個數量級並且會丟掉掃描，那看起來像調校問題，但不是。

!!! note "這些命令需要已建置的工作空間與那份錄製檔"

    撰寫本頁時並未實際執行它們。以下的參數名稱、預設值與檔案路徑*確實*是從程式樹讀出來的。

## 你正在編輯哪個檔案

兩個 pose source 不共用參數檔，而且數值不同。改錯檔案是「什麼都沒改到」最常見的方式。

| `pose_source` | 參數檔 |
|---|---|
| `ndt` | `src/launcher/autosdv_launch/config/localization/ndt_scan_matcher/ndt_scan_matcher.param.yaml` |
| `cuda_ndt` | `src/localization/cuda_ndt_matcher/src/cuda_ndt_matcher_launch/config/cuda_scan_matcher.param.yaml` |

輸入前處理鏈——crop box、voxel grid、random downsample——以同樣方式分開，位於上述各檔
旁邊的 `pointcloud_preprocessor/` 目錄下。

有了 `--symlink-install`，編輯這些 YAML 會在下次啟動時生效，不需重新建置。這也表示你
編輯的檔案*就是*載入的檔案——這值得確認而非假設，因為本儲存庫裡存在多個同名檔案：

```bash
ros2 param get /localization/pose_estimator/ndt_scan_matcher ndt.resolution
ros2 param get /localization/util/crop_box_filter_measurement_range max_x
```

問正在執行的節點。那是唯一真正關於你眼前這套系統的答案。

## 該看什麼，依什麼順序

### 1. 匹配器有被啟用嗎？

不是「行程還活著嗎」——是啟用（activated）。`ndt_scan_matcher` 有一個 `is_activated_`
閂鎖，只有 trigger service 會寫入它；節點不會自行啟用。若初始化在抵達那個服務之前丟出
例外——缺少 `map` 到 `pose.frame_id` 的 TF、地圖尚未載入、或還沒有掃描被接受——NDT 就會
被鎖在關閉狀態並維持下去。沒有任何東西會重試。此時 EKF 繼續以 IMU 與輪速推算，
`/localization/kinematic_state` 照樣以 40 Hz 運行，車輛也照樣在地圖上移動。這幅畫面裡
沒有任何東西說「壞了」。

```bash
python3 scripts/testing/localization/check_ndt_activated.py
```

離開碼 0 代表已啟用。若不是，就停在這裡：改任何參數都沒有意義。

### 2. 先驗鏈還活著嗎？

NDT 不是無中生有估出位姿——它修正 EKF 交給它的先驗。先驗過期時，匹配器做什麼都救不了。

```bash
ros2 topic hz /sensing/imu/imu_data
ros2 topic hz /localization/twist_estimator/twist_with_covariance
ros2 topic hz /localization/kinematic_state
ros2 run tf2_ros tf2_echo base_link <sensor_frame>
```

這裡出現零頻率的主題就是 bug。無法解析的 TF 也是。

去讀*其他*節點的 log，不只是匹配器的——`play_log/latest/node/*/err`。COSS 那次失敗的
自白是 `imu_corrector` 印出的數千行 `Please publish TF base_link to zedxm_imu_link`，
那是沒人在看的節點，而 `ndt_scan_matcher` 只是回報分數偏低。

先驗實際建立在其上的那兩個訊號有自己的檢查：

```bash
python3 scripts/testing/localization/check_imu_velocity.py
```

它針對這兩個訊號會出錯的三種方式做檢查：軸向或正負號錯誤、尺度誤差，以及發布率下陷或
有缺口。

### 3. 位姿好不好——用不依賴 NDT 的方式判斷

NDT 自己的分數無法告訴你 NDT 是不是對的。要用它不依賴的參照：

| 問題 | 獨立參照 |
|---|---|
| yaw 的*轉動*正確嗎？ | 在同一段時間窗內積分陀螺儀 |
| yaw 的*指向*正確嗎？ | 對地航跡角，且僅限直線路段 |
| 距離對嗎？ | 輪速里程，以及地圖本身的尺度 |
| 位姿吻合真實世界嗎？ | 掃描對地圖的最近鄰距離 |
| 地圖在遠處可信嗎？ | 從多個位姿觀察同一個地圖網格 |

其中兩項已寫成腳本：

```bash
python3 scripts/testing/localization/ndt_quality_report.py    # 散布、yaw 步進、init-to-result
python3 scripts/testing/localization/ndt_alignment_report.py  # 掃描對地圖殘差
```

### 4. 到這裡才輪到參數

每次執行只改一個變數。當兩個候選原因互相混淆時，一個 2x2 只多花兩次執行就能定案——在這張
地圖上，把解析度與裁切範圍交叉之後，單看其中任一項所得到的結論被推翻了。

## 這些數字的意義

| 數字 | 它是什麼 | 健康時 |
|---|---|---|
| 已發布位姿數 | 通過收斂門檻的影格 | 每次掃描一筆，約 10 Hz |
| 最糟發布間隔 | 位姿估計器最長的沉默 | 約 0.1 s |
| `initial_to_result_distance` | NDT 必須把先驗搬多遠 | 停車時接近零；行進時小而*穩定* |
| 位置散布 | 每幀相對於局部平滑路徑的偏離 | 對著靜態地圖抖動的位姿即使分數好也是錯的 |
| yaw 步進 | 相鄰影格間的航向變化 | 沒有孤立的大跳動 |
| `iteration_num` | 每次掃描的最佳化步數 | 個位數；卡在上限代表根本沒有位姿 |
| `exe_time_ms` | 一次掃描匹配 | 遠小於 100 ms 的掃描週期 |
| NVTL | 每點平均擬合分數 | 高於門檻且有餘裕——僅此而已（見下） |

`initial_to_result_distance` 是資訊量最大的單一數字，而且必須依運動狀態拆開看。停車時
接近零、行進時大得多，代表問題在預測路徑上——twist 尺度、IMU 或某個外參——而不在匹配器。
在 COSS 錄製檔上這組數字是行進 0.819 m 對停車 0.024 m；修正輪速尺度與 LiDAR 安裝 yaw
之後，行進值降到 0.049 m，而完全沒有動過任何一個 NDT 參數。

## NVTL 把關收斂，不排序品質

`nearest_voxel_transformation_likelihood` 會與
`converged_param_nearest_voxel_transformation_likelihood` 比較，以決定是否發布該影格。
它是一道*門檻*。它不是準確度指標，而且以最大化它為目標調校會把答案反過來：

- **它隨 `ndt.resolution` 放大。** 較粗的體素會抬高 NVTL 同時降低準確度。在這張地圖上，
  解析度 4.0 的分數明顯高於 2.0，而每幀位置散布是後者的五倍。
- **誠實但不完美的遠距回波會拉低平均值卻改善位姿。** 把 crop box 放寬使 NVTL 下降，
  同時讓 yaw 誤差減半。

所以最大化 NVTL 是在挑選粗體素與窄 crop box，無論位姿是否變好。改用位置散布、yaw 步進、
`initial_to_result_distance` 與發布連續性來排序。

關於它還有兩件事：

**NVTL 偏低代表先驗不好的機率，至少與匹配器不好一樣高。** 若它貼近門檻，先往上游看再去
動匹配器——而且要注意這個失敗會自我強化。被拒絕的影格不發布，先驗因此更舊，分數再往下掉。
已發布位姿率出現斷崖就是它的特徵。

**絕對分數只在同一個建置內可比。** NVTL 不是物理量：它取決於解析度、點數，以及計分程式
是否正確。本專案 CUDA 匹配器中的兩個缺陷曾讓它虛高約 1.45 倍，直到 2026 年 8 月修正，
所以在那之前記錄的任何門檻都是對著另一個尺度校準的。**從你正在跑的那個建置的分數分布
重新推導門檻**——絕不要跨版本、跨地圖、跨解析度或跨降採樣點數沿用。

## 各參數，以及何時該動它們

| 參數 | 什麼時候動它 | 不要拿它來 |
|---|---|---|
| `ndt.resolution` | 地圖的點密度真的改變了 | 換取 NVTL 餘裕 |
| `converged_param_nearest_voxel_transformation_likelihood` | 你已重新量測*這個*建置的分數分布 | 掩蓋先驗不良的症狀 |
| crop box `min/max_x`、`min/max_y` | 地圖遠場經量測是好的，且 yaw 約束不足 | 補償被縮放錯的 twist |
| `random_downsample_filter.sample_num` | `exe_time_ms` 有餘裕，或沒有 | 修正準確度問題 |
| `ndt.max_iterations` | `exe_time_ms` 在掃描週期內還有寬裕空間 | 修正因先驗過期造成的不收斂 |
| `initial_pose_estimation.particles_num` | 蒙地卡羅初始化不可靠 | 改善追蹤 |

### 程式樹中的預設值

讀自那兩個參數檔，2026-09-21。兩者不同之處是刻意的，而 CUDA 那個檔案在每個數值旁的註解
中帶著量測資料。

| 參數 | `ndt` | `cuda_ndt` |
|---|---|---|
| `ndt.resolution` | 4.0 | 2.0 |
| `ndt.max_iterations` | 30 | 30 |
| `ndt.trans_epsilon` | 0.01 | 0.01 |
| `ndt.step_size` | 0.1 | 0.1 |
| `ndt.num_threads` | 4 | 4 |
| `converged_param_nearest_voxel_transformation_likelihood` | 2.2 | 2.0 |
| `validation.skipping_publish_num` | 5 | 5 |
| `validation.critical_upper_bound_exe_time_ms` | 100.0 | 100.0 |
| `sensor_points.required_distance` | 10.0 | 10.0 |
| `initial_pose_estimation.particles_num` | 200 | 200 |
| crop box `min/max_x`、`min/max_y` | ±40.0 | ±60.0 |
| voxel grid `voxel_size_*` | 0.5 | 0.5 |
| `random_downsample_filter.sample_num` | 2000 | 5000 |

!!! warning "兩邊的解析度不一致，而量測結果支持 2.0"

    本儲存庫自己在 COSS 地圖上的量測顯示，解析度 2.0 在每一項位姿品質指標上都勝過 4.0，
    而 NVTL 分數反而*較低*。`cuda_ndt` 出貨的是 2.0。內建的 `ndt` 路徑仍出貨 4.0，而它的
    crop box 註解記錄的量測卻是在 2.0 下做的。如果你在 `ndt` 路徑上調校，這是第一件該試
    的事；而且就程式樹中的任何資訊而言，這個差異並不是刻意的 CPU 對 GPU 取捨。

`cuda_ndt` 有四個 CPU 路徑沒有的參數：

- `ndt.use_line_search`（`true`）——批次平行線搜尋，可減少約四分之一的迭代次數。
- `score_estimation.compute_before_scores`（`false`）——同時在初始位姿上計分，以填入兩個
  沒有其他東西消費的診斷欄位。在 AGX Orin 上這一對約花 28 ms，而對位本身是 32 ms。調校時
  打開它以觀察 NDT 把位姿搬了多遠，之後再關掉。關閉時這兩個欄位回報 NaN 而非 0.0，所以
  被停用的分數不會被誤讀成真的是零。
- `initial_pose_estimation.yaw_weight_sigma`（`30.0`，度）——初始化朝初始航向偏置的強度。
- `batch.*`（`enabled: false`）——把數次掃描排入佇列以進行 GPU 平行對位。

### `max_iterations`：那個什麼都沒發布的上限

值得留著，因為這個失敗看起來像成功。AutoSDV 曾經出貨 `max_iterations: 15`，對上 Autoware
的 30，而 `pose_source:=ndt` 在 COSS 錄製檔上完全沒有發布任何位姿。

Autoware 的匹配器把達到上限的影格視為**未收斂**並丟棄結果，所以沒有任何東西到達
`/localization/pose_estimator/pose_with_covariance`：

```
The number of iterations has reached its upper limit.
The number of iterations: 15, Limit: 15.
```

而整個系統*看起來*仍然定位正常，因為 EKF 持續以輪速里程與 IMU 發布
`/localization/kinematic_state`。關鍵的對照是：那個主題有數千筆訊息，而位姿估計器的主題
是零。

把它還原成 30 既正確又*更快*。匹配器大約四次迭代就收斂，只有最初幾幀需要更多，所以那個
過低的上限讓它永遠磨完全部 15 次，而不是接受那一幀能讓後面全部變容易的結果。現在兩邊
都是 30；量測比較表在
[儲存庫的調校指南](https://github.com/NEWSLabNTU/AutoSDV/blob/main/docs/guides/ndt-tuning.md#incident-a-max_iterations-cap-that-published-nothing)。

## 哪些不是調校旋鈕

以下這些是量測值。把它們弄對，而不是在它們上面搜尋，因為為了補償其中之一而調出來的參數，
在其他每個地方都會是錯的：

- 感測器外參，位於 `sensor_kit_calibration.yaml`
- 輪徑與編碼器計數
- IMU 座標系及其軸向約定
- 地圖本身

## 看起來像調校問題的陷阱

**未校正的安裝旋轉。** 如果 LiDAR 相對 `base_link` 有 yaw 偏轉而校正檔寫 0，NDT 會把*感測器*
擺對，軌跡看起來沒問題，但回報的航向會恆定地偏掉那個安裝誤差。它能通過每一項分數檢查，
因為匹配本身是好的。症狀是：直線路段上航向與對地航跡角相差一個常數、yaw 的*變化*仍與陀螺儀
一致，而 `initial_to_result_distance` 在行進時偏高、靜止時正常。及早檢查
`sensor_kit_calibration.yaml`，並對一個全是零的檔案抱持懷疑。

**相鄰影格統計會藏住緩慢漂移。** 10 秒內累積 30 度的 yaw 誤差，每幀只有 0.3 度，在任何
「步進 p95」指標中都很健康。要把絕對量對時間畫出來，並與獨立參照並排。

**轉彎中的對地航跡角。** 拿航向與行進方向比較是抓 yaw 偏移的方法，但有限弦長在曲線中會
系統性地落後瞬時航向，製造出一個根本不存在的偏差。把比較限制在大約 |yaw rate| < 2 deg/s。
在 COSS 錄製檔上，這讓估計值從混淆的 -11.5 度變成乾淨的 -12.66 度。

**共用的 GPU。** CUDA NDT 在整場調查中量到每次掃描 67-83 ms，而它應該只要幾毫秒。匹配器
沒問題；是另一個行程佔住了 32 GB 顯示卡中的 26 GB。線索是*只有*牆鐘時間在動——迭代次數與
NVTL 完全相同，因為爭用付出的是時間，不是收斂。迭代次數不變的時間退化，指向演算法之外。

```bash
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv
```

**把消費級 GNSS 當成真值。** 在信任 `/sensing/gnss/pose` 之前，檢查 `nav_sat_fix.status`
以及 RTCM 是否有在流動。COSS 錄製檔的定位是 `status: 0` 且沒有 NTRIP：水平散布約 20 m、
高度差了數十公尺，而且它與實際行進方向不一致。當作粗略的初始化種子可以，當參照則毫無用處。

**錄製檔會把感測器誤差烙進去。** 任何參數變更都無法消除已經寫進
`/vehicle/status/velocity_status` 的輪速尺度誤差。你可以重新發布一份縮放過的副本來測試
假設，但修正要在車上做。也要留意死掉的訊號：COSS 錄製檔中的 `steering_status` 恆為零，
因為這台車沒有轉向角感測器。

**地圖的遠場沒有近場那麼好。** 掃描對地圖的殘差隨距離增長，而超過某個距離地圖就直接結束了
——在 COSS 地圖上大約是 60 m，超過之後幾乎沒有回波落在任何已建圖的網格上。在放寬 crop box
之前，先量測地圖是否值得：

```bash
python3 scripts/testing/localization/mapcheck/map_coverage.py --run <run_dir>
python3 scripts/testing/localization/mapcheck/map_agreement.py --run <run_dir>
```

要分辨變形的地圖與會動的植被：比較同一個地圖網格從多個車輛位姿看過去的殘差。跨觀測者一致
代表地圖在那裡位移了；隨機則代表是樹葉。

## 檢查清單

在調任何東西之前：

- [ ] 重播能從頭跑到尾並記錄診斷資料
- [ ] 初始位姿刻意設定，每次執行都相同
- [ ] Release 建置——用 `just build`，而不是手寫、少了 `--cargo-args --release` 的 colcon
- [ ] `check_ndt_activated.py` 離開碼為 0
- [ ] IMU 主題有在發布，且 `imu_corrector` 的 log 乾淨
- [ ] twist 有在發布，且 EKF 位移在一段行駛中與現實相符
- [ ] 每個感測器 TF 都能從 `base_link` 解析
- [ ] 參數從執行中的節點讀回，而不是從你編輯的檔案
- [ ] 在把 GNSS 用於任何用途之前先確立其品質
- [ ] 若要相信任何時間數字，GPU 沒有與其他行程共用

每次執行要記錄：

- [ ] 已發布位姿數、對位影格數，以及最糟的發布間隔
- [ ] `initial_to_result_distance`，拆成停車與行進
- [ ] 每幀位置散布與 yaw 步進
- [ ] NVTL 僅作為門檻檢查，連同它高於門檻的餘裕
- [ ] 迭代次數與 `exe_time_ms`，對照掃描週期

相信之前先驗證：

- [ ] yaw 變化對積分陀螺儀，依時間分桶
- [ ] 航向對對地航跡角，僅限直線路段
- [ ] 行駛距離對輪速里程、也對地圖
- [ ] 每次執行只改一個變數，或把混淆的那一對做完整交叉

## 相關資料

- [定位診斷](./localization-diagnostics.md) —— 本頁提到的每個腳本，以及它各自回答的問題
- [定位方法](./localization-methods.md) —— 在 `ndt`、`cuda_ndt` 與 `mcl` 之間選擇
- [地圖](./maps.md) —— 每種方法需要什麼，以及如何建立
- [Logging Simulation](../tutorial/03-logging-simulation.md) —— 本頁所調校的那個重播，從頭講起
- 本頁背後的量測結果放在儲存庫中：
  [`docs/reports/cuda-ndt-coss-replay.md`](https://github.com/NEWSLabNTU/AutoSDV/blob/main/docs/reports/cuda-ndt-coss-replay.md)
  與
  [`docs/reports/localization-open-questions.md`](https://github.com/NEWSLabNTU/AutoSDV/blob/main/docs/reports/localization-open-questions.md)
