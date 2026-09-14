<!--
Translation Metadata:
- Source file: docker.md
- Last synced: 2026-09-14
- Translator: Claude (Anthropic)
- Status: Complete
-->

# Docker 設定（已停止維護）

!!! danger "此映像檔無法建置，請勿從這裡開始。"

    `docker/` 中的 Dockerfile 無法針對目前的儲存庫建置。請改用
    [標準安裝方式](./overview.md)。

    有兩個各自獨立的原因：

    1. **它的建置步驟已不存在。** Dockerfile 的最後一步執行
       `./scripts/setup-dev-env/setup-dev-env.sh -y`。該腳本在設定程式改寫為
       步驟註冊表時就已移除；儲存庫中不再有 `scripts/setup-dev-env/` 目錄，
       因此建置會在該行失敗。
    2. **它的基礎映像檔屬於錯誤的平台世代。** 它以
       `FROM nvcr.io/nvidia/l4t-tensorrt:r8.6.2-devel` 建置，那是 JetPack 5
       時期的映像檔。arm64 版的 Autoware 1.5.0 以 JetPack 6.2 系列（這裡是 6.2.2 或
       更新，也就是 L4T 36.5）及其隨附的
       TensorRT 為目標。

    修好它是一件實際的工作——需要新的基礎映像檔，以及一份改為驅動
    `./setup.sh --run --profile ci --yes` 而非已刪除腳本的 Dockerfile。
    在有人動手之前，本頁僅作為當初內容的記錄。

## 它原本的用途

該映像檔建立一個 NVIDIA L4T 環境，以你本機檢出的確切 commit 複製 AutoSDV，
並在其中執行設定腳本——如此容器對應的是你機器上的程式碼狀態，而不是某個
分支的最新狀態。

這些檔案仍在儲存庫的 `docker/` 目錄下：

| 檔案 | 用途 |
|------|------|
| `Dockerfile` | 上述的映像檔定義 |
| `Makefile` | `build`、`run`、`run-rocker`、`save` 目標；將本機 commit hash 作為建置參數傳入 |
| `nvidia-l4t-apt-source.list` | 映像檔內使用的 L4T apt 來源 |
| `README.md` | 原本的說明 |

## 該怎麼做

在主機上安裝。[軟體安裝](./overview.md)在 Jetson 與 amd64 上都是受支援的路徑，
而設定程式的 `ci` 設定檔正是為了非互動式的無人值守環境而存在：

```bash
./setup.sh --run --profile ci --yes
```

請注意 `ci` 刻意不含 Autoware 本身，所以它產生的機器可以取得相依套件與檢查
程式碼，但無法建置工作空間。需要建置的容器應使用 `--profile dev`。

## 如果你想讓它復活

修復的輪廓是已知的：

1. 選擇一個 L4T 36.5（JetPack 6.2.2 或更新）的基礎映像檔，搭配相符的 TensorRT。
2. 將已刪除的 `setup-dev-env.sh` 呼叫改為
   `./setup.sh --run --profile dev --yes`。
3. 決定如何處理 Autoware Debian 套件的下載（2–3 GB）——把它烘進一層會讓映像檔
   非常龐大，在執行時才取得則會讓容器無法離線使用。
4. TensorRT 引擎則完全無法預先烘進映像檔：它們同時綁定 TensorRT 版本**與**特定
   的 GPU，因此必須在容器啟動後於目標板上建置。

第 4 點正是讓一個真正自給自足的 AutoSDV 映像檔成為不可能、而非僅僅龐大的原因。
