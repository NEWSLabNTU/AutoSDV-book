<!--
Translation Metadata:
- Source file: version-control.md
- Last synced: 2026-01-09
- Translator: Claude (Anthropic)
- Status: Complete
-->

# 版本控制

本專案採用**超專案**方法來管理大量的 Git 儲存庫。AutoSDV 儲存庫本身，也就是所謂的超專案，將子專案儲存為 _Git 子模組_，但不儲存其實際資料。您可以從[此處](https://en.wikibooks.org/wiki/Git/Submodules_and_Superprojects)的教學了解超專案的概念。

Git 子模組的運作方式如同母儲存庫中的超連結。母儲存庫將子模組的資訊儲存在 `.gitmodules` 檔案中。您可以列出它們：

```sh
git submodule status
```

> **注意**
>
> 官方 Autoware 採用與我們不同的版本控制策略。請勿混淆。

## 下載包含子模組的儲存庫

下載包含子模組的儲存庫時，務必加上 `--recursive` 選項。

```sh
git clone --recursive https://github.com/NEWSLabNTU/AutoSDV.git
```

如果您忘記加上此選項，子模組目錄將會是空的。您可以在之後取得子模組內容。

```sh
cd AutoSDV/
git submodule update --init --recursive
```

## 檢查子模組

讓我們以 `src/sensor_kit/autosdv_sensor_kit_launch` 子模組為例進行檢查。開啟 `.gitmodules`，您可以看到下方的區段。它告訴我們子模組的目錄位置以及上游 URL。

```
[submodule "src/sensor_kit/autosdv_sensor_kit_launch"]
        path = src/sensor_kit/autosdv_sensor_kit_launch
        url = https://github.com/NEWSLabNTU/autosdv_sensor_kit_launch.git

```

從母儲存庫的角度來看，路徑 `src/sensor_kit/autosdv_sensor_kit_launch` 被視為一個連結檔案。它儲存被追蹤的 Git 儲存庫的提交雜湊值。您可以使用以下指令顯示提交雜湊值。如果提交雜湊值改變，我們會經歷一般的 git add 與 commit 流程來儲存它。

```sh
$ git submodule status src/sensor_kit/autosdv_sensor_kit_launch
 a1b2c3d4e5f6... src/sensor_kit/autosdv_sensor_kit_launch (heads/main)
```

## 儲存子模組內的變更

要儲存子模組內的變更，您必須以兩步驟的方式在子模組儲存庫和母儲存庫中提交變更。

讓我們以 `src/sensor_kit/autosdv_sensor_kit_launch` 子模組為例。

| 已提交的變更                                       | 推送至上游儲存庫                                                                                           |
|----------------------------------------------------|-----------------------------------------------------------------------------------------------------------|
| `autosdv_sensor_kit_launch` 子模組內的變更。      | [autosdv\_sensor\_kit\_launch](https://github.com/NEWSLabNTU/autosdv_sensor_kit_launch) 子專案儲存庫    |
| `autosdv_sensor_kit_launch` 子模組的新提交雜湊值。| [AutoSDV](https://github.com/NEWSLabNTU/AutoSDV) 母儲存庫                                                 |

完整流程如下。

```sh
# Go into the submodule and checkout the branch we want to work on.
cd src/sensor_kit/autosdv_sensor_kit_launch
git checkout main

# Do some work in the submodule.
touch a_new_file  # Create a file

# Commit and push to the upstream repo.
git add a_new_file
git commit -m 'Add a new file'
git push

# Go back to the mother repo
cd -

# Save the new commit hash on the submodule and push it to the upstream repo.
git add src/sensor_kit/autosdv_sensor_kit_launch
git commit -m 'Update the autosdv_sensor_kit_launch submodule'
git push
```
