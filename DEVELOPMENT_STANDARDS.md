# 開發規範

本文件規定本 repository 之程式開發、模擬實驗、資料保存與 Git 協作方式。目的在於維持可追溯性、可重現性及研究結果之證據界線。

## 1. Repository 分層

| 路徑 | 用途 | 管理原則 |
|---|---|---|
| `navigation/v66/` | 目前主要導航實作 | 可修改；runtime 變更須更新版本與驗證資料 |
| `archive/` | 歷史版本與過渡實作 | 原則上唯讀；修正應建立新版本，不直接改寫歷史 |
| `assets/v66/` | 目前車體、控制 USD 及幾何設定 | 修改前須保留來源與驗證紀錄 |
| `assets/vendor/` | 廠商原始資料 | 僅保存，不修改內容 |
| `evidence/` | 實驗原始資料與結果 | 原始檔不可覆寫；修正版建立新 session 或新檔名 |
| `docs/` | 架構、參數、驗證與資料索引 | 程式或實驗流程變更時同步更新 |
| `tools/` | 驗證、匯入及診斷工具 | 工具變更須附測試或執行結果 |

大型二進位檔案、影片、USD/USDC 與大型診斷資料必須依 `.gitattributes` 使用 Git LFS。Git LFS pointer 不得視為完整實驗資料。

## 2. 分支與 Git 流程

`main` 為可追溯的整合分支，不直接提交。開發流程如下：

```text
更新 main
  → 建立用途明確的分支
  → 修改與測試
  → 建立 Pull Request
  → 審查與修正
  → 合併 main
```

分支命名建議：

| 類型 | 格式 | 範例 |
|---|---|---|
| 功能 | `feature/<topic>` | `feature/scan-calibration` |
| 修正 | `fix/<topic>` | `fix/replan-timing` |
| 實驗 | `experiment/<topic>` | `experiment/smac-baseline` |
| 文件 | `docs/<topic>` | `docs/update-reproduction-guide` |
| 自動化 | `chore/<topic>` | `chore/add-ci` |

避免在同一分支混合無關的程式、資料搬移與文件變更。每一個分支應具有單一可說明的目的。

## 3. Commit 規範

Commit 標題採「動詞＋對象」或 Conventional Commits 形式，並清楚說明變更範圍：

```text
Add scan timestamp validation
Fix replan stop timer
docs: clarify evidence limitations
test: add planner regression case
chore: update release manifest
```

每一個 commit 應維持可檢查狀態。不得提交密碼、權杖、個人帳號日誌或未經確認的機密資料。修改 runtime 檔案時，應在同一 Pull Request 更新相應的 release manifest、設定快照或驗證文件。

## 4. Pull Request 規範

Pull Request 描述至少包含：

1. 變更目的與範圍；
2. 受影響的程式、設定、資產或證據；
3. 已執行的測試與結果；
4. 是否改變導航、物理、感測器或資料格式行為；
5. 尚未完成的驗證與已知限制；
6. 若為實驗，列出場景、地圖、起終點、參數、資料來源及 commit。

合併前應確認：

- `main` 保護規則未被繞過；
- Pull Request 審查討論已解決；
- 沒有將 LFS pointer 誤當成實體檔；
- 文件、manifest 與測試結果彼此一致。

## 5. 程式碼規範

- 以 Python 3 語法及現有模組介面為基準。
- 優先使用既有設定、資料結構與驗證工具，避免建立重複的參數來源。
- 單位、座標系、符號慣例及時間基準必須在介面或文件中明確記載。
- 導航估計不得以 Isaac Sim 世界真值取代輪式里程計、感測器或 SLAM 輸入。
- 例外狀況應明確失敗並保留診斷資訊，不得以靜默預設值掩蓋設定錯誤。
- 不得將模擬工程假設描述為量測值、廠商規格或實車驗證結果。

車體幾何、輪徑、輪距與 RPP 設定變更後，至少執行：

```bash
python tools/verify_repository.py
python -m compileall -q navigation/v66 tools archive
```

## 6. 實驗與證據規範

每次實驗應固定或記錄：

- Isaac Sim 版本、Build ID、作業系統、GPU、驅動及 Extension；
- 場景、車體資產、地圖、起點、終點姿態及障礙物；
- 使用的 commit/tag、設定快照、物理參數與速度限制；
- 命令、輪角／輪速、世界姿態、SLAM 姿態、掃描時戳及物理時鐘；
- 接觸採集狀態、滑移定義、停止原因與資料有效性。

證據必須區分：

| 狀態 | 可支持的結論 |
|---|---|
| 離線驗證 | 演算法在指定輸入下的計算結果 |
| 單次實跑 | 指定場景與條件下的一次觀察 |
| 重複實驗 | 固定條件下的統計比較 |
| 動態驗收 | 同時具備物理、感測、定位及重複性證據 |

影片只能支持可觀察的視覺現象，不能單獨證明定位精度、滑移率、碰撞安全或因果改善。沒有 contact callback 的資料不得解讀為「沒有接觸」。

## 7. 學術與引用規範

論文、官方文件及廠商資料應保留來源、作者／機構、日期、DOI 或 URL。方法若為改寫或差速車變體，必須明確說明與原始方法的差異，不得宣稱完整重現。

研究結果應同時記載：

- 方法定義與實作範圍；
- 評估指標與固定條件；
- 基準方法；
- 統計或重複實驗設計；
- 限制、未驗證項目及可重現步驟。

## 8. 變更前檢查清單

```text
[ ] 分支名稱與變更目的明確
[ ] 未修改 archive 或 vendor 原始資料
[ ] 大型檔案使用 Git LFS
[ ] runtime 變更已更新 manifest／設定／版本文件
[ ] Python 語法與離線測試已執行
[ ] 實驗資料包含來源、參數與有效性說明
[ ] README、docs 與實際行為一致
[ ] Pull Request 已說明限制與未完成驗證
```

