# 現有資料補充與剩餘缺口

整理日：2026-11-08。這次補充既有資料，未修改導航、車體物理或執行新的 Isaac Sim 實驗。

| 類別 | 已收錄 | 仍缺少／限制 |
|---|---|---|
| 環境 | README 記錄使用者提供的 Isaac Sim 6.0.1-rc.7；稽核與事件記錄 | 統一環境快照、Build ID、GPU 驅動、完整啟動日誌；本機 Windows 不能代表遠端容器 OS |
| 資產 | assets/v66 新車及控制、migration_inputs 舊車、assets/vendor 原廠 STEP | 完整病房及 Gaussian 重建場景未在此次補充中取得 |
| PhysX | evidence 下控制診斷、實際輪角輪速、世界姿態、接觸、有效物理設定及車體稽核 | 滑移須區分幾何速度代理與實際接觸點滑移；沒有 callback 的資料不能視為零接觸 |
| 掃描與定位 | mapping_sensor_frames.json、mapping_sensor_samples.npz、consumed_slam_inputs.npz（原包有提供者）、sensor_setup、地圖與路徑 | 安裝設定不等於實測校正；未追溯到的資料不補造 |
| 參數依據 | 有效配置、effective_physics、原廠 STEP、LDS 手冊與通訊文件 | 舊模型質量、估算慣量、候選摩擦不因此變成原廠實測值；LDS 文件也不證明當前實車採用該型號 |
| 實驗 | 原有 session、goal、arrival、停止追蹤及規劃統計（依原包內容） | 未完成固定條件多次重複實驗 |
| 比較 | 歷史版本、離線結果、兩段影片 | 未執行 Nav2 Smac 2D／State Lattice 公平對照 |
| 文獻 | docs/references 兩篇原始 PDF；ARCHITECTURE 的方法差異及已知 DOI | 完整 BibTeX／逐項論文重現驗收尚未完成；PDF 保留原出版者與作者權利 |
| 測試 | 原有離線測試與 tools/verify_repository.py；本次增加補充檔案雜湊驗證 | 沒有新增 runtime 通過證明或 CI |
| 追溯 | source_manifest.json＋supplement_manifest.json 的原始路徑、bytes、SHA-256；session 有效配置 | 歷史測試在初次 Git 建檔之前，不能事後宣稱測試對應當前 commit |

## 如何找到資料

- `evidence/mapping_session_<ID>/`：命令、輪角、輪速、車身診斷、掃描、地圖及導航結果。各 session 檔案數依原始輸出而異。
- `evidence/wheel_contact_<ID>/`：接觸對、輪組運動學、collider inventory、採集 metadata。
- `evidence/vehicle_audit_<ID>/`：靜態車體屬性及運行樣本。
- `assets/vendor/`：原廠 STEP、驅動器 PDF、原廠讀取 Encoder／移動範例 ZIP。範例保留原始封裝，未執行，勿把實車通訊腳本直接用作 Isaac Sim 控制器。
- `docs/references/`：兩篇使用者提供的 Hybrid A* 文獻。

本次未重複收錄各 ZIP 中的 runtime_sources；既有正式程式與版本存檔仍在 navigation、archive。原始 ZIP 保留本機。補充清單每列記錄 ZIP 成員來源或外部資料來源別名，不上傳個人對話或帳號日誌。

## 使用與驗證

先 `git lfs pull`，再 `python tools/verify_repository.py`。有 LFS pointer 而沒有實體內容的 checkout 不算資料完整。本次上傳成功與 hash 通過只證明檔案完整，不證明物理、SLAM 或導航通過驗收。
