# Isaac Sim 病房 AMR 建圖與路徑規劃

本 repository 為 TZAGV-B02 差速式自主移動機器人（AMR）之 Isaac Sim 研究與開發資料集，涵蓋車體移植、輪式里程計、雷射掃描、SLAM、Hybrid A*、Voronoi 代價、路徑平滑及 RPP-style 路徑追蹤。文件同時作為開發手冊、研究計畫說明及實驗資料索引。

目前主要實作為 **V66 Continuous Navigation 2.0**。本版本採差速運動基元，優先搜尋不含原地旋轉的連續行駛路徑，並以數值 CG 候選平滑、曲率速度包絡及反應時間／煞車預測改善路徑追蹤。上述方法為論文方法之差速車變體，不宣稱完整重現原始論文或具備最優性保證。

本 repository 必須區分下列驗證層級：

| 層級 | 定義 |
|---|---|
| 程式與資料完整性 | 檔案雜湊、Git LFS、Python／JSON 語法及設定一致性 |
| 離線演算法驗證 | 固定地圖與輸入上的規劃、平滑、碰撞取樣及理想運動學回放 |
| Isaac Sim 實驗 | 指定場景、物理設定與感測器條件下的單次或多次實跑 |
| 動態驗收 | 同時具備物理接觸、滑移、世界真值、SLAM、命令時基及重複測試證據 |

目前 repository 已具備前兩層及多組歷史實跑資料；V66 Continuous 2.0 的完整動態驗收仍須依指定場景與版本執行。

## 研究與開發導覽

| 目的 | 位置 |
|---|---|
| 目前完整導航程式 | [navigation/v66](navigation/v66) |
| 安裝、四方向操作、建圖與拖曳 B 點 | [操作指南](docs/OPERATION.md) |
| 全系統與路徑規劃架構 | [架構與演算法](docs/ARCHITECTURE.md) |
| 目前實際設定值 | [參數表](docs/PARAMETERS.md) |
| 問題、證據與修正位置 | [問題與處理紀錄](docs/ISSUES_AND_EVIDENCE.md) |
| 版本演進與回退 | [版本管理](docs/VERSIONING.md) |
| 檢查及實跑的證據界線 | [驗證狀態](docs/VALIDATION.md) |
| 新車 configured 與 control USD | [assets/v66](assets/v66) |
| 輪組檢查及明確手動套用的修正工具 | [tools/isaac](tools/isaac) |
| 歷史程式 | [archive](archive) |
| 選錄數據與影片 | [evidence](evidence) |
| 本次整理範圍 | [審閱與來源](docs/REVIEW_SCOPE.md) |
| 已有資料與剩餘缺口 | [資料盤點](docs/DATA_AVAILABILITY.md) |
| 學術文獻與方法來源 | [references](docs/references) |

## 研究目標與範圍

本計畫聚焦於下列問題：

1. 在具備完整車身外廓與差速運動約束的條件下，建立可在病房類室內環境運作的地圖與路徑規劃流程。
2. 以輪式里程計、局部距離觀測及 SLAM 姿態估計支援導航，而非以 Isaac Sim 世界真值取代導航估計。
3. 評估連續弧形行駛相對於原地旋轉與換向策略，在路徑長度、最小淨空、追蹤誤差、停車旋轉及抵達誤差上的差異。
4. 建立可追溯的模擬資產、參數、實驗紀錄及驗證界線，供後續 Nav2 接入與學術比較使用。

本 repository 不包含完整病房／Gaussian 重建環境，亦不宣稱已完成實車驗證。質量、慣量、摩擦、煞車能力及接觸行為若未標示為量測值，均應視為模擬工程假設或候選設定。

## 取得資料與執行完整性檢查

車體 USD、影片及部分大型診斷 JSON 使用 Git LFS。先安裝 Git LFS，接著執行下列命令。

```bash
git clone https://github.com/Jimmy0108/Isaac_sim_sickroom_pathplanning.git
cd Isaac_sim_sickroom_pathplanning
git lfs pull
python tools/verify_repository.py
```

最後一個命令只驗證檔案完整、Python 語法與設定一致性。完整模擬需要 Isaac Sim 及使用者原有病房場景。

若驗證器回報 `LFS pointer only`，表示目前 checkout 僅取得 Git LFS 指標，尚未取得對應的大型檔案；此狀態不得視為資料完整。若遠端 LFS 憑證或網路不可用，應保留該結果，不得以空檔或指標代替原始資料。

## 開發環境與執行方式

1. 使用已設定的新車 V6.6 場景，確認僅有一個有效 PhysicsScene。
2. 把 `navigation/v66/` 的內容平鋪到遠端的 `V66_Python_Navigation/`。入口內的 `PROJECT` 必須指向該資料夾。
3. STOP → RUN `CHECK_Installed_2_0.py` → RUN `AMR_V66_Navigation.py` → PLAY。
4. 等落地及介面就緒，試前後左右，再選擇新建圖或載入相符地圖。
5. A 取目前 SLAM 估計，B 可拖曳 XY 與旋轉朝向。先 Preview，再 Navigate。

車體的方向約定為光達端朝 `base_link −X`。左右輪的舊命名與邏輯左右不同，應依設定及實際綁定核對。詳見 [參數與座標](docs/PARAMETERS.md)。

## 實驗證據與目前結果

| 結果 | 所屬版本及限制 |
|---|---|
| 6.0283 m、單一前進段、0 spin、0 換向 | 2.0 離線規劃，使用保存地圖 |
| 理想 RPP 回放抵達誤差約 6.96 cm、2.38° | 無輪地滑移及 SLAM 漂移的運動學回放 |
| 建圖、保存、載圖、規劃、抵達與停穩流程 | V66 1.0 的特定實跑，估計抵達與世界位置仍有差異 |
| 1.878 m 前進、yaw +2.263° | 候選萬向輪材質實跑，命令峰值僅 0.20 m/s |
| 旋轉後偏行、高速輪速瞬態異常 | 已有數據，唯一物理根因仍待同條件驗證 |

主程式的 `OFFLINE_VALIDATION.json` 及 `RELEASE_MANIFEST_2_0.json` 隨版保存。兩段弧形測試影片屬視覺觀察證據，未與舊版做同條件時間及淨空比較。

各實驗 session 的命令、輪角／輪速、世界姿態、SLAM 輸入、有效物理設定及摘要，依 `evidence/mapping_session_<ID>/`、`evidence/vehicle_audit_<ID>/` 與 `evidence/wheel_contact_<ID>/` 分類保存。`docs/source_manifest.json` 與 `docs/supplement_manifest.json` 記錄來源、檔案大小及 SHA-256；雜湊一致僅證明檔案未被改動，不代表該實驗已通過物理或導航驗收。

## 實驗方法與評估指標

比較不同規劃或控制版本時，應固定場景、地圖、車體外廓、起點、終點姿態、障礙物、物理參數、速度上限及定位來源，至少記錄：

- 路徑長度、執行時間及最小障礙淨空；
- 位置／方向抵達誤差、最大追蹤誤差及停止原因；
- 原地旋轉次數、前後換向次數及重規劃次數；
- 左右輪命令、實際輪角／輪速、車身世界姿態及 SLAM 姿態；
- 掃描時戳、物理時鐘、接觸事件及滑移估計的資料來源與有效性。

目前尚未完成 Nav2 Smac 2D、State Lattice 或其他基準方法之同條件公平比較。

## 資產與依賴

- 已使用的模擬版本為 Isaac Sim 6.0.1-rc.7，其他版本需要重新驗證 API。
- 車殼尺寸、導航整車包絡與驅動輪心距採不同量測定義，詳見參數表。
- 本倉庫包含車體、小型落地測試場景、廠商 STEP、驅動器文件、兩篇方法文獻及既有實跑原始資料。完整病房／Gaussian 重建環境與相機原始 CAD 尚未收錄。補充範圍見 [資料盤點](docs/DATA_AVAILABILITY.md)。
- ROS 2／Nav2 接入屬後續工作。Smac 2D＋RPP 與 State Lattice＋RPP 的討論記在 [後續比較](docs/NAV2_ROADMAP.md)。

## 版本、分支與研究引用

`main` 保存可追溯的整理版本；後續程式或參數修改應建立 `codex/` 或其他具體用途分支，完成離線檢查後再提交。修改 runtime 檔案時，必須同步更新 release manifest、參數快照及相應驗證紀錄。歷史實驗若發生於 Git 建檔以前，不得事後宣稱其結果對應目前 commit。

方法背景請參考 `docs/references/`、`docs/ARCHITECTURE.md` 及 `docs/NAV2_ROADMAP.md`。正式論文引用、版本差異與未實作項目應以文件記載為準；本專案的實作結果不得直接推論為實車性能或通用場景保證。
