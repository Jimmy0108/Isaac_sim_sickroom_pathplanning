# Isaac Sim 病房 AMR 建圖與路徑規劃

純 Python 差速車研究專案，整理 2026 年 10 月至 11 月的車體移植、輪式里程計、SLAM、Hybrid A*、Voronoi field 與 RPP 開發歷程。

目前主程式為 **V66 Continuous Navigation 2.0**。它沿用 V66 輪組介面與 SLAM，加入優先連續行駛搜尋、CG 候選平滑和曲率速度包絡。這是差速驅動的論文啟發變體。其離線回放已通過，Isaac Sim 的完整動態驗收仍須按版本分別確認。

## 從哪裡開始

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

## 取得完整檔案

車體 USD、影片及部分大型診斷 JSON 使用 Git LFS。先安裝 Git LFS，接著執行下列命令。

```bash
git clone https://github.com/Jimmy0108/Isaac_sim_sickroom_pathplanning.git
cd Isaac_sim_sickroom_pathplanning
git lfs pull
python tools/verify_repository.py
```

最後一個命令只驗證檔案完整、Python 語法與設定一致性。完整模擬需要 Isaac Sim 及使用者原有病房場景。

## 在 Isaac Sim 啟動

1. 使用已設定的新車 V6.6 場景，確認僅有一個有效 PhysicsScene。
2. 把 `navigation/v66/` 的內容平鋪到遠端的 `V66_Python_Navigation/`。入口內的 `PROJECT` 必須指向該資料夾。
3. STOP → RUN `CHECK_Installed_2_0.py` → RUN `AMR_V66_Navigation.py` → PLAY。
4. 等落地及介面就緒，試前後左右，再選擇新建圖或載入相符地圖。
5. A 取目前 SLAM 估計，B 可拖曳 XY 與旋轉朝向。先 Preview，再 Navigate。

車體的方向約定為光達端朝 `base_link −X`。左右輪的舊命名與邏輯左右不同，應依設定及實際綁定核對。詳見 [參數與座標](docs/PARAMETERS.md)。

## 已有證據

| 結果 | 所屬版本及限制 |
|---|---|
| 6.0283 m、單一前進段、0 spin、0 換向 | 2.0 離線規劃，使用保存地圖 |
| 理想 RPP 回放抵達誤差約 6.96 cm、2.38° | 無輪地滑移及 SLAM 漂移的運動學回放 |
| 建圖、保存、載圖、規劃、抵達與停穩流程 | V66 1.0 的特定實跑，估計抵達與世界位置仍有差異 |
| 1.878 m 前進、yaw +2.263° | 候選萬向輪材質實跑，命令峰值僅 0.20 m/s |
| 旋轉後偏行、高速輪速瞬態異常 | 已有數據，唯一物理根因仍待同條件驗證 |

主程式的 `OFFLINE_VALIDATION.json` 及 `RELEASE_MANIFEST_2_0.json` 隨版保存。兩段弧形測試影片屬視覺觀察證據，未與舊版做同條件時間及淨空比較。

## 資產與依賴

- 已使用的模擬版本為 Isaac Sim 6.0.1-rc.7，其他版本需要重新驗證 API。
- 車殼尺寸、導航整車包絡與驅動輪心距採不同量測定義，詳見參數表。
- 本倉庫包含車體及小型落地測試場景。原有病房／Gaussian 重建環境、廠商 STEP 與相機原始 CAD 在外部保存。
- ROS 2／Nav2 接入屬後續工作。Smac 2D＋RPP 與 State Lattice＋RPP 的討論記在 [後續比較](docs/NAV2_ROADMAP.md)。
