<div align="center">

# Isaac Sim 病房 AMR
## 建圖與路徑規劃

<p>純 Python 差速車研究專案，聚焦於 Isaac Sim 中的病房 AMR 建圖、定位與連續導航。</p>

<p>
  <a href="navigation/v66"><strong>目前版本：V66 Continuous Navigation 2.0</strong></a>
  ·
  <a href="docs/OPERATION.md">操作指南</a>
  ·
  <a href="docs/VALIDATION.md">驗證狀態</a>
</p>

</div>

目前主程式為 **V66 Continuous Navigation 2.0**。它沿用 V66 輪組介面與 SLAM，加入優先連續行駛搜尋、CG 候選平滑和曲率速度包絡。這是差速驅動的論文啟發變體；離線回放已通過，Isaac Sim 的完整動態驗收仍須按版本分別確認。

> **目前狀態**　離線規劃與回放已完成；實機／完整 Isaac Sim 動態結果請以[驗證狀態](docs/VALIDATION.md)及各版本紀錄為準。

本 repository 必須區分下列驗證層級：

| 層級 | 定義 |
| --- | --- |
| 程式與資料完整性 | 檔案雜湊、Git LFS、Python／JSON 語法及設定一致性 |
| 離線演算法驗證 | 固定地圖與輸入上的規劃、平滑、碰撞取樣及理想運動學回放 |
| Isaac Sim 實驗 | 指定場景、物理設定與感測器條件下的單次或多次實跑 |
| 動態驗收 | 同時具備物理接觸、滑移、世界真值、SLAM、命令時基及重複測試證據 |

## 導覽

### 🚀 先從這裡開始

<table>
  <tr>
    <td width="50%"><strong>🧭 目前完整導航程式</strong><br><a href="navigation/v66">navigation/v66</a><br><sub>V66 Continuous Navigation 2.0 的主要入口</sub></td>
    <td width="50%"><strong>🎮 安裝與操作</strong><br><a href="docs/OPERATION.md">操作指南</a><br><sub>安裝、四方向操作、建圖與拖曳 B 點</sub></td>
  </tr>
  <tr>
    <td><strong>🧱 系統架構</strong><br><a href="docs/ARCHITECTURE.md">架構與演算法</a><br><sub>全系統流程與路徑規劃設計</sub></td>
    <td><strong>⚙️ 參數設定</strong><br><a href="docs/PARAMETERS.md">參數表</a><br><sub>目前實際使用的尺寸、座標與控制參數</sub></td>
  </tr>
</table>

### 📚 文件與紀錄

<table>
  <tr>
    <td width="50%"><strong>🧪 驗證結果</strong><br><a href="docs/VALIDATION.md">驗證狀態</a><br><sub>檢查項目、實跑證據與目前界線</sub></td>
    <td width="50%"><strong>🛠️ 問題追蹤</strong><br><a href="docs/ISSUES_AND_EVIDENCE.md">問題與處理紀錄</a><br><sub>已知問題、證據與修正位置</sub></td>
  </tr>
  <tr>
    <td><strong>🗂️ 版本管理</strong><br><a href="docs/VERSIONING.md">版本管理</a><br><sub>版本演進、差異與回退方式</sub></td>
    <td><strong>🔭 後續規劃</strong><br><a href="docs/NAV2_ROADMAP.md">後續比較</a><br><sub>ROS 2／Nav2 接入與規劃器比較</sub></td>
  </tr>
</table>

### 📦 資產與工具

| 類別 | 位置 | 用途 |
| --- | --- | --- |
| 車體 USD 與場景 | [`assets/v66`](assets/v66) | 新車 configured 與 control USD |
| 修正工具 | [`tools/isaac`](tools/isaac) | 輪組檢查及明確手動套用的修正工具 |
| 歷史程式 | [`archive`](archive) | 舊版本與歷史實作 |
| 數據與影片 | [`evidence`](evidence) | 選錄數據、回放與影片 |
| 本次整理範圍 | [`docs/REVIEW_SCOPE.md`](docs/REVIEW_SCOPE.md) | 審閱範圍與資料來源 |

## 取得完整檔案

車體 USD、影片及部分大型診斷 JSON 使用 Git LFS。先安裝 Git LFS，接著執行下列命令：

```bash
git clone https://github.com/Jimmy0108/Isaac_sim_sickroom_pathplanning.git
cd Isaac_sim_sickroom_pathplanning
git lfs pull
python tools/verify_repository.py
```

最後一個命令只驗證檔案完整、Python 語法與設定一致性。完整模擬需要 Isaac Sim 及使用者原有病房場景。

若驗證器回報 `LFS pointer only`，表示目前 checkout 僅取得 Git LFS 指標，尚未取得對應的大型檔案；此狀態不得視為資料完整。若遠端 LFS 憑證或網路不可用，應保留該結果，不得以空檔或指標代替原始資料。

## 在 Isaac Sim 啟動

1. 使用已設定的新車 V6.6 場景，確認僅有一個有效 PhysicsScene。
2. 把 `navigation/v66/` 的內容平鋪到遠端的 `V66_Python_Navigation/`。入口內的 `PROJECT` 必須指向該資料夾。
3. STOP → RUN `CHECK_Installed_2_0.py` → RUN `AMR_V66_Navigation.py` → PLAY。
4. 等落地及介面就緒，試前後左右，再選擇新建圖或載入相符地圖。
5. A 取目前 SLAM 估計，B 可拖曳 XY 與旋轉朝向。先 Preview，再 Navigate。

車體的方向約定為光達端朝 `base_link −X`。左右輪的舊命名與邏輯左右不同，應依設定及實際綁定核對。詳見[參數表](docs/PARAMETERS.md)。

## 已有證據

| 結果 | 所屬版本及限制 |
| --- | --- |
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
- 本倉庫包含車體、小型落地測試場景、廠商 STEP、驅動器文件及既有實跑原始掃描。完整病房／Gaussian 重建環境與相機原始 CAD 尚未收錄。補充範圍見[資料盤點](docs/DATA_AVAILABILITY.md)。
- ROS 2／Nav2 接入屬後續工作。Smac 2D＋RPP 與 State Lattice＋RPP 的討論記在[後續比較](docs/NAV2_ROADMAP.md)。

## 版本、分支與研究引用

`main` 保存可追溯的整理版本；後續程式或參數修改應建立 `codex/` 或其他具體用途分支，完成離線檢查後再提交。修改 runtime 檔案時，必須同步更新 release manifest、參數快照及相應驗證紀錄。歷史實驗若發生於 Git 建檔以前，不得事後宣稱其結果對應目前 commit。

方法背景請參考 `docs/references/`、`docs/ARCHITECTURE.md` 及 `docs/NAV2_ROADMAP.md`。正式論文引用、版本差異與未實作項目應以文件記載為準；本專案的實作結果不得直接推論為實車性能或通用場景保證。
