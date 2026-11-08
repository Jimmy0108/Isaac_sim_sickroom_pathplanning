# V66 navigation 1.0 — 2026-11-06

## 可沿用
- `package_new` 的差速 Hybrid A*（含倒退／原地旋轉）、raster generalized Voronoi field、RPP、輪關節角度 odom、scan matching／實驗 pose graph SLAM、格網和地板顯示。
- 車頭 base_link −X、linear sign −1、angular sign +1、邏輯左右輪轉換、encoder signs −1。
- 遙控及導航速度上限 0.9 m/s、0.9 rad/s；既有手動起步斜坡與 RPP 增益不重調。

## 必須更新，已完成
- 獨立檔名、入口、模組載入、設定、GoalB namespace 與資料輸出目錄，避免覆蓋舊共用核心。
- radius .09→.08；track .514→.4769668332689978；完整外框 .88×.56→.898×.5698；高度 .45→.444。
- planner.Config／WheelOdom 預設值同步改為新版；實際啟動仍明確傳入 JSON 值。
- Encoder articulation path 及原始左右 DOF 名稱改為明確指定，保留 front frame 的單次換邊。
- 啟動移除舊 10 N.m、grip binding、solver velocity-iteration、補建 60×25 mm 萬向輪碰撞、scene-rate 和 rendering-rate 修復。改成讀取／驗證新版 USD；失敗不擅自重寫。
- 核對六輪有效碰撞、31.5 mm 半徑／19.2629 mm 寬 caster、joint anchors／axes、剛體數、graph 目標及 evaluated wheel order。
- 自動採用一個既有 PhysicsScene 的 dt；Play 前後檢查有效場景列表，逐步驗證 callback dt 和 manager 時間增量，不猜測倍數。
- custom PhysX 2D 掃描從 app/render polling 移至 PHYSICS_POST_STEP；保留輪式 odom 時間插值，新增有界佇列丟棄計數。SLAM worker 留在背景，不在 callback 執行配準／規劃。
- primary PhysX raycast 不再強制 import／啟用 RTX。只有顯式啟用 RTX comparison 時才載入它。
- 光達 local xform ops 明確求值，避免缺少 OmniLidar schema 的離線 USD 把安裝外參當成零。
- 新版 vehicle contract 寫入地圖 metadata；不同或缺少 contract 的舊圖在 estimator 載入前拒絕。
- wheel contact tensor 記錄改為選用且失敗不阻斷啟動，預設關閉，不為診斷強迫新增 ContactReportAPI 或猜 floor 路徑。輪速／車身／時間記錄仍保留。
- 新增獨立 `V66_VehicleAudit.py`：僅讀設定與已附接 view，支援 Stop/Run/Play 及已在 Play 時附接；預設 10 模擬秒，自動或手動匯出。

## 保留／尚未驗證
- 不是更換 LiDAR 後端：仍是碰撞 raycast、720 beams、10 Hz 目標、max range 12 m。
- 新資產質量／慣量／摩擦是先前交付的模擬基準，未根據實車校正。光達 mount optical plane 未實測；D555 仍僅幾何。
- 2D 掃描高度外的障礙、無 Collider 的可見家具、差速車輪滑移不會因移植自動解決。
- V66 控制 USD 的跨 reference targetPrim 可被 USD 忽略；robotPath 字串仍有效。本版驗證該路徑並在 Play 後檢查真實求值的 jointNames，不新增第二個 command writer。
- 未實測：Isaac Sim 6.0.1-rc.7 的事件／tensor API 相容性、穩定接地、直行、0.9 m/s 達速、地圖閉環品質、A→B 抵達、RTF 改善。
- 原 package_new 與原 V66 USD 保持原檔，不把舊地圖與舊 wheel_fix/session overrides 搬入新套件。
