# V66 Continuous Navigation 2.0 — 2026-11-07

純 Python、差速車、無 ROS2／IMU。以已實跑 V66 Smooth1.1 來源和1.1.1停穩計時修正為基底。這是完整程式覆蓋包，不是車體USD或地圖。

## 安裝：ZIP 內的檔案直接放在原資料夾

1. Isaac Sim 按左側■STOP，等舊導航清理／匯出完成。
2. 備份原本執行資料夾內的 Python／JSON 程式檔。不要移動或覆蓋 slam_maps、slam_outputs、validation_outputs。
3. 將本 ZIP 解壓的全部檔案直接上傳／覆蓋到：

   /isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/new_car/python_nav/V66_Python_Navigation/

   地址結尾只有一次 V66_Python_Navigation。ZIP 是平鋪檔案，不要再包同名子資料夾。
4. 可在 Script Editor RUN CHECK_Installed_2_0.py，看到 INSTALLATION HASH CHECK PASS，確認沒有新舊模組混用。
5. 若場景重開、希望沿用之前的候選萬向輪材質，先在STOP狀態RUN V66_CasterContactCandidate.py（APPLY=True）。本版本不自動改物理材質。
6. 重新從正確外層目錄開啟 AMR_V66_Navigation.py，RUN→PLAY。確認面板與輸出版本 v66-nav-2.0-continuous-cg。

入口名稱和遠端PROJECT地址不變，沿用舊車體與控制鏈。新增的五個模組不能漏傳：path_geometry.py、trajectory_optimization.py、speed_profile.py、tracking_safety.py，以及本包的path_smoothing.py相容介面（同名覆蓋）。

## 使用

- 已有可用V66地圖：Load saved map → 等定位完成 → Create/select B，拖XY並旋轉紅箭頭 → Preview → 看路徑與訊息 → Navigate。
- 想重新建圖：Start NEW map → 遙控掃描 → Finish mapping/use candidate → 檢查後Confirm candidate。勿因更新程式就覆蓋原本可用地圖。
- A仍是SLAM估計，不讀世界位置定位。B仍是世界XY與最終朝向，忽略高度／傾斜。
- 綠=全域；黃=反應後煞車的局部預測；藍=SLAM估計軌跡；紅箭頭=B最終朝向。
- Preview輸出 search_mode=continuous_first、spins=0 表示本次找到無原地旋轉的規劃。spin_fallback 表示連續方案在設定搜尋預算內未找到，退回允許旋轉；不是證明連續路線絕對不存在。
- 碰撞、定位失效或必須前後換向仍可停車。不要把「連續過彎」理解為任何情況都不能停止。
- 選定方向後手動操作仍無航向補償。手動直行偏擺並不是本版本修復目標。

## 本版新增

1. 先在不含原地旋轉的差速動作集合搜尋；預算不足再明確退回允許spin的搜尋。前進／倒退仍支援，必要換向必須停穩。
2. 增加原地旋轉段起始代價、換向和曲率變化代價。
3. 以五次曲線作連續終點候選，限制曲率、保留起終點朝向，檢查完整矩形footprint。
4. Voronoi代價從參考點改成四角最大值。代價不是碰撞檢查的替代品。
5. 同方向行駛段使用CG數值最佳化，再插入中間點做第二次CG。目標包含Voronoi、障礙距離、曲率超限、平滑、長度及偏離種子項。最後插值並密集檢查；不合格則逐級退回已檢查方案。
6. 路徑速度包絡依曲率、角速度、側向加速度及軟體輪速限制，加上前向加速／反向煞車掃描。RPP在彎前受到速度包絡限制，不只到折點才減速。
7. 局部預測從長時間等速外推改成「反應時間＋煞車」模型，保留當下odom速度，不能靠縮小命令假裝慣性消失。此模型的減速度仍是工程設定，非實車／PhysX量測保證。
8. 保留1.1.1重規劃停穩計時修正；每次重規劃重新起算8秒。
9. 匯出hybrid_path含名義v/w/dt；planner_stats含搜尋模式、spin數、CG接受／退回及速度包絡，便於判讀綠線。

## 參數與代價

| 項目 | 值 |
|---|---:|
| 手動最大v／w | .9 m/s／.9 rad/s |
| 手動加速度／減速度 | .3／.5 m/s² |
| 導航最大v／w | .9 m/s／.6 rad/s |
| 導航加速度／減速度 | .3／.4 m/s² |
| 導航角加速度 | .8 rad/s² |
| 導航原地轉向上限 | .45 rad/s |
| RPP前視 | 速度相關 .35–.65 m，時間係數1.0 s |
| 轉向濾波 | .08 s |
| 軟體導航輪速上限 | 12 rad/s（候選包絡，不是廠商馬達額定值；不修改Joint Drive） |
| 側向加速度候選上限 | .30 m/s² |
| 路徑曲率上限 | 2.0 m⁻¹ |
| 搜尋nominal v／w | .4 m/s／.6 rad/s |
| 搜尋XY／yaw分箱 | .1 m／72格 |
| 地圖解析度／margin | .05 m／.04 m（保留） |
| Voronoi權重／alpha／有效距離 | 3.0／.4／1.0 m |
| spin起始／換向／動作變化代價 | .8／1.0／.12 |
| 加權啟發式係數 | 1.2（不宣稱最優或可採納啟發式） |
| 搜尋總預算／連續階段比例 | 45秒wall time／60% |
| CG總候選預算 | 5秒wall time；另有最後幾何驗證時間 |
| CG粗／細最大迭代 | 45／25（可能提早逾時或精度損失，不保證收斂） |
| CG長度／Voronoi／障礙／曲率超限／平滑／種子偏移權重 | .15／.6／80／12／.35／.25 |
| 局部反應時間 | .15 s（候選假設） |
| 抵達XY／yaw容許值 | .07 m／5°，另須驗證停穩 |

## 論文對照與未實作項目

- Dolgov等2008：採Hybrid連續姿態搜尋、Voronoi場及CG平滑思路；本版用差速運動基元、數值差分梯度與檢查式插值，是改寫，不是逐式完整重現。
- Meng等2023：加入四角最大Voronoi代價；不包含該文的終點可達中間姿態集合與分階段動態選點。
- 不直接使用汽車Reeds–Shepp；保留差速原地旋轉能力。雙啟發式查表尚未重現，本版仍為加權格網引導搜尋。
- 路徑平滑的軟代價不能證明無碰撞，接受前需取樣footprint檢查；取樣仍非連續CCD或動力學可行性證明。
- 連續方案可能比「停車旋轉＋倒退」更長。起點與B方向受限時，會需要提早繞出一段弧線，這是明確取捨，不保證最短。

## 已做的離線驗證

使用mapping_session_1793978783186786405.zip中的同一地圖、起點、B方向（不含之後新增的即時障礙）：

- 舊路徑保留3個spin段（約67°、129°、11.6°）及一次換向。
- 新路徑約6.03m，找到單一前進行駛段、0個規劃spin、0次換向；取樣全車碰撞檢查通過。
- CG粗階段改善目標值，細階段候選未通過全部接受條件，實際接受粗CG的檢查式插值；不把「呼叫CG」說成兩階段皆已收斂。
- 理想差速運動學RPP回放到達；途中無停車旋轉，最大參考線偏差約1.6cm。此回放沒有真實輪地接觸、滑移、SLAM漂移或Isaac排程。
- 另檢查空曠90°彎、必要原地轉向退回、不可行終點拒絕、停穩交接、局部障礙／煞車碰撞拒絕、輪速上限、Script Editor分離命名空間啟動及重規劃計時。
- 沒有修改的isaac_io、slam_core、occupancy、v66_control、v66_io、v66_bootstrap、VehicleAudit與上一包實跑來源逐位元一致。

完整結果見OFFLINE_VALIDATION.json、INTEGRATION_CHECKS.json和offline_compare.png。Isaac Sim實跑仍待你驗證。第一趟先Preview觀察，再用空曠短程B點導航；不要把本版宣稱為輪地物理問題已修復。

## 材質及舊版還原

本版不自動套用或永久寫入候選摩擦。隨附V66_CasterContactCandidate.py僅方便原本使用者在重開場景後另行套用；它修改Session層，不是新版導航的必需元件。

若回退：STOP→將先前備份的原程式與原config放回原目錄→重新開啟舊入口RUN。新模組可留在目錄，舊入口不會載入；地圖不需因回退程式而刪除。不要把新舊核心檔案混搭。

## 參考

- Dolgov et al., Practical Search Techniques in Path Planning for Autonomous Driving, 2008，使用者提供PDF。
- Meng et al., Improved Hybrid A-Star Algorithm for Path Planning in Autonomous Parking System Based on Multi-stage Dynamic Optimization, IJAT24, 459–468, 2023, DOI10.1007/s12239-023-0038-1，使用者提供PDF。
- SciPy CG API: https://docs.scipy.org/doc/scipy/reference/optimize.minimize-cg.html
- SciPy場插值API: https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.map_coordinates.html
