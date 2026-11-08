# V6.6 導航移植版 1.0（2026-11-06）

這版以 `package_new` 為來源，獨立移植到已製作的 V6.6 車體。保留純 Python、無 ROS2、無 IMU、輪式 odom＋2D 掃描 SLAM、差速 Hybrid A*＋Voronoi field、RPP、拖曳 B 點及地板路徑顯示。原 `package_new` 沒有改動。

**已完成程式移植與離線核對；尚未在你的 Isaac Sim 6.0.1-rc.7 執行。本包不能保證物理直行、地圖品質或導航驗收已通過。** 手動直行沒有加入航向補償；若等輪速仍偏行，應根據同次記錄檢查輪組接觸／滑移，不能由程式版本號推定已修好。

## 1. 要上傳哪些檔案

把整個 `V66_Python_Navigation` 資料夾上傳到：

```text
/isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/python_nav/V66_Python_Navigation/
```

核心 `.py`、`v66_nav_config.json` 必須放在同一層。不要放回舊 V10.5 資料夾覆蓋共用檔，也不要執行舊 V8～V10.5 的修復入口。

| 檔案 | 使用方式 |
|---|---|
| `AMR_V66_Navigation.py` | 唯一導航入口：Script Editor 開啟它，Stop → Run → Play |
| `v66_nav_config.json` | 新車幾何、輪式 odom、速度、規劃器及 RPP 設定 |
| `V66_VehicleAudit.py` | 可單獨執行的唯讀核對程式；也是導航啟動核對所用的資料來源 |
| `v66_app.py`、`v66_bootstrap.py`、`v66_control.py`、`v66_io.py` | 狀態機、啟動核對、手動控制及記錄；不分開執行 |
| `isaac_io.py`、`slam_core.py`、`occupancy.py`、`amr_planner_core.py`、`rpp_core.py` | 感測／odom、SLAM、格網、規劃及追蹤；不分開執行 |
| `test_v66_smoke.py` | 開發端離線檢查，使用者不用在 Isaac Sim 執行 |

若遠端資料夾不同，修改入口與獨立核對檔最上面的 `PROJECT`。如果場景 Prim 名稱不同，再修改設定檔 `paths` 及 `encoders.articulation_path`；獨立核對檔的 `BASE`、`CONTROLLER`、`LIDAR` 也要對應。

本包沒有再次附上 40 MB 的車體，沿用已交付的 `AGV_configured_v66.usd`、`AGV_control_v66.usd`。主場景外層名稱仍保留：

```text
/World/AGV_configured_v4
/World/AGV_control
```

它們的 Payload／Reference 應分別指向新版兩個 USD。`AGV_configured_v4` 只是保留的場景節點名稱，不代表仍使用舊幾何。

## 2. 新車的有效參數

| 項目 | 舊套件 | 本版 |
|---|---:|---:|
| 驅動輪半徑 | 0.09 m | **0.08 m**（直徑 0.16 m） |
| 驅動輪中心距 | 0.514 m | **0.4769668332689978 m** |
| 規劃外框長×寬 | 0.88×0.56 m | **0.898×0.5698 m** |
| 整體高度設定 | 0.45 m | **0.444 m** |
| 車殼長×寬×高（獨立核對） | 非本版依據 | **0.810×0.560×0.250 m** |
| 萬向輪半徑／胎寬 | 舊修復 0.03／0.025 m | **0.0315／0.019262928306058 m**，唯讀檢查既有碰撞 |
| 光達安裝點，相對 base_link | 舊安裝點 | **(-0.23, 0, 0.345) m**，光學平面高度仍待實測確認 |
| 車頭／線命令／角命令符號 | −X／−1／+1 | 保留 |
| 邏輯左輪／右輪 | 舊有對應 | `right_wheel_joint`／`left_wheel_joint`，encoder 皆乘 −1 |

**規劃外框與車殼尺寸不同是刻意的。** 前後保險桿、輪胎外緣也需要空間，不能只用 810×560 mm 的車殼當完整行車外框。新車輪距由兩驅動輪的中心測得，不用車寬替代。

控制器陣列順序仍為舊 +X 命名框架的 `[left_wheel_joint, right_wheel_joint]`。EncoderReader 會依實際車頭 −X 換成邏輯左右；不要再手動交換一次。

## 3. 先做單獨核對（不自動開車）

1. 開啟已置換 V6.6 的病房場景。關閉舊控制面板，先按 Stop。
2. Script Editor 開啟 **`V66_VehicleAudit.py` → Run**。
3. 面板先顯示靜態核對結果，再按 Play。它會等待 Isaac 已附接的物理 view，讀取實際關節與車身資料；**不自己建立第二套 simulation view，不發送速度、不移動物件、不修改 USD**。
4. 預設記錄 **10 個模擬秒**。環境很慢時，可隨時按 `Export now / end read-only recording`，不必等實際幾分鐘。這個按鈕只結束記錄；如果另外用手動面板開車，仍須自行按手動面板 STOP。
5. 顯示 `AUDIT FINISHED` 後取回 ZIP。它不會把模擬按 Stop，也不會影響既有導航控制，因此也能在另一個面板執行期間唯讀觀察。

輸出目錄：

```text
V66_Python_Navigation/validation_outputs/vehicle_audit_時間碼/
V66_Python_Navigation/validation_outputs/vehicle_audit_時間碼.zip
```

核對內容：車殼與完整外框、六輪有效碰撞數量、原生 Cylinder 尺寸、驅動輪心距與軸線、六輪靜態輪底高度差、11 剛體／10 關節、關節兩端錨點與軸向、驅動模式、萬向輪是否被加了馬達、材質摩擦、質量／慣量、感測器外參、控制器輪徑／輪距／目標、有效 PhysicsScene、暫存層。

Play 後另外讀取：實際全部 DOF 角度與角速度、輪速 targets（API 可用時）、控制器輸出、車身位姿與速度、六輪剛體世界位姿及速度（API 可用時）、官方模擬時鐘、callback dt、RTF。任何缺失都記成 unavailable/error，不當成零值。

`vehicle_static.json` 的 PASS 是**幾何與接線核對**。六輪輪底同平面不代表六輪已量測到地面負載；這個獨立程式沒有新增 ContactReportAPI。質量、重心、慣量與摩擦仍是新版 USD 的模擬基準，不代表實車量測值。

## 4. 正式移動、建圖、導航：只跑一個入口

1. 按 Stop，關閉舊面板與舊修復程式。開啟 **`AMR_V66_Navigation.py` → Run → Play**。
2. 啟動核對不通過時，讀錯誤及獨立核對 ZIP；不要降低檢查條件或重跑舊修復去蓋過新版。
3. 等 `Manual ready`，試前進、停止、後退、停止、左轉、停止、右轉、停止。上限為 **0.9 m/s、0.9 rad/s**，起步加速度仍為 0.3 m/s²、角加速度 1.2 rad/s²。旋轉到直行／換方向時先確認停穩。
4. 前進目標是 `(v,w)=(+0.9,0)`，後退 `(-0.9,0)`；左右原地旋轉 `(0,+0.9)`、`(0,-0.9)`。沒有直行 PD、固定偏壓或偷偷加入的航向修正。Joint Drive 原本追蹤轉速的功能仍保留。
5. 手動只要求輪組介面與新鮮局部掃描，不要求 SLAM 已定位。局部掃描被擋、過期或傾斜過大會明確停車；程式不自動把速度反覆縮小來試探。
6. 移動正常後停止，按 **Start NEW map**，再遙控繞行，從不同角度掃牆與設備，並回訪走過的位置。
7. 停穩且有新鮮成功配準後，按 **Finish mapping / use candidate**。本次候選圖被保存並凍結，可直接進入導航。
8. 要正式保留該圖，再按 **Confirm candidate / archive old / save**。兩個按鈕用途不同；Stop/export 只匯出本次資料，不會代替確認地圖。
9. 按 **Create / select B**，拖曳 `/World/PythonNavV66/GoalB` 的 XY，繞 Z 旋轉箭頭設定最終車頭方向。忽略高度及傾斜；面板顯示世界／地圖 XY 與 yaw。A 自動來自當下 SLAM 估計，不拖曳 A。
10. 按 **Preview Hybrid A* + Voronoi** 查看綠色全域路徑，再按 **Navigate A → B with RPP**。黃色是局部預測，藍色是估計軌跡，紅色是 B 最終方向。
11. 只有 SLAM 估計的位置誤差 ≤7 cm、方向誤差 ≤5°，並確認輪子與車身停穩才回報抵達。這不是實車或世界真值導航精度認證。
12. 結束按 **Stop / export this session ZIP**，等待停止確認與匯出完成。

導航輸出仍在本包的 `slam_outputs/mapping_session_時間碼.zip`；正式地圖只在本包的 `slam_maps`。舊套件的 `slam_maps`、session layers、`wheel_fix_outputs` 沒有帶進新套件。

## 5. 感測、時間與控制邊界

控制鏈為 Python 邏輯 `(v,w)` → 符號轉換 → 既有 DifferentialController → 既有 ArticulationController → 車輪。唯一馬達寫入者在物理 PRE_STEP。UI 只設定目標，不再另外驅動輪子。入口會關閉已知的 V66 手動面板與舊導航 task，但不能識別所有外部未知程式；請勿並行其他控制腳本。

輪式 odom 每個物理 POST_STEP 讀實際輪關節角度積分。模擬 2D 掃描也在 POST_STEP 排程，目標 720 束／10 Hz，先讀物理碰撞幾何，再用同一時鐘下的 odom 歷史插值配對。SLAM 處理節流仍為 0.2 模擬秒（最多 5 Hz），因此「10 Hz 原始掃描」與「5 Hz SLAM 提交上限」是不同的。掃描佇列最多 20 筆，丟棄計數會匯出；這是負載可觀測性，不是保證處理所有掃描。

**這仍是你原套件使用的 custom PhysX collision raycast。** `Lidar_2D` 提供安裝外參；本版沒有改成消費 RTX OmniLidar 的點雲，也沒有新加入真實硬體驅動。只有 Collider 能被這個感測器看到。只有視覺、沒有碰撞的家具不能當作已驗證能避開；單一掃描平面也可能看不到高低突出物。D555 保持幾何模型，沒有暗中接成深度感測器。

世界位姿只用於感測器模擬、地板繪圖／B 點世界與地圖座標的固定對齊，以及診斷／停穩安全檢查；不提供 SLAM 車輛位置，也不拿來補償直行。

PhysicsScene 沿用場景現值，`physics_hz=null` 表示讀取現有值。**不再強制設定 240 Hz、不更動 RenderingManager dt、不自動建立或停用 PhysicsScene**。有兩個有效場景時拒絕啟動，列出路徑。Play 後逐步比對 callback dt 與官方 manager 時間增量，不用乘二／除二掩蓋差異。Render 與 physics 的更新設定是分開的，參考 [Isaac Sim 官方 SimulationManager API](https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.core.simulation_manager/docs/index.html)。這是保留現況，並不保證 FPS 或 RTF 提升。

車輪換算使用輪半徑與兩輪中心距，graph 的輪速單位是 rad/s。綁定可由 `robotPath` 或 `targetPrim` 指定，參考 [官方 OmniGraph 控制器教學](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/omnigraph/omnigraph_tutorial.html)。目前 V66 控制 USD 在參照組合時可能因 USD scope 忽略跨資產的 targetPrim；本包明確核對 `robotPath` 及 Play 後求值的 jointNames，不把空 relation 誤判成已正確連接。文件與 API 存在版本差異，6.0.1-rc.7 實跑仍需核對。

## 6. 全域規劃、局部避障參數

`v66_nav_config.json` 的 `planner` 暴露原有搜尋預設值，這次沒有重新調控制增益。

| 類別 | 本版設定 |
|---|---|
| 地圖 | 解析度 0.05 m、範圍 24 m、未知區域視為阻擋 |
| 完整矩形 footprint | 長 0.898 m、寬 0.5698 m，額外 margin 0.04 m |
| Hybrid A* | XY bin 0.10 m、72 yaw bins、基元 0.65 s、名義 v 0.4 m/s／w 0.9 rad/s、碰撞採樣 0.025 m |
| 行為 | 前進、倒退、原地旋轉；倒退係數 1.3、切換成本 0.15、控制變更成本 0.03 |
| Voronoi field | weight 3.0、alpha 0.4、作用距離 1.0 m；不是取代 footprint 碰撞 |
| 搜尋上限 | 60,000 expansions、45 真實秒、heuristic weight 1.2 |
| RPP | 上限 v 0.9 m/s／w 0.9 rad/s，lookahead 0.55～0.9 m、時間 1.3 s |
| RPP 調速／碰撞 | 曲率調速、接近目標調速、距障調速、0.6 s 預測、必要時停車後重規劃 |
| 抵達 | 位置 0.07 m、方向 0.0872664626 rad（5°），另驗證停穩 |

這是自製的格網＋距離／Voronoi 代價場＋局部掃描疊加，不是 Nav2 的 Costmap2D 或官方 Nav2 RPP plugin。RPP 的路徑追蹤轉向仍保留，與手動模式移除的直行補償不同。

## 7. 報告與問題對照

| 現象／檔案 | 能檢查的內容與下一步 |
|---|---|
| 啟動失敗／`vehicle_preflight.json` 或獨立 `vehicle_static.json` | 舊輪徑、輪距、重複碰撞、關節錨點、控制器目標、重複 PhysicsScene；先修列出的項目 |
| 直行偏左／偏右／`control_tracking_v66.csv`、`control_diagnostics.json` | 先比 graph 左右目標、實際左右輪速與車身偏轉。目標不同查命令／接線；目標相同但實際輪速不同查 Drive／接觸；輪速相同仍偏查滑移、萬向輪阻力與載重 |
| `STOP: local 2D scan blocks swept footprint` | `manual_stops.json` 記錄阻擋點及 Prim；這是局部預測停車，不直接等於已實際撞到 |
| 掃描過期 vs 匹配拒絕 | 前者看時間戳／佇列；後者看 `consumed_slam_inputs.npz` 的 RMSE、accepted、odom、scan。拒絕掃描不寫入圖 |
| `Cannot finish` | 需要當前正在建新圖且最近成功配準；已凍結候選圖不能反覆 Finish，需要 Confirm 或直接設 B |
| 載圖被拒 | 車型 contract、輪徑／輪距、外框、場景識別或感測器原點不同。不要直接複製舊圖冒充新圖 |
| RTF 很低 | 核對錄到的模擬秒／真實秒與單步 dt；提高車速不能修復運算延遲 |

本版不修改既有 USD 的輪徑、輪距、輪位、材質、質量／慣量、Drive 最大力矩／增益、萬向輪碰撞與阻尼、求解器迭代。唯一暫存 USD 編輯是沿用 custom ray sensor 所需的 scene-query support，以及使用者建立／拖曳 GoalB；結束時只移除這次建立的感測器暫存層，不儲存 USD，也不清掉別人的 layer。

離線測試結果在 `validation.json`；幾何／接線核對在 `offline_asset_audit.json`。這兩份都明確標示不是 Isaac Sim 實跑結果。
