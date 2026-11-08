# 純 Python 實驗版：Encoder Odom + 2D SLAM + Hybrid A* / Voronoi + RPP

## 先知道這版的定位來源

這版不再用 Isaac Sim 的車身世界位置當作導航定位，也不使用病房 USD 障礙幾何補地圖。
定位輸入只有「PhysX 實際驅動輪轉角」和「內建光達」。不是速度指令積分。

世界座標只有兩個視覺用途：
1. 啟動時一次性讀取安裝關係與場景顯示對齊，將 map 原點畫到場景地板。
2. 讀取你拖曳的 B / InitialPose 標記，轉成使用者指定的 map 座標。
SLAM 本身從 map=(0,0,0) 開始；不接收 USD 的當前車身位置、障礙真值或實際移動距離。

這是供學習/驗證的自製實驗 SLAM 和 RPP，不是 ROS 2/Nav2/SLAM Toolbox 的正式移植。
沒有聲稱可直接上實車或已在你的 Isaac Sim 主機驗證。

## 一、上傳與執行

把本資料夾內容上傳到：

    /isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/python_nav/

不要多包一層 python_nav/python_nav。舊的掃描圖 maps/ 不會當成 SLAM 圖讀取。
主要檔案：

- AMR_SLAM_RPP.py：唯一正常啟動入口。
- isaac_slam_app.py：面板、資料流程、導航狀態與保護。
- isaac_io.py：實際輪角讀取、光達讀取、僅視覺用途的場景對齊。
- slam_core.py：里程計、ICP 掃描匹配、迴環驗證、姿態圖校正、固定地圖定位。
- occupancy.py：未知/障礙/自由空間格網與 PNG/PGM/NPZ 輸出。
- amr_planner_core.py：差速 Hybrid A* + Voronoi 場與 2D 障礙啟發式。
- rpp_core.py：RPP-style 路徑追蹤、調速、預測碰撞。
- slam_config.json：場景路徑、輪子 DOF、輪距/輪徑、速度與保護參數。

在 Script Editor 開 AMR_SLAM_RPP.py：Stop → Run → Play。
它會等至少 3 秒模擬時間及 1 秒輪角安靜，再初始化輪速里程計。
這不是用車身真值高度判斷落地，請同時目視確認車子真的已落在地板。
舊 AMR_TeleopMap.py、AGV_Navigate.py、AMR_Hybrid_Navigate_Isaac.py 請不要同時執行。
啟動時會取消已知舊 task；如果先前用其他 namespace 執行過控制器，仍請先停止它。

目前畫面中 AGV 的 base_link 實際路徑是 `/World/AGV_configured_v4/TZAGV_B02_vendor_v4/TZAGV_B02_root/base_link`，已寫入 slam_config.json。床邊手臂也有同名 base_link，所以不要清空這個明確路徑讓程式全場搜尋。若場景更換，先從錯誤訊息找出 AGV 專屬 base_link、Lidar_2D 與 DifferentialController 路徑，再更新 `paths` 三個欄位。

如果資料夾不同，只需修改 AMR_SLAM_RPP.py 的 PROJECT；唯讀檢查入口另有自己的 PROJECT。
Isaac Python 需有 NumPy、SciPy；不需要 ROS 2、Nav2 或外部 SLAM 執行檔。
Matplotlib 只有本機合成驗證腳本需要，Isaac 執行入口不會匯入它。

## 二、第一次先確認輪子，而不是直接導航

啟動時會列出：

    Actual encoder DOFs: [...]
    Logical front-left: <DOF name> <sign>
    front-right: <DOF name> <sign>

自動尋找必須唯一，不會猜 DOF 索引。找不到時會停止，不會退回速度指令或 USD 位姿當作里程計。
必要時在 Play 後執行唯讀 AMR_InspectWheelDOF.py，將輸出貼回來確認。

模型車長/車寬 0.88/0.56 m、輪半徑 0.09 m、左右驅動輪中心距 0.514 m。
輪距不是車寬。這組仍是模型值，不是重新量測實車。

原 base_link 車頭在 -X（光達端）；邏輯 front +X 是面向光達：
- 邏輯左輪對應原模型的 right 驅動輪名稱，邏輯右輪對應原 left 名稱。
- 預設兩個實際輪角乘 -1，正向轉動對應物理車頭前進。
- 控制器線速度指令預設乘 -1，角速度同號。

這些符號必須在你的關節軸向下低速驗證：前進時 Odom v 應為正，左轉 Odom w 應為正。
若不符，先停車確認 slam_config.json encoders 的對應與 sign，不要把控制符號全部亂翻。
控制器本身的輪徑/輪距設定仍需核對，本版不自動修改場景控制器、關節或物理參數。

## 三、鍵盤掃描建圖

1. 等面板顯示 SLAM 初始化完成；初始 map 姿態為 (0,0,0)。
2. 按 Enable teleop / scan，再點串流的場景視窗。
3. W/S 前進/倒退，A/D 左右轉。放開停止，空白鍵或 STOP / disarm 停車。
4. 鍵盤沒傳到遠端時，用面板 Forward/Reverse/Left/Right 短脈衝按鈕。
5. 慢速繞過可通行區，從不同位置觀察床邊/門口/牆面，最後回到曾走過的位置，測試迴環。
6. 按 Save SLAM map 儲存。此按鈕會停車再存圖，SLAM 定位後續仍會更新。

預設遙控 0.20 m/s、0.45 rad/s，鍵盤更新期限 0.75 秒牆鐘時間。
若 WebRTC 不傳 KEY_REPEAT，長按可能斷續，請放開再按或使用短脈衝按鈕。
資料過期、匹配失敗或近距離回波阻擋時，程式不允許持續行駛。
不能保證光達自車反射/重建模型回波已被解決；出現停車應看診斷，而不是關掉保護。

面板顯示 Odom XY、SLAM XY、關鍵幀數、迴環數、匹配 RMSE 與定位健康狀態。
一開始兩個位置接近，行駛後可以觀察漂移與校正；不能只看 RMSE 一個數字判定定位正確。

## 四、用場景拖曳 B 點與 RPP 導航

1. 停車，按 Create / select B。這會取消鍵盤輸入。
2. Stage 選到 /World/PythonNav/GoalB；用 Move 工具拖到要去的地板位置。
3. A 是車子當下的「SLAM 估計姿態」，不用再輸入 A。
4. 按 Preview Hybrid path，先看綠線。
5. 按 Navigate with RPP，才開始追蹤。

若 goal_yaw_from_marker=false，終點朝向使用 A→B 方向。
若設 true，可用 Rotate 工具轉 B 的局部 +X，指定物理車頭的終點朝向。
標記只有 Xform，不是雷射/碰撞用實體；拖曳 B 不會新增障礙。

場景地板上的顏色：
- 綠：SLAM map 座標下的全域規劃路徑。
- 黃：RPP 碰撞預測的局部軌跡（包含減速時的量測速度保守投影）。
- 藍：SLAM 估計行駛軌跡，不能當作世界真值軌跡。
- 紅：B。

線畫在初始地板對齊高度上方約 2.5 cm。不是新增碰撞 Mesh，也不是永久保存的 USD 路徑模型。
顯示對齊是啟動時的固定參考；若 SLAM 漂移，顯示也可能偏離真實模型，不能偷偷用世界位置修正。
Debug Draw 是共用頻道，本版刷新路徑時會清除共用的 debug 線/點。

## 五、全域、局部與重新規劃的分工

全域：
- 地圖只由 SLAM 回波生成；未知/不確定格子不可通行。
- 差速 Hybrid A*，考慮 x/y/yaw、前進/倒退、弧線與原地旋轉。
- 使用障礙距離 dO、離散近似 Voronoi 邊距離 dV 的場成本。
- 新增 2D 障礙感知啟發式加速，不把 Dijkstra 路徑當作最終輸出。
- 完整矩形車身與沿途掃掠採樣碰撞檢查；沒有用汽車轉向角或最小轉彎半徑。

局部：
- RPP-style 自製 Python 控制器：速度自適應前視點、曲率調速、接近障礙調速、終點減速。
- 自訂分段狀態機處理原地轉與前後換向，不直接複製 Nav2 的互斥參數組合。
- 最新 0.75 秒回波在 odom 座標中保留並轉到目前 map，作為局部障礙層。
- 這是點集合形式的局部障礙層，不是完整重現 Nav2 rolling costmap 的所有 layer。
- 以完整矩形 footprint 預測碰撞；量測輪速高於新命令時，使用較高速保守投影。

若阻擋：先停止，再把最新回波加入規劃快照，重新呼叫 Hybrid A*。
最多連續三次阻擋重規劃，仍無進展就停止；沒有預測行人意圖/速度或保證能繞過動態人群。
迴環造成 map→odom 改變時，會停止並重規劃，避免沿舊路徑繼續跑。
規劃/SLAM 在工作執行緒處理；USD、鍵盤及馬達輸出只由主執行緒管理。

## 六、SLAM 實際做了什麼

- encoder delta → 差速精確積分，得到連續 odom→base_link。
- 用 map→odom 修正形成掃描匹配的預測。
- 帶距離/重疊率/修正幅度門檻的 trimmed weighted point-to-point ICP，對齊最近關鍵幀子地圖。
- 累積占據機率地圖。缺回波的方向維持未知，不要求回波一定涵蓋 360°。
- 空間接近、時間相隔足夠的舊關鍵幀作迴環候選，要求連續兩個候選一致及幾何匹配門檻。
- 稀疏 least-squares 姿態圖，固定第一幀，使用 robust loss。
- 圖校正後重新積分已存掃描重建地圖，不能只移動路徑卻留下扭曲地圖。

這些是可執行的實驗實作，但不是成熟 SLAM 系統：重複病床、長走廊、動態遮擋、
低特徵環境仍可能產生退化或錯誤迴環；門檻式健康檢查不是完整機率信心評估。
里程計與匹配約束的完整相關性、協方差與參數尚未做實際校正。

時間：光達 header timestamp 首幀與模擬時間作固定偏移對齊，再在輪角里程計歷史中插值。
如果 timestamp 後續不跟隨模擬時間、輪角讀取間隔太長或資料過期，會中止。
固定未知延遲與逐射線運動補償尚未完整校正，不能聲稱已達實車等級的同步。

## 七、存圖與下次載入

新輸出位於 python_nav/slam_maps：
- room_map_latest.png / .pgm：白自由、黑障礙、灰未知/不確定。
- room_map_latest.npz：原始格網與 metadata。
- slam_graph.npz：掃描、關鍵幀、邊與姿態圖，不使用 pickle。
- estimated_poses.csv：模擬時間、odom 姿態、map 估計姿態、odom v/w。
- frame_contract.json：map→odom 與僅視覺用途的初始對齊。

每個新 session 第一次存圖若有舊圖，先備份到 slam_maps/archive/<timestamp>/。
按 Stop 重置物理狀態後，必須重新 Run，因輪角里程計原點已重置。

使用舊圖：
1. 新啟動一個 session、Play，初始化感測器。
2. 按 Load saved map。
3. Stage 會選到 InitialPose 標記。手動拖到目前車子附近，Rotate 標記 +X 朝光達車頭。
4. 按 Use dragged initial pose。它只是給匹配器一個近似起點，不是直接當成定位成功。
5. 等掃描匹配通過，再選 B 與導航。

載入模式是「固定已知地圖上的掃描匹配定位」，不是載入後繼續擴建姿態圖。
若要繼續探索新區，先儲存舊資料，再重新啟動新建圖 session；本版不支援跨 session 多圖合併。
場景檔、光達安裝或尺度改變時會拒絕沿用舊圖，不能任意略過配準檢查。

停車診斷在 python_nav/slam_outputs/stop_diagnostic.npz，規劃結果在 hybrid_path.npz。
若出錯，回傳 Script Editor 輸出與這份新診斷，舊 AGV_StopDiagnostic.npz 格式不相同。

## 八、驗證狀態

- 30 項離線單元測試通過：Odom、ICP、姿態圖、存取定位、時間插值、RPP、占據圖與規劃。
- 合成光達＋左右輪 ±0.5% 偏差測試：129 幀、83 關鍵幀、4 次幾何迴環。
  最後 Odom 位置誤差約 0.702 m，線上 SLAM 約 0.0152 m。
- 同一張合成 SLAM 地圖接 Hybrid A* / Voronoi + 理想差速 RPP：601 次搜尋展開、
  601 個 0.05 s 控制步，終點位置誤差約 0.0696 m、朝向誤差約 4.89°，符合本次設定。
- 合成資料中使用真值只是為了產生測試雷射與計算誤差，不是 runtime 的定位來源。
- 這些結果不代表你的病房、輪子關節、WebRTC、雷射回波或物理驅動已通過。
- 版本仍需在 Isaac Sim 驗證；之前的感測器誤停不保證因換演算法就消失。

本機重現（需要 numpy、scipy；圖表另外需要 matplotlib）：

    python -m unittest discover -s python_nav -p "test_*.py" -v
    python python_nav/offline_validation.py
    python python_nav/offline_navigation_validation.py

預存 validation_outputs/ 的圖片與數字全部標示 SYNTHETIC，請勿當成 Isaac Sim 執行截圖。

## 參考介面

PhysX actual DOF positions:
https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/extensions/runtime/source/omni.physics.tensors/docs/api/python.html
RPP reference (本版只實作相同核心概念，不是 C++ 插件移植):
https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/
Voronoi field / Hybrid search reference:
https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf
