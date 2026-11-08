# python_nav：遙控掃描、拖曳目標、Hybrid A* + Voronoi 導航

這是純 Python 的 Isaac Sim 工具組，不用 ROS 2。先建圖、確認覆蓋範圍，再選 B 與導航。
目前做的是「用模擬器已知的車身世界姿態拼接光達」，不是 SLAM。
22 項離線單元測試通過，但未在你的新場景實際操作 WebRTC / UI / 物理引擎。

## 上傳位置（依你截圖的專案目錄）

把本資料夾內的檔案上傳到：

    /isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/python_nav/

請勿再多包一層 python_nav/python_nav。
遠端至少要有：

    AMR_TeleopMap.py
    AMR_Hybrid_Navigate_Isaac.py
    amr_planner_core.py
    nav_toolkit.py
    nav_config.json

AMR_InspectPaths.py 是選用的唯讀檢查檔；test_*.py 是離線測試，不必在 Script Editor 執行。
如果截圖路徑有不同拼法或你之後改資料夾，需修改兩個入口的 PROJECT 常數。
地圖存入 python_nav/maps，導航結果存入 python_nav/outputs，舊程式和舊地圖不會被刪除。

## 一、先開面板與遙控掃描

1. 開啟 sickroom_with_hand_colaTest.usd（實際檔名依你的專案）。
2. 按 Stop。在 Script Editor 開啟並 Run「AMR_TeleopMap.py」。
3. 出現「AMR | Teleop, Mapping, Goal B」面板。按 Play，等車子落地穩定。
4. 按「Start / resume mapping」，再按「Enable keyboard」。
5. 點串流的場景視窗，使用 W/S 前進倒退、A/D 左右轉；可以組合 W+A 等按鍵。
6. 放開鍵會停車；Space 或「STOP / disarm」停止並取消鍵盤控制。
7. 慢慢開過可通行區，從不同位置與朝向掃描床側、牆邊、門口、桌櫃附近；不要刻意碰撞器材。
8. 按「Save map」。需要暫停掃描時按「Pause mapping」，車子也會停下。

鍵盤事件由遠端 Isaac Sim 接收。WebRTC 或瀏覽器若攔截了鍵盤，
可用面板 Forward / Reverse / Left / Right，每次點擊給 0.25 秒牆鐘時間的短脈衝。
鍵盤也有 0.75 秒輸入更新期限；若平台不送 KEY_REPEAT，長按可能變成斷續移動，
可放開再按，或改用短脈衝按鈕。這是斷線時避免持續行駛的保護，不是實車通訊保證。

預設速度 0.25 m/s、轉速 0.55 rad/s，可在 nav_config.json 的 teleop_speed / teleop_turn 調整。
啟用 proximity_stop 時，光達看到預測車身範圍有障礙會停；資料過期也不會繼續開。
先前尚未解決的自車/重建模型回波問題仍可能影響遙控，不能保證這版會消除。

## 二、看掃描地圖

Save map 或每約 15 秒牆鐘時間自動儲存後，在 python_nav/maps：

- room_map_latest.png：給人檢視的地圖。
- room_map_latest.pgm：灰階地圖。
- room_map_latest.npz：含 log_odds、seen、occupied、origin、resolution、場景資訊，供導航。
- room_map_metadata.json：地圖原點、大小、掃描次數與欄位說明。

PNG/PGM 顏色：白=已觀測可通行，黑=障礙回波，灰=未知/不確定。
第 0 列格網對應最低世界 Y，輸出的圖片已上下翻轉，以符合一般上北下南檢視。
地圖以第一次開始掃描時的車子為中心，預設 24×24 m、每格 5 cm；不代表病房一定是 24×24 m。
若感測器走出地圖邊界，程式會拒絕合併，不會靜默丟失車子位置。

請下載 PNG 看你要走的區域是否已掃到。灰色不會被當成無障礙；無回波方向也不會直接標白。
無法保證手動行駛一次就掃滿所有可通行區；器材遮蔽、光達掃描高度及地圖尺度都會影響。
床架/低腳等低於掃描平面的物體可能掃不到，因此導航還會保留簡化場景障礙檢查。

新開面板後要接著掃，按「Load saved map」再「Start / resume mapping」。
已讀取地圖只允許接在同一個 stage 檔案 identifier 下；改檔名/另存新場景需先核對配準，不能直接混用。
新開始一份地圖第一次覆寫 latest 前，會把舊 latest 檔案備份到 maps/archive/<timestamp>/。
若要從零掃，關閉面板後重新 Run，不載入舊圖，直接 Start Mapping。

## 三、直接在 Isaac Sim 選 B，不必輸入座標

1. 先停車，按「Create / select B」。這會取消鍵盤控制，避免按移動快捷鍵時車子跟著走。
2. Stage 裡會選到 /World/PythonNav/GoalB。
3. 用左側 Move 工具的平面/箭頭，將 B 拖到想去的地板位置。
4. 場景會畫紅色十字輔助標示。這個目標是沒有 Mesh/Collider 的 Xform，光達不會把它當成器材。
5. 按「Print current A/B」可在輸出查看現在的世界座標。
6. 按「Preview A -> B」先看綠色路徑；確認後按「Navigate A -> B」。

A 永遠取車子當下落地後的實際位置，不會把你拖的起點當作車子已經站在那裡。
要換 A，請先遙控車子到新的位置。單純拖一個 A 標記不會讓物理車輛移動。
B 拖動只選位置；預設終點朝向是 A→B 的方位。若需要終點朝向，
在 nav_config.json 設 goal_yaw_from_marker=true，再用 Rotate 工具轉 B 的局部 +X 箭頭。
B 的高度不參與二維規劃，但建議只在地板平面移動。

導航期間面板不發出遙控指令，避免兩支程式搶控制。Stop / disarm 會取消導航。
面板關閉、Pause、Stop、輸入過期或腳本錯誤會清零速度。
Stop 重置物理狀態後，請再次 Play、等落地，再明確按 Resume Mapping。

## 四、導航地圖來源

nav_config.json 預設：

    "map_source": "SCANNED"

SCANNED：讀剛掃好的 NPZ，未知/不確定區域一律不可通行，另保留直接底下的簡化障礙模型。
因此若 A/B 附近沒掃完整，拒絕規劃是可能且合理的；應補掃，不是縮小車身。

    "map_source": "SCENE"

SCENE：沿用先前的簡化場景模型建立地圖，不需要先掃描。
這個模式是場景模型規劃，不可宣稱是光達建圖導航。
兩個模式都使用 Hybrid A* + 離散 Voronoi field，完整矩形車身、差速運動與原地旋轉掃掠檢查。
不是任意將掃描 PNG 當成坐標圖，也不把未知區域直接清空。

地板和簡化障礙預期在 floor 所在父節點直接底下；巢狀 sickroomOBJ 重建網格不當作整塊實心 AABB。
本版仍要求簡化地板/障礙存在，因單層 2D 光達未必能完整描述車身高度內的所有器材。
若你的新場景不再有這組模型，要先確認場景結構再改 adapter。

## 五、場景路徑與模型參數

nav_config.json 的 paths 是舊場景提示。若提示不存在，會依唯一的 base_link、Lidar_2D、
DifferentialController、floor 名稱尋找。找到多個或零個時會停下並列出候選，不猜測控制哪台車。
這時先 Run 唯讀的 AMR_InspectPaths.py，把正確 prim 路徑填回 JSON。

車長 0.88、車寬 0.56、驅動輪中心距 0.514、輪半徑 0.09 m，保留模型值。
光達端是原 base_link -X；預設線速指令反向、角速度同號，仍需低速確認實際輪子軸向。
body.height=.45 m 是目前的碰撞高度帶，若模型上方新增突出結構要更新。
動態載重、腳輪、摩擦、轉矩、加速度上限尚未由物理模擬驗證。

## 六、限制與測試

- 使用 USD 真值位姿，不是 SLAM，不代表離開模擬器後能直接建圖。
- 手動遙控不是自動探索，不保證全病房已掃完；先覆蓋 A→B 所需區域。
- 光達只保留實際有效回波；無回波不等於已知自由空間。
- 使用每幀 instantaneous 輸出，但尚未驗證完整的渲染延遲/逐射線運動補償。
- 本版有近距離停車保護，沒有新動態障礙的自動重規劃繞行。
- 地板路徑：綠=規劃、藍=實際軌跡、紅=B，為地板上方約 2.5 cm 的 Debug Draw。
- Debug Draw 是共用頻道，編輯目標或新規劃會清除先前 debug 線/點；不新增真實碰撞物。
- 22 項離線單元測試通過（建圖7、控制數學7、規劃8）；所有腳本語法通過。
- WebRTC 鍵盤、omni.ui 面板、實際雷射資料與物理車輛仍需在你主機驗證。

本機測試：

    python -m unittest discover -s python_nav -p "test_*.py" -v

Isaac 模組只在遠端入口內匯入；離線測試不需要 Isaac Sim。
規劃核心需要 NumPy、SciPy，Isaac 主機是否已具備仍以實際錯誤為準。

官方介面：
https://docs.omniverse.nvidia.com/kit/docs/carbonite/210.1.12/docs/python/bindings.html
https://docs.omniverse.nvidia.com/kit/docs/omni.ui/2.26.18/omni.ui/omni.ui.Button.html
https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_rtx_lidar.html
