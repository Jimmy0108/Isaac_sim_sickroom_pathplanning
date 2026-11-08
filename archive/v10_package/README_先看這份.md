# V10.5：直接操作 → 建圖 → A 到 B

此包已完成本機程式與數學／mock 檢查，尚未在你的遠端 Isaac Sim 實測。沒有 ROS2，也不加入 IMU。四方向按鈕不使用任何直行 PD／積分／航向補償；導航時 RPP 的路徑追蹤仍保留。

## 上傳與啟動（只執行一個入口）

1. 遠端先按 **Stop**。先備份現有的 `python_nav` 資料夾，尤其是 `slam_maps`。
2. 把本包內的檔案放到同一個遠端資料夾，不要多套一層目錄：

   `/isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/python_nav/`

3. 同名核心檔案使用此包版本。不要刪除 `slam_maps`。這版使用獨立的 `v105_config.json`，不需要改你原本的 `slam_config.json`。
4. 在 Isaac Sim Script Editor 開啟 **AMR_SimpleNavigation_V10_5.py**，按 **Run**，再按 **Play**。
5. 等待落地、輪子穩定，面板顯示 `Manual ready`。初始不載入舊圖、不自動移動找定位。

若遠端路徑不同，只修改入口檔 `PROJECT`。輪車 Prim 路徑則修改 `v105_config.json` 的 `paths`。預設控制器已改為上次實跑實際存在的 `/World/AGV_control/AGV_DifferentialGraph/DifferentialController`。程式若發現有效輪徑／輪心距與軟體不一致，會明確拒絕移動，不自行猜。

**不要同時啟動舊 V10.4 入口或其他輪速程式。** 入口會先停止已知舊工作並等待清理，避免同時寫輪子。V10.4 舊啟動程式具有自動改檔行為，日後重跑可能會把共用核心覆回舊版；若回退請使用備份。

## A. 先按四個方向

| 操作 | 邏輯線速度目標 | 邏輯角速度目標 |
|---|---:|---:|
| Forward／W | +0.9 m/s | 0 |
| Reverse／S | −0.9 m/s | 0 |
| Left spin／A | 0 | +0.9 rad/s |
| Right spin／D | 0 | −0.9 rad/s |
| STOP／SPACE | 0 | 0 |

按鈕是持續操作；按 STOP 或 SPACE 結束。鍵盤放開就停止，失去按鍵更新也停止。轉彎切到直行、換方向時會先歸零，等輪式 odom 連續 0.3 秒顯示停穩才重新起步。

起步仍保留線加速度 0.3 m/s²、減速限制 0.5 m/s²、角加速度 1.2 rad/s²；緊急停止命令直接歸零。0.9 是目標上限，不代表立刻達速或所有彎道都用最高速。若 OmniGraph 有更低的輪速上限，會等比例限制 v/w 並顯示原因。前進時不會因 SLAM 角度修正偷偷加入 w。

手動操作只要求有效輪子介面與新鮮局部掃描，不要求 SLAM 健康。2D 掃描預測完整矩形車身（原 4 cm 裕度）被阻擋時會停車並顯示原因，不再嘗試一連串隱藏低速。沒有加回先前的額外 3D 保護圈。2D 掃描看不到所有高度的障礙，因此不能視為完整 3D 避障驗證。

如果這時 `w=0` 仍明顯偏轉、跳動或翻覆，先 STOP，再匯出同次 ZIP，不用再執行其他診斷腳本。車身傾斜超過 20° 會鎖定停止並輸出紀錄；需重新 Run，避免倒置後繼續建圖。

## B. 四方向正常，就開始建圖

1. 停車，按 **Start NEW map**。等狀態為 `MAPPING`。
2. 按方向按鈕或 WASD，繞不同位置掃描牆面、床邊、轉角；也回訪起點。
3. SLAM 匹配拒絕時不把該掃描整合進圖；面板會明確顯示匹配原因。只要局部掃描有效，手動操作仍可用來調整觀測位置。
4. 停穩後按 **Finish mapping / use candidate**。必須有新鮮接受的配準，否則不凍結壞圖。
5. 候選圖保存在這次 `slam_outputs/mapping_session_時間碼/candidate_map/`，隨即凍結地圖，可在同一次執行接著導航。
6. 只有你按 **Confirm candidate / archive old / save** 才備份並更新 `slam_maps`。不按也能使用當次候選圖導航。

未完成就結束的圖放在此次 `incomplete_map`，標示未完成，不會自動取代正式圖。

掃描來源是現有 **PHYSX_COLLISION_RAYCAST、720 束、10 Hz**；使用 Lidar_2D 的安裝外參，但不是消費 RTX 光達輸出的點雲。輪式 odom 每個物理步讀實際關節角度，掃描等待 odom 時間範圍涵蓋後才插值配對。世界位姿只用於感測器模擬、一次性視覺錨點與診斷／翻覆停止，不提供 SLAM 的位置或航向。

## C. 設定 B，預覽並導航

1. 按 **Create / select B**。Isaac Sim 選中 `/World/PythonNav/GoalB`。
2. 使用 Move 拖 XY；使用 Rotate 繞 Z 調整朝向。地板上的紅箭頭代表到達後的車頭方向，箭頭 +X 不需要再因 base_link −X 而反轉。
3. 面板即時顯示 B 的世界 XY、地圖 XY 與兩個座標系朝向。忽略高度、roll/pitch。
4. 按 **Preview Hybrid A* + Voronoi**。綠色線是全域路徑。
5. 按 **Navigate A -> B with RPP**。A 自動取當前 SLAM 估計；黃色為局部預測，藍色為估計軌跡。
6. 最後位置須在 7 cm 內、方向須在 5° 內，且輪式速度停穩才回報抵達。這是 SLAM 估計驗證，不是世界真值驗證。

拖曳後的 B 只有按預覽／導航才被正式擷取；導航途中移動 marker 不會默默更改重規劃的目標。若需新 B，先 STOP 再明確按導航。

若使用既有圖：按 **Load saved map**，先等待原地掃描定位；沒有自動轉動或前後移動復原。失敗時不會自行開車，你仍可明確按手動方向調整觀測位置，或改用新建圖流程；開始導航仍須定位成功。導航遇掃描過期、定位失效或阻擋時會停止；阻擋最多進行 3 次停車後全域重規劃。

## D. 結束與回傳資料

按 **Stop / export this session ZIP**，等待畫面顯示 `SESSION EXPORTED`。程式會停止模擬、解除 callback、等待工作執行緒、還原此輪 session 設定並輸出：

`python_nav/slam_outputs/mapping_session_時間碼.zip`

關閉面板或按 Timeline Stop 也會嘗試保存已收集資料。不會在結束時暗中確認／覆寫正式圖。

ZIP 內含直接關節輪速、OmniGraph 實際輪速目標、odom、診斷車身軌跡、掃描、逐筆 SLAM 輸入、B 點、規劃結果、停止原因、有效物理設定、核心原始碼與還原結果。`graph_left/right` 是控制器原始輸出順序；邏輯車頭左右輪的對應另有記錄，不混用兩者。

## 物理設定與限制

- 沿用本次實跑軟體尺寸：長 0.88 m、寬 0.56 m、輪徑半徑 0.09 m、輪心距 0.514 m；光達端為 base_link −X。保留既有 Encoder 左右／符號對應。
- 預設維持 V10.4 實跑時的有效基準：兩驅動輪 10 N·m 上限及現有 grip 材質；不是重新調校，也不是實車標定。實際摩擦與驅動值寫入 `effective_physics.json`。
- 不修改病床牆壁，不換材質方案，不自動搜尋力矩。`preserve_v104_effective_drive=false` 才改成完全使用 USD 原始驅動／材質；這不是建議你在第一輪同時更改的項目。
- 保留經前次實跑使用的 60 Hz 時鐘與重複 PhysicsScene 檢查。只在場景設定完全相同且無外部引用時才暫時停用重複場景，結束還原，不刪除或保存 USD。
- 仍需在 Isaac Sim 確認物理步 callback、關節接觸、實際直行及完整導航。取消補償不等於物理滑移一定消失。

## 開發者本機檢查

`python -m unittest test_v105_control test_v105_goal test_v105_io test_v105_app -q`

目前檢查為離線數學／mock，包含控制純度、轉向切換、障礙停止、B 座標／朝向、規劃與 RPP 理想運動追蹤、候選保存及停止取消導航。不要將其稱為 Isaac Sim 實際車輛驗收。
