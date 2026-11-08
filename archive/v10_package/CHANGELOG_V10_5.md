# V10.4 → V10.5

基準：使用者提供的 mapping_session_1793276113072335610.zip/runtime_sources，不以更早的開發副本取代實跑來源。

## 程式變更

- 新入口 AMR_SimpleNavigation_V10_5.py：完整來源載入，避開舊 pyc；停止並等待舊命令擁有者，不再透過多代文字 patch 疊加安裝。
- 新應用 v105_app.py：MANUAL／MAPPING／READY／LOCALIZE／PLAN／NAV／VERIFY，只有導航要求可信定位。啟動不自動載圖或主動移動復原。
- 新 v105_control.py：四方向直接輸出、加速斜坡、換向停穩與掃掠矩形碰撞停止；不接收 SLAM pose，不含航向 PD／積分或偏壓。
- 移除舊 app 對所有手動指令套用的 was_healthy 門檻、直行補償與 1／0.75／0.5／0.25／0.1 倍速度搜尋。舊 isaac_slam_app.py 不由新入口載入。
- Encoder／odom 改由 PHYSICS_POST_STEP 讀取更新；PHYSICS_PRE_STEP 寫出下一步唯一命令。UI、掃描接收與重運算分開；scan/plan 只在工作執行緒處理擁有的 numpy 副本。掃描時間須被 odom history 包住，不將未來時間直接夾到最新姿態。
- 新 v105_io.py：記錄直接 PhysX 關節角速度，而非只由 odom 反算；記錄控制器輸出、輪綁定及新鮮車身診斷資料。世界姿態不作定位／轉向回授。
- isaac_io.py 的 Display 新增完整目標姿態、world/map 座標說明、地板箭頭；沒有更換掃描後端或外參邏輯。
- 正式規劃呼叫 planner.plan(grid,A,B)，B 是完整 [x,y,yaw]，不再搜尋自由終點朝向。不刪除 RPP 最後旋轉段；停穩後驗證7cm與5°。
- STOP／Pause 會取消排隊中的導航；不靠取消 to_thread 來假裝終止執行緒，等待完成且丟棄過期結果。重規劃沿用明確提交的 B。
- 完成建圖先寫 candidate_map；不在 finally 自動覆蓋 slam_maps。明確確認候選圖後才備份並替換。載入另一張圖會清除舊候選圖身分。
- 匯出前停止、解除 callback、還原此輪 session layer 與時鐘，將還原錯誤如實放入 ZIP。USD 不保存。

## 參數變更與保留

| 項目 | V10.5 |
|---|---|
| 手動航向／橫向補償 | 移除，不使用舊增益 |
| 手動線速度／角速度目標 | ±0.9 m/s／±0.9 rad/s |
| 線加速／減速限制 | 0.3／0.5 m/s²，沿用 |
| 角加速度限制 | 1.2 rad/s²，沿用 |
| 換向停穩 | abs(odom.v)<0.02、abs(odom.w)<0.04，連續0.3s |
| 原矩形 footprint margin | 0.04m，沿用；無額外3D保護圈 |
| 翻覆停止 | 診斷車身傾斜超過20°，鎖定停止並輸出 |
| 導航到達 | 位置≤0.07m、朝向≤5°、輪式速度停穩 |
| 載圖／主動復原 | 不自動載圖、不自動移動；載圖僅手動點擊 |
| 光達 | PHYSX_COLLISION_RAYCAST／720束／10Hz，不更換 |
| 直行補償開關 | mapping_heading_hold=false；舊補償程式不載入 |

RPP、SLAM、Occupancy、Hybrid A*／Voronoi 核心演算法沿用本次實跑來源（RPP 為 V10.4 追蹤版本），此版重點是控制流程與有朝向 B 的接線，不是重新發明或調快配準演算法。沿用的完整參數在 v105_config.json；其中 min/max lookahead=0.55/0.90m、lookahead_time=1.3s、RPP max_v/max_w=0.9/0.9。全域搜尋 nominal_v=0.40m/s、nominal_w=0.90rad/s，與實際跟隨最高速不同。

## 未驗證項目

尚未在遠端 Isaac Sim 執行：物理 callback 的實際交付順序、輪胎／萬向輪接觸、直行品質、真實煞車距離、完整房間建圖及實際抵達。離線通過不代表這些項目已通過，也不代表造成原圖 wall_e 不一致的根因已解決。

## 核對的官方介面

- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.core.simulation_manager/docs/index.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.core.rendering_manager/docs/index.html
- https://docs.omniverse.nvidia.com/kit/docs/omni.timeline/latest/omni.timeline/omni.timeline.Timeline.html

遠端只須 Run 入口，不需要執行本機 unittest。每次遇到問題回傳同次 ZIP 即可，不要求另外跑一整套診斷。
