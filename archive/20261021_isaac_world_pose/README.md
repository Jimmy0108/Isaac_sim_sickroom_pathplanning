# Isaac Sim 純 Python：Hybrid A* + Voronoi Field 規劃導航

這是 Isaac Sim Script Editor 入口＋純 Python 規劃核心，不需要 ROS 2。
目前依照本聊天已知的場景路徑與 DifferentialController 介面撰寫。
尚未在你的 Isaac Sim 主機實測；離線測試不能取代物理引擎/感測器驗證。

## 只需上傳兩個檔案

將以下兩個檔案上傳到遠端同一個目錄：

    /isaac-sim/.local/share/ov/data/AMR_Project/AMR_Hybrid_Navigate_Isaac.py
    /isaac-sim/.local/share/ov/data/AMR_Project/amr_planner_core.py

不要只把 zip 放在該目錄；需要解壓後的 .py 檔案。
若專案位置不同，修改入口檔 PROJECT。
核心依賴 NumPy、SciPy。入口會使用 Isaac Sim 內的 Python，不是 Windows 的 Python；
如果輸出 ModuleNotFoundError: scipy，表示 Isaac Sim 的環境缺依賴，先回傳錯誤再處理。
規劃與導航不需要 Matplotlib；核心內的繪圖函式不會由入口呼叫。

## 操作

1. 開啟既有 USD，確認原差速控制器能控制車輛。
2. 確認 Collision/floor 及直接底下的 bed、cabinet、wall_* 等簡化模型位置正確。
   腳本會使用這些模型，排除巢狀 sickroomOBJ 重建網格，保留完整器材責任仍在簡化模型。
3. 按 Stop。在 Script Editor 開啟 AMR_Hybrid_Navigate_Isaac.py。
4. 設定 GOAL_XY。數值是世界公尺座標，沿用之前的格式。
5. 按 Run，再按 Play。
6. 等車子落地穩定，程式建立格網並在工作執行緒規劃。期間指令保持零，UI 可更新。
7. 規劃完成後顯示地板路徑，PREVIEW_ONLY=False 時開始回授追蹤。

若只想先看路徑，把 PREVIEW_ONLY=True。這種模式不需要光達資料、不發出導航速度，
但仍需 Play 讓車子落地、讀取正確的起點與地板高度。

## 路徑顯示

- 綠線：Hybrid A* + Voronoi 場規劃路徑。
- 藍點：起點 A；藍線：實際行駛軌跡。
- 紅點與紅箭頭：終點 B 與要求的物理車頭朝向。
- 黃點：若光達觸發停車，標示觸發回波。
- 線的位置為地板頂面上方 2.5 cm，避免被地板遮住。

使用 Debug Draw 視覺覆蓋層，不建立帶有碰撞或供光達偵測的實體路徑模型。
入口會自動啟用 isaacsim.util.debug_draw；如果繪圖不可用則不會默默開車。
Debug Draw 是共用頻道：開始畫新路徑時會清除既有 debug 線與點。
這是視窗中的路徑顯示，並非 USD 裡可永久保存的幾何；重新開啟場景需重跑程式。

## 入口上方可調參數

    GOAL_XY = (-0.516, -0.515)
    GOAL_YAW_DEG = None
    PREVIEW_ONLY = False
    LIVE_LIDAR_STOP = True
    MAX_SPEED = 0.40
    MAX_TURN = 0.70
    SAFETY_MARGIN = 0.04

GOAL_YAW_DEG：None 表示朝向 A→B 方位；也可填 0/90/180 等角度。
0° 指世界 +X，90° 指世界 +Y，均以光達端這個物理車頭定義。
本版要求 B 的完整車身與終點朝向可放入空間，不會只檢查一個點。

轉彎、倒車、原地旋轉是分段追蹤，不會把所有 XY 點直接當成連續前進。
每個動作段的邊界會減速/停車確認，再進下一段，第一版刻意保守，尚未做高速銜接。
提高 MAX_SPEED 也不會自動提高規劃核心的名義動作上限 0.40 m/s。
實際速度還會受模擬 FPS、物理驅動器、摩擦與腳輪影響。

BODY_HEIGHT=0.45 m 用於篩選可能與機器人相撞的模型高度，須隨外伸結構更新。
二維模型不支援斜地板；只接受 Z-up、水平地板（允許繞 Z 旋轉）。

## 模型與方向

- 矩形車身 0.88×0.56 m，安全間距額外加上。
- 驅動輪中心距 0.514 m，輪半徑 0.09 m；這是既有建模值，沒有以車寬代替輪距。
- 光達端朝原 base_link -X，因此導航朝向 = base yaw + pi。
- logical v 正向為光達車頭；預設給控制器的 linearVelocity 為 -v。
- angularVelocity 預設與導航 omega 同號。

第一次物理測試若車子直行方向相反，停止並核對 LINEAR_COMMAND_SIGN。
本版在可量測移動明顯與指令相反時也會中止，不能代替關節軸向/控制器實際驗證。
它直接輸入 v、omega 給既有 DifferentialController，輪速由控制器處理；
請在該節點核對 wheelRadius=0.09、wheelDistance=0.514（實際欄位名依節點版本）。
不會自動覆寫你的控制器輪徑/輪距、關節、馬達參數或模型幾何。

## 感測與避障的範圍

固定障礙：根據簡化模型的有方向包圍盒投影成格網，規劃時避開。
投影保留障礙旋轉方向，避免單純世界 AABB 過度擴大。
實際車身與預測軌跡使用旋轉矩形碰撞檢查，包含原地旋轉掃掠。

LIVE_LIDAR_STOP=True：使用既有 Lidar_2D，要求 spherical SENSOR 輸出，
配置每幀 instantaneous 掃描；不使用「回波方向必須 70/72」這種錯誤門檻。
不需重建光達、不建立第二個感測器；重用已有 reader。
檢查回波是否進入預測矩形車身＋間距，資料缺失/過期也會停車。
車身 footprint 內且在已知高度內的回波視為自車回波，外側回波不會任意排除。
沒有新障礙自動繞行或光達 SLAM；偵測到阻擋會停下保存診斷。
感測器在掃描期間移動、渲染延遲等尚未有完整逐射線運動補償。

LIVE_LIDAR_STOP=False：只有靜態模型規劃與預測檢查，不具即時光達避障。
這個模式必須由使用者明確選擇；不能用它宣稱解決先前的光達誤停。
若你仍有「自車/重建模型回波」問題，這版也可能停車，請分析新診斷，不要任意縮小車身。

## 輸出與診斷

遠端 PROJECT/HybridNavigationOutput：

- scene_map.npz：原始 occupied 格網（可給離線規劃器）、origin、resolution。
- hybrid_path.npz：路徑位置/朝向、Voronoi 場、名義動作。
- stop_diagnostic.npz：如遇預測阻擋，儲存姿態、原因、光達資料與路徑。

這個輸出目錄不覆蓋先前的 AGV_GridMap、AGV_NavigationPath 或其他控制腳本。
Run 時會取消已知舊控制 task，避免舊程式與新程式同時控制車子。
按 Pause/Stop 或程式出錯時，兩個速度輸入歸零。

## 離線檢查與實測限制

- 規劃核心沿用已驗證離線版（8 項單元測試）。
- 本版另有 7 項純數學測試：凸包/旋轉投影、格子交集、倒車、原地旋轉、分段與 -X 車頭。
- 合成地圖上用理想差速積分做回授追蹤，9 個段、605 個 0.05 s 步進，
  終點位置誤差 0.05897 m，終點朝向在 5°內，沿途採樣模型碰撞檢查通過。
- 這些不是 Isaac Sim 動態測試。不能保證馬達、輪距、腳輪、摩擦、真實病房比例或光達問題已驗證。
- 規劃器是加權 Hybrid A*，Voronoi 邊為格網近似；不宣稱最優或完備。

官方介面參考：
https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_rtx_lidar.html
https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/internal_extensions/isaacsim.util.debug_draw/docs/api.html
