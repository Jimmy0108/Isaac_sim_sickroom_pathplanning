# V66 Smooth 1.1：規劃／追蹤修改，保留車體與地圖

## 怎麼安裝

1. Isaac Sim 按 Stop，等導航匯出／清理結束。
2. 備份遠端現有的六個同名程式／設定檔。`slam_maps`、USD 車體與 swivel 設定保留。
3. 將本補丁 ZIP 中六個檔案解壓並覆蓋到目前的 `new_car/python_nav/V66_Python_Navigation/`，不是再建立一層同名子資料夾：
   - AMR_V66_Navigation.py
   - amr_planner_core.py
   - path_smoothing.py（新增，不能漏傳）
   - rpp_core.py
   - v66_app.py
   - v66_nav_config.json
4. Script Editor 關閉舊入口分頁，重新開啟入口，Run → Play。
5. 可載入現有地圖，等待定位完成，拖 B、預覽，確認面板版本 `v66-nav-1.1-smooth` 後導航。先選空間充足的短程 B 點。無需為本補丁重新掃圖。

## 從這次實際資料找到什麼

原路徑控制類型為 reverse → spin → forward → spin。原終點 connector 僅有「旋轉、向前直走、最終旋轉」，會使原本倒退到接近終點的車再換成向前。這是規劃器刻意生成的動作，不等於車輪方向錯誤。

原 Hybrid A* 路徑沒有後處理平滑；控制器的前視距離隨速度在0.55～0.9m變化，且追蹤轉向濾波只有0.04s。新設定是有依據的候選改善，不宣稱已找到所有蛇行的唯一原因。

注意手動與導航不同：手動前進指令仍是 w=0，不加入直行補償。若手動旋轉後按前進仍先有車身偏轉，要對照 command_w 與實際 body yaw rate／萬向輪迴正，不能說都是 RPP 的問題。

## 改了程式的地方

- 終點 connector 同時考慮正、反向，以及切線與端點朝向一致的 Hermite 曲線。候選都檢查完整 footprint；曲線不合適時保留旋轉＋直行方案。選擇成本包含旋轉段落的額外代價。
- 終點嘗試連接範圍由0.55m改為1.0m，讓曲線可較早銜接；不擴張可通行區域。
- 對同一前後進方向的 move group 做 cubic spline 後處理，保留起終點／車頭方向。不得跨越倒退 cusp 或 spin 硬做圓弧。
- 平滑候選使用約1cm位置取樣、最多約1.5cm相鄰距離及有限相鄰 yaw 變化，逐點檢查完整矩形 footprint。超曲率、碰撞或不合理候選回退原段；這不是連續碰撞或真實動力學保證。
- 導航旋轉段交接到直行段使用現有 StopMonitor，等待車身／輪組停穩；路徑對齊完成後也先清空轉向輸出、停穩再前進。
- 曲率造成的橫向加速度加入限速，保留曲率、障礙、終點調速及局部預測碰撞。
- 加入 rpp_tracking 記錄，匯出 carrot、cross-track、heading error、raw_w、追蹤狀態等，讓下次能判讀控制器為何轉向。

## 參數前後

| 參數 | 舊 | 新 |
|---|---:|---:|
| 前視距離 | 速度相關0.55～0.9m | 固定0.70m |
| RPP max_v | 0.9m/s | 0.9m/s，保留曲率／障礙限速 |
| RPP max_w | 0.9rad/s | 0.6rad/s |
| 計畫內原地旋轉／對齊上限 | 使用max_w | 0.45rad/s |
| 轉向濾波 steering_filter_s | 0.04s | 0.12s |
| max_angular_accel | 1.2rad/s² | 0.8rad/s² |
| tracking_yaw_damping | 0.08 | 0（保留PP位置追蹤，移除輪式yaw rate額外阻尼） |
| rotate_gain | 1.5 | 1.1 |
| align_exit_angle | 20° | 6°，且等停穩 |
| 曲率橫向加速度限制 | 無獨立參數 | 0.35m/s²候選值，非實車量測 |

手動上限仍0.9m/s及0.9rad/s，沒有新增手動航向保持。車體、輪組、摩擦、Drive、時鐘、SLAM演算法、地圖尺寸／解析度／margin均不變。

## 已驗證與未驗證

- 使用使用者此包 candidate_map、起點與B點做離線規劃及平滑：端點保持、方向保持、平滑採樣 footprint 無碰撞；見 smooth_validation.json。
- 使用相同地圖回放，包含倒退與spin邊界測試；不把離線幾何模擬當成Isaac車身控制驗收。
- 必要的 spin 可能仍存在，綠線不保證每個角都會變成圓弧。現場新障礙仍由原有檢查及重規劃處理。
- 尚未在你的 Isaac Sim 實跑。此補丁也不處理上一包世界座標B點約22.9cm與SLAM估計4.5cm之間的對齊差異。

參考 [Nav2 RPP 參數與前視距離說明](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)：本專案仍為自製Python追蹤器，不是直接使用Nav2 plugin。此版本採固定前視距離以減少速度與曲率參考互相影響；不是照抄官方預設增益。
