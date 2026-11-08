# 差速 AMR：Hybrid A* + Voronoi Field（離線版）

本版用於先驗證靜態地圖的 A→B 路徑與終點朝向。不需要 Isaac Sim、ROS 或馬達連線。
預設範例是 10×7 m 的「合成病房」，不是實際病房掃描，也不是已驗證的模擬結果。

## 使用方式

安裝依賴（電腦已有套件可跳過）：

    python -m pip install -r requirements.txt

在本資料夾開啟終端：

    python planner.py
    python -m unittest test_planner -v

預設結果放在 outputs/demo：plan.png、planned_path.csv、planned_path.npz、summary.json。
圖上：綠線是路徑，藍框是模型車身，藍點 A、紅點 B，箭頭是前方。
左圖青色線是離散近似 GVD；深色低成本、亮色高成本。
搜尋路徑不必沿著 Voronoi 中心線走；場是軟成本，碰撞才是硬限制。

指定範例地圖上的起終點（角度是「物理車頭」相對世界 +X，單位度）：

    python planner.py --start 1.2 1.4 0 --goal 8.5 3.7 90 --out outputs/custom

## 用自己的病房地圖

    python planner.py --map room.npz --start 0.6 0.9 180 --goal -0.5 -0.5 90 --config robot_config.json --out outputs/room

room.npz 必須有：
- occupied：二維 bool 陣列，True=障礙/未知，False=已知可通行；尚未膨脹車身。
- origin：[世界最小 X, 世界最小 Y]，以公尺表示，為左下格子的左下角。
- resolution：每格幾公尺，例如 0.05。第 0 列是最低 Y，向上增加。
- 車子、器材及地圖全部使用相同尺度、原點和單位。

提供 demo_ward_map.npz 作為格式範例。不要把舊 AGV_NavigationPath.npz 裡已膨脹的 free
遮罩直接反轉當作原始地圖，否則會重複加入車身空間。舊 AGV_GridMap.npz 的鍵值與編碼
需確認後轉換；本版不猜測 0/1/255/unknown 的意義。

## 模型值與場景參數分開

robot_config.json 可以修改；未指定 --config 時使用程式內同一組預設值。

| 類別 | 參數 | 預設/用途 |
|---|---|---|
| 模型幾何 | length / width | 0.88 / 0.56 m，包含原模型前後外形 |
| 驅動輪 | track | 0.514 m，左右驅動輪中心距，不是 0.56 m 車寬 |
| 驅動輪 | wheel_radius | 0.09 m |
| 場景安全間距 | margin | 四側各 0.04 m；不包含動態煞停距離 |
| 搜尋 | xy_bin / yaw_bins | 0.10 m / 72 格（5°） |
| 搜尋步長 | primitive_seconds | 0.65 s；名義 0.35 m/s 時每步約 0.228 m |
| 障礙成本 | field_range / field_weight | 1.0 m / 3.0，依房間寬度調整 |
| 名義動作 | nominal_v / nominal_w | 0.35 m/s、0.8 rad/s，只用於幾何動作展開 |

來源詳見 parameter_sources.json。輪距是模型估值，不宣稱已量測實車。
沒有套用原停車程式的 4.41 m 車長、2.65 m 軸距、方向盤轉角或汽車轉彎半徑。
若之後新增模型外伸器材，要更新車身 footprint，而不是繼續用這個矩形。

通道寬度必須容得下車身加 margin。調低場權重只會改變偏好，不能解除碰撞。
若地圖比例不確定，先量同一場景下車寬/門寬/床邊間距；不能讓規劃器縮小車身來假裝可通過。

## 實際演算法

1. 對占據格網計算障礙歐氏距離 dO。
2. 比較相鄰格子的最近障礙來源；來源充分分離且位於不同方向時標為 GVD 邊。
   再算到邊的距離 dV。這是格網 GVD 近似，不是精確連續 Voronoi，也不是只用障礙反距離。
3. Dolgov 形式的場：
   rho = alpha/(alpha+dO) * dV/(dO+dV) * ((dO-dmax)/dmax)^2，dO<dmax；否則 0。
4. 連續 (x,y,yaw) 狀態，以離散位置、朝向、前一動作作搜尋索引；優先佇列展開。
5. 差速動作：前進/倒退配五種角速度，加上左右原地轉。以定曲率精確積分。
6. 成本：距離、旋轉、倒車、換向、控制變化及沿途 Voronoi 場積分的近似。
7. 使用加權 A*（預設 1.2），不保證全域最優，也不保證有限格網搜尋完備。
8. 每個動作沿途採樣，用完整旋轉矩形對占據格子做 SAT 碰撞檢查。
   採樣間隔考慮角點旋轉速度，並受地圖解析度限制；並非連續碰撞數學證明。
9. 靠近 B 時嘗試「旋轉→直行→旋轉」的局部連接；每段也經過沿途碰撞檢查。
   不使用汽車 Reeds–Shepp 假設。超時/次數用完不等於沒有可行路。

本版沒有增加平滑後處理，以免未驗證的平滑把原可行路徑推進障礙。

## 輪速與 Isaac Sim 接軌

邏輯車頭 +X、左 +Y；v>0 前進，omega>0 逆時針：

    left  = (v - omega*track/2)/wheel_radius
    right = (v + omega*track/2)/wheel_radius

這裡 left/right 是面向光達車頭的左右，單位 rad/s。
你的 vendor base_link 車頭在 -X，因此：

    yaw_plan = wrap(yaw_base + pi)
    v_base = -v_plan
    omega_base = omega_plan

planned_path.npz 也輸出 base_link_v、base_link_omega（原控制器正線速沿原 +X 的假設）。
原模型名字中的 left_drive_wheel_link/right_drive_wheel_link 可能與邏輯左右相反；
左右輪關節軸向/正負號仍要在 Isaac Sim 低速驗證，不能直接把 CSV 丟給關節。

軌跡 CSV 第 i 列 v/omega 對應前一列到第 i 列的積分區間；最後增加零速停止事件。
這是名義幾何控制，不含加減速/馬達轉矩/摩擦/載重/腳輪動態；不得直接當作馬達時間排程。
真正導航還需定位、回授追蹤、速度規劃、即時障礙更新與保護停車。
先前 Isaac Sim 光達誤停的問題不會因更換全域規劃器自動解決。

## 參考

Dolgov et al., Practical Search Techniques in Path Planning for Autonomous Driving, 2008.
https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf
本實作借用場的公式與 Hybrid 搜尋概念，運動模型改為差速；不是論文汽車程式的原樣重現。
