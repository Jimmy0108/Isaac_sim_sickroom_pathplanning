# 目前參數與座標

來源為 navigation/v66/v66_nav_config.json 及 assets/v66/vehicle_geometry_config.json。實際啟動仍須讀取匯出的 effective_config 與 effective_physics，檔案中的預設值和場景生效值需分別核對。

## 車體與輪組

| 項目 | 值與定義 |
|---|---|
| 舊驅動輪半徑／輪距 | 0.09 m／0.514 m |
| V6.6 驅動輪半徑 | 0.08 m，直徑 160 mm |
| V6.6 驅動輪心距 | 0.4769668332689978 m |
| 萬向輪直徑 | 63 mm |
| STEP 側殼 | 810 × 560 × 250 mm |
| 導航整車包絡 | 0.898 × 0.5698 × 0.444 m |
| 安全 margin | 0.04 m |
| 車頭 | base_link −X，光達端 |
| v／w 命令符號 | −1／+1 |
| 邏輯左輪／右輪舊名稱 | right_wheel_joint／left_wheel_joint |
| 編碼器邏輯左右符號 | −1／−1 |
| LiDAR 名義安裝 | (−0.23, 0, 0.345) m |

兩種外廓量測範圍不同。導航採含突出物的整車包絡，側殼尺寸用於 CAD 比對。

輪速換算採 ωL＝(v−ωb/2)/r、ωR＝(v＋ωb/2)/r，單位 rad/s。實際還要套用車頭、關節方向及名稱綁定。輪式 odom 由實際輪角增量積分，無法單憑輪角獨立識別輪胎滑移。

## 感測與地圖

| 項目 | 設定 |
|---|---|
| 掃描後端 | PHYSX_COLLISION_RAYCAST |
| 束數與生成頻率 | 720 束、10 Hz |
| SLAM 提交最短間隔 scan_period | 0.2 s，最多約 5 Hz 配準提交 |
| 掃描最大距離／近距離 | 12 m／0.1 m |
| 掃描／定位逾時 | 0.8 s／1.2 s |
| 地圖 | 0.05 m／格、24 m 畫布 |
| Keyframe 位移 | 0.08 m |
| 自動載圖／主動定位恢復 | 關閉／關閉 |
| 掃描取樣時機 | Physics POST step |
| physics_hz | null，讀取場景原設定 |

10 Hz 掃描、5 Hz 配準提交、物理更新與畫面 FPS 分屬不同排程。實際接受及丟棄數須查匯出紀錄。

## Hybrid A* 與 Voronoi

| 項目 | 值 |
|---|---|
| XY 分箱／yaw 分箱 | 0.1 m／72 格 |
| 運動基元秒數 | 0.65 s |
| nominal v／w | 0.4 m/s／0.6 rad/s |
| 碰撞取樣步長 | 0.025 m |
| Voronoi weight／alpha／range | 3.0／0.4／1.0 m |
| rotation cost／reverse multiplier | 0.25／1.3 |
| switch／control change／spin start | 1.0／0.12／0.8 |
| heuristic weight | 1.2 |
| max expansions／搜尋預算 | 60000／45 現實秒 |
| 連續階段預算比 | 0.6 |
| connector distance／最大曲率 | 2.0 m／2.0 m⁻¹ |
| optimizer 預算 | 5 現實秒 |

## RPP 與命令

| 項目 | 值 |
|---|---|
| 手動 v／w | 0.9 m/s／0.9 rad/s |
| 手動加／減速度 | 0.3／0.5 m/s² |
| 手動角加速度 | 1.2 rad/s² |
| 導航 max v／w | 0.9 m/s／0.6 rad/s |
| 導航 acceleration／deceleration | 0.3／0.4 m/s² |
| 導航角加速度 | 0.8 rad/s² |
| 原地旋轉上限 | 0.45 rad/s |
| 前視範圍／時間 | 0.35–0.65 m／1 s |
| steering filter | 0.08 s |
| approach distance | 1.5 m |
| collision time | 0.6 s |
| reaction time | 0.15 s |
| 候選側向加速度 | 0.30 m/s² |
| 軟體輪速限制 | 12 rad/s |
| 位置／方向門檻 | 0.07 m／5° |

反應時間、減速度、側向加速度與輪速限制是目前工程設定。其適用性需由 PhysX 實跑與將來實車驗證。
