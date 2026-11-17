# Isaac Sim 病房 AMR 建圖與路徑規劃

📎 主要參考來源：[目前導航程式](navigation/v66/)、[架構與演算法](docs/ARCHITECTURE.md)、[參數表](docs/PARAMETERS.md)、[驗證狀態](docs/VALIDATION.md)、[Isaac Sim 官方文件](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/)

本專案是在 NVIDIA Isaac Sim 中實作的純 Python 差速車 AMR 研究環境，聚焦於病房場景的建圖、定位、全域路徑規劃、局部追蹤與模擬證據保存。倉庫同時保存目前 V66 Continuous Navigation 2.0、歷史版本、車體 USD、實驗資料與驗證文件。

> **目前狀態：** 離線規劃與理想運動學回放已有結果；完整 Isaac Sim 動態驗收、滑移判讀、重複性實驗與實車性能仍須依[驗證狀態](docs/VALIDATION.md)重新確認。

---

## 一句話核心思路

> **以輪式里程計與 PhysX 2D 掃描建立占據格網和 SLAM 姿態，再用差速 Hybrid A* 搜尋可行路徑、以 CG 候選平滑降低不必要的原地旋轉，最後交由具速度包絡與碰撞預測的 RPP 追蹤器輸出 \(v,\omega\)；核心取捨是在狹窄病房環境中以完整車身可行性換取連續行駛。**

---

## 技術脈絡與定位

### 領域定位與相似技術對比

本專案屬於 AMR 的 2D 建圖、定位與導航系統，輸入包括差速輪組角度、模擬 LiDAR 掃描與目標位姿，輸出為差速車線速度與角速度命令。系統不是單一規劃器，而是將感測、SLAM、全域規劃、路徑平滑、速度規劃與局部控制串成可追溯的研究管線。

| 方法類別 | 代表方法 | 核心特色 | 主要缺陷 |
|---|---|---|---|
| 網格式全域規劃 | Dijkstra、A*、Nav2 Smac 2D | 在成本格網上搜尋位置路徑 | 不直接表達車身朝向與差速運動學 |
| 運動學搜尋 | Hybrid A*、State Lattice | 在 \(x,y,\theta\) 空間搜尋可執行運動基元 | 搜尋成本較高，參數與碰撞模型敏感 |
| 局部追蹤控制 | Pure Pursuit、RPP | 依參考路徑與前視點產生即時控制 | 不能取代全域規劃與可靠定位 |
| 本專案所屬 | V66 Continuous Navigation 2.0 | 差速 Hybrid A*＋Voronoi＋CG 平滑＋RPP | 尚未完成與 Nav2 基準的同條件比較 |

### 侷限性與已知問題

**文獻與專案紀錄自述的限制：**

- 完整病房／Gaussian 重建場景、統一環境快照與 GPU 驅動資訊未完整收錄。[資料缺口](docs/DATA_AVAILABILITY.md)
- 尚未完成 Nav2 Smac 2D、State Lattice 或其他基準方法的公平比較。[後續規劃](docs/NAV2_ROADMAP.md)
- 估計抵達與世界真值可能不同；單次實跑不能直接視為 2.0 的動態驗收。[驗證紀錄](docs/VALIDATION.md)
- 接觸 callback 為空時，不能據此判定「沒有接觸」或「沒有滑移」。[問題與證據](docs/ISSUES_AND_EVIDENCE.md)

**架構原生問題：**

- 離散分箱搜尋會受到 XY／yaw resolution、搜尋預算與啟發式權重影響。[領域通識]
- Voronoi 偏好只能提供 clearance tendency，不能取代完整車身輪廓碰撞檢查。[領域通識]
- Wheel odom 無法單獨辨識輪胎滑移；需要接觸、世界姿態或其他獨立觀測交叉驗證。[領域通識]
- RPP 的減速度、反應時間與側向加速度目前屬工程設定，不等同於實車量測值。[領域通識]

### 挑戰與創新對照表

| 現有方案的挑戰 | 此技術的解決方式與意義 |
|---|---|
| 純網格路徑可能不符合差速車朝向與轉向限制 | 以連續 \(x,y,\theta\) 狀態配合差速運動基元搜尋，直接檢查車身可行性 |
| 狹窄空間中路徑可能貼近障礙物 | 將占據、距離與 Voronoi 場納入代價，並在候選路徑上執行密集輪廓取樣 |
| 大轉角造成頻繁 spin 或換向 | 優先搜尋無 spin 路徑，失敗後才放寬為允許 spin 的搜尋，再以 CG 候選平滑 |
| 固定速度可能在彎道或障礙前過快 | 依曲率、角速度、側向加速度與輪速限制建立速度包絡，並加入反應與煞車預測 |
| SLAM 掃描與輪式里程計不同步 | 依掃描時戳插值 Wheel Odom，將新鮮度、配準接受與地圖保存分開記錄 |

---

## 步驟化流程與架構解析

```text
[輪關節角度] ──→ [Wheel Odom] ──┐
                                  ├→ [掃描時刻配對] → [SLAM／占據格網] → [A 位姿]
[PhysX 2D 掃描] ────────────────┘                                      │
                                                                       ↓
[拖曳 B：XY＋yaw] ───────────────────────────────→ [差速 Hybrid A*]
                                                          ↓
                                                [CG 平滑＋車身碰撞檢查]
                                                          ↓
                                                [曲率／輪速速度包絡]
                                                          ↓
                                             [RPP＋局部碰撞與煞車預測]
                                                          ↓
                                             [Python v,ω → 差速控制器]
```

### 步驟 1：建立輪式里程計與掃描時間基準

系統從實際輪關節角度積分 Wheel Odom，並以 PhysX POST step 產生 2D 碰撞射線掃描。掃描不直接套用最新姿態，而是依掃描時間在輪式里程計中插值，避免不同時基造成 SLAM 配準誤差。

\[
\omega_L=\frac{v-\omega b/2}{r},\qquad
\omega_R=\frac{v+\omega b/2}{r}
\]

> 直覺意義：給定車體線速度 \(v\) 與角速度 \(\omega\)，依輪半徑 \(r\) 和輪心距 \(b\) 換算左右驅動輪角速度。

**關鍵細節：**

- 掃描後端為 `PHYSX_COLLISION_RAYCAST`，名義上 720 束、10 Hz；實際接受與丟棄數需查 session 紀錄。
- SLAM 提交最短間隔為 0.2 s，與掃描頻率、Physics 更新頻率及畫面 FPS 不同。
- 車頭約定為 `base_link −X`；命令符號、關節方向與邏輯左右必須依設定核對。

### 步驟 2：SLAM 配準與占據格網更新

SLAM 將時間對齊後的掃描與 Wheel Odom 做掃描匹配及圖優化，持續估計地圖座標中的車體姿態，並以未知／自由／占據三種狀態保存占據格網。配準品質不足或資料過期時，系統會暫停地圖整合，而不是以無效掃描更新地圖。

**關鍵細節：**

- 地圖解析度為 0.05 m／格，名義畫布為 24 m；這是目前配置值，不代表所有場景都適用。
- A 點使用 SLAM 當前估計；世界位姿僅作獨立診斷與驗收參考，不取代導航估計。
- 建圖流程區分候選圖凍結、正式圖確認與工作紀錄匯出，避免操作語意混淆。

### 步驟 3：差速 Hybrid A* 全域規劃

規劃器在連續 XY／yaw 姿態上進行離散分箱搜尋，使用前進、倒退與原地旋轉等差速運動基元。V66 2.0 先搜尋不含 spin 的路徑，在預算內找不到可行解後才進入允許 spin 的回退搜尋。

\[
J = J_{\text{length}}+J_{\text{rotation}}+J_{\text{reverse}}
  +J_{\text{switch}}+J_{\text{Voronoi}}+J_{\text{control}}
\]

> 直覺意義：路徑不只追求短，也同時懲罰旋轉、換向、控制變化與靠近障礙物的行為。

**關鍵細節：**

- 目前分箱為 XY 0.1 m、yaw 72 格；運動基元時間為 0.65 s。
- 碰撞檢查採 0.025 m 取樣，並以導航整車包絡而非單一輪子判斷可行性。
- 啟發式權重為 1.2，最多 60,000 次 expansion、45 個現實秒；目前不宣稱具有最短路或可採納性保證。

### 步驟 4：CG 候選平滑與路徑驗證

對同方向行駛段建立連續終點候選，執行數值 CG 平滑，再以曲率與密集車身輪廓取樣驗證。細階段候選若不符合接受條件，可退回粗階段結果或原路徑，避免只因損失函數下降而輸出不可行路徑。

**關鍵細節：**

- 平滑只改善可行候選，不會覆蓋 spin 與前後換向等規劃動作語意。
- Voronoi 代價是偏好項，最後仍以車身幾何碰撞檢查為準。
- 2.0 的離線結果顯示細 CG 候選未通過全部接受條件，最後採用粗 CG 的檢查式插值。[證據](evidence/v66_2_0_offline/validation.json)

### 步驟 5：速度包絡與 RPP 局部控制

系統依路徑曲率、最大角速度、候選側向加速度與軟體輪速限制建立速度包絡，再加入加速與煞車掃描。RPP 追蹤前視點、接近終點時降速，並以反應時間與煞車距離預測局部碰撞；碰撞拒絕後先停穩，再交回全域規劃。

\[
v_{\text{curve}} \leq \sqrt{\frac{a_{\text{lat,max}}}{|\kappa|}}
\]

> 直覺意義：曲率越大，為限制側向加速度而允許的線速度越低。

**關鍵細節：**

- 導航最大線速度／角速度為 0.9 m/s／0.6 rad/s；實際輸出仍受速度包絡及停止交接限制。
- 目前反應時間 0.15 s、碰撞預測時間 0.6 s、候選側向加速度 0.30 m/s²，均屬工程設定。
- STOP 或局部碰撞拒絕時輸出零速，完成停穩確認後才允許重規劃。

---

## 實務需求與輸出入規格

### 輸入規格

| 項目 | 說明 |
|---|---|
| **主資料格式** | 輪關節角度／角速度；PhysX 2D raycast 掃描；占據格網；目前姿態與目標姿態 |
| **Prompt／條件輸入** | 目標 B 為世界 XY 座標與繞 Z 軸 yaw；導航 A 為 SLAM 估計姿態 |
| **尺寸／解析度需求** | 掃描 720 束、名義 10 Hz；地圖 0.05 m／格；最大掃描距離 12 m、近距離 0.1 m |
| **前處理需求** | 需完成輪關節綁定、掃描時戳配對、有效 PhysicsScene 與感測器外參核對 |
| **其他限制** | 必須使用與車體 USD、floor prim、SLAM 地圖合約相符的場景；不同 Isaac Sim 版本需重新驗證 |

### 輸出規格

- **SLAM 姿態**
  - 格式：\(x,y,\theta\) 連續姿態，數值型態與序列化格式依 runtime 輸出檔案為準 `[待驗證]`
  - 物理意義：車體在地圖座標中的估計位置與朝向
  - 下游用途：提供全域規劃 A 點、RPP 追蹤與地圖更新
- **占據格網**
  - 格式：0.05 m／格的未知、自由、占據格網；儲存格式依地圖輸出檔案為準
  - 物理意義：描述目前已觀測環境的可通行性
  - 下游用途：提供碰撞檢查、距離場與 Voronoi 代價
- **全域／局部路徑**
  - 格式：含 XY、yaw、曲率與動作段資訊的離散或連續路徑；完整欄位依 `hybrid_path.npz` 定義 `[待驗證]`
  - 物理意義：差速車可追蹤的幾何路徑與速度參考
  - 下游用途：CG 驗證、速度包絡與 RPP
- **差速控制命令**
  - 格式：Python `v, w`，單位分別為 m/s 與 rad/s；下游轉換為左右輪角速度
  - 數值範圍：受導航最大速度、加減速度、角加速度與軟體輪速限制
  - 物理意義：車體線速度與繞 Z 軸角速度

### 系統與算力需求

| 項目 | 說明 |
|---|---|
| **硬體需求** | 可執行 Isaac Sim 的 GPU／主機；確切 GPU VRAM、驅動與 Build ID 未完整收錄 `[待驗證]` |
| **即時性** | 目前資料顯示曾有顯著 real-time factor 限制；通用 FPS 或延遲目標未完成驗收 `[待驗證]` |
| **軟體相依** | Isaac Sim 6.0.1-rc.7（目前紀錄）；Python 離線檢查需求見 `requirements-offline.txt`；ROS 2／Nav2 尚未整合 |

### 安裝與操作

```bash
git clone https://github.com/Jimmy0108/Isaac_sim_sickroom_pathplanning.git
cd Isaac_sim_sickroom_pathplanning
git lfs pull
python -m pip install -r requirements-offline.txt
python tools/verify_repository.py
```

在 Isaac Sim Script Editor 中執行：

1. 先執行 `navigation/v66/CHECK_Installed_2_0.py`。
2. 再執行 `navigation/v66/AMR_V66_Navigation.py`。
3. 確認車體落地、PhysicsScene、六輪接觸、感測器外參與 floor prim 後再 PLAY。
4. 以遙控建立或載入地圖，A 取目前 SLAM 姿態，拖曳 B 設定目標 XY 與 yaw。
5. 先 Preview，再 Navigate；STOP 或 SPACE 可停止。

若 `verify_repository.py` 回報 `LFS pointer only`，代表尚未取得 Git LFS 實體檔案，不能視為資料完整。

---

## 效能與量化成效

**目前可追溯的離線結果：**

- 路徑長度：6.028271 m；單一前進段；spin 與換向各 0。[`navigation/v66/OFFLINE_VALIDATION.json`](navigation/v66/OFFLINE_VALIDATION.json)
- 理想差速回放時間：約 21.65 s；位置誤差 0.069581 m；方向誤差 2.384965°。[`docs/VALIDATION.md`](docs/VALIDATION.md)
- 細 CG 候選未通過全部接受條件，最後採用粗 CG 的檢查式插值。[`evidence/v66_2_0_offline/validation.json`](evidence/v66_2_0_offline/validation.json)

**測試環境：**

- 資料集：保存地圖與固定輸入的離線回放
- 硬體環境：`[待驗證]`
- Isaac Sim 動態結果：尚無足以支持通用性能、滑移率或重複性結論的同條件統計

相關文獻與專案資料未提供可直接外推至實車的 FPS、通用成功率或病房導航精度；上述數值只適用於其明示的離線輸入與驗證條件。

---

## 開源狀態與可復現性

| 項目 | 狀態 | 補充說明 |
|---|---|---|
| **訓練資料／Dataset** | 不適用 | 本專案不是以訓練資料為核心的機器學習模型；使用 Isaac Sim 場景、掃描、地圖與實驗 session |
| **模型架構／原始碼** | 公開 | 主要程式位於 [`navigation/v66/`](navigation/v66/)，歷史實作位於 [`archive/`](archive/) |
| **車體資產／USD** | 部分公開 | [`assets/v66/`](assets/v66/)；大型檔案使用 Git LFS，完整內容需執行 `git lfs pull` |
| **預訓練模型／權重** | 不適用 | 無預訓練神經網路權重 |
| **實驗證據** | 部分公開 | [`evidence/`](evidence/) 收錄選錄資料；完整病房場景與部分外部資料未收錄 |

**復現難度評估：** 離線驗證可在一般 Python 環境執行，但 Isaac Sim 動態重現需要相符的版本、場景、USD、PhysicsScene、感測器外參與 LFS 資產。完整動態驗收仍受環境快照、接觸資料有效性與重複實驗不足限制。

---

## 研究限制與後續工作

- 完成固定場景、起終點、地圖、車體外廓與速度條件下的多次動態實驗。
- 以世界真值、SLAM 姿態、輪式里程計與接觸資料分開評估定位、滑移與控制誤差。
- 與 Nav2 Smac 2D、State Lattice＋RPP 建立公平基準比較。
- 補齊 ROS 2／Nav2 介面、環境快照、GPU／驅動資訊與可重現部署文件。

---

## 引用資訊

| 項目 | 內容 |
|---|---|
| 主要實作 | Isaac Sim 病房 AMR 建圖與路徑規劃 |
| 版本 | V66 Continuous Navigation 2.0 |
| 官方 GitHub | <https://github.com/Jimmy0108/Isaac_sim_sickroom_pathplanning> |
| 架構文件 | [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) |
| 參數文件 | [`docs/PARAMETERS.md`](docs/PARAMETERS.md) |
| 驗證文件 | [`docs/VALIDATION.md`](docs/VALIDATION.md) |
| 方法參考 | [`docs/references/`](docs/references/) |
| ROS 2／Nav2 對照 | [`docs/NAV2_ROADMAP.md`](docs/NAV2_ROADMAP.md) |

本專案實作是差速車研究變體，不宣稱完整重現 Reeds–Shepp、Nav2 Smac 或原始論文方法；引用與實驗結論應以倉庫內的版本、證據與限制說明為準。
