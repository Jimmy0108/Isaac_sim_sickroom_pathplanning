# 操作指南

## 本地檢查

```bash
python -m pip install -r requirements-offline.txt
python tools/verify_repository.py
```

Isaac 專用入口應在 Isaac Sim Script Editor 使用。電腦上的語法與雜湊檢查只代表封裝一致。

## 遠端安裝

目前入口與安裝檢查內採用以下 PROJECT。

```text
/isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/new_car/python_nav/V66_Python_Navigation
```

將 navigation/v66 的檔案平鋪到這裡。避免同名資料夾多套一層。若使用不同遠端目錄，須同步調整入口與安裝檢查路徑，更新發布雜湊後再使用。

STOP 後等待舊工作清理，備份 Python／JSON。保留原 slam_maps、slam_outputs、validation_outputs。完成部署後開啟本次入口，RUN → PLAY。

## 車體

主場景外層沿用 /World/AGV_configured_v4，引用 assets/v66/AGV_configured_v66.usd。/World/AGV_control 引用 AGV_control_v66.usd。檢查 DifferentialController 半徑 0.08 m、輪心距 0.476966833 m 及 ArticulationController 目標。

assets/v66 的 V66_TestScene.usda 是小型落地測試，地板叫 /World/Ground，原有導航配置則指向 /World/Colider/floor。場景不同時應核對 floor 路徑，不能直接拿落地場景當成病房完整驗收。

確認有效 PhysicsScene、六輪接觸、車身與感測器外參後，再啟動移動。候選材質工具與前左 swivel 工具的作用層不同，詳見問題紀錄。

## 四方向

前進與後退目標 ±0.9 m/s，左轉與右轉目標 ±0.9 rad/s。實際命令受加減速及停止交接限制。手動操作無航向或橫向補償。按 SPACE 或 STOP 停止，切換移動種類先驗證停穩。

若 w＝0 仍偏行，匯出並核對命令、實際輪速、輪心運動、萬向輪軸角與接觸。導航追蹤的轉向修正與手動補償有不同用途。

## 建圖

1. Start NEW map 建立本次地圖原點。
2. 遙控覆蓋可通行區，慢速轉彎並回訪已掃描位置。
3. Finish mapping/use candidate 等待停穩及待處理資料，凍結候選圖。
4. 檢查後 Confirm candidate/archive/save，更新正式 slam_maps 並保留舊圖。
5. Stop/export 輸出工作紀錄 ZIP。

載入舊圖時等待定位通過。車體、外參、後端與地圖合約必須相符。灰色未知區維持未知。

## 導航

A 為 SLAM 當前估計。Create/select B 後拖曳世界 XY，繞 Z 軸調整最後朝向，忽略高度及傾斜。面板同時顯示世界與地圖座標。

Preview 顯示綠色全域路徑、黃色局部預測、藍色估計軌跡及紅色 B 朝向。Navigate 後依 RPP 輸出唯一 v、w 命令。位置 7 cm、方向 5° 且停穩才可顯示估計抵達。

實際位置精度另用世界真值評分。新版本的弧形影片顯示連續轉向，實際淨空、時間與精度仍需數值紀錄。

## 回退

STOP → 等待清理 → 還原完整的舊發布包與配置 → 重新開啟該版入口 RUN → PLAY。補丁包僅覆蓋其明示檔案。Git tag、來源索引與套件內 manifest 可核對版本，勿把不同版核心混用。
