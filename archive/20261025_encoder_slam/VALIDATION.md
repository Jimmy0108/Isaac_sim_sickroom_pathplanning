# 驗證記錄（2026-10-25）

- 30 項離線測試通過：7 占據地圖、8 規劃、15 SLAM/里程計/RPP/資料邊界。
- Synthetic SLAM：129 幀、83 keyframes、4 loop constraints；Odom 終點誤差 0.7019 m，SLAM 0.01521 m。
- Synthetic SLAM 地圖上的 Hybrid / Voronoi + 理想 RPP：601 expansions，601 steps，終點誤差 0.06957 m / 0.08527 rad。
- 入口與核心全部通過 Python 語法編譯。
- Source boundary 測試確認估計器與控制 runtime 不呼叫車身世界姿態；輪角介面讀 get_dof_positions 而不是 targets。
- World display anchor 與靜態雷射安裝關係只在初始化擷取；目標與初始姿態標記是使用者輸入。
- 未進行實際 Isaac Sim、WebRTC、輪 DOF/軸向、PhysX 摩擦、腳輪或真實病房驗證。
- 合成測試不能作為實車精度或安全保證。姿態圖、ICP、回環與 RPP 都是實驗實作。
