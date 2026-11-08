# 未來 ROS 2／Nav2 對照

目前程式在 Isaac Sim 內用純 Python 運作。後續導入 ROS 2 時，可將感測、定位、地圖、全域規劃與控制替換成 Nav2 對應元件。

| 現有介面 | 未來對應 |
|---|---|
| 時戳掃描 | sensor_msgs/LaserScan |
| Wheel odom | nav_msgs/Odometry 與 odom → base_link |
| SLAM 地圖與姿態 | SLAM 節點、map → odom |
| 占據格網與局部障礙 | global/local costmap |
| Hybrid A*＋Voronoi | 自訂 planner plugin 或選用 Smac |
| RPP 核心 | Nav2 Regulated Pure Pursuit |
| Python v,w → Graph | cmd_vel → 差速驅動 |
| 拖曳 B | Nav2 goal pose，包含 XY 與 yaw |

## 討論過的比較

Smac 2D＋RPP 可作差速車基準。Smac 2D 搜尋平面位置並使用成本場，RPP 負責追蹤和曲率限速。狹窄空間中，矩形車身朝向會影響可通行性，需核對完整輪廓。State Lattice 可納入差速運動基元與朝向。

弧形方案可減少部分非必要停轉，但可能增加路長與橫向占用。比較時固定起終點、目標朝向、地圖、外框、速度及定位來源，記錄通過時間、最小淨空、追蹤誤差、停車旋轉及換向次數。

## 官方參考

- [Smac 2D](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_2d/configuring_smac_2d/)
- [RPP](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)
- [State Lattice](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/)
- [Isaac Sim 效能手冊](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/reference_material/sim_performance_optimization_handbook.html)
- [Isaac Sim 物理基礎](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/physics/simulation_fundamentals.html)

以上為後續接入規劃，倉庫尚無 ROS 2 套件或 Nav2 YAML 部署成果。
