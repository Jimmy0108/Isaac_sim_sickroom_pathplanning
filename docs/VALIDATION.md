# 驗證狀態

## 主程式 2.0

原發布包包含 OFFLINE_VALIDATION.json、INTEGRATION_CHECKS.json、RELEASE_MANIFEST_2_0.json。本次保留其原始位元組及來源雜湊。

保存地圖上的新路徑長 6.028271 m，單一前進段，spin 及換向各 0。理想差速回放約 21.65 s 抵達，位置誤差 0.069581 m，方向誤差 2.384965°。粗 CG 目標值改善，細 CG 候選未通過全部接受條件，最後採粗 CG 的檢查式插值。

既有離線測項含必要 spin 回退、不可行終點、停止交接、軟體輪速包絡、LiDAR／煞車碰撞拒絕、Script Editor 分離命名空間與重規劃計時。結果是舊發布時的離線紀錄，本次封裝檢查另記 repository_review.json。

## 實跑紀錄

| 來源 | 能支持的結論 |
|---|---|
| mapping_session_1793961409915912543 | 動態輪組異常分析的來源，最大傾角約 0.50°為前次分析值 |
| mapping_session_1793963081030061346 | V66 1.0 流程回報估計抵達、最終零命令、停穩 confirmed。summary 仍標 map_accuracy=REQUIRES_REVIEW |
| mapping_session_1793975051855213003 | 候選材質低速測試，1.878 m 前進及 yaw +2.263°，非同條件 A/B |
| mapping_session_1793978783186786405 | Smooth1.1 規劃 spin/cusp 與重規劃計時錯誤的來源 |
| wheel_contact_1793978770577875223 | 接觸採集無效，0 callbacks |
| 測試1、測試2影片 | 可觀察連續弧形轉向，無量測驗收表及同條件舊版對照 |

V66 1.0 的實跑成功回報不能直接作為 2.0 的動態驗收。估計位置與世界真值亦須分開計算。

## 本次倉庫檢查

tools/verify_repository.py 驗證來源雜湊、目前發布 manifest、Python 語法、JSON 語法、輪徑輪距一致性及 Git 大檔策略。此檢查可在一般 Python 執行，不啟動車輛。

完整原始 ZIP 仍在工作資料夾。倉庫收錄與導航問題判讀相關的資料，選取來源及 SHA-256 可查 source_manifest.json。
