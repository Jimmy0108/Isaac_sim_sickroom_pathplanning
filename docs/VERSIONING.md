# 版本管理與工作方式

## 主要來源

navigation/v66 是 2026-11-07 Continuous 2.0 的逐位元快照。AMR_V66_Navigation.py 是唯一主要入口。檔內歷史 docstring 仍可能標示 entry 1.0.2，實際發布辨識應讀 RELEASE_MANIFEST_2_0.json 與 v66_app 版本。

## 歷史資料

| 目錄 | 階段 |
|---|---|
| archive/20261014_world_pose | 世界姿態、格網與早期停車保護 |
| archive/20261021_offline_hybrid | 純離線差速 Hybrid A*＋Voronoi |
| archive/20261021_isaac_world_pose | 初期 Isaac 介接 |
| archive/20261025_teleop_mapping | 手動移動建圖與 B 點 |
| archive/20261025_encoder_slam | 輪式 odom、SLAM、RPP |
| archive/v10_package | 使用者提供的舊車移植來源 |
| archive/v66_initial | V66 1.0 完整來源 |
| archive/v66_smooth_1_1 | 分段平滑版本 |
| archive/v66_smooth_1_1_1_patch | 重規劃計時補丁，非獨立完整包 |
| archive/asset_build | 當時的 USD 移植腳本 |

歷史 README 保留當時的路徑、操作與限制。讀取 archive 前先核對版本，避免將舊輪徑輪距重新套進新車。

## Git 使用

main 保存整理後可追溯的發布版本。後續修改使用 codex/ 開頭分支，完成離線檢查與場景實跑後，再以 commit 與 tag 記錄。

```bash
git switch -c codex/next-navigation-change
python tools/verify_repository.py
git diff
git add navigation docs evidence tools
git commit -m "Describe the navigation change and evidence"
git push -u origin codex/next-navigation-change
```

主程式有安裝 hash check。修改 runtime 檔案後需更新 RELEASE_MANIFEST_2_0.json，重新部署該版本整包並保留實跑配置。source_manifest.json 是初始匯入基準，後續有意修改的來源應同步更新基準或在新版本清單註明。

## 大檔

USD、USDC、測試影片與大型接觸／診斷資料採 Git LFS。首次 clone 後執行 git lfs pull。勿以只含 LFS pointer 的 ZIP 當成完整可用模型。

## 來源可追溯

docs/source_manifest.json 記錄來源相對位置、檔案大小與 SHA-256。zip!/ 表示從原始 ZIP 選取的檔案。external_progress_report 表示從使用者提供的進度報告資料夾取得影片。

archive/asset_build 的腳本保留原工作區路徑。對應輸入已收錄 assets/migration_inputs，在新 checkout 重新生成前應先調整輸入與輸出位置。已生成 configured／control 可直接用相對引用載入。
