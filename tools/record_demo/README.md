# 錄製示範影片（tools/record_demo）

一個指令重新錄製 `docs/demo.mp4`、`docs/images/demo.gif`，並更新 README 用的截圖。
GUI 改版、外觀調整之後，可以用它重錄。

```bash
cd ~/puma560_ws
colcon build && source install/setup.bash
tools/record_demo/record_demo.sh            # 輸出到 docs/（會覆蓋）
tools/record_demo/record_demo.sh --out /tmp/demo_test   # 先輸出到其他目錄看看
tools/record_demo/record_demo.sh --keep     # 保留中間檔（影格、記錄檔）供除錯
```

整個流程約 2～3 分鐘。執行期間會出現 Gazebo 和 GUI 視窗，GUI 由腳本自動操作，**請不要去動它**。

## 需求

- 已 `colcon build`
- 有圖形桌面（`DISPLAY`）
- `ffmpeg`：`sudo apt install ffmpeg`
- 中文字型 Noto Sans CJK TC：`sudo apt install fonts-noto-cjk`
- 執行前不能有其他 Gazebo 在執行（腳本會檢查並停止，不會自動關掉你的程序）

## 流程與檔案

| 檔案 | 作用 |
|---|---|
| `record_demo.sh` | 主控腳本：檢查環境 → 啟動模擬 → 錄製 → 合成 → 編碼 → 輸出 |
| `make_demo_world.py` | 由 `teaching.sdf` 產生錄影用 world，加上 Sensors 系統與一台固定視角相機（1280×960、15 Hz） |
| `cam_recorder.py` | 訂閱 `/demo_cam`（經 `ros_gz_bridge`），以 wall-clock 時間戳記把影格存成 JPEG |
| `demo_driver.py` | 依時間軸自動操作教學 GUI（拖曳拉桿、切換矩陣、IK…），擷取 GUI 影格、字幕時間軸與 GUI 截圖 |
| `compose.py` | 依時間戳記把 Gazebo 與 GUI 影格並排、加上字幕列，並擷取 Gazebo 截圖 |

輸出：

| 檔案 | 說明 |
|---|---|
| `demo.mp4` | 1920×720、15 fps、H.264，約 55 秒 |
| `images/demo.gif` | 1280 寬、10 fps，README 首頁使用 |
| `images/gui_fk.png`、`gui_ik.png`、`gui_singular.png` | GUI 截圖 |
| `images/gazebo_exercise.png`、`gazebo_ik_solution7.png` | Gazebo 截圖（同一個 ⁰₆T、兩種構型） |

## 修改示範內容

- **示範步驟、字幕**：編輯 `demo_driver.py` 的 `run_demo()`。
  `caption()` 設定字幕、`move_to()` 平滑移動拉桿、`gui_shot()` / `cam_shot()` 指定截圖時間點。
- **相機視角**：編輯 `make_demo_world.py` 開頭的 `CAM_POS`、`CAM_TARGET`、`HFOV`。
- **版面、字型**：編輯 `compose.py` 開頭的常數。

## 設計說明

- **為什麼不直接錄螢幕？** 錄螢幕會錄到桌面上的其他視窗（瀏覽器等），而且視窗被擋住就錄不到。
  所以 Gazebo 用場景中的相機感測器、GUI 用 Qt 擷取視窗本身，兩者都和螢幕上的畫面無關。
- **為什麼不用 Gazebo GUI 的截圖服務？** 用指令列呼叫 `/gui/screenshot` 每秒只能截約 2.5 張，影片會很卡。
- **背景程序管理**：每個背景程序都用 `setsid` 放進獨立的 process group，結束時（包含出錯或 Ctrl+C）整組關閉，
  不會留下孤兒 Gazebo，也不會影響其他程序。出錯時會保留中間檔並顯示位置。
