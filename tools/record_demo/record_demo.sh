#!/usr/bin/env bash
# =====================================================================
# 錄製 PUMA560 教學模擬的示範影片，並更新 README 用的截圖
#
#   tools/record_demo/record_demo.sh            # 輸出到 docs/
#   tools/record_demo/record_demo.sh --keep     # 保留中間檔（影格、記錄檔）供除錯
#   tools/record_demo/record_demo.sh --out DIR  # 輸出到其他目錄（不覆蓋 docs/）
#
# 流程：
#   1. 由 teaching.sdf 產生加上相機的錄影 world（make_demo_world.py）
#   2. 啟動 Gazebo + 控制器（gui:=false），橋接相機影像
#   3. cam_recorder.py 錄 Gazebo 影格；demo_driver.py 自動操作 GUI 並錄 GUI 影格
#   4. compose.py 依時間戳記並排合成，ffmpeg 編成 MP4 與 GIF
#
# 輸出：<OUT>/demo.mp4、<OUT>/images/demo.gif、<OUT>/images/*.png
# 需求：已 colcon build、有圖形桌面（DISPLAY）、ffmpeg、Noto Sans CJK TC 字型
# =====================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../.." && pwd)"
OUT="$WS/docs"
KEEP=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --keep) KEEP=1; shift ;;
    --out)  OUT="$(realpath -m "$2")"; shift 2 ;;
    -h|--help) sed -n '2,19p' "$0"; exit 0 ;;
    *) echo "未知參數：$1（用 --help 查看用法）" >&2; exit 2 ;;
  esac
done

say() { echo -e "\033[1;36m[record_demo]\033[0m $*"; }
die() { echo -e "\033[1;31m[record_demo] 錯誤：\033[0m$*" >&2; exit 1; }

# ---------------- 環境檢查 ----------------
[[ -n "${DISPLAY:-}" ]] || die "需要圖形桌面（DISPLAY 未設定）"
command -v ffmpeg >/dev/null || die "找不到 ffmpeg：sudo apt install ffmpeg"
# 注意：pipefail 下不用 grep -q（提早結束會讓前面的指令收到 SIGPIPE 而被判定失敗）
fc-list | grep "Noto Sans CJK TC" >/dev/null || die "找不到 Noto Sans CJK TC 字型：sudo apt install fonts-noto-cjk"
[[ -f "$WS/install/setup.bash" ]] || die "請先在 $WS 執行 colcon build"
if pgrep -f "ign gazebo" >/dev/null; then  # pgrep 不在 pipeline 中，沒有 SIGPIPE 問題
  die "已經有 Gazebo 在執行，請先關閉（必要時：pkill -f \"ign gazebo\"）"
fi

set +u   # ROS 的 setup.bash 會用到未定義的變數
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

WORK="$(mktemp -d /tmp/puma560_demo.XXXXXX)"
RUN="$WORK/run"
mkdir -p "$RUN"
PIDS=()

# 每個背景程序都用 setsid 放進自己的 process group，結束時整組關掉，
# 不用 pkill -f（避免比對到自己，也避免 Gazebo 子程序變成孤兒）
start_bg() {
  local log="$1"; shift
  setsid "$@" >"$WORK/$log" 2>&1 &
  PIDS+=("$!")
}

stop_group() {
  local pid="$1"
  kill -INT -- "-$pid" 2>/dev/null || return 0
  for _ in $(seq 1 40); do kill -0 -- "-$pid" 2>/dev/null || return 0; sleep 0.25; done
  kill -TERM -- "-$pid" 2>/dev/null || true
  sleep 2
  kill -KILL -- "-$pid" 2>/dev/null || true
}

cleanup() {
  local rc=$?
  set +e
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do stop_group "${PIDS[$i]}"; done
  # 保險：只清除使用這次錄影 world 的 Gazebo（路徑含唯一的暫存目錄名稱）
  pkill -f "$WORK/demo_world.sdf" 2>/dev/null
  if [[ $KEEP -eq 1 || $rc -ne 0 ]]; then
    say "中間檔保留在 $WORK"
  else
    rm -rf "$WORK"
  fi
  exit $rc
}
trap cleanup EXIT
trap 'exit 130' INT TERM

wait_until() {   # wait_until <秒數> <說明> <指令...>
  local sec="$1" what="$2"; shift 2
  for _ in $(seq 1 "$sec"); do "$@" >/dev/null 2>&1 && return 0; sleep 1; done
  die "等待逾時：$what（記錄檔在 $WORK）"
}
controllers_active() { ros2 control list_controllers 2>/dev/null | grep "forward_position_controller.*active" >/dev/null; }
cam_frames_ready()   { [[ $(ls "$RUN/cam" 2>/dev/null | wc -l) -ge 5 ]]; }

# ---------------- 1. 錄影 world ----------------
say "產生錄影用 world"
python3 "$HERE/make_demo_world.py" "$WS/src/puma560_gazebo/worlds/teaching.sdf" "$WORK/demo_world.sdf"

# ---------------- 2. 啟動模擬 ----------------
say "啟動 Gazebo 與控制器"
start_bg launch.log ros2 launch puma560_gazebo teaching.launch.py gui:=false world:="$WORK/demo_world.sdf"
wait_until 120 "控制器啟動" controllers_active

say "橋接相機影像並開始錄製"
start_bg bridge.log ros2 run ros_gz_bridge parameter_bridge "/demo_cam@sensor_msgs/msg/Image[ignition.msgs.Image"
start_bg cam.log python3 "$HERE/cam_recorder.py" "$RUN/cam"
CAM_PID="${PIDS[-1]}"
wait_until 30 "相機影格" cam_frames_ready

# ---------------- 3. 自動示範 ----------------
say "執行示範（約 1 分鐘）"
python3 "$HERE/demo_driver.py" "$RUN"
sleep 1
stop_group "$CAM_PID"
wait "$CAM_PID" 2>/dev/null || true
say "相機影格 $(ls "$RUN/cam" | wc -l) 張，GUI 影格 $(ls "$RUN/gui" | wc -l) 張"

# 錄完就可以關閉模擬，後面只做影像處理
for ((i=${#PIDS[@]}-1; i>=0; i--)); do stop_group "${PIDS[$i]}"; done
PIDS=()

# ---------------- 4. 合成與編碼 ----------------
say "合成影格"
python3 "$HERE/compose.py" "$RUN" "$WORK/frames"

mkdir -p "$OUT/images"
say "編碼 MP4"
ffmpeg -y -loglevel error -framerate 15 -i "$WORK/frames/f_%05d.jpg" \
  -c:v libx264 -preset slow -crf 24 -pix_fmt yuv420p -movflags +faststart "$OUT/demo.mp4"
say "編碼 GIF"
ffmpeg -y -loglevel error -framerate 15 -i "$WORK/frames/f_%05d.jpg" \
  -vf "fps=10,scale=1280:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=160:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle" \
  "$OUT/images/demo.gif"
cp "$RUN"/shots/*.png "$OUT/images/"

say "完成："
ls -la "$OUT/demo.mp4" "$OUT/images/"
