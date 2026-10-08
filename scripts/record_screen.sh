#!/usr/bin/env bash
# デスクトップ画面(Gazebo・RViz・端末)を MP4 に録画する(講義の noVNC 環境・Ubuntu 用)。
#
#   ./scripts/record_screen.sh                 # 画面全体を録画。Ctrl+C で停止・保存
#   ./scripts/record_screen.sh -r 20 -o demo.mp4
#   ./scripts/record_screen.sh -s 1280x720     # 画面サイズを指定(自動取得できない場合)
#
# 必要: ffmpeg(無ければ sudo apt install -y ffmpeg で入れる)
# 出力: 既定で ~/kumawatch_patrol_output/videos/screen_<日時>.mp4
#       と、録画開始の時刻を書いた screen_<日時>.json(動画の自動編集 scripts/video/edl_from_run.py 用)
set -euo pipefail

FPS=15
SIZE=""
OUT="${HOME}/kumawatch_patrol_output/videos/screen_$(date +%Y%m%d_%H%M%S).mp4"
while getopts "r:o:s:h" opt; do
  case "$opt" in
    r) FPS="$OPTARG" ;;
    o) OUT="$OPTARG" ;;
    s) SIZE="$OPTARG" ;;
    *) sed -n '2,10p' "$0"; exit 0 ;;
  esac
done

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "ffmpeg がありません。インストールします (sudo apt install -y ffmpeg)"
  sudo apt-get update -q && sudo apt-get install -y -q ffmpeg
fi

DISPLAY_NAME="${DISPLAY:-:1}"
if [ -z "$SIZE" ] && command -v xdpyinfo >/dev/null 2>&1; then
  SIZE=$(xdpyinfo -display "$DISPLAY_NAME" | awk '/dimensions:/{print $2}')
fi
if [ -z "$SIZE" ] && command -v xrandr >/dev/null 2>&1; then
  SIZE=$(xrandr --display "$DISPLAY_NAME" 2>/dev/null | awk '/\*/{print $1; exit}')
fi
SIZE="${SIZE:-1920x1080}"
mkdir -p "$(dirname "$OUT")"

echo "録画開始: display=${DISPLAY_NAME} size=${SIZE} fps=${FPS}"
echo "保存先  : ${OUT}"
echo "Ctrl+C で停止します"
# 録画開始の時刻(ミッションの出来事の時刻と突き合わせて、動画を自動で切り出すため)
printf '{"start_wall": %s, "size": "%s", "fps": %s}\n' "$(date +%s.%N)" "$SIZE" "$FPS" \
  > "${OUT%.*}.json"
# -preset ultrafast: CPU 負荷を下げる(Gazebo と同時に動かすため)
# yuv420p: Windows / Mac / スマホの標準プレーヤーで再生できる形式
# crop: H.264 は幅・高さが偶数でないと作れない(noVNC の画面は 1419x1082 のように奇数になることがある)
exec ffmpeg -hide_banner -loglevel warning -y \
  -f x11grab -framerate "$FPS" -video_size "$SIZE" -i "${DISPLAY_NAME}" \
  -vf "crop=trunc(iw/2)*2:trunc(ih/2)*2" \
  -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p "$OUT"
