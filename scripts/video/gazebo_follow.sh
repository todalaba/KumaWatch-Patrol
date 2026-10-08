#!/usr/bin/env bash
# Gazebo の画面(gzclient)のカメラを、ロボットを追いかけるように切り替える(動画撮影用)。
#
#   ./scripts/video/gazebo_follow.sh             # $TURTLEBOT3_MODEL(講義環境は happy_mini)を追う
#   ./scripts/video/gazebo_follow.sh waffle      # モデル名を指定
#
# Gazebo の画面で、左の World タブ → Models → ロボット名 を右クリック → Follow でも同じことができる。
# 俯瞰に戻すときは、同じメニューで追従を解除するか、Gazebo を起動し直す。
# 画面に映さずに追従映像だけ録りたいときは、chase_camera ノード(docs/video_plan.md)を使う。
set -euo pipefail
MODEL="${1:-${TURTLEBOT3_MODEL:-waffle}}"
if ! command -v gz >/dev/null 2>&1; then
  echo "gz コマンドがありません(source /usr/share/gazebo/setup.sh を実行してください)"; exit 1
fi
echo "Gazebo の画面のカメラを ${MODEL} に追従させます"
gz camera -c gzclient_camera -f "${MODEL}"
