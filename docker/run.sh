#!/usr/bin/env bash
# X11 を使って Gazebo / RViz をホストの画面に表示する(Linux ホスト向け)
set -e
xhost +local:root >/dev/null 2>&1 || true
docker run -it --rm \
  --net=host \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$(pwd)/output:/root/kumawatch_patrol_output" \
  ${GPU_FLAGS:-} \
  kumawatch_patrol "$@"
