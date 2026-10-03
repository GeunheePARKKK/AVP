#!/usr/bin/env bash
# ReBound 실행. 첫 인자는 ReBound 포맷 데이터셋 경로 (기본: session_025).
#
# ReBound 는 Python 3.8 + open3d 0.14.1 을 요구하지만 이 노트북은 3.10 이다.
# 최신 버전(open3d 0.20, numpy 1.26)으로 맞춰도 동작한다.
set -e
DATA="${1:-$HOME/ros2_ws/cam_lidar_recording/rebound/session_025}"
LIBS="${REBOUND_LIBS:-$HOME/.local/rebound_libs}"

if [ ! -d "$LIBS/open3d" ]; then
  echo "의존성 설치: $LIBS"
  pip3 install --target "$LIBS" open3d nuscenes-devkit pyquaternion matplotlib
fi

cd "$HOME/Documents/ReBound/src"
PYTHONPATH="$LIBS" python3 lct.py -f "$DATA"
