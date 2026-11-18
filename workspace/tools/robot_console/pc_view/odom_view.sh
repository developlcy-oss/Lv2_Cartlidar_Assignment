#!/usr/bin/env bash
# PC 감시 창: 신호등 정답 순서 (PC 에 꽂은 신호등 OpenCR) + 로봇 odom 오차 (Jetson 에서 ssh 로 받음, 로봇 쪽은 읽기만)
#   ./odom_view.sh                       신호등 /dev/ttyACM0 · Jetson cart11@cart11.local
#   SERIAL=/dev/ttyACM1 JETSON=… ./odom_view.sh
# Jetson 에 로봇 콘솔 (start.sh) 이 떠 있어야 자료가 옴. 공유 연결 소켓 (~/.ssh/lv2_jetson.sock) 이 있으면 그것으로, 없으면 비밀번호를 물음
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../../../.." && pwd)"
JETSON="${JETSON:-cart11@cart11.local}"
SERIAL="${SERIAL:-/dev/ttyACM0}"
MAP="${MAP:-$REPO/workspace/ros2_ws/src/mission_planner/maps/map_amcl.yaml}"
PY="${PY:-$HOME/venvs/cad/bin/python}"
"$PY" -c "import matplotlib, PIL, yaml" 2>/dev/null || PY=python3
"$PY" -c "import matplotlib" 2>/dev/null || { echo "✗ matplotlib 있는 파이썬 없음 (PY=… 로 지정)"; exit 1; }
SOCK="$HOME/.ssh/lv2_jetson.sock"
SSH="ssh"; [ -S "$SOCK" ] && SSH="ssh -o ControlPath=$SOCK"
CMD="$SSH $JETSON 'source /opt/ros/lyrical/setup.bash; exec python3 -u -' < '$HERE/odom_stream_jetson.py'"
exec "$PY" "$HERE/odom_view.py" "$CMD" "$SERIAL" "$MAP"
