#!/usr/bin/env bash
# 로봇 콘솔 시작 (Jetson, 첫 터미널). 콘솔 데몬이 모드마다 bringup · 할 일을 띄우고 끔. 끄기: Ctrl+C (모드도 모두 꺼짐)
#   ./start.sh        → 모드 0 (대기 · 수동 조작) 으로 시작
#   ./start.sh 1      → 그 모드로 시작          ./start.sh -  → 아무것도 띄우지 않고 기다림
# 다른 터미널: ./log.sh (로그) · ./tui.sh (수동 조작 · 모드 바꾸기)
source "$(dirname "$0")/env.sh" || exit 1
command -v ros2 >/dev/null || { echo "✗ ros2 없음"; exit 1; }
if pgrep -f "robot_console/console_daemon.py" >/dev/null; then echo "✗ 콘솔이 이미 떠 있음 (pgrep -af console_daemon)"; exit 1; fi
echo "로봇 콘솔 — 모드 ${1:-0} 로 시작. 로그 ~/robot_logs/ · 다른 터미널에서 ./log.sh · ./tui.sh"
exec python3 "$HERE/console_daemon.py" "${1:-0}"
