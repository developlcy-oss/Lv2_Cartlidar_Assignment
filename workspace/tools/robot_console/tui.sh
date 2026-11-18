#!/usr/bin/env bash
# 로봇 콘솔 TUI (Jetson, 다른 터미널): 비상정지 · 수동 조작 · 모드 바꾸기. 끄기: Esc (로봇 · 모드는 그대로)
source "$(dirname "$0")/env.sh" || exit 1
exec python3 "$HERE/console_tui.py"
