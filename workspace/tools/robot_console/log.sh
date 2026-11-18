#!/usr/bin/env bash
# 로봇 콘솔 로그 보기 (Jetson, 다른 터미널). 읽기만 함. 끄기: Ctrl+C
source "$(dirname "$0")/env.sh" || exit 1
exec python3 "$HERE/console_log.py"
