#!/usr/bin/env bash
# 인지 시연: /perception/set_mode 로 인지 기능을 켜고 붙잡아 둠 (결과는 log.sh 창). 끝날 때 0 으로 끔.
#   $1 = 모드 비트 (기본 57 = SEARCH_FRONT 1 + SEARCH_REAR 8 + READ_MARKER 16 + READ_SIGNAL 32). 바코드 64 는 팔 카메라가 라벨을 봐야 함
M=${1:-57}
for i in $(seq 1 60); do ros2 service list 2>/dev/null | grep -qx /perception/set_mode && break; sleep 2; done
call() { ros2 service call /perception/set_mode robot_interfaces/srv/SetPerceptionMode "{mode: $1}"; }
trap 'echo "인지 끔"; call 0; exit 0' INT TERM
call "$M" || exit 1
echo "인지 모드 $M 켬 — 결과는 로그 창 ([신호등] · [퍽] · 장소 태그). 끄려면 TUI 에서 다른 모드 · '.'"
while true; do sleep 1; done
