#!/usr/bin/env bash
# 팔 시험용 최소 브링업 (2026-11-13): OpenCR 두 보드 + 컨트롤러 + 키보드 리셋 (/joy) + manipulator_node 만.
# 카메라 · 라이다 · TFmini · 인식 · 계획기 · ekf 없음 -> 꺼내기 · 놓기 (arm_cmd.sh · dice.sh) 시험용 (집기 pick 은 팔 카메라 필요라 안 됨).
#
#   ~/arm_only.sh            보드 확인 (읽기만) -> 괜찮으면 브링업
#   ~/arm_only.sh --check    보드 확인만
#   ~/arm_only.sh --force    보드 확인 결과와 관계없이 브링업
# 띄운 뒤 다른 터미널에서 python3 ~/kbd_joy.py -> r (팔은 STOW 에 머묾, 2026-11-18), 그다음 ~/dice.sh real - <눈> --bench (STOW 면 먼저 pose DRIVE)
set -u
MODE="${1:-}"
ROS_WS="${ROS_WS:-$HOME/Lv2_Cartlidar_Assignment/workspace/ros2_ws}"
TOOL="$ROS_WS/src/opencr_hardware/scripts/opencr_arm_tool.py"
PORTS="${PORTS:-$(ls /dev/ttyACM* 2>/dev/null | tr "\n" " ")}"
set +u; source "$ROS_WS/install/setup.bash"; set -u

if pgrep -f "[r]os2_control_node" > /dev/null; then
  echo "!! 브링업 (ros2_control_node) 이 떠 있어 보드 포트를 쓰는 중 -> 보드를 확인할 수 없음. 그 터미널에서 Ctrl+C 로 끈 뒤 다시"; exit 2
fi
# 보드 확인: opencr_arm_tool.py monitor (포트에 쓰지 않음). 호스트 명령이 없으니 CMD_TIMEOUT · HOLD 는 정상,
#   SYNCREAD · MOTOR_HW · MOTOR_CONFIG · ROLE_MISMATCH · FAULT (0xB9) 가 있으면 모터를 못 읽는 것 (모터 전원 · 보드 리셋)
bad=0; roles=""
for p in $PORTS; do
  if [ ! -e "$p" ]; then echo "  $p: 없음 (USB 케이블)"; bad=1; continue; fi
  line=$(timeout 6 python3 "$TOOL" monitor --port "$p" --seconds 1.5 2>&1 | grep -m1 -o "role=[0-9] fw=[0-9]* .*err=0x[0-9A-Fa-f]*[^a]*")
  role=$(echo "$line" | grep -o "role=[0-9]" | cut -d= -f2)
  err=$(echo "$line" | grep -o "err=0x[0-9A-Fa-f]*" | cut -d= -f2)
  name=$([ "$role" = 2 ] && echo 팔 || ([ "$role" = 1 ] && echo 바퀴 || echo "?"))
  if [ -z "$err" ]; then echo "  $p: 응답 없음"; bad=1; continue; fi
  roles="$roles $role"
  if [ $(( err & 0xB9 )) -ne 0 ]; then
    echo "  $p ($name 보드): $(echo "$line" | grep -o "err=0x[0-9A-Fa-f]* [A-Z_|]*") -> 모터를 못 읽음"; bad=1
  else
    if [ "$role" = 2 ] && [ $(( err & 0x40 )) -ne 0 ]; then
      echo "  $p (팔 보드): 오류 $err = HOLD (보드가 재부팅 안 됨 — 토크가 안 걸린 채일 수 있음). 모터 전원 켠 채 팔 보드 USB 뽑았다 꽂기 · 리셋 권장"
    else
      echo "  $p ($name 보드): 괜찮음 (오류 $err)"
    fi
  fi
done
case "$roles" in *1*) ;; *) echo "  바퀴 보드 (role 1) 없음 — 하드웨어가 두 보드 모두 있어야 시작함"; bad=1;; esac
case "$roles" in *2*) ;; *) echo "  팔 보드 (role 2) 없음"; bad=1;; esac
if [ "$bad" = 1 ]; then
  echo "!! 보드 문제: 모터 전원 (배터리 스위치) 을 켜고, 보드 USB 를 뽑았다 꽂거나 리셋 버튼 -> 다시 ~/arm_only.sh"
  [ "$MODE" = --force ] || exit 1
  echo "   (--force: 그래도 띄움)"
fi
[ "$MODE" = --check ] && exit 0
if pgrep -f "[r]os2_control_node" > /dev/null; then
  echo "!! 브링업 (ros2_control_node) 이 이미 떠 있음 -> 그 터미널에서 Ctrl+C 로 끈 뒤 다시"; exit 2
fi


# USB 허브가 다시 잡히면 포트 번호가 바뀜 (ttyACM2 · 3 등) -> 지금 있는 두 포트를 넘김 (브링업 기본값은 ttyACM0,1)
exec ros2 launch bringup bringup.launch.py skill_enabled:=true sim:=false ports:="$(echo $PORTS | tr ' ' ',')" \
  use_sensors:=false use_perception:=false use_tfmini:=false use_planner:=false use_ekf:=false
