#!/usr/bin/env bash
# up.sh — 젯슨에서 bringup 을 순서대로 띄우고 단계마다 준비를 확인 (11-23)
#
#   ./up.sh drive [run_demo 인자...]     # 주행 시연 (run_demo.sh real "$@")
#   ./up.sh mission [bringup 추가 인자...]   # 미션 (READY · REARM 뒤 Enter 로 /mission/start)
#                                        #   ⚠ 기본으로 max_wz:=0.5 (회전 상한 0.5 rad/s) 를 줌 — 바꾸려면 max_wz:=값 을 뒤에 붙임
#   ./up.sh down                         # 남은 ROS 프로세스 목록 (사용자 · tty · 경과) 을 보여 주고 kill 을 입력하면 정리
#
# ⚠ 젯슨 화면이나 tmux 에서 실행하세요 (ssh 가 끊기면 정리가 제대로 안 될 수 있음). 물리 비상정지는 없음 — 리모컨 1 이 정지 수단.
# 환경변수: REPO (기본 ~/Lv2_Cartlidar_Assignment) · DEMO (기본 ~/demo_v2) · UP_DRY=1 (명령만 찍음)
#   UP_KILL_LEFTOVER=1 (정리 때 그룹 밖으로 나간 우리 자손도 끔) · UP_ALLOW_MISMATCH=1 (install ≠ REPO 여도 진행)
#   ARM_LOOK=0 (drive ④ 건너뜀) · UP_ARM_CAMERA=0 · UP_SKIP_ARM_WARN=1 · UP_LC_STUCK=25 (lifecycle 개입 전 대기 s)
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export REPO="${REPO:-$HOME/Lv2_Cartlidar_Assignment}"
export DEMO="${DEMO:-$HOME/demo_v2}"
export UP_DRY="${UP_DRY:-0}"

MODE="${1:-}"
[ $# -gt 0 ] && shift

usage() { sed -n '2,/^set -u/p' "${BASH_SOURCE[0]}" | sed '$d' | sed 's/^# \{0,1\}//'; exit "${1:-2}"; }
case "$MODE" in drive|mission|down) ;; -h|--help) usage 0 ;; *) usage 2 ;; esac
export UP_MODE="$MODE"

# shellcheck source=up_lib.sh
source "$HERE/up_lib.sh"

# ---------------------------------------------------------------- 시작 안내 (어느 저장소 · 어떤 판인지)
say "REPO = $REPO"
if [ -d "$REPO/.git" ]; then say "git 판: $(git -C "$REPO" log -1 --oneline 2>/dev/null || echo '읽을 수 없음')"
else warn "$REPO 는 git 저장소가 아님"; fi
say "DEMO = $DEMO · 모드 = $MODE$([ "$UP_DRY" = 1 ] && echo " · UP_DRY=1")"

# 터미널 · tmux 확인 (ssh -t 없이 쓰면 Ctrl+C 가 안 전달되고 read 가 바로 끝남)
if [ "$UP_DRY" != 1 ]; then
  [ -t 0 ] || { echo "✗ 터미널이 필요합니다 (ssh -t 로 접속하거나 젯슨 화면 · tmux 에서 실행)" >&2; exit 1; }
  [ -n "${TMUX:-}${STY:-}" ] || warn "tmux/screen 밖에서 실행 중 — ssh 가 끊기면 정리가 제대로 안 될 수 있음 (HUP 을 받으면 정리는 시도함)"
fi
[ "$MODE" = mission ] && say "미션 기본값: max_wz:=0.5 (회전 상한 0.5 rad/s) — 바꾸려면 인자로 max_wz:=값"
say "물리 비상정지 없음 — 리모컨 1 이 정지 수단"

# ---------------------------------------------------------------- ROS 환경
if [ "$UP_DRY" = 1 ]; then
  dry_say "source /opt/ros/lyrical/setup.bash · $REPO/workspace/ros2_ws/install/setup.bash · ONNX LD_LIBRARY_PATH"
else
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/lyrical/setup.bash || { echo "✗ /opt/ros/lyrical/setup.bash 없음" >&2; exit 1; }
  if [ -f "$REPO/workspace/ros2_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$REPO/workspace/ros2_ws/install/setup.bash"
  else echo "✗ $REPO/workspace/ros2_ws/install 없음 — 먼저 colcon build" >&2; exit 1; fi
  # ONNX Runtime (robot_console/env.sh 와 같음)
  [ -d "$HOME/onnxruntime/lib" ] && export LD_LIBRARY_PATH="$HOME/onnxruntime/lib:${LD_LIBRARY_PATH:-}"
  set -u
fi

# ---------------------------------------------------------------- down (잠금 없이 · preflight (1)(2) 만 · 목록을 보여 주고 y 를 받아야 끔)
if [ "$MODE" = down ]; then
  if [ "$UP_DRY" != 1 ] && pgrep -f 'robot_console/[c]onsole_daemon.py' >/dev/null 2>&1; then
    warn "robot_console 의 console_daemon 이 떠 있음 — 그쪽이 띄운 프로세스를 끌 수 있으니 목록을 잘 보세요"
  fi
  if [ "$UP_DRY" != 1 ]; then
    mkdir -p "$LOG_ROOT"; exec 9> "$LOG_ROOT/.up.lock"
    flock -n 9 || { echo "✗ 다른 up.sh 가 실행 중 — 그 터미널에서 Ctrl+C 로 끝내세요 (down 으로 끄면 팔이 접히지 않음)" >&2; exit 1; }
  fi
  say "남은 것 정리 (목록 확인 후 kill 입력)"
  if preflight_procs; then ok "정리 끝"; exit 0; fi
  warn "정리하지 않고 끝냄"; exit 1
fi

# robot_console (console_daemon.py) 이 떠 있으면 거부 — 같은 하드웨어 · 포트를 두 곳에서 잡게 됨
if [ "$UP_DRY" != 1 ] && pgrep -f 'robot_console/[c]onsole_daemon.py' >/dev/null 2>&1; then
  echo "✗ robot_console 의 console_daemon.py 가 떠 있음 (pgrep -af console_daemon) — 콘솔을 끝낸 뒤 다시 (또는 콘솔에서 띄우세요)" >&2
  exit 1
fi

# 시작 때 점검 (경고만): run_demo.sh 가 source 할 install ↔ REPO · arm_look 설치 여부 (drive)
UP_ARM_LOOK_MISSING=0
if [ "$MODE" = drive ]; then
  check_install_source
  if [ "${ARM_LOOK:-on}" != 0 ] && [ ! -f "$REPO/workspace/ros2_ws/install/bringup/share/bringup/launch/arm_look.launch.py" ]; then
    UP_ARM_LOOK_MISSING=1
    warn "install 에 arm_look.launch.py 가 없음 (colcon build 전?) — ④ 를 건너뜀. ARM_LOOK=0 을 권장 (run_demo 도 같은 값으로)"
  fi
fi
export UP_ARM_LOOK_MISSING

# ---------------------------------------------------------------- 중복 실행 방지 (flock)
mkdir -p "$LOG_ROOT"
if [ "$UP_DRY" != 1 ]; then
  exec 9> "$LOG_ROOT/.up.lock"
  if ! flock -n 9; then
    echo "✗ 다른 up.sh 가 이미 실행 중 ($LOG_ROOT/.up.lock) — 그쪽을 Ctrl+C 로 끝내거나 ./up.sh down" >&2
    exit 1
  fi
fi

# 모드 파일
if [ "$MODE" = drive ]; then
  # shellcheck source=up_drive.sh
  source "$HERE/up_drive.sh"
else
  # shellcheck source=up_mission.sh
  source "$HERE/up_mission.sh"
fi

# ---------------------------------------------------------------- 종료 처리: INT · TERM · EXIT → teardown (한 번만)
_IN_SIGNAL=0
_on_signal() {
  # 정리 도중 두 번째 신호는 무시 (정리가 중단되면 setsid 로 뜬 bringup · 하드웨어가 남음)
  if [ "$_IN_SIGNAL" = 1 ]; then warn "정리 중입니다 — 기다리세요 (최대 약 1 분, 강제 종료는 ./up.sh down)"; return 0; fi
  _IN_SIGNAL=1
  echo; warn "신호 받음 — 정리 중 (최대 약 1 분)"
  declare -F mission_stop >/dev/null && mission_stop       # mission 모드에서만 정의됨 (/mission/stop 뒤 멈춤 확인)
  jobs -p | xargs -r kill 2>/dev/null || true             # 상태 보여 주던 도우미 등
  trap '' INT                                             # teardown 중 Ctrl+C 로 중단되지 않게
  teardown; exit 130
}
trap _on_signal INT TERM HUP
trap 'trap "" INT; teardown; print_summary' EXIT

if [ "$UP_DRY" = 1 ]; then RUN="${RUN:-$LOG_ROOT/DRY_$MODE}"; else run_dir_init "$MODE"; fi
# 11-23: 이 화면 출력도 $RUN/up.log 에 남김 (첫 실물 3 회가 어느 단계에서 멈췄는지 화면에만 있어 못 찾음).
#   tee -i = Ctrl+C 를 무시 (같은 프로세스 그룹이라 tee 가 먼저 죽으면 정리 출력이 끊김) · PIPE 무시 = tee 가 HUP 로 죽어도 정리는 계속
if [ "$UP_DRY" != 1 ]; then
  trap '' PIPE
  exec > >(tee -i -a "$RUN/up.log") 2>&1
fi

# ---------------------------------------------------------------- 사전 점검 (한 번만 — 여기서 부르고 UP_PREFLIGHT_DONE=1. 모드 파일은 안 부름)
#  drive 에 인자가 없으면 사용법만 보여 주고 끝나므로 점검도 안 함
if ! { [ "$MODE" = drive ] && [ "$#" -eq 0 ]; }; then preflight; fi

# ---------------------------------------------------------------- 실행
if [ "$MODE" = drive ]; then
  drive_up "$@"
else
  mission_up "$@"
  mission_start
fi
