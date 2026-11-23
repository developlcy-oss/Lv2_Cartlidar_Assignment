#!/usr/bin/env bash
# up_drive.sh — 주행 모드 (./up.sh drive [run_demo 인자...])  ※ up.sh 가 source 함 (직접 실행 아님)
# 하는 일 = drive_demo README 단계 4~9 (실물 주행 절차) 를 순서대로 자동화:
#   preflight → ① base_bringup → ② scan_rotate → ③ bringup mode:=drive → ④ arm_look → ⑤ check_jetson → ⑥ E-stop 풀기 → ⑦ run_demo real "$@"
# 쓰는 것: up_lib.sh 의 say ok warn die ask_enter start_bg wait_node wait_topic wait_service wait_active wait_estop_clear preflight
#          변수 RUN REPO DEMO PORTS UP_DRY (up.sh · up_lib.sh 가 정함)
# 환경 변수: ARM_LOOK (기본 on, run_demo 와 같음 · 0 이면 ④ 건너뜀) · UP_ARM_CAMERA=0 (Arducam 이 이미 떠 있을 때 use_arm_camera:=false)
#            UP_SKIP_ARM_WARN=1 (③ 앞 "손 빼기" 확인 생략 — 권장 안 함)
# 저장소 파일은 안 고침 (run_demo.sh 포함).

# 시험 (UP_DRY=1) 에서는 내 쪽 직접 명령 (ros2 control · grep · check_jetson · run_demo) 도 실행하지 않고 찍기만 함
_dry() { [ "${UP_DRY:-0}" = 1 ]; }

# run_demo.sh 사용법 (맨 위 주석) — 인자가 없을 때
_drive_usage() {
  say "사용법: ./up.sh drive <run_demo 인자>   (끝에 run_demo.sh real 이 붙음)"
  sed -n 2,5p "$DEMO/run_demo.sh" 2>/dev/null | sed 's/^#/ /'
  say "  (자세한 인자 · 환경변수는 $DEMO/run_demo.sh 맨 위 주석 · drive_demo README)"
  say "환경변수: ARM_LOOK=0 · UP_ARM_CAMERA=0 · UP_SKIP_ARM_WARN=1 · UP_ALLOW_MISMATCH=1 · UP_KILL_LEFTOVER=1 · UP_LC_STUCK=25 (자세한 건 up.sh 맨 위)"
  say "예상 소요: 켜기 약 2~4 분 (① ~ ⑥) — 기다리는 동안 Ctrl+C 를 누르지 마세요. 젯슨 화면이나 tmux 에서 실행 (ssh 가 끊기면 정리가 제대로 안 될 수 있음)"
  say "예) ./up.sh drive slow auto init   (자율 한 바퀴, 출발 자세 /initialpose)"
  say "    ./up.sh drive slow step init 0-0   (첫 구간만, 구간마다 Enter)"
}

# loc_monitor (C++ 판 lib/perception · 예전 lib/localization) 를 SIGINT — 우리가 띄운 그룹 안의 것만 (남의 loc_monitor 는 안 건드림)
_stop_loc_monitor_ours() {
  local p n=0
  if _dry; then say "[dry] loc_monitor 가 우리 그룹 안에 있으면 SIGINT (README 단계 7)"; return 0; fi
  for p in $(pgrep -f 'lib/(localization|perception)/loc_monitor' 2>/dev/null); do
    _is_ours "$p" && { kill -INT "$p" 2>/dev/null; n=$((n+1)); }
  done
  [ "$n" -gt 0 ] && warn "loc_monitor $n 개가 떠 있어 SIGINT (use_loc_monitor:=false 가 안 먹었음)" || ok "loc_monitor 없음 (use_loc_monitor:=false 적용)"
  return 0
}

# AMCL 위치 불확실도 확인 — /amcl_pose 를 받고 covariance[0] (x 분산) 이 UP_AMCL_COV (기본 0.05 m²) 이하여야 함. 못 받으면 die
#  --csv 출력 = stamp.sec,nanosec,frame_id, x,y,z, qx,qy,qz,qw, covariance(36) → 11 번째 필드 = covariance[0]. 읽지 못하면 warn (형식이 다를 때)
UP_AMCL_WARNED=""
_check_amcl_sigma() {
  local out cov
  if _dry; then say "[dry] /amcl_pose 를 한 번 받아 covariance[0] ≤ ${UP_AMCL_COV:-0.05} 확인"; return 0; fi
  out=$(timeout 10 ros2 topic echo --once --csv --qos-reliability reliable --qos-durability transient_local /amcl_pose 2>/dev/null | head -1)
  # 기본은 경고 · 기록만 (run_demo init 가 /initialpose 를 주기 전일 수 있어서). UP_AMCL_STRICT=1 이면 die
  # ⑦ 앞에서 UP_AMCL_WARNED 를 다시 보여 줌
  _amcl_bad() { if [ "${UP_AMCL_STRICT:-0}" = 1 ]; then die "$1"; else warn "$1"; note_event "$1"; UP_AMCL_WARNED+="$1; "; fi; }
  [ -n "$out" ] || { _amcl_bad "/amcl_pose 를 못 받음 — AMCL 이 위치를 못 냄 ($RUN/loc.log)"; return 0; }
  cov=$(printf '%s' "$out" | awk -F, '{print $11}')
  if ! printf '%s' "$cov" | grep -qE '^[0-9.eE+-]+$'; then warn "/amcl_pose covariance 를 읽지 못함 (출력 형식 확인): ${out:0:80}"; return 0; fi
  if awk -v c="$cov" -v m="${UP_AMCL_COV:-0.05}" 'BEGIN{exit !(c+0>m+0)}'; then
    _amcl_bad "AMCL 불확실도가 큼 (covariance[0]=$cov > ${UP_AMCL_COV:-0.05}) — 출발 자리 · /initialpose 확인"; return 0
  fi
  ok "AMCL covariance[0]=$cov"
}

# 단계 ① 뒤: controller_manager · diff_drive_controller 가 active 가 될 때까지 (팔 대기 자세 거부 문구가 나오면 바로 안내)
# 11-23: /odometry/filtered 이 안 나올 때 EKF 입력 · 출력 Hz 와 ekf 노드 유무를 찍음 (첫 실물 3 회 원인 찾기용, 읽기만)
_diag_ekf() {
  local t r
  warn "EKF 진단 (각 6 s 측정):"
  for t in /odometry/filtered /odom_icr /odom /imu/data /joint_states; do
    r=$(_hz_once "$t"); printf '    %-20s %s\n' "$t" "${r:+$r Hz}${r:-메시지 없음}"
  done | tee -a "$RUN/diag_ekf.log"
  _ros2 node list 2>/dev/null | grep -E "ekf|odom_icr|imu" | sed 's/^/    노드 /' | tee -a "$RUN/diag_ekf.log"
}

_wait_controllers() {
  local timeout=${1:-60} end out
  if _dry; then say "[dry] ros2 control list_controllers → diff_drive_controller active 확인 (${timeout}s)"; return 0; fi
  end=$(_deadline "$timeout")   # 실제 경과 시간 (SECONDS) 기준 — ros2 호출이 느려도 한도를 지킴
  while [ "$SECONDS" -lt "$end" ]; do
    # 팔 대기 자세가 아니면 ros2_control 이 거부하고 launch 가 꺼짐 (README 단계 4)
    if grep -q '대기 자세가 아님' "$RUN/base.log" 2>/dev/null; then
      grep '대기 자세가 아님' "$RUN/base.log" | head -3 >&2
      die "팔이 대기 자세가 아님 — ros2_control 이 거부함. 두 OpenCR 의 팔 보드를 손으로 대기 자세 (가운데 J1 3482 · J5 2048 쪽) 로 맞춘 뒤 다시: opencr_arm_tool.py monitor --port <팔 보드 by-path> (README 단계 3)"
    fi
    if grep -q 'OpenCR 부팅 점검 실패' "$RUN/base.log" 2>/dev/null; then
      die "OpenCR 부팅 점검 실패 (보드 FAULT) — 12 V 켠 채 두 보드 RESET 한 뒤 ./up.sh drive 다시"
    fi
    out=$(timeout 8 ros2 control list_controllers 2>/dev/null)
    if printf '%s\n' "$out" | grep -E 'diff_drive_controller.*active' >/dev/null; then
      ok "diff_drive_controller active"
      printf '%s\n' "$out" | grep -qE 'arm_controller.*active' && ok "arm_controller active" || warn "arm_controller 가 active 로 안 보임 (주행만이면 무관, 팔 BOX_VIEW 는 필요)"
      return 0
    fi
    sleep 2
  done
  die "${timeout} s 안에 diff_drive_controller 가 active 가 안 됨 — $RUN/base.log 확인"
}

drive_up() {
  # 인자가 없으면 run_demo 사용법만 보여 주고 끝 (아무것도 띄우지 않음)
  if [ "$#" -eq 0 ]; then _drive_usage; return 0; fi

  say "주행 모드: ./run_demo.sh real $*"
  step_mark "① base"
  # install 출처 점검 (check_install_source) 과 preflight 는 up.sh 가 이미 함 (preflight 는 한 번만)

  # ── ① 바퀴 · 팔 하드웨어 · 리모컨 (README 단계 4) ──
  say "① base_bringup (rc100 · obstacle_lock:=false · loc_lock:=false) — ⚠ 이때부터 팔 토크가 켜져 자세를 붙잡음"
  # ports 인자는 base_bringup.launch.py 가 받음 (기본 auto) — preflight 가 PORTS 를 정했으면 그대로 넘김
  local base_args=(rc100:=true obstacle_lock:=false loc_lock:=false)
  [ -n "${PORTS:-}" ] && base_args+=("ports:=$PORTS")
  # rc100:=true 는 use_teleop (기본 true, base_bringup.launch.py 190 행) 가 필요 — 기본값 그대로 두면 됨 (false 로 주면 RuntimeError)
  check_launch_args opencr_hardware base_bringup.launch.py "${base_args[@]}" use_teleop
  start_bg base ros2 launch opencr_hardware base_bringup.launch.py "${base_args[@]}"
  _wait_controllers 60
  wait_node /controller_manager 20 || die "/controller_manager 안 보임 — $RUN/base.log 확인"
  wait_node /twist_mux 20 || die "/twist_mux 안 보임 — $RUN/base.log 확인"

  # ── ② 라이다 두 대 (README 단계 5) ──
  step_mark "② scan"
  say "② scan_rotate.py (라이다 두 대 → /scan_front · /scan_rear)"
  start_bg scan python3 "$DEMO/scan_rotate.py"
  wait_topic /scan_front 30 5 || die "/scan_front 이 5 Hz 안 나옴 — $RUN/scan.log (라이다 USB · scan_rotate) 확인"
  wait_topic /scan_rear 30 5 || die "/scan_rear 이 5 Hz 안 나옴 — $RUN/scan.log (라이다 USB · scan_rotate) 확인"

  # ── ③ 위치 추정 (README 단계 6) — 팔이 대기 → 주행 자세로 펴짐 ──
  step_mark "③ 위치 추정"
  say "③ bringup mode:=drive — ⚠ 켜질 때 팔이 대기 → 주행 자세로 펴집니다 (J3→J2→J4→J1→J5)"
  if [ "${UP_SKIP_ARM_WARN:-0}" != 1 ]; then
    ask_enter "[팔 안전] 팔 주변에서 손을 빼고, 리모컨 1 (정지) 을 준비하세요. Enter = 띄우기, q = 끝"
  fi
  # use_loc_monitor 는 bringup.launch.py 가 선언하지 않지만 명령줄 인자는 include 한 localization.launch.py (54 행 선언) 에 전역으로
  # 전달돼 먹음 (README 단계 6 과 같음) — 그래서 bringup 이 아니라 localization 쪽 선언으로 확인. loc_monitor 노드는 respawn 이 아님.
  local loc_args=(mode:=drive use_hardware:=false use_sensors:=false use_perception:=false use_tfmini:=false)
  check_launch_args bringup bringup.launch.py "${loc_args[@]}"
  check_launch_args localization localization.launch.py use_loc_monitor
  UP_ARM_EXTENDED=1   # 이 launch 가 팔을 폄 — 이후 teardown 은 끄기 전에 팔을 접어 봄 (fold_arm)
  start_bg loc ros2 launch bringup bringup.launch.py "${loc_args[@]}" use_loc_monitor:=false
  wait_active /map_server 90 || die "/map_server 가 active 안 됨 — $RUN/loc.log (ros2 lifecycle set /map_server activate)"
  wait_active /amcl 90 || die "/amcl 이 active 안 됨 — $RUN/loc.log. ⚠ inactive 면 라이다가 반대로 도는 것처럼 보임 (11-22 함정)"
  wait_topic /odometry/filtered 60 5 || { _diag_ekf; die "/odometry/filtered 이 5 Hz 안 나옴 — 위 EKF 입력 Hz · $RUN/loc.log (ekf) 확인"; }
  _check_amcl_sigma
  _dry && say "[dry] 지도 · AMCL · EKF 준비 판정은 실제 실행에서만" || ok "지도 · AMCL · EKF 준비"
  # 안전망 (README 단계 7): use_loc_monitor:=false 가 안 먹어 loc_monitor 가 떠 있으면 끔 — 우리가 띄운 그룹 (loc) 안의 것만
  _stop_loc_monitor_ours

  # ── ④ 팔 카메라 상자 재기 (ARM_LOOK 기본 on, run_demo 와 같은 변수) ──
  step_mark "④ 팔 카메라"
  case "${ARM_LOOK:-on}" in
    0)
      say "④ 건너뜀 (ARM_LOOK=0 — 라이다만)" ;;
    1|on|dry)
      if [ "${UP_ARM_LOOK_MISSING:-0}" = 1 ]; then
        warn "④ 건너뜀 — install 에 arm_look.launch.py 없음 (ARM_LOOK=0 권장). 상자 재기 서비스가 없어 run_demo 가 라이다만으로 갈 수 있음"
        note_event "④ 팔 카메라 건너뜀 (arm_look install 없음)"
      else
      say "④ arm_look.launch.py (팔 Arducam + 상자 윗면 측정 서비스, ARM_LOOK=${ARM_LOOK:-on})"
      local arm_args=()
      [ "${UP_ARM_CAMERA:-1}" = 0 ] && arm_args+=(use_arm_camera:=false)
      start_bg arm_look ros2 launch bringup arm_look.launch.py "${arm_args[@]}"
      wait_service /perception/place_box/measure 60 || { warn "팔 카메라 서비스 없음 — run_demo 가 라이다만으로 감 ($RUN/arm_look.log)"; note_event "④ /perception/place_box/measure 서비스 없음 (라이다만)"; }
      fi ;;
    *)
      die "ARM_LOOK 은 0 · dry · on 중 하나 (지금 ${ARM_LOOK})" ;;
  esac

  # ── ⑤ 점검 (README 단계 7) — 마지막 줄 '점검 통과' ──
  step_mark "⑤ 점검"
  say "⑤ check_jetson.sh (노드 · 토픽 · 라이다 방향 · 위치 · 배터리)"
  if _dry; then
    say "[dry] (cd $DEMO && ./check_jetson.sh) > $RUN/check_jetson.log  → 마지막 줄 '점검 통과' 확인"
  else
    (cd "$DEMO" && timeout 60 ./check_jetson.sh) >"$RUN/check_jetson.log" 2>&1
    if tail -n 1 "$RUN/check_jetson.log" | grep -q '점검 통과'; then
      ok "점검 통과"
    else
      warn "점검 실패 항목 (✗):"
      grep '✗' "$RUN/check_jetson.log" >&2 || tail -n 15 "$RUN/check_jetson.log" >&2
      note_event "check_jetson 실패 항목이 있는데 계속 진행 ($RUN/check_jetson.log)"
      ask_yes "[점검 실패] 실패 항목이 있어도 계속합니까? (전체 로그 $RUN/check_jetson.log)"
    fi
  fi

  # ── ⑥ E-stop 풀기 (리모컨 2+3) ──
  step_mark "⑥ E-stop"
  say "⑥ E-stop 풀기"
  wait_estop_clear 120 || die "E-stop 이 안 풀림 — 리모컨 2+3 확인 (rc100_joy 가 떴는지: ros2 node list | grep rc100)"

  # ── ⑦ 시연 실행 (README 단계 9) — 포그라운드 ──
  step_mark "⑦ 시연"
  say "⑦ 체크: 리모컨 전원 ON · 1 로 멈추고 2+3 으로 풀리는지 한 번 확인 · ARM_LOOK=${ARM_LOOK:-on} (BOX_VIEW 로 주행 내내 팔이 펴짐, 팔↔벽 최소 약 4 cm)"
  case " $* " in *" step "*) ;; *" auto "*|*" max "*) warn "자율 주행 (step 없음) — 첫 실물이면 step 으로 S4 · S2 에서 팔 ↔ 벽을 눈으로 확인하세요" ;; esac
  [ -n "$UP_AMCL_WARNED" ] && warn "⑦ 앞 재확인 — AMCL 경고가 있었음: $UP_AMCL_WARNED"
  [ "${UP_ESTOP_UNTESTED:-0}" = 1 ] && warn "⑦ 앞 재확인 — E-stop 이 걸린 상태를 못 봐서 시험 안 됨: 리모컨 1 로 멈추는지 직접 확인"
  say "⑦ 출발 자리: (0.694, 0.950) · 방향 180° 에 로봇을 놓았는지 확인 (init 인자가 있으면 이 자세를 /initialpose 로 줌)"
  ask_enter "[출발 확인] 로봇이 출발 자리에 있고 리모컨 1 (정지) 준비됐으면 Enter = 시연 시작, q = 끝"
  local rc=0
  if _dry; then
    say "[dry] (cd $DEMO && ./run_demo.sh real $*)"
  else
    (cd "$DEMO" && ./run_demo.sh real "$@") || rc=$?
  fi
  _dry && return 0
  [ "$rc" -eq 0 ] && ok "run_demo 끝 (정상)" || warn "run_demo 종료 코드 $rc"
  return "$rc"   # teardown 은 up.sh 의 trap 이 함
}
