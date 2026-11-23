#!/usr/bin/env bash
# test_up_lib.sh — up_lib.sh 대기 함수 로직 시험 (가짜 ros2 셸 함수 사용, UP_DRY 없이, ROS 불필요)
#   bash test_up_lib.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export UP_KILL_LEFTOVER=0 UP_DRY=0 UP_PREFLIGHT_DONE=0 UP_LC_STUCK=0 UP_POLL=0.1 UP_REARM_NOTE=1 LOG_ROOT="$(mktemp -d)" HOME="${HOME:-/tmp}"
# shellcheck source=up_lib.sh
source "$HERE/up_lib.sh"
mkdir -p "$LOG_ROOT/r"; RUN="$LOG_ROOT/r"

PASS=0; FAIL=0
sub() { ( "$@" ); }   # die (exit 1) 를 내는 함수를 서브셸에서 시험
t() {   # t 이름 기대코드 명령...
  local name=$1 want=$2; shift 2
  "$@" >/dev/null 2>&1; local rc=$?
  if [ "$rc" = "$want" ]; then PASS=$((PASS+1)); echo "  PASS $name"; else FAIL=$((FAIL+1)); echo "  FAIL $name (rc=$rc, 기대 $want)"; fi
}

# 가짜 ros2: 상태는 파일로 (서브셸 $(...) 에서 호출되므로 변수 대신 파일)
S="$LOG_ROOT/state"; mkdir -p "$S"
CALLS="$S/calls"; : > "$CALLS"
ros2() {
  echo "$*" >> "$CALLS"
  case "$1 $2" in
    "node list")
      # 데몬 캐시 흉내: --no-daemon 일 때만 /late 가 보임
      echo /a; case "$*" in *--no-daemon*) echo /late ;; esac ;;
    "service list") cat "$S/services" 2>/dev/null ;;
    "topic list") cat "$S/topics" 2>/dev/null ;;
    "topic hz") echo "average rate: $(cat "$S/hz" 2>/dev/null || echo 0)" ;;
    "lifecycle get")
      _nd=$3; [ "$_nd" = --no-daemon ] && _nd=$4   # 11-23: wait_active 가 --no-daemon 도 번갈아 씀
      n=$(cat "$S/lc_$(basename "$_nd")" 2>/dev/null || echo unconfigured); echo "$n [1]" ;;
    "lifecycle set")
      # activate 는 두 번째 시도부터 먹힘
      c=$(cat "$S/act_$(basename "$3")" 2>/dev/null || echo 0)
      case "$4" in
        configure) echo inactive > "$S/lc_$(basename "$3")" ;;
        activate) c=$((c+1)); echo $c > "$S/act_$(basename "$3")"; [ $c -ge 2 ] && echo active > "$S/lc_$(basename "$3")" ;;
      esac ;;
    "topic echo") cat "$S/estop" 2>/dev/null ;;
    "launch bringup") cat "$S/show_args" 2>/dev/null ;;
    "action list") cat "$S/actions" 2>/dev/null ;;
    "action send_goal") echo "$*" >> "$S/goals"; echo "Goal finished with status: SUCCEEDED  success: true" ;;
  esac
}

echo "== wait_node"
t "보이는 노드 (데몬)"          0 wait_node /a 2
t "데몬 캐시에 없고 --no-daemon 에만 보임" 0 wait_node /late 2
t "없는 노드 = 실패"            1 wait_node /nope 1

echo "== wait_service"
echo "/perception/place_box/measure" > "$S/services"
t "서비스 있음"                 0 wait_service /perception/place_box/measure 2
t "서비스 없음 = 실패"          1 wait_service /x 1

echo "== wait_topic"
printf '/scan_front\n/e_stop\n' > "$S/topics"
t "토픽만 확인"                 0 wait_topic /scan_front 2
t "없는 토픽"                   1 wait_topic /scan_rear 1
echo 9.8 > "$S/hz"
t "Hz 충분 (9.8 ≥ 5)"           0 wait_topic /scan_front 2 5
t "Hz 부족 (9.8 < 20)"          1 wait_topic /scan_front 0.5 20

echo "== wait_active"
echo inactive > "$S/lc_amcl"; echo 0 > "$S/act_amcl"
t "inactive → 재시도 끝에 active" 0 wait_active amcl 5
grep -c "lifecycle set /amcl activate" "$CALLS" | grep -qx 2 && { PASS=$((PASS+1)); echo "  PASS activate 정확히 2 번 시도"; } || { FAIL=$((FAIL+1)); echo "  FAIL activate 시도 횟수"; }
echo unconfigured > "$S/lc_map_server"; echo 0 > "$S/act_map_server"
t "unconfigured → configure → active" 0 wait_active /map_server 6
echo inactive > "$S/lc_stuck"; echo -99 > "$S/act_stuck"
t "끝내 active 안 됨 = 실패"    1 wait_active stuck 2

echo "== wait_estop_clear"
printf 'data: true\n---\n' > "$S/estop"
t "true 로 남음 = 실패 (안내 출력)" 1 sub wait_estop_clear 2
printf 'data: false\n---\n' > "$S/estop"
t "false = 성공"                0 wait_estop_clear 2
wait_estop_clear 2 2>&1 | grep -q "이미 false" && { PASS=$((PASS+1)); echo "  PASS 시작부터 false 면 그 사실을 찍음"; } || { FAIL=$((FAIL+1)); echo "  FAIL 이미 false 안내"; }
rm -f "$S/estop"
t "토픽 없음 (메시지 0 개) = die" 1 sub wait_estop_clear 1
sub wait_estop_clear 1 2>&1 | grep -q "토픽 없음 — rc100_joy · opencr 확인" && { PASS=$((PASS+1)); echo "  PASS 토픽 없음 문구"; } || { FAIL=$((FAIL+1)); echo "  FAIL 토픽 없음 문구"; }
# e_stop 구독 QoS (reliable + transient_local) 로 호출하는지
grep -q "transient_local /e_stop" "$CALLS" && grep -q "reliability reliable" "$CALLS" \
  && { PASS=$((PASS+1)); echo "  PASS /e_stop QoS reliable + transient_local"; } || { FAIL=$((FAIL+1)); echo "  FAIL /e_stop QoS"; }

echo "== check_launch_args"
printf "Arguments (pass arguments as '<name>:=<value>'):\n\n    'mode':\n        x\n    'max_wz':\n        y\n" > "$S/show_args"
t "선언된 인자만 = 통과"        0 check_launch_args bringup bringup.launch.py mode:=drive max_wz
cla=$(check_launch_args bringup bringup.launch.py mode:=drive rotate_wz:=0.5 use_loc_monitor:=false 2>&1); rc=$?
[ "$rc" = 1 ] && printf '%s' "$cla" | grep -q "rotate_wz use_loc_monitor" && { PASS=$((PASS+1)); echo "  PASS 없는 인자 warn (rotate_wz · use_loc_monitor)"; } || { FAIL=$((FAIL+1)); echo "  FAIL check_launch_args warn: $cla"; }
rm -f "$S/show_args"
t "--show-args 가 비면 확인 못 함 (실패 아님)" 0 check_launch_args bringup bringup.launch.py mode

echo "== start_bg · teardown"
start_bg sleeper sleep 300
sleep 1
pid=$(awk 'END{print $1}' "$RUN/pids")
kill -0 "$pid" 2>/dev/null && { PASS=$((PASS+1)); echo "  PASS start_bg 실행 · pids 기록"; } || { FAIL=$((FAIL+1)); echo "  FAIL start_bg"; }
[ "$(ps -o pgid= -p "$pid" | tr -d ' ')" = "$pid" ] && { PASS=$((PASS+1)); echo "  PASS 새 프로세스 그룹 (PGID=PID)"; } || { FAIL=$((FAIL+1)); echo "  FAIL PGID"; }
teardown >/dev/null 2>&1
kill -0 "$pid" 2>/dev/null && { FAIL=$((FAIL+1)); echo "  FAIL teardown 이 안 끔"; } || { PASS=$((PASS+1)); echo "  PASS teardown 이 그룹을 끔"; }

echo "== teardown 은 남의 PID 를 안 끔"
# 남의 프로세스: 우리 pids 에 없는 ROS 비슷한 프로세스 (argv0 = "ros2 run fake" 로 패턴에 걸리게)
setsid bash -c 'exec -a "ros2 run other_session_fake" sleep 300' >/dev/null 2>&1 &
other=$!
# 우리 자손이지만 그룹 밖으로 나간 것: 우리 그룹의 bash 가 새 세션으로 sleep 을 띄움
_TD_DONE=0
start_bg mine bash -c 'setsid sleep 301 & wait'
sleep 1
mine_pid=$(pgrep -f '^sleep 301' | head -1)
kill -0 "$other" 2>/dev/null && [ -n "$mine_pid" ] && { PASS=$((PASS+1)); echo "  PASS 준비 (남의 $other · 우리 자손 $mine_pid)"; } || { FAIL=$((FAIL+1)); echo "  FAIL 준비 other=$other mine=$mine_pid"; }
_ros_pids | grep -qx "$other" && { PASS=$((PASS+1)); echo "  PASS 남의 프로세스가 _ros_pids 에는 보임"; } || { FAIL=$((FAIL+1)); echo "  FAIL _ros_pids 에 안 보임"; }
teardown > "$S/td1.out" 2>&1
kill -0 "$other" 2>/dev/null && { PASS=$((PASS+1)); echo "  PASS UP_KILL_LEFTOVER=0: 남의 PID 안 끔"; } || { FAIL=$((FAIL+1)); echo "  FAIL 남의 PID 가 꺼짐"; }
kill -0 "$mine_pid" 2>/dev/null && { PASS=$((PASS+1)); echo "  PASS UP_KILL_LEFTOVER=0: 우리 자손도 목록만"; } || { FAIL=$((FAIL+1)); echo "  FAIL 0 인데 우리 자손이 꺼짐"; }
grep -q "남의 ROS 프로세스" "$S/td1.out" && grep -q "$other" "$S/td1.out" && { PASS=$((PASS+1)); echo "  PASS 남의 것은 목록으로 보여 줌"; } || { FAIL=$((FAIL+1)); echo "  FAIL 남의 목록 없음"; }
# 두 번째: UP_KILL_LEFTOVER=1 — 우리 자손만 끔, 남의 것은 그대로
_TD_DONE=0; UP_KILL_LEFTOVER=1
# pids 파일에 죽은 그룹이 있어도 _ours 는 자손 스냅샷을 구함 → 새로 띄워 확인
: > "$RUN/pids"
start_bg mine2 bash -c 'setsid sleep 302 & wait'
sleep 1
mine2=$(pgrep -f '^sleep 302' | head -1)
teardown > "$S/td2.out" 2>&1
UP_KILL_LEFTOVER=0
kill -0 "$mine2" 2>/dev/null && { FAIL=$((FAIL+1)); echo "  FAIL UP_KILL_LEFTOVER=1 인데 우리 자손이 남음"; } || { PASS=$((PASS+1)); echo "  PASS UP_KILL_LEFTOVER=1: 우리 자손 끔"; }
kill -0 "$other" 2>/dev/null && { PASS=$((PASS+1)); echo "  PASS UP_KILL_LEFTOVER=1 에서도 남의 PID 안 끔"; } || { FAIL=$((FAIL+1)); echo "  FAIL =1 에서 남의 PID 가 꺼짐"; }
kill "$other" 2>/dev/null; pkill -f '^sleep 301' 2>/dev/null

echo "== teardown 전 팔 접기 (하드웨어 launch 가 있었을 때만)"
_TD_DONE=0; : > "$RUN/pids"; : > "$S/goals"; echo /manipulator/pose > "$S/actions"
start_bg base sleep 300
UP_ARM_EXTENDED=1 teardown >/dev/null 2>&1
grep -q "manipulator/pose.*DRIVE" "$S/goals" && { PASS=$((PASS+1)); echo "  PASS 팔을 편 적이 있으면 DRIVE 를 보냄"; } || { FAIL=$((FAIL+1)); echo "  FAIL DRIVE 안 보냄"; }
_TD_DONE=0; : > "$RUN/pids"; : > "$S/goals"
start_bg base sleep 300      # base 는 떠 있어도 팔을 편 적 없음 (UP_ARM_EXTENDED 없음)
UP_ARM_EXTENDED=0 teardown >/dev/null 2>&1
[ -s "$S/goals" ] && { FAIL=$((FAIL+1)); echo "  FAIL 편 적 없는데 팔 접기"; } || { PASS=$((PASS+1)); echo "  PASS 팔을 편 적 없으면 접지 않음"; }
_TD_DONE=0; : > "$RUN/pids"; : > "$S/goals"; rm -f "$S/actions"
start_bg base sleep 300
UP_ARM_EXTENDED=1 teardown >/dev/null 2>&1
[ -s "$S/goals" ] && { FAIL=$((FAIL+1)); echo "  FAIL 서버 없는데 보냄"; } || { PASS=$((PASS+1)); echo "  PASS 액션 서버 없으면 건너뜀"; }

echo "== preflight_procs: y 없이는 안 끔 (입력 없음 = 안 끔)"
setsid bash -c 'exec -a "ros2 run other_session_fake2" sleep 300' >/dev/null 2>&1 &
o2=$!; sleep 0.5
preflight_procs </dev/null >/dev/null 2>&1; rc=$?
[ "$rc" = 1 ] && kill -0 "$o2" 2>/dev/null && { PASS=$((PASS+1)); echo "  PASS 입력 없으면 안 끄고 실패 반환"; } || { FAIL=$((FAIL+1)); echo "  FAIL rc=$rc"; }
echo n | preflight_procs >/dev/null 2>&1; kill -0 "$o2" 2>/dev/null && { PASS=$((PASS+1)); echo "  PASS n 이면 안 끔"; } || { FAIL=$((FAIL+1)); echo "  FAIL n 인데 꺼짐"; }
echo kill | preflight_procs >/dev/null 2>&1; sleep 0.5; kill -0 "$o2" 2>/dev/null && { FAIL=$((FAIL+1)); echo "  FAIL kill 인데 안 꺼짐"; } || { PASS=$((PASS+1)); echo "  PASS kill 이면 끔"; }

echo "== kill 을 정확히 입력해야 끔 (y · Enter 는 안 끔)"
setsid bash -c 'exec -a "ros2 run other_session_fake3" sleep 300' >/dev/null 2>&1 &
o3=$!; sleep 0.5
echo y | preflight_procs >/dev/null 2>&1; kill -0 "$o3" 2>/dev/null && { PASS=$((PASS+1)); echo "  PASS y 로는 안 끔"; } || { FAIL=$((FAIL+1)); echo "  FAIL y 로 꺼짐"; }
echo | preflight_procs >/dev/null 2>&1; kill -0 "$o3" 2>/dev/null && { PASS=$((PASS+1)); echo "  PASS Enter 로는 안 끔"; } || { FAIL=$((FAIL+1)); echo "  FAIL Enter 로 꺼짐"; }
preflight_procs </dev/null 2>&1 | grep -q "사용자" && { PASS=$((PASS+1)); echo "  PASS 목록에 사용자 · tty · 경과 열"; } || { FAIL=$((FAIL+1)); echo "  FAIL 목록 열"; }
kill "$o3" 2>/dev/null

echo "== ROS_PAT: 편집기 · tail 은 목록에서 뺌"
bash -c 'exec -a "vim run_demo.sh" sleep 300' >/dev/null 2>&1 &
ed=$!; sleep 0.3
_ros_pids | grep -qx "$ed" && { FAIL=$((FAIL+1)); echo "  FAIL vim 이 목록에 섞임"; } || { PASS=$((PASS+1)); echo "  PASS vim run_demo.sh 는 목록에서 뺌"; }
kill "$ed" 2>/dev/null

echo "== wait_active: 같은 상태로 UP_LC_STUCK 초 머문 뒤에만 개입"
echo inactive > "$S/lc_slow"; echo 0 > "$S/act_slow"; : > "$CALLS"
UP_LC_STUCK=4 wait_active slow 2 >/dev/null 2>&1
grep -q "lifecycle set /slow" "$CALLS" && { FAIL=$((FAIL+1)); echo "  FAIL 25 s 규칙: 너무 일찍 개입"; } || { PASS=$((PASS+1)); echo "  PASS 머문 시간 < 한도면 개입 안 함"; }
echo inactive > "$S/lc_slow2"; echo 0 > "$S/act_slow2"
UP_LC_STUCK=1 wait_active slow2 6 >/dev/null 2>&1; rc=$?
[ "$rc" = 0 ] && grep -q "lifecycle set /slow2 activate" "$CALLS" && { PASS=$((PASS+1)); echo "  PASS 한도를 넘기면 activate 로 개입"; } || { FAIL=$((FAIL+1)); echo "  FAIL 한도 넘겨도 개입 안 함 rc=$rc"; }

echo "== ros2 CLI 가 멈추면 timeout 으로 끊김"
mkdir -p "$LOG_ROOT/bin"; printf '#!/bin/sh\nsleep 20\n' > "$LOG_ROOT/bin/ros2"; chmod +x "$LOG_ROOT/bin/ros2"
(
  unset -f ros2; PATH="$LOG_ROOT/bin:$PATH"; UP_ROS2_TMO=1
  t0=$SECONDS; wait_node /x 3 >/dev/null 2>&1; rc=$?; el=$((SECONDS - t0))
  [ "$rc" = 1 ] && [ "$el" -le 7 ]
) && { PASS=$((PASS+1)); echo "  PASS 멈춘 ros2 에도 마감 (SECONDS 기준)에 끝남"; } || { FAIL=$((FAIL+1)); echo "  FAIL 멈춘 ros2 에서 안 끝남"; }

echo "== ask_yes: yes 만 통과"
t "yes 입력 = 계속"             0 bash -c 'source "'"$HERE"'/up_lib.sh"; echo yes | ask_yes x'
out=$(bash -c 'source "'"$HERE"'/up_lib.sh"; ask_yes x <<< ""; echo CONTINUED' 2>&1 </dev/null); printf '%s' "$out" | grep -q CONTINUED && { FAIL=$((FAIL+1)); echo "  FAIL Enter 로 계속됨"; } || { PASS=$((PASS+1)); echo "  PASS Enter 는 계속이 아님 (끝)"; }

echo "== wait_* 반환값 무시 금지 (정적 검사)"
bad=$(grep -nE '^[[:space:]]*wait_(node|topic|service|active|estop_clear)[[:space:]]' "$HERE/up_drive.sh" "$HERE/up_mission.sh" | grep -v '||' || true)
[ -z "$bad" ] && { PASS=$((PASS+1)); echo "  PASS 모든 wait_* 호출에 || 가 있음"; } || { FAIL=$((FAIL+1)); echo "  FAIL || 없는 wait_*:"; echo "$bad"; }
# 모든 ros2 직접 호출에 timeout 이 있는지 (mission 파일: ros2 service call 등)
bad2=$(grep -nE '(^|[ (`$])ros2 (service call|lifecycle|node list|control)' "$HERE/up_mission.sh" "$HERE/up_drive.sh" | grep -vE 'timeout|_ros2|#|DRY|say|warn|echo|wait_|die' || true)
[ -z "$bad2" ] && { PASS=$((PASS+1)); echo "  PASS ros2 직접 호출에 timeout 이 있음"; } || { FAIL=$((FAIL+1)); echo "  FAIL timeout 없는 ros2:"; echo "$bad2"; }

echo "== install 불일치 = die (UP_ALLOW_MISMATCH=1 이면 경고만)"
mk="$LOG_ROOT/inst"; mkdir -p "$mk/a/workspace/ros2_ws/install" "$mk/a/workspace/tools/drive_demo" "$mk/b" "$mk/h"; : > "$mk/a/workspace/ros2_ws/install/setup.bash"
t "다른 REPO = die"             1 sub env HOME="$mk/h" DEMO="$mk/a/workspace/tools/drive_demo" REPO="$mk/b" bash -c 'source "'"$HERE"'/up_lib.sh"; check_install_source'
t "UP_ALLOW_MISMATCH=1 = 통과"  0 sub env HOME="$mk/h" UP_ALLOW_MISMATCH=1 DEMO="$mk/a/workspace/tools/drive_demo" REPO="$mk/b" bash -c 'source "'"$HERE"'/up_lib.sh"; check_install_source'
t "같은 REPO = 통과"            0 sub env HOME="$mk/h" DEMO="$mk/a/workspace/tools/drive_demo" REPO="$mk/a" bash -c 'source "'"$HERE"'/up_lib.sh"; check_install_source'

trap - INT TERM HUP   # teardown 이 시험 셸에 건 무시 설정을 풀어야 자식 bash 가 신호를 받음
echo "== 정리 중 두 번째 신호는 무시 (정리 한 번만)"
cat > "$LOG_ROOT/sig.sh" <<EOS
source "$HERE/up_lib.sh"; RUN=""
teardown() { sleep 2; echo td >> "$LOG_ROOT/td.count"; }
eval "\$(sed -n '/^_IN_SIGNAL=0/,/^}/p' "$HERE/up.sh")"
trap _on_signal INT TERM HUP
( sleep 0.5; kill -INT \$\$; sleep 0.7; kill -TERM \$\$ ) & disown
while :; do sleep 0.1; done
EOS
: > "$LOG_ROOT/td.count"
bash "$LOG_ROOT/sig.sh" > "$LOG_ROOT/sig.out" 2>&1
[ "$(wc -l < "$LOG_ROOT/td.count")" = 1 ] && grep -q "정리 중입니다" "$LOG_ROOT/sig.out" && { PASS=$((PASS+1)); echo "  PASS 두 번째 신호 = 안내만, 정리 1 번"; } || { FAIL=$((FAIL+1)); echo "  FAIL 두 번째 신호: $(cat "$LOG_ROOT/sig.out" | tr '\n' '|')"; }

echo "== start_bg 가 잠금 fd 9 를 자식에게 안 넘김"
exec 9> "$LOG_ROOT/.lk"; : > "$RUN/pids"; _TD_DONE=0
start_bg fdtest bash -c 'ls /proc/$$/fd > '"$LOG_ROOT"'/fds.out; sleep 1'
sleep 0.8; grep -qx 9 "$LOG_ROOT/fds.out" && { FAIL=$((FAIL+1)); echo "  FAIL fd 9 상속"; } || { PASS=$((PASS+1)); echo "  PASS fd 9 닫힘"; }
exec 9>&-; teardown >/dev/null 2>&1

echo "== _opencr_check: 요약 줄 없으면 실패 (jetson_check.sh 없음 · timeout)"
od="$LOG_ROOT/dm"; mkdir -p "$od"
( DEMO="$od"; _opencr_check ) >/dev/null 2>&1; rc=$?
[ "$rc" = 1 ] && { PASS=$((PASS+1)); echo "  PASS jetson_check.sh 없음 = 점검 못 함 (실패)"; } || { FAIL=$((FAIL+1)); echo "  FAIL 없는데 통과 rc=$rc"; }
printf '#!/bin/sh\necho "  OpenCR ok"\n' > "$od/jetson_check.sh"; chmod +x "$od/jetson_check.sh"
( DEMO="$od"; _opencr_check ) >/dev/null 2>&1; rc=$?
[ "$rc" = 1 ] && { PASS=$((PASS+1)); echo "  PASS 요약 줄 없는 출력 = 실패"; } || { FAIL=$((FAIL+1)); echo "  FAIL 요약 없이 통과"; }
printf '#!/bin/sh\necho "  OpenCR ok"; echo "== 요약: ✅ 3 · ⚠ 0 · ❌ 0"\n' > "$od/jetson_check.sh"
( DEMO="$od"; _opencr_check ) >/dev/null 2>&1; rc=$?
[ "$rc" = 0 ] && { PASS=$((PASS+1)); echo "  PASS 요약 줄 있고 이상 없음 = 통과"; } || { FAIL=$((FAIL+1)); echo "  FAIL 정상인데 실패 rc=$rc"; }

trap - INT TERM HUP
echo "== 정리(teardown) 시작 뒤에는 신호로 안 끊김"
cat > "$LOG_ROOT/sig2.sh" <<EOS
source "$HERE/up_lib.sh"; RUN=""
_ros_pids() { :; }
( sleep 0.3; kill -INT \$\$ ) & disown
trap 'echo SIGGOT >> "$LOG_ROOT/sig2.out"' INT
teardown_probe() { teardown; sleep 1; echo ALIVE >> "$LOG_ROOT/sig2.out"; }
fold_arm() { sleep 1; }; UP_ARM_EXTENDED=1
teardown_probe
EOS
: > "$LOG_ROOT/sig2.out"; bash "$LOG_ROOT/sig2.sh" >/dev/null 2>&1
grep -q ALIVE "$LOG_ROOT/sig2.out" && ! grep -q SIGGOT "$LOG_ROOT/sig2.out" && { PASS=$((PASS+1)); echo "  PASS teardown 이 INT 를 무시 (끝까지 감)"; } || { FAIL=$((FAIL+1)); echo "  FAIL teardown 중 신호: $(cat "$LOG_ROOT/sig2.out" | tr '\n' ' ')"; }

echo "== collision_monitor 규칙 (정적): 없으면 die, use_collision_monitor:=false 만 예외"
grep -q 'use_collision_monitor:=false' "$HERE/up_mission.sh" && grep -q 'collision_monitor 노드가 없음.*\$RUN' "$HERE/up_mission.sh" && grep -A3 'node list.*collision_monitor' "$HERE/up_mission.sh" | grep -q die \
  && { PASS=$((PASS+1)); echo "  PASS 없으면 die · 인자로만 건너뜀"; } || { FAIL=$((FAIL+1)); echo "  FAIL collision_monitor 규칙"; }

echo "== UP_DRY"
( UP_DRY=1; wait_node /x 1 && wait_active y 1 && wait_estop_clear 1 && wait_topic /t 1 5 && start_bg z sleep 1 && teardown ) >/dev/null 2>&1 \
  && { PASS=$((PASS+1)); echo "  PASS UP_DRY=1 은 실행 없이 통과"; } || { FAIL=$((FAIL+1)); echo "  FAIL UP_DRY"; }

rm -rf "$LOG_ROOT"
echo "결과: PASS $PASS · FAIL $FAIL"
[ "$FAIL" = 0 ]
