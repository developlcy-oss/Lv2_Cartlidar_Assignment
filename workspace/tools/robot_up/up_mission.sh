# up_mission.sh — 미션 모드 (up.sh 가 source). mission_up · mission_start 함수.
# 전제: up_lib.sh 의 say ok warn die RUN start_bg wait_node wait_active wait_estop_clear ask_enter check_launch_args teardown, 변수 REPO PORTS.
# preflight 는 up.sh 가 한 번 함 (여기서 안 부름). Ctrl+C 는 up.sh 의 trap 이 mission_stop → teardown 순서로 처리.
# 단계: preflight → bringup(rc100) → 노드 → lifecycle active → /mission/ready == READY → REARM → Enter → /mission/start → status
set -u

# 준비 확인 · lifecycle 대기 한도 (초)
MISSION_NODE_TIMEOUT=${MISSION_NODE_TIMEOUT:-180}   # Nav2 는 /joint_states 뒤 nav2_delay 10 s 늦게 뜸 + 컨트롤러 기동
MISSION_LC_TIMEOUT=${MISSION_LC_TIMEOUT:-120}
MISSION_READY_TIMEOUT=${MISSION_READY_TIMEOUT:-300}

# DRY 이면 명령만 찍음
_m_run() { if [ "${UP_DRY:-0}" = 1 ]; then echo "[DRY] $*"; return 0; fi; "$@"; }

# /mission/ready 마지막 값 (transient_local 이라 늦게 구독해도 받음). 없으면 빈 문자열
_m_ready_value() {
  timeout 6 ros2 topic echo --once --qos-reliability reliable --qos-durability transient_local \
    --field data /mission/ready 2>/dev/null | sed -e '/^---$/d' -e 's/^"\(.*\)"$/\1/' | head -1
}

# 무엇이 OK 가 아닌지 보여 주고 최근 mission_node 로그 일부도 찍음
_m_show_not_ready() {
  warn "준비 안 됨: $1"
  _ros2 node list 2>/dev/null | grep -E 'mission_node|amcl|bt_navigator|controller_server|planner_server|map_server' | sed 's/^/    노드: /' || true
}

mission_up() {
  step_mark "① bringup"
  say "== 미션 모드: bringup → 준비 확인 (예상 3~6 분 — 기다리는 동안 Ctrl+C 를 누르지 마세요) =="
  say "회전 상한 max_wz:=0.5 rad/s 를 기본으로 줌 (바꾸려면 ./up.sh mission max_wz:=값)"
  say "팔이 앞으로 펴집니다 (DRIVE 자세). 주변 · 팔 앞을 비우세요."
  ask_enter "[팔 안전] 팔이 펴져도 되면 Enter (q = 끝)"

  # ① bringup: 라이다는 bringup sensors 가 /dev/lidar_* 로 잡음. AMCL 시작 자세는 bringup 이 HOME 으로 넣음 (덮어쓰지 않음).
  #    사용자 인자 ("$@") 가 뒤에서 앞 기본값을 덮어씀.
  #    rotate_wz 는 안 줌 (bringup.launch.py 324 행 주석: max_wz 만 주면 navigation 이 min(YAML 0.75, max_wz) 로 맞춤. 같이 주면
  #    max_wz 를 낮출 때 rotate > max 로 거부될 수 있음). rc100 은 bringup 이 선언하지 않고 include 한 base_bringup 으로 전달됨 (use_teleop 기본 true 라 그대로 됨).
  local m_args=(rc100:=true ports:="${PORTS:-auto}" max_wz:=0.5 "$@")
  local a user_names=()
  for a in ports max_wz "$@"; do user_names+=("${a%%:=*}"); done
  check_launch_args bringup bringup.launch.py "${user_names[@]}"        # 오타 · 없는 인자 경고
  check_launch_args opencr_hardware base_bringup.launch.py rc100 use_teleop
  UP_ARM_EXTENDED=1   # bringup 이 팔을 폄 — 이후 teardown 은 끄기 전에 팔을 접어 봄 (fold_arm)
  start_bg bringup ros2 launch bringup bringup.launch.py "${m_args[@]}"
  step_mark "② 노드"

  # ② 핵심 노드
  say "② /controller_manager · /mission_node 기다림"
  wait_node /controller_manager "$MISSION_NODE_TIMEOUT" || die "/controller_manager 안 뜸 — $RUN/bringup.log 확인"
  wait_node /mission_node "$MISSION_NODE_TIMEOUT"       || die "/mission_node 안 뜸 — $RUN/bringup.log 확인"
  ok "controller_manager · mission_node 확인"

  # ③ lifecycle 노드 (Nav2 는 /joint_states 뒤 nav2_delay 10 s 늦게 시작).
  #    위치 추정: map_server · amcl, 주행: controller · planner · behavior · velocity_smoother · bt_navigator
  step_mark "③ lifecycle"
  say "③ lifecycle 노드 active 기다림 (Nav2 는 늦게 뜸)"
  local n
  for n in map_server amcl controller_server planner_server behavior_server velocity_smoother bt_navigator; do
    wait_active "/$n" "$MISSION_LC_TIMEOUT" || die "/$n 가 active 안 됨 — $RUN/bringup.log 확인"
  done
  # collision_monitor 는 설정에 따라 없을 수 있음. 없으면 크래시일 수도 있으니 warn · 요약에 남김 (진행은 함)
  # use_collision_monitor (navigation.launch.py 159 행, 기본 true) 를 사용자가 false 로 줬을 때만 건너뜀. 아니면 꼭 있어야 함 → 없으면 die
  local cm_off=0
  for a in "$@"; do [ "${a//[[:space:]]/}" = "use_collision_monitor:=false" ] && cm_off=1; done
  if [ "$cm_off" = 1 ]; then warn "use_collision_monitor:=false — collision_monitor 확인 건너뜀 (충돌 감시 없음)"; note_event "collision_monitor 끔 (사용자 인자)"
  else
    if [ "${UP_DRY:-0}" != 1 ]; then
      _ros2 node list 2>/dev/null | grep -qx '/collision_monitor' || die "/collision_monitor 노드가 없음 — 크래시일 수 있음 ($RUN/bringup.log). 일부러 끄려면 use_collision_monitor:=false"
    fi
    wait_active /collision_monitor "$MISSION_LC_TIMEOUT" || die "/collision_monitor 가 active 안 됨 — $RUN/bringup.log 확인"
  fi
  ok "lifecycle 노드 active"

  # ④ /mission/ready == READY
  step_mark "④ READY"
  say "④ /mission/ready == READY 기다림 (최대 ${MISSION_READY_TIMEOUT} s)"
  local t0 now v last="" st; local -A inact_since=()
  t0=$(date +%s)
  if [ "${UP_DRY:-0}" = 1 ]; then
    echo "[DRY] ros2 topic echo --once --qos-durability transient_local --field data /mission/ready  (READY 까지 3 s 마다)"
  else
    while :; do
      v=$(_m_ready_value)
      if [ "$v" = "READY" ]; then break; fi
      now=$(date +%s)
      if [ "$v" != "$last" ]; then say "  준비 확인: ${v:-(아직 메시지 없음)}"; last="$v"; fi
      if [ $((now - t0)) -ge "$MISSION_READY_TIMEOUT" ]; then
        _m_show_not_ready "${v:-메시지 없음}"
        die "READY 안 됨 (${MISSION_READY_TIMEOUT} s) — $RUN/bringup.log 의 mission_node 준비 표 확인"
      fi
      # inactive 로 UP_LC_STUCK (25 s) 이상 머문 lifecycle 만 다시 시도 (정상 기동 중인 lifecycle_manager 와 안 겹치게)
      for n in amcl bt_navigator; do
        st=$(_ros2 lifecycle get "/$n" 2>/dev/null | awk 'NR==1 {print $1}')
        if [ "$st" = inactive ]; then
          [ -n "${inact_since[$n]:-}" ] || inact_since[$n]=$SECONDS
          if [ $((SECONDS - inact_since[$n])) -ge "${UP_LC_STUCK:-25}" ]; then
            warn "/$n inactive 로 $((SECONDS - inact_since[$n])) s 멈춤 → activate 재시도"
            _ros2 lifecycle set "/$n" activate >/dev/null 2>&1 || true
            UP_RETRIES=$((UP_RETRIES+1)); note_event "/$n activate 개입 (READY 대기 중)"; inact_since[$n]=$SECONDS
          fi
        else unset 'inact_since[$n]'; fi
      done
      sleep 3
    done
  fi
  ok "미션 READY"

  # ⑤ E-stop 풀기 (리모컨 REARM)
  step_mark "⑤ E-stop"
  say "⑤ E-stop 해제 확인"
  wait_estop_clear 120 || die "E-stop 안 풀림 — 리모컨 2+3 (REARM) 확인"
  ok "E-stop 해제"
}

# 상태를 찍는 python 도우미 (stdin 코드 아님: -c). 끝나면 종료코드 0 = DONE, 1 = FAILED, 130 = 중단
_m_status_py='
import sys, rclpy
from robot_interfaces.msg import MissionStatus
S = {0: "IDLE", 1: "RUNNING", 2: "PAUSED", 3: "DONE", 4: "FAILED"}
rclpy.init(); n = rclpy.create_node("up_mission_status"); last = [None]
def on(m):
    k = (m.step_index, m.state, m.reason)
    if k != last[0]:
        last[0] = k
        print(f"미션 단계 {m.step_index} {m.step_name} {S.get(m.state, m.state)} {m.reason}", flush=True)
n.create_subscription(MissionStatus, "/mission/status", on, 10)
try:
    while rclpy.ok() and (last[0] is None or last[0][1] not in (3, 4)):
        rclpy.spin_once(n, timeout_sec=0.5)
except KeyboardInterrupt:
    sys.exit(130)
sys.exit(0 if last[0] and last[0][1] == 3 else 1)
'

# 출발 자리 확인 (robot_console 의 start_check 재사용: 위치 감시 OK · 차체 ↔ 지도 여유 · 설계 출발 자리와 차이)
_m_start_check_py='
import sys, rclpy
sys.path.insert(0, sys.argv[1])
from start_check import wait_and_check
rclpy.init(); n = rclpy.create_node("up_start_check")
sys.exit(0 if wait_and_check(n, lambda t: print(t, flush=True)) else 1)
'

mission_start() {
  step_mark "⑥ 시작"
  say "== 미션 시작 =="
  ask_enter "[출발 확인] 미션 시작 (로봇이 움직입니다) — Enter = 시작, q = 끝"

  # 출발 자리 확인 (막히면 시작 안 함)
  say "출발 자리 확인 중"
  if [ "${UP_DRY:-0}" = 1 ]; then
    echo "[DRY] python3 -c <start_check> $REPO/workspace/tools/robot_console/actions"
  else
    python3 -c "$_m_start_check_py" "$REPO/workspace/tools/robot_console/actions" \
      || die "출발 자리 확인 실패 — 로봇을 출발 자리로 옮긴 뒤 다시 실행"
  fi

  # /mission/start (준비 확인이 덜 되었으면 거부됨 → 3 s 마다 최대 20 번)
  local i out rc started=0
  for i in $(seq 1 20); do
    if [ "${UP_DRY:-0}" = 1 ]; then
      echo "[DRY] ros2 service call /mission/start std_srvs/srv/Trigger {}"; started=1; break
    fi
    out=$(timeout 15 ros2 service call /mission/start std_srvs/srv/Trigger '{}' 2>&1); rc=$?
    # rc 124 = 15 s 안에 응답 없음 (서비스가 없으면 ros2 service call 은 끝없이 기다림)
    [ "$rc" = 124 ] && warn "mission_node 응답 없음 (15 s) — /mission/start 서비스 확인"
    if printf '%s' "$out" | grep -q 'success=True\|success: true'; then started=1; break; fi
    warn "시작 거부 ($i/20): $(printf '%s' "$out" | grep -o "message='[^']*'" | head -1)"
    sleep 3
  done
  [ "$started" = 1 ] || die "/mission/start 가 받아들여지지 않음"
  ok "★ 미션 시작됨"

  # /mission/status 계속 찍음. Ctrl+C = /mission/stop 뒤 teardown (up.sh trap)
  if [ "${UP_DRY:-0}" = 1 ]; then
    echo "[DRY] python3 -c <status>  (DONE/FAILED 까지; Ctrl+C = ros2 service call /mission/stop std_srvs/srv/Trigger {} → teardown)"
    return 0
  fi
  local pid rc=0
  python3 -c "$_m_status_py" &
  pid=$!
  # Ctrl+C 는 up.sh 의 trap (_on_signal) 이 처리: mission_stop → 이 도우미 kill → teardown → exit 130. wait 는 신호로 중단됨.
  wait "$pid" || rc=$?
  case "$rc" in
    0) ok "★ 미션 완료 (DONE)"; teardown ;;
    1) warn "미션 실패 (FAILED)"; teardown; return 1 ;;
    *) teardown; return "$rc" ;;
  esac
}

# /mission/status 가 RUNNING 을 벗어났는지 (또는 메시지가 아예 없는지) 확인하는 도우미: 최대 N 초. 벗어나면 0, 아직 RUNNING 이면 1
_m_wait_stopped_py='
import sys, time, rclpy
from robot_interfaces.msg import MissionStatus
limit = float(sys.argv[1]); rclpy.init(); n = rclpy.create_node("up_mission_stopwait"); st = [None]
def on(m): st[0] = m.state
n.create_subscription(MissionStatus, "/mission/status", on, 10)
end = time.time() + limit
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.3)
    if st[0] is not None and st[0] != 1:   # 1 = RUNNING
        sys.exit(0)
sys.exit(1 if st[0] == 1 else 0)   # 메시지를 못 받았으면 멈출 미션이 없다고 봄
'

# 미션 정지 요청 (/mission/stop Trigger) 뒤 상태가 RUNNING 을 벗어날 때까지 최대 10 s 기다림 (집기 · 놓기 도중에 팔을 접지 않게).
# up.sh 의 Ctrl+C 처리가 정의돼 있을 때 부름. 실패해도 계속 (teardown 이 이어짐)
mission_stop() {
  if [ "${UP_DRY:-0}" = 1 ]; then echo "[DRY] ros2 service call /mission/stop std_srvs/srv/Trigger {}  → /mission/status 가 멈춤 상태가 되거나 10 s 까지 기다림"; return 0; fi
  warn "/mission/stop 요청"
  timeout 10 ros2 service call /mission/stop std_srvs/srv/Trigger '{}' >/dev/null 2>&1 || warn "/mission/stop 응답 없음"
  if timeout 13 python3 -c "$_m_wait_stopped_py" 10 2>/dev/null; then ok "미션 멈춤 확인"
  else warn "미션이 아직 동작 중일 수 있음 — 그래도 정리함 (팔이 쥔 퍽을 손으로 받치세요)"; note_event "mission stop 뒤에도 RUNNING 이었을 수 있음"; fi
  return 0
}
