# shellcheck shell=bash
# up_lib.sh — up.sh 공통 함수 (source 해서 씀). 계약: scratchpad/up/CONTRACT.md
# UP_DRY=1 이면 실행 없이 명령만 찍음. set -u 가정 (set -e 는 쓰지 않음).

: "${UP_DRY:=0}"
: "${DEMO:=$HOME/demo_v2}"
: "${REPO:=$HOME/Lv2_Cartlidar_Assignment}"   # 젯슨 기준 저장소 (11-23 결정). ~/git 판은 11-18 판이라 쓰지 않음
: "${LOG_ROOT:=$HOME/robot_logs}"
: "${RUN:=}"
: "${PORTS:=}"
: "${UP_KILL_LEFTOVER:=0}"   # 1 = teardown 때 우리가 띄운 프로세스의 자손 (그룹 밖으로 나간 것) 도 끔. 남의 프로세스는 어느 값이든 안 끔
: "${UP_POLL:=1}"           # 대기 함수 확인 간격 (초) — 시험에서 줄임
: "${UP_REARM_NOTE:=10}"    # REARM 안내 반복 간격 (초)

# ROS · 로봇 관련 프로세스 패턴 — 실행 파일 위치 · 명령 기준 (편집기 · tail 등이 파일 이름만으로 걸리지 않게).
# 첫 글자를 [x] 로 감싸서 pgrep 이 자기 명령줄과 맞지 않게 함
ROS_PAT='/opt/ros/|install/[^ ]*/(lib|bin)/|[r]os2 (launch|run|bag)|[-][-]ros-args|[r]viz2|python3? [^ ]*(scan_rotate|demo_follow|arm_look|v2_filter|box_scan_guard|puck_survey)\.py|[r]un_demo\.sh'
# 위 패턴에 걸려도 목록에서 뺄 argv0 (편집기 · 보기 · 셸 래퍼) 와 일시적 ros2 CLI · 데몬
ROS_SKIP_ARGV0='vim|vi|nvim|nano|less|more|tail|head|cat|grep|egrep|rg|ssh|sshd|tmux|screen|man|watch|sudo|timeout'
ROS_SKIP_CMD='ros2 (topic|node|service|lifecycle|action|control|param|interface|pkg|doctor|wtf)|_ros2_daemon|ros2 daemon'
# 팔 하드웨어를 잡고 있을 프로세스 — 끄면 팔이 접히지 않고 토크가 꺼짐
ARM_HW_PAT='opencr_hardware|controller_manager|manipulator'

# 끝 요약용 기록
UP_T0=$SECONDS; UP_STEP_T=$SECONDS; UP_STEPS=(); UP_EVENTS=(); UP_RETRIES=0; UP_WARNS=0; UP_ESTOP_UNTESTED=0
# ---------------------------------------------------------------- 출력
say()  { printf '\033[36m· %s\033[0m\n' "$*"; }
ok()   { printf '\033[32m✓ %s\033[0m\n' "$*"; }
warn() { printf '\033[33m⚠ %s\033[0m\n' "$*" >&2; UP_WARNS=$((UP_WARNS+1)); }
note_event() { UP_EVENTS+=("$*"); }   # 끝 요약에 남길 일 (재시도 · 건너뜀 · 시험 안 됨)
step_mark() {   # step_mark 이름 : 앞 단계 소요 시간을 기록하고 새 단계 시작
  UP_STEPS+=("${UP_STEP_NAME:-시작 전} $((SECONDS - UP_STEP_T)) s"); UP_STEP_NAME=$1; UP_STEP_T=$SECONDS
}
_dry() { [ "$UP_DRY" = 1 ]; }
dry_say() { printf '\033[35m[DRY] %s\033[0m\n' "$*"; }

die() {
  printf '\033[31m✗ %s\033[0m\n' "$*" >&2
  if [ -n "$RUN" ] && [ -d "$RUN" ]; then
    echo "── 로그 끝 (최근 수정 3 개 · 각 15 줄) ──" >&2
    local f
    for f in $(ls -t "$RUN"/*.log 2>/dev/null | head -3); do
      echo "[$(basename "$f")]" >&2; tail -n 15 "$f" >&2
    done
  fi
  teardown
  exit 1
}

# ros2 호출: 외부 명령이면 timeout 8 로 감쌈 (CLI 가 걸리는 일 방지). 셸 함수 (시험용 가짜) 면 그대로
_ros2() {
  if [ "$(type -t ros2)" = file ]; then timeout "${UP_ROS2_TMO:-8}" ros2 "$@"; else ros2 "$@"; fi
}

# 실행 폴더 — 모드 이름으로 만듦. mode 를 안 주면 $UP_MODE
run_dir_init() {
  local mode=${1:-${UP_MODE:-run}}
  RUN="$LOG_ROOT/$(date +%y%m%d_%H%M)_$mode"
  mkdir -p "$RUN" && : > "$RUN/pids"
  export RUN
  say "로그 폴더: $RUN"
}

# ---------------------------------------------------------------- 백그라운드 실행
# start_bg NAME CMD... : setsid 새 프로세스 그룹. PGID 를 $RUN/pids 에 기록 (teardown 이 역순으로 끔)
start_bg() {
  local name=$1; shift
  if _dry; then dry_say "start_bg $name: $*"; return 0; fi
  [ -n "$RUN" ] || run_dir_init
  say "시작: $name  ($*)"
  setsid "$@" > "$RUN/$name.log" 2>&1 < /dev/null 9>&- &   # 9>&- : up.sh 잠금 fd 를 자식이 물고 있지 않게
  local pid=$!
  # setsid 는 자식이 그룹 리더가 되므로 PGID = PID
  echo "$pid $name" >> "$RUN/pids"
  sleep 0.3
  kill -0 "$pid" 2>/dev/null || warn "$name 가 바로 끝남 — $RUN/$name.log 확인"
  return 0
}

# ---------------------------------------------------------------- 대기 함수
# 공통: 마감까지 UP_POLL 간격으로 조건을 확인. 시도 홀수 = 데몬 캐시, 짝수 = --no-daemon (데몬이 늦게 보이는 함정)
_deadline() { local s=${1%%.*}; echo $(( SECONDS + ${s:-0} )); }   # 소수점 초는 버림

wait_node() {
  local node=$1 t=${2:-30} end n=0 out
  [[ $node == /* ]] || node="/$node"
  if _dry; then dry_say "wait_node $node ${t}s"; return 0; fi
  end=$(_deadline "$t")
  while :; do
    n=$((n+1))
    if [ $((n % 2)) = 1 ]; then out=$(_ros2 node list 2>/dev/null); else out=$(_ros2 node list --no-daemon 2>/dev/null); fi
    if printf '%s\n' "$out" | grep -qxF "$node"; then ok "노드 $node 보임"; return 0; fi
    [ "$SECONDS" -ge "$end" ] && break
    sleep "$UP_POLL"
  done
  warn "노드 $node 가 ${t}s 안에 안 보임"; return 1
}

wait_service() {
  local srv=$1 t=${2:-30} end n=0 out
  if _dry; then dry_say "wait_service $srv ${t}s"; return 0; fi
  end=$(_deadline "$t")
  while :; do
    n=$((n+1))
    if [ $((n % 2)) = 1 ]; then out=$(_ros2 service list 2>/dev/null); else out=$(_ros2 service list --no-daemon 2>/dev/null); fi
    if printf '%s\n' "$out" | grep -qxF "$srv"; then ok "서비스 $srv 보임"; return 0; fi
    [ "$SECONDS" -ge "$end" ] && break
    sleep "$UP_POLL"
  done
  warn "서비스 $srv 가 ${t}s 안에 안 보임"; return 1
}

# wait_topic TOPIC T [MIN_HZ] : 토픽이 보이고, MIN_HZ 가 있으면 실제 발행 속도도 확인 (ros2 topic hz 의 average rate)
wait_topic() {
  local topic=$1 t=${2:-30} hz=${3:-} end n=0 out rate
  if _dry; then dry_say "wait_topic $topic ${t}s ${hz:+≥${hz}Hz}"; return 0; fi
  end=$(_deadline "$t")
  while :; do
    n=$((n+1))
    if [ $((n % 2)) = 1 ]; then out=$(_ros2 topic list 2>/dev/null); else out=$(_ros2 topic list --no-daemon 2>/dev/null); fi
    if printf '%s\n' "$out" | grep -qxF "$topic"; then
      if [ -z "$hz" ]; then ok "토픽 $topic 보임"; return 0; fi
      rate=$(_hz_once "$topic")
      if [ -n "$rate" ] && awk -v r="$rate" -v m="$hz" 'BEGIN{exit !(r+0>=m+0)}'; then
        ok "토픽 $topic ${rate} Hz (≥ $hz)"; return 0
      fi
      say "$topic 속도 ${rate:-측정 못 함} Hz (< $hz) — 다시 확인"
    fi
    [ "$SECONDS" -ge "$end" ] && break
    sleep "$UP_POLL"
  done
  warn "토픽 $topic 이 ${t}s 안에 준비 안 됨${hz:+ (≥ $hz Hz)}"; return 1
}
_hz_once() {   # 마지막 "average rate: X" 의 X
  local o
  if [ "$(type -t ros2)" = file ]; then o=$(timeout 6 ros2 topic hz --window 10 "$1" 2>/dev/null); else o=$(ros2 topic hz "$1" 2>/dev/null); fi
  printf '%s\n' "$o" | awk '/average rate:/ {r=$3} END {print r}'
}

# wait_active NODE T : lifecycle active 까지. Nav2 lifecycle_manager 가 정상 기동 중일 수 있어, 같은 상태로 UP_LC_STUCK (기본 25) 초
#  머문 뒤에만 개입 (unconfigured → configure, inactive → activate. amcl inactive 함정). 개입은 끝 요약에 남김
wait_active() {
  local node=$1 t=${2:-60} end st last_st="" since tries=0 n=0
  [[ $node == /* ]] || node="/$node"
  if _dry; then dry_say "wait_active $node ${t}s (같은 상태로 ${UP_LC_STUCK:-25}s 머물면 개입)"; return 0; fi
  end=$(_deadline "$t"); since=$SECONDS
  while :; do
    n=$((n+1))   # 11-23: 홀수 = 데몬, 짝수 = --no-daemon (데몬이 끊겨 있으면 데몬 조회만으로는 영원히 "응답 없음")
    if [ $((n % 2)) = 1 ]; then st=$(_ros2 lifecycle get "$node" 2>/dev/null | awk 'NR==1 {print $1}')
    else st=$(_ros2 lifecycle get --no-daemon "$node" 2>/dev/null | awk 'NR==1 {print $1}'); fi
    [ "$st" = active ] && { ok "$node active"; return 0; }
    if [ "$st" != "$last_st" ]; then last_st=$st; since=$SECONDS; fi
    if [ $((SECONDS - since)) -ge "${UP_LC_STUCK:-25}" ]; then
      case "$st" in
        unconfigured) say "$node unconfigured 로 $((SECONDS - since)) s 멈춰 있음 → configure"
          _ros2 lifecycle set "$node" configure >> "${RUN:-/dev/null}/lifecycle.log" 2>&1
          UP_RETRIES=$((UP_RETRIES+1)); note_event "$node configure 개입 ($((SECONDS - since)) s 멈춤)"; since=$SECONDS ;;
        inactive)
          tries=$((tries+1))
          say "$node inactive 로 $((SECONDS - since)) s 멈춰 있음 → activate (시도 $tries)"
          _ros2 lifecycle set "$node" activate >> "${RUN:-/dev/null}/lifecycle.log" 2>&1
          UP_RETRIES=$((UP_RETRIES+1)); note_event "$node activate 개입 $tries 번째 ($((SECONDS - since)) s 멈춤)"; since=$SECONDS ;;
      esac
    fi
    [ "$SECONDS" -ge "$end" ] && break
    sleep "$UP_POLL"
  done
  warn "$node 가 ${t}s 안에 active 가 안 됨 (지금 ${st:-응답 없음}) — ros2 lifecycle set $node activate"; return 1
}

# wait_estop_clear T : /e_stop (std_msgs/Bool, RELIABLE + TRANSIENT_LOCAL — opencr_hardware names::qos::e_stop) 가 false 가 될 때까지
#  - 한 번도 메시지를 못 받으면 (T 가 지나도) 토픽 자체가 없는 것 → die (rc100_joy · opencr 가 안 떴을 가능성)
#  - 처음 받은 값이 이미 false 면 통과하되 그 사실을 찍음 (REARM 을 이미 눌렀거나 처음부터 풀려 있음)
wait_estop_clear() {
  local t=${1:-120} end out last=-9999 got=0 first=1 seen_true=0
  if _dry; then dry_say "wait_estop_clear ${t}s (메시지를 한 번도 못 받으면 die)"; return 0; fi
  end=$(_deadline "$t")
  while :; do
    out=$(_ros2 topic echo --once --qos-reliability reliable --qos-durability transient_local /e_stop std_msgs/msg/Bool 2>/dev/null)
    if printf '%s\n' "$out" | grep -qE 'data:[[:space:]]*(true|false)'; then
      got=1
      printf '%s\n' "$out" | grep -qE 'data:[[:space:]]*true' && seen_true=1
      if printf '%s\n' "$out" | grep -qE 'data:[[:space:]]*false'; then
        if [ "$seen_true" != 1 ]; then
          UP_ESTOP_UNTESTED=1; note_event "E-stop 이 걸린 상태를 못 봄 — 리모컨 1 로 멈추는지 직접 확인 필요"
          warn "E-stop 이 걸린 상태를 못 봄 (시험 안 됨) — 리모컨 1 을 눌러 멈추는지 직접 확인하세요"
        fi
        if [ "$first" = 1 ]; then ok "/e_stop 이미 false 로 시작 (REARM 을 이미 눌렀거나 처음부터 풀려 있음)"
        else ok "/e_stop 풀림 (false)"; fi
        return 0
      fi
      first=0
      if [ $((SECONDS - last)) -ge "$UP_REARM_NOTE" ]; then
        warn "비상정지 중 (/e_stop = true) — 리모컨 2+3 (REARM) 을 누르세요"; last=$SECONDS
      fi
    elif [ $((SECONDS - last)) -ge "$UP_REARM_NOTE" ]; then
      warn "/e_stop 값을 아직 못 받음 — rc100_joy · opencr 노드 확인"; last=$SECONDS
    fi
    [ "$SECONDS" -ge "$end" ] && break
    sleep "$UP_POLL"
  done
  [ "$got" = 1 ] || die "/e_stop 토픽 없음 — rc100_joy · opencr 확인 (${t}s 동안 메시지 한 번도 못 받음)"
  warn "/e_stop 이 ${t}s 안에 안 풀림"; return 1
}

# ask_enter MSG : Enter = 계속, q = 끝 (teardown 후 exit 0)
ask_enter() {
  local ans=""
  if _dry; then dry_say "ask_enter: $*"; return 0; fi
  printf '\033[1m▶ %s  [Enter = 계속 · q = 끝] \033[0m' "$*"
  read -r ans || ans=q
  case "$ans" in q|Q) say "사용자가 끝냄"; teardown; exit 0 ;; esac
  return 0
}

# ask_yes MSG : 정확히 yes 를 입력해야 계속. 그 외 (Enter 포함 · 입력 없음) = 끝 (teardown 후 exit 0)
ask_yes() {
  local ans=""
  if _dry; then dry_say "ask_yes: $*"; return 0; fi
  printf '\033[1m▶ %s  [계속하려면 yes 입력 · 그 외 = 끝] \033[0m' "$*"
  read -r ans || ans=q
  [ "$ans" = yes ] && return 0
  say "계속하지 않고 끝냄"; teardown; exit 0
}

# ---------------------------------------------------------------- preflight
# 남은 ROS 프로세스 PID 목록 (자기 셸 · 부모 · up.sh 줄 제외)
_ancestors() {   # 나 · 부모 · 조상 PID (ssh bash -c "...명령 문구..." 가 패턴에 걸려도 죽이지 않게)
  local p=$$
  while [ -n "$p" ] && [ "$p" -gt 1 ]; do echo "$p"; p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' '); done
}
_ros_pids() {
  local p cmd anc a0
  anc=" $(_ancestors | tr '\n' ' ') "
  for p in $(pgrep -f "$ROS_PAT" 2>/dev/null); do
    case "$anc" in *" $p "*) continue ;; esac
    cmd=$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null) || continue
    case "$cmd" in *up.sh*|*pgrep*|*pkill*) continue ;; esac
    a0=$(basename "${cmd%% *}")
    case "$a0" in vim|vi|nvim|nano|less|more|tail|head|cat|grep|egrep|rg|ssh|sshd|tmux|screen|man|watch|sudo|timeout) continue ;; esac
    printf '%s' "$cmd" | grep -qE "$ROS_SKIP_CMD" && continue
    echo "$p"
  done
}
_my_uid_pids() { local p; for p in "$@"; do [ "$(ps -o uid= -p "$p" 2>/dev/null | tr -d ' ')" = "$(id -u)" ] && echo "$p"; done; }   # 내 계정 것만 (남의 계정은 못 끔)

# 남은 것 끄기 (INT → TERM → KILL). 호출 전에 사용자에게 y 를 받았어야 함 (preflight_procs · down). 남의 프로세스도 포함하므로 묻기 없이 부르지 말 것
kill_ros_all() {
  local pids p i
  pids=$(_my_uid_pids $(_ros_pids) | tr '\n' ' ')
  [ -z "${pids// /}" ] && return 0
  if _dry; then dry_say "kill_ros_all: $pids"; return 0; fi
  # shellcheck disable=SC2086
  kill -INT $pids 2>/dev/null
  for i in 1 2 3 4 5 6; do sleep 0.5; [ -z "$(_my_uid_pids $(_ros_pids))" ] && return 0; done
  pids=$(_my_uid_pids $(_ros_pids) | tr '\n' ' ')
  # shellcheck disable=SC2086
  [ -n "${pids// /}" ] && kill -TERM $pids 2>/dev/null
  for i in 1 2 3 4; do sleep 0.5; [ -z "$(_my_uid_pids $(_ros_pids))" ] && return 0; done
  for p in $(_my_uid_pids $(_ros_pids)); do kill -KILL "$p" 2>/dev/null; done
  sleep 0.5
  return 0
}

# ROS 프로세스가 하나도 없을 때만 fastdds 공유 메모리 지움 (살아 있는 노드의 shm 을 지우면 DDS 가 깨짐)
clean_shm() {
  if [ -n "$(_ros_pids)" ]; then warn "ROS 프로세스가 남아 있어 /dev/shm/fastdds* 는 지우지 않음"; return 1; fi
  if _dry; then dry_say "rm -f /dev/shm/fastdds* /dev/shm/sem.fastdds*"; return 0; fi
  # 11-23: ros2 daemon 은 _ros_pids 에서 빠지지만 shm 을 지우면 통신이 끊겨 노드를 못 찾음 (첫 실물 4 회 원인) → 먼저 끔 (다음 ros2 명령 때 다시 뜸)
  timeout 10 ros2 daemon stop >/dev/null 2>&1
  rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  say "/dev/shm 의 fastdds 잔재 지움"
  return 0
}

# preflight (1)(2) = down 모드도 이것만 씀: 남은 ROS 프로세스 목록 (사용자 · tty · 경과 · 명령) → "kill" 을 정확히 입력해야만 끔 → 모두 0 이면 shm 지움
# 반환: 0 = 깨끗함 (또는 kill 로 끔), 1 = 남아 있음 (안 끄겠다고 함 · 입력 없음 · 안 꺼짐)
preflight_procs() {
  local n i ans="" row arm=0 other=0 cmd
  n=$(_ros_pids | wc -l)
  if [ "$n" -gt 0 ]; then
    warn "남은 ROS 프로세스 $n 개 (다른 세션 · 다른 사람이 띄운 것일 수 있음):"
    printf '    %-7s %-10s %-7s %-9s %s\n' PID 사용자 tty 경과 명령
    for i in $(_ros_pids); do
      row=$(ps -o user=,tty=,etime= -p "$i" 2>/dev/null)
      cmd=$(tr '\0' ' ' < "/proc/$i/cmdline" 2>/dev/null | cut -c1-100)
      printf '    %-7s %s  %s\n' "$i" "$row" "$cmd"
      printf '%s' "$cmd" | grep -qE "$ARM_HW_PAT" && arm=1
      [ "$(ps -o uid= -p "$i" 2>/dev/null | tr -d ' ')" = "$(id -u)" ] || other=1
    done
    [ "$arm" = 1 ] && warn "팔 하드웨어를 쓰는 프로세스가 있음 — 끄면 팔이 접히지 않고 토크가 꺼져 처질 수 있음 (팔 시험 중인지 확인)"
    [ "$other" = 1 ] && warn "다른 계정의 프로세스는 끌 수 없음 (sudo 필요 · 직접 확인) — 끄기 대상에서 뺌"
    if _dry; then dry_say "끌지 묻기 생략 (실제로는 kill 을 입력해야만 끔)"; return 0; fi
    printf '위 프로세스를 끄려면 kill 이라고 입력 (그 외 = 안 끔): '; read -r ans || ans=n
    [ "$ans" = kill ] || return 1
    kill_ros_all
    [ -z "$(_my_uid_pids $(_ros_pids))" ] || { warn "프로세스가 안 꺼짐 — ps -ef | grep ros 로 직접 확인"; return 1; }
    [ -z "$(_ros_pids)" ] || { warn "다른 계정의 프로세스가 남음 — 직접 확인"; return 1; }
  else ok "남은 ROS 프로세스 없음"; fi
  clean_shm
  return 0
}

_opencr_check() {   # 0 = 통과, 1 = FAULT·MOTOR_CONFIG 등 · 출력은 $RUN/opencr_check.log
  local out rc=0
  out=$(cd "$DEMO" && timeout 60 ./jetson_check.sh opencr 2>&1) || true
  printf '%s\n' "$out" | tee "${RUN:-/dev/null}/opencr_check.log" 2>/dev/null | sed 's/^/    /'
  # jetson_check.sh 가 없거나 timeout 으로 끝나면 마지막의 "== 요약:" 줄이 없음 → 점검 못 함 = 실패
  if ! printf '%s\n' "$out" | grep -q '== 요약:'; then warn "OpenCR 점검을 못 함 (jetson_check.sh 없음 · timeout · 요약 줄 없음)"; rc=1; fi
  # err 필드 (예: err FAULT|MOTOR_CONFIG) 와 ❌ 줄
  if printf '%s\n' "$out" | grep -E 'err .*(FAULT|MOTOR_CONFIG|MOTOR_HW|ROLE_MISMATCH|SYNCREAD)' >/dev/null; then rc=1; fi
  # 요약 줄 (== 요약: ✅ N · ⚠ N · ❌ N) 의 ❌ 개수가 1 이상이면 실패 (요약 줄 자체에 ❌ 글자가 늘 있으므로 개수를 읽음)
  printf '%s\n' "$out" | grep '== 요약:' | grep -qE '❌ *[1-9]' && rc=1
  # 배터리 판정 (경고만): 11.1 V 미만 또는 nan
  printf '%s\n' "$out" | grep -oE '배터리 [^ ]+ V' | while read -r _ v _; do
    case "$v" in nan*|"") echo "WARN" ;; *) awk -v v="$v" 'BEGIN{exit !(v+0<11.1)}' && echo "WARN" ;; esac
  done | grep -q WARN && warn "배터리 11.1 V 미만이거나 값 없음 — 충전 · 12 V 확인"
  return $rc
}

# preflight : 한 번만 돔 (up.sh 가 부르고 UP_PREFLIGHT_DONE=1 을 export — 모드 파일은 부르지 않음)
preflight() {
  local i
  [ "${UP_PREFLIGHT_DONE:-0}" = 1 ] && return 0
  say "── 사전 점검 ──"
  # (1)(2) 남은 프로세스 · shm
  preflight_procs || die "남은 프로세스가 있어 중단 — 직접 확인하거나 ./up.sh down 으로 정리 후 다시"
  # (3) OpenCR 두 대
  if _dry; then dry_say "($DEMO) ./jetson_check.sh opencr  — FAULT · MOTOR_CONFIG · 배터리 판정"; else
    for i in 1 2 3; do
      if _opencr_check; then ok "OpenCR 두 보드 점검 통과"; break; fi
      [ "$i" = 3 ] && die "OpenCR 점검 3 번 실패 — FAULT/MOTOR_CONFIG 면 12 V 를 켠 채 두 보드 RESET"
      warn "OpenCR 이상 — 12 V 를 켠 채 두 보드의 RESET 버튼을 누르세요 (다시 점검 $i/3)"
      ask_enter "RESET 했으면"
      sleep 3
    done
  fi
  # (4) 라이다
  for i in /dev/lidar_front /dev/lidar_rear; do
    if _dry; then dry_say "test -e $i"; elif [ -e "$i" ]; then ok "$i 있음"; else die "$i 없음 — 라이다 USB · udev(99-sensors.rules) 확인"; fi
  done
  # (5) ttyACM 정확히 2 개 → PORTS
  if _dry; then PORTS=/dev/ttyACM0,/dev/ttyACM1; dry_say "PORTS=$PORTS"; else
    local acm=()
    shopt -s nullglob; acm=(/dev/ttyACM*); shopt -u nullglob
    [ "${#acm[@]}" = 2 ] || die "/dev/ttyACM* 가 ${#acm[@]} 개 (${acm[*]:-없음}) — 정확히 2 개여야 함 (보드 리셋 뒤 번호가 바뀌었는지 확인)"
    PORTS=$(IFS=,; echo "${acm[*]}")
    ok "PORTS=$PORTS"
  fi
  export PORTS
  export UP_PREFLIGHT_DONE=1
  return 0
}

# ---------------------------------------------------------------- 우리가 띄운 프로세스 판별 (남의 프로세스 보호)
# $RUN/pids 의 PGID 를 그룹 ID 또는 세션 ID 로 가진 프로세스 + 그 자손 (PPID 사슬). 부모가 죽으면 사슬이 끊기므로 끄기 전에 미리 구해 둠
_ours() {
  [ -n "$RUN" ] && [ -f "$RUN/pids" ] || return 0
  local pg; pg=" $(awk '{print $1}' "$RUN/pids" | tr '\n' ' ') "
  ps -eo pid=,ppid=,pgid=,sid= 2>/dev/null | awk -v pg="$pg" -v me="$$" '
    BEGIN { n = split(pg, a, " "); for (i = 1; i <= n; i++) g[a[i]] = 1 }
    { ids[NR] = $1; par[$1] = $2; if (($3 in g) || ($4 in g)) mark[$1] = 1 }
    END {
      ch = 1
      while (ch) { ch = 0; for (i = 1; i <= NR; i++) { p = ids[i]; if (!(p in mark) && (par[p] in mark)) { mark[p] = 1; ch = 1 } } }
      for (p in mark) if (p != me) print p
    }'
}
_is_ours() { _ours | grep -qx "$1"; }   # 이 PID 가 우리가 띄운 것인가

# ---------------------------------------------------------------- launch 인자 확인
# check_launch_args PKG FILE ARG... : `ros2 launch PKG FILE --show-args` 에 없는 인자는 warn 만 (실행은 막지 않음)
#  ARG 는 이름만 (use_teleop) 또는 use_teleop:=true 도 됨. --show-args 를 못 받으면 확인 못 했다고만 알림.
#  주의: 인자는 ros2 launch 명령줄에서 전역 launch 설정으로 퍼져 include 한 파일에서 선언된 것도 먹음 (예: bringup 이 선언하지
#        않은 rc100 · use_loc_monitor 는 include 한 base_bringup · localization 이 선언). 그러므로 그 인자는 선언한 파일로 따로 확인할 것.
check_launch_args() {
  local pkg=$1 file=$2 out a miss=()
  shift 2
  if _dry; then dry_say "check_launch_args $pkg $file $*"; return 0; fi
  if [ "$(type -t ros2)" = file ]; then out=$(timeout 30 ros2 launch "$pkg" "$file" --show-args 2>/dev/null)
  else out=$(ros2 launch "$pkg" "$file" --show-args 2>/dev/null); fi
  if [ -z "$out" ]; then warn "launch 인자 확인 못 함 ($pkg $file --show-args 가 비어 있음)"; return 0; fi
  for a in "$@"; do
    a=${a%%:=*}
    printf '%s\n' "$out" | grep -qF "'$a':" || miss+=("$a")
  done
  if [ "${#miss[@]}" -gt 0 ]; then
    warn "$pkg $file 에 선언 안 된 인자: ${miss[*]} — 오타거나 효과 없을 수 있음 (--show-args 로 확인)"
    return 1
  fi
  ok "launch 인자 확인: $pkg $file ($*)"
  return 0
}

# ---------------------------------------------------------------- run_demo.sh 가 어느 install 을 source 하는지 (경고만)
# run_demo.sh 는 ① $DEMO/../../.. ② ~/Lv2_Cartlidar_Assignment ③ ~/GIT/lv2_CartLidar/Lv2_Cartlidar_Assignment 순으로 setup.bash 가 있는
# 첫 후보를 source 해서 up.sh 가 먼저 source 한 $REPO 를 덮어씀. 다르면 서로 다른 빌드가 섞임 → die (UP_ALLOW_MISMATCH=1 이면 경고만).
check_install_source() {
  local d1 first="" c
  d1=$(cd "$DEMO/../../.." 2>/dev/null && pwd)
  for c in "$d1" "$HOME/Lv2_Cartlidar_Assignment" "$HOME/GIT/lv2_CartLidar/Lv2_Cartlidar_Assignment"; do
    [ -n "$c" ] && [ -f "$c/workspace/ros2_ws/install/setup.bash" ] && { first=$c; break; }
  done
  if [ -z "$first" ]; then
    warn "run_demo.sh 가 source 할 install 을 못 찾음 — run_demo 가 처음부터 실패할 수 있음"; return 0
  fi
  if [ "$(readlink -f "$first")" = "$(readlink -f "${REPO:-/nonexistent}")" ]; then
    ok "run_demo 가 source 할 install = $first (= REPO, 같음)"
  else
    warn "run_demo 가 source 할 install = $first  ≠  REPO = ${REPO:-?}  → 서로 다른 빌드가 섞임 (scan_rotate 의 URDF 도 ~/Lv2_Cartlidar_Assignment 고정)"
    warn "  해결: DEMO 를 \$REPO/workspace/tools/drive_demo 로, 또는 ~/Lv2_Cartlidar_Assignment 를 REPO 로 가는 링크로"
    if [ "${UP_ALLOW_MISMATCH:-0}" = 1 ]; then warn "UP_ALLOW_MISMATCH=1 — 그래도 진행"; note_event "install ≠ REPO 인데 진행 (UP_ALLOW_MISMATCH=1)"
    else die "install 이 REPO 와 다름 — 위 해결을 하거나 UP_ALLOW_MISMATCH=1 로 일부러 진행"; fi
  fi
  return 0
}

# ---------------------------------------------------------------- 끄기
# 팔 접기: 하드웨어 launch 를 끄기 전에 /manipulator/pose 에 DRIVE 를 보내 봄 (시간 한도 15 s, 실패해도 계속 진행)
#  ⚠ 확인 필요: 토크가 꺼질 때 팔이 떨어지는지 (DRIVE 로 접어 두면 꺼질 때 덜 떨어지는지) — 실물에서 한 번 확인할 것
#  형식: robot_interfaces/action/ArmPose (Goal: string name) — workspace/tools/robot_console/actions/arm_demo.py 와 같음
fold_arm() {
  local out rc=0
  if _dry; then dry_say "fold_arm: /manipulator/pose 서버가 있으면 DRIVE (15 s 한도)"; return 0; fi
  if [ "$(type -t ros2)" = file ]; then out=$(timeout 6 ros2 action list 2>/dev/null); else out=$(ros2 action list 2>/dev/null); fi
  if ! printf '%s\n' "$out" | grep -qxF /manipulator/pose; then say "팔 접기 건너뜀 (/manipulator/pose 서버 없음)"; return 0; fi
  say "팔을 DRIVE 자세로 접어 봄 (15 s 한도)"
  if [ "$(type -t ros2)" = file ]; then
    out=$(timeout 15 ros2 action send_goal /manipulator/pose robot_interfaces/action/ArmPose '{name: DRIVE}' 2>&1) || rc=$?
  else out=$(ros2 action send_goal /manipulator/pose robot_interfaces/action/ArmPose '{name: DRIVE}' 2>&1) || rc=$?; fi
  [ -n "$RUN" ] && printf '%s\n' "$out" >> "$RUN/fold_arm.log" 2>/dev/null
  if [ "$rc" = 0 ] && printf '%s\n' "$out" | grep -qE 'success: *(true|True)'; then ok "팔 DRIVE 자세 확인"
  else warn "팔 접기 실패 (rc=$rc) — 계속 진행 (비상정지 · HOLD · 서버 거절일 수 있음). 토크가 꺼지면 팔이 처질 수 있으니 손으로 받칠 것"; fi
  return 0
}

_TD_DONE=0
teardown() {
  [ "$_TD_DONE" = 1 ] && return 0
  _TD_DONE=1
  _IN_SIGNAL=1; trap '' INT TERM HUP   # die · q · DONE 으로 시작한 정리도 신호로 끊기지 않게
  local pid name pids=() names=() i j alive mine others p
  if _dry; then dry_say "teardown (팔 접기 → pids 역순 SIGINT → TERM → KILL, 그룹 밖 남은 것은 목록만)"; return 0; fi
  say "── 정리 ──"
  mine=$(_ours | tr '\n' ' ')   # 끄기 전에 구해 둠 (부모가 죽으면 PPID 사슬이 끊김)
  if [ -n "$RUN" ] && [ -f "$RUN/pids" ]; then
    while read -r pid name; do [ -n "$pid" ] && { pids+=("$pid"); names+=("${name:-?}"); }; done < "$RUN/pids"
    # 우리가 팔을 편 적이 있을 때만 (drive ③ · mission bringup 을 띄운 뒤 UP_ARM_EXTENDED=1) 끄기 전에 팔 접기.
    # 끝 자세는 DRIVE 그대로 — 토크가 빠질 때 처지는지는 실물에서 확인해 README 에 적을 것
    [ "${UP_ARM_EXTENDED:-0}" = 1 ] && fold_arm
    for ((i=${#pids[@]}-1; i>=0; i--)); do
      pid=${pids[i]}; name=${names[i]}
      kill -0 "$pid" 2>/dev/null || continue
      kill -INT -- "-$pid" 2>/dev/null
      alive=1
      for j in 1 2 3 4 5 6 7 8 9 10; do sleep 0.5; kill -0 "$pid" 2>/dev/null || { alive=0; break; }; done
      if [ "$alive" = 1 ]; then
        warn "$name SIGINT 로 안 꺼짐 → TERM"
        kill -TERM -- "-$pid" 2>/dev/null; sleep 2
        kill -0 "$pid" 2>/dev/null && { warn "$name → KILL"; kill -KILL -- "-$pid" 2>/dev/null; }
      fi
      say "$name 끝"
    done
  fi
  # 그룹 밖에 남은 것: 우리 자손 (mine) 과 남의 것을 가름. 남의 것은 어느 설정에서도 안 끔
  sleep 0.5
  local left_mine=""
  for p in $mine; do kill -0 "$p" 2>/dev/null && left_mine+="$p "; done
  if [ -n "$left_mine" ]; then
    if [ "$UP_KILL_LEFTOVER" = 1 ]; then
      warn "그룹 밖으로 나간 우리 자손 $left_mine— 끔 (UP_KILL_LEFTOVER=1)"
      # shellcheck disable=SC2086
      kill -INT $left_mine 2>/dev/null; sleep 1
      for p in $left_mine; do kill -0 "$p" 2>/dev/null && kill -TERM "$p" 2>/dev/null; done; sleep 1
      for p in $left_mine; do kill -0 "$p" 2>/dev/null && kill -KILL "$p" 2>/dev/null; done
    else
      warn "그룹 밖으로 나간 우리 자손 $left_mine— 안 끔 (UP_KILL_LEFTOVER=1 로 끌 수 있음)"
    fi
  fi
  others=""
  for p in $(_ros_pids); do case " $mine " in *" $p "*) ;; *) others+="$p " ;; esac; done
  if [ -n "$others" ]; then
    warn "남의 ROS 프로세스 (우리가 띄운 것 아님 — 건드리지 않음): $others"
    for p in $others; do printf '    %s  %s\n' "$p" "$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | cut -c1-110)" >&2; done
    warn "  정리하려면 ./up.sh down (목록을 보고 y)"
  fi
  return 0
}

# ---------------------------------------------------------------- 끝 요약 (EXIT 에서)
print_summary() {
  local e
  echo
  say "══ 요약 ══"
  say "모드: ${UP_MODE:-?} · REPO: $REPO ($(git -C "$REPO" log -1 --format=%h 2>/dev/null || echo 'HEAD 읽을 수 없음')) · 총 $((SECONDS - UP_T0)) s"
  UP_STEPS+=("${UP_STEP_NAME:-} $((SECONDS - UP_STEP_T)) s")
  [ -n "${UP_STEP_NAME:-}" ] && say "단계별: $(printf '%s · ' "${UP_STEPS[@]}" | sed 's/ · $//')"
  say "재시도(개입) $UP_RETRIES 번 · 경고 $UP_WARNS 개"
  for e in "${UP_EVENTS[@]}"; do warn "  · $e"; done
  [ -n "$RUN" ] && say "로그 폴더: $RUN"
  warn "물리 비상정지 없음 — 리모컨 1 이 유일한 정지 수단"
  say "젯슨 화면이나 tmux 에서 실행하세요 (ssh 가 끊기면 정리가 제대로 안 될 수 있음)"
  return 0
}
