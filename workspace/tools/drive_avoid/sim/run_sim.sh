#!/usr/bin/env bash
# 장애물 주행 Gazebo 시험 (노트북, 2026-11-17 · 저장소 파일은 안 고침)
#   Gazebo 경기장 (상자 · L 은 실측 자리로 옮김) + AMCL + 경기 도구 Nav2 노드 (RPP · Spin) + 규칙 v2 필터
#   + 장애물 감시 · 정지 관문 · 주행 관리자 + 장애물 시나리오 · Gazebo 실제 자세로 충돌 여유 측정
#
#   ./sim/run_sim.sh <시나리오.json|none> [이름]
#   ./sim/run_sim.sh stop                       남은 시험 프로세스 정리
# 환경 변수: ROUTE="box3 box5" (기본 = 어제 순서 한 바퀴 + HOME) · SPEED=max|full|slow (기본 full, max = 경기 max 0.45 m/s)
#            GUARD_ARGS="-p close_range:=0.0 ..." (감시 인자, 예전 방식 비교: sim/guard_v1_args.txt) · GUI=1 (Gazebo 창) · DWELL=1.0 (정지점 서기 s)
#            NO_AVOID=1 (장애물 멈춤 끔, 비교용) · REC=1 (진단 녹화) · SIM_DOMAIN=74 · WS=~/lv2_sim/install (빌드: README)
# 결과: runs/<월일_시각>_시뮬_<이름>/ — summary.txt · truth_summary.txt · run.log · events.json · truth.csv · track.csv
# 본문을 { } 로 묶음: 실행 중에 이 파일을 고쳐도 엉뚱한 줄을 실행하지 않게
{
set +u
HERE="$(cd "$(dirname "$0")" && pwd)"; TOOL="$(dirname "$HERE")"; DEMO="$(cd "$TOOL/../drive_demo" && pwd)"
PAT='gz sim|gz-sim|gazebo_sim.launch|localization.launch|lib/nav2_amcl|lifecycle_manager|nav2_map_server|scan_merge|scan_body_mask|parameter_bridge|robot_state_publisher|odom_icr|ekf_node|controller_server|behavior_server|v2_filter.py|stop_gate.py|obstacle_guard.py|avoid_runner.py|sim/scenario.py|ros_gz_sim'
stop_all() {
  for sig in INT TERM KILL; do
    pids=$(pgrep -f "$PAT" | grep -vx "$$")
    [ -z "$pids" ] && return
    kill -$sig $pids 2>/dev/null; sleep 3
  done
}
if [ "$1" = stop ]; then stop_all; echo "정리 끝"; exit 0; fi
if pgrep -f "gz sim|gz-sim-main|gz-sim-server" > /dev/null; then echo "Gazebo 가 이미 돌고 있음 → ./sim/run_sim.sh stop 뒤 다시"; exit 1; fi

SC="${1:-none}"; [ "$SC" != none ] && SC="$(readlink -f "$SC")"
NAME="${2:-$(basename "${SC%.json}")}"
export ROS_DOMAIN_ID=${SIM_DOMAIN:-74} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_IP=127.0.0.1   # 이 PC 안에서만 (Jetson 과 안 섞임)
source /opt/ros/lyrical/setup.bash
source "${WS:-$HOME/lv2_sim/install}/setup.bash"
MP="$(ros2 pkg prefix mission_planner)/share/mission_planner/maps"
OUT="$TOOL/runs/$(date +%m%d_%H%M%S)_시뮬_$NAME"; mkdir -p "$OUT"; LOG="$OUT/sim.log"
SPEED=${SPEED:-full}
DEMO_PARAMS="$DEMO/params_demo.yaml"; FILTER="$DEMO/v2_filter.py"
if [ "$SPEED" = max ]; then   # 11-16 경기 max 모드 (바탕화면 demo_real: 직진 0.45 · RPP 곡선 감속 반경을 속도에 맞춤 · 필터 직진 상한 max(vx, 0.30))
  VF=${MAX_VF:-0.45}; VR=0.15; SW=0.5; MAXD=${MAX_DIR:-$HOME/Desktop/CartRider/2026-11-14_규칙v2계획/demo_real}
  DEMO_PARAMS="$MAXD/params_demo_max.yaml"; FILTER="$MAXD/v2_filter.py"
elif [ "$SPEED" = full ]; then VF=0.30; VR=0.15; SW=0.5; else VF=0.20; VR=0.10; SW=0.4; fi
MRF=$(python3 -c "print(round($VF * 1.1585 / 0.201, 2))"); MRR=$(python3 -c "print(round($VR * 1.1585 / 0.201, 2))")
ROUTE="${ROUTE:-box1 box2 box6 box3 box5 box4 L3 L2 L1 HOME}"
{ echo "이름: $NAME"; echo "시각: $(date '+%F %T')"; echo "시나리오: $SC"; echo "경로: HOME → $ROUTE"; echo "속도: $SPEED (전진 $VF · 후진 $VR · 제자리 $SW)"
  echo "장애물 멈춤: $([ -n "$NO_AVOID" ] && echo 끔 || echo 켬)"; echo "GUI=${GUI:-0}"
  echo "추가 인자: Gazebo 장애물 시나리오 $(basename "$SC") · 경기 도구 ($SPEED) RPP 전진 $VF · 후진 $VR · Spin · 규칙 v2 필터 (벨트 2.0 · 꺾는 중 모터 ≤ 3.0) · 장애물 감시 · 정지 관문"
  echo "감시 인자: ${GUARD_ARGS:-기본 (원래 스캔 · 차체 상자 앞 0.16 · 0.75 m 안 한 스캔 · TFmini)}"
  echo "저장소: drive_avoid (sim/run_sim.sh)"; } > "$OUT/settings.txt"
[ "$SC" != none ] && cp "$SC" "$OUT/"

PIDS=(); SCN=""
start() { setsid "$@" >> "$LOG" 2>&1 < /dev/null & PIDS+=($!); }
cleanup() {
  [ -n "$SCN" ] && kill -INT -- -"$SCN" 2>/dev/null && sleep 3
  for p in "${PIDS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 3
  for p in "${PIDS[@]}"; do kill -KILL -- -"$p" 2>/dev/null; done
  stop_all
}
trap cleanup EXIT TERM

echo "▶ Gazebo (창 $([ "${GUI:-0}" = 1 ] && echo 있음 || echo 없음)) · 기록 $OUT"
start ros2 launch navigation gazebo_sim.launch.py nav:=false rviz:=false gui:=$([ "${GUI:-0}" = 1 ] && echo true || echo false)
for i in $(seq 1 90); do   # 팔 주행 자세 명령이 끝날 때까지 (gazebo_sim 의 bash 과정이 끝남)
  grep -q "bash-[0-9]*\]: process has finished" "$LOG" && break; sleep 1
done
grep -q "bash-[0-9]*\]: process has finished" "$LOG" || { echo "✗ Gazebo · 팔 자세가 90 s 안에 안 끝남 (sim.log)"; exit 1; }
sleep 2
python3 "$HERE/scenario.py" --move-boxes --known none --out "$OUT" | tee -a "$LOG"
python3 "$HERE/make_known_map.py" "$MP" "$OUT/known_sim" | tee -a "$LOG"

echo "▶ 위치 추정 (AMCL · 스캔 합치기, Python 판)"
start ros2 launch localization localization.launch.py use_sim_time:=true use_amcl_node:=true use_loc_monitor:=false loc_cpp:=false \
  map:="$MP/map_amcl.yaml" objects_map:="$OUT/known_sim.yaml" loc_map:="$MP/map_amcl.yaml"
# (Gazebo 실제 자세는 scenario.py 가 gz-transport 로 바로 받음)

echo "▶ Nav2 (경기 도구 설정) · 규칙 v2 필터 · 정지 관문 · 장애물 감시"
sed -e "s/__VF__/$VF/g" -e "s/__VR__/$VR/g" -e "s/__MRF__/$MRF/g" -e "s/__MRR__/$MRR/g" -e "s#__ODOM__#/odometry/filtered#g" "$DEMO_PARAMS" > "$OUT/params.yml"
SIMP=(-p use_sim_time:=true)
start ros2 run nav2_controller controller_server --ros-args --params-file "$OUT/params.yml" "${SIMP[@]}" -r cmd_vel:=cmd_vel_raw
start ros2 run nav2_behaviors behavior_server --ros-args --params-file "$OUT/params.yml" "${SIMP[@]}" -r cmd_vel:=cmd_vel_raw
start python3 "$FILTER" --ros-args "${SIMP[@]}" -p out_topic:=cmd_vel_gate_in -p vx_fwd:=$VF -p vx_rev:=$VR -p spin_w:=$SW \
  -p belt:=2.0 -p turn_motor:=3.0 -p odom_topic:=/odometry/filtered -p rev_max:=0.40
start python3 "$TOOL/stop_gate.py" --ros-args "${SIMP[@]}" -p in_topic:=cmd_vel_gate_in -p out_topic:=/cmd_vel
start python3 "$TOOL/obstacle_guard.py" --ros-args "${SIMP[@]}" -p known_map:="$OUT/known_sim.yaml" ${GUARD_ARGS:-}
start ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_demo "${SIMP[@]}" \
  -p autostart:=true -p bond_timeout:=0.0 -p node_names:="['controller_server', 'behavior_server']"
sleep 6
ensure_active() {
  local n st k
  for n in "$@"; do
    for k in $(seq 1 25); do
      st=$(timeout 8 ros2 lifecycle get "/$n" 2>/dev/null | awk '{print $1}')
      case "$st" in
        active) echo "  ✓ $n active"; break ;;
        unconfigured) timeout 20 ros2 lifecycle set "/$n" configure >> "$LOG" 2>&1 ;;
        inactive) timeout 20 ros2 lifecycle set "/$n" activate >> "$LOG" 2>&1 ;;
        *) sleep 1 ;;
      esac
      [ "$k" = 25 ] && echo "  ✗ $n 이 active 가 안 됨 (지금 ${st:-응답 없음})"
    done
  done
}
ensure_active map_server amcl controller_server behavior_server

setsid python3 "$HERE/scenario.py" --scenario "$SC" --known "$OUT/known_sim.yaml" --out "$OUT" > "$OUT/scenario.log" 2>&1 < /dev/null & SCN=$!
[ -n "${REC:-}" ] && start ros2 bag record -o "$OUT/bag" --topics /imu/data /avoid/status /avoid/blocked /scan_front /scan_rear /scan_merged /tf /tf_static /odometry/filtered /cmd_vel /tfmini/range /tfmini/scan /joint_states   # 진단용 녹화 (REC=1)
echo "▶ 주행 (경로 HOME → $ROUTE)"
python3 "$TOOL/avoid_runner.py" --gz --route "$ROUTE" --out "$OUT" --dwell "${DWELL:-1.0}" ${NO_AVOID:+--no-avoid} 2>&1 | (trap '' INT; tee "$OUT/run.log")
kill -INT -- -"$SCN" 2>/dev/null; sleep 4; SCN=""
echo "──────── 결과"
cat "$OUT/summary.txt" "$OUT/truth_summary.txt" 2>/dev/null
grep "\[시나리오\]" "$OUT/scenario.log" 2>/dev/null
echo "기록: $OUT"
}
