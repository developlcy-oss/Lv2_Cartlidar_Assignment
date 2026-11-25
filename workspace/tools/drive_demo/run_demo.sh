#!/usr/bin/env bash
# 규칙 v2 계획 시연 — Nav2 (controller_server RPP · behavior_server Spin) + 규칙 v2 필터 + 계획 따라가기. 저장소 파일은 안 고침.
#   ./run_demo.sh real [slow|full] [step] [init] [구간 예 0-0]     Jetson (로봇): 로봇 쪽 launch 3 개가 떠 있어야 함 (시연_가이드.md)
#   ./run_demo.sh sim  [slow|full] [step] [구간]                   노트북: loopback 시뮬로 같은 것을 미리 확인 (Gazebo 없음)
#     slow = 전진 0.20 · 후진 0.10 m/s · 제자리 0.4 rad/s (첫 실물 권장)   full = 0.30 · 0.15 · 0.5 (시뮬 18:46 과 같음)
#     max  = 전진 MAX_VF (기본 0.45 = diff_drive 한계) · 후진 0.15 · 제자리 0.5 + 벨트 2.0 · 꺾는 중 모터 ≤ 5.5 (11-18) · 정지 앞에서만 감속 · params_demo_max.yaml (11-16, 주행만 최고 속도)
#     step = 구간마다 Enter 로 출발 (q = 끝)   init = 출발 자세를 /initialpose 로 한 번 (로봇을 출발 자리에 정확히 놓았을 때)
#     구간 = 그 구간만 (예 0-0 = 첫 구간 signal → box1 만, 2-4)
#   ./run_demo.sh real|sim slow|full|max auto [init] [step]       11-21 완전 자율: 출발 → box1·2·6·3·5·4 → L3·2·1 → 출발 자세 복귀 (정지점 3 s, 실패 곳 건너뜀 · 연속 2 곳 실패면 복귀)
#     auto 환경 변수: AUTO_ORDER=plan|opt (계획 demo_plan_auto.json | demo_plan_auto_opt.json, PLAN= 이 우선) · AUTO_DWELL=3 · AUTO_FROM=box3 (그곳부터 재개)
#                     AUTO_FAIL_HOME=2 · HOME_TOL=0.03,5 (복귀 허용 m,°) — 빈 값 = demo_follow 기본값
#   환경 변수: SAFETY=0 (안전 끔: collision_monitor · 라이다 장애물 층 · 돌아갈 길 planner, 기본 1) · NO_RECOVER=1 (되찾기 끔) · SIM_TURN=0.79,0.70 (시뮬만, 실물처럼 덜 돎) · RUN_TAG= · REV_MAX= · OFF_PATH= · PLAN= · PARAMS= · SIM_DOMAIN= · SIM_MAP=
#             BOX_APPROACH=1|0|dry (11-21 상자 앞 5 cm 다가가기, 기본 1 · dry = 재기만) · BOX_APPROACH_AT=box1,L2 (그곳만, 빈 값 = 상자 9 곳) · BOX_APPROACH_LOOSE=1 (잡음 큰 라이다용 느슨한 판정)
#             AP_RETURN=1|0 (5 cm 뒤 후진 틈 ≥ 0.12 m → 다음 길에 이음 켬 · 끔) · AP_CUT_YAW=20 (끊는 점 ↔ 정지점 방향 차 한도 °, 넘으면 도착 뒤 후진 측정 · 0 = 기존) — auto 아니어도 넘김, 빈 값 = demo_follow 기본값
#             DEMO_BAG=0 (rosbag 기록 끔, 기본 1 → 기록 폴더/bag) · DEMO_BAG_PARTICLES=0 (/particle_cloud 빼기)
#             FRONT_GUARD=0 (11-21 상자 구간 앞 라이다 원본 정지 소스 끔 — 차체 반사로 잘못 멈출 때)
#             11-22: AP_MEASURE_GAP=14.5 (상자 측정 자리 = 측정 때 앞 끝 ↔ 상자 틈 cm, 틈 ≤ 20 cm 면 S 자 한 번으로 5 cm)
#             11-21 (3): AP_MID_FIX=2 (0.20 m 자리 S 자 옆 보정 최대 횟수, 0 = 확인만) · AP_MID_TOL=8 (허용 옆 오차 mm)
#                        TURN_DIRECT=5 (이 각 ° 이하 제자리는 Spin 대신 cmd_vel 직접, 0 = 늘 Spin) · TURN_ROOM=1|0 (상자 앞 Spin 전 틈 확보 후진)
#                        AP_LEAVE_ALL=1|0 (다가가기 안 된 상자 정지점도 떼기 후진) — 빈 값 = demo_follow 기본값 (위 값)
#             11-22 밤: ARM_LOOK=on|dry|0 — 팔 카메라 (Arducam) 로 상자 윗면을 잼. **기본 on (켜짐)**, 0 = 끔. 서비스 (/manipulator/pose · 인식 place_box) 가 안 보이면 (시뮬 · 인식 안 뜸) 자동으로 꺼짐 (라이다만).
#                        ARM_LOOK_POSE=BOX_VIEW (기본) = 자율 시작 전 (출발 확인 뒤) 팔 DRIVE → BOX_VIEW 한 번, **주행 내내 유지**, 끝 (정상 · Ctrl+C · 실패) 에 DRIVE 로 접음. 안전 정지 때는 팔 안 움직임 (안내만).
#                        상자 측정 자리 (틈 14.5 cm, 허용 10~20 cm) 에서는 팔을 안 움직이고 라이다 측정 + 팔 카메라 재기만 (윗면 일부만 보임). 못 펴면 경고 후 라이다만.
#                        ⚠ 팔 끝이 라이다 높이 위라 벽 (39 cm) · 높은 물체 접촉을 아무 장치도 못 봄 → 첫 실물은 step 모드로 S4 · S2 에서 팔–벽을 눈으로 확인.
#                        ⚠ on 의 3 cm · 3° 문턱은 실물 미검증 — 성공하던 상자에서도 다가가기를 포기할 수 있음 (dry 로 먼저 값 확인 가능).
#                        ARM_LOOK_POSE=ALIGN = 틈 30 cm 1 차 정지, 허용 20~65 cm — 측정 때만 펴고 접은 뒤 틈 14.5 cm 라이다와 비교 (예전 동작).
#                        dry = 재서 기록만 (움직임엔 안 씀) · on = 3 cm · 3° 넘게 다르면 5 cm 다가가기 포기 · ARM_LOOK_SOLO=1 (on 에서 라이다 실패 곳을 팔 값 단독으로 다가감, 기본 0 — 위험 README).
#                        인식 place_box 가 떠 있어야 함 (bringup arm_look.launch.py, README)
#             11-22 밤 v2: SPIN_MODE=span_cap|sum|span (v2_filter 제자리 예산 방식, 기본 span_cap = 방향 범위 ≤ 90° + 왕복 총량 ≤ SPIN_TOTAL_DEG, 사용자 결정 11-22 밤 · sum = 예전 규칙) · SPIN_TOTAL_DEG=180
#   11-21 상자 구간 (정지점 0.40 m 안): 정지 띠 · HOLD 는 그 상자 점만 지운 라이다 (box_scan_guard.py, SAFETY=1) · 0.30 m 안 속도 ≤ 0.15 m/s
# 멈추기: 이 터미널 Ctrl+C (Nav2 동작 취소 → 0) · 리모컨 1 (E-stop, 풀기 2+3) — 소프트웨어 정지, 물리 비상정지 아님
# 기록: 이 폴더 runs/<월일_시각>_<실물|시뮬>_<speed>/ → 노트북 2026-11-12_기준주행/runs/ 로 복사하면 주행기록 HTML 에 Nav2 기록으로 보임
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
BELT="${BELT:-}"; TURN_MOTOR="${TURN_MOTOR:-}"   # 11-15: 필터의 벨트 기어비 · 꺾는 중 모터 상한 (빈 값 = 1.5 · 5.5 (11-18 arc_test), 예 BELT=2.0 TURN_MOTOR=2.6)
PLAN_ENV="${PLAN:-}"; PLAN="${PLAN:-$HERE/demo_plan.json}"   # 11-15: 다른 계획 시험 (PLAN=… ./run_demo.sh sim slow) · PARAMS= 로 Nav2 설정 · SIM_DOMAIN= 로 시뮬 ROS_DOMAIN_ID
SAFETY="${SAFETY:-1}"   # 11-19: 1 = params_demo_safety.yaml 덧씌우기 + planner_server (Detour) · collision_monitor (v2_filter 뒤) · 0 = 예전 그대로
BOX_APPROACH="${BOX_APPROACH:-1}"; BOX_APPROACH_AT="${BOX_APPROACH_AT:-}"; BOX_APPROACH_LOOSE="${BOX_APPROACH_LOOSE:-0}"   # LOOSE=1 → --box-loose · 11-21: 상자 앞 다가가기 (1 = 켬 · 0 = 끔 · dry = /scan_front 로 재고 로그만) · 그곳만 (예 box1,L2)
case "$BOX_APPROACH" in 1) BA=on; BA_TAG="" ;; 0) BA=off; BA_TAG=_접근끔 ;; dry) BA=dry; BA_TAG=_접근측정 ;; *) echo "BOX_APPROACH 는 1 · 0 · dry 중 하나 (지금 $BOX_APPROACH)"; exit 2 ;; esac
DEMO_BAG="${DEMO_BAG:-1}"; DEMO_BAG_PARTICLES="${DEMO_BAG_PARTICLES:-1}"   # 11-21: rosbag 기록 (위치 튐 뒤 분석) · /particle_cloud 는 크면 0 으로 뺌
SIM_TURN="${SIM_TURN:-}"   # 11-16: 시뮬만 — 실물처럼 덜 도는 로봇 (예 SIM_TURN=0.79,0.70 = 꺾기 79 % · 제자리 70 %, sim_turn_scale.py)
MODE=""; SPEED=slow; STEP=""; INIT=""; LEGS=""; AUTO=""
for a in "$@"; do
  case "$a" in
    real|sim) MODE=$a ;; slow|full|max) SPEED=$a ;; step) STEP=--step ;; init) INIT=--set-initial ;; auto) AUTO=--auto ;;
    [0-9]*) LEGS=$a ;; *) echo "모르는 인자: $a"; exit 2 ;;
  esac
done
[ -n "$MODE" ] || { sed -n 2,18p "$0"; exit 2; }
[ -n "$AUTO" ] && [ -n "$LEGS" ] && { echo "auto 에서는 구간 ($LEGS) 대신 AUTO_FROM=<곳 이름> 으로 재개 (예 AUTO_FROM=box3)"; exit 2; }
if [ "$SPEED" = full ]; then VF=0.30; VR=0.15; SW=0.5; else VF=0.20; VR=0.10; SW=0.4; fi
if [ "$SPEED" = max ]; then   # 11-16: 주행만 최고 속도 — 규칙 v2 (제자리 0.5 · 후진 0.15 · 반경 0.20 · 꺾는 중 모터 3.0) 는 그대로, 직진만 빠르게
  VF=${MAX_VF:-0.45}; VR=0.15; SW=0.5; BELT=${BELT:-2.0}; TURN_MOTOR=${TURN_MOTOR:-5.5}; PARAMS=${PARAMS:-$HERE/params_demo_max.yaml}
  [ -z "$PLAN_ENV" ] && [ -f "$HERE/demo_plan_max.json" ] && PLAN=$HERE/demo_plan_max.json   # 같은 계획 + 끝 (출발 구역) 방향 90°
  [ -z "$AUTO" ] && export DEMO_AUTO_SKIP=${DEMO_AUTO_SKIP:-1}   # step 없이 돌 때: 되찾기 2 번도 안 되면 그 조각 건너뛰고 끝까지 (멈추지 않음)
fi
if [ -n "$AUTO" ]; then   # 11-21 완전 자율
  # DEMO_AUTO_SKIP (조각 건너뛰기) 은 auto 에서 끔: 조각을 건너뛰면 그 구간이 '성공' 으로 끝나 엉뚱한 자리에서 다음 곳으로 감.
  #   auto 는 되찾기까지 실패한 구간을 '그 곳 실패' 로 보고 demo_follow --auto 가 건너뛰기 · 연속 실패 복귀를 맡음 (실패 처리는 한 곳에서만)
  [ "${DEMO_AUTO_SKIP:-}" = 1 ] && echo "⚠ auto 에서는 DEMO_AUTO_SKIP=1 을 무시 (실패 처리는 demo_follow --auto)"
  unset DEMO_AUTO_SKIP
  if [ -z "$PLAN_ENV" ]; then
    case "${AUTO_ORDER:-plan}" in plan) PLAN=$HERE/demo_plan_auto.json ;; opt) PLAN=$HERE/demo_plan_auto_opt.json ;;
      *) echo "AUTO_ORDER 는 plan · opt 중 하나 (지금 $AUTO_ORDER)"; exit 2 ;; esac
    [ -f "$PLAN" ] || { echo "✗ $(basename "$PLAN") 없음 — 먼저 만들기 ($([ "${AUTO_ORDER:-plan}" = opt ] && echo '노트북에서 수십 분' || echo '노트북 약 15~20분')):"
      echo "  cd $HERE && python3 auto_order.py --base demo_plan_wide.json --legs legs.json --order ${AUTO_ORDER:-plan} --home-pose 0.6936,0.95,180 --home-legs --home-face 120 --deadline 120 --out $(basename "$PLAN")"; exit 2; }
  fi
fi
FA=()   # demo_follow 추가 인자 (11-21): 환경 변수를 지정했을 때만 넘김 → 안 주면 demo_follow 기본값
if [ -n "$AUTO" ]; then FA+=(--auto)
  [ -n "${AUTO_DWELL:-}" ] && FA+=(--dwell "$AUTO_DWELL"); [ -n "${AUTO_FROM:-}" ] && FA+=(--from "$AUTO_FROM")
  [ -n "${AUTO_FAIL_HOME:-}" ] && FA+=(--auto-fail-home "$AUTO_FAIL_HOME"); [ -n "${HOME_TOL:-}" ] && FA+=(--home-tol "$HOME_TOL")
fi
case "${AP_RETURN:-}" in 1) FA+=(--ap-return-stop) ;; 0) FA+=(--no-ap-return-stop) ;; "") ;; *) echo "AP_RETURN 은 1 · 0 중 하나 (지금 $AP_RETURN)"; exit 2 ;; esac
[ -n "${AP_CUT_YAW:-}" ] && FA+=(--ap-cut-yaw "$AP_CUT_YAW")   # box_approach 공통 (auto 아니어도)
# 11-21 (3) S 자 옆 보정 · 상자 앞 Spin "Collision Ahead" 대책 (지정했을 때만 넘김)
[ -n "${AP_MEASURE_GAP:-}" ] && FA+=(--ap-measure-gap "$AP_MEASURE_GAP")   # 11-22 측정 틈 [cm] (기본 14.5)
[ -n "${AP_MID_FIX:-}" ] && FA+=(--ap-mid-fix "$AP_MID_FIX"); [ -n "${AP_MID_TOL:-}" ] && FA+=(--ap-mid-tol "$AP_MID_TOL")
[ -n "${TURN_DIRECT:-}" ] && FA+=(--turn-direct "$TURN_DIRECT")
case "${TURN_ROOM:-}" in 1) FA+=(--turn-room) ;; 0) FA+=(--no-turn-room) ;; "") ;; *) echo "TURN_ROOM 은 1 · 0 중 하나 (지금 $TURN_ROOM)"; exit 2 ;; esac
case "${AP_LEAVE_ALL:-}" in 1) FA+=(--ap-leave-all) ;; 0) FA+=(--no-ap-leave-all) ;; "") ;; *) echo "AP_LEAVE_ALL 은 1 · 0 중 하나 (지금 $AP_LEAVE_ALL)"; exit 2 ;; esac
case "${ARM_LOOK:-on}" in 1|on) FA+=(--arm-look on); [ "${ARM_LOOK_SOLO:-0}" = 1 ] && FA+=(--arm-solo) ;; dry) FA+=(--arm-look dry) ;; 0) ;; *) echo "ARM_LOOK 은 0 · dry · on 중 하나 (지금 $ARM_LOOK)"; exit 2 ;; esac
case "${ARM_LOOK_POSE:-BOX_VIEW}" in BOX_VIEW|ALIGN) FA+=(--arm-pose "${ARM_LOOK_POSE:-BOX_VIEW}") ;; *) echo "ARM_LOOK_POSE 는 BOX_VIEW · ALIGN 중 하나 (지금 $ARM_LOOK_POSE)"; exit 2 ;; esac   # 11-22 밤: 팔 카메라 측정 자세 (ARM_LOOK 켠 때만 쓰임)
# RPP 곡선 감속 반경 = 그 속도로 반경 1 m 를 돌 때 바깥 모터가 상한 (벨트 2.0 · 모터 3.0 → 0.201 m/s) 이 되는 값 (params_demo_max.yaml 의 __MRF__ · __MRR__)
MRF=$(python3 -c "print(round($VF * 1.1585 / 0.201, 2))"); MRR=$(python3 -c "print(round($VR * 1.1585 / 0.201, 2))")
set +u; source /opt/ros/lyrical/setup.bash; set -u
for d in "$(cd "$HERE/../../.." 2>/dev/null && pwd)" "$HOME/Lv2_Cartlidar_Assignment" "$HOME/GIT/lv2_CartLidar/Lv2_Cartlidar_Assignment"; do   # 첫 후보 = 이 폴더가 든 저장소 (workspace/tools/drive_demo)
  [ -f "$d/workspace/ros2_ws/install/setup.bash" ] && { set +u; source "$d/workspace/ros2_ws/install/setup.bash"; set -u; REPO=$d; break; }
done
OUT="$HERE/runs/$(date +%m%d_%H%M)_$([ $MODE = real ] && echo 실물 || echo 시뮬)_$SPEED${AUTO:+_자동}${BELT:+_벨트$BELT}${TURN_MOTOR:+_모터$TURN_MOTOR}${SIM_TURN:+_실물형회전}${LEGS:+_구간$LEGS}$BA_TAG${RUN_TAG:+_$RUN_TAG}"   # 11-15: RUN_TAG= (같은 분에 여러 시뮬)
mkdir -p "$OUT"
LOG="$OUT/nav.log"
ODOM=/odometry/filtered; SIMP=(); OUTT=cmd_vel_nav; SCAN=/scan_merged; FRONT=/scan_front   # FRONT = 상자 구간 가까운 곳 정지에 쓰는 앞 라이다 원본 (11-21)
if [ $MODE = sim ]; then
  export ROS_DOMAIN_ID=${SIM_DOMAIN:-73} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  ODOM=/odom; SIMP=(-p use_sim_time:=true); OUTT=cmd_vel; SCAN=/scan   # loopback_simulator 라이다 = scan
  [ -n "$SIM_TURN" ] && OUTT=cmd_vel_turn   # 필터 → sim_turn_scale → cmd_vel
else   # 실물: 로봇 쪽과 같은 ROS 설정 그대로. 이미 Nav2 가 떠 있으면 이름이 겹침
  if ros2 node list 2>/dev/null | grep -qE "^/(controller_server|behavior_server|planner_server|collision_monitor|velocity_smoother|v2_filter)$"; then
    echo "✗ controller_server · behavior_server · planner_server · collision_monitor · velocity_smoother · v2_filter 중 이미 있음 (bringup mode:=drive · use_nav2:=false 인지, 이 스크립트가 두 번 떴는지 확인)"; exit 1; fi
  # 11-19: bringup 이 /system/mode (transient_local) 를 냄. mission 이면 미션 Nav2 · 감시가 같이 떠 있어 시연과 겹침. 토픽이 없으면 (옛 bringup) 경고만
  SYSMODE=$(timeout 5 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /system/mode std_msgs/msg/String 2>/dev/null | sed -n "s/^data: *['\"]\{0,1\}\([a-z_]*\).*/\1/p")
  if [ "$SYSMODE" = mission ]; then echo "✗ 미션 모드 bringup 이 떠 있음 — mode:=drive 로 띄울 것"; exit 1; fi
  [ -z "$SYSMODE" ] && echo "⚠ /system/mode 를 못 받음 (옛 bringup?) — mode:=drive 인지 직접 확인"
  ros2 topic info /cmd_vel_nav 2>/dev/null | grep -q "Subscription count: [1-9]" || echo "⚠ /cmd_vel_nav 를 받는 쪽 (twist_mux) 이 안 보임 — base_bringup 이 떠 있는지 (터미널 1)"
fi
sed -e "s/__VF__/$VF/g" -e "s/__VR__/$VR/g" -e "s/__MRF__/$MRF/g" -e "s/__MRR__/$MRR/g" -e "s#__ODOM__#$ODOM#g" "${PARAMS:-$HERE/params_demo.yaml}" > "$OUT/params.yml"
PF=(--params-file "$OUT/params.yml")   # Nav2 서버 설정 (SAFETY=1 이면 safety.yml 을 뒤에 덧씌움)
if [ "$SAFETY" = 1 ]; then sed -e "s#__SCAN__#$SCAN#g" "$HERE/params_demo_safety.yaml" > "$OUT/safety.yml"; PF+=(--params-file "$OUT/safety.yml"); fi
PIDS=()
start() { setsid "$@" >> "$LOG" 2>&1 < /dev/null & PIDS+=($!); }
BAG_PID=""
stop_bag() {   # 11-21: ros2 bag record 는 SIGINT 로 닫아야 파일 끝 (mcap 요약 · metadata.yaml) 이 남음 → 최대 10 s 기다림
  [ -n "$BAG_PID" ] || return 0
  kill -INT -- -"$BAG_PID" 2>/dev/null
  for k in $(seq 1 20); do kill -0 "$BAG_PID" 2>/dev/null || break; sleep 0.5; done
  kill -KILL -- -"$BAG_PID" 2>/dev/null; BAG_PID=""
}
cleanup() { stop_bag; for p in "${PIDS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 2; for p in "${PIDS[@]}"; do kill -KILL -- -"$p" 2>/dev/null; done; }
trap cleanup EXIT TERM
if [ "$DEMO_BAG" = 1 ]; then   # 11-21: 기록 폴더/bag 에 rosbag (백그라운드, 로그 bag.log). 없는 토픽은 생기면 그때부터 기록
  BT=(/amcl_pose /odometry/filtered /scan_merged /scan_front /tf /tf_static /loc_guard/pose /loc_guard/event /cmd_vel_raw /cmd_vel /odom_icr /imu/data /diagnostics /scan_guard /scan_front_guard /drive_demo/box_zone /drive_demo/speed_cap /collision_monitor_state /perception/place_box/status /perception/place_box/prior /rosout)   # 11-23: 팔 측정 진단용 3 개 추가
  [ "$DEMO_BAG_PARTICLES" = 0 ] || BT+=(/particle_cloud)
  BOPT=(); BFMT=sqlite3
  BH=$(ros2 bag record --help 2>&1)
  [ $MODE = sim ] && { BT+=(/odom /scan /clock); grep -q -- "--use-sim-time" <<<"$BH" && BOPT+=(--use-sim-time); }   # 시뮬 = sim time 기준
  if grep -q "mcap" <<<"$BH"; then BFMT=mcap; BOPT+=(-s mcap); grep -q zstd_fast <<<"$BH" && BOPT+=(--storage-preset-profile zstd_fast)   # mcap 은 청크 zstd 압축
  else grep -q -- "--compression-mode" <<<"$BH" && BOPT+=(--compression-mode file --compression-format zstd); fi
  grep -q -- "--max-bag-size" <<<"$BH" && BOPT+=(-b 1073741824)   # 1 GiB 마다 파일 나눔 (Jetson 디스크)
  ENVSIG=(env --default-signal=INT); env --default-signal=INT true 2>/dev/null || ENVSIG=()   # & 로 띄운 프로세스는 SIGINT 무시로 시작 → 되살려야 stop_bag 의 SIGINT 가 먹음 (coreutils ≥ 8.31)
  setsid "${ENVSIG[@]}" ros2 bag record -o "$OUT/bag" "${BOPT[@]}" --topics "${BT[@]}" > "$OUT/bag.log" 2>&1 < /dev/null & BAG_PID=$!
fi

echo "규칙 v2 시연 ($MODE · $SPEED: 전진 $VF · 후진 $VR m/s · 제자리 $SW rad/s) · 기록 $OUT" | tee -a "$LOG"
echo "  상자 앞 다가가기: $([ $BA = on ] && echo '켬 (앞 끝 ↔ 상자 5 cm, 0.40 m 안 정지 띠 · HOLD 는 상자 점만 뺌 — 리모컨 1 준비)' || { [ $BA = dry ] && echo '재기만 (dry, 안 움직임)' || echo '끔 (BOX_APPROACH=0)'; })${BOX_APPROACH_AT:+ · $BOX_APPROACH_AT 만}$([ "$BOX_APPROACH_LOOSE" = 1 ] && echo ' · 느슨한 판정')" | tee -a "$LOG"
echo "  S 자 옆 보정 ${AP_MID_FIX:-2} 번 · 허용 ${AP_MID_TOL:-8} mm · 제자리 ≤ ${TURN_DIRECT:-5}° 직접 회전 · 상자 앞 Spin 전 틈 확보 $([ "${TURN_ROOM:-1}" = 1 ] && echo 켬 || echo 끔) · 안 된 곳도 떼기 $([ "${AP_LEAVE_ALL:-1}" = 1 ] && echo 켬 || echo 끔)" | tee -a "$LOG"
[ -n "$AUTO" ] && echo "  ⚠ 자율: 상자 0.40 m 안은 그 상자 점을 지운 라이다로 정지 띠 작동 (상자 둘레 5 cm 안 손·발은 못 봄) — 리모컨 1(E-stop)에 손 · 계획 $(basename "$PLAN")${FA[*]:+ · ${FA[*]}}" | tee -a "$LOG"
echo "  rosbag: $([ "$DEMO_BAG" = 1 ] && echo "$OUT/bag ($BFMT · 토픽 ${#BT[@]} 개$([ "$DEMO_BAG_PARTICLES" = 0 ] && echo ' · particle_cloud 뺌'))" || echo '끔 (DEMO_BAG=0)')" | tee -a "$LOG"
NODES="'controller_server', 'behavior_server'"
if [ $MODE = sim ]; then
  MAP="${SIM_MAP:-$REPO/workspace/ros2_ws/src/mission_planner/maps/map_amcl.yaml}"   # 11-15: SIM_MAP= 으로 다른 지도 (예 실측판)
  start ros2 run nav2_map_server map_server --ros-args --params-file "$OUT/params.yml" "${SIMP[@]}" -p yaml_filename:="$MAP"
  start ros2 run nav2_loopback_sim loopback_simulator --ros-args --params-file "$OUT/params.yml" "${SIMP[@]}"
  start ros2 run tf2_ros static_transform_publisher --x 0.0 --y 0.0 --z 0.0755 --frame-id base_link --child-frame-id base_scan
  NODES="'map_server', 'loopback_simulator', $NODES"
  [ -n "$SIM_TURN" ] && start python3 "$HERE/sim_turn_scale.py" --ros-args "${SIMP[@]}" -p k_curve:=${SIM_TURN%,*} -p k_spin:=${SIM_TURN#*,}
fi
start ros2 run nav2_controller controller_server --ros-args "${PF[@]}" "${SIMP[@]}" -r cmd_vel:=cmd_vel_raw
start ros2 run nav2_behaviors behavior_server --ros-args "${PF[@]}" "${SIMP[@]}" -r cmd_vel:=cmd_vel_raw
FOUT=$OUTT   # v2_filter 출력: SAFETY=1 이면 collision_monitor 를 거쳐 $OUTT 로
if [ "$SAFETY" = 1 ]; then
  FOUT=cmd_vel_cm_in
  start ros2 run nav2_planner planner_server --ros-args "${PF[@]}" "${SIMP[@]}"
  start ros2 run nav2_collision_monitor collision_monitor --ros-args --params-file "$OUT/safety.yml" "${SIMP[@]}" -p cmd_vel_out_topic:=$OUTT
  # 11-21: 상자 점만 지운 정지용 스캔 (/scan_guard · /scan_front_guard). 시뮬 · FRONT_GUARD=0 은 앞 원본 소스 없이 (늘 빈 스캔)
  start python3 "$HERE/box_scan_guard.py" --ros-args "${SIMP[@]}" -p merged_in:=$SCAN -p front_in:=$([ $MODE = real ] && [ "${FRONT_GUARD:-1}" != 0 ] && echo $FRONT || echo '""')
  NODES="$NODES, 'collision_monitor', 'planner_server'"   # planner_server 맨 뒤: global_costmap 이 map TF 를 기다림 (시뮬은 demo_follow 의 /initialpose 뒤에 생김)
fi
start python3 "$HERE/v2_filter.py" --ros-args "${SIMP[@]}" -p out_topic:=$FOUT -p vx_fwd:=$VF -p vx_rev:=$VR -p spin_w:=$SW \
  -p spin_mode:=${SPIN_MODE:-span_cap} -p spin_total_deg:=$(printf %.1f "${SPIN_TOTAL_DEG:-180}") -p belt:=$(printf %.3f "${BELT:-1.5}") -p turn_motor:=$(printf %.3f "${TURN_MOTOR:-5.5}") -p odom_topic:=$ODOM -p rev_max:=$(printf %.3f "${REV_MAX:-$([ $MODE = real ] && echo 0.50 || echo 0.40)}")
start ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_demo "${SIMP[@]}" \
  -p autostart:=true -p bond_timeout:=0.0 -p node_names:="[$NODES]"

{ echo "이름: 계획따라_$([ $MODE = real ] && echo 실물 || echo 시뮬)_$SPEED${AUTO:+_자동}${BELT:+_벨트$BELT}${TURN_MOTOR:+_모터$TURN_MOTOR}${SIM_TURN:+_실물형회전}${LEGS:+_구간$LEGS}$BA_TAG${RUN_TAG:+_$RUN_TAG}"; echo "시각: $(date '+%F %T')"
  echo "경로: $(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['course'])" "$PLAN")"; echo "GUI=0"
  echo "추가 인자: $([ $MODE = real ] && echo '실물 (AMCL · EKF)' || echo 'loopback 시뮬') · 계획 $(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['plan'])" "$PLAN") · RPP 전진 $VF · 후진 $VR · Spin · 규칙 v2 필터 (제자리 ≤ $SW · 벨트 ${BELT:-1.5} · 꺾는 중 모터 ≤ ${TURN_MOTOR:-5.5} · 이어진 후진 ≤ ${REV_MAX:-$([ $MODE = real ] && echo 0.50 || echo 0.40)} m)${PARAMS:+ · Nav2 설정 $(basename "$PARAMS")} · 안전 SAFETY=$SAFETY$([ "$SAFETY" = 1 ] && echo ' (collision_monitor · 라이다 장애물 층 · Detour)')${SIM_TURN:+ · 실물형 회전 (꺾기 × ${SIM_TURN%,*} · 제자리 × ${SIM_TURN#*,})} · 상자 앞 다가가기 BOX_APPROACH=$BOX_APPROACH${BOX_APPROACH_AT:+ ($BOX_APPROACH_AT 만)}$([ "$BOX_APPROACH_LOOSE" = 1 ] && echo ' · 느슨한 판정 (BOX_APPROACH_LOOSE=1)') · rosbag DEMO_BAG=$DEMO_BAG${AUTO:+ · 완전 자율 (AUTO_ORDER=${AUTO_ORDER:-plan})}${FA[*]:+ · demo_follow ${FA[*]}}"
  [ $MODE = real ] && echo "실물 벨트: ${HW_BELT:-2.0} (11-15 v7 44T → 22T, 다르면 HW_BELT=)"
  echo "저장소: Nav2 노드만 (시연 폴더 $(basename "$HERE"))"; } > "$OUT/settings.txt"

sleep 3
# 11-15 21:0x: Jetson 에서 lifecycle 응답이 시간 초과로 사라져 서버가 inactive 로 남는 일이 잦음 (map_server · behavior_server, #276 같은 DDS 지연)
#   → 출발 전에 직접 확인하고 unconfigured 면 configure, inactive 면 activate (로봇은 움직이지 않음 — 서버만 켬)
ensure_active() {
  local n st k
  for n in "$@"; do
    for k in $(seq 1 25); do
      st=$(timeout 8 ros2 lifecycle get "/$n" 2>/dev/null | awk '{print $1}')
      case "$st" in
        active) echo "  ✓ $n active" | tee -a "$LOG"; break ;;
        unconfigured) timeout 20 ros2 lifecycle set "/$n" configure >> "$LOG" 2>&1 ;;
        inactive) timeout 20 ros2 lifecycle set "/$n" activate >> "$LOG" 2>&1 ;;
        *) sleep 1 ;;
      esac
      if [ "$k" = 25 ]; then echo "  ✗ $n 이 active 가 안 됨 (지금 ${st:-응답 없음}) — 다른 터미널에서 ros2 lifecycle set /$n activate" | tee -a "$LOG"; fi
    done
  done
}
if [ $MODE = sim ]; then ensure_active map_server loopback_simulator controller_server behavior_server $([ "$SAFETY" = 1 ] && echo collision_monitor)
  [ "$SAFETY" = 1 ] && echo "  … planner_server 는 /initialpose 뒤 map TF 가 생기면 lifecycle_manager 가 켬" | tee -a "$LOG"
else ensure_active map_server controller_server behavior_server $([ "$SAFETY" = 1 ] && echo collision_monitor planner_server); fi
# 11-19: --scan-topic = 정지 래치가 직접 보는 라이다 (11-21: SAFETY=1 이면 상자 점 뺀 /scan_guard). SAFETY=0 이면 demo_follow 기본값 (/scan_merged)
if [ "$SPEED" = max ]; then echo "⚠ max: 상자 정지점 0.30 m 밖은 전진 $VF m/s · 안은 0.15 · 상자 점 뺀 라이다 정지 · TFmini 비상정지 없음 — 경로에 사람 없게 · 리모컨 1 에 엄지" | tee -a "$LOG"; fi
python3 "$HERE/demo_follow.py" --plan "$PLAN" --out "$OUT" $([ $MODE = sim ] && echo --sim) $STEP $INIT ${LEGS:+--legs "$LEGS"} ${OFF_PATH:+--off-path "$OFF_PATH"} ${NO_RECOVER:+--no-recover} $([ "$SAFETY" = 1 ] || echo --no-detour) $([ "$SAFETY" = 1 ] && echo --scan-topic /scan_guard) --box-approach $BA ${BOX_APPROACH_AT:+--box-approach-at "$BOX_APPROACH_AT"} $([ "$BOX_APPROACH_LOOSE" = 1 ] && echo --box-loose) --odom-topic "$ODOM" "${FA[@]}" 2>&1 | (trap '' INT; tee "$OUT/follow.log")   # 11-15: Ctrl+C 로 tee 가 먼저 죽으면 demo_follow 가 BrokenPipe 로 저장 못 함
cleanup; trap - EXIT
grep -h "잘림 횟수" "$LOG" | tail -1
[ "$DEMO_BAG" = 1 ] && { [ -f "$OUT/bag/metadata.yaml" ] && echo "rosbag: $OUT/bag ($(du -sh "$OUT/bag" | cut -f1))" || echo "⚠ rosbag 이 깔끔히 안 닫힘 — $OUT/bag.log 확인 (ros2 bag reindex $OUT/bag 로 살릴 수 있음)"; }
echo "기록: $OUT"
