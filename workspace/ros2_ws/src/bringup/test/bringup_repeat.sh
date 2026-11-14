#!/usr/bin/env bash
# bringup 을 N 번 띄우고 내리며 컨트롤러가 다 뜨는지 센다 (#275 확인용, Jetson 실물).
#
#   bringup/test/bringup_repeat.sh [횟수] [-- bringup 인자 …]
#   예) bringup/test/bringup_repeat.sh 10 -- use_teleop:=false use_localization:=true use_nav2:=true
#
# 매번: bringup.launch.py 를 띄움 → WAIT_S (기본 60) 초 뒤
#   ① ros2 control list_controllers 의 active 수 (EXPECT, 기본 4 = joint_state · imu · diff_drive · arm. gripper 를 쓰면 5)
#   ② /joint_states 가 TOPIC_S (기본 10) 초 안에 오는지
#   를 보고 내림 (INT → TERM → KILL). 둘 다 되면 성공.
# 결과: OUT (기본 ~/real_drive/bringup_repeat/<YYMMDD_HHMM>) 에 판마다 launch 로그 · summary.txt
#   (성공 수, 판마다 active 수 · /joint_states · spawner 오류 줄 수).
# 인지 모드를 켠 부하로 재려면 다른 터미널에서 인지 모드를 켜 두거나 bringup 인자로 넘길 것 (부하가 실패와 관련, #275).
# 먼저: source install/setup.bash. 다른 bringup 이 떠 있으면 안 됨 (컨트롤러 이름이 겹침).
set -u
N=10
if [ $# -gt 0 ] && [ "$1" != "--" ]; then N=$1; shift; fi
[ $# -gt 0 ] && [ "$1" = "--" ] && shift
ARGS=("$@")
WAIT_S=${WAIT_S:-60}
TOPIC_S=${TOPIC_S:-10}
EXPECT=${EXPECT:-4}
OUT=${OUT:-$HOME/real_drive/bringup_repeat/$(date +%y%m%d_%H%M)}
mkdir -p "$OUT"
echo "bringup ${N} 번, 인자: ${ARGS[*]:-(없음)}, ${WAIT_S} s 뒤 active ${EXPECT} 개 기대 -> $OUT" | tee "$OUT/summary.txt"

ok=0
for k in $(seq 1 "$N"); do
  log="$OUT/bringup_$k.log"
  setsid ros2 launch bringup bringup.launch.py "${ARGS[@]}" > "$log" 2>&1 &
  pid=$!
  sleep "$WAIT_S"
  active=$(timeout 30 ros2 control list_controllers 2>/dev/null | grep -c " active")
  if timeout "$TOPIC_S" ros2 topic echo --once /joint_states sensor_msgs/msg/JointState > /dev/null 2>&1; then js=yes; else js=no; fi
  lock=$(grep -c "Failed to acquire lock" "$log")
  contact=$(grep -c "Could not contact service" "$log")
  hw=$(grep -c "Successful 'activate' of hardware" "$log")
  if [ "$active" -ge "$EXPECT" ] && [ "$js" = yes ]; then res=성공; ok=$((ok + 1)); else res=실패; fi
  printf '%2d %s  active %s/%s  /joint_states %s  하드웨어 activate %s  잠금 실패 %s  서비스 실패 %s\n' \
    "$k" "$res" "$active" "$EXPECT" "$js" "$hw" "$lock" "$contact" | tee -a "$OUT/summary.txt"
  # 내리기: launch 프로세스 묶음 전체 (setsid 로 따로 묶음)
  for sig in INT TERM KILL; do
    kill -"$sig" -- -"$pid" 2>/dev/null || break
    for _ in $(seq 1 15); do kill -0 -- -"$pid" 2>/dev/null || break 2; sleep 1; done
  done
  sleep 5
done
echo "성공 $ok / $N" | tee -a "$OUT/summary.txt"
[ "$ok" -eq "$N" ]
