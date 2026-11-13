#!/usr/bin/env bash
# teleop 처럼 팔 스킬 하나씩 (2026-11-13). 시뮬 · 실물 같은 명령:
#   arm_cmd.sh <sim|real> pick                       상자 위 퍽 45° 집기 (toppick_tilt — 실물은 노드 설정 skill.toppick_tilt_policy 의 정책)
#   arm_cmd.sh <sim|real> load <열> <칸>              쥔 퍽을 팔레트 칸에 (pallet_load_<열>_<칸>_tilt), 예: load 3 1
#   arm_cmd.sh <sim|real> unload <열> <칸>            칸에서 꺼냄 (pallet_unload_<열>_<칸>_tilt)
#   arm_cmd.sh <sim|real> place <x> <y> <z>           쥔 퍽을 base_link (x, y) 에, z = 놓을 윗면 높이 (place_at:base_link:x:y:z)
#   arm_cmd.sh real look <x> <y> <z>                 실물: 퍽 예상 중심 (base_link) 을 팔 카메라로 내려다봄 (target_look, pick 전에 필요)
#   arm_cmd.sh real place_cell <L> <dx> <dy>          실물: 장소 L 칸 (dx, dy = m1 · 0 · p1) 에 놓기 (place_<L>_<dx>_<dy>_tilt, 지도 TF 필요)
#   arm_cmd.sh <sim|real> raw <스킬 이름>              그대로 보냄
#   시뮬 전용: transit · reset · cells (상자 위 3x3 칸 좌표) · status (퍽 자리) · clear (상자 위 퍽 치움) · fill <열_칸> ... (팔레트에 퍽 바로 채우기, 예 fill 3_0 3_3 2_1)
# 실물 (real) 은 젯슨에서 (bringup skill_enabled:=true 상태): ros2 action send_goal /manipulator/execute_skill
set -u
MODE="${1:-}"; shift || true; C="${1:-}"; shift || true
case "$C" in
  pick) S="toppick_tilt" ;;
  load) S="pallet_load_${1}_${2}_tilt" ;;
  unload) S="pallet_unload_${1}_${2}_tilt" ;;
  place) S="place_at:base_link:${1}:${2}:${3}" ;;
  look) S="target_look:base_link:${1}:${2}:${3}" ;;
  place_cell) S="place_${1}_${2}_${3}_tilt" ;;
  raw) S="$1" ;;
  transit|reset|cells|status|clear) S="$C" ;;
  fill) S="fill $*" ;;
  *) sed -n 2,12p "$0"; exit 1 ;;
esac
T="${ARM_TIMEOUT:-120}"
if [ "$MODE" = sim ]; then
  set +u; source "$HOME/ros2_lyrical/install/setup.bash" 2>/dev/null; set -u
  exec python3 "$(cd "$(dirname "$0")" && pwd)/sim_cmd.py" "$S" "$T"
elif [ "$MODE" = real ]; then
  set +u; [ -n "${ROS_WS:-}" ] && source "$ROS_WS/install/setup.bash"; set -u
  if [ "$C" = pick ] || [ "$C" = look ]; then ros2 service call /perception/set_mode robot_interfaces/srv/SetPerceptionMode "{mode: 4}" > /dev/null 2>&1 || true; fi   # 인식 켜기 (#190 실물 시험 메모)
  exec ros2 action send_goal /manipulator/execute_skill robot_interfaces/action/ExecuteSkill "{policy: '$S', timeout: $T}" --feedback
else
  sed -n 2,12p "$0"; exit 1
fi
