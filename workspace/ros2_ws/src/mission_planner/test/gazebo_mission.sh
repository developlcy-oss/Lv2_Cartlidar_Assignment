#!/usr/bin/env bash
# Gazebo 주행 + mock (팔 · 인식 · 정렬 · 회전) 으로 plan 모드 미션 전체 시험 (2026-11-06).
#   진짜: Gazebo 경기장 (navigation gazebo_sim.launch.py: Nav2 + AMCL, SDF 지도), drive_node (/drive/navigate),
#         mission_node (plan 모드, 측량 끔), mission_planner_node (Q1 예제 등록표)
#   가짜: mock_robot.py --real_nav (manipulator · perception · /drive/align · /drive/execute_skill — drive_node 의 것은 remap 으로 비킴)
#
#   test/gazebo_mission.sh [결과 폴더]     기본 /tmp/gazebo_mission. 끝나면 결과 폴더/mock_out.json (goto 별 도착 자세 · 최종 상태)
#   test/gazebo_mission.sh stop            시험 프로세스 정리
#   GZ_ARGS="use_loc_monitor:=true" test/gazebo_mission.sh   gazebo_sim.launch.py 에 인자 더 주기 (#133)
# 먼저: source install/setup.bash, colcon build (mission_planner · mission · control · navigation · localization · waffle_4wd_description, navigation README 참고)
# 시각: 모든 로그 시각은 시뮬레이션 시각 (Gazebo 는 실시간의 약 0.5 ~ 0.66 배)
set +u
PAT='gz-sim-main|gz sim|gazebo_sim.launch|navigation.launch|localization.launch|nav2_|/amcl|lifecycle_manager|map_server|scan_merge|scan_body_mask|parameter_bridge|robot_state_publisher|tfmini_scan_to_range|camera_info_fix|lib/control/drive|mission/mission_node|planner.launch|mission_planner_node|mock_robot.py|collision_monitor|velocity_smoother|ros_gz_sim'
stop_all() {
  for sig in INT TERM KILL; do
    pids=$(pgrep -f "$PAT" | grep -vx "$$")
    [ -z "$pids" ] && return
    kill -$sig $pids 2>/dev/null; sleep 4
  done
}
if [ "$1" = "stop" ]; then stop_all; exit 0; fi
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="${1:-/tmp/gazebo_mission}"
MIS=$(ros2 pkg prefix mission)/share/mission/config
CTL=$(ros2 pkg prefix control)/share/control/config
MPL=$(ros2 pkg prefix mission_planner)/share/mission_planner/config
export GZ_IP=127.0.0.1   # Gazebo 통신을 이 PC 안으로 (샌드박스 · 다른 PC 와 섞이지 않게)
# ROS 2 통신도 이 PC 안으로. 유선 · 무선 · tailscale 이 함께 켜진 PC 에서 SUBNET 이면 Nav2 lifecycle 응답이
#   사라져 bringup 이 멈춤 (2026-11-08, 2 번 연속 → LOCALHOST 로 정상). 다른 값이 필요하면 SIM_DISCOVERY_RANGE=SUBNET.
#   시험 중 다른 터미널에서 토픽을 보려면 그 터미널도 export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_AUTOMATIC_DISCOVERY_RANGE=${SIM_DISCOVERY_RANGE:-LOCALHOST}
rm -rf "$OUT"; mkdir -p "$OUT/log"
# 노드 이름 키 (drive_node:) 가 있는 yaml 값은 -p 보다 우선이라 덮어쓸 값도 yaml 로
cat > "$OUT/drive_sim.yaml" <<'YAML'
drive_node:
  ros__parameters:
    use_sim_time: true
    scan:
      rear_camera: false       # Gazebo 뒤 깊이 → rear_obstacle 노드 없음
    navigate:
      loc_gate:
        enabled: false         # gazebo_sim 은 loc_monitor 를 끔 (#121)
YAML
SIM="-p use_sim_time:=true"
ros2 launch navigation gazebo_sim.launch.py gui:=false rviz:=false $GZ_ARGS > "$OUT/gz.log" 2>&1 &
echo "[gz_mission] Gazebo + Nav2 시작"
for i in $(seq 1 200); do n=$(grep -c "Managed nodes are active" "$OUT/gz.log" 2>/dev/null); [ "${n:-0}" -ge 3 ] && break; sleep 3; done
echo "[gz_mission] Nav2 active ($(grep -c 'Managed nodes are active' "$OUT/gz.log"))"
ros2 run control drive --ros-args -r __node:=drive_node --params-file $CTL/robot_geometry.yaml --params-file $CTL/drive.yaml \
  --params-file "$OUT/drive_sim.yaml" -r /drive/align:=/drive/align_real -r /drive/execute_skill:=/drive/execute_skill_real > "$OUT/drive.log" 2>&1 &
ros2 run mission mission_node --ros-args -r __node:=mission_node --params-file $MIS/mission.yaml --params-file $MIS/mission_table.yaml \
  --params-file $MIS/mission_plan.yaml $SIM -p survey.enabled:=false -p log_dir:="$OUT/log" > "$OUT/mission.log" 2>&1 &
ros2 launch mission_planner planner.launch.py pucks_file:=$MPL/problem_q1_example.yaml use_sim_time:=true log_dir:="$OUT/log/planner" > "$OUT/planner.log" 2>&1 &
python3 "$HERE/mock_robot.py" --real_nav --approach $MPL/approach_poses.yaml --out "$OUT/mock_out.json" --timeout ${TIMEOUT:-5400} \
  --ros-args $SIM > "$OUT/mock.log" 2>&1 &
MOCK=$!
sleep 20
echo "[gz_mission] /mission/start"; ros2 service call /mission/start std_srvs/srv/Trigger | tail -1
wait $MOCK
echo "[gz_mission] $(tail -1 "$OUT/mock.log")"
stop_all
