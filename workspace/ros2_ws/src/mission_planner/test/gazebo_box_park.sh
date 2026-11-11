#!/usr/bin/env bash
# Gazebo 박스 앞 정차 시험 (#157): TFmini 비상 정지 잠금을 실물과 같은 경로로 켜고 박스 앞 단계가 움직이는지.
#   진짜: Gazebo 경기장 (gazebo_sim.launch.py nav:=false), Nav2 + AMCL (navigation.launch.py, 출력 /cmd_vel_nav),
#         twist_mux (drive 50 · nav 10, 잠금 obstacle 60 = /perception/obstacle_stop), obstacle_stop (perception lidar.yaml), drive_node
#   가짜: 인식 결과 (ARM_ALIGN, 화면 가운데) 만 — box_park_scenario.py
#   순서: 박스 1 남쪽 stage (0.40 m) 에서 FINE 0.13 → turn_p17 → turn_0 → /drive/navigate 출발 구역
#   통과 = 네 단계 모두 성공 (수정 전 drive_node 는 FINE 뒤 정지가 켜져 turn_p17 시간 초과)
#   PLACE=1 (#160): 장소 L1 stage (빈 상자, 0.45 m) 에서 박스 접근 (box_only, 인식 결과 없음) 0.13 → navigate 출발 구역
#                   (수정 전에는 퍽이 없어 COARSE · FINE 모두 "물체 놓침")
#
#   test/gazebo_box_park.sh [결과 폴더]     기본 /tmp/gazebo_box_park. 결과 폴더/result.json, 종료 코드 0 = 통과
#   PLACE=1 test/gazebo_box_park.sh [결과 폴더]   장소 상자 시나리오 (기본 결과 폴더 /tmp/gazebo_box_park_place)
#   test/gazebo_box_park.sh stop            시험 프로세스 정리
#   CTL_WS=<작업공간> test/gazebo_box_park.sh   control 만 다른 작업공간 것으로 (수정 전 · 후 비교)
# 먼저: source install/setup.bash (mission_planner · control · perception · navigation · localization · waffle_4wd_description · twist_mux)
#
# Gazebo 와 실물의 차이 (시험에서만 맞춤):
#   - EKF 없음 → drive_node 의 /odometry/filtered 를 /odom 으로, 합친 스캔은 /scan_merged (실물 /scan) → remap
#   - Gazebo TFmini 잡음 약 ±3 cm (박스 앞 0.10 ~ 0.23) → 최근 7 개 중앙값 (range_median.py, 보정된 TFmini 흉내) 을
#     obstacle_stop · drive_node 에 주고, align.box.spread_m 0.10 (기본 0.03 이면 늘 "박스 거리 무효"). 실물 잡음은 #87 실측
#   - Nav2 bringup 이 bond 시간 초과로 가끔 실패함 (nav.log "Aborting bringup") → 다시 실행
set +u
HERE="$(cd "$(dirname "$0")" && pwd)"
PAT='gz-sim-main|gz sim|gazebo_sim.launch|navigation.launch|nav2_|/amcl|lifecycle_manager|map_server|scan_merge|scan_body_mask|parameter_bridge|robot_state_publisher|tfmini_scan_to_range|camera_info_fix|lib/control/drive|lib/twist_mux/twist_mux|lib/perception/obstacle_stop|collision_monitor|velocity_smoother|ros_gz_sim|box_park_scenario.py|range_median.py'
stop_all() {
  # 이 스크립트와 부른 셸 (명령줄에 패턴 단어가 들어 있을 수 있음) 은 빼고
  local keep=" $$ " p=$$
  while [ "$p" -gt 1 ]; do p=$(ps -o ppid= -p "$p" | tr -d ' '); [ -z "$p" ] && break; keep="$keep$p "; done
  for sig in INT TERM KILL; do
    pids=$(pgrep -f "$PAT" | while read -r x; do [[ "$keep" == *" $x "* ]] || echo "$x"; done)
    [ -z "$pids" ] && return
    kill -$sig $pids 2>/dev/null; sleep 3
  done
}
if [ "$1" = "stop" ]; then stop_all; exit 0; fi
[ -n "$CTL_WS" ] && source "$CTL_WS/install/setup.bash"
if [ -n "$PLACE" ]; then OUT="${1:-/tmp/gazebo_box_park_place}"; else OUT="${1:-/tmp/gazebo_box_park}"; fi
export GZ_IP=127.0.0.1
export ROS_AUTOMATIC_DISCOVERY_RANGE=${SIM_DISCOVERY_RANGE:-LOCALHOST}   # gazebo_mission.sh 와 같은 이유 (Nav2 lifecycle)
rm -rf "$OUT"; mkdir -p "$OUT"
echo "control = $(ros2 pkg prefix control)" > "$OUT/info.txt"
MAPS=$(ros2 pkg prefix mission_planner)/share/mission_planner/maps
CTL=$(ros2 pkg prefix control)/share/control/config
PER=$(ros2 pkg prefix perception)/share/perception/config
X=1.468; Y=0.3147; YAW=1.5708   # approach_poses.yaml B1:S stage
SCEN_ARGS=""
if [ -n "$PLACE" ]; then X=2.9847; Y=0.95; YAW=0.0; SCEN_ARGS="--place"; fi   # approach_poses.yaml L1 stage (#160)

cat > "$OUT/mux.yaml" <<'YAML'
twist_mux:
  ros__parameters:
    use_sim_time: true
    use_stamped: true
    topics:
      drive: {topic: cmd_vel_drive, timeout: 0.25, priority: 50}
      nav:   {topic: cmd_vel_nav,   timeout: 0.5,  priority: 10}
    locks:
      obstacle: {topic: /perception/obstacle_stop, timeout: 0.5, priority: 60}
YAML
# 노드 이름 키가 있는 yaml 값은 -p 보다 우선이라 덮어쓸 값도 yaml 로
cat > "$OUT/drive_sim.yaml" <<'YAML'
drive_node:
  ros__parameters:
    use_sim_time: true
    scan:
      rear_camera: false       # Gazebo 뒤 깊이 → rear_obstacle 노드 없음
    navigate:
      loc_gate:
        enabled: false         # loc_monitor 안 띄움
    align:
      box:
        spread_m: 0.10         # Gazebo TFmini 잡음 (위 설명)
YAML

ros2 launch navigation gazebo_sim.launch.py gui:=false rviz:=false nav:=false x:=$X y:=$Y yaw:=$YAW > "$OUT/gz.log" 2>&1 &
echo "[box_park] Gazebo 시작"
for i in $(seq 1 60); do grep -q "publishing #3" "$OUT/gz.log" 2>/dev/null && break; sleep 2; done
sleep 8   # 팔 자세 · scan_body_mask 학습
ros2 launch navigation navigation.launch.py use_sim_time:=true odom_topic:=/odom cmd_vel_out:=/cmd_vel_nav \
  map:=$MAPS/map_amcl.yaml objects_map:=$MAPS/map_nav.yaml loc_map:=$MAPS/map_amcl.yaml use_loc_monitor:=false \
  start_x:=$X start_y:=$Y start_yaw:=$YAW > "$OUT/nav.log" 2>&1 &
for i in $(seq 1 100); do n=$(grep -c "Managed nodes are active" "$OUT/nav.log" 2>/dev/null); [ "${n:-0}" -ge 3 ] && break; sleep 3; done
echo "nav active: $(grep -c 'Managed nodes are active' "$OUT/nav.log") / 3" | tee -a "$OUT/info.txt"
python3 "$HERE/range_median.py" --ros-args -p use_sim_time:=true > "$OUT/med.log" 2>&1 &
ros2 run twist_mux twist_mux --ros-args --params-file "$OUT/mux.yaml" -r cmd_vel_out:=/cmd_vel > "$OUT/mux.log" 2>&1 &
ros2 run perception obstacle_stop --ros-args --params-file $PER/lidar.yaml -p use_sim_time:=true \
  -p range_topic:=/tfmini/range_med > "$OUT/obstacle.log" 2>&1 &
ros2 run control drive --ros-args -r __node:=drive_node --params-file $CTL/robot_geometry.yaml --params-file $CTL/drive.yaml \
  --params-file "$OUT/drive_sim.yaml" -r /odometry/filtered:=/odom -r /tfmini/range:=/tfmini/range_med -r /scan:=/scan_merged \
  > "$OUT/drive.log" 2>&1 &
sleep 8
echo "[box_park] 시나리오 (시뮬레이션이 실시간보다 느려 수 분)"
timeout 1500 python3 "$HERE/box_park_scenario.py" 0.69365 0.95 3.14159265 $SCEN_ARGS > "$OUT/scenario.log" 2>&1
RC=$?
tail -1 "$OUT/scenario.log" > "$OUT/result.json"
cat "$OUT/result.json"; echo
stop_all
[ $RC -eq 0 ] && echo "[box_park] 통과" || echo "[box_park] 실패 (종료 코드 $RC, 로그 $OUT)"
exit $RC
