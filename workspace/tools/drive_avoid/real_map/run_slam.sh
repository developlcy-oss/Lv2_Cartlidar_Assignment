#!/usr/bin/env bash
# 실물 미로 bag → slam_toolbox 지도. 이 PC 안에서만 (ROS_DOMAIN_ID 77 · LOCALHOST) — Jetson 과 안 섞임.
#   ./run_slam.sh [배속=2.0] [이름=maze_real_slam]
RATE=${1:-2.0}; NAME=${2:-maze_real_slam}
source /opt/ros/lyrical/setup.bash
set -u
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
cd "$(dirname "$0")"
ros2 run slam_toolbox sync_slam_toolbox_node --ros-args --params-file slam_params.yaml -p use_sim_time:=true > ${NAME}_slam.log 2>&1 &
S=$!
trap 'kill -INT $S 2>/dev/null; sleep 2; kill $S 2>/dev/null' EXIT
sleep 3
ros2 lifecycle set /slam_toolbox configure && ros2 lifecycle set /slam_toolbox activate || { echo "slam_toolbox 켜기 실패"; exit 1; }
ros2 lifecycle get /slam_toolbox
echo "$(date +%T) 재생 시작 (x$RATE)"
ros2 bag play ${IN:-maze_slam_in} --clock 50 --rate $RATE --disable-keyboard-controls > ${NAME}_play.log 2>&1
echo "$(date +%T) 재생 끝 — 남은 스캔 처리 기다림"
for i in $(seq 1 60); do c=$(ps -o %cpu= -p $S | cut -d. -f1); [ "${c:-0}" -lt 10 ] && break; sleep 2; done
sleep 5
ros2 run nav2_map_server map_saver_cli -f $PWD/$NAME --ros-args -p use_sim_time:=true > ${NAME}_saver.log 2>&1 && echo "지도 저장: $NAME.pgm/.yaml"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$PWD/$NAME'}" > /dev/null && echo "자세 그래프: $NAME.posegraph"
ls -la $NAME.*
