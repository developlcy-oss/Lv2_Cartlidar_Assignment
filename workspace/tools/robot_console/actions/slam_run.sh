#!/usr/bin/env bash
# SLAM 녹화 주행: 주행 시연 경로를 따라가며 bag 녹화 → PC 에서 Cartographer (data/261114_1330_maze_map 의 make_bag_carto.py · run_offline.sh)
HERE="$(cd "$(dirname "$0")/.." && pwd)"; REPO="$(cd "$HERE/../../.." && pwd)"
B=$HOME/real_drive/bags/slam_$(date +%y%m%d_%H%M); mkdir -p "$(dirname "$B")"
ros2 bag record -o "$B" --topics /scan_front /scan_rear /scan_front_filtered /scan_rear_filtered /scan_merged /odom /odom_icr /odometry/filtered /imu/data /tf /tf_static /joint_states /cmd_vel </dev/null &
R=$!
trap 'kill -INT $R 2>/dev/null; wait $R; echo "bag: $B"' EXIT
sleep 3
echo "녹화 중: $B — 주행 시연 경로 시작"
bash "$REPO/workspace/tools/drive_demo/run_demo.sh" real slow init
