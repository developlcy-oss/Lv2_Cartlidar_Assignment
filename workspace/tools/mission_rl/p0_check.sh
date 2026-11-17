#!/usr/bin/env bash
# P0 (2026-11-17): 팀 팔 확인 스크립트를 지금 저장소 · 이 PC (Isaac Lab 3.0) 에서 그대로 다시 돌림 — 팀 파일은 안 고치고 결과만 이 폴더로.
#   ./p0_check.sh [chain|place|unload|all]   (기본 all, 차례로 · Isaac 은 한 번에 하나만)
#   결과: results/p0_<이름>.json · results/p0_<이름>.log
# 팀 run_rl.sh 를 거치지 않는 까닭: 결과 JSON 을 팀 폴더에 덮어쓰고 학습기록_자동.md 에 줄을 더함.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; MP="$(cd "$HERE/../../urdf_and_usd/policy/manipulator" && pwd)"
PY="${ISAACLAB_PY:-$HOME/pa-course/IsaacLab-6.1/.venv/bin/python}"
export PYTHONDONTWRITEBYTECODE=1   # 팀 폴더에 __pycache__ 안 만들게
# ROS 환경 변수는 빼고 (Isaac 과 섞이지 않게, 팀 isaac/run.sh 와 같은 방식)
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION PYTHONPATH LD_LIBRARY_PATH CMAKE_PREFIX_PATH
run() {  # 이름 · 스크립트 · 인자...
  local name=$1 script=$2; shift 2
  echo "▶ $name: $(basename "$script") $*"
  local t0=$(date +%s)
  ( cd "$MP" && "$PY" "$script" "$@" --out "$HERE/results/p0_$name.json" ) > "$HERE/results/p0_$name.log" 2>&1
  echo "  끝 (종료 $?, $(( $(date +%s) - t0 )) s) · $(python3 -c "import json,sys;d=json.load(open(sys.argv[1]));print(d.get('summary'))" "$HERE/results/p0_$name.json" 2>/dev/null | cut -c1-300)"
}
W=${1:-all}
[ "$W" = all -o "$W" = chain ] && run chain "$MP/chain_pallet_eval.py" --num_envs 192 --scripted_pick 2 --grip_open 0.0042 --seed 2
[ "$W" = all -o "$W" = place ] && run place "$MP/pallet_place_check.py"
[ "$W" = all -o "$W" = unload ] && run unload "$MP/pallet_place_check.py" --unload --row_speed ""
