#!/usr/bin/env bash
# waffle_bridge.py 실행기 (Isaac Sim 6.2 단독). 호스트 ROS (Lyrical) 환경을 비우고 Isaac 내장 jazzy rclpy 를 쓴다.
#   ./run_bridge.sh [--headless] [--usd <waffle_4wd.usda>] [--cmd_timeout 0.5]
# Isaac Sim 위치: ISAACSIM_PY=/경로/python ./run_bridge.sh ...  (기본 ~/env_isaacsim62/bin/python)
# 호스트 ROS_DOMAIN_ID · ROS_AUTOMATIC_DISCOVERY_RANGE 는 그대로 넘긴다 (양쪽 같아야 토픽이 보임)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="${ISAACSIM_PY:-$HOME/env_isaacsim62/bin/python}"
[ -x "$PY" ] || { echo "Isaac Sim 6.2 Python 을 찾지 못했습니다: $PY (ISAACSIM_PY 로 지정)"; exit 1; }
CORE="$("$PY" -c 'import isaacsim, os; print(os.path.join(os.path.dirname(isaacsim.__file__), "exts", "isaacsim.ros2.core"))')"
[ -d "$CORE/jazzy/lib" ] || { echo "isaacsim.ros2.core jazzy 라이브러리 없음: $CORE"; exit 1; }
keep=()
for v in HOME USER DISPLAY XAUTHORITY XDG_RUNTIME_DIR WAYLAND_DISPLAY LANG ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS; do
  [ -n "${!v:-}" ] && keep+=("$v=${!v}")
done
exec env -i "${keep[@]}" PATH=/usr/bin:/bin OMNI_KIT_ACCEPT_EULA=YES \
  ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp LD_LIBRARY_PATH="$CORE/jazzy/lib" \
  "$PY" "$HERE/waffle_bridge.py" "$@"
