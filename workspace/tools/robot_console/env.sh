# 콘솔 공통 환경 (start.sh · log.sh · tui.sh 가 source). ROS 2 + 저장소 빌드 결과를 불러옴
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
set +u
source /opt/ros/lyrical/setup.bash
if [ -f "$REPO/workspace/ros2_ws/install/setup.bash" ]; then
  source "$REPO/workspace/ros2_ws/install/setup.bash"
else
  echo "✗ $REPO/workspace/ros2_ws/install 없음 — 먼저 colcon build" >&2; return 1 2>/dev/null || exit 1
fi
