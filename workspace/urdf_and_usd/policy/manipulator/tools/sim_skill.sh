#!/usr/bin/env bash
# 시뮬 스킬 서버 켜기 · 끄기 (2026-11-13). 호스트에서:
#   tools/sim_skill.sh start [--headless] [--policy policies/sidepick45_k9.pt] [--act_delay 0,1]   창 1 개로 띄움 (GPU 잠금 확인 · 기록)
#   tools/sim_skill.sh stop                                                                      끄고 잠금 지움
#   tools/sim_skill.sh log                                                                       서버 로그 끝 20 줄
set -u
HERE="$(cd "$(dirname "$0")/.." && pwd)"; cd "$HERE"
LOCK="$HERE/.gpu_lock"; NAME="시뮬 스킬 서버"
case "${1:-}" in
  start)
    shift
    if [ -f "$LOCK" ] && ! grep -q "$NAME" "$LOCK"; then echo "GPU 사용 중: $(cat "$LOCK")"; echo "끝난 뒤 다시 하거나, 그 작업을 멈추세요."; exit 1; fi
    echo "$NAME · 시작 $(date +%H:%M) · 끝 = tools/sim_skill.sh stop" > "$LOCK"
    docker exec -d isaac-lab-base bash -c "cd /workspace/isaaclab/scripts/lv2_urdfusd/policy/manipulator && export ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib:\$LD_LIBRARY_PATH && env -u HEADLESS /workspace/isaaclab/isaaclab.sh -p tools/sim_skill_server.py --viz kit $* > sim_skill_server.log 2>&1"
    echo "서버 띄우는 중 (1 분쯤) ..."
    for i in $(seq 1 120); do
      grep -aq "준비됨" sim_skill_server.log 2>/dev/null && { echo "준비됨. 이제 tools/arm_cmd.sh sim <명령> 을 쓰세요."; exit 0; }
      grep -aq "Traceback" sim_skill_server.log 2>/dev/null && { echo "서버 오류:"; grep -a -A8 Traceback sim_skill_server.log | tail -8; exit 1; }
      sleep 2
    done
    echo "2 분 넘게 준비 안 됨 — tools/sim_skill.sh log 로 확인"; exit 1 ;;
  stop)
    docker exec isaac-lab-base bash -c 'pkill -INT -f "[s]im_skill_server.py"; sleep 3; pkill -9 -f "[s]im_skill_server.py"'
    grep -q "$NAME" "$LOCK" 2>/dev/null && rm -f "$LOCK"; echo "껐습니다." ;;
  log) tail -20 sim_skill_server.log | grep -av "Warning\|warn" ;;
  *) sed -n 2,5p "$0" ;;
esac
