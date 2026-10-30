#!/usr/bin/env bash
# 팀 공유본 isaac/ (로봇 설정 · USD) + 이 rl/ 폴더를 Docker 컨테이너 isaac-lab-base 가 보는 폴더로 복사 (rsync, 바뀐 것만).
#   ./sync_to_docker.sh          ->  ~/dockerWorkspace/IsaacLab/scripts/waffle_shared_261028/isaac/   (컨테이너: /workspace/isaaclab/scripts/waffle_shared_261028/isaac/rl)
#   ./sync_to_docker.sh --pull   <-  학습 결과 (rl/logs) 와 수집 데이터 (*.json) 를 되가져옴
# 폴더 이름의 공백 ("urdf_261028 (1)") 을 피하려고 컨테이너 쪽 이름은 waffle_shared_261028 로 고정.
set -eu
ISAAC="$(cd "$(dirname "$0")/.." && pwd)"
DST="${ISAACLAB_HOST_DIR:-$HOME/dockerWorkspace/IsaacLab}/scripts/waffle_shared_261028/isaac"
if [ "${1:-}" = "--pull" ]; then
  [ -d "$DST/rl/logs" ] && rsync -a "$DST/rl/logs/" "$ISAAC/rl/logs/"
  for f in store_handover.json insert_starts.json; do [ -f "$DST/rl/$f" ] && cp -f "$DST/rl/$f" "$ISAAC/rl/$f"; done
  echo "가져옴: $DST/rl/{logs,*.json} -> $ISAAC/rl/"
  exit 0
fi
mkdir -p "$DST"
rsync -a --delete --exclude 'rl/logs/' --exclude '*.log' --exclude '__pycache__/' --exclude 'rl/store_handover.json' --exclude 'rl/insert_starts.json' "$ISAAC/" "$DST/"
for f in store_handover.json insert_starts.json; do [ -f "$ISAAC/rl/$f" ] && [ ! -f "$DST/rl/$f" ] && cp "$ISAAC/rl/$f" "$DST/rl/$f"; done
echo "복사함: $ISAAC -> $DST (컨테이너: /workspace/isaaclab/scripts/waffle_shared_261028/isaac)"
