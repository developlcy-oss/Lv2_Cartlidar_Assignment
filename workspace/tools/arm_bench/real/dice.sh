#!/usr/bin/env bash
# 주사위 배치 teleop 래퍼 (2026-11-13, 넣는거 세션): 퍽스토리지 (팔레트) 칸에서 퍽을 꺼내 장소 상자 3x3 에 주사위 눈대로 놓기.
# 스킬 하나하나는 같은 폴더의 arm_cmd.sh 로 보냄 -> 실물 (젯슨, /manipulator/execute_skill) 과 Isaac 시뮬
# (policy/manipulator/tools/sim_skill_server.py) 에서 같은 명령으로 동작.
#
#   ./dice.sh sim  -    <눈 1~6> [옵션]      Isaac 시뮬 (L 자리는 - 로 둠, place_at:base_link 좌표 사용)
#   ./dice.sh real <L>  <눈 1~6> [옵션]      실물 (젯슨에서, bringup 뜬 상태. place_<L>_<dx>_<dy>_tilt)
#
# 옵션:
#   --slots "1_0 1_1 ..."  꺼낼 팔레트 칸 (열_칸). 기본 = 1_0 부터 차례로 눈 개수만큼.
#                          실물은 팔레트 점유 인식이 없으므로 실제로 퍽이 있는 칸을 적어 줄 것.
#   --far dy+|dy-          실물: 먼 줄 (J1 에서 먼 쪽) 이 지도 +Y 인지 -Y 인지 (기본 dy+). 먼 칸부터 놓아
#                          쥔 퍽이 이미 놓은 퍽 위를 지나지 않게 함 (mission place_order 와 같은 취지).
#   --cell0 "x y z"        시뮬: 가운데 칸 (0,0) 좌표 (base_link). 기본 = 서버 cells 명령으로 받음.
#   --pitch 0.06           시뮬: 칸 간격 [m] (--cell0 를 쓸 때만).
#   --transit              스킬 사이에 transit (이동 자세) 삽입 (시뮬 전용).
#   --record               실물: ros2 bag 기록 (joint_states · 팔 카메라 · /perception/pucks · tf) -> ~/실측/dice_<시각>/
#   --timeout 60           스킬 하나 제한 시간 [s]
#   --keep_going           실패해도 다음 퍽 계속
#   --dry                  보낼 스킬 이름만 출력하고 끝
#
# 주사위 눈 -> 칸 (dx, dy): mission_planner/poses.py DICE 와 동일 (지도 +Y 가 위, 발제 03 쪽).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
ARM_CMD="${ARM_CMD:-$HERE/arm_cmd.sh}"
MODE="${1:?sim|real}"; L="${2:?장소 L (sim 은 -)}"; DIE="${3:?눈 1~6}"; shift 3
SLOTS=""; FAR="dy+"; CELL0=""; PITCH=0.06; TRANSIT=0; RECORD=0; TIMEOUT=60; KEEP=0; DRY=0
while [ $# -gt 0 ]; do case "$1" in
  --slots) SLOTS="$2"; shift 2;; --far) FAR="$2"; shift 2;; --cell0) CELL0="$2"; shift 2;;
  --pitch) PITCH="$2"; shift 2;; --transit) TRANSIT=1; shift;; --record) RECORD=1; shift;;
  --timeout) TIMEOUT="$2"; shift 2;; --keep_going) KEEP=1; shift;; --dry) DRY=1; shift;;
  *) echo "모르는 옵션: $1"; exit 2;; esac; done
case "$DIE" in 1|2|3|4|5|6) ;; *) echo "눈은 1~6"; exit 2;; esac
[ "$MODE" = real ] && [ "$L" = "-" ] && { echo "real 은 장소 L 필요 (예: L1)"; exit 2; }
[ "$DRY" = 0 ] && { [ -x "$ARM_CMD" ] || { echo "arm_cmd.sh 필요: $ARM_CMD (ARM_CMD= 로 지정 가능)"; exit 3; }; }

# 눈 -> 칸 목록 (poses.py DICE), 먼 칸부터
CELLS=$(python3 - "$DIE" "$FAR" <<'PY'
import sys
DICE = {1: [(0, 0)], 2: [(-1, 1), (1, -1)], 3: [(-1, 1), (0, 0), (1, -1)],
        4: [(-1, 1), (1, 1), (-1, -1), (1, -1)],
        5: [(-1, 1), (1, 1), (0, 0), (-1, -1), (1, -1)],
        6: [(-1, 1), (-1, 0), (-1, -1), (1, 1), (1, 0), (1, -1)]}
die, far = int(sys.argv[1]), sys.argv[2]
sgn = 1 if far == "dy+" else -1
for dx, dy in sorted(DICE[die], key=lambda c: (-sgn * c[1], abs(c[0]))):
    print(f"{dx} {dy}")
PY
) || exit 3
N=$(echo "$CELLS" | wc -l)
[ -z "$SLOTS" ] && SLOTS=$(python3 -c "
rows=[(r,i) for r in (1,2,3) for i in range(4)]
print(' '.join(f'{r}_{i}' for r,i in rows[:$N]))")
NS=$(echo $SLOTS | wc -w)
[ "$NS" -lt "$N" ] && { echo "칸 $N 개 필요한데 --slots 가 $NS 개"; exit 2; }

# 시뮬: 칸 좌표 (base_link) 확보
declare -A XY
if [ "$MODE" = sim ]; then
  [ "$DRY" = 1 ] && [ -z "$CELL0" ] && CELL0="0.30 0.00 0.070"  # dry: 보기용 임시 좌표
  if [ -n "$CELL0" ]; then
    read -r X0 Y0 Z0 <<< "$CELL0"
    while read -r dx dy; do XY["$dx,$dy"]=$(python3 -c "print(f'{$X0+$dx*$PITCH:.4f} {$Y0+$dy*$PITCH:.4f} {$Z0:.4f}')"); done <<< "$CELLS"
  else
    CJ=$(ARM_TIMEOUT="$TIMEOUT" "$ARM_CMD" sim cells) || { echo "서버 cells 실패 — --cell0 'x y z' 로 직접 주세요"; exit 3; }
    while read -r dx dy; do
      v=$(echo "$CJ" | python3 -c "
import sys, json, re
s = sys.stdin.read()
m = re.search(r'\{.*\}', s, re.S)
d = json.loads(m.group(0)) if m else {}
d = d.get('cells', d.get('msg', d))
d = json.loads(d) if isinstance(d, str) else d
c = d['$dx,$dy']
print(f\"{c['x']:.4f} {c['y']:.4f} {c['z']:.4f}\")") || { echo "cells 응답에서 칸 $dx,$dy 못 읽음: $CJ"; exit 3; }
      XY["$dx,$dy"]="$v"
    done <<< "$CELLS"
  fi
fi

idx(){ case "$1" in -1) echo m1;; 0) echo 0;; 1) echo p1;; esac; }
BAG=""
if [ "$RECORD" = 1 ] && [ "$MODE" = real ] && [ "$DRY" = 0 ]; then
  BAG="$HOME/실측/dice_$(date +%m%d_%H%M%S)"
  mkdir -p "$BAG"
  ros2 bag record -o "$BAG/bag" /joint_states /arm_camera/image_raw /perception/pucks /tf /tf_static > "$BAG/bag.log" 2>&1 &
  BAGPID=$!; echo "[dice] 기록 시작 -> $BAG (PID $BAGPID)"
fi
LOG="$HERE/dice_$(date +%m%d_%H%M%S).log"; touch "$LOG"
say(){ echo "[dice] $*" | tee -a "$LOG"; }
run(){ # $* = arm_cmd 하위 명령 (unload 3 1 / place x y z / place_cell L m1 p1 / transit)
  say "-> $*"
  [ "$DRY" = 1 ] && return 0
  ARM_TIMEOUT="$TIMEOUT" "$ARM_CMD" "$MODE" "$@" >> "$LOG" 2>&1
}
say "모드 $MODE, 장소 $L, 눈 $DIE (칸 $N 개), 꺼낼 칸: $(echo $SLOTS | tr ' ' ',' | cut -d, -f1-$N)"
OK=0; FAIL=0; i=0
for s in $SLOTS; do
  [ $i -ge $N ] && break
  cell=$(echo "$CELLS" | sed -n "$((i+1))p"); read -r dx dy <<< "$cell"
  r="${s%_*}"; k="${s#*_}"
  if run unload "$r" "$k"; then
    [ "$TRANSIT" = 1 ] && run transit
    if [ "$MODE" = sim ]; then set -- place ${XY[$dx,$dy]}; else set -- place_cell "$L" "$(idx $dx)" "$(idx $dy)"; fi
    if run "$@"; then OK=$((OK+1)); else FAIL=$((FAIL+1)); say "놓기 실패 (칸 $dx,$dy)"; [ "$KEEP" = 0 ] && break; fi
  else
    FAIL=$((FAIL+1)); say "꺼내기 실패 (팔레트 $s)"; [ "$KEEP" = 0 ] && break
  fi
  i=$((i+1))
done
[ -n "$BAG" ] && { kill $BAGPID 2>/dev/null; say "기록 끝 -> $BAG"; }
say "끝: 성공 $OK / 시도 $((OK+FAIL)) (목표 $N). 자세한 로그 $LOG"
[ "$FAIL" = 0 ] && exit 0 || exit 1
