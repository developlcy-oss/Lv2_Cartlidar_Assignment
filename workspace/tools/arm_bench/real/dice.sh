#!/usr/bin/env bash
# 주사위 배치 teleop 래퍼 (2026-11-13, 넣는거 세션): 퍽스토리지 (팔레트) 칸에서 퍽을 꺼내 장소 상자 3x3 에 주사위 눈대로 놓기.
# 스킬 하나하나는 같은 폴더의 arm_cmd.sh 로 보냄 -> 실물 (젯슨, /manipulator/execute_skill) 과 Isaac 시뮬
# (policy/manipulator/tools/sim_skill_server.py) 에서 같은 명령으로 동작.
#
#   ./dice.sh sim  -    <눈 1~6> [옵션]      Isaac 시뮬 (L 자리는 - 로 둠, place_at:base_link 좌표 사용)
#   ./dice.sh real <L>  <눈 1~6> [옵션]      실물 (젯슨에서, bringup 뜬 상태. place_<L>_<dx>_<dy>_tilt — 위치 인식 · 지도 필요)
#   ./dice.sh real -    <눈 1~6> --bench     실물 책상 시험 (지도 없이 place_at:base_link). 상자 = 로봇 앞 끝에서 5 cm (5 ~ 6 cm, 다르면 --cell0), 가운데 맞춤
#                                           -> 가운데 칸 0.2853 0 0.0633. 눈 그림의 위 (dy+) = 로봇 앞 (먼 쪽), 오른쪽 (dx+) = 로봇 오른쪽
#
# 옵션:
#   --slots "1_0 1_1 ..."  꺼낼 팔레트 칸 (열_칸). 기본 = 계획기 차례로 앞에서 눈 개수만큼.
#   --occupied "2_1 ..."   꺼내지 않고 팔레트에 남아 있는 다른 퍽 칸 (기본 없음). 실물은 점유 인식이 없으니 실제대로 적을 것.
#   [계획기 · 미션과 같게 (2026-11-14)] 꺼내는 차례 = mission_planner slots.py 의 unload_order (열 규칙: 앞 열에 퍽이 있으면
#     뒤 열 못 꺼냄 · 막는 쌍 BLOCKERS 가 빈 칸부터 · 앞 열 · UNLOAD_PREF) 를 그대로 불러서 씀 (설치된 mission_planner, 없으면 같은 규칙 사본).
#     규칙상 못 꺼내는 조합 (예: 1 열에 퍽이 남았는데 2 열 칸) 이면 시작하지 않음. 놓는 차례 = poses.py place_order (J1 에서 먼 칸부터).
#     Isaac 2048 대 98.3 % (v5, 11-13) 가 이 차례로 잰 값.
#   --far dy+|dy-          실물: 먼 줄 (J1 에서 먼 쪽) 이 지도 +Y 인지 -Y 인지 (기본 dy+). 먼 칸부터 놓아
#                          쥔 퍽이 이미 놓은 퍽 위를 지나지 않게 함 (mission place_order 와 같은 취지).
#   --cell0 "x y z"        시뮬: 가운데 칸 (0,0) 좌표 (base_link). 기본 = 서버 cells 명령으로 받음.
#   --removed "2_3 ..."    쓰지 않을 팔레트 칸 (기본 = PALLET_REMOVED 환경 변수, 없으면 없음). 팔레트 v5 (11-13) = 10 칸: 1 · 2 열 0 ~ 3, 3 열 0 · 1
#                          이 칸은 --slots 에 적으면 거부, 기본 칸 고르기에서도 뺌. 모든 칸 다 쓰려면 --removed ""
#   --bench                실물: 지도 없이 base_link 좌표로 놓기 (--cell0 기본 "0.2853 0.0 0.0633" = 앞 끝 0.1328 + 5 cm + 반 변 0.1025, 6 cm 면 "0.2953 0.0 0.0633")
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
SLOTS=""; OCC=""; REMOVED="${PALLET_REMOVED-}"; FAR="dy+"; CELL0=""; BENCH=0; PITCH=0.06; TRANSIT=0; RECORD=0; TIMEOUT=60; KEEP=0; DRY=0
while [ $# -gt 0 ]; do case "$1" in
  --slots) SLOTS="$2"; shift 2;; --occupied) OCC="$2"; shift 2;; --removed) REMOVED="$2"; shift 2;; --far) FAR="$2"; shift 2;; --cell0) CELL0="$2"; shift 2;; --bench) BENCH=1; shift;;
  --pitch) PITCH="$2"; shift 2;; --transit) TRANSIT=1; shift;; --record) RECORD=1; shift;;
  --timeout) TIMEOUT="$2"; shift 2;; --keep_going) KEEP=1; shift;; --dry) DRY=1; shift;;
  *) echo "모르는 옵션: $1"; exit 2;; esac; done
case "$DIE" in 1|2|3|4|5|6) ;; *) echo "눈은 1~6"; exit 2;; esac
[ "$MODE" = real ] && [ "$BENCH" = 0 ] && [ "$L" = "-" ] && { echo "real 은 장소 L 필요 (예: L1), 책상 시험은 --bench"; exit 2; }
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
# 칸 고르기 · 꺼내는 차례: 계획기 slots.py 를 그대로 (설치본 -> 저장소 소스 -> 같은 규칙 사본)
SLOTS=$(python3 - "$N" "$REMOVED" "$OCC" $SLOTS <<'PY2'
import os, sys
for p in (os.path.expanduser("~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/src/mission_planner"),):
    sys.path.insert(0, p)
try:
    from mission_planner.slots import SLOT_SHAPE, UNLOAD_PREF, _Rules
    rules = _Rules("free"); src = "mission_planner.slots"
except Exception as e:   # 계획기 없음 -> 같은 규칙 (v5, 11-13)
    SLOT_SHAPE = {(r, i): 0 for r in (1, 2) for i in range(4)}; SLOT_SHAPE.update({(3, 0): 0, (3, 1): 0})
    UNLOAD_PREF = {1: [3, 0, 2, 1], 2: [2, 0, 3, 1], 3: [1, 0]}
    BL = {(1, 0): ((1, 1),), (1, 1): ((1, 0),), (1, 2): ((1, 3),)}
    class _R:
        def can_take(self, occ, s): return not any(rr < s[0] for (rr, ii) in occ if (rr, ii) != s)
        def unload_order(self, occ, slots):
            occ, todo, out = dict(occ), set(slots), []
            while todo:
                ok = [s for s in todo if self.can_take(occ, s)]
                if not ok: return None, 0
                s = min(ok, key=lambda s: (any(b in occ for b in BL.get(s, ())), s[0], UNLOAD_PREF[s[0]].index(s[1])))
                out.append(s); todo.discard(s); del occ[s]
            return out, 0
    rules = _R(); src = f"사본 ({e.__class__.__name__})"
n, removed, occ_in, sl = int(sys.argv[1]), set(sys.argv[2].split()), sys.argv[3].split(), sys.argv[4:]
def parse(s):
    r, _, i = s.partition("_")
    if not (r.isdigit() and i.isdigit() and (int(r), int(i)) in SLOT_SHAPE):
        sys.exit(f"칸 {s}: 팔레트 v5 에 없음 (1 · 2 열 0 ~ 3, 3 열 0 · 1)")
    if s in removed:
        sys.exit(f"칸 {s} 는 쓰지 않는 칸 (--removed)")
    return (int(r), int(i))
take = [parse(s) for s in sl]
rest = [parse(s) for s in occ_in]
if not take:
    key = lambda s: (s[0], UNLOAD_PREF[s[0]].index(s[1]))
    take = sorted([s for s in SLOT_SHAPE if f"{s[0]}_{s[1]}" not in removed and s not in rest], key=key)[:n]
occ = {s: 1 for s in take + rest}
order, blocked = rules.unload_order(occ, take)
if order is None:
    sys.exit("계획기 규칙상 못 꺼냄 (앞 열에 퍽이 남아 있으면 뒤 열 칸을 꺼낼 수 없음): 꺼낼 " + " ".join(sl) + " / 남은 " + " ".join(occ_in))
print(" ".join(f"{r}_{i}" for r, i in order))
print(f"[dice] 꺼내는 차례 = {src} unload_order (막힌 채 꺼냄 {blocked})", file=sys.stderr)
PY2
) || exit 2
NS=$(echo $SLOTS | wc -w)
[ "$NS" -lt "$N" ] && { echo "칸 $N 개 필요한데 --slots 가 $NS 개"; exit 2; }

# 시뮬: 칸 좌표 (base_link) 확보
declare -A XY
if [ "$MODE" = real ] && [ "$BENCH" = 1 ]; then  # 책상: 눈 그림 위 (dy+) = 로봇 앞 (+x), 오른쪽 (dx+) = 로봇 오른쪽 (−y)
  [ -z "$CELL0" ] && CELL0="0.2853 0.0 0.0633"
  read -r X0 Y0 Z0 <<< "$CELL0"
  while read -r dx dy; do XY["$dx,$dy"]=$(python3 -c "print(f'{$X0+$dy*$PITCH:.4f} {$Y0-$dx*$PITCH:.4f} {$Z0:.4f}')"); done <<< "$CELLS"
elif [ "$MODE" = sim ]; then
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

# 놓는 차례 = poses.py place_order: J1 (base_link x 0.096) 에서 먼 칸부터 (좌표를 아는 sim · bench). 지도 칸 (place_cell) 은 --far 규칙
if [ "$MODE" = sim ] || [ "$BENCH" = 1 ]; then
  CELLS=$(while read -r dx dy; do read -r x y z <<< "${XY[$dx,$dy]}"; python3 -c "import math;print(f'{-math.hypot($x-0.096,$y):.5f} $dx $dy')"; done <<< "$CELLS" | sort -s -n -k1,1 | cut -d' ' -f2-)
fi
idx(){ case "$1" in -1) echo m1;; 0) echo 0;; 1) echo p1;; esac; }
BAG=""
if [ "$RECORD" = 1 ] && [ "$MODE" = real ] && [ "$DRY" = 0 ]; then
  BAG="$HOME/실측/dice_$(date +%m%d_%H%M%S)"
  mkdir -p "$BAG"
  ros2 bag record -o "$BAG/bag" --topics /joint_states /arm_camera/image_raw /perception/pucks /tf /tf_static > "$BAG/bag.log" 2>&1 &
  BAGPID=$!; echo "[dice] 기록 시작 -> $BAG (PID $BAGPID)"
fi
LOG="$HERE/dice_$(date +%m%d_%H%M%S).log"; touch "$LOG"
say(){ echo "[dice] $*" | tee -a "$LOG"; }
run(){ # $* = arm_cmd 하위 명령 (unload 3 1 / place x y z / place_cell L m1 p1 / transit)
  say "-> $*"
  [ "$DRY" = 1 ] && return 0
  local k unfolded=0
  for k in 1 2 3 4 5 6 7 8 9; do   # 노드가 켜질 때 정리 중이면 거부 -> 10 s 뒤 다시 (최대 90 s)
    local before; before=$(wc -l < "$LOG")
    ARM_TIMEOUT="$TIMEOUT" "$ARM_CMD" "$MODE" "$@" >> "$LOG" 2>&1 && return 0
    tail -n +"$((before + 1))" "$LOG" | grep -q "Goal was rejected" || return 1
    if [ "$MODE" = real ] && [ $unfolded = 0 ]; then   # 2026-11-18: 브링업 뒤 팔이 STOW 에 머묾, STOW 에서 스킬 거절 -> 먼저 폄
      say "  거부됨 -> 팔이 STOW 일 수 있어 pose DRIVE 먼저 (노드 단계 펴기)"
      ARM_TIMEOUT="$TIMEOUT" "$ARM_CMD" real pose DRIVE >> "$LOG" 2>&1 || say "  pose DRIVE 실패"
      unfolded=1; continue
    fi
    say "  거부됨 (켜질 때 정리 중이거나 다른 스킬 중) -> 10 s 뒤 다시 ($k/9)"; sleep 10
  done
  return 1
}
say "모드 $MODE, 장소 $L, 눈 $DIE (칸 $N 개), 꺼낼 칸 (계획기 차례): $(echo $SLOTS | tr ' ' ',' | cut -d, -f1-$N) | 남은 퍽: ${OCC:-없음} (없는 칸: ${REMOVED:-없음})"
case " $SLOTS " in *" 3_"*) say "주의: 3 열 칸을 꺼낼 때 1 · 2 열에 남은 퍽이 없어야 함 (3 열 궤적이 앞 열 위를 낮게 지나감, 계획기 규칙과 같음)";; esac
OK=0; FAIL=0; i=0
for s in $SLOTS; do
  [ $i -ge $N ] && break
  cell=$(echo "$CELLS" | sed -n "$((i+1))p"); read -r dx dy <<< "$cell"
  r="${s%_*}"; k="${s#*_}"
  if run unload "$r" "$k"; then
    [ "$TRANSIT" = 1 ] && run transit
    if [ "$MODE" = sim ] || [ "$BENCH" = 1 ]; then set -- place ${XY[$dx,$dy]}; else set -- place_cell "$L" "$(idx $dx)" "$(idx $dy)"; fi
    if run "$@"; then OK=$((OK+1)); else FAIL=$((FAIL+1)); say "놓기 실패 (칸 $dx,$dy)"; [ "$KEEP" = 0 ] && break; fi
    i=$((i+1))   # 놓기를 시도한 뒤에만 다음 칸으로
  else
    # 2026-11-15: 꺼내기 실패면 이 칸 (dx, dy) 은 비워 두지 않고 다음 퍽이 그대로 씀 (예전: 칸을 건너뛰어 눈 모양에 구멍 · 차례가 흐트러짐)
    FAIL=$((FAIL+1)); say "꺼내기 실패 (팔레트 $s) -> 칸 $dx,$dy 는 다음 퍽으로"; [ "$KEEP" = 0 ] && break
  fi
done
[ $i -lt $N ] && say "칸 $((N - i)) 개를 못 채움 (꺼낼 퍽이 모자람 — --slots 에 칸을 더 주거나 팔레트를 채울 것)"
[ -n "$BAG" ] && { kill $BAGPID 2>/dev/null; say "기록 끝 -> $BAG"; }
say "끝: 성공 $OK / 시도 $((OK+FAIL)) (목표 $N). 자세한 로그 $LOG"
[ "$FAIL" = 0 ] && exit 0 || exit 1
