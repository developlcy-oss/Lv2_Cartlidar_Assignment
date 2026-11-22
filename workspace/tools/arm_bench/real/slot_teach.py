#!/usr/bin/env python3
"""퍽스토리지 (팔레트 v5) 칸 위치 맞추기 (2026-11-13, 젯슨, 브링업 상태). 칸마다 실물 퍽에 집게를 손으로 맞춰 기록하고,
그 차이로 노드 팔레트 궤적 (pallet_place_traj.json) 을 고친 판을 만든다.

    python3 ~/slot_teach.py                 10 칸 모두 차례로 (각 칸: 칸 위 -> 맞춤 -> ok)
    python3 ~/slot_teach.py 1_3 2_2         고른 칸만
    python3 ~/slot_teach.py --make          기록 (~/실측/slot_teach.json) -> ~/pallet_place_traj_taught.json (노드용)
    python3 ~/slot_teach.py --show          기록된 칸별 차이 (mm · 도) 만 출력
    python3 ~/slot_teach.py --box [셀 …] [--cx 0.2853]   상자 위 놓는 자리 맞추기 (책상: 상자 = 앞 끝에서 5 cm 가운데, 셀 '1,1' = 먼 줄 왼쪽 … 기본 9 칸)
    python3 ~/slot_teach.py --fit_box       상자 기록 -> 관절 영점 (ik.joint_offsets) 제안 · 셀별 잔차 (노드 놓기 IK 용)
    python3 ~/slot_teach.py --make_box      상자 기록 -> ~/place_box_teach.json (노드 place.teach: 칸마다 모델 오차를 빼고 IK, 2026-11-17)
    python3 ~/slot_teach.py --set 3_1 왼 5 [node|offset]   맞춰 보지 않고 말로: 쥐는 자리에서 5 mm 왼쪽 (offset = 영점 보정 더한 자리 기준)

준비: 브링업 (skill_enabled:=true) + kbd_joy.py 에서 r -> 'startup: arm at STOW' 뒤 (팔이 STOW 면 여기서 노드에 pose DRIVE 를 보내 폄). **맞출 칸에만 퍽 하나**를 넣어 둠 (다른 칸은 비움).
맞춤 기준 (노드 쥐는 자리): 퍽이 두 손가락 정가운데, 손가락 가운데가 퍽 윗면보다 15 mm 아래, 집게 45° 기울임 그대로.
  조정 명령 (meas_session 과 같음): '아래 5' · '위 3' · '멀리 2' · '가까이 2' · '왼 1' · '오른 1' (mm, 로봇 기준) · '집게 42' (손가락 사이 mm)
  ok = 맞춤 끝 · 기록 / b = 이 칸 건너뜀 / q = 끝
시작 자세 = 노드가 실제로 보내는 값 (모델 궤적, pallet.apply_joint_offsets 가 true 면 + ik.joint_offsets) 의 칸 위 (place 구간 처음) -> 맞춘 관절 - 노드 쥐는 자리 = 칸 보정.
--make: 맞춘 칸은 그 보정을 칸 궤적 (carry · place · release · retract · return 전부, rotate · rotate_back 은 앞 0 -> 칸 1 로 서서히) 에 더함.
        안 맞춘 칸은 같은 열에서 맞춘 칸들의 평균 (없으면 그대로). 노드에 넣는 법은 --make 끝에 출력.
"""
import datetime
import json
import math
import os
import sys

REPO = os.environ.get("TEAM_WS", os.path.expanduser("~/Lv2_Cartlidar_Assignment/workspace"))
# 2026-11-21 수직 집기 판: TEACH_TRAJ=<config>/pallet_place_traj_vertical.json TEACH_ORIG=~/pallet_place_traj_vertical_orig.json
#   TEACH_REC=~/실측/slot_teach_vertical.json TEACH_TAUGHT=~/pallet_place_traj_vertical_taught.json (45° 판 기록과 섞이지 않게)
TRAJ = os.environ.get("TEACH_TRAJ", os.path.join(REPO, "ros2_ws/src/control/config/pallet_place_traj.json"))
YAML = os.path.join(REPO, "ros2_ws/src/control/config/manipulator.yaml")
OUT_DIR = os.environ.get("TEACH_OUT", os.path.expanduser("~/실측"))
REC = os.environ.get("TEACH_REC", os.path.join(OUT_DIR, "slot_teach.json"))
BOX_REC = os.path.join(OUT_DIR, "box_teach.json")
BOX_TAUGHT = os.environ.get("TEACH_BOX_OUT", os.path.expanduser("~/place_box_teach.json"))   # 노드 place.teach.file 로 넣을 파일
BOX_CELL0 = (0.2853, 0.0, 0.0633)   # 2026-11-15 상자 = 로봇 앞 끝 0.1328 + 5 cm + 반 변 0.1025 (예전 12.72 cm 는 0.3625), --cx 로 바꿈. 책상 시험 가운데 칸 (dice.sh --bench 와 같음): 놓을 윗면 높이 = base_link z
BOX_PITCH = 0.06
BOX_ORDER = ["-1,1", "1,1", "-1,0", "0,0", "1,0", "-1,-1", "0,-1", "1,-1", "0,1"]
ORIG = os.environ.get("TEACH_ORIG", os.path.expanduser("~/pallet_place_traj_orig.json"))   # 노드 궤적을 바꾸기 전 원본 (--make 는 늘 이걸 기준으로)
TAUGHT = os.environ.get("TEACH_TAUGHT", os.path.expanduser("~/pallet_place_traj_taught.json"))
ORDER = ["1_3", "1_0", "1_2", "1_1", "2_2", "2_0", "2_3", "2_1", "3_1", "3_0"]   # 계획기 꺼내기 차례 (앞 열부터)


def offsets(force=False):
    """노드가 팔레트 궤적에 더하는 값: pallet.apply_joint_offsets 가 true 면 ik.joint_offsets, false 면 0 (#273, 11-14).
    force=True 면 설정과 관계없이 ik.joint_offsets."""
    import yaml
    d = yaml.safe_load(open(YAML, encoding="utf-8"))
    node = next(iter(d.values()))["ros__parameters"]
    if not force and not node.get("pallet", {}).get("apply_joint_offsets", True):
        return [0.0] * 5
    return [float(v) for v in node["ik"]["joint_offsets"]][:5]


# 집게 값 <-> 손가락 사이 (mm): 실측 3 점 (-0.764 = 66.7, 0.0798 = 36.4 (#190), 0.922 = 23.8) 사이 직선.
#   (meas_session '집게 42' 의 직선식은 0.21 rad = 실물 약 34 mm 라 거의 안 벌어짐 -> 퍽을 누름, 11-14 실물)
GRIP_PTS = [(-0.764, 66.7), (0.0798, 36.4), (0.922, 23.8)]


def grip_for_mm(mm):
    (g0, w0), (g1, w1), (g2, w2) = GRIP_PTS
    if mm >= w1:
        return g0 + (w0 - mm) / (w0 - w1) * (g1 - g0)
    return g1 + (w1 - mm) / (w1 - w2) * (g2 - g1)


TEACH_OPEN_MM = 50.0   # 맞출 때 벌림 (퍽 30 mm + 양쪽 10 mm)


def straight_weights(place, d, margin=0.02):
    """place 점 (위 -> 아래) 마다 보정 비율 w: 아래에서부터 q + d 가 관절 한계 (margin 안쪽) 안인 동안 1, 그 위는 0 -> 1 직선.
    각 점에서 한계를 넘으면 넘지 않는 만큼으로 줄임."""
    lo, hi = limits()
    n = len(place)
    ok = [all(lo[j] + margin <= place[i][j] + d[j] <= hi[j] - margin for j in range(5)) for i in range(n)]
    k = n                                   # k = 전부 보정 구간 시작 (위 -> 아래 번호)
    while k > 0 and ok[k - 1]:
        k -= 1
    w = [1.0] * n
    for i in range(k):
        w[i] = (i + 1) / (k + 1)
        for j in range(5):                  # 한계 넘지 않게
            if d[j] > 0 and place[i][j] + w[i] * d[j] > hi[j] - margin:
                w[i] = max(0.0, min(w[i], (hi[j] - margin - place[i][j]) / d[j]))
            if d[j] < 0 and place[i][j] + w[i] * d[j] < lo[j] + margin:
                w[i] = max(0.0, min(w[i], (lo[j] + margin - place[i][j]) / d[j]))
    return w


def limits():
    """manipulator.yaml arm.lower_limits · upper_limits (관절 1 ~ 5, 노드가 경유점을 거부하는 기준)."""
    import yaml
    node = next(iter(yaml.safe_load(open(YAML, encoding="utf-8")).values()))["ros__parameters"]
    return node["arm"]["lower_limits"][:5], node["arm"]["upper_limits"][:5]


def load_traj(base=False):
    """base=True: 보정 계산 기준 = 원본 (ORIG 가 있으면, 없으면 지금 노드 파일). False: 지금 노드가 쓰는 파일."""
    t = json.load(open(ORIG if base and os.path.exists(ORIG) else TRAJ, encoding="utf-8"))
    return t, {f"{s['row']}_{s['index']}": s for s in t["slots"]}


def seg(s, name):
    return next(g for g in s["segments"] if g["name"] == name)["q"]


def load_rec():
    return json.load(open(REC, encoding="utf-8")) if os.path.exists(REC) else {}


def fk_mm(m, q):
    p, _ = m.fk(list(q) + [0.0] * (5 - len(q)))
    return [v * 1000 for v in p]


def show(m=None):
    rec = load_rec()
    if not rec:
        print("기록 없음:", REC); return
    if m is None:
        sys.path.insert(0, os.path.join(REPO, "tools/arm_bench/real")); import meas_session as m   # noqa: E401
    print("칸   | 관절 보정 (도) j1 j2 j3 j4 | 손끝 차이 (mm, 노드 쥐는 자리 -> 맞춘 자리) x y z | 시각")
    for k in ORDER:
        if k in rec:
            r = rec[k]
            a, b = fk_mm(m, r["q_node"]), fk_mm(m, r["q_meas"])
            print(f"{k}  | " + " ".join(f"{math.degrees(v):+5.1f}" for v in r["delta"][:4])
                  + " | " + " ".join(f"{y - x:+6.1f}" for x, y in zip(a, b)) + f" | {r['t']}")


def make():
    t, slots = load_traj(base=True)
    rec = load_rec()
    if not rec:
        sys.exit(f"기록 없음: {REC} (먼저 칸을 맞추세요)")
    off = offsets()
    for k, r in rec.items():   # 보정 = 맞춘 관절 - 원본 쥐는 자리 (노드 파일이 이미 바뀌었어도 두 번 더하지 않음)
        g = [v + o for v, o in zip(seg(slots[k], "place")[-1][:5], off)]
        r["delta"] = [x - y for x, y in zip(r["q_meas"], g)]
    rows = {}
    for k, r in rec.items():
        rows.setdefault(k[0], []).append(r["delta"])
    used = {}
    for k, s in slots.items():
        if k in rec:
            d, how = rec[k]["delta"], "맞춤"
        elif k[0] in rows:
            ds = rows[k[0]]
            d, how = [sum(x[j] for x in ds) / len(ds) for j in range(5)], f"{k[0]} 열 평균"
        else:
            used[k] = "그대로"; continue
        used[k] = how
        place_w = straight_weights(seg(s, "place"), d)   # place 위 -> 아래 점마다 보정 비율
        for g in s["segments"]:
            for i, q in enumerate(g["q"]):
                # 2026-11-15: 칸 위에서 곧게 내려가게. 아래 (쥐는 자리) 에서부터 관절 한계가 허락하는 높이까지는 측정 차이를 '전부'
                #   (= 실물에서 칸 축을 따라 곧게), 그 위는 0 -> 1 로 서서히 (높이 이동 구간은 그대로, 관절 4 한계 2.0).
                #   (11-14 판은 place 전체를 0 -> 1 로 서서히 넣어 내려가는 길이 휘어, 실물 1_3 · 2_0 · 2_1 에서 집게가 퍽 윗면에 먼저 닿음
                #    -> 관절 2 가 0.15 ~ 0.24 rad 위에서 멈춤, 2_0 빈손)
                if g["name"] == "place":
                    f = place_w[i]
                elif g["name"] == "release":
                    f = 1.0
                elif g["name"] == "retract":   # place 를 거꾸로 (맨 아래 점 빼고)
                    f = place_w[::-1][1:][i] if i < len(place_w) - 1 else 0.0
                else:
                    f = 0.0
                for j in range(5):
                    q[j] = round(q[j] + f * d[j], 5)
        if "q_front" in s:
            pass                               # 꺼내기 끝 앞 자세는 그대로 (칸에서 먼 곳)
    lo, hi = limits()
    bad = [f"{s['row']}_{s['index']} {g['name']} 관절 {j + 1} {q[j]:+.3f}" for s in t["slots"] for g in s["segments"]
           for q in g["q"] for j in range(5) if not lo[j] <= q[j] <= hi[j]]
    if bad:
        sys.exit(f"관절 한계 (manipulator.yaml arm.lower/upper_limits) 밖 경유점 {len(bad)} 개 -> 만들지 않음: " + ", ".join(bad[:6]))
    t["note"] = (f"{datetime.datetime.now():%Y-%m-%d %H:%M} slot_teach.py 실물 칸 맞춤 반영 ("
                 + ", ".join(f"{k} {v}" for k, v in sorted(used.items())) + "). 원본: " + (ORIG if os.path.exists(ORIG) else TRAJ) + "  ") + t["note"]
    json.dump(t, open(TAUGHT, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("만듦:", TAUGHT)
    for k in ORDER:
        print(f"  {k}: {used.get(k)}")
    print("\n노드에 넣기 (원본은 백업 · 되돌리기는 git checkout):")
    print(f"  cp {TRAJ} {ORIG} && cp {TAUGHT} {TRAJ}")
    print("  -> 브링업 다시 띄우기 (노드가 시작할 때 읽음). 되돌리기: "
          f"cd {REPO} && git checkout ros2_ws/src/control/config/pallet_place_traj.json")


def teach(names):
    sys.path.insert(0, os.path.join(REPO, "tools/arm_bench/real"))
    import meas_session as m                   # Arm (arm_controller 액션) · jog · fk 그대로
    doc, slots = load_traj()
    off = offsets()
    vertical = any("enter_tilt_deg" in s for s in doc["slots"])
    bad = [k for k in names if k not in slots]
    if bad:
        sys.exit(f"없는 칸 {bad} (팔레트 v5: {' '.join(ORDER)})")
    m.P(f"\n===== 칸 맞추기 {datetime.datetime.now():%Y-%m-%d %H:%M} 칸 {' '.join(names)} | ik.joint_offsets {off} =====")
    a = m.Arm()
    q, _ = a.show()
    if q[1] < -1.9:   # STOW (2026-11-18 부터 브링업 뒤 팔이 STOW 에 머묾) -> 노드 단계 펴기
        if not m.node_unfold():
            sys.exit("STOW 에서 pose DRIVE 실패 -> 노드 로그 확인 뒤 다시")
        q, _ = a.show()
        a.cmd = list(q)   # 2026-11-22 실물: 갱신 안 하면 a.cmd 가 STOW 로 남아 '집게 벌림' 이 팔을 STOW 자세로 다시 접음
    rec = load_rec()
    os.makedirs(OUT_DIR, exist_ok=True)
    try:
        for k in names:
            s = slots[k]
            pl = seg(s, "place")
            above = [v + o for v, o in zip(pl[0][:5], off)]       # 노드가 보내는 칸 위 (모델 + 보정)
            grasp = [v + o for v, o in zip(pl[-1][:5], off)]      # 노드가 보내는 쥐는 자리
            m.P(f"\n[{k}] {s['shape']} 칸 — 이 칸에만 퍽을 넣었는지 확인. 칸 위 (노드 값) 로 감")
            if vertical:
                m.P(f"  수직 판 맞춤 기준: 퍽이 두 손가락 정가운데, 손가락 가운데가 퍽 중심보다 10 mm 위 (toppick 쥐는 높이), "
                    f"집게 {s.get('enter_tilt_deg', 5):.0f}° (거의 수직) 그대로")
            if m.ask("시작? (y = 시작 / s = 건너뜀)") != "y":
                continue
            a.move(a.cmd, grip_for_mm(TEACH_OPEN_MM), label=f"집게 {TEACH_OPEN_MM:.0f} mm 벌림")
            if not a.move(above, label=f"{k} 칸 위 (노드 값)"):
                continue
            if vertical:   # 2026-11-22 실물: 수직 판 노드 쥐는 자리가 모델 오차로 칸 벽 · 바닥을 누를 수 있음 -> 30 mm 위에서 멈춤
                m.P("  노드 쥐는 자리 30 mm 위로 내릴까? (거기서 '아래' · '멀리' 등으로 맞춤)")
                hi, err = m.cart_step(grasp, [0.0, 0.0, 0.03])
                if m.ask("내림? (y / n)") == "y":
                    a.move(hi if err < 1.0 else above, label=f"{k} 쥐는 자리 30 mm 위 (노드 값)" if err < 1.0 else "30 mm 위 못 풂 -> 칸 위 그대로")
            else:
                m.P("  노드 쥐는 자리로 내릴까? (퍽 바로 위면 '아래' 로 직접 내려도 됨)")
                if m.ask("내림? (y / n)") == "y":
                    a.move(grasp, label=f"{k} 쥐는 자리 (노드 값)")
            qm = m.jog(a, above, f"{k} 칸 위")
            if qm is not None:
                delta = [x - y for x, y in zip(qm, grasp)]
                rec[k] = {"q_meas": qm, "q_node": grasp, "delta": delta, "t": f"{datetime.datetime.now():%m-%d %H:%M}"}
                json.dump(rec, open(REC, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
                a_mm, b_mm = fk_mm(m, grasp), fk_mm(m, qm)
                m.P(f"[기록] {k} 관절 보정 (도) " + " ".join(f"{math.degrees(v):+.1f}" for v in delta)
                    + " | 손끝 차이 (mm) " + " ".join(f"{y - x:+.1f}" for x, y in zip(a_mm, b_mm)))
                if m.ask("쥐어 볼까? (y = 닫고 2 초 뒤 열기 / n)") == "y":
                    a.move(a.cmd, 0.923, confirm=False, T=1.5, label="닫기")
                    _, g = a.show("쥔 집게")
                    m.P(f"  집게 {g:+.3f} rad ({'쥠 범위 안' if 0.30 <= g <= 0.773 else '쥠 범위 밖 — 빈손이거나 너무 큼'})")
                    a.move(a.cmd, grip_for_mm(TEACH_OPEN_MM), confirm=False, T=1.5, label=f"집게 열기 {TEACH_OPEN_MM:.0f} mm")
            a.move(above, label="칸 위로 올림")
            a.move([above[0]] + m.HIGH, label="높은 자세")
    except m.Back:
        m.P("  (끝냄)")
    a.move([0] + m.HIGH, label="정면 높은 자세")
    m.P(f"기록: {REC} (칸 {len(rec)} 개). 다음: python3 ~/slot_teach.py --make")


def set_nudge(slot, d, mm, base):
    """맞춰 보지 않고 말로 보정: 칸 slot 의 쥐는 자리에서 방향 d (meas_session jog 와 같은 이름) 로 mm 옮긴 자리를 기록.
    base = node (노드가 지금 보내는 자리 기준) | offset (ik.joint_offsets 를 더한 자리 기준 — 11-14 이전 slot_teach 가 간 자리)."""
    sys.path.insert(0, os.path.join(REPO, "tools/arm_bench/real")); import meas_session as m   # noqa: E401
    _, slots = load_traj(base=True)
    if slot not in slots:
        sys.exit(f"없는 칸 {slot}")
    g = seg(slots[slot], "place")[-1][:5]
    seen = [v + o for v, o in zip(g, offsets(force=(base == "offset")))]
    node = [v + o for v, o in zip(g, offsets())]
    if d in ("멀리", "가까이"):
        p, _ = m.fk(seen); ux, uy = p[0] - 0.096, p[1]; nn = math.hypot(ux, uy) or 1.0
        v = ((1 if d == "멀리" else -1) * ux / nn, (1 if d == "멀리" else -1) * uy / nn, 0)
    elif d in m.DIRS:
        v = m.DIRS[d]
    else:
        sys.exit(f"방향은 {' · '.join(list(m.DIRS) + ['멀리', '가까이'])}")
    qm, err = m.cart_step(seen, [c * mm / 1000 for c in v])
    if err > 1.0:
        sys.exit("그 방향으로 IK 안 됨")
    rec = load_rec()
    rec[slot] = {"q_meas": qm, "q_node": node, "delta": [x - y for x, y in zip(qm, node)],
                 "t": f"{datetime.datetime.now():%m-%d %H:%M}", "how": f"{base} 자리에서 {d} {mm:g} mm (말로)"}
    os.makedirs(OUT_DIR, exist_ok=True)
    json.dump(rec, open(REC, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"[기록] {slot}: {rec[slot]['how']}")
    show(m)


def _pt():
    """팀 팔레트 궤적 생성기의 팀 URDF 팔 (IK · FK)."""
    sys.path.insert(0, os.path.join(REPO, "urdf_and_usd/policy/manipulator"))
    import pallet_traj as PT
    return PT, PT.Arm(PT.URDF)


BOX_TARGET_BIAS = (0.0, 0.0018, 0.002)   # box_target 이 칸 가운데 · 윗면에 더하는 값 (옛 노드 place_at 옆 1.8 mm · 틈 2 mm)


def box_cell(cell):
    """셀 'dx,dy' -> 칸 가운데 · 상자 윗면 (base_link) = 손으로 맞춘 퍽 바닥 자리 (맞춤 기준: 바닥이 윗면에 막 닿고 칸 가운데)."""
    dx, dy = (int(v) for v in cell.split(","))
    return [BOX_CELL0[0] + dy * BOX_PITCH, BOX_CELL0[1] - dx * BOX_PITCH, BOX_CELL0[2]]


def box_target(cell):
    """셀 'dx,dy' (눈 그림: dy+ = 먼 쪽, dx+ = 로봇 오른쪽) -> 맞추기 시작 자리 = 퍽 바닥 (base_link, 노드 place_at 과 같게: 옆 1.8 mm 왼쪽 · 틈 2 mm)."""
    return [v + b for v, b in zip(box_cell(cell), BOX_TARGET_BIAS)]


def box_q(PT, arm, p_bottom, offs):
    """노드처럼: 45° 집게로 퍽 바닥 = p 가 되는 모델 관절 + ik.joint_offsets (명령)."""
    import numpy as np
    yaw = math.atan2(p_bottom[1] - arm.j1[1], p_bottom[0] - arm.j1[0])
    q, e, t = arm.ik_down(np.array(p_bottom), yaw, None, off=PT.BOTTOM, tilt=math.radians(45.0))
    return [v + o for v, o in zip(q, offs)], e


def teach_box(cells):
    sys.path.insert(0, os.path.join(REPO, "tools/arm_bench/real"))
    import meas_session as m
    PT, arm = _pt()
    offs = offsets(force=True)            # 노드 놓기 IK 는 ik.joint_offsets 를 씀 (팔레트와 달리)
    m.P(f"\n===== 상자 놓는 자리 맞추기 {datetime.datetime.now():%Y-%m-%d %H:%M} 셀 {' '.join(cells)} | ik.joint_offsets {offs} =====")
    m.P("  상자 (205 x 205 x 70) = 로봇 앞 끝에서 5 cm (5 ~ 6 cm, 다르면 --cx 상자 가운데 x), 가운데 맞춤. 셀 '1,1' = 먼 줄 왼쪽 (로봇 기준), '0,0' = 가운데, '1,-1' = 가까운 줄 오른쪽")
    m.P("  맞춤 기준: 쥔 퍽 바닥이 상자 윗면에 막 닿고, 퍽 중심이 그 칸 가운데 (칸 표시 · 자). 집게 45° 그대로")
    a = m.Arm()
    rec = json.load(open(BOX_REC, encoding="utf-8")) if os.path.exists(BOX_REC) else {}
    try:
        a.hold_puck()
        for c in cells:
            p = box_target(c)
            q, e = box_q(PT, arm, p, offs)
            qa, _ = box_q(PT, arm, [p[0], p[1], p[2] + 0.04], offs)
            if e > 0.002:
                m.P(f"  [{c}] IK 안 됨 (오차 {e * 1000:.1f} mm) -> 건너뜀"); continue
            m.P(f"\n[{c}] 목표 퍽 바닥 (base_link mm) {[round(v * 1000, 1) for v in p]}")
            if m.ask("시작? (y / s)") != "y":
                continue
            a.move(qa[:5], label=f"{c} 4 cm 위 (노드 값)")
            a.move(q[:5], label=f"{c} 놓는 자리 (노드 값)")
            qm = m.jog(a, q[:5], f"{c} 놓는 자리")
            if qm is not None:
                rec[c] = {"q_meas": qm, "q_node": q[:5], "target": p, "bottom": box_cell(c), "cx": BOX_CELL0[0],
                          "t": f"{datetime.datetime.now():%m-%d %H:%M}"}
                os.makedirs(OUT_DIR, exist_ok=True)
                json.dump(rec, open(BOX_REC, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
                m.P(f"[기록] {c}")
            a.move(qa[:5], label="4 cm 위로")
    except m.Back:
        m.P("  (끝냄)")
    m.P(f"기록: {BOX_REC} (셀 {len(rec)} 개). 다음: python3 ~/slot_teach.py --fit_box")


def fit_box():
    import numpy as np
    from scipy.optimize import least_squares
    PT, arm = _pt()
    rec = json.load(open(BOX_REC, encoding="utf-8")) if os.path.exists(BOX_REC) else {}
    if len(rec) < 3:
        sys.exit(f"상자 기록이 {len(rec)} 개 — 3 개 이상 필요 ({BOX_REC})")
    rows = []
    for c, v in rec.items():
        t = np.array(v["target"]) + np.array([0.0, 0.0, PT.GRASP_DZ + PT.PUCK_H / 2])   # 퍽 바닥 -> TCP
        yaw = math.atan2(t[1] - arm.j1[1], t[0] - arm.j1[0])
        a_dir = np.array([math.sin(math.radians(45)) * math.cos(yaw), math.sin(math.radians(45)) * math.sin(yaw), -math.cos(math.radians(45))])
        rows.append((c, np.array(v["q_meas"][:5], dtype=float), t, a_dir))

    def res(x):
        out = []
        for _, q, t, a_dir in rows:
            qq = q.copy(); qq[:4] -= x
            p, R = arm.tcp(qq)
            out += list((p - t) * 1000.0) + list((R[:, 0] - a_dir) * 20.0)
        return np.array(out)
    old = np.array(offsets(force=True)[:4])
    for name, x in (("지금 ik.joint_offsets", old), ("맞춘 값", least_squares(res, old).x)):
        rr = res(x).reshape(-1, 6)
        print(f"{name} (rad) {[round(float(v), 4) for v in x] + [0.0]} | 셀별 놓는 자리 오차 mm: "
              + ", ".join(f"{c} {np.linalg.norm(e[:3]):.1f}" for (c, *_), e in zip(rows, rr)))
    print("-> 맞춘 값을 manipulator.yaml ik.joint_offsets 에 넣으면 노드 IK (상자 놓기 · 내려다보기 · 집기 접근) 가 이 값을 씀 (팔레트 궤적은 따로)")


def make_box():
    """상자 기록 -> 노드 place.teach 파일 (base_link 퍽 바닥 · 손 맞춤 관절). 노드가 칸마다 모델 오차 = 바닥 자리 − 노드 FK 를 구해 놓기 IK 목표에서 뺌."""
    rec = json.load(open(BOX_REC, encoding="utf-8")) if os.path.exists(BOX_REC) else {}
    if len(rec) < 3:
        sys.exit(f"상자 기록이 {len(rec)} 개 — 3 개 이상 필요, 9 칸 권장 ({BOX_REC})")
    cells = {}
    for c, v in sorted(rec.items()):
        # bottom 없는 예전 기록 (11-17 전): 시작 자리 target 에서 옛 치우침을 빼서 칸 가운데 · 윗면으로
        b = v.get("bottom") or [t - d for t, d in zip(v["target"], BOX_TARGET_BIAS)]
        cells[c] = {"bottom": [round(x, 5) for x in b], "q": [round(x, 5) for x in v["q_meas"][:5]], "t": v.get("t", "")}
        print(f"  {c}: 퍽 바닥 (base_link mm) {[round(x * 1000, 1) for x in b]}  상자 가운데 x {v.get('cx', '기록 없음')}")
    out = {"note": "slot_teach.py --box 손 맞춤 (쥔 퍽 바닥이 상자 윗면에 막 닿고 칸 가운데). bottom = 그 퍽 바닥 (frame), q = /joint_states 관절 1 ~ 5 (명령값). "
                   "노드 place.teach 가 칸마다 모델 오차 (bottom + 퍽 반 높이 + 잡는 높이 − 노드 FK(q)) 를 구해 놓기 IK 목표에서 뺌",
           "frame": "base_link", "made": f"{datetime.datetime.now():%Y-%m-%d %H:%M}", "cells": cells}
    json.dump(out, open(BOX_TAUGHT, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"만듦: {BOX_TAUGHT} (칸 {len(cells)} 개)")
    if len(cells) < 6:
        print("  주의: 6 칸 미만이면 노드가 2 차 대신 1 차로 맞춤 (칸 사이 오차 더 남음). 9 칸 권장")
    print("\n노드에 넣기:")
    print(f"  cp {BOX_TAUGHT} {os.path.join(REPO, 'ros2_ws/src/control/config/place_box_teach.json')}")
    print("  manipulator.yaml place.teach.enabled: true -> colcon build --packages-select control -> 브링업 다시")
    print("  확인: 첫 놓기 때 노드 로그 'place.teach: cell … model error (…) mm' · 'place.teach: N / N cell(s) in use'")


def main():
    global BOX_CELL0
    args = sys.argv[1:]
    if "--cx" in args:   # 상자 가운데 x (base_link, m). 6 cm 면 0.2953
        i = args.index("--cx")
        BOX_CELL0 = (float(args[i + 1]), BOX_CELL0[1], BOX_CELL0[2])
        del args[i:i + 2]
    if "--box" in args:
        teach_box([x for x in args if not x.startswith("-")] or BOX_ORDER)
        return
    if "--fit_box" in args:
        fit_box()
        return
    if "--make_box" in args:
        make_box()
        return
    if args and args[0] == "--set":   # --set 3_1 왼 5 [offset]
        set_nudge(args[1], args[2], float(args[3]), args[4] if len(args) > 4 else "node")
        return
    if "--make" in args:
        make()
    elif "--show" in args:
        show()
    else:
        teach([x for x in args if not x.startswith("-")] or ORDER)


if __name__ == "__main__":
    main()
