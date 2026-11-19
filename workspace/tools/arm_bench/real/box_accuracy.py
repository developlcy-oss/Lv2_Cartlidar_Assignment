#!/usr/bin/env python3
"""상자 위 팔 끝 위치 정확도 측정 (2026-11-19, #292 Onbeom 요청) — slot_teach.py 는 그대로 두고 따로 만든 측정 도구.

목적: 노드가 상자 위에 놓을 때 (place_at · box_look 과 같은 IK + 지금 ik.joint_offsets) 실제 끝점이 목표에서 얼마나 어긋나는지를
      **자로 독립 측정**하고, 반복 흩어짐 · 접근 방향 · 하중 (빈 집게 / 퍽) 의 영향을 나눠 기록한다.

정의 (모든 기록 공통)
- 좌표계: base_link (로봇 기준 앞 +x · 왼 +y · 위 +z), 단위 mm. 상자 칸 = slot_teach.py --box 와 같음
  (가운데 칸 (0,0) = BOX_CELL0, 칸 간격 60 mm, 셀 'dx,dy': dy+ = 먼 쪽, dx+ = 로봇 오른쪽).
- 목표 점 = **쥔 퍽 바닥 가운데** (노드 place_at 과 같음: 집게 45° 기울임, TCP = gripper_roll_link 앞 0.14 m = 퍽 중심 + 15 mm,
  퍽 바닥 = TCP 아래 45 mm, 상자 윗면 + 2 mm 틈). 빈 집게 (load empty) 는 **두 손가락 끝 가운데**를 재고 기록에 점 이름을 남김.
- 자 측정 d = 실측 − 목표: dx = 앞 + / 뒤 −, dy = 왼 + / 오른 −, dz = 퍽 바닥이 상자 윗면에서 위 몇 mm (목표 2 mm) − 2.
- 노드 값 = 모델 IK + ik.joint_offsets (manipulator.yaml, --offsets 로 시험값을 줄 수 있음 — yaml 은 안 바꿈).

기록: ~/실측/box_acc/box_acc_<날짜>.jsonl (한 시도 = 한 줄, 덮어쓰지 않음). 손 맞춤 (jog) 을 하면 q_meas 도 남아 fit 에 씀.

실행 (젯슨, 브링업 + 리셋 뒤. 팔이 STOW 면 노드에 pose DRIVE 를 먼저 보냄)
  python3 box_accuracy.py run  [--reps 3] [--approach top|left|right|far|near|all] [--cells 0,0 1,1 ...] [--load puck|empty]
                               [--no_jog] [--offsets a,b,c,d,e] [--cx 0.2853] [--tool "자 ±1 mm"]
  python3 box_accuracy.py sag  [--cell 0,0] [--reps 3]        같은 칸 · 같은 노드 값에서 빈 집게 ↔ 퍽 쥔 손가락 끝 비교
  python3 box_accuracy.py report [파일 ...]                  칸별 평균 편차 · 반복 σ · 접근 방향 · 맞춤용/확인용 RMS · 처짐 비교
  python3 box_accuracy.py fit    [파일 ...]                  손 맞춤 (q_meas) 으로 영점 맞춤 (맞춤용 칸만) → 모델 잔차 · 확인용 칸 예측 (적용 안 함)
기본 칸: 맞춤용 0,0 · -1,1 · 1,1 · -1,-1 · 1,-1 · 0,1, 확인용 -1,0 · 1,0 · 0,-1 (--cells 로 바꾸면 그 칸은 BOX_ROLE 대로)
"""
import datetime
import glob
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
os.environ.setdefault("TEAM_WS", os.path.normpath(os.path.join(HERE, "..", "..", "..")))   # slot_teach 의 REPO = 이 저장소
sys.path.insert(0, HERE)
import slot_teach as ST   # noqa: E402  (box_target · box_q · offsets · _pt 그대로 씀)

OUT_DIR = os.path.join(os.path.expanduser("~/실측"), "box_acc")
FIT_CELLS = ["0,0", "-1,1", "1,1", "-1,-1", "1,-1", "0,1"]
CHECK_CELLS = ["-1,0", "1,0", "0,-1"]
BOX_ROLE = {**{c: "fit" for c in FIT_CELLS}, **{c: "check" for c in CHECK_CELLS}}
APPROACH = {   # 놓는 자리로 들어가기 전 점 (퍽 바닥 목표에서, m): 위 4 cm + 옆 3 cm
    "top": (0.0, 0.0, 0.04), "left": (0.0, 0.03, 0.04), "right": (0.0, -0.03, 0.04),
    "far": (0.03, 0.0, 0.04), "near": (-0.03, 0.0, 0.04)}
SETTLE = 1.0   # s, 놓는 자리 도착 뒤 처짐이 가라앉을 때까지


def opt(args, name, default=None, n=1):
    if name not in args:
        return default
    i = args.index(name)
    v = args[i + 1:i + 1 + n]
    del args[i:i + 1 + n]
    return v[0] if n == 1 else v


def flag(args, name):
    if name in args:
        args.remove(name)
        return True
    return False


def out_file():
    os.makedirs(OUT_DIR, exist_ok=True)
    return os.path.join(OUT_DIR, f"box_acc_{datetime.datetime.now():%m%d}.jsonl")


def save(row):
    with open(out_file(), "a", encoding="utf-8") as f:
        f.write(json.dumps(row, ensure_ascii=False) + "\n")


def unfold_if_stow(m, a):
    """2026-11-18 부터 브링업 뒤 팔이 STOW 에 머묾 -> 노드에 pose DRIVE (노드가 FOLD_VIA 를 거쳐 단계 순서로 폄)."""
    q, _ = a.read()
    if q[1] > -1.9:
        return
    m.P("  팔이 STOW -> 노드에 pose DRIVE (단계 펴기)")
    out = subprocess.run(["ros2", "action", "send_goal", "/manipulator/pose", "robot_interfaces/action/ArmPose", "{name: DRIVE}"],
                         capture_output=True, text=True, timeout=180).stdout
    if "success: true" not in out:
        sys.exit("pose DRIVE 실패 -> 노드 로그 확인 ('startup: arm at STOW' 뒤인지, 리셋 했는지)")
    a.cmd, a.grip = a.read()


def ask_mm(m, prompt):
    return m.ask(prompt, float, empty_ok=True)


def ruler(m, point):
    m.P(f"  자로 잼: {point} 의 목표 대비 (칸 표시 기준, 로봇 기준 방향, mm, 못 재면 Enter)")
    dx = ask_mm(m, "앞 + / 뒤 −  (dx)")
    dy = ask_mm(m, "왼 + / 오른 −  (dy)")
    h = ask_mm(m, "상자 윗면에서 위로 몇 mm (닿으면 0)")
    return dx, dy, h


def node_pose(PT, arm, p, offs, pre):
    q, e = ST.box_q(PT, arm, p, offs)
    qa, ea = ST.box_q(PT, arm, [p[0] + pre[0], p[1] + pre[1], p[2] + pre[2]], offs)
    return q, qa, max(e, ea)


def run(args):
    import meas_session as m
    reps = int(opt(args, "--reps", "3"))
    appr = opt(args, "--approach", "top")
    load = opt(args, "--load", "puck")
    tool = opt(args, "--tool", None)
    no_jog = flag(args, "--no_jog")
    cx = opt(args, "--cx", None)
    off_s = opt(args, "--offsets", None)
    cells = []
    if "--cells" in args:
        i = args.index("--cells")
        cells = [c for c in args[i + 1:] if not c.startswith("--")]
        del args[i:i + 1 + len(cells)]
    cells = cells or FIT_CELLS + CHECK_CELLS
    if cx:
        ST.BOX_CELL0 = (float(cx), ST.BOX_CELL0[1], ST.BOX_CELL0[2])
    approaches = list(APPROACH) if appr == "all" else [appr]
    if any(x not in APPROACH for x in approaches) or load not in ("puck", "empty"):
        sys.exit(f"--approach {'|'.join(APPROACH)}|all, --load puck|empty")
    offs = [float(v) for v in off_s.split(",")] if off_s else ST.offsets(force=True)
    PT, arm = ST._pt()
    m.P(f"\n===== 상자 위 정확도 {datetime.datetime.now():%Y-%m-%d %H:%M} | 칸 {' '.join(cells)} | 반복 {reps} | 접근 {','.join(approaches)} "
        f"| 하중 {load} | offsets {[round(v, 4) for v in offs]}{' (시험값, yaml 아님)' if off_s else ''} | 상자 가운데 x {ST.BOX_CELL0[0]:.4f} =====")
    tool = tool or m.ask("측정 도구 · 대략 오차 (예: '자 ±1 mm', Enter = 자 ±1 mm)", empty_ok=True) or "자 ±1 mm"
    point = "쥔 퍽 바닥 가운데" if load == "puck" else "두 손가락 끝 가운데"
    a = m.Arm()
    unfold_if_stow(m, a)
    if load == "puck":
        a.hold_puck()
    else:
        a.move(a.cmd, m.CLOSE, label="집게 닫기 (빈 집게)")
    n = 0
    try:
        for rep in range(1, reps + 1):
            for ap in approaches:
                for c in cells:
                    p = ST.box_target(c)
                    q, qa, e = node_pose(PT, arm, p, offs, APPROACH[ap])
                    if e > 0.002:
                        m.P(f"  [{c}] IK 안 됨 ({e * 1000:.1f} mm) -> 건너뜀")
                        continue
                    role = BOX_ROLE.get(c, "fit")
                    m.P(f"\n[{c} · {role} · {ap} · {rep}/{reps}] 목표 {point} (base_link mm) {[round(v * 1000, 1) for v in p]}")
                    if m.ask("시작? (y / s 건너뜀)") != "y":
                        continue
                    a.move(qa[:5], label=f"{c} 들어가기 전 ({ap})")
                    a.move(q[:5], label=f"{c} 놓는 자리 (노드 값)")
                    a.wait(SETTLE)
                    q_read, g_read = a.read(1.0)
                    dx, dy, h = ruler(m, point)
                    q_meas = None
                    if not no_jog and load == "puck" and m.ask("손으로 맞춰 기록할까? (y / n)") == "y":
                        q_meas = m.jog(a, q[:5], f"{c} 놓는 자리")
                    save({"t": f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S}", "cell": c, "role": role, "rep": rep,
                          "approach": ap, "load": load, "point": point, "tool": tool,
                          "target_mm": [round(v * 1000, 2) for v in p], "offsets": offs, "offsets_trial": bool(off_s),
                          "q_node": q[:5], "q_read": q_read, "grip_read": g_read, "q_meas": q_meas,
                          "ruler_mm": {"dx": dx, "dy": dy, "dz": None if h is None else h - 2.0, "h_above_top": h}})
                    n += 1
                    m.P(f"[기록] {c} {ap} {rep} -> {out_file()}")
                    a.move(qa[:5], confirm=False, label="위로")
    except m.Back:
        m.P("  (끝냄)")
    m.P(f"기록 {n} 개 -> {out_file()}. 다음: python3 box_accuracy.py report")


def sag(args):
    import meas_session as m
    cell = opt(args, "--cell", "0,0")
    reps = int(opt(args, "--reps", "3"))
    PT, arm = ST._pt()
    offs = ST.offsets(force=True)
    p = ST.box_target(cell)
    q, qa, e = node_pose(PT, arm, p, offs, APPROACH["top"])
    if e > 0.002:
        sys.exit("IK 안 됨")
    m.P(f"\n===== 처짐 비교 (빈 집게 ↔ 퍽) {datetime.datetime.now():%Y-%m-%d %H:%M} | 칸 {cell} | 반복 {reps} =====")
    m.P("  두 조건 모두 **두 손가락 끝 가운데**를 같은 칸 표시 기준으로 잼 (같은 노드 값 자세)")
    tool = m.ask("측정 도구 · 대략 오차 (Enter = 자 ±1 mm)", empty_ok=True) or "자 ±1 mm"
    a = m.Arm()
    unfold_if_stow(m, a)
    try:
        for rep in range(1, reps + 1):
            for load in ("empty", "puck"):
                if load == "puck":
                    a.hold_puck()
                    mass = m.ask("퍽 종류 · 무게 (예 '원기둥 13 g')", empty_ok=True)
                else:
                    a.move(a.cmd, m.OPEN, label="집게 열기 (퍽 빼기)")
                    input("    퍽을 빼고 Enter: ")
                    a.move(a.cmd, m.CLOSE, confirm=False, label="빈 집게 닫기")
                    mass = None
                a.move(qa[:5], label="위")
                a.move(q[:5], label="노드 값 자세")
                a.wait(SETTLE)
                q_read, g_read = a.read(1.0)
                dx, dy, h = ruler(m, "두 손가락 끝 가운데")
                save({"t": f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S}", "kind": "sag", "cell": cell, "rep": rep, "load": load,
                      "mass": mass, "point": "두 손가락 끝 가운데", "tool": tool, "target_mm": [round(v * 1000, 2) for v in p],
                      "offsets": offs, "q_node": q[:5], "q_read": q_read, "grip_read": g_read,
                      "ruler_mm": {"dx": dx, "dy": dy, "h_above_top": h}})
                a.move(qa[:5], confirm=False, label="위로")
    except m.Back:
        m.P("  (끝냄)")
    m.P(f"기록 -> {out_file()}. 다음: python3 box_accuracy.py report")


def load_rows(files):
    files = files or sorted(glob.glob(os.path.join(OUT_DIR, "box_acc_*.jsonl")))
    rows = []
    for f in files:
        rows += [json.loads(l) for l in open(f, encoding="utf-8") if l.strip()]
    if not rows:
        sys.exit(f"기록 없음 ({OUT_DIR})")
    return rows


def stats(vals):
    v = [x for x in vals if x is not None]
    if not v:
        return None, None, 0
    mu = sum(v) / len(v)
    sd = math.sqrt(sum((x - mu) ** 2 for x in v) / (len(v) - 1)) if len(v) > 1 else 0.0
    return mu, sd, len(v)


def report(args):
    rows = load_rows(args)
    acc = [r for r in rows if r.get("kind") != "sag"]
    print(f"정확도 시도 {len(acc)} · 처짐 시도 {len(rows) - len(acc)} (자 측정 d = 실측 − 목표, mm, base_link 앞 + · 왼 + · 위 +)")
    groups = {}
    for r in acc:
        groups.setdefault((r["offsets_trial"], tuple(round(v, 4) for v in r["offsets"])), []).append(r)
    for (trial, offs), rs in groups.items():
        print(f"\n== offsets {list(offs)}{' (시험값)' if trial else ' (yaml)'}")
        print("칸      쓰임   접근   n | 평균 dx    dy    dz | 반복 σ dx   dy   dz | 3D 평균 거리")
        keys = sorted({(r["cell"], r["role"], r["approach"]) for r in rs})
        role_d = {"fit": [], "check": []}
        for c, role, ap in keys:
            sub = [r for r in rs if (r["cell"], r["role"], r["approach"]) == (c, role, ap)]
            s = [stats([x["ruler_mm"][k] for x in sub]) for k in ("dx", "dy", "dz")]
            d3 = [math.sqrt(sum((x["ruler_mm"][k] or 0.0) ** 2 for k in ("dx", "dy", "dz"))) for x in sub
                  if all(x["ruler_mm"][k] is not None for k in ("dx", "dy", "dz"))]
            role_d[role] += d3
            fmt = lambda t, i: "   -" if t[i] is None else f"{t[i]:+6.1f}" if i == 0 else f"{t[i]:5.1f}"
            print(f"{c:7s} {role:5s} {ap:5s} {len(sub):2d} | {fmt(s[0], 0)} {fmt(s[1], 0)} {fmt(s[2], 0)} | "
                  f"{fmt(s[0], 1)} {fmt(s[1], 1)} {fmt(s[2], 1)} | {sum(d3) / len(d3) if d3 else float('nan'):5.1f}")
        for role, d in role_d.items():
            if d:
                print(f"  {role:5s} RMS {math.sqrt(sum(x * x for x in d) / len(d)):5.1f} · 최대 {max(d):5.1f} mm (시도 {len(d)})")
    sg = [r for r in rows if r.get("kind") == "sag"]
    if sg:
        print("\n== 처짐 비교 (두 손가락 끝, 같은 노드 값): 빈 집게 ↔ 퍽")
        for load in ("empty", "puck"):
            sub = [r for r in sg if r["load"] == load]
            s = [stats([x["ruler_mm"][k] for x in sub]) for k in ("dx", "dy", "h_above_top")]
            f = lambda v: "  -" if v is None else f"{v:5.1f}"
            print(f"  {load:5s} n {len(sub)} | 평균 dx {f(s[0][0])} dy {f(s[1][0])} 높이 {f(s[2][0])} | σ {f(s[0][1])} {f(s[1][1])} {f(s[2][1])}")
        e = [stats([x["ruler_mm"]["h_above_top"] for x in sg if x["load"] == l])[0] for l in ("empty", "puck")]
        if None not in e:
            print(f"  퍽을 쥐면 손가락 끝 높이 {e[1] - e[0]:+.1f} mm (− = 처짐)")


def fit(args):
    import numpy as np
    from scipy.optimize import least_squares
    rows = [r for r in load_rows(args) if r.get("kind") != "sag" and r.get("q_meas") and r["load"] == "puck"]
    if len(rows) < 3:
        sys.exit(f"손 맞춤 (q_meas) 기록 {len(rows)} 개 — 3 개 이상 필요")
    PT, arm = ST._pt()

    def target(r):
        t = np.array(r["target_mm"]) / 1000 + np.array([0.0, 0.0, PT.GRASP_DZ + PT.PUCK_H / 2])   # 퍽 바닥 -> TCP (slot_teach --fit_box 와 같음)
        yaw = math.atan2(t[1] - arm.j1[1], t[0] - arm.j1[0])
        return t, np.array([math.sin(math.radians(45)) * math.cos(yaw), math.sin(math.radians(45)) * math.sin(yaw), -math.cos(math.radians(45))])

    def err(r, x):
        q = np.array(r["q_meas"][:5], float)
        q[:4] -= x
        p, R = arm.tcp(q)
        t, a_dir = target(r)
        return (p - t) * 1000.0, (R[:, 0] - a_dir) * 20.0

    fit_rows = [r for r in rows if r["role"] == "fit"]
    chk_rows = [r for r in rows if r["role"] == "check"]
    res = lambda x: np.concatenate([np.concatenate(err(r, x)) for r in fit_rows])
    old = np.array(rows[0]["offsets"][:4])
    new = least_squares(res, old).x
    print(f"손 맞춤 {len(rows)} 개 (맞춤용 {len(fit_rows)} · 확인용 {len(chk_rows)}), 모델 = 팀 URDF IK (팔레트 궤적 생성기)")
    for name, x in (("지금 offsets", old), ("맞춘 offsets", new)):
        for lab, rs in (("맞춤용", fit_rows), ("확인용", chk_rows)):
            d = [float(np.linalg.norm(err(r, x)[0])) for r in rs]
            if d:
                print(f"  {name} {lab}: 모델 잔차 RMS {math.sqrt(sum(v * v for v in d) / len(d)):5.1f} · 최대 {max(d):5.1f} mm")
    print(f"→ 맞춘 값 (적용 안 함, 검증용 --offsets): {','.join(f'{v:.4f}' for v in new)},0.0")
    print("  검증: python3 box_accuracy.py run --offsets <위 값> --no_jog  (같은 칸 · 같은 조건으로 자 측정 → report 에서 보정 전후 비교)")


def main():
    args = sys.argv[1:]
    if not args or args[0] not in ("run", "sag", "report", "fit"):
        print(__doc__)
        return
    {"run": run, "sag": sag, "report": report, "fit": fit}[args[0]](args[1:])


if __name__ == "__main__":
    main()
