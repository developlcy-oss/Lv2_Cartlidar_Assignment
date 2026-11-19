#!/usr/bin/env python3
"""갑자기 나타남 시험 결과 모음 (runs/*갑자기* · *장애물없음*): 놓을 때 속도 · 멈출 때까지 · 여유 (6 각형 = 차체 + 팔 끝, 차체만) → 표 · JSON
   python3 sim/sudden_table.py [runs 폴더]   → 화면 표 + runs/sudden_table.json (주행기록 페이지가 읽음)"""
import csv, glob, json, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
R = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(HERE), "runs")
HEX = np.array([[0.25, 0.07], [0.25, -0.07], [0.1328, -0.1676], [-0.2135, -0.1676], [-0.2135, 0.1676], [0.1328, 0.1676]])
BODY = np.array([[0.1328, -0.1676], [0.1328, 0.1676], [-0.2135, 0.1676], [-0.2135, -0.1676]])
PARTS = {"box": [(0, 0, 0.205, 0.205)], "wall": [(0, 0, 0.10, 0.55)], "person": [(0, 0.11, 0.26, 0.10), (0, -0.11, 0.26, 0.10)]}


def bnd(poly, step=0.01):
    out = []
    for a, b in zip(poly, np.roll(poly, -1, 0)):
        n = max(2, int(np.linalg.norm(b - a) / step))
        out.append(a + (b - a) * np.linspace(0, 1, n, endpoint=False)[:, None])
    return np.vstack(out)


def rect_sd(P, cx, cy, yaw, a, b):
    c, s = math.cos(yaw), math.sin(yaw)
    u, v = c * (P[:, 0] - cx) + s * (P[:, 1] - cy), -s * (P[:, 0] - cx) + c * (P[:, 1] - cy)
    qx, qy = np.abs(u) - a / 2, np.abs(v) - b / 2
    return np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0)


def clear(poly, pose, o):
    x, y, th = pose
    B = bnd(poly)
    c, s = math.cos(th), math.sin(th)
    W = np.c_[x + c * B[:, 0] - s * B[:, 1], y + s * B[:, 0] + c * B[:, 1]]
    ox, oy, oyaw = o["pose"]
    co, so = math.cos(oyaw), math.sin(oyaw)
    best = 9.0
    for lx, ly, a, b in PARTS[o["shape"]]:
        best = min(best, float(rect_sd(W, ox + co * lx - so * ly, oy + so * lx + co * ly, oyaw, a, b).min()))
    return best


rows = []
for d in sorted(glob.glob(os.path.join(R, "*/"))):
    base = os.path.basename(d.rstrip("/"))
    if "무시" in base or not ("갑자기" in base or "장애물없음" in base):
        continue
    if not os.path.isfile(os.path.join(d, "truth_summary.txt")):
        continue
    st = open(os.path.join(d, "settings.txt"), encoding="utf-8").read()
    ev = json.load(open(os.path.join(d, "events.json"), encoding="utf-8")) if os.path.isfile(os.path.join(d, "events.json")) else []
    sc = json.load(open(os.path.join(d, "scenario_events.json"), encoding="utf-8")) if os.path.isfile(os.path.join(d, "scenario_events.json")) else {"obstacles": []}
    o = next((x for x in sc["obstacles"] if x.get("pose")), None)
    T = [(float(r["t"]), float(r["x"]), float(r["y"]), math.radians(float(r["yaw"]))) for r in csv.DictReader(open(os.path.join(d, "truth.csv")))]
    row = {"run": base, "guard": "예전" if "scan_merged" in st else "새", "shape": o["shape"] if o else "-", "gap": o.get("gap_m") if o else None,
           "v": o.get("v_at_spawn") if o else None, "travel": o.get("stop_travel_m") if o else None, "stop_after": o.get("stop_after_s") if o else None,
           "n_stop": len(ev), "false_stops": 0, "why": "", "hex": None, "body": None}
    if o:
        t0, t1 = o["spawned"], o["removed"] or T[-1][0]
        seg = [p for p in T if t0 <= p[0] <= t1]
        row["hex"] = round(min(clear(HEX, p[1:], o) for p in seg), 3) if seg else None
        row["body"] = round(min(clear(BODY, p[1:], o) for p in seg), 3) if seg else None
        mine = [e for e in ev if e["t"] >= t0 - 0.2]
        row["why"] = (mine[0].get("guard_why") or mine[0].get("why", "")) if mine else "멈추지 않음"
        row["false_stops"] = len(ev) - len(mine[:1])
    else:
        row["false_stops"] = len(ev)
        row["why"] = " / ".join(sorted({e.get("guard_why", "") for e in ev})) or "-"
    rows.append(row)
json.dump(rows, open(os.path.join(R, "sudden_table.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
f = lambda v, k=2, m=1: "-" if v is None else f"{v * m:.{k}f}"  # noqa: E731
print(f"{'시험':40s} 감시 모양    간격   속도   멈출때까지   시간   여유(팔끝)  여유(차체)  다른멈춤  멈춘조건")
for r in rows:
    print(f"{r['run'][12:52]:40s} {r['guard']:3s} {r['shape']:6s} {f(r['gap'])} m {f(r['v'])} {f(r['travel'], 1, 100):>6s} cm {f(r['stop_after'])} s "
          f"{f(r['hex'], 1, 100):>7s} cm {f(r['body'], 1, 100):>7s} cm  {r['false_stops']:3d}     {r['why']}")
