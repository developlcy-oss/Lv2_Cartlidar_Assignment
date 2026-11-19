"""회전 속도 상한 시나리오: 반경별 상한 + 어제 경기 순서 한 바퀴 (길 모음 경로) 에 입힌 속도 · 시간."""
import json, math, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
LIB = json.load(open(os.path.join(HERE, "..", "..", "drive_demo", "legs.json")))   # 규칙 v2 길 모음 (경기 때 쓴 것)
LEGS = {g["name"]: g for g in LIB["legs"] if g.get("ok")}
ROUTE = ["HOME", "box1", "box2", "box6", "box3", "box5", "box4", "L3", "L2", "L1", "HOME"]
R_W, BELT, SEP = 0.0335, 2.0, 0.317          # 실물 v7 벨트
M_PER = R_W * BELT                           # 모터 1 rad/s → 바퀴 둘레 m/s
M_STRAIGHT = 7.0                             # 직진 모터 (사용자 "직진속도 7")
TURN = 3.0                                   # 잠정 회전 중 모터 상한
VREV, SPIN_W, AX = 0.15, 0.5, 1.0            # 후진 · 제자리 · 가감속 [m/s²]
V_STRAIGHT = M_STRAIGHT * M_PER              # 0.469

def lin(k, k0, m0, k1, m1):
    f = min(max((k - k0) / (k1 - k0), 0.0), 1.0)
    return m0 + (m1 - m0) * f
OPTS = [
    {"id": "now", "name": "지금 (규칙 v2 · 11-16 최고 속도판)", "short": "지금",
     "desc": "직진 0.45 m/s. 반경 2 m 부터 상한이 줄어 반경 1 m 에서 모터 3.0",
     "motor": lambda k: lin(k, 0.5, 0.45 * (1 + 0.5 * SEP / 2) / M_PER, 1.0, TURN), "vmax": 0.45},
    {"id": "B", "name": "B: 직진 7 → 반경 1 m 에서 3.0", "short": "B",
     "desc": "반경 1 m 이하 3.0 은 지금과 같고, 직선에서 반경 1 m 까지 곡률에 비례해 서서히",
     "motor": lambda k: lin(k, 0.0, M_STRAIGHT, 1.0, TURN), "vmax": V_STRAIGHT},
    {"id": "C", "name": "C: 직진 7 → 반경 0.5 m 에서 3.0", "short": "C",
     "desc": "A 와 B 의 중간",
     "motor": lambda k: lin(k, 0.0, M_STRAIGHT, 2.0, TURN), "vmax": V_STRAIGHT},
    {"id": "A", "name": "A: 직진 7 → 반경 0.25 m 에서 3.0", "short": "A",
     "desc": "가장 급한 계획 반경 (0.25 m) 에서만 3.0, 그 사이는 곡률에 비례",
     "motor": lambda k: lin(k, 0.0, M_STRAIGHT, 4.0, TURN), "vmax": V_STRAIGHT},
]
def vcap(o, k, d):
    m = o["motor"](abs(k))
    v = m * M_PER / (1 + abs(k) * SEP / 2)
    return min(v, o["vmax"] if d > 0 else VREV)
# 반경별 표 · 곡선
radii = [None, 3.0, 2.0, 1.5, 1.0, 0.75, 0.5, 0.35, 0.25, 0.2]
curve = []
for o in OPTS:
    pts = []
    for R in list(np.round(np.geomspace(0.2, 6.0, 60), 3)) + [1e9]:
        k = 0.0 if R > 1e8 else 1.0 / R
        v = vcap(o, k, 1)
        pts.append({"R": None if R > 1e8 else float(R), "v": round(v, 4), "m": round(v * (1 + k * SEP / 2) / M_PER, 3)})
    curve.append({"id": o["id"], "pts": pts})
table = []
for o in OPTS:
    row = {"id": o["id"]}
    for R in radii:
        k = 0.0 if R is None else 1.0 / R
        v = vcap(o, k, 1)
        row["inf" if R is None else str(R)] = [round(v, 3), round(v * (1 + k * SEP / 2) / M_PER, 2)]
    table.append(row)
# 한 바퀴 경로: 조각 → 점 (x, y, 곡률, 방향, 조각번호)
def piece_points(pc):
    P = np.array(pc["poses"])[:, :3]
    if len(P) < 3:
        return None
    ds = np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))
    s = np.r_[0, np.cumsum(ds)]
    th = np.unwrap(P[:, 2])
    # 곡률: 5 cm 창의 방향 변화 / 거리
    k = np.zeros(len(P))
    for i in range(len(P)):
        j0, j1 = max(0, i - 2), min(len(P) - 1, i + 2)
        if s[j1] - s[j0] > 1e-6:
            k[i] = (th[j1] - th[j0]) / (s[j1] - s[j0])
    return P, s, k
def profile(o, segs):
    """조각들 → 시간 · 점마다 속도. 조각 끝마다 멈춤 (방향 바뀜 · 제자리)"""
    T, out, over, tmax_turn = 0.0, [], 0.0, 0.0
    for sg in segs:
        if sg["kind"] == "S":
            dt = abs(sg["dth"]) / SPIN_W
            out.append({"t0": round(T, 2), "t1": round(T + dt, 2), "spin": True, "x": sg["x"], "y": sg["y"]})
            T += dt
            continue
        P, s, k = sg["P"], sg["s"], sg["k"]
        d = 1 if sg["kind"] == "F" else -1
        v = np.array([vcap(o, kk, d) for kk in k])
        v[0] = v[-1] = 0.0
        for i in range(1, len(v)):
            v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * AX * (s[i] - s[i - 1])))
        for i in range(len(v) - 2, -1, -1):
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * AX * (s[i + 1] - s[i])))
        ts = [T]
        for i in range(1, len(v)):
            ts.append(ts[-1] + 2 * (s[i] - s[i - 1]) / max(v[i] + v[i - 1], 1e-3))
        for i in range(len(v)):
            mot = v[i] * (1 + abs(k[i]) * SEP / 2) / M_PER
            if abs(k[i]) > 0.3 and i:
                tmax_turn = max(tmax_turn, mot)
                if mot > TURN + 0.05:
                    over += ts[i] - ts[i - 1]
        out.append({"t": [round(x, 2) for x in ts[::2]], "x": [round(x, 3) for x in P[::2, 0]], "y": [round(x, 3) for x in P[::2, 1]],
                    "v": [round(d * x, 3) for x in v[::2]], "k": [round(x, 2) for x in k[::2]]})
        T = ts[-1]
    return T, out, over, tmax_turn
segs, stops = [], []
for a, b in zip(ROUTE, ROUTE[1:]):
    g = LEGS[f"{a}>{b}"]
    for pc in g["pieces"]:
        if pc["kind"] == "S":
            last = segs[-1] if segs else None
            x, y = (last["P"][-1, 0], last["P"][-1, 1]) if last and "P" in last else (g["start"][0], g["start"][1])
            segs.append({"kind": "S", "dth": pc["dth"], "x": round(float(x), 3), "y": round(float(y), 3)})
            continue
        r = piece_points(pc)
        if r:
            segs.append({"kind": pc["kind"], "P": r[0], "s": r[1], "k": r[2]})
    stops.append({"n": b, "x": g["goal"][0], "y": g["goal"][1]})
# 곡률 분포 (전진 거리 기준)
allk = np.concatenate([np.abs(sg["k"][1:]) for sg in segs if sg["kind"] == "F"])
allds = np.concatenate([np.diff(sg["s"]) for sg in segs if sg["kind"] == "F"])
bins = [(0, 0.2, "직선 (반경 > 5 m)"), (0.2, 0.5, "반경 2 ~ 5 m"), (0.5, 1.0, "반경 1 ~ 2 m"), (1.0, 2.0, "반경 0.5 ~ 1 m"), (2.0, 99, "반경 < 0.5 m")]
dist = [{"name": n, "m": round(float(allds[(allk >= lo) & (allk < hi)].sum()), 2)} for lo, hi, n in bins]
laps = []
DWELL = 2.0
for o in OPTS:
    T, prof, over, tmt = profile(o, segs)
    laps.append({"id": o["id"], "time": round(T, 1), "time_dwell": round(T + DWELL * (len(ROUTE) - 2), 1), "over_s": round(over, 1),
                 "turn_motor_max": round(tmt, 2), "prof": prof})
out = {"opts": [{k: v for k, v in o.items() if k not in ("motor",)} for o in OPTS], "curve": curve, "table": table, "radii": radii,
       "laps": laps, "dist": dist, "stops": stops, "route": ROUTE,
       "consts": {"belt": BELT, "r": R_W, "sep": SEP, "m_straight": M_STRAIGHT, "turn": TURN, "v_straight": round(V_STRAIGHT, 3), "ax": AX, "vrev": VREV, "spin_w": SPIN_W, "dwell": DWELL},
       "fwd_m": round(float(allds.sum()), 2)}
out["map"] = json.load(open(os.path.join(HERE, "map.json")))   # 미로 벽 · 상자 그림 (주행기록 페이지와 같은 발제 지도 도형)
js = json.dumps(out, ensure_ascii=False, separators=(",", ":"))
open(os.path.join(HERE, "data.json"), "w").write(js)
t = open(os.path.join(HERE, "template.html"), encoding="utf-8").read()
open(os.path.join(HERE, "회전속도_비교.html"), "w", encoding="utf-8").write(t.replace("/*__DATA__*/null", js))   # 페이지 (저장소에는 안 올림)
for L in laps:
    print(L["id"], "한 바퀴", L["time"], "s (+정지점 서기", L["time_dwell"], ") · 모터 3.0 넘는 꺾기", L["over_s"], "s · 꺾는 중 최고 모터", L["turn_motor_max"])
print("곡률 분포 (전진 m):", dist, "· 전진 합", out["fwd_m"])
for r in table: print(r["id"], {k: v for k, v in r.items() if k != "id"})
