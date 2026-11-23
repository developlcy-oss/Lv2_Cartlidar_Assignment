#!/usr/bin/env python3
"""반경 넓힌 시연 계획 만들기 (하이브리드 A* + Dubins 끝맺음) — 정지점은 그대로, 구간 길만 새로.

  python3 widen_plan.py demo_plan_max.json demo_plan_wide.json --radii 0.5,0.4 --r-floor 0.4 --r-rev 0.25 --keep 2,9
    (11-19 demo_plan_wide.json 을 만든 명령. 구간마다 몇 초 ~ 2 분. --keep = 원래 길 그대로 둘 구간, --legs 2-4 = 그 구간만 새로)
목표 (11-19 사용자 요청): 벽에 안 닿는 범위에서 반경 크게 · 빠르게 · 제자리 회전과 방향 맞추기 최소.
  - 비용 = 시간: 전진 ds / v(곡률) (v2_filter 바깥 모터 상한과 같은 식) · 후진 ds / 0.15 · 방향 바뀜 +CUSP s · 제자리 회전 +SPIN_PEN s / 15°
    · 벽 여유 10 cm 밑이면 벌점 (여유 클수록 좋음) · 곡률 바뀜 작은 벌점 (매끄럽게)
  - 반경: 전진 곡률 {0, ±1/1.0, ±1/0.6, ±1/r} (r = --radii 중 고름) + 급한 0.25 는 벌점 · 후진 ≥ --r-rev
  - 정지점: 마지막 APP m 를 곧게 들어감 (RPP 가 방향을 맞춰 들어가 정지점 방향 맞추기가 거의 0)
  - 이어진 후진 ≤ REV_MAX 0.30 m (필터 0.40 에서 따라가기 끝 밀기 0.10 여유) · 후진 반경 ≥ --r-rev (0.25, 상자에서 빠져나오기만)
  - 벽 여유 (차체 테두리 ~ 막힌 칸 가장자리, map_nav + map_amcl) ≥ HARD 4 cm. 출발 정지점에서 0.12 m 까지는 정지점 여유 그대로, 0.50 m 까지 HARD 로 올림 ·
    정지점 앞 곧은 들어가기 시작점 0.30 m 안도 같게 (상자 · 기둥 옆 정지점은 원래 4 cm 가 안 됨)
  - 안전 띠 (params_demo_safety.yaml collision_monitor VelocityStop: 전진 빠름 · 중간 · 느림 앞 띠, 후진 뒤 띠) 에 라이다 높이 장애물 (map_amcl) 이 안 들어오게.
    FootprintApproach (1.2 s 안에 닿을 속도면 줄임) 감속을 시간 비용에 넣음
  - 정지점 앞 곧게 들어가기는 0.25 → 안 되면 0.15 → 0.08 m (상자가 좁은 칸에 있는 S5 · S6)
  - 전진 급한 반경 0.25 는 벌점 (정지점 근처 좁은 칸만) · 방향 바뀜 때 굽은 채 멈춤 · 출발 벌점 (조각 시작 방향 맞추기 줄이기)
  - 지도: mission_planner/maps/map_nav (벽 · 상자 · 기둥) + map_amcl
  - 11-22 --arm 0.06: 팔 (BOX_VIEW 자세, 주행 내내) 외곽 ARM_POLY 도 검사 — 최저 z 0.184 < 벽 0.39 라 벽 · 기둥 (map_amcl) 만, 상자 · 퍽은 팔보다 낮아 뺌.
    팔 여유 ≥ --arm (AMCL ±3 cm 여유) 를 차체 요구 HARD 와 같은 줄로 맞춰 (팔 여유 − (ARM_HARD − HARD)) 차체 여유와 작은 쪽을 씀
"""
import argparse, heapq, json, math, sys, time
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, map_coordinates

from pathlib import Path
import os
REPO = os.environ.get('LV2_REPO', str(Path(__file__).resolve().parents[3])).rstrip('/') + '/'   # 이 파일이 든 저장소 (workspace/tools/drive_demo) · 사본에서 돌릴 때 LV2_REPO=<저장소>
RES, OX, OY = 0.025, -0.15, -0.15
FP = np.array([[0.25, 0.07], [0.25, -0.07], [0.1328, -0.1676], [-0.2135, -0.1676], [-0.2135, 0.1676], [0.1328, 0.1676]])
ARM_POLY = np.array([[0.318, 0.064], [0.318, -0.064], [0.034, -0.064], [0.034, 0.064]])   # 11-22 BOX_VIEW 팔 몸통 외곽 (base 기준 x 0.034 ~ 0.318, 폭 ±0.064)
ARM_HARD = None   # 팔 ↔ 벽 여유 요구 [m] (None = 팔 검사 안 함 · --arm 0.06)
SEP = 0.317 * 1.29
MS = 0.0335 * 2.0 * 5.5          # 바깥 바퀴 둘레 속도 상한 [m/s] (v2_filter turn_motor 5.5 · 벨트 2.0)
VF, VR = 0.45, 0.15
REV_MAX = 0.30   # 규칙 v2 필터 0.40 (실물 0.50) — 따라가기가 끝에서 더 밀어 (EXT_R 0.10) 필터에 걸리지 않게 0.10 여유
HARD, HARD_NEAR, SOFT = 0.040, 0.012, 0.10
NEAR = 0.15
CUSP, SPIN_PEN, KCH, KCLR = 4.0, 6.0, 0.25, 6.0
APP = 0.25
BAND_MARGIN = 0.01   # 안전 띠 바깥과 라이다 장애물 사이 최소 [m]
T_APPROACH = 1.2     # FootprintApproach time_before_collision [s]
CUSP_CURVE = 1.5     # 방향 바뀜 때 굽은 채 멈추거나 굽은 채 출발하면 (방향 맞추기 생김) 벌점 [s]
GOAL_SLOW = 0.35     # 정지점 이 거리 안은 느린 띠
R_TIGHT, KTIGHT = 0.25, 30.0   # 급한 전진 반경 · 그 벌점 [s/m] (1 m 쓰면 30 s 손해로 봄)
STEP = 0.10
XY_BIN, TH_BIN = 0.05, math.radians(5)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def vcap(k):
    return min(VF, MS / (1 + SEP * abs(k) / 2))


class Clear:
    def __init__(self):
        dts = []
        for n in ('map_nav', 'map_amcl'):
            im = np.array(Image.open(REPO + f'workspace/ros2_ws/src/mission_planner/maps/{n}.pgm'))
            dts.append(distance_transform_edt(im >= 128) * RES)
        self.H = dts[0].shape[0]
        self.dt = np.minimum(*dts) - RES / 2          # 막힌 칸 가장자리까지 (대략)
        self.dta = dts[1] - RES / 2                   # 라이다 높이 장애물만 (map_amcl: 벽 · 기둥, 상자 없음) — 안전 띠 · 감속 판단용
        def edge_pts(poly):
            out = []
            for i in range(len(poly) - 1):
                a, b = np.array(poly[i]), np.array(poly[i + 1])
                n = max(2, int(np.hypot(*(b - a)) / 0.01))
                out += [a + (b - a) * t for t in np.linspace(0, 1, n)]
            return np.array(out)
        # params_demo_safety.yaml collision_monitor VelocityStop 띠 (바깥 테두리 + 속 몇 줄)
        self.band = {
            'fast': edge_pts([[0.1328, 0.1676], [0.2828, 0.1676], [0.4, 0.07], [0.4, -0.07], [0.2828, -0.1676], [0.1328, -0.1676]]),
            'mid': edge_pts([[0.1328, 0.1676], [0.2328, 0.1676], [0.35, 0.07], [0.35, -0.07], [0.2328, -0.1676], [0.1328, -0.1676]]),
            'slow': edge_pts([[0.1328, 0.1676], [0.1828, 0.1676], [0.3, 0.07], [0.3, -0.07], [0.1828, -0.1676], [0.1328, -0.1676]]),
            'back': np.array([[x, y] for x in np.linspace(-0.2135, -0.3135, 6) for y in np.linspace(-0.17, 0.17, 12)]),
        }
        bd = []
        for i in range(len(FP)):
            a, b = FP[i], FP[(i + 1) % len(FP)]
            n = max(2, int(np.hypot(*(b - a)) / 0.01))
            bd += [a + (b - a) * t for t in np.linspace(0, 1, n, endpoint=False)]
        self.bd = np.array(bd)
        arm = []
        for i in range(len(ARM_POLY)):
            a, b = ARM_POLY[i], ARM_POLY[(i + 1) % len(ARM_POLY)]
            n = max(2, int(np.hypot(*(b - a)) / 0.01))
            arm += [a + (b - a) * t for t in np.linspace(0, 1, n, endpoint=False)]
        self.arm = np.array(arm)
        # 안쪽 원 반지름 (차체가 늘 덮는 원) — 2D 길찾기 어림값용
        r = 9.0
        for i in range(len(FP)):
            a, b = FP[i], FP[(i + 1) % len(FP)]
            t = np.clip(-a @ (b - a) / ((b - a) @ (b - a)), 0, 1)
            r = min(r, np.hypot(*(a + (b - a) * t)))
        self.r_in = r

    def at(self, x, y, dt=None):
        """점들의 막힌 칸까지 거리 (쌍선형)"""
        col = (np.asarray(x) - OX) / RES - 0.5
        row = self.H - 1 - ((np.asarray(y) - OY) / RES - 0.5)
        return map_coordinates(self.dt if dt is None else dt, [row, col], order=1, mode='constant', cval=-1.0)

    def body_pts(self, P, pts, dt):
        P = np.atleast_2d(P)
        c, s = np.cos(P[:, 2]), np.sin(P[:, 2])
        X = P[:, None, 0] + c[:, None] * pts[None, :, 0] - s[:, None] * pts[None, :, 1]
        Y = P[:, None, 1] + s[:, None] * pts[None, :, 0] + c[:, None] * pts[None, :, 1]
        return self.at(X.ravel(), Y.ravel(), dt).reshape(X.shape).min(axis=1)

    def band_ok(self, P, name):
        """안전 띠 (collision_monitor VelocityStop) 에 라이다 장애물이 안 들어오는지"""
        return bool(np.all(self.body_pts(P, self.band[name], self.dta) >= BAND_MARGIN))

    def free_run(self, q, k, d, maxL):
        """같은 곡률로 계속 갈 때 차체가 라이다 장애물에 닿기까지 길이 (FootprintApproach 감속 어림)"""
        n = max(2, int(maxL / 0.04))
        E = arc(q[0], q[1], q[2], k, d, maxL, n)
        c = self.body_pts(E, self.bd, self.dta)
        bad = np.where(c < 0.0)[0]
        return maxL if len(bad) == 0 else (bad[0]) * maxL / n

    def arm_clear(self, P):
        """P: (n,3) 자세 → 각 자세 팔 외곽 ↔ 벽 · 기둥 (map_amcl) 최소 여유"""
        return self.body_pts(P, self.arm, self.dta)

    def poses(self, P):
        """P: (n,3) 자세 → 각 자세 차체 테두리 최소 여유 (ARM_HARD 면 팔 여유 − (ARM_HARD − HARD) 와 작은 쪽)"""
        P = np.atleast_2d(P)
        c, s = np.cos(P[:, 2]), np.sin(P[:, 2])
        X = P[:, None, 0] + c[:, None] * self.bd[None, :, 0] - s[:, None] * self.bd[None, :, 1]
        Y = P[:, None, 1] + s[:, None] * self.bd[None, :, 0] + c[:, None] * self.bd[None, :, 1]
        body = self.at(X.ravel(), Y.ravel()).reshape(X.shape).min(axis=1)
        if ARM_HARD is None:
            return body
        return np.minimum(body, self.arm_clear(P) - (ARM_HARD - HARD))


def arc(x, y, th, k, d, L, n):
    """곡률 k · 방향 d (+1 전진 / −1 후진) 로 길이 L 을 n 등분한 자세들 (끝 포함, 시작 뺌)"""
    s = np.linspace(L / n, L, n) * d
    if abs(k) < 1e-9:
        return np.stack([x + s * math.cos(th), y + s * math.sin(th), np.full(n, th)], 1)
    t = th + k * s
    return np.stack([x + (np.sin(t) - math.sin(th)) / k, y - (np.cos(t) - math.cos(th)) / k, t], 1)


# ---------- Dubins (전진만, 반경 r) ----------
def dubins(q0, q1, r):
    """가장 짧은 Dubins 길 → [(곡률, 길이), ...] 또는 None"""
    dx, dy = q1[0] - q0[0], q1[1] - q0[1]
    D = math.hypot(dx, dy) / r
    th = math.atan2(dy, dx)
    a, b = (q0[2] - th) % (2 * math.pi), (q1[2] - th) % (2 * math.pi)
    sa, sb, ca, cb, cab = math.sin(a), math.sin(b), math.cos(a), math.cos(b), math.cos(a - b)
    best = None

    def put(w, t, p, q):
        nonlocal best
        if t is None or min(t, p, q) < -1e-9:
            return
        L = t + p + q
        if best is None or L < best[0]:
            best = (L, w, (t, p, q))
    m = lambda v: v % (2 * math.pi)
    # LSL
    p2 = 2 + D * D - 2 * cab + 2 * D * (sa - sb)
    if p2 >= 0:
        tmp = math.atan2(cb - ca, D + sa - sb)
        put('LSL', m(-a + tmp), math.sqrt(p2), m(b - tmp))
    # RSR
    p2 = 2 + D * D - 2 * cab + 2 * D * (sb - sa)
    if p2 >= 0:
        tmp = math.atan2(ca - cb, D - sa + sb)
        put('RSR', m(a - tmp), math.sqrt(p2), m(-b + tmp))
    # LSR
    p2 = -2 + D * D + 2 * cab + 2 * D * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(-ca - cb, D + sa + sb) - math.atan2(-2.0, p)
        put('LSR', m(-a + tmp), p, m(-m(b) + tmp))
    # RSL
    p2 = -2 + D * D + 2 * cab - 2 * D * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(ca + cb, D - sa - sb) - math.atan2(2.0, p)
        put('RSL', m(a - tmp), p, m(b - tmp))
    # RLR
    tmp = (6 - D * D + 2 * cab + 2 * D * (sa - sb)) / 8
    if abs(tmp) <= 1:
        p = m(2 * math.pi - math.acos(tmp))
        t = m(a - math.atan2(ca - cb, D - sa + sb) + p / 2)
        put('RLR', t, p, m(a - b - t + p))
    # LRL
    tmp = (6 - D * D + 2 * cab + 2 * D * (sb - sa)) / 8
    if abs(tmp) <= 1:
        p = m(2 * math.pi - math.acos(tmp))
        t = m(-a - math.atan2(ca - cb, D + sa - sb) + p / 2)
        put('LRL', t, p, m(m(b) - a - t + p))
    if best is None:
        return None
    L, w, seg = best
    out = []
    for c, l in zip(w, seg):
        if l * r > 1e-6:
            out.append(({'L': 1 / r, 'R': -1 / r, 'S': 0.0}[c], l * r))
    return out


def roll(q, segs, d=1):
    """(곡률, 길이) 목록을 1 cm 간격 자세로"""
    x, y, th = q
    P = []
    for k, L in segs:
        n = max(1, int(math.ceil(L / 0.01)))
        A = arc(x, y, th, k, d, L, n)
        P.append(A)
        x, y, th = A[-1]
    return np.concatenate(P) if P else np.zeros((0, 3))


# ---------- 하이브리드 A* ----------
def heur_grid(C, goal_xy):
    """막힌 곳을 피한 2D 거리 (차체 안쪽 원 + HARD 가 지나갈 수 있는 칸, 8 방향 Dijkstra, 5 cm 칸)"""
    nx, ny = int(4.8 / XY_BIN), int(6.7 / XY_BIN)
    gx, gy = np.meshgrid(OX + (np.arange(nx) + 0.5) * XY_BIN, OY + (np.arange(ny) + 0.5) * XY_BIN)
    ok = (C.at(gx.ravel(), gy.ravel()) >= C.r_in + HARD_NEAR - 0.02).reshape(gx.shape)
    dist = np.full(gx.shape, np.inf)
    i0, j0 = int((goal_xy[1] - OY) / XY_BIN), int((goal_xy[0] - OX) / XY_BIN)
    dist[i0, j0] = 0
    pq = [(0.0, i0, j0)]
    nb = [(di, dj, math.hypot(di, dj) * XY_BIN) for di in (-1, 0, 1) for dj in (-1, 0, 1) if di or dj]
    while pq:
        d, i, j = heapq.heappop(pq)
        if d > dist[i, j]:
            continue
        for di, dj, w in nb:
            a, b = i + di, j + dj
            if 0 <= a < ny and 0 <= b < nx and ok[a, b] and d + w < dist[a, b]:
                dist[a, b] = d + w
                heapq.heappush(pq, (d + w, a, b))
    return dist


def band_name(k, d, near_goal):
    if d < 0:
        return 'back'
    v = vcap(k)
    return 'slow' if near_goal else ('fast' if v >= 0.25 else 'mid')


def seg_cost(C, P, k, d, need, near_goal=False):
    """자세 묶음 P (곡률 k, 방향 d) 의 시간 비용 + 벽 여유 벌점. 막히면 None (need = 점마다 요구 여유)
    11-19: 안전 띠 (collision_monitor 정지) 에 라이다 장애물이 들어오면 막힘 · FootprintApproach 감속을 시간에 넣음"""
    clr = C.poses(P)
    if np.any(clr < need):
        return None, clr.min()
    if not C.band_ok(P, band_name(k, d, near_goal)):
        return None, clr.min()
    L = (len(P)) * (np.hypot(*(P[1, :2] - P[0, :2])) if len(P) > 1 else STEP)
    v = vcap(k) if d > 0 else VR
    run = C.free_run(tuple(P[-1]), k, d, v * T_APPROACH)
    v = max(min(v, run / T_APPROACH), 0.02)
    t = L / v
    t += KCLR * np.mean(np.maximum(SOFT - clr, 0)) * L
    return t, clr.min()


def plan_leg(C, q0, qg, r, deadline=60.0, r_rev=0.28, APP=APP, spin_pen=None, cusp=None):
    """q0 정지점 → qg 정지점 (마지막 APP m 곧게 전진). 돌려줌: [(kind, poses | dth)], 정보"""
    SPIN_PEN_ = SPIN_PEN if spin_pen is None else spin_pen   # 11-21 --spin-pen: 제자리 회전 벌점 [s / 15°] (크게 = 곡선 · 방향 바뀜으로 대신)
    CUSP_ = CUSP if cusp is None else cusp
    qa = (qg[0] - APP * math.cos(qg[2]), qg[1] - APP * math.sin(qg[2]), qg[2])   # 곧은 들어가기 시작
    hg = heur_grid(C, qa)

    def h(x, y):
        i, j = int((y - OY) / XY_BIN), int((x - OX) / XY_BIN)
        g = hg[i, j] if 0 <= i < hg.shape[0] and 0 <= j < hg.shape[1] else np.inf
        return max(g, math.hypot(x - qa[0], y - qa[1])) / VF

    c0 = float(C.poses(np.array([q0]))[0])
    cA = float(C.poses(roll(qa, [(0.0, APP)])).min())            # 곧은 들어가기 (정지점 앞 APP m) 의 여유
    RAMP = 0.30

    def need(moved, X=None, Y=None):
        """점마다 요구 여유: 가운데 HARD · 출발 정지점에서 RAMP m 동안 (정지점 여유 → HARD) · 들어가기 시작점 RAMP m 안도 같게"""
        moved = np.asarray(moved, float)
        n = np.minimum(HARD, c0 - 0.010 + np.clip((moved - 0.12) / (0.50 - 0.12), 0, 1) * (HARD - c0 + 0.010))
        if X is not None:
            dA = np.hypot(np.asarray(X) - qa[0], np.asarray(Y) - qa[1])
            n = np.minimum(n, np.minimum(HARD, cA - 0.005 + np.clip(dA / RAMP, 0, 1) * (HARD - cA + 0.005)))
        return n
    need0 = c0 - 0.005
    kf = [0.0] + [v for rr in (1.0, 0.6, r) + ((R_TIGHT,) if R_TIGHT < r else ()) for v in (1 / rr, -1 / rr)]
    K = {1: kf,                                                          # 전진: 반경 ≥ r 기본, R_TIGHT 는 벌점 (정지점 근처 좁은 칸만)
         -1: [0.0, 0.5 / r_rev, -0.5 / r_rev, 1.0 / r_rev, -1.0 / r_rev]}   # 후진: 0.15 m/s 라 모터 여유 — 상자에서 빠져나올 때만 (반경 ≥ r_rev)
    W = 1.4
    t0 = time.time()
    # 노드: (x, y, th, d, k, rev, moved) · 부모 · 움직임
    nodes = [(q0[0], q0[1], q0[2], 0, 0.0, 0.0, 0.0)]
    par, mv, gco = [-1], [None], [0.0]
    key = lambda x, y, th, d, rev: (int(round(x / XY_BIN)), int(round(y / XY_BIN)), int(round(wrap(th) / TH_BIN)) % 72, d, int(math.ceil(rev / 0.1 - 1e-6)))
    best_g = {key(q0[0], q0[1], q0[2], 0, 0): 0.0}
    pq = [(W * h(q0[0], q0[1]), 0)]
    found, n_exp, shots = None, 0, 0
    best_d = [9.0, 0, 0, 0, 0]
    while pq and time.time() - t0 < deadline:
        f, ni = heapq.heappop(pq)
        x, y, th, d, k, rev, moved = nodes[ni]
        g = gco[ni]
        n_exp += 1
        # Dubins 끝맺음 (전진으로 qa 까지)
        dist_a = math.hypot(x - qa[0], y - qa[1])
        if dist_a < best_d[0]:
            best_d[:] = [dist_a, x, y, math.degrees(th), d]
        if d == 1 and dist_a < 0.025 and abs(wrap(th - qa[2])) < math.radians(4):   # 거의 닿음 → 끝을 살짝 밀어 맞춤
            found = (ni, [], g, 'tol')
            break
        if d >= 0 and dist_a < 2.0 and (n_exp % 4 == 0 or dist_a < 0.5):
            for rr in [max(r, 0.8), r] + ([R_TIGHT] if dist_a < 0.8 else []):
                segs = dubins((x, y, th), qa, rr)
                if not segs:
                    continue
                shots += 1
                P = roll((x, y, th), segs)
                if len(P) < 2:
                    continue
                A = roll(tuple(P[-1]), [(0.0, APP)])
                clr = C.poses(P[::3])
                sP = moved + 0.03 * np.arange(1, len(clr) + 1)          # 3 cm 마다 (roll 은 1 cm 간격)
                if np.any(clr < need(sP, P[::3, 0], P[::3, 1])):
                    continue
                okb, tt, q = True, 0.0, (x, y, th)
                for kk, L in segs:
                    S = roll(q, [(kk, L)])
                    ng_ = np.hypot(S[:, 0] - qg[0], S[:, 1] - qg[1]) < GOAL_SLOW
                    for part, nm in ((S[~ng_], band_name(kk, 1, False)), (S[ng_], 'slow')):
                        if len(part) and not C.band_ok(part[::3], nm):
                            okb = False
                    v = vcap(kk)
                    run = min(C.free_run(tuple(p), kk, 1, v * T_APPROACH) for p in S[::10])
                    tt += L / max(min(v, run / T_APPROACH), 0.02)
                    q = tuple(S[-1])
                if not okb:
                    continue
                Aok = C.band_ok(A[np.hypot(A[:, 0] - qg[0], A[:, 1] - qg[1]) >= GOAL_SLOW], 'mid') if len(A) else True
                if not Aok or not C.band_ok(A, 'slow'):
                    continue
                tt += KCLR * np.mean(np.maximum(SOFT - clr, 0)) * sum(L for _, L in segs)
                tt += KTIGHT * sum(L for kk, L in segs if abs(kk) > 1 / r + 1e-9)
                found = (ni, segs, g + tt, rr)
                break
            if found:
                break
        for dn in (1, -1):
            for kn in K[dn]:
                if dn == -1 and rev + STEP > REV_MAX + 1e-9:
                    continue
                P = arc(x, y, th, kn, dn, STEP, 5)
                c, cm = seg_cost(C, P, kn, dn, need(moved + STEP * np.arange(1, 6) / 5, P[:, 0], P[:, 1]),
                                 math.hypot(P[-1, 0] - qg[0], P[-1, 1] - qg[1]) < GOAL_SLOW)
                if c is None:
                    continue
                if dn > 0 and abs(kn) > 1 / r + 1e-9:
                    c += KTIGHT * STEP
                if d != 0 and dn != d:
                    c += CUSP_CURVE * ((abs(k) > 1e-9) + (abs(kn) > 1e-9))   # 굽은 채 멈춤 · 굽은 채 출발
                c += (CUSP_ if (d != 0 and dn != d) else 0.0) + KCH * min(abs(kn - k) * (r if dn > 0 else r_rev), 2.0) * (d == dn)
                nx_, ny_, nth = P[-1]
                nrev = rev + STEP if dn == -1 else (0.0 if d == 1 or d == 0 else rev)   # 전진 시작하면 후진 셈 0 (0.15 m 규칙은 STEP 0.10 이 짧아 넉넉히 둠)
                kk = key(nx_, ny_, nth, dn, nrev)
                ng = g + c
                if ng < best_g.get(kk, 1e18) - 1e-9:
                    best_g[kk] = ng
                    nodes.append((nx_, ny_, nth, dn, kn, nrev, moved + STEP))
                    par.append(ni)
                    mv.append(('arc', kn, dn))
                    gco.append(ng)
                    heapq.heappush(pq, (ng + W * h(nx_, ny_), len(nodes) - 1))
        # 제자리 회전 (벌점 큼, 멈춰 있을 때 어디서나)
        for dth in (math.radians(15), -math.radians(15)):
            nth = wrap(th + dth)
            fr = (0.25, 0.5, 0.75, 1.0) if ARM_HARD is not None else (0.5, 1.0)   # 11-22 팔 모서리 반경 0.324 m > 차체 0.271 — 팔 켜면 3.75° 마다
            if C.poses(np.array([[x, y, th + dth * f] for f in fr])).min() < need(moved, x, y):
                continue
            kk = key(x, y, nth, 0, rev)
            ng = g + abs(dth) / 0.5 + SPIN_PEN_ + (CUSP_ if d != 0 else 0.0)
            if ng < best_g.get(kk, 1e18) - 1e-9:
                best_g[kk] = ng
                nodes.append((x, y, nth, 0, 0.0, rev, moved))
                par.append(ni)
                mv.append(('spin', dth))
                gco.append(ng)
                heapq.heappush(pq, (ng + W * h(x, y), len(nodes) - 1))
    if not found:
        return None, {'exp': n_exp, 'shots': shots, 't': round(time.time() - t0, 1), 'closest': [round(v, 3) for v in best_d]}
    ni, segs, gtot, rr = found
    chain = []
    while ni > 0:
        chain.append(ni)
        ni = par[ni]
    chain.reverse()
    # 조각으로 (1 cm 간격)
    pieces = []
    cur = np.array([q0])

    def add(kind, P):
        if pieces and pieces[-1][0] == kind and kind != 'S':
            pieces[-1][1].extend(P.tolist())
        else:
            pieces.append([kind, [list(q0) if not pieces else None] if False else P.tolist()])
    pos = tuple(q0)
    for ci in chain:
        m = mv[ci]
        if m[0] == 'spin':
            if pieces and pieces[-1][0] == 'S':
                pieces[-1][1] += m[1]
            else:
                pieces.append(['S', m[1]])
            pos = (pos[0], pos[1], wrap(pos[2] + m[1]))
            continue
        _, kn, dn = m
        P = arc(*pos, kn, dn, STEP, 10)
        kind = 'F' if dn > 0 else 'R'
        if pieces and pieces[-1][0] == kind:
            pieces[-1][1].extend(P.tolist())
        else:
            pieces.append([kind, [list(pos)] + P.tolist()])
        pos = tuple(P[-1])
    P = roll(pos, segs)
    if not segs:   # 허용 오차로 닿음: 마지막 전진 조각 끝 0.6 m 를 부드럽게 밀어 qa 에 맞추고 방향은 모양에서 다시
        Q = np.array(pieces[-1][1])
        dx, dy = qa[0] - Q[-1, 0], qa[1] - Q[-1, 1]
        se = np.r_[np.cumsum(np.hypot(*np.diff(Q[::-1, :2], axis=0).T))[::-1], 0.0]
        u = np.clip(1 - se / 0.6, 0, 1)
        w = u * u * (3 - 2 * u)
        Q[:, 0] += w * dx
        Q[:, 1] += w * dy
        m = se < 0.6
        tx, ty = np.gradient(Q[:, 0]), np.gradient(Q[:, 1])
        Q[m, 2] = np.arctan2(ty, tx)[m]
        Q[-1] = qa
        pieces[-1][1] = Q.tolist()
        pos = qa
    A = roll(tuple(P[-1]) if len(P) else pos, [(0.0, APP)])
    P = np.concatenate([P, A])
    if pieces and pieces[-1][0] == 'F':
        pieces[-1][1].extend(P.tolist())
    else:
        pieces.append(['F', [list(pos)] + P.tolist()])
    # 끝을 정확히 qg 로
    pieces[-1][1][-1] = list(qg)
    out = []
    for kind, v in pieces:
        if kind == 'S':
            out.append({'kind': 'S', 'dth': float(v)})
        else:
            out.append({'kind': kind, 'poses': [[round(a, 4), round(b, 4), round(c, 5)] for a, b, c in v]})
    return out, {'exp': n_exp, 'shots': shots, 't': round(time.time() - t0, 1), 'cost': round(gtot, 1), 'r_end': rr}


def stats(C, pieces):
    """조각별 길이 · 최소 반경 · 최소 여유 · 이론 시간"""
    out, T = [], 0.0
    for pc in pieces:
        if pc['kind'] == 'S':
            out.append(f"제자리 {math.degrees(pc['dth']):+.0f}°")
            T += abs(pc['dth']) / 0.5 + 1.5
            continue
        P = np.array(pc['poses'])
        ds = np.hypot(*np.diff(P[:, :2], axis=0).T)
        s = np.r_[0, np.cumsum(ds)]
        th = np.unwrap(P[:, 2])
        rad = []
        for i in range(len(P)):
            j = np.searchsorted(s, s[i] + 0.10)
            if j >= len(P):
                break
            dth = abs(th[j] - th[i])
            rad.append(0.10 / dth if dth > 1e-6 else 99)
        rmin = min(rad) if rad else 99
        clr = C.poses(P)
        kind = pc['kind']
        tt = sum(ds[i] / (vcap(1 / rad[min(i, len(rad) - 1)]) if (kind == 'F' and rad) else (VF if kind == 'F' else VR)) for i in range(len(ds)))
        T += tt + 1.0
        out.append(f"{'전진' if kind == 'F' else '후진'} {s[-1]:.2f} m 최소반경 {rmin:.2f} 여유 {clr.min() * 100:.1f} cm (가운데 {clr[(s > NEAR) & (s < s[-1] - NEAR)].min() * 100 if ((s > NEAR) & (s < s[-1] - NEAR)).any() else float('nan'):.1f})")
    return out, T


def restop(d):
    """정지점 (legs goal · course_text route 줄) 을 box_quiet.stop_pose 로. 상자 아닌 끝 (A1) · 출발은 그대로. 각 leg 에 'box' 정보"""
    import re
    import box_quiet as bq
    for L in d['legs']:
        t = bq.leg_target(L)
        if t in bq.BOX_STOPS:
            L['goal'] = bq.stop_pose(t)
            L['box'] = {'name': t, 'center': list(bq.BOXES[t]), 'gap': bq.STOP_GAP, 'front': bq.FRONT}
    lines = []
    for ln in d.get('course_text', '').split('\n'):
        m = re.match(r'(\s*- \{stop: )(\w+)(, x: )[-\d.]+(, y: )[-\d.]+(, face: )[-\d.]+(\}.*)', ln)
        if m and m.group(2) in bq.BOX_STOPS:
            x, y, th = bq.stop_pose(m.group(2))
            ln = f"{m.group(1)}{m.group(2)}{m.group(3)}{x:.4f}{m.group(4)}{y:.4f}{m.group(5)}{math.degrees(th):.1f}}}   # 11-22 줄자 실측 상자 {bq.BOXES[m.group(2)]} 면 중앙, 앞 끝 ↔ 면 {bq.STOP_GAP * 100:.1f} cm"
        lines.append(ln)
    d['course_text'] = '\n'.join(lines).replace('# 정지점 = course_v1_L2_실측.yaml (상자 실측)', '# 정지점 = 11-22 줄자 실측 상자 좌표 (box_quiet.stop_pose, 이전 course_v1_L2_실측.yaml)')
    d['plan'] = d['plan'].replace('7cm_실측L', '발제10.7cm')
    d['course'] = d['plan'].split('_끝')[0] + '.yaml'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--radii', default='0.5,0.4', help='시험할 최소 반경 (큰 것부터)')
    ap.add_argument('--r-floor', type=float, default=0.4, help='이 반경 이상인 길이 있으면 그중에서 고름')
    ap.add_argument('--r-bonus', type=float, default=2.0, help='반경 1 m 당 비용 덜어 줌 [s] (같으면 큰 반경)')
    ap.add_argument('--legs', default='')
    ap.add_argument('--keep', default='', help='원래 길 그대로 둘 구간 번호 (예 2,9 — 11-19 시뮬에서 새 길이 더 나빴던 구간)')
    ap.add_argument('--r-rev', type=float, default=0.25, help='후진 최소 반경 (상자에서 빠져나오기)')
    ap.add_argument('--deadline', type=float, default=30.0)
    ap.add_argument('--spin-pen', type=float, default=SPIN_PEN, help='제자리 회전 벌점 [s / 15°] (11-19 기본 6, 11-21 곡선으로 이어 가기는 크게)')
    ap.add_argument('--cusp', type=float, default=CUSP, help='방향 바뀜 (전진 ↔ 후진) 벌점 [s]')
    ap.add_argument('--restop', action='store_true', help='11-21: 정지점을 box_quiet.stop_pose (발제 상자 면 중앙, 앞 끝 ↔ 면 STOP_GAP) 로 바꾸고 course_text 도 고침')
    ap.add_argument('--hard', type=float, default=None, help='벽 여유 요구 [m] (기본 HARD 0.04)')
    ap.add_argument('--arm', type=float, nargs='?', const=0.06, default=None, help='11-22 팔 (BOX_VIEW) 외곽 ↔ 벽 · 기둥 여유 요구 [m] (값 없이 --arm = 0.06, 안 주면 팔 검사 안 함)')
    a = ap.parse_args()
    if a.hard is not None:
        globals()['HARD'] = a.hard
    if a.arm is not None:
        globals()['ARM_HARD'] = a.arm
    C = Clear()
    d = json.load(open(a.src))
    legs = d['legs']
    if a.restop:
        restop(d)
    sel = range(len(legs))
    if a.legs:
        lo, hi = (int(v) for v in a.legs.split('-'))
        sel = range(lo, hi + 1)
    q = list(d['start'])
    starts = []
    for L in legs:
        starts.append(q)
        q = list(L['goal'])
    T0 = T1 = 0.0
    keep = {int(v) for v in a.keep.split(',') if v}
    for i in sel:
        L = legs[i]
        if i in keep:
            print(f"· {L['name']}: 원래 길 그대로 (--keep)", flush=True)
            continue
        q0, qg = starts[i], L['goal']
        _, t_old = stats(C, L['pieces'])
        cands = []
        for app in (0.25, 0.15, 0.08):
            for r in [float(v) for v in a.radii.split(',')]:
                res, info = plan_leg(C, q0, qg, r, a.deadline, a.r_rev, app, a.spin_pen, a.cusp)
                print(f"  {L['name']}: r {r} · 곧게 {app} → " + (f"비용 {info['cost']} s · 조각 {''.join(p['kind'] for p in res)}" if res else f"길 없음 ({info})"), flush=True)
                if res:
                    info['app'] = app
                    cands.append((info['cost'] - a.r_bonus * r, r, res, info))
            if cands:
                break
        if not cands:
            print(f"✗ {L['name']}: 길 못 찾음 — 원래 길 그대로", flush=True)
            continue
        good = [c for c in cands if c[1] >= a.r_floor] or cands
        _, r, res, info = min(good, key=lambda c: c[0])
        st, t_new = stats(C, res)
        T0 += t_old
        T1 += t_new
        L['pieces'] = res
        L['widen'] = {'r': r, **info}
        print(f"✓ {L['name']} r {r} · 이론 {t_old:.1f} → {t_new:.1f} s · {info}", flush=True)
        for s in st:
            print('    ' + s, flush=True)
    if not d['plan'].endswith('_반경넓힘'):
        d['plan'] = d['plan'] + '_반경넓힘'
    d['note_widen'] = f"11-19 widen_plan.py: 하이브리드 A* (구간마다 최소 반경 {a.radii} 중 r ≥ {a.r_floor} 에서 시간 최소, 제자리 · 방향 바뀜 벌점, 정지점 앞 {APP} m 곧게, 벽 여유 ≥ {HARD * 100:.1f} cm" + (f", 팔 ↔ 벽 ≥ {ARM_HARD * 100:.1f} cm" if ARM_HARD is not None else '') + ") 로 구간 길만 새로 — 정지점 그대로" + (f" · 원래 길 그대로 S{', S'.join(str(k) for k in sorted(keep))}" if keep else "")
    json.dump(d, open(a.dst, 'w'), ensure_ascii=False, indent=1)
    print(f"이론 시간 (조각 사이 1 s · 제자리 1.5 s 더함) {T0:.0f} → {T1:.0f} s → {a.dst}")


if __name__ == '__main__':
    main()
