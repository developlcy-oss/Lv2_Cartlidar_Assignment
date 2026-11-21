"""11-21 상자 앞면 재기 · 다가가기 계획 (box face) — 순수 함수 (ROS 없이 import, test_box_face.py 가 시험).

배경: 실물 시연 로봇 (차동 구동) 이 상자 (205 × 205 × 70 mm) 앞에서 앞 LDS-03 로 상자의 로봇 쪽 면을 잰 뒤,
  상판 앞 끝 (base_link x 0.1328) ↔ 상자 앞면 5 cm · 상자 중앙선 위 · 상자 법선 방향으로 다가감.
라이다: laser_front = base_link (0.1318, 0, 0.0525), rpy (π, 0, π) → 2D 회전행렬 [[−1, 0], [0, 1]]
  → bx = 0.1318 − r·cosθ, by = +r·sinθ (yaw 만 쓰면 y 부호가 틀림 → 반드시 2 × 2 행렬).
LDS-03: range_min ≈ 0.12 m, 각 간격 ≈ 1°, 거리 잡음 σ ≈ 1 cm (추정) → 측정은 앞 끝 ↔ 상자 0.15 ~ 0.35 m 에서만,
  마지막은 오도메트리로 다가감 (plan_approach 의 단계).
좌표: 모두 base_link (측정 순간 로봇 자세 = (0, 0, 0)). 각도 [rad], 거리 [m].
정의:
  (cx, cy) = 상자 앞면 선분 중점, nyaw = 로봇 → 상자 쪽 면 법선 yaw (n = (cos, sin)),
  gap = 원점 ↔ 앞면 직선 법선 거리 (n·C) − FRONT_OFFSET,
  lateral = 상자 중심선 (C 를 지나 n 방향) 에 대한 로봇 원점 옆 어긋남의 반대 부호 (+ = 상자 중앙이 로봇 왼쪽 → 왼쪽으로 가야 함),
  yaw_err = wrap(nyaw) (로봇 yaw = 0).
회전 중심: 차동 구동은 바퀴 축 (ICR, base_link x −0.0658) 둘레로 돎 → plan_approach 는 ICR 이 움직이는 길로 계획
  (회전 단계에서 base_link 원점도 조금 움직이는 것을 반영, icr_x = 0 이면 base_link 제자리 회전과 같음).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

FRONT_OFFSET = 0.1328   # base_link → 상판 앞 끝 x [m]
BOX_SIDE = 0.205        # 상자 한 변 [m]
LASER_FRONT_TF = (0.1318, 0.0, -1.0, 0.0, 0.0, 1.0)   # 기본 (tx, ty, r00, r01, r10, r11): base = t + R·laser
ICR_X = -0.0658         # 회전 중심 (바퀴 축) base_link x [m]
FRONT_CORNERS = ((0.1328, 0.1676), (0.1328, -0.1676))   # 상판 앞 모서리 base_link (x, y) [m]

# find_box_face 기본값 (cfg dict 로 일부만 바꿀 수 있음)
FACE_CFG = {
    "x_min": 0.14, "x_max": 0.60, "y_abs": 0.30,   # 기본 창 (base_link)
    "expect_win": 0.10,       # expect 가 있으면 그 주변 ± 이 거리만 (법선 방향 ±win, 접선 방향 ±(BOX_SIDE/2 + win))
    "angle_inc": math.radians(1.0),   # 스캔 각 간격 Δθ (군집 문턱 · 끝점 양자화 보정)
    "laser_xy": (0.1318, 0.0),        # 라이다 위치 (거리 r 계산용)
    "clu_min": 0.03, "clu_k": 3.0,    # 군집 끊기: 이웃 간격 > max(clu_min, clu_k·r·Δθ)
    "ransac_thr": 0.008,      # RANSAC inlier 문턱 [m]
    "ransac_max_src": 80,     # 전수 쌍 탐색에 쓸 점 수 상한 (넘으면 고르게 솎음, 결정적)
    "refine_thr": 0.03,       # RANSAC 뒤 PCA 직선에서 이 거리 안 점을 창 전체에서 다시 모음 (σ 1 cm 잡음에 8 mm 만 쓰면 끝점이 빠져 짧아짐)
    "run_min": 0.010,         # 다시 모은 점의 선 방향 이웃 간격 문턱 = max(run_min, run_k · 기대 간격) 넘으면 끊음
    "run_k": 2.5,             #   기대 간격 = r·Δθ / max(cos 입사각, 0.4) — 점 1 개 빠짐은 잇고 2 cm 틈 옆 퍽은 끊음
    "rms_max": 0.005,         # 최종 inlier 직선 맞춤 RMS 잔차 상한 [m] (주의: 거리 잡음 σ 가 이보다 크면 늘 실패)
    "min_ransac": 6,          # RANSAC (8 mm) inlier 최소 — σ 1 cm 잡음이면 8 mm 안 점이 절반쯤이라 재수집 전엔 느슨하게
    "min_inliers": 10,        # 재수집 (refine_thr) 뒤 최종 inlier 최소
    "len_nom": BOX_SIDE, "len_tol": 0.025,
    "yaw_max": math.radians(25.0),
    "occl_dr": 0.02,          # 끝점 바깥 이웃이 이만큼 더 가까우면 가림 의심
    "end_quant": True,        # 끝마다 r·Δθ/2 바깥으로 더함
}

# 실물 거리 잡음 σ 가 1 cm 근처일 때 쓸 느슨한 값 (기본은 엄격 — 실패하면 움직이지 않고 기존 동작으로 가므로 안전 쪽).
#   dry 로그의 실패 why 에 찍히는 "RMS x mm" 가 늘 7 mm 넘으면 σ ≈ 1 cm 로 보고 아래를 씀:
#   find_box_face(pts, cfg=FACE_CFG_LOOSE), average_faces(faces, **AVG_LOOSE)  (AVG_LOOSE 에 min_ok 가 있으니 따로 넘기지 말 것)
#   대가: 선 방향 끊기 문턱이 2.5 cm 라 상자 옆 2 cm 틈에 놓인 퍽은 앞면에 붙어 "너무 김" 으로 실패함 (퍽 2 cm 틈 구분은 포기).
#   11-21 조정 (σ 1 cm 합성 스캔, 상자 네 변, gap 0.25 ~ 0.35, ±10°, 5 장 × 100 회 × 10 장면):
#     5 장 평균 성공 99.9 %, lateral 오차 p95 8 ~ 18 mm · yaw p95 1.1 ~ 1.9°.
#     lateral 은 거리 × yaw 오차라 (|C| 0.38 ~ 0.48 m) 5 장 · σ 1 cm 의 직선 기울기 한계 (한 장 1.4 ~ 1.7° = CRLB) 에 붙어 있음
#     → 더 줄이려면 장수를 늘리거나 가까이서 다시 재야 함 (문턱으로 골라내도 가우스 잡음이면 정확도는 안 좋아짐).
#   refine_thr 0.04 (4σ): 끝점 · 한가운데 점이 덜 빠짐 (2 cm 면 기울기 sd 1.4 → 2.1° 로 나빠짐).
#   AVG: 중앙값 ±2 cm · ±4° 밖 측정만 빼고 4 장 이상 남아야 ok (쌍별 방식은 5 장 중 한 쌍만 벗어나도 실패라 σ 1 cm 에서 40 % 실패).
FACE_CFG_LOOSE = {"rms_max": 0.015, "run_min": 0.025, "refine_thr": 0.04}
AVG_LOOSE = {"spread_mode": "median", "spread_xy": 0.04, "spread_yaw": math.radians(8.0), "min_ok": 4}

# plan_approach 기본 상한 (cfg dict 로 바꿀 수 있음)
PLAN_CFG = {
    "lat_max": 0.06,                  # |lateral| 상한 [m]
    "turn_max": math.radians(25.0),   # 회전 단계마다 상한
    "drive_max": 0.40,                # |d| 상한
    "back_max": 0.35,                 # 후진 상한
    "icr_x": ICR_X,
    "min_turn": math.radians(0.3),    # 이보다 작은 회전은 뺌
    "near_dist": 0.01,                # base_link ↔ 목표 P 가 이보다 가까우면 회전만
    "single_lat": 0.015,              # 옆 오차 이하 + 기울기 이하이면 drive 하나만
    "single_yaw": math.radians(2.0),
    "turn_check_gap": 0.10,           # 회전 자세에서 gap 이 이보다 작으면 turn_clear 검사
    "turn_margin": 0.005,             # 앞 모서리 ↔ 상자 앞면 평면 최소 여유 [m]
}


def wrap(a):
    """각을 [−π, π] 로"""
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class FaceResult:
    ok: bool
    why: str
    cx: float = 0.0
    cy: float = 0.0
    nyaw: float = 0.0
    length: float = 0.0
    n_pts: int = 0
    gap: float = 0.0
    lateral: float = 0.0
    yaw_err: float = 0.0
    rms: float = 0.0          # 직선 맞춤 RMS 잔차 [m]


def _fill(f):
    """cx, cy, nyaw 로 gap · lateral · yaw_err 채움"""
    c, s = math.cos(f.nyaw), math.sin(f.nyaw)
    f.gap = c * f.cx + s * f.cy - FRONT_OFFSET
    f.lateral = -s * f.cx + c * f.cy
    f.yaw_err = wrap(f.nyaw)
    return f


def scan_to_base(ranges, angle_min, angle_inc, range_min, range_max, tf6=LASER_FRONT_TF):
    """LaserScan → base_link (x, y) 목록 (스캔 순서). inf · nan · [range_min, range_max] 밖 제외"""
    tx, ty, r00, r01, r10, r11 = tf6
    out = []
    for i, r in enumerate(ranges):
        if r is None:
            continue
        r = float(r)
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        th = angle_min + i * angle_inc
        lx, ly = r * math.cos(th), r * math.sin(th)
        out.append((tx + r00 * lx + r01 * ly, ty + r10 * lx + r11 * ly))
    return out


# ---------------------------------------------------------------- 선분 맞춤 도우미

def _pca_line(pts):
    """최소제곱 직선: (중심 mx, my, 방향 각 φ). 방향 = 공분산 주축"""
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    syy = sum((p[1] - my) ** 2 for p in pts)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    return mx, my, 0.5 * math.atan2(2.0 * sxy, sxx - syy)


def _ransac(pts, thr, max_src):
    """결정적 RANSAC (전수 쌍, MSAC 비용). inlier 번호 목록"""
    n = len(pts)
    if n < 2:
        return list(range(n))
    step = max(1, math.ceil(n / max_src))
    src = list(range(0, n, step))
    best, best_cost = None, None
    thr2 = thr * thr
    for a in range(len(src)):
        pa = pts[src[a]]
        for b in range(a + 1, len(src)):
            pb = pts[src[b]]
            dx, dy = pb[0] - pa[0], pb[1] - pa[1]
            L = math.hypot(dx, dy)
            if L < 0.02:
                continue
            nx, ny = -dy / L, dx / L
            cost = 0.0
            for p in pts:
                e = (p[0] - pa[0]) * nx + (p[1] - pa[1]) * ny
                cost += min(e * e, thr2)
            if best_cost is None or cost < best_cost:
                best_cost, best = cost, (pa, nx, ny)
    if best is None:
        return list(range(n))
    pa, nx, ny = best
    return [i for i, p in enumerate(pts) if abs((p[0] - pa[0]) * nx + (p[1] - pa[1]) * ny) <= thr]


def _rms(inl):
    """PCA 직선에 대한 RMS 잔차"""
    mx, my, phi = _pca_line(inl)
    tx_, ty_ = math.cos(phi), math.sin(phi)
    return math.sqrt(sum((-(p[0] - mx) * ty_ + (p[1] - my) * tx_) ** 2 for p in inl) / len(inl))


def _refine(inl, allpts, thr, run_min, run_k, lx, ly, dth):
    """inlier PCA 직선 → 창 전체에서 thr 안 점을 모아 선 방향으로 이어진 구간 (원래 inlier 중앙 포함) 만 → 한 번 더.
    이웃 간격 문턱은 그 자리 기대 간격 (r·Δθ / cos 입사각) 의 run_k 배 (최소 run_min)"""

    def gap_ok(a, b, ga):
        mx_, my_ = 0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])
        r = math.hypot(mx_ - lx, my_ - ly)
        cos_inc = abs((mx_ - lx) * nx_ + (my_ - ly) * ny_) / max(r, 1e-9)
        return ga <= max(run_min, run_k * r * dth / max(cos_inc, 0.4))

    for _ in range(2):
        mx, my, phi = _pca_line(inl)
        tx_, ty_ = math.cos(phi), math.sin(phi)
        nx_, ny_ = -ty_, tx_
        near = [p for p in allpts if abs(-(p[0] - mx) * ty_ + (p[1] - my) * tx_) <= thr]
        if len(near) < 2:
            return inl
        near.sort(key=lambda p: (p[0] - mx) * tx_ + (p[1] - my) * ty_)
        ss = [(p[0] - mx) * tx_ + (p[1] - my) * ty_ for p in near]
        k = min(range(len(ss)), key=lambda i: abs(ss[i]))   # 중심 (s = 0) 에 가장 가까운 점에서 양쪽으로
        lo = hi = k
        while lo > 0 and gap_ok(near[lo - 1], near[lo], ss[lo] - ss[lo - 1]):
            lo -= 1
        while hi < len(ss) - 1 and gap_ok(near[hi], near[hi + 1], ss[hi + 1] - ss[hi]):
            hi += 1
        inl = near[lo:hi + 1]
    return inl


def _expect_xyyaw(expect):
    if expect is None:
        return None
    if isinstance(expect, FaceResult):
        return expect.cx, expect.cy, expect.nyaw
    e = tuple(expect)
    return float(e[0]), float(e[1]), (float(e[2]) if len(e) > 2 else 0.0)


def _segment(inl, lx, ly, dth, quant):
    """inlier 점 → (cx, cy, nyaw, length, 끝점 2 개, 끝점 laser 거리 2 개)"""
    mx, my, phi = _pca_line(inl)
    tx_, ty_ = math.cos(phi), math.sin(phi)
    nx, ny = ty_, -tx_
    if nx * mx + ny * my < 0:          # 법선은 로봇 → 상자 쪽 (n·C > 0)
        nx, ny = -nx, -ny
    tx_, ty_ = -ny, nx                 # 접선 = 법선 왼쪽 90°
    d = nx * mx + ny * my
    ss = [tx_ * p[0] + ty_ * p[1] for p in inl]
    i0 = min(range(len(ss)), key=ss.__getitem__)
    i1 = max(range(len(ss)), key=ss.__getitem__)
    s0, s1 = ss[i0], ss[i1]
    r0 = math.hypot(inl[i0][0] - lx, inl[i0][1] - ly)
    r1 = math.hypot(inl[i1][0] - lx, inl[i1][1] - ly)
    if quant:
        s0 -= r0 * dth / 2.0
        s1 += r1 * dth / 2.0
    sm = 0.5 * (s0 + s1)
    cx, cy = nx * d + tx_ * sm, ny * d + ty_ * sm
    return cx, cy, math.atan2(ny, nx), s1 - s0, (inl[i0], inl[i1]), (r0, r1)


def _occluded(end, r_end, allpts, lx, ly, dth, dr, inl_set):
    """끝점 각 바로 바깥 (4Δθ 안) 에 더 가까운 점이 있으면 가림 의심"""
    a_end = math.atan2(end[1] - ly, end[0] - lx)
    for p in allpts:
        if p in inl_set:
            continue
        da = abs(wrap(math.atan2(p[1] - ly, p[0] - lx) - a_end))
        if 0 < da <= 4.0 * dth and math.hypot(p[0] - lx, p[1] - ly) < r_end - dr:
            return True
    return False


# ---------------------------------------------------------------- 상자 앞면 찾기

def find_box_face(pts, expect=None, cfg=None) -> FaceResult:
    """base_link 점 목록에서 상자 앞면 선분을 찾음. expect = (cx, cy[, nyaw]) 또는 FaceResult (그 주변만 봄)"""
    c = {**FACE_CFG, **(cfg or {})}
    lx, ly = c["laser_xy"]
    dth = c["angle_inc"]
    # 1. 창 거르기
    ex = _expect_xyyaw(expect)
    win = []
    for p in pts:
        x, y = float(p[0]), float(p[1])
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        if ex is None:
            if c["x_min"] <= x <= c["x_max"] and abs(y) <= c["y_abs"]:
                win.append((x, y))
        else:
            ecx, ecy, eyaw = ex
            dx, dy = x - ecx, y - ecy
            along = dx * math.cos(eyaw) + dy * math.sin(eyaw)
            side = -dx * math.sin(eyaw) + dy * math.cos(eyaw)
            if abs(along) <= c["expect_win"] and abs(side) <= BOX_SIDE / 2 + c["expect_win"]:
                win.append((x, y))
    if len(win) < c["min_inliers"]:
        return FaceResult(False, f"상자 못 찾음 (창 안 점 {len(win)} 개) · RMS - · 점 {len(win)} 개", n_pts=len(win))
    # 라이다 기준 각으로 정렬 (스캔 시작각이 앞쪽에 걸려도 군집이 끊기지 않게, 앞 = 각 0 근처)
    win.sort(key=lambda p: math.atan2(p[1] - ly, p[0] - lx))
    # 2. 군집
    clusters, cur = [], [win[0]]
    for a, b in zip(win, win[1:]):
        r = max(math.hypot(a[0] - lx, a[1] - ly), math.hypot(b[0] - lx, b[1] - ly))
        if math.hypot(b[0] - a[0], b[1] - a[1]) > max(c["clu_min"], c["clu_k"] * r * dth):
            clusters.append(cur)
            cur = []
        cur.append(b)
    clusters.append(cur)
    # 3 ~ 4. 군집마다 선분 (L 자 모서리면 첫 선분 inlier 를 빼고 한 번 더)
    cands, fails = [], []
    for cl in clusters:
        rest = cl
        for _ in range(2):
            if len(rest) < c["min_inliers"]:
                break
            idx = _ransac(rest, c["ransac_thr"], c["ransac_max_src"])
            inl = [rest[i] for i in idx]
            iset = set(idx)
            rest = [p for i, p in enumerate(rest) if i not in iset]
            if len(inl) < c["min_ransac"]:
                fails.append((len(inl), f"선분 점 {len(inl)} 개 < {c['min_ransac']} (RANSAC)", _rms(inl) if len(inl) >= 2 else None))
                continue
            inl = _refine(inl, win, c["refine_thr"], c["run_min"], c["run_k"], lx, ly, dth)
            if len(inl) < c["min_inliers"]:
                fails.append((len(inl), f"선분 점 {len(inl)} 개 < {c['min_inliers']}", _rms(inl) if len(inl) >= 2 else None))
                continue
            cx, cy, nyaw, length, ends, rends = _segment(inl, lx, ly, dth, c["end_quant"])
            if any(abs(cx - o.cx) < 0.01 and abs(cy - o.cy) < 0.01 for _, o in cands):
                continue                      # 쪼개진 군집에서 같은 면이 또 나옴
            rms = _rms(inl)
            ye = wrap(nyaw)
            if abs(ye) > c["yaw_max"]:
                fails.append((len(inl), f"기울기 {math.degrees(abs(ye)):.0f}° > {math.degrees(c['yaw_max']):.0f}°", rms))
                continue
            if abs(length - c["len_nom"]) > c["len_tol"]:
                msg = f"길이 {length:.3f} m ≠ {c['len_nom']:.3f}"
                if length < c["len_nom"]:
                    inl_set = set(inl)
                    occ = [_occluded(e, re, win, lx, ly, dth, c["occl_dr"], inl_set) for e, re in zip(ends, rends)]
                    msg += " (끝 가림 의심)" if any(occ) else " (가림 없음, 짧음)"
                else:
                    msg += " (너무 김 — 벽 · 이웃 상자와 붙었을 수 있음)"
                fails.append((len(inl), msg, rms))
                continue
            if rms > c["rms_max"]:
                fails.append((len(inl), f"직선 잔차 RMS {rms * 1000:.1f} mm > {c['rms_max'] * 1000:.1f} mm", rms))
                continue
            dist = math.hypot(cx, cy)
            score = abs(length - c["len_nom"]) / c["len_tol"] + dist / c["x_max"]
            cands.append((score, FaceResult(True, "", cx, cy, nyaw, length, len(inl), rms=rms)))
    # 5. 고르기
    if not cands:
        if fails:
            fails.sort(key=lambda t: -t[0])
            n, msg, rms = fails[0]
            rtxt = f"{rms * 1000:.1f} mm" if rms is not None else "-"
            return FaceResult(False, f"{msg} · RMS {rtxt} · 점 {n} 개 (창 {len(win)} 개)", n_pts=n,
                              rms=rms if rms is not None else 0.0)
        return FaceResult(False, f"상자 못 찾음 (군집 {len(clusters)} 개 모두 점 부족) · RMS - · 점 0 개 (창 {len(win)} 개)",
                          n_pts=len(win))
    cands.sort(key=lambda t: t[0])
    f = _fill(cands[0][1])
    f.why = f"상자면 길이 {f.length:.3f} m · 점 {f.n_pts} 개 · RMS {f.rms * 1000:.1f} mm · 후보 {len(cands)} 개"
    return f


AVG_SPREAD_XY = 0.01               # ok 측정끼리 중점 차 상한 [m] (모든 쌍)
AVG_SPREAD_YAW = math.radians(2.0)  # ok 측정끼리 nyaw 차 상한 (모든 쌍)


def _med(v):
    v = sorted(v)
    m = len(v) // 2
    return v[m] if len(v) % 2 else 0.5 * (v[m - 1] + v[m])


def average_faces(faces, min_ok=2, spread_xy=AVG_SPREAD_XY, spread_yaw=AVG_SPREAD_YAW, spread_mode="pair") -> FaceResult:
    """ok 인 측정만 평균: nyaw 원형 평균, cx · cy · length 중앙값.
    일관성 (spread_mode):
      "pair"   (기본) — ok 측정 쌍 가운데 중점 차 > spread_xy 또는 yaw 차 > spread_yaw 가 있으면 실패 ("측정 흔들림")
      "median" — 중앙값 (cx, cy, nyaw) 에서 중점 거리 > spread_xy/2 또는 yaw 차 > spread_yaw/2 인 측정은 빼고,
                 남은 수 ≥ min_ok 이면 남은 것만 평균 (AVG_LOOSE 용)"""
    ok = [f for f in faces if f is not None and f.ok]
    n_in = len(ok)
    if spread_mode == "median" and len(ok) >= max(1, min_ok):
        mcx, mcy = _med([f.cx for f in ok]), _med([f.cy for f in ok])
        m0 = ok[0].nyaw
        myaw = m0 + _med([wrap(f.nyaw - m0) for f in ok])
        keep = [f for f in ok if math.hypot(f.cx - mcx, f.cy - mcy) <= spread_xy / 2
                and abs(wrap(f.nyaw - myaw)) <= spread_yaw / 2]
        if len(keep) < max(1, min_ok):
            return FaceResult(False, f"측정 흔들림 (중앙값 ±{spread_xy * 50:.1f} cm · ±{math.degrees(spread_yaw) / 2:.1f}° 안 "
                                     f"{len(keep)} 개 < {min_ok}, ok {len(ok)} 개)"
                                     f" · RMS 중앙 {_med([f.rms for f in ok]) * 1000:.1f} mm", n_pts=len(keep))
        ok = keep
    elif spread_mode not in ("pair", "median"):
        raise ValueError(f"spread_mode {spread_mode!r} ∉ (pair, median)")
    if len(ok) < max(1, min_ok):
        bad = [f for f in faces if f is not None and not f.ok]
        rr = sorted(f.rms for f in faces if f is not None and f.rms > 0)
        rtxt = f"{rr[len(rr) // 2] * 1000:.1f} mm" if rr else "-"
        return FaceResult(False, f"유효 측정 {len(ok)} 개 < {min_ok} (전체 {len(faces)} 개) · RMS 중앙 {rtxt}"
                                 + (f" · 첫 실패: {bad[0].why}" if bad else ""))
    dxy = max((math.hypot(a.cx - b.cx, a.cy - b.cy) for i, a in enumerate(ok) for b in ok[i + 1:]), default=0.0)
    dyaw = max((abs(wrap(a.nyaw - b.nyaw)) for i, a in enumerate(ok) for b in ok[i + 1:]), default=0.0)
    if spread_mode == "pair" and (dxy > spread_xy or dyaw > spread_yaw):
        return FaceResult(False, f"측정 흔들림 (중점 차 {dxy * 100:.1f} cm / {spread_xy * 100:.1f}, "
                                 f"yaw 차 {math.degrees(dyaw):.1f}° / {math.degrees(spread_yaw):.1f}°, ok {len(ok)} 개)"
                                 f" · RMS 중앙 {sorted(f.rms for f in ok)[len(ok) // 2] * 1000:.1f} mm"
                                 f" · 점 중앙 {sorted(f.n_pts for f in ok)[len(ok) // 2]} 개",
                          n_pts=len(ok))

    def med(v):
        v = sorted(v)
        m = len(v) // 2
        return v[m] if len(v) % 2 else 0.5 * (v[m - 1] + v[m])

    nyaw = math.atan2(sum(math.sin(f.nyaw) for f in ok), sum(math.cos(f.nyaw) for f in ok))
    out = FaceResult(True, "", med([f.cx for f in ok]), med([f.cy for f in ok]), nyaw,
                     med([f.length for f in ok]), int(round(med([f.n_pts for f in ok]))),
                     rms=med([f.rms for f in ok]))
    _fill(out)
    out.why = f"평균 {len(ok)}/{len(faces)} 개" + (f" (중앙값 밖 {n_in - len(ok)} 개 뺌)" if n_in > len(ok) else "") + f" · RMS {out.rms * 1000:.1f} mm · 점 {out.n_pts} 개"
    return out


# ---------------------------------------------------------------- 다가가기 계획

def _pose_mul(pose, dx, dy):
    x, y, th = pose
    return x + math.cos(th) * dx - math.sin(th) * dy, y + math.sin(th) * dx + math.cos(th) * dy


def turn_clearance(face, angle, pose=(0.0, 0.0, 0.0), cfg=None):
    """pose (측정 base_link 기준 x, y, yaw) 에서 angle 만큼 ICR 둘레로 돌 때 앞 모서리 ↔ 상자 앞면 평면 최소 여유 [m].
    음수 = 평면을 넘음 (부딪힐 수 있음). 평면 기준이라 보수적 (상자 폭 밖 모서리도 셈)"""
    c = {**PLAN_CFG, **(cfg or {})}
    nx, ny = math.cos(face.nyaw), math.sin(face.nyaw)
    d = nx * face.cx + ny * face.cy
    ix, iy = _pose_mul(pose, c["icr_x"], 0.0)
    worst = math.inf
    steps = max(2, int(abs(angle) / math.radians(1.0)) + 2)
    for corner in FRONT_CORNERS:
        px, py = _pose_mul(pose, corner[0], corner[1])
        vx, vy = px - ix, py - iy
        for k in range(steps + 1):
            a = angle * k / steps
            ca, sa = math.cos(a), math.sin(a)
            qx, qy = ix + ca * vx - sa * vy, iy + sa * vx + ca * vy
            worst = min(worst, d - (nx * qx + ny * qy))
    return worst


def turn_clear(face, angle, pose=(0.0, 0.0, 0.0), cfg=None):
    """회전 중 앞 모서리가 상자 앞면 평면 (− 여유) 을 넘지 않으면 True"""
    c = {**PLAN_CFG, **(cfg or {})}
    return turn_clearance(face, angle, pose, c) >= c["turn_margin"]


def plan_approach(face, gap_target=0.05, standoff=0.0, cfg=None):
    """(단계 목록 | None, 이유). 단계 = ("turn", rad 왼쪽 +) · ("drive", m 앞 +).
    목표 base_link 자세 P = C − n·(FRONT_OFFSET + gap_target + standoff), heading = nyaw.
    길은 ICR (바퀴 축) 이 지나는 길: 돌기 a1 → 곧게 d → 돌기 a2"""
    c = {**PLAN_CFG, **(cfg or {})}
    if face is None or not face.ok:
        return None, "상자면 측정 실패: " + (face.why if face is not None else "없음")
    _fill(face)
    if abs(face.lateral) > c["lat_max"]:
        return None, f"옆 어긋남 {face.lateral * 100:+.1f} cm > {c['lat_max'] * 100:.1f} cm"
    psi = wrap(face.nyaw)
    back = FRONT_OFFSET + gap_target + standoff
    px, py = face.cx - math.cos(psi) * back, face.cy - math.sin(psi) * back
    icr = c["icr_x"]
    q0 = (icr, 0.0)                                        # 지금 ICR
    qx, qy = _pose_mul((px, py, psi), icr, 0.0)            # 목표 ICR
    vx, vy = qx - q0[0], qy - q0[1]
    dist = math.hypot(vx, vy)

    if math.hypot(px, py) < c["near_dist"]:                # base_link 가 이미 목표 근처 → 회전만 (ICR 둘레라 원점이 |icr|·각 만큼 밀림)
        steps = [("turn", psi)]
    elif abs(vy) <= c["single_lat"] and abs(psi) <= c["single_yaw"]:
        steps = [("drive", vx)]
    else:
        bearing = math.atan2(vy, vx)
        if vx < 0:                                         # 뒤쪽 → 후진
            a1, d = wrap(bearing + math.pi), -dist
        else:
            a1, d = bearing, dist
        steps = [("turn", a1), ("drive", d), ("turn", wrap(psi - a1))]
    steps = [s for s in steps if not (s[0] == "turn" and abs(s[1]) < c["min_turn"])]

    # 상한
    for kind, v in steps:
        if kind == "turn" and abs(v) > c["turn_max"]:
            return None, f"회전 {math.degrees(v):+.1f}° > {math.degrees(c['turn_max']):.0f}°"
        if kind == "drive":
            if v < 0 and -v > c["back_max"]:
                return None, f"후진 {-v:.3f} m > {c['back_max']:.2f} m"
            if abs(v) > c["drive_max"]:
                return None, f"직진 {v:.3f} m > {c['drive_max']:.2f} m"

    # 근거리 회전 안전: 회전 자세의 gap 이 turn_check_gap 보다 작으면 앞 모서리 쓸기 검사
    pose = (0.0, 0.0, 0.0)
    nx, ny = math.cos(face.nyaw), math.sin(face.nyaw)
    dplane = nx * face.cx + ny * face.cy
    for i, (kind, v) in enumerate(steps):
        if kind == "turn":
            gap_here = dplane - (nx * pose[0] + ny * pose[1]) - FRONT_OFFSET
            if gap_here < c["turn_check_gap"]:
                cl = turn_clearance(face, v, pose, c)
                if cl < c["turn_margin"]:
                    return None, (f"{i + 1} 단계 회전 {math.degrees(v):+.1f}° 때 앞 모서리가 상자면 "
                                  f"{-cl * 100:.1f} cm 넘음 (여유 {c['turn_margin'] * 100:.1f} cm 필요)")
            ix, iy = _pose_mul(pose, icr, 0.0)              # ICR 고정 회전
            th = pose[2] + v
            pose = (ix - math.cos(th) * icr, iy - math.sin(th) * icr, th)
        else:
            pose = (pose[0] + math.cos(pose[2]) * v, pose[1] + math.sin(pose[2]) * v, pose[2])

    desc = " → ".join(f"돌기 {math.degrees(v):+.1f}°" if k == "turn" else f"{'직진' if v >= 0 else '후진'} {abs(v):.3f} m"
                      for k, v in steps) or "그대로"
    return steps, f"gap {face.gap:.3f} → {gap_target + standoff:.3f} m, 옆 {face.lateral * 100:+.1f} cm, " \
                  f"기울기 {math.degrees(face.yaw_err):+.1f}°: {desc}"


def simulate(steps, icr_x=ICR_X):
    """단계 실행 뒤 base_link 자세 (x, y, yaw) — 시험용 (ICR 둘레 회전 · 곧은 주행)"""
    pose = (0.0, 0.0, 0.0)
    for kind, v in steps or []:
        if kind == "turn":
            ix, iy = _pose_mul(pose, icr_x, 0.0)
            th = pose[2] + v
            pose = (ix - math.cos(th) * icr_x, iy - math.sin(th) * icr_x, th)
        else:
            pose = (pose[0] + math.cos(pose[2]) * v, pose[1] + math.sin(pose[2]) * v, pose[2])
    return pose
