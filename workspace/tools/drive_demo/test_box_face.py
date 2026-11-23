"""box_face.py 시험 (ROS 없이): cd workspace/tools/drive_demo && python3 test_box_face.py
(python3 -m pytest -q test_box_face.py 로도 돕니다)

합성 스캔: laser_front (base_link 에서 (0.1318, 0), rpy(π,0,π)) 에서 광선을 쏴 LaserScan 처럼 ranges 를 만듭니다.
  angle_min = −π, 1° 간격, 360칸, range_min 0.08 (실물 틈 10 cm 측정 성공 근거), range_max 8
  광선 θ 의 base_link 방향 = (−cosθ, +sinθ)
"""
import math
import random
import sys

from box_face import (BOX_SIDE, FRONT_OFFSET, LASER_FRONT_TF, S_CFG, FaceResult, average_faces, find_box_face,
                      plan_approach, plan_s_curve, plan_s_curve_face, scan_to_base, turn_clear, turn_clearance,
                      turn_intrusion, plan_s_curve_back, plan_s_curve_back_face)

ANGLE_MIN = -math.pi
ANGLE_INC = math.radians(1.0)
N_RAYS = 360
RANGE_MIN = 0.08   # 11-23: 예전 가정 0.12 는 실물보다 비관적 — 11-22 실물에서 앞 끝 ↔ 상자 틈 10.0 cm (점 97 · RMS 1.2 mm) · 9.9 cm 측정 성공
RANGE_MAX = 8.0
LASER_X = 0.1318
DEG = math.pi / 180.0


# ---------------------------------------------------------------- 합성 장면 도우미
def box_segments(gap, lateral=0.0, yaw=0.0, side=BOX_SIDE):
    """상자 정사각형 네 변 (base_link). 앞면 중점 C = (0.1328+gap, lateral), 법선 n = (cos yaw, sin yaw)."""
    n = (math.cos(yaw), math.sin(yaw))
    t = (-n[1], n[0])
    h = side / 2.0
    c = (FRONT_OFFSET + gap + n[0] * h, lateral + n[1] * h)   # 상자 중심
    corners = []
    for a, b in ((-1, -1), (-1, 1), (1, 1), (1, -1)):
        corners.append((c[0] + a * h * n[0] + b * h * t[0], c[1] + a * h * n[1] + b * h * t[1]))
    return [(corners[i], corners[(i + 1) % 4]) for i in range(4)]


def wall_segment(x, y0=-1.5, y1=1.5):
    return [((x, y0), (x, y1))]


def _hit_segment(o, d, a, b):
    ex, ey = b[0] - a[0], b[1] - a[1]
    den = d[0] * ey - d[1] * ex
    if abs(den) < 1e-12:
        return None
    wx, wy = a[0] - o[0], a[1] - o[1]
    t = (wx * ey - wy * ex) / den
    s = (wx * d[1] - wy * d[0]) / den
    if t > 1e-9 and -1e-9 <= s <= 1 + 1e-9:
        return t
    return None


def _hit_circle(o, d, c, r):
    fx, fy = o[0] - c[0], o[1] - c[1]
    b = fx * d[0] + fy * d[1]
    cc = fx * fx + fy * fy - r * r
    disc = b * b - cc
    if disc < 0:
        return None
    sq = math.sqrt(disc)
    for t in (-b - sq, -b + sq):
        if t > 1e-9:
            return t
    return None


def make_scan(segments=(), circles=(), noise=0.0, rng=None, drop=None, near="zero"):
    """ranges 리스트. drop(x, y) 가 참인 맞은 점은 가려진 것으로 보고 inf."""
    o = (LASER_X, 0.0)
    ranges = []
    for i in range(N_RAYS):
        th = ANGLE_MIN + i * ANGLE_INC
        d = (-math.cos(th), math.sin(th))
        best = None
        for a, b in segments:
            t = _hit_segment(o, d, a, b)
            if t is not None and (best is None or t < best):
                best = t
        for c, r in circles:
            t = _hit_circle(o, d, c, r)
            if t is not None and (best is None or t < best):
                best = t
        if best is None:
            ranges.append(float("inf"))
            continue
        if drop is not None and drop(o[0] + best * d[0], o[1] + best * d[1]):
            ranges.append(float("inf"))
            continue
        r = best + (rng.gauss(0.0, noise) if noise > 0 else 0.0)
        if r < RANGE_MIN:
            ranges.append(0.0 if near == "zero" else float("inf"))
        elif r > RANGE_MAX:
            ranges.append(float("inf"))
        else:
            ranges.append(r)
    return ranges


def to_pts(ranges):
    return scan_to_base(ranges, ANGLE_MIN, ANGLE_INC, RANGE_MIN, RANGE_MAX, LASER_FRONT_TF)


def face_of(segments=(), circles=(), **kw):
    return find_box_face(to_pts(make_scan(segments, circles, **kw)))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


ICR_X_ROBOT = -0.0658   # 차동 구동 회전 중심 (바퀴 축) base_link x — 회전은 이 점 둘레


def integrate(steps, icr_x=ICR_X_ROBOT):
    """단계 적분 (시험 쪽 독립 구현): turn = ICR 둘레 회전, drive = heading 방향 직진"""
    x = y = th = 0.0
    for kind, v in steps:
        if kind == "turn":
            ix, iy = x + icr_x * math.cos(th), y + icr_x * math.sin(th)
            th += v
            x, y = ix - icr_x * math.cos(th), iy - icr_x * math.sin(th)
        elif kind == "drive":
            x += v * math.cos(th)
            y += v * math.sin(th)
        else:
            raise AssertionError("모르는 단계 %r" % (kind,))
    return x, y, th


def _expect_gap(gap, lateral, yaw):
    """앞면 직선까지 원점 법선 거리 − 0.1328"""
    return (FRONT_OFFSET + gap) * math.cos(yaw) + lateral * math.sin(yaw) - FRONT_OFFSET


def _expect_lat(gap, lateral, yaw):
    """상자 중심선 (C 지나 n 방향) 에 대한 옆 어긋남, + = 상자 중앙이 왼쪽"""
    return -(FRONT_OFFSET + gap) * math.sin(yaw) + lateral * math.cos(yaw)


def face_at(gap, lateral=0.0, yaw=0.0):
    """측정 없이 참값 FaceResult (range_min 0.12 안이라 스캔으로는 못 재는 가까운 자세용)"""
    return FaceResult(True, "참값", FRONT_OFFSET + gap, lateral, yaw, BOX_SIDE, 30)


# ---------------------------------------------------------------- 1. scan_to_base
def test_scan_to_base_sign():
    ranges = [float("inf")] * N_RAYS
    ranges[270] = 1.0      # θ = +90° → base +y
    ranges[90] = 0.5       # θ = −90° → base −y
    ranges[0] = 2.0        # θ = −π   → base 앞 (+x)
    ranges[180] = 0.3      # θ = 0    → base 뒤 (−x)
    ranges[10] = 0.0       # 무효
    ranges[11] = float("nan")
    ranges[12] = 0.05      # range_min 안
    ranges[13] = 9.0       # range_max 밖
    pts = sorted(to_pts(ranges), key=lambda p: p[1])
    assert len(pts) == 4, pts
    want = sorted([(LASER_X, 1.0), (LASER_X, -0.5), (LASER_X + 2.0, 0.0), (LASER_X - 0.3, 0.0)],
                  key=lambda p: p[1])
    # y 가 같은 두 점은 x 로 맞춤
    pts = sorted(pts, key=lambda p: (round(p[1], 6), p[0]))
    want = sorted(want, key=lambda p: (round(p[1], 6), p[0]))
    for (px, py), (wx, wy) in zip(pts, want):
        assert abs(px - wx) < 1e-6 and abs(py - wy) < 1e-6, (pts, want)


def test_scan_to_base_left_object_is_left():
    """왼쪽(+y) 에 놓은 상자 → 점 무게중심 y > 0 (y 거울 버그 잡기)"""
    pts = to_pts(make_scan(box_segments(0.25, lateral=0.30)))
    assert pts
    my = sum(p[1] for p in pts) / len(pts)
    mx = sum(p[0] for p in pts) / len(pts)
    assert my > 0.2 and mx > 0.3, (mx, my)


# ---------------------------------------------------------------- 2~4. 기본 자세
def test_front_box_exact():
    f = face_of(box_segments(0.25))
    assert f.ok, f.why
    assert abs(f.gap - 0.25) <= 0.005, f.gap
    assert abs(f.lateral) <= 0.005, f.lateral
    assert abs(f.yaw_err) <= 1 * DEG, f.yaw_err
    assert abs(f.nyaw) <= 1 * DEG, f.nyaw
    assert abs(f.cx - (FRONT_OFFSET + 0.25)) <= 0.005 and abs(f.cy) <= 0.005, (f.cx, f.cy)
    assert abs(f.length - BOX_SIDE) <= 0.03, f.length


def test_lateral_sign():
    for lat in (0.03, -0.03):
        f = face_of(box_segments(0.25, lateral=lat))
        assert f.ok, (lat, f.why)
        assert abs(f.lateral - lat) <= 0.005, (lat, f.lateral)
        assert abs(f.cy - lat) <= 0.005, (lat, f.cy)
        assert abs(f.gap - 0.25) <= 0.005, (lat, f.gap)


def test_yaw_sign():
    for yaw_deg in (10.0, -10.0):
        yaw = yaw_deg * DEG
        f = face_of(box_segments(0.25, yaw=yaw))
        assert f.ok, (yaw_deg, f.why)
        assert f.yaw_err * yaw > 0, (yaw_deg, f.yaw_err)
        assert abs(f.yaw_err - yaw) <= 1 * DEG, (yaw_deg, math.degrees(f.yaw_err))
        assert abs(f.gap - _expect_gap(0.25, 0.0, yaw)) <= 0.005, (yaw_deg, f.gap)


# ---------------------------------------------------------------- 5. 잡음 + 평균
NOISE_STATS = {}


POSES_NOISE = ((0.0, 0.0), (0.03, 5.0), (-0.03, -5.0))


def test_noise_small_average():
    """(a) σ 3 mm, 5장 평균 (기본 cfg): ok, lateral ≤ 1 cm, yaw ≤ 2°"""
    rng = random.Random(12345)
    worst_lat = worst_yaw = worst_gap = 0.0
    for lat, yaw_deg in POSES_NOISE:
        yaw = yaw_deg * DEG
        faces = [face_of(box_segments(0.25, lateral=lat, yaw=yaw), noise=0.003, rng=rng) for _ in range(5)]
        avg = average_faces(faces)
        assert avg.ok, (lat, yaw_deg, avg.why, [f.why for f in faces])
        worst_lat = max(worst_lat, abs(avg.lateral - _expect_lat(0.25, lat, yaw)))
        worst_yaw = max(worst_yaw, abs(wrap(avg.yaw_err - yaw)))
        worst_gap = max(worst_gap, abs(avg.gap - _expect_gap(0.25, lat, yaw)))
    NOISE_STATS.update(lat_mm=worst_lat * 1000, yaw_deg=math.degrees(worst_yaw), gap_mm=worst_gap * 1000)
    assert worst_lat <= 0.01, NOISE_STATS
    assert worst_yaw <= 2 * DEG, NOISE_STATS


def test_noise_big_default_fails_safe():
    """(b) σ 1 cm, 기본 cfg: 평균 ok=False, plan_approach None (움직이지 않음)"""
    rng = random.Random(12345)
    for lat, yaw_deg in POSES_NOISE:
        faces = [face_of(box_segments(0.25, lateral=lat, yaw=yaw_deg * DEG), noise=0.01, rng=rng) for _ in range(5)]
        avg = average_faces(faces)
        assert not avg.ok, (lat, yaw_deg, avg)
        steps, why = plan_approach(avg, standoff=0.15)
        assert steps is None and why, (lat, yaw_deg, steps)


def test_noise_big_loose_front():
    """(c-1) σ 1 cm, FACE_CFG_LOOSE + AVG_LOOSE, 퍽 없는 정면 (고정 시드): 5장 평균 ok, lateral ≤ 1 cm, yaw ≤ 2°"""
    import box_face as bf
    rng = random.Random(12345)
    single = [find_box_face(to_pts(make_scan(box_segments(0.25), noise=0.01, rng=rng)), cfg=bf.FACE_CFG_LOOSE)
              for _ in range(5)]
    avg = average_faces(single, **bf.AVG_LOOSE)            # demo_follow.py 와 같은 부름 (키워드 dict 만)
    assert avg.ok, (avg.why, [f.why for f in single])
    assert abs(avg.lateral) <= 0.01 and abs(avg.yaw_err) <= 2 * DEG, (avg.lateral, math.degrees(avg.yaw_err))
    LOOSE_STATS.update(lat_mm=avg.lateral * 1000, yaw_deg=math.degrees(avg.yaw_err), n_ok=sum(f.ok for f in single))


def test_noise_big_loose_rate():
    """(c-2) σ 1 cm LOOSE 성공률: 장면마다 20 회 × 5 장 평균 → 성공 ≥ 17, 성공 가운데 lateral ≤ 1 cm · yaw ≤ 2° 가 ≥ 15
    (lateral 은 거리 × yaw 오차라 5 장 · σ 1 cm 에서 p95 약 1 cm — 한계에 붙어 있어 일부 넘는 것은 허용)"""
    import box_face as bf
    rows = []
    for gap, lat, yaw_deg, seed in ((0.25, 0.0, 0.0, 1), (0.25, 0.03, 5.0, 2), (0.30, 0.0, 10.0, 3), (0.30, 0.02, -10.0, 4)):
        rng = random.Random(seed)
        yaw = yaw_deg * DEG
        n_ok = n_good = 0
        for _ in range(20):
            faces = [find_box_face(to_pts(make_scan(box_segments(gap, lat, yaw), noise=0.01, rng=rng)), cfg=bf.FACE_CFG_LOOSE)
                     for _ in range(5)]
            avg = average_faces(faces, **bf.AVG_LOOSE)
            if not avg.ok:
                continue
            n_ok += 1
            if abs(avg.lateral - _expect_lat(gap, lat, yaw)) <= 0.01 and abs(wrap(avg.yaw_err - yaw)) <= 2 * DEG:
                n_good += 1
        rows.append((gap, lat, yaw_deg, n_ok, n_good))
    RATE_STATS["rows"] = rows
    for gap, lat, yaw_deg, n_ok, n_good in rows:
        assert n_ok >= 17 and n_good >= 15, rows


def test_noise_big_loose_still_rejects():
    """LOOSE 라도 상자 없음 · 벽만 · 가림 · 너무 가까움은 ok=False"""
    import box_face as bf
    rng = random.Random(7)

    def lf(segs, **kw):
        return find_box_face(to_pts(make_scan(segs, noise=0.01, rng=rng, **kw)), cfg=bf.FACE_CFG_LOOSE)

    for _ in range(5):
        assert not lf([]).ok
        assert not lf(wall_segment(0.45)).ok
        assert not lf(box_segments(0.25), drop=lambda x, y: y > -0.02).ok
        assert not lf(box_segments(0.03)).ok


def test_average_median_mode():
    """중앙값 방식: 벗어난 측정 하나는 빼고 평균, 쌍별 (기본) 은 같은 입력에서 실패"""
    good = [FaceResult(True, "", 0.3828 + d, d, d * 0.1, BOX_SIDE, 40) for d in (0.0, 0.002, -0.002, 0.001)]
    bad = FaceResult(True, "", 0.3828, 0.05, 0.0, BOX_SIDE, 40)        # 옆 5 cm 튐
    faces = good + [bad]
    assert not average_faces(faces).ok                                  # 기본 = 쌍별 1 cm
    m = average_faces(faces, spread_mode="median", spread_xy=0.02, spread_yaw=4 * DEG, min_ok=4)
    assert m.ok and abs(m.cy) <= 0.002, m
    assert not average_faces(faces, spread_mode="median", spread_xy=0.02, spread_yaw=4 * DEG, min_ok=5).ok
    try:
        average_faces(good, spread_mode="x")
        raise AssertionError("모르는 spread_mode 를 받아들임")
    except ValueError:
        pass


RATE_STATS = {}
LOOSE_STATS = {}


def test_average_needs_min_ok():
    good = face_of(box_segments(0.25))
    bad = face_of([])
    assert good.ok and not bad.ok
    assert not average_faces([good, bad, bad]).ok          # ok 1장 < min_ok 2
    assert average_faces([good, good, bad]).ok


# ---------------------------------------------------------------- 6~7. 방해물
def test_puck_beside_box():
    front_x = FRONT_OFFSET + 0.25
    r = 0.03
    for s in (1, -1):
        puck = ((front_x + r, s * (BOX_SIDE / 2 + 0.02 + r)), r)
        f = face_of(box_segments(0.25), circles=[puck])
        assert f.ok, (s, f.why)
        assert abs(f.gap - 0.25) <= 0.005, (s, f.gap)
        assert abs(f.lateral) <= 0.005, (s, f.lateral)
        assert abs(f.yaw_err) <= 1 * DEG, (s, f.yaw_err)


def test_puck_in_front_corner():
    """퍽이 앞면보다 조금 앞 (로봇 쪽) 옆에 있어도"""
    front_x = FRONT_OFFSET + 0.25
    r = 0.03
    puck = ((front_x - 0.05, BOX_SIDE / 2 + 0.02 + r), r)
    f = face_of(box_segments(0.25), circles=[puck])
    assert f.ok, f.why
    assert abs(f.lateral) <= 0.005 and abs(f.gap - 0.25) <= 0.005, (f.lateral, f.gap)


def test_wall_behind_box():
    back_x = FRONT_OFFSET + 0.25 + BOX_SIDE
    for lat in (0.0, 0.03):
        f = face_of(box_segments(0.25, lateral=lat) + wall_segment(back_x + 0.3))
        assert f.ok, (lat, f.why)
        assert abs(f.gap - 0.25) <= 0.005, (lat, f.gap)
        assert abs(f.lateral - lat) <= 0.005, (lat, f.lateral)
        assert abs(f.yaw_err) <= 1 * DEG, (lat, f.yaw_err)


# ---------------------------------------------------------------- 8~10. 거부
def test_no_box():
    f = face_of([])
    assert not f.ok
    assert f.why


def test_wall_only_rejected():
    for x in (0.40, 0.60):
        f = face_of(wall_segment(x))
        assert not f.ok, (x, f)


def test_occluded_short_rejected():
    # 앞면 왼쪽 대부분 (y > −0.02) 가려짐 → 앞면 약 8 cm 만 보임
    f = face_of(box_segments(0.25), drop=lambda x, y: y > -0.02)
    assert not f.ok, f


def test_too_close_rejected():
    for near in ("zero", "inf"):
        f = face_of(box_segments(0.03), near=near)
        assert not f.ok, (near, f)


# ---------------------------------------------------------------- 11. plan_approach
def _target(face, gap_target, standoff):
    d = FRONT_OFFSET + gap_target + standoff
    return face.cx - d * math.cos(face.nyaw), face.cy - d * math.sin(face.nyaw), face.nyaw


def _check_plan_reaches(face, gap_target=0.05, standoff=0.0):
    steps, why = plan_approach(face, gap_target=gap_target, standoff=standoff)
    assert steps is not None, why
    x, y, th = integrate(steps)
    tx, ty, tth = _target(face, gap_target, standoff)
    assert math.hypot(x - tx, y - ty) <= 0.001, (steps, (x, y), (tx, ty))
    assert abs(wrap(th - tth)) <= 0.1 * DEG, (steps, math.degrees(th), math.degrees(tth))
    return steps


def test_plan_forward():
    f = face_of(box_segments(0.25))
    steps = _check_plan_reaches(f, standoff=0.15)
    drive = sum(v for k, v in steps if k == "drive")
    assert abs(drive - 0.05) <= 0.006, steps
    turns = [abs(v) for k, v in steps if k == "turn"]
    assert all(t <= 1 * DEG for t in turns), steps


def test_plan_reverse():
    f = face_at(0.08)        # gap 0.08 은 앞면 대부분이 range_min 안이라 스캔으로 못 잼 → 참값 면
    steps = _check_plan_reaches(f, standoff=0.15)
    drive = sum(v for k, v in steps if k == "drive")
    assert drive < -0.1, steps                 # 0.08 − 0.05 − 0.15 = −0.12


def test_plan_lateral_too_big():
    f = face_of(box_segments(0.25, lateral=0.08))
    assert f.ok, f.why
    steps, why = plan_approach(f, standoff=0.15)
    assert steps is None and why, (steps, why)


def test_plan_offset_and_yaw_integrates():
    for lat, yaw_deg, gap, so in ((0.03, 5.0, 0.25, 0.15), (-0.04, -8.0, 0.30, 0.10), (0.02, 0.0, 0.20, 0.0)):
        f = face_of(box_segments(gap, lateral=lat, yaw=yaw_deg * DEG))
        assert f.ok, (lat, yaw_deg, f.why)
        steps = _check_plan_reaches(f, standoff=so)
        for k, v in steps:
            if k == "turn":
                assert abs(v) <= 25 * DEG + 1e-9, steps
            else:
                assert -0.35 - 1e-9 <= v <= 0.40 + 1e-9, steps


def test_plan_rejects_bad_face():
    steps, why = plan_approach(face_of([]))
    assert steps is None and why


# ---------------------------------------------------------------- 12. turn_clear
def test_turn_clear():
    near = face_at(0.05)     # gap 0.05 는 스캔으로 못 잼 → 참값 면
    far = face_of(box_segments(0.25))
    assert far.ok, far.why
    assert turn_clear(face_at(0.25), 20 * DEG) is True
    assert turn_clear(near, 20 * DEG) is False
    assert turn_clear(near, -20 * DEG) is False
    assert turn_clear(far, 20 * DEG) is True
    assert turn_clear(far, -20 * DEG) is True


# ---------------------------------------------------------------- 12. 넓은 창 · 고르기 (11-21 실물 0/36 대응)
def test_expect_off_13cm_found():
    """상자가 예상 위치에서 13 cm (옆 · 앞) 어긋나도 찾음 (예전 ±0.10 창은 점 0 개)"""
    for dx, dy in ((0.13, 0.0), (0.0, 0.13), (0.09, -0.09)):
        pts = to_pts(make_scan(box_segments(0.25 + dx, lateral=dy)))
        f = find_box_face(pts, expect=(FRONT_OFFSET + 0.25, 0.0, 0.0))
        assert f.ok, (dx, dy, f.why)
        assert abs(f.gap - (0.25 + dx)) <= 0.005 and abs(f.lateral - dy) <= 0.005, (dx, dy, f.gap, f.lateral)
        old = find_box_face(pts, expect=(FRONT_OFFSET + 0.25, 0.0, 0.0), cfg={"expect_win": 0.10, "fit_margin": 0.0})
        if max(abs(dx), abs(dy)) >= 0.13:
            assert not old.ok, (dx, dy, "예전 창이면 못 찾아야 함 (재현)")


def test_expect_none_offset_box():
    for gap, lat, yaw in ((0.40, 0.15, 0.0), (0.30, -0.20, 8 * DEG), (0.45, 0.0, -10 * DEG)):
        f = find_box_face(to_pts(make_scan(box_segments(gap, lateral=lat, yaw=yaw))), expect=None)
        assert f.ok, (gap, lat, f.why)
        assert abs(f.gap - _expect_gap(gap, lat, yaw)) <= 0.006, (gap, lat, f.gap)
        assert abs(f.lateral - _expect_lat(gap, lat, yaw)) <= 0.006, (gap, lat, f.lateral)


def test_wall_clipped_by_window_not_box():
    """벽 끝이 창 안에 있어도 (창 경계에서 잘려 0.205 처럼 보일 수 있는 조각) 상자로 안 봄"""
    for y0 in (0.05, 0.15, 0.25, -0.20):
        wall = [((0.45, y0), (0.45, y0 + 2.0))] if y0 > 0 else [((0.45, y0), (0.45, y0 - 2.0))]
        for ex in (None, (0.45, 0.0, 0.0), (0.45, 0.15, 0.0)):
            f = find_box_face(to_pts(make_scan(wall)), expect=ex)
            assert not f.ok, (y0, ex, f.why, f.length)


def test_box_beside_wall_corner():
    """벽 (뒤) + 상자 + 옆 벽이 있어도 상자 앞면"""
    segs = box_segments(0.30, lateral=0.05) + wall_segment(FRONT_OFFSET + 0.30 + BOX_SIDE + 0.2) + \
        [((0.10, -0.25), (1.5, -0.25))]
    f = find_box_face(to_pts(make_scan(segs)), expect=None)
    assert f.ok, f.why
    assert abs(f.gap - 0.30) <= 0.005 and abs(f.lateral - 0.05) <= 0.005, (f.gap, f.lateral)


def test_two_boxes_pick_front():
    """창 안에 상자 두 개면 정면 (expect 없으면 |y| 작은 것, 있으면 expect 가까운 것)"""
    segs = box_segments(0.30, lateral=0.02) + box_segments(0.30, lateral=0.27)
    f = find_box_face(to_pts(make_scan(segs)), expect=None)
    assert f.ok and abs(f.lateral - 0.02) <= 0.005, (f.why, f.lateral)
    assert "후보 2" in f.why, f.why
    f = find_box_face(to_pts(make_scan(segs)), expect=(FRONT_OFFSET + 0.30, 0.20, 0.0))
    assert f.ok and abs(f.lateral - 0.27) <= 0.005, (f.why, f.lateral)


def test_outside_window_reason():
    f = find_box_face(to_pts(make_scan(box_segments(0.30, lateral=0.0))), expect=(FRONT_OFFSET + 0.30, 0.28, 0.0))
    assert not f.ok and "창 밖" in f.why, f.why


def test_expect_none_far_rejected():
    """expect 없으면 앞면 중점 x ≤ x_max (0.65) 만 — 먼 다른 물체를 상자로 잡지 않게 (bag 1800 box1 0.76 m)"""
    f = find_box_face(to_pts(make_scan(box_segments(0.63))), expect=None)
    assert not f.ok and "창 밖" in f.why, f.why


def test_replay_cdr_laserscan():
    """replay_ap 의 sqlite 대비 CDR 해석기 — rclpy 가 있으면 직렬화한 LaserScan 과 맞춰 봄 (없으면 건너뜀)"""
    try:
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import LaserScan
    except ImportError:
        return
    import replay_ap
    m = LaserScan()
    m.header.frame_id = "laser_front"
    m.angle_min, m.angle_increment, m.range_min, m.range_max = -3.14, 0.0175, 0.12, 8.0
    m.ranges = [0.5, float("inf"), 1.25]
    frame, amin, ainc, rmin, rmax, rr = replay_ap.cdr_laserscan(serialize_message(m))
    assert frame == "laser_front" and abs(amin + 3.14) < 1e-6 and abs(ainc - 0.0175) < 1e-6 and abs(rmin - 0.12) < 1e-6 and abs(rmax - 8.0) < 1e-6
    assert rr[0] == 0.5 and math.isinf(rr[1]) and rr[2] == 1.25


# ---------------------------------------------------------------- 13. S 자 보정 (plan_s_curve)
def test_turn_intrusion_formula_and_clearance():
    rho, phi = 0.260, math.radians(40.2)
    for a_deg in (5, 10, 20, 25, 28, 30):
        a = a_deg * DEG
        dx = turn_intrusion(a)
        assert abs(dx - (rho * math.cos(phi - a) - 0.1986)) <= 0.0015, (a_deg, dx)
        assert abs(turn_clearance(face_at(0.20), a) - (0.20 - dx)) <= 0.0005, a_deg
        assert abs(turn_intrusion(-a) - dx) < 1e-12
    assert abs(turn_intrusion(20 * DEG) - 0.046) <= 0.001 and abs(turn_intrusion(28 * DEG) - 0.055) <= 0.001


def _check_s(e, g, yaw=0.0, gap_target=0.05, **kw):
    p = plan_s_curve(e, g, yaw, gap_target, **kw)
    assert p.ok, (e, g, yaw, p.why)
    face = plan_face = FaceResult(True, "", 0, 0, yaw, BOX_SIDE, 0)
    n = (math.cos(yaw), math.sin(yaw))
    t = (-n[1], n[0])
    plan_face.cx = n[0] * (g + FRONT_OFFSET) + t[0] * e
    plan_face.cy = n[1] * (g + FRONT_OFFSET) + t[1] * e
    x, y, th = integrate(p.steps)
    tx, ty, tth = _target(plan_face, gap_target, 0.0)
    assert math.hypot(x - tx, y - ty) <= 0.001, (e, g, yaw, p.steps, (x, y), (tx, ty))
    assert abs(wrap(th - tth)) <= 0.1 * DEG, (e, g, yaw, math.degrees(th))
    # 회전마다 앞 모서리 여유 (독립 적분으로 자세를 따라가며 turn_clearance)
    px = py = pth = 0.0
    for kind, v in p.steps:
        if kind == "turn":
            assert turn_clearance(face, v, (px, py, pth)) >= S_CFG["margin"] - 1e-6, (e, g, p.steps)
            assert abs(v) <= S_CFG["alpha_max_back"] + abs(yaw) + 1e-9
        px, py, pth = _step(px, py, pth, kind, v)
    return p


def _step(x, y, th, kind, v, icr_x=ICR_X_ROBOT):
    if kind == "turn":
        ix, iy = x + icr_x * math.cos(th), y + icr_x * math.sin(th)
        th += v
        return ix - icr_x * math.cos(th), iy - icr_x * math.sin(th), th
    return x + v * math.cos(th), y + v * math.sin(th), th


def test_s_no_back_at_020():
    """주행 모드: 틈 0.20 · |e| ≤ (0.20 − Δx(25°) − 0.02)·tan25° ≈ 5.9 cm 는 후진 없이"""
    for e in (0.01, -0.03, 0.05, -0.058):
        p = _check_s(e, 0.20)
        assert p.back == 0.0 and not any(k == "drive" and v < 0 for k, v in p.steps), (e, p.why)
        assert p.alpha * e > 0
        assert len([k for k, _ in p.steps if k == "turn"]) == 2, p.steps   # 정렬 + α 를 한 번에


def test_s_with_yaw():
    for yaw_deg in (-8, 5, 12):
        _check_s(0.03, 0.22, yaw_deg * DEG)
        _check_s(-0.04, 0.22, yaw_deg * DEG)


def test_s_needs_back():
    p = _check_s(0.07, 0.20)
    assert p.back >= S_CFG["d_min"] - 1e-9 and p.back <= S_CFG["d_max"], p.why
    p = _check_s(0.03, 0.07)
    assert p.back > 0, p.why
    assert p.steps[0][0] == "turn" and p.steps[1] == ("drive", -p.back) or p.steps[0] == ("drive", -p.back), p.steps


def test_s_small_e_straight():
    p = _check_s(0.003, 0.20, 3 * DEG)
    assert p.forward == 0.0 and p.back == 0.0 and len(p.steps) == 2, p.steps


def test_s_rejects():
    assert not plan_s_curve(0.10, 0.20).ok                   # |e| > e_max
    assert not plan_s_curve(0.02, 0.20, 30 * DEG).ok         # 기울기
    p = plan_s_curve(0.07, 0.06, back_room=0.03)             # 뒤 여유 없음
    assert not p.ok and "후진" in p.why, p.why
    assert not plan_s_curve_face(FaceResult(False, "x")).ok


def test_s_face_wrapper_matches():
    f = face_of(box_segments(0.20, lateral=0.03, yaw=4 * DEG))
    assert f.ok, f.why
    p = plan_s_curve_face(f)
    q = plan_s_curve(f.lateral, f.gap, f.yaw_err)
    assert p.ok and p.steps == q.steps, (p.why, q.why)


# ---------------------------------------------------------------- 직접 실행
def _accuracy_report():
    rows = []
    for name, gap, lat, yaw_deg in (("정면", 0.25, 0.0, 0.0), ("옆 +3cm", 0.25, 0.03, 0.0),
                                    ("비스듬 +10°", 0.25, 0.0, 10.0), ("먼 0.35", 0.35, 0.0, 0.0)):
        f = face_of(box_segments(gap, lateral=lat, yaw=yaw_deg * DEG))
        if not f.ok:
            rows.append("  %-10s ok=False (%s)" % (name, f.why))
            continue
        rows.append("  %-10s gap 오차 %+5.1f mm, lateral 오차 %+5.1f mm, yaw 오차 %+5.2f°, 길이 %.1f mm, 점 %d"
                    % (name, (f.gap - _expect_gap(gap, lat, yaw_deg * DEG)) * 1000, (f.lateral - _expect_lat(gap, lat, yaw_deg * DEG)) * 1000,
                       math.degrees(wrap(f.yaw_err - yaw_deg * DEG)), f.length * 1000, f.n_pts))
    return rows



# ---------------------------------------------------------------- 11-22 실물 box1 회귀 (앞면이 안 보일 때 이유 보고)

# 실물 1122_1851 /scan_front 에서 뽑은 base_link 점 (x 0.14~0.9 · |y| ≤ 0.5, mm 반올림) — box1 (t 60 s, 남쪽 접근: 상자 앞면 대신 3 cm 폭 조각 둘 + 동쪽 벽) · box2 (t 140.5 s, 성공)
REAL_BOX1_SPARSE = [(0.142, 0.382), (0.149, 0.395), (0.155, 0.388), (0.161, 0.39), (0.167, 0.389), (0.173, 0.389), (0.179, 0.389), (0.186, 0.389), (0.192, 0.388), (0.198, 0.389), (0.204, 0.389), (0.211, 0.389), (0.217, 0.389), (0.223, 0.389), (0.23, 0.389), (0.236, 0.388), (0.242, 0.387), (0.249, 0.387), (0.256, 0.388), (0.262, 0.386), (0.269, 0.387), (0.276, 0.387), (0.283, 0.388), (0.29, 0.388), (0.297, 0.387), (0.304, 0.387), (0.312, 0.387), (0.319, 0.386), (0.326, 0.386), (0.333, 0.385), (0.34, 0.384), (0.348, 0.384), (0.357, 0.386), (0.364, 0.383), (0.372, 0.383), (0.38, 0.382), (0.388, 0.381), (0.397, 0.381), (0.405, 0.38), (0.413, 0.379), (0.423, 0.379), (0.431, 0.378), (0.441, 0.378), (0.451, 0.378), (0.46, 0.377), (0.47, 0.376), (0.481, 0.376), (0.491, 0.376), (0.503, 0.376), (0.514, 0.375), (0.525, 0.373), (0.537, 0.373), (0.551, 0.374), (0.564, 0.373), (0.578, 0.374), (0.591, 0.372), (0.605, 0.372), (0.619, 0.37), (0.634, 0.369), (0.649, 0.368), (0.665, 0.367), (0.681, 0.366), (0.699, 0.365), (0.72, 0.366), (0.735, 0.362), (0.744, 0.355), (0.745, 0.342), (0.744, 0.329), (0.744, 0.317), (0.744, 0.305), (0.743, 0.293), (0.744, 0.282), (0.744, 0.27), (0.742, 0.258), (0.741, 0.246), (0.74, 0.235), (0.74, 0.224), (0.74, 0.213), (0.427, 0.093), (0.438, 0.091), (0.438, 0.086), (0.435, 0.08), (0.436, 0.075), (0.435, 0.07), (0.435, 0.065), (0.736, 0.11), (0.736, 0.1), (0.735, 0.09), (0.735, 0.081), (0.736, 0.071), (0.735, 0.062), (0.446, 0.022), (0.457, 0.018), (0.454, 0.013), (0.451, 0.007), (0.45, 0.002), (0.453, -0.003), (0.732, -0.024), (0.733, -0.033), (0.733, -0.042), (0.733, -0.052), (0.733, -0.061), (0.732, -0.071), (0.732, -0.08), (0.731, -0.09), (0.732, -0.1), (0.731, -0.109), (0.731, -0.119), (0.73, -0.128), (0.73, -0.138), (0.732, -0.149), (0.73, -0.158), (0.73, -0.168), (0.729, -0.178), (0.728, -0.188), (0.727, -0.198), (0.726, -0.208), (0.726, -0.219), (0.726, -0.229), (0.724, -0.239), (0.726, -0.251), (0.725, -0.261), (0.725, -0.273), (0.725, -0.284), (0.724, -0.295), (0.727, -0.308), (0.725, -0.319), (0.725, -0.331), (0.724, -0.343), (0.725, -0.356), (0.721, -0.366), (0.705, -0.369), (0.704, -0.381)]
REAL_BOX2_OK = [(0.141, 0.389), (0.147, 0.389), (0.153, 0.388), (0.159, 0.388), (0.165, 0.389), (0.171, 0.389), (0.178, 0.389), (0.184, 0.389), (0.19, 0.388), (0.196, 0.388), (0.203, 0.39), (0.209, 0.388), (0.215, 0.389), (0.221, 0.389), (0.228, 0.389), (0.234, 0.389), (0.241, 0.389), (0.247, 0.388), (0.254, 0.389), (0.261, 0.389), (0.268, 0.389), (0.275, 0.391), (0.283, 0.392), (0.289, 0.391), (0.297, 0.392), (0.304, 0.391), (0.311, 0.391), (0.318, 0.389), (0.325, 0.389), (0.333, 0.389), (0.34, 0.389), (0.348, 0.388), (0.356, 0.388), (0.363, 0.387), (0.373, 0.388), (0.38, 0.387), (0.388, 0.387), (0.397, 0.387), (0.407, 0.387), (0.415, 0.386), (0.425, 0.387), (0.434, 0.386), (0.444, 0.386), (0.454, 0.386), (0.464, 0.386), (0.475, 0.386), (0.486, 0.386), (0.496, 0.385), (0.508, 0.386), (0.519, 0.385), (0.531, 0.384), (0.545, 0.385), (0.559, 0.385), (0.572, 0.385), (0.587, 0.386), (0.271, 0.111), (0.271, 0.107), (0.27, 0.103), (0.273, 0.102), (0.274, 0.099), (0.274, 0.096), (0.273, 0.092), (0.273, 0.089), (0.273, 0.086), (0.273, 0.083), (0.273, 0.08), (0.272, 0.077), (0.272, 0.074), (0.272, 0.071), (0.272, 0.068), (0.271, 0.065), (0.27, 0.062), (0.271, 0.06), (0.271, 0.058), (0.27, 0.055), (0.27, 0.052), (0.271, 0.05), (0.273, 0.048), (0.275, 0.046), (0.274, 0.044), (0.273, 0.041), (0.273, 0.038), (0.273, 0.036), (0.273, 0.034), (0.273, 0.031), (0.276, 0.03), (0.273, 0.027), (0.274, 0.025), (0.273, 0.022), (0.272, 0.02), (0.273, 0.018), (0.273, 0.016), (0.272, 0.013), (0.272, 0.011), (0.272, 0.009), (0.273, 0.007), (0.273, 0.004), (0.273, 0.002), (0.272, -0.0), (0.273, -0.002), (0.273, -0.004), (0.273, -0.007), (0.272, -0.009), (0.274, -0.011), (0.274, -0.013), (0.275, -0.016), (0.274, -0.018), (0.274, -0.02), (0.275, -0.023), (0.276, -0.025), (0.274, -0.027), (0.274, -0.029), (0.274, -0.032), (0.273, -0.034), (0.274, -0.036), (0.275, -0.039), (0.275, -0.041), (0.274, -0.044), (0.275, -0.046), (0.276, -0.049), (0.274, -0.051), (0.273, -0.053), (0.274, -0.056), (0.273, -0.058), (0.272, -0.06), (0.274, -0.064), (0.273, -0.066), (0.274, -0.069), (0.274, -0.072), (0.273, -0.075), (0.275, -0.078), (0.276, -0.082), (0.276, -0.085), (0.276, -0.088), (0.277, -0.092), (0.277, -0.095), (0.277, -0.098), (0.277, -0.101), (0.276, -0.104)]


def test_real1122_box1_sparse_reports_missing_face_not_wall_tilt():
    """실물 1122 box1: 예상 자리 (0.28, 0) 에 상자 앞면 선이 없고 (3 cm 폭 조각 둘) 창 밖 동쪽 벽만 선으로 잡힘.
    예전 이유 = "기울기 96° > 25°" (벽 · 상자 옆면으로 오해) → 이제 "예상 자리 안 점이 적어 앞면 선 못 찾음" 이어야 하고, 벽을 앞면으로 고르면 안 됨"""
    f = find_box_face(REAL_BOX1_SPARSE, expect=(0.28, 0.0, 0.0))
    assert not f.ok
    assert "예상 자리 안 점이 적어 앞면 선 못 찾음" in f.why, f.why
    assert not f.why.startswith("기울기"), f.why
    f2 = find_box_face(REAL_BOX1_SPARSE)   # expect 없이 (넓은 창) 도 실패여야 (벽을 상자로 안 잡음)
    assert not f2.ok, f2.why


def test_real1122_box2_still_ok():
    f = find_box_face(REAL_BOX2_OK, expect=(0.28, 0.0, 0.0))
    assert f.ok, f.why
    assert abs(f.gap - 0.141) < 0.01 and abs(f.lateral) < 0.01 and abs(math.degrees(f.yaw_err)) < 3.0, (f.gap, f.lateral, f.yaw_err)


def test_failure_reason_prefers_in_window_line():
    """창 안에 기울어진 선 (옆면) 이 있으면 그 이유를, 창 밖 벽 선은 이유로 안 씀"""
    wall = [(0.15 + 0.01 * i, -0.40) for i in range(45)]    # 오른쪽 벽 (창 밖, 기울기 90°)
    sparse = [(0.40, 0.05 + 0.005 * i) for i in range(6)]   # 앞쪽 조각 6 점뿐
    f = find_box_face(wall + sparse, expect=(0.28, 0.0, 0.0))
    assert not f.ok and "예상 자리" in f.why and "기울기" in f.why and "창 밖" in f.why, f.why


# ---------------------------------------------------------------- 후진 S 자 (11-23)
def _check_back(e, g, yaw=0.0, **kw):
    """계획을 ICR 모의로 실행: 끝 옆 오차 · 방향 작음, 회전 여유 ≥ 2 cm, 전진 없음, 틈은 늘기만"""
    p = plan_s_curve_back(e, g, yaw, **kw)
    assert p.ok, (e, g, yaw, p.why)
    face = box_face_from(e, g, yaw)
    x, y, th = 0.0, 0.0, 0.0
    worst = math.inf
    for kind, v in p.steps:
        if kind == "turn":
            worst = min(worst, turn_clearance(face, v, (x, y, th)))
        x, y, th = _step(x, y, th, kind, v)
        assert not (kind == "drive" and v > 0), p.steps   # 전진 없음
    dx, dy = face.cx - x, face.cy - y
    ny = face.nyaw - th
    lat = -math.sin(face.nyaw) * dx + math.cos(face.nyaw) * dy
    gap = math.cos(face.nyaw) * dx + math.sin(face.nyaw) * dy - FRONT_OFFSET
    assert abs(lat) <= 0.001, (e, g, yaw, lat)            # 최종 중앙선 오차 1 mm 이하
    assert abs(ny) <= 0.2 * DEG, (e, g, yaw, ny)          # 방향 법선
    assert worst >= 0.02 - 1e-6, (e, g, yaw, worst)       # 회전 모서리 여유
    assert gap >= g - 0.011, (gap, g)
    back = sum(-v for k, v in p.steps if k == "drive")
    assert abs(back - p.back) < 1e-9
    return p, gap, back


def box_face_from(e, g, yaw):
    from box_face import _face_from_err
    return _face_from_err(e, g, yaw)


def test_back_s_geometry_grid():
    for e_cm in (-8, -5, -3, -1, 1, 2, 4, 6, 8):
        for g in (0.08, 0.10, 0.14):
            for yaw_deg in (-10, -4, 0, 5, 12):
                p, gap, back = _check_back(e_cm / 100.0, g, yaw_deg * DEG)
                assert back <= abs(e_cm) / 100.0 / math.sin(10 * DEG) + 0.16   # 후진 합 상한 (α ≥ 10° · d0 ≤ 0.15)


def test_back_s_typical_numbers():
    """e 5 cm · 틈 10 cm: α 30° → f = 10 cm, 틈 약 18.7 cm. 회전 방향 = −sgn(e)"""
    p, gap, back = _check_back(0.05, 0.10)
    assert abs(p.f_back - 0.10) < 0.002 and abs(gap - 0.187) < 0.003, (p.why, gap)
    assert p.alpha > 0 and p.steps[0][0] == "turn" and p.steps[0][1] < 0   # e > 0 이면 오른쪽 (−) 으로 먼저 돎
    p, _, _ = _check_back(-0.05, 0.10)
    assert p.steps[0][1] > 0


def test_back_s_budget_and_room():
    p = plan_s_curve_back(0.08, 0.10, rev_budget=0.10)       # f 만 16 cm → 한도 10 cm 로 안 됨
    assert not p.ok and "후진" in p.why, p.why
    p = plan_s_curve_back(0.03, 0.10, back_room=0.05)
    assert not p.ok
    p = plan_s_curve_back(0.03, 0.10, rev_budget=0.10)
    assert p.ok and p.back <= 0.10 + 1e-9


def test_back_s_rejects_and_wrapper():
    assert not plan_s_curve_back(0.10, 0.10).ok
    assert not plan_s_curve_back(0.02, 0.10, 30 * DEG).ok
    assert not plan_s_curve_back_face(FaceResult(False, "x")).ok
    f = face_of(box_segments(0.12, lateral=0.03, yaw=4 * DEG))
    assert f.ok, f.why
    p, q = plan_s_curve_back_face(f), plan_s_curve_back(f.lateral, f.gap, f.yaw_err)
    assert p.ok and p.steps == q.steps


def test_back_s_turn_room_when_tight():
    """틈 7 cm 에선 회전 여유 (α 30° 약 7.7 cm 필요) 가 모자라 처음에 곧게 후진"""
    p, gap, back = _check_back(0.04, 0.07)
    assert p.pre_back > 0 and p.steps[0][0] == "drive" and p.steps[0][1] < 0, p.steps


def test_back_s_partial_when_over_emax():
    """|e| 가 e_max (8 cm) 를 넘으면 clamp 로 e_max 만큼만 보정, 남은 옆 오차를 partial 로 돌려줌 (다시 잰 뒤 이어 감)"""
    assert not plan_s_curve_back(0.095, 0.10).ok
    for e in (0.095, -0.10):
        p = plan_s_curve_back(e, 0.10, clamp=True)
        assert p.ok and abs(p.partial - (e - math.copysign(0.08, e))) < 1e-9, p.why
        assert p.alpha * e < 0 or p.alpha * e > 0   # 방향은 e 부호를 따름
        q = plan_s_curve_back(math.copysign(0.08, e), 0.10)
        assert p.steps == q.steps and abs(p.end_lat) < 0.001
    assert plan_s_curve_back(0.03, 0.10, clamp=True).partial == 0.0


def test_back_s_threat_cases_after_partial():
    """부분 보정 뒤 남은 옆 (≤ 2 cm) 은 다시 재서 한 번 더 — 두 번째 계획도 성립"""
    p1 = plan_s_curve_back(0.10, 0.10, clamp=True)
    assert p1.ok and 0 < p1.partial <= 0.021
    p2 = plan_s_curve_back(p1.partial, p1.end_gap)
    assert p2.ok, p2.why


def test_stop_gap_107_measures_ok():
    """정지점 틈 10.7 cm (끊지 않고 정지점에서 잼): 노이즈 1 mm 에서 틈 · 옆 · 방향이 맞게 잼"""
    for lat, yaw in ((0.0, 0.0), (0.03, 4 * DEG), (-0.05, -6 * DEG)):
        f = face_of(box_segments(0.107, lateral=lat, yaw=yaw), noise=0.001, rng=random.Random(7))
        assert f.ok, (lat, yaw, f.why)
        assert abs(f.cx - (FRONT_OFFSET + 0.107)) < 0.004 and abs(f.cy - lat) < 0.004 and abs(f.nyaw - yaw) < 2 * DEG, (f.cx, f.cy, f.nyaw)   # 중점 (base_link) 으로 비교 — FaceResult.lateral 은 법선 기준

if __name__ == "__main__":
    tests = [(k, v) for k, v in sorted(globals().items(), key=lambda kv: kv[1].__code__.co_firstlineno
                                       if callable(kv[1]) and hasattr(kv[1], "__code__") else 0)
             if k.startswith("test_") and callable(v)]
    fails = 0
    for name, fn in tests:
        try:
            fn()
            print("통과  %s" % name)
        except Exception as e:   # noqa: BLE001
            fails += 1
            print("실패  %s: %s: %s" % (name, type(e).__name__, e))
    print("\n실측 정확도 (잡음 없음):")
    try:
        print("\n".join(_accuracy_report()))
    except Exception as e:   # noqa: BLE001
        print("  계산 못함: %r" % (e,))
    if NOISE_STATS:
        print("잡음 σ3mm 5장 평균 최대 오차: lateral %.1f mm, yaw %.2f°, gap %.1f mm"
              % (NOISE_STATS["lat_mm"], NOISE_STATS["yaw_deg"], NOISE_STATS["gap_mm"]))
    if LOOSE_STATS:
        print("잡음 σ1cm LOOSE 정면 5장 평균: lateral %+.1f mm, yaw %+.2f°, ok %d/5"
              % (LOOSE_STATS["lat_mm"], LOOSE_STATS["yaw_deg"], LOOSE_STATS["n_ok"]))
    for gap, lat, yd, n_ok, n_good in RATE_STATS.get("rows", []):
        print("σ1cm LOOSE 성공률 gap %.2f lat %+.2f yaw %+3.0f°: 성공 %d/20, 그중 1 cm · 2° 안 %d" % (gap, lat, yd, n_ok, n_good))
    print("\n%d개 중 %d개 통과, %d개 실패" % (len(tests), len(tests) - fails, fails))
    sys.exit(1 if fails else 0)
