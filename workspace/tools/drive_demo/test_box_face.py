"""box_face.py 시험 (ROS 없이): cd workspace/tools/drive_demo && python3 test_box_face.py
(python3 -m pytest -q test_box_face.py 로도 돕니다)

합성 스캔: laser_front (base_link 에서 (0.1318, 0), rpy(π,0,π)) 에서 광선을 쏴 LaserScan 처럼 ranges 를 만듭니다.
  angle_min = −π, 1° 간격, 360칸, range_min 0.12, range_max 8
  광선 θ 의 base_link 방향 = (−cosθ, +sinθ)
"""
import math
import random
import sys

from box_face import (BOX_SIDE, FRONT_OFFSET, LASER_FRONT_TF, S_CFG, FaceResult, average_faces, find_box_face,
                      plan_approach, plan_s_curve, plan_s_curve_face, scan_to_base, turn_clear, turn_clearance,
                      turn_intrusion)

ANGLE_MIN = -math.pi
ANGLE_INC = math.radians(1.0)
N_RAYS = 360
RANGE_MIN = 0.12
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
