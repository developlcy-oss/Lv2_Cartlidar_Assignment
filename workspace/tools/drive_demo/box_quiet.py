"""11-20 상자 앞 정지 띠 끄기 (box quiet) — 순수 함수 (ROS 없이 import, test_box_quiet.py 가 시험).

배경: 앞 LDS-03 를 1.5 cm 낮춰 (laser_front z 0.0525, 스캔면 바닥 위 59.75 mm) 높이 70 mm 상자가 라이다에 보임.
  시연 정지점은 대부분 상자 바로 앞 (앞 끝 ↔ 상자 −8.8 ~ +2.5 cm) 이라 다가가는 동안 collision_monitor 정지 띠와
  demo_follow HOLD 가 상자를 장애물로 보고 멈춤 → 우회로 빠짐.
사용자 결정 (11-20): 상자 정지점 0.40 m 안에서는 정지 띠 (collision_monitor) 와 HOLD 를 끔.
11-21 사용자 결정으로 바꿈: 끄지 않고 그 상자 점만 지운 라이다 (box_scan_guard.py) 로 정지 띠 · HOLD 를 그대로 둠.
  점 지우기 구간 = 0.40 m 안 (끝 0.42 m 밖, 예전 끄기와 같은 거리 — box_face 다가가기가 정지점 0.30 ~ 0.35 m 앞에서 재고 옆으로도 움직임),
  속도 ≤ 0.15 m/s 는 0.30 m 안만 (BOX_CAP_M). 파일 이름은 그대로 둠.
판정: 구간 목적지 이름이 상자 (box1 ~ box6) 또는 팔레트 상자 자리 (L1 ~ L3) 이면 "상자 구간" — 계획 json 에는 상자 표시가 따로
  없지만 구간 이름 ("S0 signal→box1") · 들르기 구간의 "to" 가 목적지를 담고, 정지점 = stop_pose (11-21 밤부터 발제 상자 좌표 기준, 이전 = 실측 course_v1_L2_실측.yaml).
  A1 (출발 구역 복귀) · signal 은 상자 아님. 거리 = map 자세 (AMCL) 의 base_link ↔ 구간 정지점 (goal x, y) 직선 거리.
  정지점에서 차체 앞 끝 ≈ 상자 앞면이므로 이 거리 ≈ 차체 앞 끝 ↔ 상자 거리.
"""
from __future__ import annotations

import math

BOX_QUIET_M = 0.40      # 상자 정지점 이 거리 [m] 안이면 상자 구간 (11-20 끔 → 11-21 상자 점만 지움) (0 = 기능 끔, --box-quiet-m)
BOX_QUIET_HYST = 0.02   # 구간 끝 거리 = BOX_QUIET_M + 이 값 (경계에서 껐다 켰다 깜빡이지 않게)
BOX_STOPS = frozenset(["box1", "box2", "box3", "box4", "box5", "box6", "L1", "L2", "L3"])
BOX_CAP_M = 0.30        # 11-21 상자 정지점 이 거리 [m] 안이면 속도 상한 (demo_follow ZONE_V → v2_filter /drive_demo/speed_cap)
CAP_LEAD = 0.05         # 상한을 이만큼 [m] 일찍 켬: demo_follow 타이머 0.1 s × max 전진 0.45 m/s ≈ 4.5 cm (pa8-76 시뮬: 경계 안 첫 표본 0.20~0.39 m/s)
# 11-21 상자 점 지우기 (box_scan_guard.py) — 상자 좌표 = 발제 (tools/drive_avoid/real_map/objects_from_slam.py), 205 mm 정사각형 · 변이 지도 축과 나란함
BOX_HALF = 0.1025       # 상자 반 변 [m]
# 11-22 줄자 실측 상자 중심 (발제와 최대 13 cm 다름: box1 y +13, box4 x +10.6, box6 (+9, -8.6), box3 y +9.6). 발제 값 = PUB_BOXES
BOXES = {"box1": (1.45, 1.08), "box2": (3.65, 3.17), "box3": (2.93, 5.45), "box4": (0.84, 5.38), "box5": (2.25, 5.38), "box6": (3.76, 3.8), "L1": (3.72, 1.01), "L2": (2.21, 3.72), "L3": (3.7, 5.35)}
PUB_BOXES = {"box1": (1.468, 0.950), "box2": (3.670, 3.152), "box3": (2.936, 5.354), "box4": (0.734, 5.354),
             "box5": (2.202, 5.354), "box6": (3.670, 3.886), "L1": (3.670, 0.950), "L2": (2.202, 3.700), "L3": (3.670, 5.354)}
# 11-21 (밤) 정지점도 발제 상자 좌표로 통일 (실물 1800 ~ 1824: 어제 실측 정지점이 상자와 13 cm 어긋나 5 cm 다가가기 0/36).
#   정지점 = 상자 면 중앙 정면, 상판 앞 끝 (base_link x FRONT) ↔ 상자 면 STOP_GAP (기존 course "상자 면 10.7 cm 앞" 규칙).
#   방향 = FACE_YAW (로봇이 보는 방향, 상자 바깥 법선의 반대): 손그림 — box1 남쪽에서 (90°), L2 동쪽 면 (180°), 나머지 그대로.
FRONT = 0.1328          # 상판 앞 끝 base_link x [m]
STOP_GAP = 0.107        # 정지점에서 앞 끝 ↔ 상자 면 [m]
FACE_YAW = {"box1": 90.0, "box2": 90.0, "box3": 90.0, "box4": 90.0, "box5": 0.0, "box6": 0.0,
            "L1": 0.0, "L2": 180.0, "L3": 90.0}
GUARD_MARGIN = 0.05     # 상자 점 지우기 여유 [m] (AMCL 오차) — 반 변 + 이 값 안의 점만 지움
BODY_BOX = (-0.2285, 0.2285, -0.1776, 0.1776)   # 앞 라이다 원본에서 지울 차체 base_link [x0, x1, y0, y1] (scan_merger self_box 와 같음)


def leg_target(g):
    """구간 목적지 이름: 들르기 구간은 "to", 계획 구간은 이름 "S0 signal→box1" 의 → 뒤. 없으면 "" """
    t = g.get("to") or (g.get("name") or "").split("→")[-1]
    t = (t or "").strip()
    if t.upper() in ("L1", "L2", "L3", "HOME"):
        return t.upper()
    return t.lower() if t.lower().startswith("box") else t


def box_goal(g):
    """상자 구간이면 정지점 (x, y), 아니면 None"""
    if leg_target(g) not in BOX_STOPS or not g.get("goal"):
        return None
    return float(g["goal"][0]), float(g["goal"][1])


def quiet_next(pose, goal, quiet, radius=BOX_QUIET_M, hyst=BOX_QUIET_HYST):
    """다음 끔 상태 (True = 정지 띠 · HOLD 끔). pose (x, y[, yaw]) · goal (x, y) 가 None 이거나 radius ≤ 0 이면 False (켬, 안전 쪽).
    지금 꺼져 있으면 radius + hyst 넘게 멀어질 때 켬, 켜져 있으면 radius 안으로 들어올 때 끔"""
    if radius <= 0 or pose is None or goal is None:
        return False
    d = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
    return d <= radius + hyst if quiet else d <= radius


def cap_on(pose, goal, radius=BOX_CAP_M, lead=CAP_LEAD):
    """11-21 속도 상한을 걸까 (상자 정지점 radius + lead 안 — 0.1 s 타이머 늦음만큼 일찍). pose · goal 이 None 이거나 radius ≤ 0 이면 False"""
    if radius <= 0 or pose is None or goal is None:
        return False
    return math.hypot(pose[0] - goal[0], pose[1] - goal[1]) <= radius + lead


def box_center(name):
    """상자 가운데 (x, y) map, 모르는 이름은 None"""
    return BOXES.get(name)


def stop_pose(name, gap=STOP_GAP, yaw_deg=None):
    """정지점 [x, y, yaw rad] (map): 상자 면 중앙 정면, 앞 끝 ↔ 면 gap. 모르는 이름은 None"""
    c = BOXES.get(name)
    if c is None:
        return None
    th = math.radians(FACE_YAW[name] if yaw_deg is None else yaw_deg)
    d = BOX_HALF + gap + FRONT
    return [round(c[0] - d * math.cos(th), 4), round(c[1] - d * math.sin(th), 4), th]


def face_center(name, yaw):
    """yaw (rad) 방향으로 볼 때 마주 보는 상자 면 중앙 (x, y) map. 모르는 이름은 None"""
    c = BOXES.get(name)
    if c is None:
        return None
    return c[0] - BOX_HALF * math.cos(yaw), c[1] - BOX_HALF * math.sin(yaw)


def in_box(pt, center, half):
    return abs(pt[0] - center[0]) <= half and abs(pt[1] - center[1]) <= half


def guard_mask(pts_map, center, half=BOX_HALF, margin=GUARD_MARGIN):
    """지울 점 표시 (True = 상자 점): map 점 목록 (None = 무효 광선) · 상자 가운데. center None = 지우지 않음"""
    if center is None:
        return [False] * len(pts_map)
    h = half + margin
    return [p is not None and in_box(p, center, h) for p in pts_map]


def in_body(q, box=BODY_BOX):
    return box[0] <= q[0] <= box[1] and box[2] <= q[1] <= box[3]


def quat_rot(q, v):
    """쿼터니언 (x, y, z, w) 로 3D 벡터 회전 (앞 라이다가 거꾸로 달려 roll π 라 yaw 만으로는 안 됨)"""
    x, y, z, w = q
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx))


def scan_points(ranges, angle_min, angle_inc, rmin, rmax, tf):
    """스캔 → 목표 프레임 2D 점 목록 (무효 광선은 None). tf = ((tx, ty, tz), (qx, qy, qz, qw)) = 목표 ← 라이다"""
    (tx, ty, _), q = tf
    out = []
    for i, r in enumerate(ranges):
        if math.isinf(r) or math.isnan(r) or not (rmin <= r <= rmax):
            out.append(None)
            continue
        a = angle_min + i * angle_inc
        p = quat_rot(q, (r * math.cos(a), r * math.sin(a), 0.0))
        out.append((p[0] + tx, p[1] + ty))
    return out
