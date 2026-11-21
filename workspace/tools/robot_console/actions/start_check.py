#!/usr/bin/env python3
"""출발 자리 확인 (검토 C 승인, 2026-11-20): 모드 1 · 4 의 할 일 시작 직전.

1. /localization/status 가 OK 가 될 때까지 기다림 (최대 60 s). CONFIRM 전에는 판정하지 않음.
2. 그때 AMCL 자세 (/amcl_pose) 로
   - footprint (+ padding 0.01, nav2_params.yaml 과 같은 다각형) 가 map_nav 점유칸과 겹치거나 가장 가까운 점유칸까지 여유 < 3 cm → 시작 안 함
     (3 cm = 칸 반 0.0125 + AMCL 정지 오차 여분, padding 은 다각형에 이미 들어 있어 한 번만 셈)
   - 설계 출발 자리 (approach_poses.yaml HOME) 와 10 cm 또는 20° 넘게 다르면 경고만
11-20 실물: 위치 감시 OK (잔차 0.0003) 인 (0.81, 1.00, 142°) 에서 뒤 모서리가 벽 칸과 겹쳐 MPPI 모든 궤적 충돌 → 이 판정이 막음.
"""
import math
import time
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

REPO = Path(__file__).resolve().parents[4]
MP = REPO / 'workspace/ros2_ws/src/mission_planner'
NAV2 = REPO / 'workspace/ros2_ws/src/navigation/config/nav2_params.yaml'
CLEAR_MIN, WARN_XY, WARN_YAW_DEG, OK_WAIT_S = 0.03, 0.10, 20.0, 60.0


def footprint():
    d = yaml.safe_load(open(NAV2, encoding='utf-8'))
    lc = d['local_costmap']['local_costmap']['ros__parameters']
    pts = yaml.safe_load(lc['footprint'])
    pad = float(lc.get('footprint_padding', 0.0))
    return [(x + math.copysign(pad, x), y + math.copysign(pad, y)) for x, y in pts]


def load_map():
    m = yaml.safe_load(open(MP / 'maps/map_nav.yaml', encoding='utf-8'))
    img = np.asarray(Image.open(MP / 'maps' / m['image']).convert('L'), dtype=float) / 255.0
    occ = (1.0 - img) > m['occupied_thresh']
    return occ[::-1, :], m['resolution'], m['origin'][:2]


def seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def inside(p, poly):
    c = False
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[i - 1]
        if (y1 > p[1]) != (y2 > p[1]) and p[0] < (x2 - x1) * (p[1] - y1) / (y2 - y1) + x1:
            c = not c
    return c


def check_pose(x, y, yaw):
    """돌려줌: (막힘 여부, 여유 m, 겹친 칸 수, 설명)"""
    occ, res, org = load_map()
    c, s = math.cos(yaw), math.sin(yaw)
    poly = [(x + px * c - py * s, y + px * s + py * c) for px, py in footprint()]
    r = 0.7
    i0, i1 = int((x - r - org[0]) / res), int((x + r - org[0]) / res)
    j0, j1 = int((y - r - org[1]) / res), int((y + r - org[1]) / res)
    clear, hits = 9.9, 0
    for j in range(max(j0, 0), min(j1, occ.shape[0])):
        for i in range(max(i0, 0), min(i1, occ.shape[1])):
            if not occ[j, i]:
                continue
            p = (org[0] + (i + 0.5) * res, org[1] + (j + 0.5) * res)
            if inside(p, poly):
                hits += 1
                clear = 0.0
                continue
            d = min(seg_dist(p, poly[k], poly[k - 1]) for k in range(len(poly))) - res / 2
            clear = min(clear, max(d, 0.0))
    blocked = hits > 0 or clear < CLEAR_MIN
    why = f'차체가 지도 점유칸 {hits} 칸과 겹침' if hits else f'가장 가까운 점유칸까지 {clear * 100:.1f} cm'
    return blocked, clear, hits, why


def home():
    d = yaml.safe_load(open(MP / 'config/approach_poses.yaml', encoding='utf-8'))
    return d.get('HOME', {}).get('stage', [0.69365, 0.95, math.pi])


def wait_and_check(node, say):
    """할 일 시작 전에 부름. 돌려줌: True = 시작해도 됨"""
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    import rclpy
    st, pose = {'v': ''}, {'v': None}
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    s1 = node.create_subscription(String, '/localization/status', lambda m: st.__setitem__('v', m.data), latched)
    s2 = node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: pose.__setitem__('v', m), latched)
    end = time.time() + OK_WAIT_S
    last = ''
    while time.time() < end and not (st['v'] == 'OK' and pose['v'] is not None):
        rclpy.spin_once(node, timeout_sec=0.1)
        if st['v'] != last:
            last = st['v']
            say(f'출발 자리 확인: 위치 감시 {last or "(아직 없음)"} — OK 를 기다림')
    node.destroy_subscription(s1)
    node.destroy_subscription(s2)
    if not (st['v'] == 'OK' and pose['v'] is not None):
        say(f'✗ 출발 자리 확인: {OK_WAIT_S:.0f} s 안에 위치 감시 OK 가 안 됨 ({st["v"] or "상태 없음"}) — 시작 안 함')
        return False
    p = pose['v'].pose.pose
    q = p.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    blocked, clear, hits, why = check_pose(p.position.x, p.position.y, yaw)
    hx, hy, hyaw = home()
    dxy = math.hypot(p.position.x - hx, p.position.y - hy)
    dyaw = abs(math.degrees(math.atan2(math.sin(yaw - hyaw), math.cos(yaw - hyaw))))
    say(f'출발 자리 확인: AMCL ({p.position.x:.3f}, {p.position.y:.3f}, {math.degrees(yaw):.0f}°) · 설계 출발 자리와 '
        f'{dxy * 100:.1f} cm · {dyaw:.0f}° · {why}')
    if dxy > WARN_XY or dyaw > WARN_YAW_DEG:
        say(f'  경고: 설계 출발 자리 ({hx:.3f}, {hy:.3f}, {math.degrees(hyaw):.0f}°) 와 {WARN_XY * 100:.0f} cm 또는 {WARN_YAW_DEG:.0f}° 넘게 다름')
    if blocked:
        say(f'✗ 시작 안 함: {why} (기준 여유 {CLEAR_MIN * 100:.0f} cm) — 로봇을 출발 자리로 옮긴 뒤 모드를 다시 켜세요')
        return False
    return True


if __name__ == '__main__':
    # PC 확인: 11-20 실물 자리 (막혀야 함) 와 설계 출발 자리 (통과해야 함)
    for name, (x, y, yaw) in {'11-20 실물 (0.814, 0.999, 141.7°)': (0.814, 0.999, 2.473), '설계 출발 자리': tuple(home())}.items():
        b, c, h, why = check_pose(x, y, yaw)
        print(f'{name}: {"막힘" if b else "통과"} — {why}')
