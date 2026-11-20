"""11-20 상자 앞 정지 띠 끄기 (box quiet) — 순수 함수 (ROS 없이 import, test_box_quiet.py 가 시험).

배경: 앞 LDS-03 를 1.5 cm 낮춰 (laser_front z 0.0525, 스캔면 바닥 위 59.75 mm) 높이 70 mm 상자가 라이다에 보임.
  시연 정지점은 대부분 상자 바로 앞 (앞 끝 ↔ 상자 −8.8 ~ +2.5 cm) 이라 다가가는 동안 collision_monitor 정지 띠와
  demo_follow HOLD 가 상자를 장애물로 보고 멈춤 → 우회로 빠짐.
사용자 결정 (11-20): 상자 정지점 0.40 m 안에서는 정지 띠 (collision_monitor) 와 HOLD 를 끔.
판정: 구간 목적지 이름이 상자 (box1 ~ box6) 또는 팔레트 상자 자리 (L1 ~ L3) 이면 "상자 구간" — 계획 json 에는 상자 표시가 따로
  없지만 구간 이름 ("S0 signal→box1") · 들르기 구간의 "to" 가 목적지를 담고, 정지점 = 그 상자 실측 정지 자세 (course_v1_L2_실측.yaml).
  A1 (출발 구역 복귀) · signal 은 상자 아님. 거리 = map 자세 (AMCL) 의 base_link ↔ 구간 정지점 (goal x, y) 직선 거리.
  정지점에서 차체 앞 끝 ≈ 상자 앞면이므로 이 거리 ≈ 차체 앞 끝 ↔ 상자 거리.
"""
from __future__ import annotations

import math

BOX_QUIET_M = 0.40      # 상자 정지점 이 거리 [m] 안이면 정지 띠 · HOLD 끔 (0 = 기능 끔, --box-quiet-m)
BOX_QUIET_HYST = 0.02   # 다시 켜는 거리 = BOX_QUIET_M + 이 값 (경계에서 껐다 켰다 깜빡이지 않게)
BOX_STOPS = frozenset(["box1", "box2", "box3", "box4", "box5", "box6", "L1", "L2", "L3"])


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
