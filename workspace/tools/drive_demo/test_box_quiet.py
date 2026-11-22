"""box_quiet.py 시험 (ROS 없이): cd workspace/tools/drive_demo && python3 -m pytest -q test_box_quiet.py"""
import json
import os

import pytest

import math

import box_quiet as bq
from box_quiet import BOX_QUIET_HYST, BOX_QUIET_M, box_goal, leg_target, quiet_next

HERE = os.path.dirname(os.path.abspath(__file__))


def test_leg_target_from_name_and_to():
    assert leg_target({"name": "S0 signal→box1"}) == "box1"
    assert leg_target({"name": "S7 L3→L2"}) == "L2"
    assert leg_target({"name": "S9 L1→A1"}) == "A1"
    assert leg_target({"name": "S3 box6→L3", "to": "l3"}) == "L3"   # 들르기 구간은 "to" 우선
    assert leg_target({}) == ""


def test_box_goal_only_box_stops():
    assert box_goal({"name": "S0 signal→box1", "goal": [1.8053, 1.0855, 3.14]}) == (1.8053, 1.0855)
    assert box_goal({"name": "S8 L2→L1", "goal": [3.3697, 0.9675, 0.0]}) == (3.3697, 0.9675)
    assert box_goal({"name": "S9 L1→A1", "goal": [0.734, 0.95, 1.57]}) is None   # 출발 구역 = 상자 아님
    assert box_goal({"name": "S5 box5→HOME", "to": "HOME", "goal": [0.7, 0.95, 0.0]}) is None
    assert box_goal({"name": "S0 signal→box1"}) is None   # 정지점 없음


@pytest.mark.parametrize("plan", ["demo_plan_max.json", "demo_plan_wide.json", "demo_plan_auto.json"])
def test_plans_box_legs(plan):
    """시연 계획: S0 ~ S8 은 상자 구간, S9 (→A1) 만 아님"""
    legs = json.load(open(os.path.join(HERE, plan), encoding="utf-8"))["legs"]
    box = [g["leg"] for g in legs if box_goal(g) is not None]
    assert box == list(range(9))


def test_quiet_enter_exit_with_hysteresis():
    g = (1.0, 1.0)
    assert quiet_next((1.0 + BOX_QUIET_M + 0.001, 1.0), g, False) is False   # 0.40 밖 = 켬
    assert quiet_next((1.0 + BOX_QUIET_M, 1.0), g, False) is True            # 0.40 안 = 끔
    assert quiet_next((1.0, 1.0 + BOX_QUIET_M + 0.01), g, True) is True      # 끈 채 0.41: 아직 끔 (깜빡임 막음)
    assert quiet_next((1.0, 1.0 + BOX_QUIET_M + BOX_QUIET_HYST + 0.001), g, True) is False   # 0.42 밖 = 다시 켬
    assert quiet_next((1.0, 1.0 + BOX_QUIET_M + 0.01), g, False) is False    # 켠 채 0.41: 그대로 켬


def test_quiet_safe_defaults():
    assert quiet_next(None, (0.0, 0.0), True) is False        # 자세 모름 = 켬
    assert quiet_next((0.0, 0.0, 0.0), None, True) is False   # 상자 구간 아님 / 구간 끝 = 켬
    assert quiet_next((0.0, 0.0), (0.0, 0.0), False, radius=0.0) is False   # --box-quiet-m 0 = 기능 끔
    assert quiet_next((0.0, 0.0), (0.0, 0.0), True, radius=0.0) is False


def test_cap_only_inside_030():
    g = (1.0, 1.0)
    assert bq.BOX_CAP_M == pytest.approx(0.30) and bq.BOX_CAP_M < BOX_QUIET_M   # 점 지우기 (0.40) 안쪽에서만 느리게
    assert bq.cap_on((1.0 + 0.299, 1.0), g) is True
    assert bq.cap_on((1.0 + 0.301, 1.0), g) is True                       # 0.30 ~ 0.35 = 미리 켬
    assert bq.cap_on((1.0 + bq.BOX_CAP_M + bq.CAP_LEAD + 0.001, 1.0), g) is False
    assert bq.BOX_CAP_M + bq.CAP_LEAD < BOX_QUIET_M                         # 상자 구간 (0.40) 안에서만
    assert bq.cap_on(None, g) is False and bq.cap_on((1.0, 1.0), None) is False and bq.cap_on((1.0, 1.0), g, 0.0) is False


def test_every_box_stop_has_box_coords():
    """계획의 상자 정지점마다 상자 좌표가 있어야 점을 지움 (없으면 상자에 멈춤)"""
    for plan in ("demo_plan.json", "demo_plan_max.json"):
        legs = json.load(open(os.path.join(HERE, plan), encoding="utf-8"))["legs"]
        for g in legs:
            if box_goal(g) is not None:
                assert bq.box_center(leg_target(g)) is not None, (plan, g["name"])


def test_guard_mask_only_inside_box():
    c = (2.0, 2.0)
    h = bq.BOX_HALF + bq.GUARD_MARGIN
    pts = [(2.0, 2.0), (2.0 + h - 0.001, 2.0), (2.0 + h + 0.001, 2.0), None]
    assert bq.guard_mask(pts, c) == [True, True, False, False]
    assert bq.guard_mask(pts, None) == [False] * 4


def test_body_box():
    assert bq.in_body((0.20, 0.10))
    assert not bq.in_body((0.24, 0.0))


def test_scan_points_upside_down_lidar():
    """roll π + yaw π (= y 축 180°, 쿼터니언 (0, 1, 0, 0)): 라이다 +x 광선 → base_link -x, +y 광선 → base_link +y"""
    tf = ((0.1318, 0.0, 0.0525), (0.0, 1.0, 0.0, 0.0))
    pts = bq.scan_points([1.0, 1.0], 0.0, math.pi / 2, 0.05, 12.0, tf)
    assert pts[0] == pytest.approx((0.1318 - 1.0, 0.0))
    assert pts[1] == pytest.approx((0.1318, 1.0))
    s2 = math.sqrt(0.5)
    tf_yaw = ((0.0, 0.0, 0.0), (0.0, 0.0, s2, s2))   # yaw 90°
    assert bq.scan_points([2.0], 0.0, 0.1, 0.05, 12.0, tf_yaw)[0] == pytest.approx((0.0, 2.0))
    assert bq.scan_points([float("inf"), float("nan"), 0.01], 0.0, 0.1, 0.05, 12.0, tf_yaw) == [None, None, None]


# 11-21 (밤) 정지점 = 발제 상자 면 중앙 정면, 앞 끝 ↔ 면 STOP_GAP
@pytest.mark.parametrize("name", sorted(bq.FACE_YAW))
def test_stop_pose_gap_and_center(name):
    x, y, th = bq.stop_pose(name)
    fx, fy = x + bq.FRONT * math.cos(th), y + bq.FRONT * math.sin(th)    # 상판 앞 끝 가운데
    cx, cy = bq.face_center(name, th)
    assert math.hypot(cx - fx, cy - fy) == pytest.approx(bq.STOP_GAP, abs=1e-3)
    # 옆 어긋남 0 (면 중앙 정면)
    assert abs(-math.sin(th) * (cx - fx) + math.cos(th) * (cy - fy)) < 1e-3


def test_stop_pose_directions():
    assert bq.stop_pose("box1") == pytest.approx([1.45, 0.7377, math.pi / 2], abs=1e-4)    # 손그림: 남쪽에서 (11-22 줄자 실측 상자 (1.45, 1.08))
    assert bq.stop_pose("L2") == pytest.approx([2.5523, 3.72, math.pi], abs=1e-4)          # 손그림: 동쪽 면 (실측 (2.21, 3.72))
    assert bq.stop_pose("A1") is None and bq.face_center("A1", 0.0) is None


@pytest.mark.parametrize("plan", ["demo_plan_wide.json", "demo_plan_auto.json"])
def test_plan_stops_from_boxes(plan):
    """시연 계획 정지점 = stop_pose (어제 실측 좌표가 남아 있지 않게) · leg 'box' 정보"""
    d = json.load(open(os.path.join(HERE, plan), encoding="utf-8"))
    for g in d["legs"]:
        t = leg_target(g)
        if t in bq.BOX_STOPS:
            assert g["goal"] == pytest.approx(bq.stop_pose(t), abs=1e-4), g["name"]
            assert g["box"]["name"] == t and g["box"]["gap"] == bq.STOP_GAP
