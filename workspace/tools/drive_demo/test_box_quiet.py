"""box_quiet.py 시험 (ROS 없이): cd workspace/tools/drive_demo && python3 -m pytest -q test_box_quiet.py"""
import json
import os

import pytest

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


@pytest.mark.parametrize("plan", ["demo_plan_max.json", "demo_plan_wide.json"])
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
