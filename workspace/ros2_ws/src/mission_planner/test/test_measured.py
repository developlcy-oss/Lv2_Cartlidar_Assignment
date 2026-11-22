"""잰 퍽 자리 집기 규칙 (measured.py, 사용자 규칙 11-16: 퍽 중심 = 박스 중심에서 3 ~ 6 cm)"""
import copy
import math
import os
import random

import yaml

from mission_planner import measured as M
from mission_planner.mission_lines import to_lines, turn_name
from mission_planner.planner import plan
from mission_planner.poses import START_BOXES

PROB = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "problem_q1_example.yaml"), encoding="utf-8"))


def with_xy(prob, place):
    """place(박스, side) -> 박스 중심 기준 (dx, dy) [m]"""
    p = copy.deepcopy(prob)
    for r in p["pucks"]:
        cx, cy = START_BOXES[r["box"]]
        dx, dy = place(r["box"], r["side"])
        r["xy"] = [cx + dx, cy + dy]
    return p


def picks(pl):
    return [s for s in pl["steps"] if s["type"] == "pick"]


def test_no_xy_unchanged(world):
    """xy 가 없으면 지금 계획과 같음"""
    a = plan(world, PROB)
    assert all("look" not in s for s in picks(a))


def test_fixed_slots_same_as_table(world):
    """xy = 고정 ±40 mm 자리 -> 틈 50 mm (규칙 1) -> 회전 · J1 거리 · 차례가 자리표와 같음"""
    a = plan(world, PROB)
    b = plan(world, with_xy(PROB, lambda bx, sd: (0.04 * sd, 0.0)))
    assert [s["puck"] for s in picks(a)] == [s["puck"] for s in picks(b)]
    for s, t in zip(picks(a), picks(b)):
        assert s["at"] == t["at"]
        assert abs(s["turn_deg"] - t["turn_deg"]) < 0.01
        assert abs(s["j1_dist"] - t["j1_dist"]) < 0.005   # 표 = 회전 전 거리, 잰 자리 = 회전 뒤


def test_tight_pair_neighbor_behind(world):
    """두 퍽이 지도 X 로 붙음 (틈 10 mm): 남 · 북 면에서는 옆 퍽이 닫는 방향 옆 -> ±X 면이 열린 박스 (B1 E · B5 W · B6 W) 는
    그 면으로 옆 퍽을 뒤에 두고, 못 맞추는 박스 (B2 · B3 = 남 · 북만) 는 경고.
    2026-11-22 줄자 실측 좌표: box4 가 동쪽으로 10.6 cm 옮겨져 B4:W 가 열림 -> box4 는 더 이상 위험 아님 (이전 {2, 3, 4})"""
    pr = with_xy(PROB, lambda bx, sd: (0.02 * sd, 0.035))   # 중심 거리 40 mm, 박스 중심에서 40 mm
    pl = plan(world, pr)
    risky = {int(w.split()[1].rstrip(":")) for w in pl["warnings"] if w.startswith("박스 ") and "실패 위험" in w}
    assert risky == {2, 3}
    seen = set()
    for s in picks(pl):
        first = s["box"] not in seen
        seen.add(s["box"])
        assert REACH_OK(s)
        if first and s["box"] not in risky:
            assert s["nb_deg"] is not None and s["nb_deg"] <= M.NB_OK + 1e-6, s
            assert abs(s["j1_yaw_deg"]) <= M.J1_YAW_MAX + 1e-6
    assert {s["at"] for s in picks(pl) if s["box"] in (1, 5, 6)} == {"B1:E", "B5:W", "B6:W"}


def REACH_OK(s):
    return 0.12 <= s["j1_dist"] <= 0.35


def test_random_layouts_valid(world):
    """무작위 자리 (중심 3 ~ 6 cm, 중심 거리 35 mm 이상) 20 판: 계획 · 미션 줄이 나오고 target_look 이 잰 자리"""
    rng = random.Random(3)
    for _ in range(20):
        P = {}
        for b in range(1, 7):
            while True:
                q = [(rng.uniform(0.03, 0.06), rng.uniform(-math.pi, math.pi)) for _ in range(2)]
                xy = [(r * math.cos(t), r * math.sin(t)) for r, t in q]
                if math.dist(*xy) >= 0.035:
                    break
            P[(b, -1)], P[(b, 1)] = xy
        pl = plan(world, with_xy(PROB, lambda bx, sd: P[(bx, sd)]))
        lines, _ = to_lines(world, pl)
        assert len(picks(pl)) == 12
        for s in picks(pl):
            assert "look" in s and REACH_OK(s), s
        looks = [ln for ln in lines if "target_look" in ln]
        assert len(looks) == 12


def test_turn_name_fine():
    assert turn_name(17.06) == "turn_p17"
    assert turn_name(12.5, fine=True) == "turn_p12.5"
    assert turn_name(-0.3, fine=True) == "turn_0"
    assert turn_name(-23.37, fine=True) == "turn_m23.4"
