import os

import pytest
import yaml

from mission_planner.planner import TILT_SLOTS, plan
from mission_planner.poses import DICE

PROB = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "problem_q1_example.yaml"), encoding="utf-8"))


@pytest.mark.parametrize("mode", ["per_location", "single_load"])
def test_plan_q1(world, mode):
    p = plan(world, PROB, mode)
    picks = [s for s in p["steps"] if s["type"] == "pick"]
    places = [s for s in p["steps"] if s["type"] == "place"]
    assert sorted(s["puck"] for s in picks) == list(range(12)) and sorted(s["puck"] for s in places) == list(range(12))
    assert [p["groups"][L]["group"] for L in ("L1", "L2", "L3")] == ["Y", "R", "G"]  # P2-01 오름차순, R4 Y2 G6
    for L, a in p["allocation"].items():
        assert len(a["ids"]) == p["groups"][L]["count"]
        cells = sorted(s["cell"] for s in places if s["at"] == L)
        assert cells == sorted(f"{dx},{dy}" for dx, dy in DICE[len(a["ids"])])
    assert all(s["tilt"] and tuple(s["slot"]) in TILT_SLOTS for s in picks)          # 모두 45°
    assert all(s["mode"] == "tilt" for s in places)
    for s in picks:                                                                  # 상자 퍽만 제자리 회전 (inline 은 0)
        a = world.approaches[s["at"]]
        if s["shape"] == "box" and not a.extra["inline"]:
            assert 16.5 < abs(s["turn_deg"]) < 17.5
        else:
            assert s["turn_deg"] == 0.0
    if mode == "per_location":
        assert all(s["slot"][0] in (1, 2) for s in picks)            # 느린 3 열 안 씀
    else:
        rows = [s["slot"][0] for s in picks]
        assert rows == sorted(rows, reverse=True)                      # 3 열 -> 2 열 -> 1 열로 싣기
        out = [s["from_slot"][0] for s in places]
        assert out == sorted(out)                                     # 1 열 -> 2 열 -> 3 열로 꺼내기
    assert p["steps"][0]["type"] == "observe" and p["steps"][-1]["type"] == "return"
    assert {s["at"] for s in p["steps"] if s["type"] == "verify"} == {"L1", "L2", "L3"}


@pytest.mark.parametrize("mode", ["per_location", "single_load"])
@pytest.mark.parametrize("stop", ["observe", "first_pick", "first_place"])
def test_stop_after(world, mode, stop):
    """부분 실물 시험: 그 지점까지만 하고 HOME 으로 돌아와 3 s 정지"""
    from mission_planner.mission_lines import to_lines
    full = plan(world, PROB, mode)
    p = plan(world, PROB, mode, stop_after=stop)
    st = p["steps"]
    types = [s["type"] for s in st]
    assert types[0] == "observe" and types[-1] == "return" and p["stop_after"] == stop
    assert p["warnings"][0].startswith("부분 시험")
    lines = to_lines(world, p)[0]
    assert lines[-1] == "wait 3"
    if stop == "observe":
        assert types == ["observe", "return"] and lines == ["wait 3"]
        return
    assert types[-2] == "navigate" and st[-2]["to"] == "HOME"
    if stop == "first_pick":
        picks = [s for s in st if s["type"] == "pick"]
        first = next(s for s in full["steps"] if s["type"] == "pick")
        assert picks and {s["at"] for s in picks} == {first["at"]}
        assert "place" not in types and "verify" not in types
    else:
        places = [s for s in st if s["type"] == "place"]
        L = places[0]["at"]
        assert {s["at"] for s in places} == {L}
        assert len(places) == len(full["allocation"][L]["ids"])
        assert [s["at"] for s in st if s["type"] == "verify"] == [L]
        assert st[types.index("verify") - 1] == dict(st[types.index("verify") - 1], type="navigate", to=L)
        assert any(x.startswith("approach_box") for x in lines) and any(x.startswith(f"verify {L} ") for x in lines)


def test_stop_after_bad(world):
    with pytest.raises(ValueError):
        plan(world, PROB, "per_location", stop_after="half")
