import os

import pytest
import yaml

import random
import time

import copy

from mission_planner import hybrid as hyb
from mission_planner.allocation import MissionError, count_signal, enumerate_allocations, order_groups, parse_pucks
from mission_planner.planner import ROW_ORDER, TILT_SLOTS, plan
from mission_planner.poses import DICE

PROB = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "problem_q1_example.yaml"), encoding="utf-8"))


def check_pallet(steps, in_row=True):
    """팔레트 규칙: 앞 열 (번호 작은 열) 에 퍽이 있으면 뒤 열 칸을 싣거나 꺼낼 수 없음 (쥔 퍽이 찬 칸 위로 못 지나감). 칸은 퍽 하나.
    in_row: 같은 열 안도 ROW_ORDER 순서로 싣고 그 반대로 꺼냄 (hybrid · per_location 은 지킴.
    single_load 는 L 별로만 반대 순서라 다른 L 퍽이 같은 열에 남음 — 팔레트 문서 규칙은 열 단위만이라 검사하지 않음)"""
    occ = {}
    for s in steps:
        if s["type"] not in ("pick", "place"):
            continue
        r, i = s["slot"] if s["type"] == "pick" else s["from_slot"]
        blocked = [k for k in occ if k[0] < r or (in_row and k[0] == r and ROW_ORDER.index(k[1]) > ROW_ORDER.index(i))]
        assert not blocked, f"{s['type']} 퍽 {s['puck']} 칸 {(r, i)}: 앞에 {blocked}"
        if s["type"] == "pick":
            assert (r, i) not in occ
            occ[(r, i)] = s["puck"]
        else:
            assert occ.pop((r, i)) == s["puck"]
    assert not occ


def check_each_place_once(steps):
    """L 마다 놓기는 한 정차에서 한 번에 다 (재검증 전 재방문 없음)"""
    seen, prev = [], None
    for s in steps:
        if s["type"] == "place" and s["at"] != prev:
            assert s["at"] not in seen, f"{s['at']} 두 번 들러 놓음"
            seen.append(s["at"])
        prev = s["at"] if s["type"] == "place" else None


@pytest.mark.parametrize("mode", ["hybrid", "per_location", "single_load"])
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
    check_pallet(p["steps"], in_row=mode != "single_load")
    check_each_place_once(p["steps"])
    if mode == "hybrid":
        assert p["hybrid"]["row3"] == 0 and all(s["slot"][0] in (1, 2) for s in picks)   # Q1 은 동시에 6 개 이하
    elif mode == "per_location":
        assert all(s["slot"][0] in (1, 2) for s in picks)            # 느린 3 열 안 씀
    else:
        rows = [s["slot"][0] for s in picks]
        assert rows == sorted(rows, reverse=True)                      # 3 열 -> 2 열 -> 1 열로 싣기
        out = [s["from_slot"][0] for s in places]
        assert out == sorted(out)                                     # 1 열 -> 2 열 -> 3 열로 꺼내기
    assert p["steps"][0]["type"] == "observe" and p["steps"][-1]["type"] == "return"
    assert {s["at"] for s in p["steps"] if s["type"] == "verify"} == {"L1", "L2", "L3"}


def test_hybrid_not_worse_q1(world):
    """hybrid 는 per_location (특수한 경우) 보다 경로가 길지 않음"""
    h, pl = plan(world, PROB, "hybrid"), plan(world, PROB, "per_location")
    assert h["cost_m"] <= pl["cost_m"] + 1e-6
    assert h["hybrid"]["objective"] == pytest.approx(h["cost_m"])   # 3 열 벌점 0


def lying_violations(steps):
    """같은 박스에서 누운 퍽을 세운 퍽보다 먼저 집은 박스 목록 (#193)"""
    order = {}
    for k, s in enumerate(steps):
        if s["type"] == "pick":
            order.setdefault(s["box"], []).append((k, s["pose"]))
    return sorted(b for b, ps in order.items()
                  if any(p == "standing" for _, p in ps) and any(p == "lying" for _, p in ps)
                  and min(k for k, p in ps if p == "lying") < max(k for k, p in ps if p == "standing"))


def _lay_boxes(prob, rng, prob_lying=0.5):
    """직육면체 퍽 일부를 눕힘 (원기둥은 눕지 않음, perception_survey lying_cylinder)"""
    prob = copy.deepcopy(prob)
    for r in prob["pucks"]:
        r["pose"] = "lying" if r["shape"] == "box" and rng.random() < prob_lying else "standing"
    return prob


def test_pose_parse():
    rows = copy.deepcopy(PROB["pucks"])
    assert all(p.pose == "standing" for p in parse_pucks(rows))       # 없으면 standing
    rows[2]["pose"] = "lying"
    rows[3]["pose"] = None                                             # null 도 standing (mission survey_pucks.hpp)
    ps = parse_pucks(rows)
    assert ps[2].pose == "lying" and ps[3].pose == "standing"
    rows[2]["pose"] = "upside"
    with pytest.raises(MissionError):
        parse_pucks(rows)


@pytest.mark.parametrize("mode", ["hybrid", "per_location", "single_load"])
def test_lying_after_q1(world, mode):
    """Q1 직육면체 퍽을 모두 눕힘: hybrid 는 같은 박스 세운 퍽 먼저를 꼭 지킴, 다른 방식은 어기면 경고"""
    prob = _lay_boxes(PROB, random.Random(0), prob_lying=1.0)
    p = plan(world, prob, mode)
    v = lying_violations(p["steps"])
    warned = [w for w in p["warnings"] if "순서 못 지킴" in w]
    if mode == "hybrid":
        assert v == [] and warned == []
    assert bool(v) == bool(warned)
    assert all(s["pose"] == ("lying" if s["shape"] == "box" else "standing") for s in p["steps"] if s["type"] == "pick")
    check_pallet(p["steps"], in_row=mode != "single_load")


def test_hybrid_lying_constraint_binds(world):
    """무작위 문제: 순서 제약이 실제로 경로를 바꾸는 경우가 있고, 바꾼 경로는 제약 없는 것보다 짧지 않음"""
    rng = random.Random(11)
    nodes = world.hybrid_nodes
    changed = 0
    for _ in range(25):
        prob = _lay_boxes(_random_problem(rng), rng)
        counts = count_signal(prob["signal"])
        gl = {f"L{k + 1}": g for k, (g, _) in enumerate(order_groups(counts, prob["barcode"]))}
        alloc, _ = next(iter(enumerate_allocations(parse_pucks(prob["pucks"]), counts, limit=1)))
        free = hyb.plan_hybrid(world, alloc, gl, nodes, lying_after=False)
        tied = hyb.plan_hybrid(world, alloc, gl, nodes)
        assert tied is not None and tied[0] >= free[0] - 1e-9
        steps = [dict(type="pick", box=p.box, pose=p.pose) for ev in tied[1] if ev[0] == "pick" for p in ev[2]]
        assert lying_violations(steps) == []
        free_steps = [dict(type="pick", box=p.box, pose=p.pose) for ev in free[1] if ev[0] == "pick" for p in ev[2]]
        changed += bool(lying_violations(free_steps))
    assert changed > 0


def _random_problem(rng):
    while True:
        sig = "".join(rng.choice("RYG") for _ in range(12))
        c = {g: sig.count(g) for g in "RYG"}
        if not all(1 <= c[g] <= 6 for g in "RYG"):
            continue
        nred = rng.randint(c["R"], 12 - c["G"])
        colors = ["red"] * nred + ["green"] * (12 - nred)
        rng.shuffle(colors)
        slots = [(b, s) for b in range(1, 7) for s in (-1, 1)]
        pucks = [dict(id=i, color=colors[i], shape=rng.choice(["cylinder", "box"]), box=slots[i][0], side=slots[i][1]) for i in range(12)]
        prob = dict(signal=sig, barcode=rng.choice(["P2-01", "P2-02", "P2-03", "P2-04"]), pucks=pucks)
        try:
            list(enumerate_allocations(parse_pucks(pucks), count_signal(sig), limit=1))
        except MissionError:
            continue
        return prob


def test_hybrid_random_valid_and_fast(world):
    """무작위 문제 20 개: 퍽 12 개 집고 놓기, 팔레트 규칙, L 한 번씩, 할당 전부를 봐도 빠름 (Jetson 여유)"""
    rng = random.Random(5)
    plan(world, PROB, "hybrid")   # 비용표 (World 당 한 번) 는 시간에서 뺌
    worst = 0.0
    for _ in range(20):
        prob = _lay_boxes(_random_problem(rng), rng)
        t = time.perf_counter()
        p = plan(world, prob, "hybrid")
        worst = max(worst, time.perf_counter() - t)
        st = p["steps"]
        assert sorted(s["puck"] for s in st if s["type"] == "pick") == list(range(12))
        assert sorted(s["puck"] for s in st if s["type"] == "place") == list(range(12))
        check_pallet(st)
        check_each_place_once(st)
        assert lying_violations(st) == [] and not any("순서 못 지킴" in w for w in p["warnings"])
        for L, a in p["allocation"].items():
            assert sorted(s["puck"] for s in st if s["type"] == "place" and s["at"] == L) == sorted(a["ids"])
        assert p["steps"][-1]["type"] == "return"
    assert worst < 1.0, f"hybrid plan {worst:.2f} s"


@pytest.mark.parametrize("mode", ["hybrid", "per_location", "single_load"])
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
    with pytest.raises(ValueError):
        plan(world, PROB, "mixed")
