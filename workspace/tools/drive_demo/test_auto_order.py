"""auto_order.py 생성물 시험 (widen 다시 안 돌림): cd workspace/tools/drive_demo && python3 -m pytest -q test_auto_order.py"""
import json
import math
import os
import warnings

import pytest

from auto_order import STOPS, best_order, lib_time, order_time, plan_order

HERE = os.path.dirname(os.path.abspath(__file__))
SPIN_MAX_DEG, REV_MAX = 90.0, 0.40   # 규칙 v2 이음매 (demo_follow.py · v2_filter.py 와 같음)


def load(n):
    return json.load(open(os.path.join(HERE, n), encoding="utf-8"))


AUTO = load("demo_plan_auto.json")
WIDE = load("demo_plan_wide.json")


def plen(pc):
    P = pc["poses"]
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(P, P[1:]))


def first_spin(pieces):   # demo_follow.first_spin 과 같음
    sp = moved = 0.0
    for pc in pieces:
        if pc["kind"] == "S":
            sp += abs(math.degrees(pc["dth"]))
            moved = 0.0
            continue
        moved += plen(pc)
        if moved >= 0.15:
            break
    return sp


def first_rev(pieces):   # demo_follow.first_rev 와 같음
    rev = fw = 0.0
    for pc in pieces:
        if pc["kind"] == "S":
            continue
        L = plen(pc)
        if pc["kind"] == "R":
            rev += L
            fw = 0.0
        else:
            fw += L
            if fw >= 0.15:
                break
    return rev


def reverse_pieces(pieces):
    """끝에서 거꾸로 읽은 조각 (도착 쪽 제자리 · 후진을 first_* 로 셈)"""
    return [pc if pc["kind"] == "S" else {**pc, "poses": pc["poses"][::-1]} for pc in pieces[::-1]]


def arrive_spin(pieces):
    return first_spin(reverse_pieces(pieces))


def arrive_rev(pieces):
    return first_rev(reverse_pieces(pieces))


def first_pose(pieces):
    return next(pc["poses"][0] for pc in pieces if pc["kind"] != "S")


def last_pose(pieces):
    return next(pc["poses"][-1] for pc in pieces[::-1] if pc["kind"] != "S")


def end_pose(pieces):
    """끝 자세 (마지막 움직임 자세 + 그 뒤 제자리 각)"""
    x, y, th = last_pose(pieces)
    for pc in pieces[::-1]:
        if pc["kind"] != "S":
            break
        th += pc["dth"]
    return [x, y, th]


def check_end(pieces):
    """끝 자세 = home_pose (±1 cm · ±1°) · 끝 제자리 < 90° (90 = 규칙 v2 한도 경계값이면 경고)"""
    e, h = end_pose(pieces), AUTO["home_pose"]
    assert near(e, h, 0.01)
    assert abs(math.degrees(math.remainder(e[2] - h[2], 2 * math.pi))) <= 1.0
    sp = arrive_spin(pieces)
    assert sp <= SPIN_MAX_DEG + 1e-6
    if sp >= SPIN_MAX_DEG - 1e-6:
        warnings.warn(f"끝 제자리 {sp:.0f}° = 규칙 v2 한도 경계값")
    for pc in pieces:
        if pc["kind"] == "S":
            assert set(pc) == {"kind", "dth"} and isinstance(pc["dth"], float)


def near(a, b, tol=1e-3):
    return math.hypot(a[0] - b[0], a[1] - b[1]) < tol


def test_keys():
    for k in ("plan", "course", "course_text", "start", "legs", "note_max", "note_widen", "order", "home_pose", "home_legs"):
        assert k in AUTO, k
    assert "자동" in AUTO["plan"] and "출발자세복귀" in AUTO["plan"]
    assert AUTO["order"] == plan_order(WIDE)
    assert sorted(AUTO["order"]) == sorted(STOPS)


def test_home_pose_is_start():
    h, s = AUTO["home_pose"], AUTO["start"]
    assert near(h, s, 1e-6)
    assert abs(math.remainder(h[2] - s[2], 2 * math.pi)) < 1e-6
    assert h == pytest.approx([0.6936, 0.95, math.pi], abs=1e-6)


def test_last_leg_goes_home():
    S9 = AUTO["legs"][-1]
    assert len(AUTO["legs"]) == 10 and S9["leg"] == 9
    assert S9["goal"] == pytest.approx(AUTO["home_pose"], abs=1e-6)
    check_end(S9["pieces"])
    assert near(first_pose(S9["pieces"]), WIDE["legs"][8]["goal"], 1e-4)
    assert "widen" in S9 and not S9.get("widen_failed")


def test_s0_to_s8_same_as_wide():
    for a, w in zip(AUTO["legs"][:9], WIDE["legs"][:9]):
        assert a == w


@pytest.mark.parametrize("stop", STOPS)
def test_home_legs(stop):
    hl = AUTO["home_legs"]
    assert set(hl) == set(STOPS)
    g = hl[stop]
    assert g["name"] == f"H {stop}→HOME"
    assert g["goal"] == pytest.approx(AUTO["home_pose"], abs=1e-6)
    stop_pose = next(L["goal"] for L in WIDE["legs"] if L["name"].split("→")[-1].strip() == stop)
    assert near(first_pose(g["pieces"]), stop_pose, 1e-4)   # 시작 = 그 정지점
    check_end(g["pieces"])
    assert all(pc["kind"] in ("F", "R", "S") for pc in g["pieces"])
    assert all(("poses" in pc) if pc["kind"] != "S" else ("dth" in pc) for pc in g["pieces"])
    # 끝 = 곧은 전진 들어가기 (마지막 움직임 조각 F, 마지막 0.05 m 방향 그대로) 뒤 제자리 하나까지
    mv = [pc for pc in g["pieces"] if pc["kind"] != "S"]
    assert mv[-1]["kind"] == "F" and sum(pc["kind"] == "S" for pc in g["pieces"][g["pieces"].index(mv[-1]):]) <= 1
    P = mv[-1]["poses"]
    assert abs(math.remainder(P[-1][2] - P[-6][2], 2 * math.pi)) < math.radians(2)


def seams():
    """(이름, 앞 길 조각, 다음 길 조각): 계획 이음매 + 정지점에서 home_legs 로 갈아타는 이음매"""
    legs = AUTO["legs"]
    out = [(f"{a['name']} | {b['name']}", a["pieces"], b["pieces"]) for a, b in zip(legs, legs[1:])]
    for L in legs[:-1]:
        stop = L["name"].split("→")[-1].strip()
        out.append((f"{L['name']} | H {stop}", L["pieces"], AUTO["home_legs"][stop]["pieces"]))
    return out


@pytest.mark.parametrize("name,prev,nxt", seams(), ids=[s[0] for s in seams()])
def test_seam_rule_v2(name, prev, nxt):
    assert arrive_spin(prev) + first_spin(nxt) <= SPIN_MAX_DEG + 1e-6
    assert arrive_rev(prev) + first_rev(nxt) <= REV_MAX + 1e-6


def test_opt_order_from_legs_json():
    T = lib_time(load("legs.json"))
    t, order = best_order(T)
    assert t == pytest.approx(188.3, abs=0.5)
    assert sorted(order) == sorted(STOPS) and len(order) == 9
    assert order_time(order, T) == pytest.approx(t)
    assert order_time(AUTO["order"], T) >= t


def rev_runs_max(pieces):
    """규칙 v2: 이어진 후진 (0.15 m 전진해야 다시 셈) 최대 [m]"""
    run = fw = best = 0.0
    for pc in pieces:
        if pc["kind"] == "S":
            continue
        L = plen(pc)
        if pc["kind"] == "R":
            run += L
            fw = 0.0
            best = max(best, run)
        else:
            fw += L
            if fw >= 0.15:
                run = 0.0
    return best


def spin_runs_max(pieces):
    """규칙 v2: 이어진 제자리 각 합 (0.15 m 움직여야 다시 셈) 최대 [deg]"""
    sp = mv = best = 0.0
    for pc in pieces:
        if pc["kind"] == "S":
            sp += abs(math.degrees(pc["dth"]))
            best = max(best, sp)
            continue
        mv += plen(pc)
        if mv >= 0.15:
            sp = mv = 0.0
    return best


@pytest.mark.parametrize("stop", STOPS)
def test_home_leg_rules_and_continuity(stop):
    g = AUTO["home_legs"][stop]
    assert rev_runs_max(g["pieces"]) <= REV_MAX + 1e-6
    assert spin_runs_max(g["pieces"]) < SPIN_MAX_DEG
    # 움직임 조각 안: 점 간격 ≤ 3 cm · 방향 바뀜 ≤ 5° (두 단계 이음매 W 에서 끊김 없음)
    for pc in g["pieces"]:
        if pc["kind"] == "S":
            continue
        for p, q in zip(pc["poses"], pc["poses"][1:]):
            assert math.hypot(q[0] - p[0], q[1] - p[1]) <= 0.03
            assert abs(math.degrees(math.remainder(q[2] - p[2], 2 * math.pi))) <= 5.0
    w = g.get("widen", {})
    if w.get("two_stage"):
        F = [pc for pc in g["pieces"] if pc["kind"] == "F"][-1]["poses"]
        assert any(max(abs(u - v) for u, v in zip(p, w["via"])) < 1e-4 for p in F)   # W 를 멈추지 않고 지나감


@pytest.mark.parametrize("plan", [WIDE, AUTO])
def test_no_spin_pieces_between_stops(plan):
    """11-22 곡선으로 이어 가기: 정지점 사이 구간 (S0 ~ S8) 에 제자리 회전 조각 없음 (HOME 끝 맞추기 S9 · home_legs 는 예외)"""
    for L in plan["legs"][:9]:
        assert all(pc["kind"] != "S" for pc in L["pieces"]), L["name"]


ARM_MIN = 0.06   # 11-22 팔 (BOX_VIEW) 외곽 ↔ 벽 · 기둥 (map_amcl) 최소 [m] — widen_plan --arm 0.06 (AMCL ±3 cm 여유)


def arm_paths():
    out = [(L["name"], L["pieces"]) for L in AUTO["legs"]] + [(f"W {L['name']}", L["pieces"]) for L in WIDE["legs"][:9]]
    return out + [(h["name"], h["pieces"]) for h in AUTO["home_legs"].values()]


@pytest.fixture(scope="module")
def clear():
    import widen_plan
    return widen_plan.Clear()


@pytest.mark.parametrize("name,pieces", arm_paths(), ids=[p[0] for p in arm_paths()])
def test_arm_wall_clearance(clear, name, pieces):
    """주행 내내 팔이 BOX_VIEW (최저 z 0.184 < 벽 0.39) — 정지점 · 제자리 회전 (2° 마다) 포함 팔 외곽 ↔ 벽 ≥ ARM_MIN"""
    import numpy as np
    P, q = [], None
    for pc in pieces:
        if pc["kind"] == "S":
            n = max(2, int(abs(pc["dth"]) / math.radians(2)) + 1)
            P += [[q[0], q[1], q[2] + pc["dth"] * t] for t in np.linspace(0, 1, n)]
            q = [q[0], q[1], q[2] + pc["dth"]]
        else:
            P += pc["poses"]
            q = pc["poses"][-1]
    c = clear.arm_clear(np.array(P))
    i = int(np.argmin(c))
    assert c[i] >= ARM_MIN, f"{name}: 팔 ↔ 벽 {c[i] * 100:.2f} cm @ {P[i]}"
