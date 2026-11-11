"""score.py: 에피소드 기록 -> 채점 항목별 예상 점수 (가짜 기록으로)."""
import json
import os

import pytest
import yaml

from mission_planner.score import WEIGHTS, load_episode, score

PROB = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "problem_q1_example.yaml"), encoding="utf-8"))
GROUPS = {"L1": ("Y", 2), "L2": ("R", 4), "L3": ("G", 6)}   # Q1: RGRRGRGGYGYG · P2-01


def _cells(err=0):
    return [{"error": err if k == 0 else 0, "count": 1, "offset": 0.0, "color": 1, "shape": 1} for k in range(9)]


def write_episode(d, seq="RGRRGRGGYGYG", code="P2-01", passed=True, ids_ok=True, ret=None, state="DONE"):
    os.makedirs(os.path.join(d, "planner"))
    counts = {g: seq.count(g) for g in "RYG"}
    json.dump(dict(signal=dict(sequence=list(seq), counts=counts), barcode=dict(code=code)), open(os.path.join(d, "planner", "signal.json"), "w"))
    json.dump(dict(groups={L: dict(group=g, count=n) for L, (g, n) in GROUPS.items()}), open(os.path.join(d, "planner", "allocation.json"), "w"))
    log = [dict(kind="observe", ok=True, sequence=seq, barcode=code)]
    for L in GROUPS:
        log.append(dict(kind="verify", place=L, observed=True, passed=passed, message="", cells=_cells(0 if passed else 5)))
    log.append(dict(kind="verify_ids", read=12, duplicate=[] if ids_ok else [3], missing=[] if ids_ok else [7], unexpected=[], passed=ids_ok))
    if ret is not None:
        log.append(dict(kind="return", **ret))
    log.append(dict(kind="finish", state=state, reason=""))
    with open(os.path.join(d, "mission_log.jsonl"), "w") as f:
        for e in log:
            f.write(json.dumps(e) + "\n")
    return d


GOOD_RET = dict(ok=True, inside=True, min_margin_m=0.09, moved_m=0.004, turned_deg=0.5)


def test_perfect(tmp_path):
    rows = score(load_episode(write_episode(str(tmp_path / "e"), ret=GOOD_RET)), PROB)
    assert {k: v for k, v, _, _ in rows} == WEIGHTS and sum(WEIGHTS.values()) == 100


@pytest.mark.parametrize("ret, expect", [
    (None, 0),                                         # 기록 없음
    (dict(GOOD_RET, inside=False, min_margin_m=-0.01), 0),
    (dict(GOOD_RET, moved_m=0.05), 0),                 # 3 초 동안 움직임
])
def test_return_fail(tmp_path, ret, expect):
    rows = dict((k, v) for k, v, _, _ in score(load_episode(write_episode(str(tmp_path / "e"), ret=ret)), PROB))
    assert rows["복귀"] == expect


def test_partial(tmp_path):
    # 자리 두 개 틀린 신호 (횟수 그대로: R ↔ G 맞바꿈), 바코드 틀림, verify 실패 (칸 하나 색 오류), ID 중복 · 누락
    ep = write_episode(str(tmp_path / "e"), seq="GRRRGRGGYGYG", code="P2-03", passed=False, ids_ok=False, ret=GOOD_RET)
    rows = dict((k, v) for k, v, _, _ in score(load_episode(ep), PROB))
    assert rows["신호"] == 10 and rows["횟수"] == 9 and rows["바코드"] == 0
    assert rows["눈"] == 0
    assert rows["색·모양"] == pytest.approx(6 * (1 - 1 / 2) + 6 * (1 - 1 / 4) + 6 * (1 - 1 / 6), abs=0.01)
    assert rows["재검증"] == pytest.approx(8 * 10 / 12, abs=0.01)


def test_no_logs(tmp_path):
    os.makedirs(tmp_path / "e")
    rows = dict((k, v) for k, v, _, _ in score(load_episode(str(tmp_path / "e")), PROB))
    assert sum(rows.values()) == 0
