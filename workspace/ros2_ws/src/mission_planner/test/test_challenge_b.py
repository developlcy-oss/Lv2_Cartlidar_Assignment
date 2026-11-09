"""도전 B 제출물 (mission_challenge_b): 탐색 6 / 6, 순차 선택 3 / 6 (V1 · V2 · V3 실패)."""
import csv
import os

from mission_planner.challenge_b import main

CASES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cases_b.yaml")


def test_challenge_b(tmp_path):
    assert main(["--cases", CASES, "--out", str(tmp_path), "--repeat", "3"]) == 0
    for f in ("cases.json", "expected.json", "results.csv", "report.md"):
        assert (tmp_path / f).exists(), f
    rows = list(csv.DictReader(open(tmp_path / "results.csv", encoding="utf-8")))
    assert len(rows) == 12
    passed = {m: {r["case"][:2] for r in rows if r["method"] == m and r["pass"] == "True"} for m in ("search", "sequential")}
    assert passed["search"] == {"V1", "V2", "V3", "V4", "X1", "X2"}
    assert passed["sequential"] == {"V4", "X1", "X2"}
