"""발제 후 검토 제출물 (mission_submit): Q1 예제 통과 · 불가능 사례 실패."""
import json
import os

import yaml

from mission_planner.submission import main

HERE = os.path.dirname(os.path.abspath(__file__))
Q1 = os.path.join(HERE, "..", "config", "problem_q1_example.yaml")
FILES = ("problem.md", "signals.json", "placement.csv", "allocation_answers.md", "allocation_answers.json", "verification.md")


def test_q1_example(tmp_path):
    assert main(["--problem", Q1, "--out", str(tmp_path)]) == 0
    for f in FILES:
        assert (tmp_path / f).exists(), f
    sig = json.loads((tmp_path / "signals.json").read_text(encoding="utf-8"))
    assert sig["counts"] == {"R": 4, "Y": 2, "G": 6}
    assert [sig["locations"][L]["group"] for L in ("L1", "L2", "L3")] == ["Y", "R", "G"]
    ans = json.loads((tmp_path / "allocation_answers.json").read_text(encoding="utf-8"))
    assert ans["n"] == 6
    for a in ans["answers"]:   # 12 개 모두 한 번씩
        assert sorted(a["L1"] + a["L2"] + a["L3"]) == list(range(12))
    # placement.csv 로 다시 넣어도 같은 정답
    out2 = tmp_path / "csv"
    assert main(["--placement", str(tmp_path / "placement.csv"), "--signal", "RGRRGRGGYGYG", "--barcode", "P2-01",
                 "--out", str(out2)]) == 0
    assert json.loads((out2 / "allocation_answers.json").read_text(encoding="utf-8")) == ans


def test_impossible_case(tmp_path):
    c = next(c for c in yaml.safe_load(open(os.path.join(HERE, "cases_b.yaml"), encoding="utf-8"))["cases"]
             if c["name"].startswith("X2"))
    m = {"r": "red", "g": "green", "c": "cylinder", "b": "box"}
    rows = [dict(id=i, color=m[s[0]], shape=m[s[1]], box=i // 2 + 1, side=-1 if i % 2 == 0 else 1)
            for i, s in enumerate(c["pucks"])]
    prob = tmp_path / "x2.yaml"
    prob.write_text(yaml.safe_dump(dict(signal=c["signal"], barcode=c["barcode"], pucks=rows)), encoding="utf-8")
    assert main(["--problem", str(prob), "--out", str(tmp_path / "out")]) == 2
    v = (tmp_path / "out" / "verification.md").read_text(encoding="utf-8")
    assert "**실패**" in v and "한 모양으로 못 만듦" in v
    assert not (tmp_path / "out" / "allocation_answers.json").exists()
