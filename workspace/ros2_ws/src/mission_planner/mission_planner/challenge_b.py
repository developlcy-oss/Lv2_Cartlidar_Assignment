"""도전 B. 퍽 할당 알고리즘 비교 (발제 v5 10 쪽) 제출물.

  mission_challenge_b [--cases test/cases_b.yaml] [--out challenge/B] [--repeat 200]
  -> cases.json · expected.json · results.csv · report.md

비교: 순차 선택 (allocate_greedy, R · G 를 ID 순으로 먼저) vs 탐색 (enumerate_allocations, Y 모양을 먼저 정하고 전부 나열).
입력 순서 = ID 0 ~ 11, 정렬 규칙 = 바코드 (allocation.order_groups) 고정. 탐색 방식의 "선택 ID" 는 나열 첫 번째 (ID 가 가장 작은 조합).
경기에서는 그 중 경로 비용이 가장 작은 것을 고른다 (planner.plan).
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import sys
import time
from datetime import date

import yaml

from .allocation import (MissionError, allocate_greedy, count_signal, enumerate_allocations, order_groups, parse_pucks,
                         y_options)

HERE = os.path.dirname(os.path.abspath(__file__))
COL = {"r": "red", "g": "green"}
SHP = {"c": "cylinder", "b": "box"}


def default_cases():
    for p in (os.path.join(HERE, "..", "test", "cases_b.yaml"),):
        if os.path.exists(p):
            return os.path.normpath(p)
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("mission_planner"), "test", "cases_b.yaml")


def rows_of(codes):
    return [dict(id=i, color=COL[c[0]], shape=SHP[c[1]], box=i // 2 + 1, side=-1 if i % 2 == 0 else 1) for i, c in enumerate(codes)]


def check(alloc, counts):
    """발제 조건: 12 개 중복 · 누락 없음, R 빨강만, G 초록만, Y 한 모양, 개수 = 횟수."""
    ids = sorted(p.id for g in alloc.values() for p in g)
    return (ids == list(range(12))
            and all(p.color == "red" for p in alloc["R"]) and len(alloc["R"]) == counts["R"]
            and all(p.color == "green" for p in alloc["G"]) and len(alloc["G"]) == counts["G"]
            and len(alloc["Y"]) == counts["Y"] and len({p.shape for p in alloc["Y"]}) == 1)


def search_first(pucks, counts):
    y_options(pucks, counts)               # 불가능이면 여기서 명시적 오류
    for alloc, yshape in enumerate_allocations(pucks, counts):
        return alloc, yshape
    raise MissionError("유효 할당 없음")


METHODS = {"sequential": allocate_greedy, "search": search_first}


def run_case(case, method, repeat):
    pucks = parse_pucks(rows_of(case["pucks"]))
    counts = count_signal(case["signal"])
    groups = order_groups(counts, case["barcode"])
    fn = METHODS[method]
    t0 = time.perf_counter()
    for _ in range(repeat):
        try:
            res, err = fn(pucks, counts), None
        except MissionError as e:
            res, err = None, str(e)
    ms = (time.perf_counter() - t0) * 1000 / repeat
    row = dict(case=case["name"], method=method, valid_case=not case.get("impossible", False), time_ms=round(ms, 4))
    if res is None:
        row.update(success=False, error=True, L1="", L2="", L3="", y_shape="", reason=err)
    else:
        alloc, yshape = res
        ok = check(alloc, counts)
        row.update(success=ok, error=False, y_shape=yshape, reason="" if ok else "조건 위반",
                   **{f"L{k + 1}": " ".join(str(p.id) for p in alloc[g]) for k, (g, _) in enumerate(groups)})
    # 판정: 유효 사례는 성공해야, 불가능 사례는 명시적 오류여야 통과
    row["pass"] = row["success"] if row["valid_case"] else row["error"]
    return row


def expected_of(case):
    counts = count_signal(case["signal"])
    groups = order_groups(counts, case["barcode"])
    e = dict(name=case["name"], counts=counts, locations={f"L{k + 1}": g for k, (g, _) in enumerate(groups)})
    if case.get("impossible"):
        try:
            y_options(parse_pucks(rows_of(case["pucks"])), counts)
            e.update(result="error", reason="(오류가 나야 하는데 안 남)")
        except MissionError as err:
            e.update(result="error", reason=str(err))
    else:
        allocs = list(enumerate_allocations(parse_pucks(rows_of(case["pucks"])), counts))
        e.update(result="ok", y_count=counts["Y"], n_valid_allocations=len(allocs),
                 y_shapes=sorted({ys for _, ys in allocs}))
    return e


def why_greedy_ok(case):
    """순차 선택이 성공한 이유: R · G 를 ID 순으로 가져간 뒤 남은 퍽이 우연히 한 모양."""
    pucks = parse_pucks(rows_of(case["pucks"]))
    counts = count_signal(case["signal"])
    alloc, yshape = allocate_greedy(pucks, counts)
    left = ", ".join(f"{p.id}({p.color[0]}{p.shape[0]})" for p in alloc["Y"])
    return f"R · G 를 ID 순으로 가져간 뒤 남은 Y 후보 [{left}] 가 모두 {yshape}"


def write_report(out, cases, rows, repeat, src):
    lines = ["# 도전 B. 퍽 할당 알고리즘 비교", "",
             f"- 날짜: {date.today().isoformat()} · 환경: Python {platform.python_version()} · {platform.machine()}",
             f"- 사례: `{os.path.basename(src)}` 6 건 (유효 4 · 불가능 2). 불가능 사례는 소프트웨어 오류 시험용 (공식 출제 아님)",
             "- 입력 순서: 퍽 ID 0 ~ 11 (박스 = id // 2 + 1, 짝수 X − 40 · 홀수 X + 40). ID 와 색은 고정 대응 아님",
             "- 정렬 규칙: P2-01 · P2-03 오름차순, P2-02 · P2-04 내림차순, 동률이면 R → Y → G",
             f"- 실행 시간: 사례마다 {repeat} 번 돌린 평균 (ms)", "",
             "## 방식", "",
             "| 방식 | 코드 | 내용 |", "|---|---|---|",
             "| 순차 선택 (기본) | `mission_planner/allocation.py` `allocate_greedy` | R 은 빨강을, G 는 초록을 ID 순으로 먼저 가져가고 남은 것을 Y 로. 남은 것이 섞이면 실패 |",
             "| 탐색 (개선) | `mission_planner/allocation.py` `y_options` + `enumerate_allocations` | Y 에 들어갈 빨강 수 = 빨강 − R, 초록 수 = 초록 − G 로 정해짐 → Y 모양 (2 가지) 마다 같은 색 · 모양 안에서 ID 조합 전부 나열. 불가능이면 먼저 명시적 오류 |", "",
             "## 결과", "",
             "| 사례 | 방식 | 판정 | 시간 ms | L1 | L2 | L3 | Y 모양 | 실패 원인 |", "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['case']} | {r['method']} | {'통과' if r['pass'] else '**실패**'} | {r['time_ms']} | {r['L1']} | {r['L2']} | {r['L3']} | {r['y_shape']} | {r['reason']} |")
    lines += ["", "## 요약", "", "| 방식 | 통과 / 6 | 유효 4 건 성공 | 불가능 2 건 명시적 오류 | 평균 시간 ms |", "|---|---|---|---|---|"]
    for m in METHODS:
        rs = [r for r in rows if r["method"] == m]
        lines.append(f"| {m} | {sum(r['pass'] for r in rs)} | {sum(r['success'] for r in rs if r['valid_case'])} / 4 | "
                     f"{sum(r['error'] for r in rs if not r['valid_case'])} / 2 | {sum(r['time_ms'] for r in rs) / len(rs):.4f} |")
    lines += ["", "## 순차 선택: 사례별 성공 · 실패 이유", ""]
    for c in cases:
        if c.get("impossible"):
            continue
        r = next(r for r in rows if r["case"] == c["name"] and r["method"] == "sequential")
        if r["success"]:
            lines.append(f"- {c['name']}: {why_greedy_ok(c)}")
        else:
            lines.append(f"- {c['name']}: **실패** — {r['reason']}. 탐색 방식은 Y 모양을 먼저 정해 같은 퍽으로 성공")
    seq_fail = [r["case"] for r in rows if r["method"] == "sequential" and r["valid_case"] and not r["success"]]
    search_ok = all(r["pass"] for r in rows if r["method"] == "search")
    lines += ["", "## 결론", "",
              f"- 탐색 방식: 유효 4 건 {'모두' if search_ok else '일부만'} 12 개를 중복 · 누락 없이 할당, 불가능 2 건은 할당 전에 y_options 에서 원인을 밝힌 명시적 오류 (안전 정지).",
              f"- 순차 선택: 유효 사례 {len(seq_fail)} 건 실패 ({', '.join(seq_fail) or '없음'}). R · G 가 ID 순으로 먼저 가져가면서 Y 에 필요한 모양을 써 버려 남은 후보 모양이 섞임.",
              "- 순차 선택도 불가능 2 건에서 오류를 내지만, 할당 가능 여부를 먼저 따지지 않아 \"불가능한 문제\" 와 \"순서 때문에 실패\" 를 구별하지 못한다 (V1 · X2 의 실패 원인 문구가 같음).",
              "- 12 개라 탐색 비용은 작다 (조합 수 = 같은 색 · 모양 안 선택뿐). 경기 계획기는 이 나열에서 경로 비용 최소를 고른다 (`planner.plan`).", "",
              "## 파일", "", "- `cases.json` 입력 6 건 · `expected.json` 기대 결과 · `results.csv` 방식 × 사례 12 행",
              "- 다시 만들기: `ros2 run mission_planner mission_challenge_b --out challenge/B`"]
    open(os.path.join(out, "report.md"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description="도전 B 제출물 만들기")
    ap.add_argument("--cases", default=None, help="기본 = 패키지 test/cases_b.yaml")
    ap.add_argument("--out", default="challenge/B")
    ap.add_argument("--repeat", type=int, default=200)
    a = ap.parse_args(argv)
    a.cases = a.cases or default_cases()
    cases = yaml.safe_load(open(a.cases, encoding="utf-8"))["cases"]
    os.makedirs(a.out, exist_ok=True)
    json.dump([dict(name=c["name"], signal=c["signal"], barcode=c["barcode"], pucks=rows_of(c["pucks"]),
                    impossible=bool(c.get("impossible"))) for c in cases],
              open(os.path.join(a.out, "cases.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    json.dump([expected_of(c) for c in cases], open(os.path.join(a.out, "expected.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    rows = [run_case(c, m, a.repeat) for c in cases for m in METHODS]
    keys = ["case", "method", "valid_case", "pass", "success", "error", "time_ms", "L1", "L2", "L3", "y_shape", "reason"]
    with open(os.path.join(a.out, "results.csv"), "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows({k: r[k] for k in keys} for r in rows)
    write_report(a.out, cases, rows, a.repeat, a.cases)
    for m in METHODS:
        rs = [r for r in rows if r["method"] == m]
        print(f"[{m}] 통과 {sum(r['pass'] for r in rs)} / {len(rs)}")
    print(f"-> {a.out}/ cases.json · expected.json · results.csv · report.md")
    search_ok = all(r["pass"] for r in rows if r["method"] == "search")
    return 0 if search_ok else 1


if __name__ == "__main__":
    sys.exit(main())
