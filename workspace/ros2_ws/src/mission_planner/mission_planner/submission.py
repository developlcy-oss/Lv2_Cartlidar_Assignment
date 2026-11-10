"""발제 후 검토 제출물 (발제 v5 07 쪽): problem.md · signals.json · placement.csv · 할당 정답표 · 검증 기록.

  mission_submit --problem <문제>.yaml [--out submission/review]
  mission_submit --placement placement.csv --signal RGRRGRGGYGYG --barcode P2-01 [--out …]

입력 퍽 = 계획기 문제 파일 형식 (id = 퍽 AprilTag ID 0 ~ 11 · color · shape · box 1 ~ 6 · side −1|+1). placement.csv 도 같은 열.
검증 순서는 발제 04 쪽 운영자 검증 ① ~ ④ 를 그대로 따름. 규칙 위반이면 검증 기록에 실패 원인을 쓰고 종료 코드 2.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from datetime import date

import yaml

from .allocation import (BARCODES, GROUPS, MissionError, count_signal, enumerate_allocations, order_groups,
                         parse_pucks, y_options)

# 발제 07 쪽 시작 박스 1 ~ 6 중심 (mm). 퍽 = 중심 X ± 40, Y 같음, 바닥 Z 70.
BOX_CENTER_MM = {1: (1468, 950), 2: (3670, 3152), 3: (2936, 5354), 4: (734, 5354), 5: (2202, 5354), 6: (3670, 3886)}
PUCK_DX_MM, PUCK_Z_MM = 40, 70
ANSWER_LIMIT = 5000   # 정답표에 적을 할당 수 상한 (12 개라 실제로는 수백 개 이하)


def puck_xyz_mm(p):
    cx, cy = BOX_CENTER_MM[p.box]
    return cx + PUCK_DX_MM * p.side, cy, PUCK_Z_MM


def read_placement_csv(path):
    with open(path, encoding="utf-8") as f:
        return [dict(id=r["id"], color=r["color"], shape=r["shape"], box=r["box"], side=r["side"]) for r in csv.DictReader(f)]


def verify(rows, signal, barcode):
    """발제 04 쪽 ① ~ ④. -> (checks [(단계, 통과, 내용)], 결과 dict 또는 None)."""
    checks = []
    try:
        pucks = parse_pucks(rows)
        nr = sum(p.color == "red" for p in pucks)
        checks.append(("① 퍽 12 개 ID · 색 · 모양 등록", True,
                       f"ID 0 ~ 11 한 번씩, 빨강 {nr} · 초록 {12 - nr}, 자리 겹침 없음"))
    except MissionError as e:
        return checks + [("① 퍽 12 개 ID · 색 · 모양 등록", False, str(e))], None
    try:
        counts = count_signal(signal)
        checks.append(("② 신호 후보 (합 12, 각 1 ~ 6)", True, " · ".join(f"{g} {counts[g]}" for g in GROUPS)))
        groups = order_groups(counts, barcode)
        checks.append(("② 바코드 해석", True, f"{barcode} = {'내림' if BARCODES[barcode] == 'desc' else '오름'}차순 -> "
                       + " · ".join(f"L{k + 1} {g}" for k, (g, _) in enumerate(groups))))
    except MissionError as e:
        return checks + [("② 신호 · 바코드", False, str(e))], None
    try:
        opts = y_options(pucks, counts)
        checks.append(("③ Y 한 모양 + 남은 퍽이 R · G 에 정확히", True,
                       " / ".join(f"Y = {o['shape']} (빨강 {o['y_red']} · 초록 {o['y_green']})" for o in opts)))
    except MissionError as e:
        return checks + [("③ Y 한 모양 + 남은 퍽이 R · G 에 정확히", False, str(e))], None
    allocs = list(enumerate_allocations(pucks, counts, limit=ANSWER_LIMIT))
    checks.append(("④ 유효 전체 할당 1 개 이상", bool(allocs), f"{len(allocs)} 개"
                   + (f" (상한 {ANSWER_LIMIT} 에서 멈춤)" if len(allocs) >= ANSWER_LIMIT else "")))
    return checks, dict(pucks=pucks, counts=counts, groups=groups, allocs=allocs)


def write_all(out, rows, signal, barcode, checks, res, src):
    os.makedirs(out, exist_ok=True)
    ok = res is not None and all(c[1] for c in checks)
    # placement.csv
    if res:
        with open(os.path.join(out, "placement.csv"), "w", encoding="utf-8", newline="") as f:
            w = csv.writer(f)
            w.writerow(["id", "color", "shape", "box", "side", "x_mm", "y_mm", "z_mm"])
            for p in sorted(res["pucks"], key=lambda p: p.id):
                w.writerow([p.id, p.color, p.shape, p.box, p.side, *puck_xyz_mm(p)])
    # signals.json
    sig = dict(sequence=list(signal), barcode=barcode)
    if res:
        sig.update(counts=res["counts"], order="desc" if BARCODES[barcode] == "desc" else "asc",
                   locations={f"L{k + 1}": dict(group=g, count=n) for k, (g, n) in enumerate(res["groups"])})
    json.dump(sig, open(os.path.join(out, "signals.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    # 할당 정답표
    if res:
        group_of_L = {f"L{k + 1}": g for k, (g, _) in enumerate(res["groups"])}
        answers = [dict(y_shape=ys, **{L: [p.id for p in a[g]] for L, g in group_of_L.items()}) for a, ys in res["allocs"]]
        json.dump(dict(locations=group_of_L, n=len(answers), answers=answers),
                  open(os.path.join(out, "allocation_answers.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        lines = ["# 할당 정답표", "", f"장소 그룹: " + " · ".join(f"{L} = {g}" for L, g in group_of_L.items())
                 + f" / 유효 할당 {len(answers)} 개 (모두 정답)", "",
                 "R · G 는 색만, Y 는 한 모양 (색 섞임 가능). 아래 중 어느 것이든 정답이다.", "",
                 "| # | Y 모양 | " + " | ".join(f"{L} ({g})" for L, g in group_of_L.items()) + " |",
                 "|---|---|" + "---|" * 3]
        for i, a in enumerate(answers[:50], 1):
            lines.append(f"| {i} | {a['y_shape']} | " + " | ".join(", ".join(map(str, a[L])) for L in group_of_L) + " |")
        if len(answers) > 50:
            lines.append(f"\n(앞 50 개만. 전체는 allocation_answers.json)")
        open(os.path.join(out, "allocation_answers.md"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    # 검증 기록
    v = ["# 검증 기록", "", f"- 날짜: {date.today().isoformat()}", f"- 입력: `{src}`",
         f"- 결과: **{'통과' if ok else '실패'}**", "", "| 단계 | 결과 | 내용 |", "|---|---|---|"]
    v += [f"| {s} | {'통과' if t else '실패'} | {d} |" for s, t, d in checks]
    open(os.path.join(out, "verification.md"), "w", encoding="utf-8").write("\n".join(v) + "\n")
    # problem.md
    pm = ["# 팀 제출 문제", "", f"- 신호 (12 회): `{signal}`", f"- 바코드: `{barcode}`"]
    if res:
        c = res["counts"]
        pm += [f"- 횟수: R {c['R']} · Y {c['Y']} · G {c['G']}",
               "- 장소 그룹: " + " · ".join(f"L{k + 1} = {g} ({n})" for k, (g, n) in enumerate(res["groups"])), "",
               "## 퍽 12 개 (ID · 색 · 모양 · 시작 좌표 mm, 발제 좌표)", "",
               "| ID | 색 | 모양 | 박스 | x | y | z |", "|---|---|---|---|---|---|---|"]
        for p in sorted(res["pucks"], key=lambda p: p.id):
            x, y, z = puck_xyz_mm(p)
            pm.append(f"| {p.id} | {p.color} | {p.shape} | {p.box} | {x} | {y} | {z} |")
    pm += ["", "## 관련 파일", "", "- `signals.json` — 신호 순서 · 횟수 · 바코드 · 장소 그룹",
           "- `placement.csv` — 퍽 속성 · 시작 좌표", "- `allocation_answers.md` · `.json` — 유효 할당 정답 전부",
           f"- `verification.md` — 발제 04 쪽 운영자 검증 ① ~ ④ 결과 ({'통과' if ok else '실패'})"]
    open(os.path.join(out, "problem.md"), "w", encoding="utf-8").write("\n".join(pm) + "\n")
    return ok


def main(argv=None):
    ap = argparse.ArgumentParser(description="발제 후 검토 제출물 만들기")
    ap.add_argument("--problem", help="계획기 문제 파일 (signal · barcode · pucks)")
    ap.add_argument("--placement", help="placement.csv (id,color,shape,box,side)")
    ap.add_argument("--signal")
    ap.add_argument("--barcode")
    ap.add_argument("--out", default="submission/review")
    a = ap.parse_args(argv)
    if a.problem:
        prob = yaml.safe_load(open(a.problem, encoding="utf-8"))
        rows, signal, barcode, src = prob["pucks"], prob["signal"], prob["barcode"], a.problem
    elif a.placement and a.signal and a.barcode:
        rows, signal, barcode, src = read_placement_csv(a.placement), a.signal, a.barcode, a.placement
    else:
        ap.error("--problem 또는 --placement · --signal · --barcode")
    checks, res = verify(rows, signal, barcode)
    ok = write_all(a.out, rows, signal, barcode, checks, res, src)
    for s, t, d in checks:
        print(f"[{'통과' if t else '실패'}] {s}: {d}")
    print(f"-> {a.out}/ problem.md · signals.json · placement.csv · allocation_answers.md/.json · verification.md")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
