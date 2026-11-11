"""미션 에피소드 폴더 하나 -> 채점 항목별 예상 점수 (경기 뒤 점검 · 반복 시험 기록용).

  ros2 run mission_planner mission_score <에피소드 폴더> --problem <정답 문제 yaml>  [--json 결과.json]
    에피소드 폴더 = mission_node 기록 폴더 <log_dir>/<YYMMDD_HHMM>/ (mission_log.jsonl, planner/signal.json · allocation.json)
    정답 문제 = 계획기 문제 파일 형식 (signal 12 개 · barcode, config/problem_q1_example.yaml). pucks 는 쓰지 않음

배점 (팀 채점표, 합 100) 과 이 스크립트의 셈 방식 (★ 발제 원문의 부분 점수 규칙이 아니라 팀 가정 — 원문이 다르면 여기를 고칠 것):
  신호 12     관측 순서 (signal.json) 의 자리마다 정답과 같으면 1 점
  횟수 9      R · Y · G 횟수마다 3 점
  바코드 4    코드가 같으면 4 점
  그룹 9      L1 ~ L3 그룹 (allocation.json) 이 정답 (바코드 정렬 규칙) 과 같으면 장소마다 3 점
  색·모양 18  장소마다 6 점: 마지막 verify 가 passed 면 6, 아니면 6 × (1 − 색 · 모양 · 빈 칸 오류 칸 / 그 장소 퍽 수)
  눈 30       장소마다 10 점: 마지막 verify 가 passed (칸 중심 10 mm · 개수 · 색 · 모양 모두)
  재검증 8    verify_ids passed 면 8, 아니면 8 × (읽은 ID 중 중복 · 기대 밖 아닌 것 / 12)
  복귀 10     마지막 "return" 이 inside 이고 3 초 동안 2 cm · 3° 안에서 정지, finish DONE
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import yaml

from .allocation import MissionError, count_signal, order_groups

WEIGHTS = {"신호": 12, "횟수": 9, "바코드": 4, "그룹": 9, "색·모양": 18, "눈": 30, "재검증": 8, "복귀": 10}
SHAPE_COLOR_ERRORS = (1, 5, 6)   # CheckDice MISSING · WRONG_COLOR · WRONG_SHAPE
STILL_M, STILL_DEG = 0.02, 3.0


def _find(ep, name):
    for p in (os.path.join(ep, "planner", name), os.path.join(ep, name)):
        if os.path.exists(p):
            return p
    return None


def _load_json(ep, name):
    p = _find(ep, name)
    return json.load(open(p, encoding="utf-8")) if p else None


def load_episode(ep):
    log = []
    p = os.path.join(ep, "mission_log.jsonl")
    if os.path.exists(p):
        log = [json.loads(line) for line in open(p, encoding="utf-8") if line.strip()]
    return dict(log=log, signal=_load_json(ep, "signal.json"), allocation=_load_json(ep, "allocation.json"))


def score(ep_data, problem):
    """-> [(항목, 점수, 배점, 근거)]"""
    out = []
    add = lambda k, v, why: out.append((k, round(v, 2), WEIGHTS[k], why))  # noqa: E731
    log, sig, alloc = ep_data["log"], ep_data["signal"], ep_data["allocation"]
    ans_seq = list(str(problem["signal"]).strip())
    ans_counts = count_signal(ans_seq)
    ans_groups = [g for g, _ in order_groups(ans_counts, problem["barcode"])]

    # 신호 · 횟수 · 바코드: 계획기 signal.json (관측값 그대로, 계획기가 받은 /perception/signal · barcode)
    if sig is None:
        for k in ("신호", "횟수", "바코드"):
            add(k, 0, "signal.json 없음 (plan 전에 멈춤)")
    else:
        seq = list(sig["signal"]["sequence"])
        hit = sum(a == b for a, b in zip(seq, ans_seq))
        add("신호", hit, f"{''.join(seq)} / 정답 {''.join(ans_seq)} ({hit}/12 자리)")
        cnt = sig["signal"]["counts"]
        ok = [g for g in ("R", "Y", "G") if cnt.get(g) == ans_counts[g]]
        add("횟수", 3 * len(ok), f"{cnt} / 정답 {ans_counts}")
        code = sig["barcode"]["code"]
        add("바코드", 4 if code == problem["barcode"] else 0, f"{code} / 정답 {problem['barcode']}")

    # 그룹: allocation.json 장소별 그룹
    places = ("L1", "L2", "L3")
    if alloc is None:
        add("그룹", 0, "allocation.json 없음")
        got = {}
    else:
        got = {L: alloc["groups"][L]["group"] for L in places if L in alloc["groups"]}
        ok = [L for L, g in zip(places, ans_groups) if got.get(L) == g]
        add("그룹", 3 * len(ok), f"{got} / 정답 {dict(zip(places, ans_groups))}")

    # 색·모양 · 눈: 장소마다 마지막 verify (재방문 재검증이 앞 결과를 덮음)
    last = {}
    for e in log:
        if e.get("kind") == "verify" and e.get("place") in places:
            last[e["place"]] = e
    cs, dice, why_cs, why_dice = 0.0, 0.0, [], []
    for L, g in zip(places, ans_groups):
        v = last.get(L)
        n = ans_counts[g]
        if v is None or not v.get("observed"):
            why_cs.append(f"{L} verify 없음")
            why_dice.append(f"{L} verify 없음")
            continue
        if v.get("passed"):
            cs += 6
            dice += 10
            why_cs.append(f"{L} 통과")
            why_dice.append(f"{L} 통과")
            continue
        bad = sum(c.get("error") in SHAPE_COLOR_ERRORS for c in v.get("cells", []))
        cs += 6 * max(0.0, 1 - bad / n)
        why_cs.append(f"{L} 오류 칸 {bad}/{n}")
        why_dice.append(f"{L} 실패 ({v.get('message', '')})")
    add("색·모양", cs, ", ".join(why_cs))
    add("눈", dice, ", ".join(why_dice))

    # 재검증: 마지막 verify_ids
    vids = [e for e in log if e.get("kind") == "verify_ids"]
    if not vids:
        add("재검증", 0, "verify_ids 없음 (verify.record_ids 꺼짐 또는 마지막 verify 전에 멈춤)")
    else:
        j = vids[-1]
        if j.get("passed"):
            add("재검증", 8, f"ID {j.get('read')} 개 읽음, 중복 · 누락 없음")
        else:
            wrong = set(j.get("duplicate", [])) | set(j.get("unexpected", []))
            good = len(set(range(12)) - set(j.get("missing", [])) - wrong)
            add("재검증", 8 * good / 12, f"맞은 ID {good}/12 (중복 {j.get('duplicate')}, 누락 {j.get('missing')}, 기대 밖 {j.get('unexpected')})")

    # 복귀: 마지막 return + finish DONE
    rets = [e for e in log if e.get("kind") == "return"]
    fin = next((e for e in reversed(log) if e.get("kind") == "finish"), None)
    done = fin is not None and fin.get("state") == "DONE"
    if not rets:
        add("복귀", 0, "return 기록 없음 (return_check 꺼짐 또는 복귀 전에 멈춤)")
    else:
        r = rets[-1]
        still = r.get("moved_m", 0.0) <= STILL_M and r.get("turned_deg", 0.0) <= STILL_DEG
        ok = bool(r.get("ok") and r.get("inside") and still and done)
        add("복귀", 10 if ok else 0,
            f"inside {r.get('inside')}, 여유 {r.get('min_margin_m')} m, 움직임 {r.get('moved_m')} m · {r.get('turned_deg')}°, "
            f"finish {fin.get('state') if fin else '없음'}")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="미션 에피소드 폴더 -> 채점 항목별 예상 점수")
    ap.add_argument("episode", help="<log_dir>/<YYMMDD_HHMM> (mission_log.jsonl 이 있는 폴더)")
    ap.add_argument("--problem", required=True, help="정답 문제 yaml (signal · barcode)")
    ap.add_argument("--json", default=None, help="결과를 이 파일에도 씀")
    a = ap.parse_args(argv)
    try:
        prob = yaml.safe_load(open(a.problem, encoding="utf-8"))
        rows = score(load_episode(a.episode), prob)
    except (MissionError, OSError, KeyError) as e:
        print(f"[오류] {e}", file=sys.stderr)
        return 2
    total = sum(r[1] for r in rows)
    print(f"{'항목':6s} {'점수':>6s} / 배점  근거")
    for k, v, w, why in rows:
        print(f"{k:6s} {v:6.1f} / {w:3d}  {why}")
    print(f"{'합계':6s} {total:6.1f} / 100   (★ 부분 점수 규칙은 팀 가정, score.py 머리말)")
    if a.json:
        json.dump(dict(episode=a.episode, total=round(total, 2), items=[dict(item=k, score=v, max=w, why=why) for k, v, w, why in rows]),
                  open(a.json, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
