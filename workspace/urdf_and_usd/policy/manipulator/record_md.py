"""학습 · 평가 · 등록 기록을 학습기록_자동.md 에 표 한 줄씩 자동으로 붙임 (손으로 쓰는 정책_학습기록.md 는 건드리지 않음).

    python record_md.py train <실행 폴더 | 실험 이름 (가장 최근 실행)>   학습 1 회차: 반복 · 대 수 · 걸린 시간 · 마지막 10 % 반복의 지표 평균 · 경고
    python record_md.py toppick <toppick_eval JSON>                      집기 평가 (toppick_eval.py --out)
    python record_md.py chain <chain_pallet_eval JSON>                   집기 -> 칸 이어 평가 (칸별 성공률 · 손 안 어긋남)
    python record_md.py register <등록 이름>                             정책 등록 (policies/registry.json)
run_rl.sh 의 toppick · toppick_eval · chain_eval · register 가 끝나면 알아서 부름. tensorboard 는 Isaac Lab venv 에 있음 (없으면 지표 칸만 비움).
같은 실행 폴더 · 결과 파일을 다시 기록하면 그 줄을 바꿔 씀 (줄 끝 숨은 표시 <!-- key -->).
"""
from __future__ import annotations

import glob
import json
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
MD = os.path.join(HERE, "학습기록_자동.md")
SECTIONS = {
    "train": ("## 학습 회차", "| 기록 | 실행 폴더 | 과제 | 대 수 | 반복 | 이어받음 | 걸린 시간 | 성공률 (마지막 10 %) | 배치별 A · B · C · 무작위 | 들어 올림 | 옆 퍽 흔듦 | 평균 보상 | 행동 표준편차 (처음 → 끝) | 경고 |"),
    "toppick": ("## 집기 평가 (toppick_eval)", "| 기록 | 정책 | 시드 | 배치 | 무작위화 | 대 수 | 성공 | 배치별 rand · A · B · C | 상자 · 원기둥 | 좌표 고정 · 따라감 | 넘어짐 | 옆 퍽 | 결과 파일 |"),
    "chain": ("## 집기 → 칸 이어 평가 (chain_pallet_eval)", "| 기록 | 정책 | 시드 | 배치 | 대 수 | 이어서 성공 | 집기 | 집은 것 중 넣기 | 실패 단계 (집기 · 넘겨받기 · 운반 · 안 앉음) | 칸별 이어서 성공 (열:칸) | 손 안 어긋남 넘겨받을 때 닫는 축 · 집게 축 mm (중앙값/p95) | 어긋남별 넣기 <1 · 1-3 · ≥3 mm | 결과 파일 |"),
    "register": ("## 정책 등록 (register_policy)", "| 기록 | 이름 | 과제 | 원본 체크포인트 | 반복 | sha256 | latest | ROS 모델 | 평가 | 메모 |"),
}
HEAD = """# 정책 학습 · 평가 자동 기록

`record_md.py` 가 쓰는 파일 (직접 고치지 말 것, 같은 회차를 다시 기록하면 그 줄이 바뀜). 설정 · 원인 분석 · 결정은 손으로 [정책_학습기록.md](정책_학습기록.md) 에.
- 학습 지표는 마지막 10 % 반복의 평균 (학습 중 지표 = 확률적 행동). 평가 (결정적 행동) 와 섞어 읽지 말 것.
- 체크포인트 고르기 시드 1, 보고 시드 2 (#71). 등록 목록 원본은 `policies/registry.json`.
"""


def _cell(v):
    return "-" if v is None or v == "" else str(v).replace("|", "/").replace("\n", " ")


def upsert(kind, cells, key):
    """kind 절의 표에 한 줄 넣기 (key 가 같은 줄이 있으면 바꿈)."""
    title, header = SECTIONS[kind]
    text = open(MD, encoding="utf-8").read() if os.path.isfile(MD) else HEAD
    row = "| " + " | ".join(_cell(c) for c in cells) + f" | <!-- {key} -->"
    lines = text.rstrip("\n").split("\n")
    if title not in lines:
        lines += ["", title, "", header, "|" + "---|" * (header.count("|") - 1)]
    i = lines.index(title)
    j = i + 1
    while j < len(lines) and not lines[j].startswith("## "):
        j += 1
    body = lines[i:j]
    hit = [k for k, ln in enumerate(body) if ln.endswith(f"<!-- {key} -->")]
    if hit:
        body[hit[0]] = row
    else:
        last = max(k for k, ln in enumerate(body) if ln.startswith("|"))
        body.insert(last + 1, row)
    lines[i:j] = body
    open(MD, "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print(f"[record_md] {os.path.basename(MD)} · {title} · {key}")


def now():
    return time.strftime("%Y-%m-%d %H:%M")


def pct(r):
    return "-" if not r or r.get("n") in (0, None) else f"{r['ok']}/{r['n']} ({r['pct']:.0f} %)"


# ---------------------------------------------------------------- 학습
def train(arg):
    run = os.path.abspath(arg)
    if not os.path.isdir(run) or not glob.glob(os.path.join(run, "events*")):
        runs = sorted(glob.glob(os.path.join(HERE, "logs", "rsl_rl", arg, "*/")))
        if not runs:
            sys.exit(f"실행 폴더 없음: {arg}")
        run = runs[-1].rstrip("/")
    import yaml
    ag = yaml.unsafe_load(open(os.path.join(run, "params", "agent.yaml"), encoding="utf-8"))
    ev = yaml.unsafe_load(open(os.path.join(run, "params", "env.yaml"), encoding="utf-8"))
    task = json.load(open(os.path.join(run, "run.json"))).get("task", "-") if os.path.isfile(os.path.join(run, "run.json")) else "-"
    ms = sorted(int(m) for m in re.findall(r"model_(\d+)\.pt", " ".join(os.listdir(run))))
    resume = f"{ag.get('load_run')}/{ag.get('load_checkpoint')}" if ag.get("resume") else "처음부터"
    m = dict(sr=None, by=None, lift=None, nb=None, rew=None, std=None, dur=None)
    warn = []
    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
        ea = EventAccumulator(sorted(glob.glob(os.path.join(run, "events*")))[-1], size_guidance={"scalars": 0})
        ea.Reload()
        tags = set(ea.Tags()["scalars"])

        def tail(tag, frac=0.1):
            if tag not in tags:
                return None
            s = ea.Scalars(tag)
            last = s[-1].step
            v = [x.value for x in s if x.step >= last * (1 - frac)]
            return sum(v) / len(v)

        def f(v, p=True):
            return "-" if v is None else (f"{v * 100:.0f} %" if p else f"{v:.0f}")
        m["sr"] = f(tail("Metrics/success_rate"))
        m["by"] = " · ".join(f(tail(f"Metrics/success_{k}")) for k in ("A", "B", "C", "rand"))
        m["lift"] = f(tail("Metrics/lifted_ratio"))
        m["nb"] = f(tail("Metrics/neighbor_disturbed"))
        m["rew"] = f(tail("Train/mean_reward"), False)
        if "Policy/mean_std" in tags:
            s = ea.Scalars("Policy/mean_std")
            s0, s1 = s[0].value, tail("Policy/mean_std")
            m["std"] = f"{s0:.2f} → {s1:.2f}"
            mid = min(s[len(s) // 2:], key=lambda x: x.value).value
            if s1 > 1.0 and s1 > 2 * mid:
                warn.append(f"표준편차가 뒤 절반에서 {mid:.2f} → {s1:.2f} 로 커짐 (탐색 폭주, 마지막 체크포인트가 최선 아닐 수 있음)")
        if "Metrics/success_rate" in tags:
            s = ea.Scalars("Metrics/success_rate")
            n = len(s)
            best_k = max(range(10), key=lambda k: sum(x.value for x in s[k * n // 10:(k + 1) * n // 10]) / max(1, n // 10))
            if best_k < 9:
                lo, hi = s[best_k * n // 10].step, s[min(n - 1, (best_k + 1) * n // 10 - 1)].step
                warn.append(f"성공률 최고 구간 = 반복 {lo} ~ {hi} (마지막 아님) -> 그 근처 체크포인트도 평가")
        wall = [x.wall_time for x in ea.Scalars("Train/mean_reward")] if "Train/mean_reward" in tags else []
        if wall:
            h = (wall[-1] - wall[0]) / 3600
            m["dur"] = f"{h:.1f} h"
    except ImportError:
        warn.append("tensorboard 없음 (Isaac Lab venv 파이썬으로 실행)")
    cells = [now(), f"`{os.path.relpath(run, HERE)}`", task, ev.get("scene", {}).get("num_envs"), f"{ms[-1] if ms else '-'} / {ag.get('max_iterations')}",
             resume, m["dur"], m["sr"], m["by"], m["lift"], m["nb"], m["rew"], m["std"], "; ".join(warn)]
    upsert("train", cells, "train:" + os.path.relpath(run, HERE))


# ---------------------------------------------------------------- 평가
def toppick(path):
    d = json.load(open(path, encoding="utf-8"))
    s = d["summary"]
    g = d.get("groups", {})
    cells = [now(), f"`{s.get('policy')}`", s.get("seed"), s.get("layout"), s.get("dr"), s.get("num_envs"), pct(s.get("success")),
             " · ".join(pct(g.get(k)) for k in ("rand", "A", "B", "C")), f"{pct(g.get('box'))} · {pct(g.get('cyl'))}",
             f"{pct(g.get('latched'))} · {pct(g.get('follow'))}", s.get("toppled"), s.get("neighbor_bad"), f"`{os.path.relpath(os.path.abspath(path), HERE)}`"]
    upsert("toppick", cells, f"toppick:{s.get('policy')}:{s.get('seed')}:{s.get('layout')}:{s.get('dr')}:{s.get('num_envs')}")


def chain(path):
    d = json.load(open(path, encoding="utf-8"))
    s, ps = d["summary"], d["per_slot"]
    st = s["stages"]
    h = s["in_hand"]["handover"]

    def mp(c):
        return "-" if not h.get(c) else f"{h[c]['median']}/{h[c]['p95']}"
    slots = " ".join(f"{k} {v['chain']['ok']}/{v['chain']['n']}" for k, v in ps.items())
    bh = s["seated_by_handover_offset"]
    cells = [now(), f"`{s['policy']}`", s["seed"], s["layout"], s["num_envs"], pct(s["chain"]), pct(s["pick"]), pct(s["place_given_pick"]),
             f"{st['pick_fail']} · {st['transit_drop']} · {st['carry_slip']} · {st['not_seated']}", slots, f"{mp('close_mm')} · {mp('appr_mm')}",
             " · ".join(pct(bh.get(k)) for k in ("<1mm", "1-3mm", ">=3mm")), f"`{os.path.relpath(os.path.abspath(path), HERE)}`"]
    upsert("chain", cells, f"chain:{s['policy']}:{s['seed']}:{s['layout']}:{s['num_envs']}")


def register(name):
    reg = json.load(open(os.path.join(HERE, "policies", "registry.json"), encoding="utf-8"))
    p = next(x for x in reg["policies"] if x["name"] == name)
    ev = "; ".join(f"{os.path.basename(e['file'])} " + (pct(e.get("chain")) if e.get("chain") else pct(e.get("success"))) for e in p.get("evals", []))
    cells = [p["registered"], f"`{p['file']}`", p["task"], f"`{p.get('source')}`", p.get("iteration"), p["sha256"], "예" if p.get("latest") else "",
             f"`{p['ros_model']}`" if p.get("ros_model") else "", ev, p.get("note")]
    upsert("register", cells, f"register:{name}")


if __name__ == "__main__":
    if len(sys.argv) < 3 or sys.argv[1] not in ("train", "toppick", "chain", "register"):
        sys.exit(__doc__)
    {"train": train, "toppick": toppick, "chain": chain, "register": register}[sys.argv[1]](sys.argv[2])
