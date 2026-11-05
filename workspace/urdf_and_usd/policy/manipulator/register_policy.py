"""학습 체크포인트 (rsl_rl model_*.pt) -> 배포용 정책 등록. Isaac 시뮬레이터 없이 돎 (torch · rsl_rl · onnx 만, Isaac Lab venv 파이썬).

  1. 체크포인트의 actor (관측 정규화 + MLP) 를 rsl_rl 내보내기와 같은 모양으로 TorchScript (.pt) · ONNX (.onnx, 한 파일) 로 내보냄
     -> policies/<이름>.pt · policies/<이름>.onnx. 두 파일 출력이 같은지 (onnxruntime 있으면) 확인.
  2. --latest 면 policies/<과제>_latest.pt 를 이것으로 (평가 스크립트 기본값). 원래 latest 가 등록부에 없으면 <과제>_prev_<날짜>.pt 로 보관.
  3. --ros 면 ONNX 를 ROS robot_policy/config/models/<ros 이름>.onnx 로 복사 (policies.yaml 항목은 손으로, obs_dim 36 · action tcp_delta(4)).
  4. policies/registry.json 에 한 줄 (이름 · 과제 · 원본 체크포인트 · 반복 · sha256 · 평가 결과 파일 · 메모). 학습기록_자동.md 에도 한 줄.

    ./run_rl.sh register <체크포인트 | 실행 폴더 (마지막 model_*)> <이름> [--latest] [--ros toppick_tilt] [--eval toppick_eval.json ...] [--note "..."]
    예) ./run_rl.sh register logs/rsl_rl/waffle_toppick/2026-11-04_20-46-45/model_14999.pt toppick_TP3 --latest --ros toppick_tilt
"""
from __future__ import annotations

import argparse
import datetime as _dt
import glob
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys

import torch
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
POL = os.path.join(HERE, "policies")
REG = os.path.join(POL, "registry.json")
ROS_MODELS = os.path.normpath(os.path.join(HERE, "..", "..", "..", "ros2_ws", "src", "robot_policy", "config", "models"))
TASK_OF = {"waffle_toppick": ("Waffle-TopPick-Direct", "toppick"), "waffle_sidepick": ("Waffle-SidePick-Direct", "sidepick"),
           "waffle_store": ("Waffle-Store-Direct", "store"), "waffle_insert": ("Waffle-Insert-Direct", "insert")}


def resolve_ckpt(p):
    p = os.path.abspath(p)
    if os.path.isdir(p):
        ms = sorted(glob.glob(os.path.join(p, "model_*.pt")), key=lambda f: int(re.findall(r"model_(\d+)\.pt", f)[0]))
        if not ms:
            sys.exit(f"체크포인트 없음: {p}")
        return ms[-1]
    return p


def build_actor(ckpt):
    """체크포인트 actor_state_dict -> rsl_rl MLPModel (분포 없이 = 결정적 출력 = 평균). 숨은 층 · 활성 함수는 params/agent.yaml."""
    from tensordict import TensorDict
    from rsl_rl.models import MLPModel

    sd = torch.load(ckpt, map_location="cpu", weights_only=False)
    a = sd["actor_state_dict"]
    lin = sorted({int(k.split(".")[1]) for k in a if k.startswith("mlp.") and k.endswith(".weight")})
    obs_dim, act_dim = a[f"mlp.{lin[0]}.weight"].shape[1], a[f"mlp.{lin[-1]}.weight"].shape[0]
    hidden = [a[f"mlp.{i}.weight"].shape[0] for i in lin[:-1]]
    act, cfg_path = "elu", os.path.join(os.path.dirname(ckpt), "params", "agent.yaml")
    if os.path.isfile(cfg_path):
        ag = yaml.unsafe_load(open(cfg_path, encoding="utf-8"))
        act = (ag.get("actor") or ag.get("policy") or {}).get("activation", act)
    norm = any(k.startswith("obs_normalizer.") for k in a)
    m = MLPModel(TensorDict({"policy": torch.zeros(1, obs_dim)}, batch_size=[1]), {"actor": ["policy"]}, "actor", act_dim,
                 hidden_dims=hidden, activation=act, obs_normalization=norm, distribution_cfg=None)
    missing, unexpected = m.load_state_dict({k: v for k, v in a.items() if not k.startswith("distribution.")}, strict=False)
    assert not missing and not unexpected, (missing, unexpected)
    return m.eval(), sd.get("iter"), obs_dim, act_dim, hidden, act


def export(m, obs_dim, pt_path, onnx_path):
    jit = torch.jit.script(m.as_jit().to("cpu").eval())
    jit.save(pt_path)
    ox = m.as_onnx(verbose=False).to("cpu").eval()
    tmp = onnx_path + ".tmp"
    torch.onnx.export(ox, ox.get_dummy_inputs(), tmp, export_params=True, opset_version=18,
                      input_names=ox.input_names, output_names=ox.output_names)
    import onnx  # 작은 모델: 바깥 데이터 파일 (.onnx.data) 없이 한 파일로 (ROS onnx_runner 가 한 파일만 읽게)
    onnx.save_model(onnx.load(tmp, load_external_data=True), onnx_path, save_as_external_data=False)
    for f in (tmp, tmp + ".data"):
        if os.path.exists(f):
            os.remove(f)
    x = torch.randn(64, obs_dim) * 2.0
    with torch.no_grad():
        ref = jit(x)
    diff = None
    try:
        import onnxruntime as ort
        s = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
        diff = 0.0
        for k in range(8):  # 배치 1 로 내보냈을 수 있어 한 줄씩 (ROS 도 한 줄씩)
            out = torch.from_numpy(s.run(None, {s.get_inputs()[0].name: x[k:k + 1].numpy()})[0])
            diff = max(diff, float((out - ref[k:k + 1]).abs().max()))
    except ImportError:
        pass
    return diff


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()[:16]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ckpt", help="model_*.pt 또는 실행 폴더 (마지막 model_*)")
    ap.add_argument("name", help="등록 이름 (예 toppick_TP3) -> policies/<이름>.pt")
    ap.add_argument("--latest", action="store_true", help="policies/<과제>_latest.pt 로도")
    ap.add_argument("--ros", default=None, help="ROS robot_policy/config/models/<이 이름>.onnx 로 복사 (예 toppick_tilt)")
    ap.add_argument("--eval", nargs="*", default=[], help="평가 결과 JSON (toppick_eval · chain_pallet_eval) - 등록부에 요약을 붙임")
    ap.add_argument("--note", default="")
    a = ap.parse_args()

    ckpt = resolve_ckpt(a.ckpt)
    exp = os.path.basename(os.path.dirname(os.path.dirname(ckpt)))
    task, short = TASK_OF.get(exp, (exp, exp.replace("waffle_", "")))
    m, it, od, ad, hidden, act = build_actor(ckpt)
    os.makedirs(POL, exist_ok=True)
    pt, ox = os.path.join(POL, a.name + ".pt"), os.path.join(POL, a.name + ".onnx")
    diff = export(m, od, pt, ox)
    print(f"[Register] {ckpt} (반복 {it}) -> {pt}, {ox} | 관측 {od} 행동 {ad} 숨은 층 {hidden} {act} | ONNX-TorchScript 최대 차 {diff}")
    if diff is not None and diff > 1e-4:
        sys.exit("ONNX 와 TorchScript 출력이 다름 -> 등록 중지")

    reg = json.load(open(REG, encoding="utf-8")) if os.path.isfile(REG) else {"note": "등록한 정책 목록 (register_policy.py 가 씀). latest = 평가 스크립트 기본값", "policies": []}
    today = _dt.datetime.now().strftime("%Y-%m-%d %H:%M")
    if a.latest:
        lat = os.path.join(POL, f"{short}_latest.pt")
        known = {p.get("sha256") for p in reg["policies"]}
        if os.path.isfile(lat) and sha(lat) not in known:  # 등록부에 없던 옛 latest (예 TP2) 를 보관
            keep = os.path.join(POL, f"{short}_prev_{_dt.date.today():%Y%m%d}.pt")
            shutil.copy2(lat, keep)
            reg["policies"].append(dict(name=os.path.basename(keep)[:-3], task=task, file=os.path.relpath(keep, HERE), sha256=sha(keep),
                                        registered=today, note="register_policy.py 이전 latest 를 보관 (출처는 정책_학습기록.md)"))
            print(f"[Register] 이전 latest 보관 -> {keep}")
        shutil.copy2(pt, lat)
    ros = None
    if a.ros:
        os.makedirs(ROS_MODELS, exist_ok=True)
        ros = os.path.join(ROS_MODELS, a.ros + ".onnx")
        shutil.copy2(ox, ros)
        print(f"[Register] ROS 모델 -> {ros} (robot_policy/config/policies.yaml 의 '{a.ros}' 항목 obs_dim {od} 확인)")
    evals = []
    for e in a.eval:
        s = json.load(open(e, encoding="utf-8")).get("summary", {})
        evals.append(dict(file=os.path.relpath(os.path.abspath(e), HERE), **{k: s[k] for k in ("seed", "layout", "dr", "num_envs", "success", "chain", "pick", "place_given_pick") if k in s}))
    try:
        git = subprocess.run(["git", "-C", HERE, "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    except OSError:
        git = ""
    ent = dict(name=a.name, task=task, file=os.path.relpath(pt, HERE), onnx=os.path.relpath(ox, HERE), sha256=sha(pt),
               source=os.path.relpath(ckpt, HERE), iteration=it, obs_dim=od, action_dim=ad, hidden=hidden, activation=act,
               latest=bool(a.latest), ros_model=os.path.relpath(ros, HERE) if ros else None, evals=evals, note=a.note, git=git, registered=today)
    reg["policies"] = [p for p in reg["policies"] if p["name"] != a.name] + [ent]
    for p in reg["policies"]:
        if a.latest and p["task"] == task and p["name"] != a.name:
            p["latest"] = False
    json.dump(reg, open(REG, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"[Register] 등록부 {REG}")
    subprocess.run([sys.executable, os.path.join(HERE, "record_md.py"), "register", a.name], check=False)


if __name__ == "__main__":
    main()
