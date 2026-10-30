"""넣기 정책 + 손 IK 내리기 확인 (창 없이): 학습된 넣기 정책이 퍽을 통 축 위에 맞추면 (퍽 바닥-축 ≤ 1.5 mm, 입구 위 12 mm 안, 직립)
그 뒤는 야코비안 IK 로 (집게 방향 유지) 퍽 바닥 중심을 통 축에 맞추며 1 mm / 스텝 내려 입구 아래 18 mm -> 집게 열기 0.5 s -> 집게 위로 4 cm 뺌.
통 안으로 '내리기' 가 물리적으로 되는지 (정책 보상 문제인지, 막힌 것인지) 가르기 위한 것. 결과 = 통별 퍽 깊이·직립·팔-몸체 힘.

    ./run_isaac.sh store_hybrid [환경 수]
"""

import argparse
import glob
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--policy", default=None)
parser.add_argument("--cats", default="AC")
parser.add_argument("--align_mm", type=float, default=2.5)
parser.add_argument("--up_min", type=float, default=0.985)
parser.add_argument("--depth", type=float, default=0.010, help="[m] 쥔 채 내리는 깊이 (퍽 바닥, 입구 아래). 261028 새 집게 10 mm (15 이상이면 손가락이 통 윗면에 닿아 끼임)")
parser.add_argument("--collect", default=None, help="넣기 정책 시작 상태 기록 -> 이 json (통 축 6 mm·입구 위 20 mm 안에서 쥔 상태, 3 스텝마다)")
parser.add_argument("--gui", action="store_true", help="창으로 보기 (run_isaac.sh store_hybrid_play)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = not args_cli.gui
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import waffle_isaac  # noqa: E402,F401
from isaaclab.utils.math import quat_apply  # noqa: E402
from waffle_isaac.store_env_cfg import WaffleStoreEnvCfg  # noqa: E402


def main():
    fixed = os.path.join(HERE, "policies", "store_latest.pt")  # 기본 = 지금 옮기기 모델 (261028 ST-N5)
    path = args_cli.policy or (fixed if os.path.exists(fixed) else sorted(glob.glob(os.path.join(HERE, "logs/rsl_rl/waffle_store/*/exported/policy.pt")))[-1])
    cfg = WaffleStoreEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.start_cats = tuple(args_cli.cats)
    env = gym.make("Waffle-Store-Direct", cfg=cfg)
    u = env.unwrapped
    pol = torch.jit.load(path, map_location=u.device).eval()
    env.reset()
    n, dev, r = u.num_envs, u.device, u.robot
    nb = r.num_base_dofs if hasattr(r, "num_base_dofs") else 6
    cols = torch.tensor([nb + j for j in u._arm], device=dev)
    phase = torch.zeros(n, dtype=torch.long, device=dev)  # 0 정책, 1 내리기, 2 열기, 3 빼기, 4 끝
    pt = torch.zeros(n, dtype=torch.long, device=dev)
    ok_cnt = torch.zeros(n, dtype=torch.long, device=dev)
    t_align = torch.full((n,), -1.0, device=dev)
    fmax = torch.zeros(n, device=dev)
    ffmax = torch.zeros(n, device=dev)
    fwhen = [""] * n
    dz_goal = torch.zeros(n, device=dev)
    tube = u.tube.clone()
    print("[Hybrid] 정책", path, "| 통", ["CPN"[i] for i in tube.tolist()], flush=True)

    def ik_step(dp_w, mask):
        """잡는 점 변위 dp (월드) + 자세 유지 -> 팔 관절 변화 (감쇠 최소제곱)."""
        J = r.data.body_link_jacobian_w.torch[:, u._a5]  # (N, 6, dof)
        J = J[:, :, cols]
        q5 = r.data.body_link_quat_w.torch[:, u._a5]
        rr = quat_apply(q5, u._off)  # a5 원점 -> 잡는 점
        Jl = J[:, 0:3] - torch.cross(rr.unsqueeze(-1).expand(-1, -1, 5), J[:, 3:6], dim=1)
        Jf = torch.cat((Jl, J[:, 3:6] * 0.05), dim=1)  # 자세 유지는 약하게
        err = torch.cat((dp_w, torch.zeros(n, 3, device=dev)), dim=1)
        lam = 1e-4
        JJt = Jf @ Jf.transpose(1, 2) + lam * torch.eye(6, device=dev)
        dq = (Jf.transpose(1, 2) @ torch.linalg.solve(JJt, err.unsqueeze(-1))).squeeze(-1)
        q = r.data.joint_pos.torch[:, u._arm]
        u.arm_target[mask] = torch.clamp(q + dq, u._lo, u._hi)[mask]

    steps = int(cfg.episode_length_s / u.step_dt) + 60
    rows = []
    for t in range(steps):
        obs = u._get_observations()
        with torch.inference_mode():
            a = pol(obs["policy"])
        u._pre_physics_step(a)
        s = u._state()
        clear = -s["depth"]
        good = (s["dxy"] < args_cli.align_mm / 1000) & (clear < 0.012) & (clear > -0.002) & (s["u"][:, 2] > args_cli.up_min) & s["held"]
        ok_cnt = torch.where(good & (phase == 0), ok_cnt + 1, torch.zeros_like(ok_cnt))
        start = (phase == 0) & (ok_cnt >= 3)
        phase[start], pt[start] = 1, 0
        t_align[start] = t * u.step_dt
        # 1 내리기: 퍽 바닥 xy -> 축 (0.5 배), z 1 mm / 스텝, 입구 아래 18 mm 까지
        m1 = phase == 1
        if m1.any():
            dp = torch.zeros(n, 3, device=dev)
            dp[:, :2] = (s["t"][:, :2] - s["pb"][:, :2]) * 0.5
            dp[:, 2] = -0.001
            ik_step(dp, m1)
            u.actions[m1, 5] = -1.0
            done1 = m1 & ((s["depth"] > args_cli.depth) | (pt > 60))
            phase[done1], pt[done1] = 2, 0
        m2 = phase == 2
        if m2.any():
            u.arm_target[m2] = r.data.joint_pos.torch[m2][:, u._arm]
            u.actions[m2, 5] = 1.0
            phase[m2 & (pt >= 15)] = 3
            pt[m2 & (pt >= 15)] = 0
        m3 = phase == 3
        if m3.any():
            dp = torch.zeros(n, 3, device=dev)
            dp[:, 2] = 0.002
            ik_step(dp, m3)
            u.actions[m3, 5] = 1.0
            phase[m3 & (pt >= 20)] = 4
        m4 = phase == 4
        if m4.any():
            u.arm_target[m4] = r.data.joint_pos.torch[m4][:, u._arm]
            u.actions[m4, 5] = 1.0
        pt += 1
        for _ in range(cfg.decimation):
            u._apply_action()
            u.scene.write_data_to_sim()
            u.sim.step(render=args_cli.gui)
            u.scene.update(u.physics_dt)
            fa, ff = u._force("arm_body_contact"), u._force("finger_body_contact")
            for i in torch.nonzero((ff > ffmax) & (ff > 1.0)).flatten().tolist():
                fwhen[i] = f"단계 {int(phase[i])} {ff[i].item():.1f} N"
            fmax = torch.maximum(fmax, fa)
            ffmax = torch.maximum(ffmax, ff)
        if args_cli.collect and t % 3 == 0:
            s = u._state()
            keep = s["held"] & (s["dxy"] < 0.006) & (-s["depth"] < 0.02) & (phase <= 1)
            jp = r.data.joint_pos.torch
            pq = u.puck.data.root_quat_w.torch
            for i in torch.nonzero(keep).flatten().tolist():
                rows.append({"tube": int(u.tube[i]), "phase": int(phase[i]), "depth": round(float(s["depth"][i]), 5), "dxy": round(float(s["dxy"][i]), 5),
                             "q": [round(float(v), 5) for v in jp[i, u._arm].tolist()], "finger": [float(jp[i, u._gl[0]]), float(jp[i, u._gr[0]])],
                             "puck_b": [round(float(v), 5) for v in s["pc"][i].tolist()], "puck_quat": [float(v) for v in pq[i].tolist()]})
        if t % 30 == 29:
            s = u._state()
            print(f"[Hybrid] t={(t + 1) * u.step_dt:.1f}s 단계 {phase.tolist()} 축 mm {[round(v * 1000, 1) for v in s['dxy'].tolist()]} 입구 아래 mm {[round(v * 1000) for v in s['depth'].tolist()]}"
                  f" 직립 {[round(v, 3) for v in s['u'][:, 2].tolist()]} 퍽-몸체 N {[round(v, 1) for v in u._force('puck_body_contact').tolist()]}", flush=True)
    s = u._state()
    for i in range(n):
        print(f"[Hybrid] env{i} 통 {'CPN'[int(tube[i])]}: 맞춤 {t_align[i].item():.1f}s, 단계 {int(phase[i])}, 퍽 입구 아래 {s['depth'][i].item() * 1000:.0f} mm, "
              f"축 {s['dxy'][i].item() * 1000:.1f} mm, 직립 {s['u'][i, 2].item():.3f}, 쥠 {int(s['held'][i])}, 팔(링크 2 ~ 5)-몸체 최대 {fmax[i].item():.1f} N, 손가락-몸체 최대 {ffmax[i].item():.1f} N ({fwhen[i]})", flush=True)
    if args_cli.collect:
        import json
        json.dump({"note": "넣기 시작 상태 (Isaac, ST3 맞춤 + 손 IK 내리기 중): 팔 관절, 손가락 (왼, 오), 퍽 중심 (base_link), 퍽 자세 (월드 = 루트 요 0, x,y,z,w), 통, 깊이 (+ = 입구 아래)",
                   "rows": rows}, open(args_cli.collect, "w"))
        print(f"[Hybrid] 시작 상태 {len(rows)} 개 -> {args_cli.collect}", flush=True)
    stored = (s["depth"] > 0.06) & (s["dxy"] < 0.006)
    print(f"[Hybrid] 통 안 (입구 아래 60 mm 넘게) {int(stored.sum())}/{n}, 맞춤까지 간 것 {int((t_align >= 0).sum())}/{n}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
