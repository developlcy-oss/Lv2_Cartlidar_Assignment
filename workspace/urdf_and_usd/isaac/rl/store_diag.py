"""스토리지 넣기 진단 (창 없이): 학습된 정책을 돌리며 1 s 마다 목표 통, 퍽 바닥-통 축 거리, 퍽 바닥 - 입구 높이, 퍽 직립, 쥠, 손가락,
팔-몸체 접촉 힘, 퍽-몸체 접촉 힘, 1 번 관절 각 출력. 끝에 통별 성공 (통 안에 놓고 뺌) 집계.

    ./run_isaac.sh store_diag [환경 수]
"""

import argparse
import glob
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=8)
parser.add_argument("--policy", default=None)
parser.add_argument("--cats", default="A", help="시작 종류 (A 집은 직후, B 가는 도중, C 통 위). 기본 A = 집은 직후부터 끝까지")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import waffle_isaac  # noqa: E402,F401
from waffle_isaac.store_env_cfg import WaffleStoreEnvCfg  # noqa: E402


def main():
    path = args_cli.policy or (os.path.join(HERE, "policies", "store_latest.pt") if os.path.exists(os.path.join(HERE, "policies", "store_latest.pt")) else sorted(glob.glob(os.path.join(HERE, "logs/rsl_rl/waffle_store/*/exported/policy.pt")))[-1])
    cfg = WaffleStoreEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.start_cats = tuple(args_cli.cats)
    env = gym.make("Waffle-Store-Direct", cfg=cfg)
    u = env.unwrapped
    pol = torch.jit.load(path, map_location=u.device).eval()
    obs, _ = env.reset()
    n = u.num_envs
    done_any = torch.zeros(n, dtype=torch.bool, device=u.device)
    ok = torch.zeros(n, dtype=torch.bool, device=u.device)
    ins = torch.zeros(n, dtype=torch.bool, device=u.device)
    fmax = torch.zeros(n, device=u.device)
    tube0 = u.tube.clone()
    print("[Diag] 정책", path, "| 통", ["CPN"[i] for i in tube0.tolist()], flush=True)
    for t in range(int(cfg.episode_length_s / u.step_dt) - 2):
        with torch.inference_mode():
            a = pol(obs["policy"])
        obs, _, term, trunc, _ = env.step(a)
        live = ~done_any
        ok |= u.succeeded & live
        ins |= u.ever_inserted & live
        fmax = torch.maximum(fmax, u._force("arm_body_contact") * live.float())
        done_any |= term
        if t % 30 == 29:
            s = u._state()
            print(f"[Diag] t={(t + 1) * u.step_dt:.1f}s 축 거리 mm {[round(v * 1000) for v in s['dxy'].tolist()]} | 바닥-입구 mm {[round(-v * 1000) for v in s['depth'].tolist()]}"
                  f" | 직립 {[round(v, 2) for v in s['u'][:, 2].tolist()]}", flush=True)
            close_w, neg_appr = u.a5_axes_w()
            appr_b, close_b = u.vec_bl(-neg_appr), u.vec_bl(close_w)
            tilt = u.base_tilt() * 57.3
            print(f"[Diag]   기울기 분해 deg: 퍽 {[round(float(v), 1) for v in torch.rad2deg(torch.acos(s['u'][:, 2].clamp(-1, 1)))[:8].tolist()]}"
                  f" | 집게 피치 {[round(float(v), 1) for v in torch.rad2deg(torch.asin(appr_b[:, 2].clamp(-1, 1)))[:8].tolist()]}"
                  f" | 집게 롤 (닫는 축 z) {[round(float(v), 1) for v in torch.rad2deg(torch.asin(close_b[:, 2].clamp(-1, 1)))[:8].tolist()]}"
                  f" | 차체 {[round(float(v), 1) for v in tilt[:8].tolist()]} | 관절5 {[round(float(v), 2) for v in u.robot.data.joint_pos.torch[:8, u._arm[4]].tolist()]}", flush=True)
            print(f"[Diag]   잡는 점 xy (base) {[[round(v, 3) for v in r] for r in s['gb'][:8, :2].tolist()]} | 목표 통 축 {[[round(v, 3) for v in r] for r in s['t'][:8, :2].tolist()]}"
                  f" | 퍽 바닥 xy {[[round(v, 3) for v in r] for r in s['pb'][:8, :2].tolist()]}", flush=True)
            print(f"[Diag]   쥠 {s['held'].int().tolist()} 손가락 {[round(v, 4) for v in s['fing'].tolist()]} | 팔-몸체 N {[round(v, 1) for v in u._force('arm_body_contact').tolist()]}"
                  f" 퍽-몸체 N {[round(v, 1) for v in u._force('puck_body_contact').tolist()]} | 1 번 관절 {[round(v, 2) for v in u.robot.data.joint_pos.torch[:, u._arm[0]].tolist()]}"
                  f" | 중간 종료 {done_any.int().tolist()}", flush=True)
    for i, nm in enumerate("CPN"):
        msk = tube0 == i
        if msk.any():
            print(f"[Diag] {nm} 통: 넣음 {int(ins[msk].sum())}/{int(msk.sum())}, 성공 (놓고 뺌) {int(ok[msk].sum())}/{int(msk.sum())}", flush=True)
    print(f"[Diag] 팔-몸체 최대 힘 N {[round(v, 1) for v in fmax.tolist()]}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
