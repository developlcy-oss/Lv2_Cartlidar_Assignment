"""정렬·집기 진단 (창 없이): 학습된 정책을 돌리며 1 s 마다 잡는 점-목표 거리 (수평·높이), 집게 수평도, 손가락, 팔 관절 (한계), 접촉 출력.

    ./run_isaac.sh sidepick_diag [환경 수]
"""

import argparse
import glob
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=8)
parser.add_argument("--policy", default=None)
parser.add_argument("--near", type=float, default=0.0, help="역방향 커리큘럼 (퍽을 손가락 사이에 두고 시작) 비율. 기본 0 = 실제 과제만")
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
from waffle_isaac.sidepick_env_cfg import WaffleSidePickEnvCfg  # noqa: E402


def main():
    path = args_cli.policy or sorted(glob.glob(os.path.join(HERE, "logs/rsl_rl/waffle_sidepick/*/exported/policy.pt")))[-1]
    cfg = WaffleSidePickEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    if hasattr(cfg, "near_start_prob"):
        cfg.near_start_prob = args_cli.near
    env = gym.make("Waffle-SidePick-Direct", cfg=cfg)
    u = env.unwrapped
    pol = torch.jit.load(path, map_location=u.device).eval()
    obs, _ = env.reset()
    print("[Diag] 정책", path, flush=True)
    for t in range(int(cfg.episode_length_s / u.step_dt) - 2):
        with torch.inference_mode():
            a = pol(obs["policy"])
        obs, *_ = env.step(a)
        if t % 30 == 29:
            g = u.grasp_w()
            to_t = u.grasp_target_w() - g
            ax, ay, az = u.a5_axes_w()
            appr = -az
            tb = u.to_base(u.puck_pos_w())
            print(f"[Diag] t={(t + 1) * u.step_dt:.1f}s 퍽 (base) {[[round(v, 3) for v in r] for r in tb[:4].tolist()]}", flush=True)
            print(f"[Diag]   목표까지 수평 mm {[round(v * 1000) for v in to_t[:8, :2].norm(dim=1).tolist()]} 높이 mm {[round(v * 1000) for v in to_t[:8, 2].tolist()]}"
                  f" | 집게 방향 z {[round(v, 2) for v in appr[:8, 2].tolist()]} | 집게가 퍽 향함 {[round(v, 2) for v in ((appr[:8, :2] * torch.nn.functional.normalize(to_t[:8, :2], dim=1)).sum(1)).tolist()]}", flush=True)
            print(f"[Diag]   손가락 {[round(v, 4) for v in u.robot.data.joint_pos.torch[:8, u._gl[0]].tolist()]} 집게 행동 {[round(v, 1) for v in a[:8, -1].tolist()]}"
                  f" | 팔-상자 접촉 {u._contact('arm_box_contact')[:8].int().tolist()} | 퍽 직립 {[round(v, 2) for v in u.puck_up()[:8].tolist()]}", flush=True)
            q = u.robot.data.joint_pos.torch[:, u._arm]
            print(f"[Diag]   팔 관절 env0~2 {[[round(v, 2) for v in r] for r in q[:3].tolist()]} 한계 {[round(v, 2) for v in u._lo.tolist()]} ~ {[round(v, 2) for v in u._hi.tolist()]}", flush=True)
    lift = u.lift()
    held = (lift > 0.03) & (torch.linalg.norm(u.grasp_target_w() - u.grasp_w(), dim=1) < 0.03) & (u.puck_up() > 0.95)
    print(f"[Diag] 끝: 쥐고 3 cm 넘게 세워 든 대 {int(held.sum())}/{u.num_envs} ({held.float().mean().item() * 100:.0f} %), 넘어진 퍽 {int((u.puck_up() < 0.7).sum())}, "
          f"들기 mm {[round(v * 1000) for v in lift[:16].tolist()]}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
