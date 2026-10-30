"""진단: 정렬·집기 환경 리셋 직후 (행동 0) 잡는 점·퍽·목표 위치 (base 좌표), 집게 방향."""
import os
import sys

from isaaclab.app import AppLauncher
import argparse

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
simulation_app = AppLauncher(args_cli).app
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import waffle_isaac  # noqa: E402,F401
from waffle_isaac.sidepick_env_cfg import WaffleSidePickEnvCfg  # noqa: E402


def main():
    cfg = WaffleSidePickEnvCfg()
    cfg.scene.num_envs = 6
    cfg.ready_noise = 0.0
    cfg.near_start_prob = 1.0
    env = gym.make("Waffle-SidePick-Direct", cfg=cfg)
    u = env.unwrapped
    env.reset()
    a = torch.zeros(6, 6, device=u.device)
    a[:, 5] = 1.0
    qs = torch.tensor([[0.103, 1.196, 0.013, -1.209, 0.0], [-0.088, 1.15, 0.238, -1.387, 0.0], [-0.219, 1.144, 0.281, -1.425, 0.0]], device=u.device)
    for k in range(80):
        env.step(a)
        if k >= 20:  # 20 스텝 뒤부터 IK 자세로 관절 목표를 천천히 옮김 (env0~2), 직접 덮어씀
            f = min(1.0, (k - 19) / 40)
            q0 = torch.tensor([[0.0, 1.16, 0.53, -1.69, 0.0]], device=u.device)
            u.arm_target[:3] = q0 + (qs - q0) * f
        if k in (0, 19, 50, 79):
            g = u.to_base(u.grasp_w()); t = u.to_base(u.grasp_target_w()); p = u.to_base(u.puck_pos_w())
            ax, ay, az = u.a5_axes_w()
            ee = u.to_base(u.robot.data.body_link_pos_w.torch[:, u._a5])
            print(f"[R] step {k}: EE {[[round(v,3) for v in r] for r in ee[:3].tolist()]}", flush=True)
            print(f"[R]   grasp {[[round(v,3) for v in r] for r in g[:3].tolist()]} target {[[round(v,3) for v in r] for r in t[:3].tolist()]} puck {[[round(v,3) for v in r] for r in p[:3].tolist()]}", flush=True)
            print(f"[R]   approach(-az) {[[round(v,2) for v in r] for r in (-az)[:3].tolist()]} close(ax) {[[round(v,2) for v in r] for r in ax[:3].tolist()]} q {[[round(v,2) for v in r] for r in u.robot.data.joint_pos.torch[:3, u._arm].tolist()]}", flush=True)
            print(f"[R]   잡는 점-목표 mm {[round(v * 1000) for v in torch.linalg.norm(g - t, dim=1)[:3].tolist()]} 팔-상자 접촉 {u._contact('arm_box_contact')[:3].int().tolist()} 퍽 {[[round(v,3) for v in r] for r in p[:3].tolist()]}", flush=True)
            print(f"[R]   root {[[round(v,3) for v in r] for r in u.robot.data.root_link_pos_w.torch[:3].tolist()]} env_origin {[[round(v,2) for v in r] for r in u.scene.env_origins[:3].tolist()]}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
