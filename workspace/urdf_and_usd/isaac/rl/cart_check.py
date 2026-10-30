"""진단: 정렬·집기 직교 행동 (N7) 이 되는지 손 제어로 - 잡는 점을 목표로 (5 mm / 스텝) -> 닫기 -> 5 cm 들기. 실제 과제 배치 (커리큘럼 0)."""
import argparse
import os
import sys

from isaaclab.app import AppLauncher

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
    cfg.scene.num_envs = 32
    cfg.near_start_prob = 0.0
    cfg.box_shape_prob = 0.0
    env = gym.make("Waffle-SidePick-Direct", cfg=cfg)
    u = env.unwrapped
    env.reset()
    n = u.num_envs
    for k in range(170):
        g, t = u.to_base(u.grasp_w()), u.to_base(u.grasp_target_w())
        if k < 30:  # 앞 6 cm·같은 높이로
            goal = t.clone(); goal[:, 0] -= 0.0; ax = (t - g)[:, :2]; 
            d = t - g
            d[:, :2] = d[:, :2] - torch.nn.functional.normalize(d[:, :2], dim=1) * 0.04 * (torch.linalg.norm(d[:, :2], dim=1, keepdim=True) > 0.04)
            grip = 1.0
        elif k < 70:
            d = t - g; grip = 1.0
        elif k < 100:
            d = torch.zeros_like(g); grip = -1.0
        else:
            d = torch.zeros_like(g); d[:, 2] = 0.003; grip = -1.0
        a = torch.cat((torch.clamp(d / cfg.cart_scale, -1, 1), torch.full((n, 1), grip, device=u.device)), dim=1)
        env.step(a)
        if k in (29, 69, 99, 169):
            dist = torch.linalg.norm(u.grasp_target_w() - u.grasp_w(), dim=1)
            print(f"[C] 스텝 {k}: 잡는 점-목표 mm 중앙 {dist.median().item() * 1000:.0f} (최대 {dist.max().item() * 1000:.0f}), 들기 mm {[round(v * 1000) for v in u.lift()[:12].tolist()]}, "
                  f"넘어짐 {int((u.puck_up() < 0.7).sum())}, 팔-상자 {int(u._contact('arm_box_contact').sum())}, 집게 수평 {(1 - (-u.a5_axes_w()[2])[:, 2].abs()).mean().item():.3f}", flush=True)
    lift = u.lift()
    ok = (lift > 0.03) & (u.puck_up() > 0.95)
    print(f"[C] 손 제어 결과: 세워 든 대 {int(ok.sum())}/{n}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
