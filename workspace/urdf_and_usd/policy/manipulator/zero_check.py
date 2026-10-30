"""진단: 넣기·옮기기 환경을 행동 0 (팔 그대로, 집게 닫음) 으로 1.5 s -> 종료 이유·쥠·접촉 (리셋 직후 상태가 맞는지)."""
import argparse
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", default="Waffle-Store-Direct")
parser.add_argument("--num_envs", type=int, default=64)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True
simulation_app = AppLauncher(args_cli).app
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import waffle_isaac  # noqa: E402,F401
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402


def main():
    cfg = parse_env_cfg(args_cli.task, num_envs=args_cli.num_envs)
    env = gym.make(args_cli.task, cfg=cfg)
    u = env.unwrapped
    env.reset()
    n = u.num_envs
    a = torch.zeros(n, cfg.action_space, device=u.device)
    a[:, -1] = -1.0
    s0 = u._state()
    print(f"[Zero] 리셋 직후 쥠 {int(s0['held'].sum())}/{n}, 퍽-잡는 점 mm 평균 {torch.linalg.norm(s0['pc'] - s0['gb'], dim=1).mean().item() * 1000:.1f}", flush=True)
    reasons = torch.zeros(5, dtype=torch.long)
    first = torch.ones(n, dtype=torch.bool, device=u.device)
    for t in range(45):
        _, _, term, trunc, _ = env.step(a)
        m = term & first
        for r in range(5):
            reasons[r] += int(((u.term_reason == r) & m).sum())
        first &= ~(term | trunc)
        if t in (0, 5, 44):
            s = u._state()
            print(f"[Zero] 스텝 {t}: 쥠 {int(s['held'].sum())}/{n}, 팔 링크-몸체 1 N 넘음 {int((u._force('arm_body_contact') > 1).sum())}, "
                  f"손가락-몸체 {int((u._force('finger_body_contact') > 1).sum())}, 퍽-몸체 {int((u._force('puck_body_contact') > 1).sum())}, "
                  f"퍽 직립 평균 {s['u'][:, 2].mean().item():.4f} 최소 {s['u'][:, 2].min().item():.3f}, 손가락 {s['fing'].mean().item():.4f}, 퍽-잡는 점 mm {torch.linalg.norm(s['pc'] - s['gb'], dim=1).mean().item() * 1000:.1f}", flush=True)
    print(f"[Zero] 1.5 s 안 첫 종료 이유: 놓침 {int(reasons[1])}, 세게 부딪힘 {int(reasons[2])}, 차체 밀림 {int(reasons[3])}, 넘어짐 {int(reasons[4])}, 안 끝남 {int(first.sum())}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
