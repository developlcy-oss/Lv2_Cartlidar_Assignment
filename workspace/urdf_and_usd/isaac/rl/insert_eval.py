"""넣기 전용 평가 (창 없이): 학습된 정책 (결정적 행동) 으로 환경마다 에피소드 1 번 -> 시작 위치별 (통 위 / 모따기 / 통 안) 성공률,
통 안 깊이, 팔 링크·손가락-몸체 최대 힘. '통 위' 가 실제로 ST3 가 넘겨주는 상태라 이 값이 중요.

    ./run_isaac.sh insert_eval [환경 수]
"""

import argparse
import glob
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=192)
parser.add_argument("--policy", default=None)
parser.add_argument("--above_only", action="store_true", help="통 위 시작만")
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
from waffle_isaac.insert_env_cfg import WaffleInsertEnvCfg  # noqa: E402


def main():
    path = args_cli.policy or sorted(glob.glob(os.path.join(HERE, "logs/rsl_rl/waffle_insert/*/exported/policy.pt")))[-1]
    cfg = WaffleInsertEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    if args_cli.above_only:
        cfg.start_weight = (1.0, 0.0, 0.0)
    env = gym.make("Waffle-Insert-Direct", cfg=cfg)
    u = env.unwrapped
    pol = torch.jit.load(path, map_location=u.device).eval()
    obs, _ = env.reset()
    n, dev = u.num_envs, u.device
    bucket = u.start_bucket.clone()
    tube = u.tube.clone()
    done = torch.zeros(n, dtype=torch.bool, device=dev)
    ok = torch.zeros(n, dtype=torch.bool, device=dev)
    why = torch.zeros(n, dtype=torch.long, device=dev)  # 0 시간 끝, 1 성공, 2 놓침, 3 세게 부딪힘
    f_arm = torch.zeros(n, device=dev)
    f_fing = torch.zeros(n, device=dev)
    depth_end = torch.zeros(n, device=dev)
    for t in range(int(cfg.episode_length_s / u.step_dt)):
        with torch.inference_mode():
            a = pol(obs["policy"])
        live = ~done
        s = u._state()
        depth_end = torch.where(live, s["depth"], depth_end)
        f_arm = torch.maximum(f_arm, u._force("arm_body_contact") * live.float())
        f_fing = torch.maximum(f_fing, u._force("finger_body_contact") * live.float())
        obs, _, term, trunc, _ = env.step(a)
        ok |= u.succeeded & live
        new = (term | trunc) & live
        why = torch.where(new & ~ok & term, u.term_reason + 1, why)  # 2 놓침, 3 세게 부딪힘, 4 차체 밀림, 5 넘어짐
        for i in torch.nonzero(new & ~ok & term & (u.term_reason == 2)).flatten().tolist()[:6]:
            print(f"[Eval]   env{i} 세게 부딪힘: 팔 링크 {u.term_arm_f[i].item():.1f} N, 손가락 {u.term_fing_f[i].item():.1f} N, 깊이 {s['depth'][i].item() * 1000:.0f} mm, 축 {s['dxy'][i].item() * 1000:.1f} mm", flush=True)
        done |= new
    names = ("통 위", "모따기", "통 안")
    print(f"[Eval] 정책 {path}", flush=True)
    for b in range(3):
        m = bucket == b
        if m.any():
            print(f"[Eval] {names[b]} 시작 {int(m.sum())} 대: 성공 {int(ok[m].sum())} ({ok[m].float().mean().item() * 100:.0f} %), "
                  f"놓침 종료 {int((why[m] == 2).sum())}, 세게 부딪힘 {int((why[m] == 3).sum())}, 차체 밀림 {int((why[m] == 4).sum())}, 넘어짐 {int((why[m] == 5).sum())}, 시간 끝 {int(((why[m] == 0) & ~ok[m]).sum())}", flush=True)
    for i, nm in enumerate("CPN"):
        m = tube == i
        if m.any():
            print(f"[Eval] {nm} 통 {int(m.sum())} 대: 성공 {ok[m].float().mean().item() * 100:.0f} %", flush=True)
    print(f"[Eval] 전체 성공 {ok.float().mean().item() * 100:.1f} % | 팔 링크-몸체 최대 힘 N: 평균 {f_arm.mean().item():.1f}, 최대 {f_arm.max().item():.1f}, "
          f"1 N 넘은 대수 {int((f_arm > 1).sum())} | 손가락-몸체 최대 힘 N: 평균 {f_fing.mean().item():.1f}, 최대 {f_fing.max().item():.1f}", flush=True)
    fail = ~ok
    if fail.any():
        print(f"[Eval] 실패 대 마지막 깊이 mm (+ = 입구 아래) {[round(v * 1000) for v in depth_end[fail][:24].tolist()]}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
