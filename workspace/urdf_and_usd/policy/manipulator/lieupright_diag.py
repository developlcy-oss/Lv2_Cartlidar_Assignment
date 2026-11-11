"""2026-11-10 (Onbeom, P1 B2 진단): 'at' 시작 (각본 단계 1 끝, 잡는 점 · 벌림) 에서 정해진 행동으로 쥐기 · 들기가 되는지.
  0 ~ 9 스텝: 움직임 0 · 벌림 유지 / 10 ~ 29: 움직임 0 · 닫음 / 30 ~ 59: 위로 (z +1) · 닫음.  스텝마다 집계 출력."""
import argparse, os, sys
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--kind", type=int, default=4)
parser.add_argument("--air", type=int, default=0, help="1 = 움직임 없이 닫기만 (above 시작이면 허공에서 빈손 닫기), 손가락이 쥠 범위에 연속으로 있던 스텝 수 셈")
parser.add_argument("--empty", type=int, default=0, help="1 = 처음 10 스텝 잡는 점을 base y 로 5 cm 비킨 뒤 같은 행동 (빈손 닫기)")
AppLauncher.add_app_launcher_args(parser)
a_ = parser.parse_args(); a_.headless = True
app = AppLauncher(a_).app
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gymnasium as gym, torch  # noqa
import waffle_isaac  # noqa
from waffle_isaac.lieupright_env_cfg import WaffleLieUprightB2EnvCfg  # noqa
cfg = WaffleLieUprightB2EnvCfg(); cfg.scene.num_envs = a_.num_envs
sp = [0.0] * 5; sp[a_.kind] = 1.0; cfg.start_probs = tuple(sp); cfg.seed = 3
env = gym.make("Waffle-LieUprightB2-Direct", cfg=cfg); u = env.unwrapped
env.reset()
n, dev = u.num_envs, u.device
p0 = u.puck_pos_w().clone(); q0 = u.robot.data.joint_pos.torch[:, u._arm].clone()
print(f"[Diag] kind {a_.kind}, 대 {n}, kind 확인 {u.kind.float().mean().item():.2f}, beta {u.beta.mean().item():.3f} rho {u.rho.mean().item():.3f}", flush=True)
ended = torch.zeros(n, dtype=torch.bool, device=dev); why = {}; hold_steps = 0
run_ = torch.zeros(n, device=dev); run_max = torch.zeros(n, device=dev)
for t in range(60):
    a = torch.zeros(n, 6, device=dev)
    a[:, -1] = 1.0 if t < 10 else -1.0
    if a_.empty and t < 10:
        a[:, 1] = 1.0
    if t >= 30 and not a_.air:
        a[:, 2] = 1.0
    fg_ = u.robot.data.joint_pos.torch[:, u._gl[0]]
    inr = (fg_ > cfg.grasp_finger_range[0]) & (fg_ < cfg.grasp_finger_range[1])
    run_ = torch.where(inr, run_ + 1, torch.zeros_like(run_)); run_max = torch.maximum(run_max, run_)
    _, r, te, tr, _ = env.step(a)
    for k_ in ('t_drop', 't_nb', 't_self', 't_ab', 't_bb', 't_topple'):
        why[k_] = why.get(k_, 0) + int((getattr(u, k_) & ~ended).sum())
    hold_steps += int((u.holding() & ~ended).sum())
    ended |= te | tr
    if t in (0, 1, 2, 5, 9, 12, 15, 20, 29, 35, 45, 59):
        g = u.grasp_w(); pc = u.puck_pos_w()
        fing = u.robot.data.joint_pos.torch[:, u._gl[0]]
        dq = (u.robot.data.joint_pos.torch[:, u._arm] - q0).abs().amax(1)
        print(f"[Diag] t {t:2d} | 퍽 이동 mm 중앙 {((pc - p0).norm(dim=1) * 1000).median().item():6.1f} 최대 {((pc - p0).norm(dim=1) * 1000).max().item():6.1f} | "
              f"잡는 점-퍽 mm {((g - pc).norm(dim=1) * 1000).median().item():5.1f} | 손가락 중앙 {fing.median().item() * 1000:6.2f} mm "
              f"(최소 {fing.min().item() * 1000:6.2f} 최대 {fing.max().item() * 1000:6.2f}) | holding {u.holding().float().mean().item():.2f} grasp_seen {u.grasp_seen.float().mean().item():.2f} "
              f"| 관절 변화 최대 rad {dq.median().item():.3f} | lift mm {(u.lift().median().item() * 1000):6.1f} | 보상 {r.mean().item():6.2f} | was_held {u.was_held.float().mean().item():.2f} | 누적 쥠 스텝 {hold_steps} | 누적 끝남 {int(ended.sum())} {why}", flush=True)
print(f"[Diag] 손가락이 쥠 범위에 연속으로 있던 스텝 최대: 중앙 {run_max.median().item():.0f} 최대 {run_max.max().item():.0f} (쥠 판정 {cfg.hold_n_on} 스텝), {cfg.hold_n_on} 스텝 이상인 대 {int((run_max >= cfg.hold_n_on).sum())} (분포 {sorted(run_max.int().tolist())}), 범위 밖으로 끝난 대 {int((~inr).sum())}/{n}", flush=True)
env.close(); app.close()
