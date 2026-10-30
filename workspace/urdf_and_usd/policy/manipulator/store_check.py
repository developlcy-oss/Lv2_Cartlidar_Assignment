"""스토리지 넣기 전 확인 (창 없이, Isaac PhysX): `Waffle-Store-Direct` 장면으로

  1. 퍽 떨어뜨리기: 통 3 개 (가운데 C, +y P, −y N) 축 위, 퍽 바닥 = 입구 + 5 mm 에서 세워 놓음 -> 2 s 뒤 퍽 바닥 높이·축 거리·직립
     (a) 축에 딱 맞게 (b) 1.5 mm 벗어나고 3° 기울게 (MuJoCo 에서 끼었던 조건, 정책_학습기록 4.9 절)
  2. 팔-몸체 접촉 센서: 팔을 앞 (닿지 않음) -> 뒤로 접어 몸체·스토리지에 누름 -> 센서 힘

    ./run_isaac.sh store_check   -> sim/isaac/store_check.json
"""

import argparse
import json
import math
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--grid", default=None, help="C|P|N: 그 통 축 둘레 7x7 (1.5 mm 간격) 에 똑바로 떨어뜨려 들어가는 자리 찾기")
parser.add_argument("--start_dz", type=float, default=0.005, help="[m] 떨어뜨리기 시작 퍽 바닥 = 입구 + 이 값 (음수 = 통 안)")
parser.add_argument("--radius", type=float, default=None, help="[m] 퍽 반지름 바꿔 시험 (기본 0.015)")
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
from waffle_isaac.dice2_cfg import PUCK_H  # noqa: E402
from waffle_isaac.store_env_cfg import WaffleStoreEnvCfg  # noqa: E402


TIMELINE = [False]


def main():
    cfg = WaffleStoreEnvCfg()
    cfg.scene.num_envs = 49 if args_cli.grid else 6
    if args_cli.radius:
        cfg.scene.puck = cfg.scene.puck.replace(spawn=cfg.scene.puck.spawn.replace(radius=args_cli.radius))
    cfg.start_cats = ("A",)  # 팔은 앞쪽 (통에서 멀리)
    env = gym.make("Waffle-Store-Direct", cfg=cfg)
    u = env.unwrapped
    env.reset()
    dev, n = u.device, u.num_envs

    def run(sec, arm_target=None, grip_open=True):
        u.actions[:, 5] = 1.0 if grip_open else -1.0
        if arm_target is not None:
            u.arm_target[:] = arm_target
        fmax = torch.zeros(n, device=dev)
        for k in range(int(sec / u.physics_dt)):
            if TIMELINE[0] and k % 12 == 0:
                st = u._state()
                print(f"[StoreCheck] t={k * u.physics_dt:.1f} 깊이 mm {[round(v * 1000) for v in st['depth'].tolist()]} 축 mm {[round(v * 1000, 1) for v in st['dxy'].tolist()]}"
                      f" 직립 {[round(v, 3) for v in st['u'][:, 2].tolist()]} 퍽-몸체 N {[round(v, 1) for v in u._force('puck_body_contact').tolist()]}", flush=True)
            u._apply_action()
            u.scene.write_data_to_sim()
            u.sim.step(render=False)
            u.scene.update(u.physics_dt)
            fmax = torch.maximum(fmax, torch.maximum(u._force("arm_body_contact"), u._force("finger_body_contact")))
        return fmax

    ids = torch.arange(n, device=dev)
    if args_cli.grid:
        ti = "CPN".index(args_cli.grid)
        u.tube[:] = ti
        t = u._tubes[u.tube]
        g = torch.arange(-3, 4, device=dev) * 0.0015
        gx, gy = torch.meshgrid(g, g, indexing="ij")
        root = u.robot.data.root_link_pos_w.torch
        c = root + u._bl + torch.stack((t[:, 0] + gx.flatten(), t[:, 1] + gy.flatten(), t[:, 2] + args_cli.start_dz + PUCK_H / 2), dim=1)
        quat = torch.tensor([[0.0, 0.0, 0.0, 1.0]], device=dev).repeat(n, 1)
        u.puck.write_root_pose_to_sim_index(root_pose=torch.cat((c, quat), dim=1), env_ids=ids)
        u.puck.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=ids)
        run(2.0, u.robot.data.joint_pos.torch[:, u._arm].clone())
        s = u._state()
        dep = (s["depth"] * 1000).round().int().view(7, 7).tolist()
        print(f"[StoreCheck] 격자 {args_cli.grid} 통 (반지름 {args_cli.radius or 0.015}): 행 = x {[-4.5, -3, -1.5, 0, 1.5, 3, 4.5]} mm, 열 = y, 값 = 입구 아래 깊이 mm", flush=True)
        for row in dep:
            print(f"[StoreCheck]   {row}", flush=True)
        env.close()
        return

    # 1. 떨어뜨리기
    u.tube[:] = torch.tensor([0, 1, 2, 0, 1, 2], device=dev)
    t = u._tubes[u.tube]
    off = torch.zeros(n, 3, device=dev)
    off[3:, 0] = 0.0015
    tilt = torch.zeros(n, device=dev)
    tilt[3:] = math.radians(3.0)
    root = u.robot.data.root_link_pos_w.torch
    c = root + u._bl + torch.stack((t[:, 0], t[:, 1], t[:, 2] + args_cli.start_dz + PUCK_H / 2), dim=1) + off
    quat = torch.stack((torch.sin(tilt / 2), torch.zeros_like(tilt), torch.zeros_like(tilt), torch.cos(tilt / 2)), dim=1)  # x 축 기울기 (x,y,z,w)
    u.puck.write_root_pose_to_sim_index(root_pose=torch.cat((c, quat), dim=1), env_ids=ids)
    u.puck.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=ids)
    q_front = u.robot.data.joint_pos.torch[:, u._arm].clone()
    TIMELINE[0] = True
    f_front = run(2.0, q_front)
    TIMELINE[0] = False
    s = u._state()
    drop = []
    for i in range(n):
        drop.append({"tube": "CPN"[int(u.tube[i])], "case": "on_axis" if i < 3 else "off1.5mm_tilt3deg",
                     "bottom_z_mm": round(float(s["pb"][i, 2]) * 1000, 1), "below_rim_mm": round(float(s["depth"][i]) * 1000, 1),
                     "axis_off_mm": round(float(s["dxy"][i]) * 1000, 1), "upright": round(float(s["u"][i, 2]), 3)})
        print(f"[StoreCheck] 떨어뜨리기 {drop[-1]}", flush=True)

    # 2. 팔-몸체 접촉: 앞 자세에서는 0, 뒤로 접어 누르면 힘
    q_back = torch.tensor([[3.14, 1.3, 0.3, 0.3, 0.0]], device=dev).repeat(n, 1)  # 뒤로 돌려 스토리지 위로 내림 (261028 팔): MuJoCo 에서 gripper_roll_link 13 mm 파고듦
    f_back = run(3.0, q_back)
    contact = {"front_pose_max_N": [round(v, 2) for v in f_front.tolist()], "folded_into_body_max_N": [round(v, 2) for v in f_back.tolist()],
               "arm_q_after": [round(v, 2) for v in u.robot.data.joint_pos.torch[0, u._arm].tolist()]}
    print(f"[StoreCheck] 팔-몸체 접촉 {contact}", flush=True)
    json.dump({"drop": drop, "arm_body_contact": contact}, open(os.path.join(HERE, "store_check.json"), "w"), ensure_ascii=False, indent=1)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
