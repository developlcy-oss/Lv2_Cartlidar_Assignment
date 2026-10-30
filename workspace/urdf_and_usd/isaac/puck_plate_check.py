"""퍽 플레이트 점검: 로봇 (주행 자세) 이 자리 잡은 뒤 컵 12 개에 퍽을 턱 위 drop mm 에서 놓고 2 초 뒤 앉은 자리를 잰다.

    ./run.sh 와 같은 Isaac Lab Python 으로:  PYTHONPATH=$PWD <isaaclab.sh -p> puck_plate_check.py [--shape cyl|box] [--light] [--drop 2]

- 컵 자리 · 방향 · 턱 높이 = ../waffle_4wd_description_261028/meshes/parts/puck_plate.json (플레이트 원점 = base_link (0.096, 0, 0.111)).
- 퍽: 원기둥 지름 30 · 높이 60 (45 g) 또는 상자 30 x 30 x 60 (55 g, 컵 사각 구멍 방향으로 돌림).
- 판정: 퍽 바닥이 턱 높이 ±1.5 mm, 컵 가운데 2.5 mm 안, 기울기 6° 안. 가벼운 판 (--light) 은 컵이 꽉 찬 상자라 퍽이 컵 위에 얹힌다 (넣기 시험은 정밀 판).
- Isaac Lab 3.0 쿼터니언 순서는 (x, y, z, w).
"""
import argparse
import json
import math
import os

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--shape", choices=("cyl", "box"), default="cyl")
ap.add_argument("--light", action="store_true", help="가벼운 판 USD (기본: 정밀 판)")
ap.add_argument("--drop", type=float, default=2.0, help="턱 위 놓는 높이 [mm]")
ap.add_argument("--settle", type=float, default=1.0, help="퍽을 놓기 전 로봇이 자리 잡는 시간 [s]")
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg  # noqa: E402
from isaaclab_physx.physics import PhysxCfg  # noqa: E402

from waffle_4wd_isaac import robot_cfg as R  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PLATE_JSON = os.path.join(HERE, "..", "waffle_4wd_description_261028", "meshes", "parts", "puck_plate.json")
PLATE = (0.096, 0.0, 0.111)          # puck_plate_joint (base_link 기준)
HZ = 240


def main():
    cups = json.load(open(PLATE_JSON, encoding="utf-8"))["cups"]
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / HZ, device=args.device, physics=PhysxCfg(),
                                                              physics_material=R.CONTACT_MATERIAL))
    g = sim_utils.GroundPlaneCfg(physics_material=R.CONTACT_MATERIAL)
    g.func("/World/ground", g)
    cfg = (R.WAFFLE_4WD_CFG if args.light else R.WAFFLE_4WD_FULL_CFG).replace(prim_path="/World/Robot")
    cfg.init_state.joint_pos = R.pose("drive")
    robot = Articulation(cfg)
    pucks = []
    for i in range(len(cups)):           # 처음엔 로봇 옆 바닥에 두고, 로봇이 자리 잡은 뒤 컵으로 옮김
        common = dict(rigid_props=sim_utils.RigidBodyPropertiesCfg(solver_position_iteration_count=16, max_depenetration_velocity=1.0),
                      mass_props=sim_utils.MassPropertiesCfg(mass=0.045 if args.shape == "cyl" else 0.055),
                      collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
                      visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.85, 0.15, 0.15)))
        spawn = (sim_utils.CylinderCfg(radius=0.015, height=0.06, axis="Z", **common) if args.shape == "cyl"
                 else sim_utils.CuboidCfg(size=(0.03, 0.03, 0.06), **common))
        pucks.append(RigidObject(RigidObjectCfg(prim_path=f"/World/Puck{i:02d}", spawn=spawn,
                                                init_state=RigidObjectCfg.InitialStateCfg(pos=(1.0 + 0.1 * i, 1.0, 0.031)))))
    sim.reset()
    dt = sim.get_physics_dt()
    hold, _ = robot.find_joints(R.ARM_JOINTS + R.FINGER_JOINTS, preserve_order=True)
    tgt = torch.tensor([[R.pose("drive")[n] for n in R.ARM_JOINTS + R.FINGER_JOINTS]], device=sim.device)

    def run(n):
        for _ in range(n):
            robot.actuators.target_command.set_position_index(value=tgt, joint_ids=hold)
            robot.write_data_to_sim()
            sim.step()
            robot.update(dt)
            for p in pucks:
                p.update(dt)

    run(int(args.settle * HZ))
    rb = robot.data.root_pos_w.torch[0].tolist()
    for c, p in zip(cups, pucks):
        a = math.radians(c["square_deg"]) / 2
        pose = [rb[0] + PLATE[0] + c["center_mm"][0] / 1000, rb[1] + PLATE[1] + c["center_mm"][1] / 1000,
                rb[2] + PLATE[2] + (c["seat_mm"] + 30.0 + args.drop) / 1000, 0.0, 0.0, math.sin(a), math.cos(a)]
        p.write_root_pose_to_sim(torch.tensor([pose], device=sim.device))
        p.write_root_velocity_to_sim(torch.zeros(1, 6, device=sim.device))
    run(2 * HZ)

    rb = robot.data.root_pos_w.torch[0].tolist()
    out, ok = [], 0
    for c, p in zip(cups, pucks):
        pos = p.data.root_pos_w.torch[0].tolist()
        q = p.data.root_quat_w.torch[0].tolist()                          # x, y, z, w
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q[0] ** 2 + q[1] ** 2)))))
        dz = (pos[2] - 0.03 - (rb[2] + PLATE[2] + c["seat_mm"] / 1000)) * 1000
        dxy = math.hypot(pos[0] - rb[0] - PLATE[0] - c["center_mm"][0] / 1000, pos[1] - rb[1] - PLATE[1] - c["center_mm"][1] / 1000) * 1000
        good = abs(dz) < 1.5 and dxy < 2.5 and tilt < 6.0
        ok += good
        out.append(dict(radius_mm=c["radius_mm"], seat_mm=c["seat_mm"], dz_mm=round(dz, 2), dxy_mm=round(dxy, 2), tilt_deg=round(tilt, 2), ok=good))
        print(f"[puck_plate] 컵 r {c['radius_mm']:5.1f} 턱 {c['seat_mm']:4.1f} mm: 높이 {dz:+6.2f} mm, 수평 {dxy:5.2f} mm, 기울기 {tilt:5.2f}° {'OK' if good else 'NG'}")
    kind = "light" if args.light else "full"
    print(f"[puck_plate] {kind} {args.shape}: {ok}/{len(cups)} 컵에 앉음 (놓은 높이 턱 위 {args.drop} mm)")
    os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
    json.dump(dict(kind=kind, shape=args.shape, drop_mm=args.drop, seated=ok, cups=out),
              open(os.path.join(HERE, "results", f"puck_plate_{kind}_{args.shape}.json"), "w"), ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
    app.close()
