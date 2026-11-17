"""미션 시뮬 P1 (2026-11-17): 미션 계획 (mission_planner plan.json) 의 집기 · 놓기 단계를 Isaac 한 장면에서 차례로 실행해 단계별 성공률을 잼.

  ~/pa-course/IsaacLab-6.1/.venv/bin/python mission_sim.py --plan results/plan_q1/plan.json --num_envs 16 [--gui]

바탕 = 팀 TopPick 환경 (Waffle-TopPick-Direct: 정밀 로봇 · kinematic 상자 · IK · 접촉 센서) — 팀 파일은 불러 쓰기만 (urdf_and_usd/policy/manipulator).
로봇은 환경 원점에 그대로 두고, 미션 단계마다 상자 · 퍽 · L 상자를 '도킹 뒤 로봇에서 본 자리' 로 옮겨 놓음 (= 로봇이 주행해 온 것과 같은 상대 배치).
  - 주행은 이번 판에서 순간이동 (도킹 오차만 뽑아 넣음). 주행 자체는 Gazebo · 실물로 확인됨 → P2 에서 도킹 정책으로 바꿈
  - 퍽 12 개는 따로 만든 강체: 팔레트에 실은 퍽 · L 에 놓은 퍽이 다음 단계까지 그대로 남음
  - 같은 상자를 다시 들르면 지난번 남은 퍽 자리 (상자 기준, 밀렸으면 밀린 그대로) 로 되살림

단계 (ROS manipulator_node · mission_lines 와 같은 순서):
  집기 = 도킹 (오차) → ±17° 회전 (회전 중심 center_x, 오차) → 퍽 다시 재기 (target_look σ) → 고정 동작 집기 (chain_pallet_eval 와 같은 경로)
         → 앞 운반 자세 → 칸 궤적 (pallet_timeline, 처짐 보정 ki) → 판정 (칸 축 3 mm · 높이 ±3 mm · 기울기 5°)
  놓기 = 도킹 (오차, 앞뒤는 TFmini 로 거의 맞춤) → 칸 빼기 (놓기 궤적 거꾸로) → L 칸 놓기 (ROS placeAt: 45° · 앞뒤 4 cm · 벌림 36.4 mm)
         → 판정 (칸 목표에서 수평 거리 · 서 있음 · 상자 위)
결과: results/<이름>/summary.json · steps.csv · 화면 표
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--plan", required=True, help="mission_plan 의 plan.json")
ap.add_argument("--poses", default=None, help="approach_poses.yaml (기본 저장소 mission_planner/config)")
ap.add_argument("--num_envs", type=int, default=16, help="같은 미션을 다른 오차로 동시에")
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--dock_sigma", default="3,8,1.0", help="도킹 오차 σ: 앞뒤 mm, 옆 mm, 방향 ° (놓기는 이 오차가 그대로 놓기 오차)")
ap.add_argument("--turn_sigma", type=float, default=1.0, help="[°] ±17° 회전 오차 σ")
ap.add_argument("--look_sigma", type=float, default=2.0, help="[mm] 집기 전 퍽 자리 다시 재기 (target_look) 오차 σ")
ap.add_argument("--grip_open", type=float, default=0.0125, help="[m 손가락] 집기 벌림 (0.0125 = 실물 고정 집기 열림 66.7 mm, manipulator.yaml finger_m_at_open. chain P0 은 0.0042)")
ap.add_argument("--place_release", type=float, default=-0.0026, help="[m 손가락] L 놓기 벌림 (ROS 0.0798 rad = 36.4 mm)")
ap.add_argument("--puck_mu", type=float, default=0.5)
ap.add_argument("--contact_offset", type=float, default=0.002)
ap.add_argument("--ki", type=float, default=4.0, help="처짐 보정 적분")
ap.add_argument("--steps", default="", help="이 단계 번호만 (예 '2-14', 시험용). 빈 값 = 전부")
ap.add_argument("--name", default=None)
ap.add_argument("--gui", action="store_true")
AppLauncher.add_app_launcher_args(ap)
a = ap.parse_args()
a.headless = not a.gui
app = AppLauncher(a).app

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, "..", ".."))
MP = os.path.join(WS, "urdf_and_usd", "policy", "manipulator")
sys.path.insert(0, MP)

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
import yaml  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_apply_inverse, quat_from_euler_xyz  # noqa: E402

import waffle_isaac  # noqa: E402,F401  (과제 등록)
from pallet_timeline import Speeds, seg_speed, timeline  # noqa: E402
from waffle_isaac.dice2_cfg import BOX_H, PUCK_H, PUCK_Z, box_cfg, puck_cfg  # noqa: E402
from waffle_isaac.robot_cfg import ARM_JOINTS, FINGER_RELEASED  # noqa: E402
from waffle_isaac.toppick_env_cfg import READY_TILT, WaffleTopPickEnvCfg, set_dr  # noqa: E402

PLAN = json.load(open(a.plan, encoding="utf-8"))
POSES = yaml.safe_load(open(a.poses or os.path.join(WS, "ros2_ws", "src", "mission_planner", "config", "approach_poses.yaml"), encoding="utf-8"))
TRAJ = json.load(open(os.path.join(MP, "pallet_place_traj.json"), encoding="utf-8"))
SLOT = {(s["row"], s["index"]): s for s in TRAJ["slots"]}
Q_FRONT = TRAJ["transit"]["q_front_tilt"]
SP = Speeds()
DS = [float(v) for v in a.dock_sigma.split(",")]
NAME = a.name or time.strftime("%m%d_%H%M%S") + "_" + os.path.splitext(os.path.basename(os.path.dirname(os.path.abspath(a.plan))))[0]
OUT = os.path.join(HERE, "results", NAME)
os.makedirs(OUT, exist_ok=True)
BOX_HALF = 0.205 / 2

# ---- 미션 퍽 (계획에서 id · 모양 · 색 · 상자 · 쪽) ----
PUCKS = {}
for s in PLAN["steps"]:
    if s["type"] == "pick":
        PUCKS[s["puck"]] = dict(shape="box" if s["shape"] == "box" else "cyl", color=s["color"], box=s["box"], side=s["side"], pose=s["pose"])
PIDS = sorted(PUCKS)
RGB = {"red": (0.85, 0.15, 0.12), "green": (0.15, 0.65, 0.2)}

# ---- 환경 설정: TopPick + 미션 퍽 12 · L 상자 ----
cfg = WaffleTopPickEnvCfg()
cfg.scene.num_envs = a.num_envs
cfg.seed = a.seed
cfg.near_start_prob = 0.0
cfg.tilt_prob = 1.0
cfg.gripper_open = a.grip_open
set_dr(cfg, "-friction")
mat = sim_utils.RigidBodyMaterialCfg(static_friction=a.puck_mu, dynamic_friction=a.puck_mu, friction_combine_mode="min")
col = sim_utils.CollisionPropertiesCfg(contact_offset=a.contact_offset, rest_offset=0.0)
for i, pid in enumerate(PIDS):
    p = PUCKS[pid]
    pc = puck_cfg(f"M{pid}", p["shape"], RGB.get(p["color"], (0.5, 0.5, 0.5)), (-0.55 + 0.08 * (i % 6), 0.55 + 0.08 * (i // 6)))
    setattr(cfg.scene, f"puckM{pid}", pc.replace(spawn=pc.spawn.replace(physics_material=mat, collision_props=col)))
cfg.scene.boxL = box_cfg("L", (-0.4, -0.6))
r_ = cfg.scene.robot
cfg.scene.robot = r_.replace(spawn=r_.spawn.replace(collision_props=col))


def main():
    env = gym.make("Waffle-TopPick-Direct", cfg=cfg)
    u = env.unwrapped
    n, dev, dt = u.num_envs, u.device, u.physics_dt
    g = torch.Generator(device=dev).manual_seed(a.seed)
    env.reset()
    allid = torch.arange(n, device=dev)
    org = u.scene.env_origins
    ez = torch.tensor([[0.0, 0.0, 1.0]], device=dev).expand(n, 3)
    ex = torch.tensor([[1.0, 0.0, 0.0]], device=dev).expand(n, 3)
    cmd = u.robot.actuators.target_command
    wheels0 = torch.zeros(n, len(u._wheels), device=dev)
    zeros = torch.zeros(n, device=dev)
    st = dict(bias=torch.zeros(n, 5, device=dev))

    # ---- 기본 동작 ----
    def obj(k):
        return u.scene[k]

    def root():
        d = u.robot.data
        return d.root_link_pos_w.torch, d.root_link_quat_w.torch

    def yaw_of(q):
        v = quat_apply(q, ex)
        return torch.atan2(v[:, 1], v[:, 0])

    def put(k, xy_b, yaw_b, z, ids=None):
        """차체 (지금 루트) 좌표 xy · 요 → 월드, 높이 z 는 환경 바닥 기준"""
        ids = allid if ids is None else ids
        if len(ids) == 0:
            return
        rp, rq = root()
        yr = yaw_of(rq)
        c, s = torch.cos(yr), torch.sin(yr)
        pose = obj(k).data.default_root_pose.torch.clone()
        pose[:, 0] = rp[:, 0] + c * xy_b[:, 0] - s * xy_b[:, 1]
        pose[:, 1] = rp[:, 1] + s * xy_b[:, 0] + c * xy_b[:, 1]
        pose[:, 2] = org[:, 2] + z
        pose[:, 3:7] = quat_from_euler_xyz(zeros, zeros, yr + yaw_b)
        obj(k).write_root_pose_to_sim_index(root_pose=pose[ids], env_ids=ids)
        obj(k).write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(ids), 6, device=dev), env_ids=ids)

    def park(k, i, ids=None):
        """쓰지 않는 물체 자리: 차체 양옆 y ±0.66 줄 (팔이 안 닿음)"""
        col, row = i % 12, i // 12   # 12 칸 줄: 0 = +0.66 · 1 = −0.66 · 2 = +0.72 · 3 = −0.72 (환경 간격 1.5 의 반 0.75 안)
        xy = torch.tensor([[-0.70 + 0.06 * col, (0.66 + 0.06 * (row // 2)) * (1.0 if row % 2 == 0 else -1.0)]], device=dev).expand(n, 2)
        put(k, xy, zeros, PUCK_H / 2 + 0.001, ids)

    def phys(arm, grip, k=1):
        for _ in range(k):
            cmd.set_position_index(value=arm.contiguous(), joint_ids=u._arm)
            cmd.set_position_index(value=grip.expand(-1, 2).contiguous(), joint_ids=u._gl + u._gr)
            cmd.set_velocity_index(value=wheels0, joint_ids=u._wheels)
            u.scene.write_data_to_sim()
            u.sim.step(render=a.gui)
            u.scene.update(dt)

    def gripv(v):
        return torch.full((n, 1), float(v), device=dev)

    def qmeas():
        return u.robot.data.joint_pos.torch[:, u._arm]

    def ppos(pid):
        return obj(f"puckM{pid}").data.root_pos_w.torch

    def pquat(pid):
        return obj(f"puckM{pid}").data.root_quat_w.torch

    def upright(pid):
        return quat_apply(pquat(pid), ez)[:, 2].abs()

    def to_b(p_w):
        rp, rq = root()
        return quat_apply_inverse(rq, p_w - rp)

    def in_hand_err(pid):
        """쥔 퍽 중심 − 기대 중심 (잡는 점 − 위 grasp_dz, 기울임 모드 = 차체 위) [m]"""
        return torch.linalg.norm(ppos(pid) - (u.grasp_w() - quat_apply(root()[1], ez) * cfg.grasp_dz), dim=1)

    def ik_move(goal_w, grip, sec, gain=0.3):
        """잡는 점 → goal (30 Hz, 환경 IK · cart_scale · 앞섬 제한 그대로 = chain_pallet_eval ik_move)"""
        for _ in range(int(sec / u.step_dt)):
            dp = (goal_w - u.grasp_w()) * gain
            dp = dp * torch.clamp(cfg.cart_scale / torch.linalg.norm(dp, dim=1, keepdim=True).clamp(min=1e-9), max=1.0)
            dq = u._ik_step(quat_apply_inverse(root()[1], dp))
            q = qmeas()
            u.arm_target = torch.clamp(torch.clamp(u.arm_target + dq, q - cfg.target_lead_max, q + cfg.target_lead_max), u._lo, u._hi)
            phys(u.arm_target, gripv(grip), cfg.decimation)

    def stream(qs, grips):
        """관절 목표 열 (스텝마다 (n, 5) 또는 공통 5) 을 처짐 보정 (ROS sag_comp, ki · ±0.15) 과 함께"""
        for q, gv in zip(qs, grips):
            tq = q if torch.is_tensor(q) else torch.tensor([q], device=dev).expand(n, 5)
            if a.ki > 0:
                st["bias"] = torch.clamp(st["bias"] + a.ki * dt * (tq - qmeas()), -0.15, 0.15)
            u.arm_target = tq + st["bias"]
            phys(u.arm_target, gv if torch.is_tensor(gv) else gripv(gv))

    def jmove(q_goal, speed, grip, settle=0.0):
        """대마다 지금 관절 → q_goal 관절 직선 (가장 먼 대가 speed), 처짐 보정 새로 시작 (chain 넘겨받기와 같음)"""
        q0 = qmeas().clone()
        qg = q_goal if torch.is_tensor(q_goal) else torch.tensor([q_goal], device=dev).expand(n, 5)
        m = max(1, int(math.ceil(float((qg - q0).abs().max()) / (speed * dt))))
        st["bias"] = torch.zeros(n, 5, device=dev)
        qs = [q0 + (qg - q0) * (k / m) for k in range(1, m + 1)] + [qg] * int(settle / dt)
        stream(qs, [grip] * len(qs))

    def gauss(shape, s):
        return torch.randn(shape, device=dev, generator=g) * s

    def sample_dock():
        return torch.stack((gauss((n,), DS[0] / 1000), gauss((n,), DS[1] / 1000), gauss((n,), math.radians(DS[2]))), dim=1)

    def dock_tf(p_b, err, th, xr):
        """도킹 기준 자리 p_b (n, 2) → 실제 차체 자리 (도킹 오차 err = (앞뒤, 옆, 요) · 회전 중심 (xr, 0) 둘레 th 회전 뒤) 좌표"""
        c, s = torch.cos(err[:, 2]), torch.sin(err[:, 2])
        d = p_b - err[:, :2]
        q = torch.stack((c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]), dim=1)
        o = torch.stack((xr - xr * torch.cos(th), -xr * torch.sin(th)), dim=1)
        d2 = q - o
        ct, sn = torch.cos(th), torch.sin(th)
        return torch.stack((ct * d2[:, 0] + sn * d2[:, 1], -sn * d2[:, 0] + ct * d2[:, 1]), dim=1)

    def puck_bottom(pid):
        """퍽 축 (위 방향) · 바닥 가운데 (월드)"""
        axv = quat_apply(pquat(pid), ez)
        axv = axv * torch.where(axv[:, 2:3] < 0, -1.0, 1.0)
        return axv, ppos(pid) - axv * (PUCK_H / 2)

    # ---- 시작: 미션 퍽 · TopPick 자체 물체 치움, 팔 = 앞 운반 자세 ----
    for i, pid in enumerate(PIDS):
        park(f"puckM{pid}", i)
    for j, k in enumerate(("puckT_cyl", "puckT_box", "puckN")):
        park(k, 16 + j)   # 줄 1 칸 4 ~ 6
    put("boxA", torch.tensor([[-0.60, 0.40]], device=dev).expand(n, 2), zeros, BOX_H / 2)
    put("boxL", torch.tensor([[-0.60, -0.40]], device=dev).expand(n, 2), zeros, BOX_H / 2)
    q0 = u.robot.data.default_joint_pos.torch.clone()
    q0[:, u._arm] = torch.tensor([Q_FRONT], device=dev)
    u.robot.write_joint_position_to_sim_index(position=q0, env_ids=allid)
    u.arm_target = q0[:, u._arm].clone()
    phys(u.arm_target, gripv(cfg.gripper_closed), 120)

    status = {pid: torch.zeros(n, dtype=torch.long, device=dev) for pid in PIDS}   # 0 상자 위 · 1 팔레트 · 2 L 위 · -1 잃음
    box_rel = {}      # 퍽 id -> 상자 기준 (x, y, 요) (n, 3): 집은 뒤 남은 퍽 (다시 들를 때 그 자리로)
    on_L = {}         # L -> [퍽 id]
    final = {}        # 퍽 id -> L 떠날 때 판정 dict (n,) 텐서
    rows = []
    cur_box, cur_L = None, None
    sel = None
    if a.steps:
        lo, _, hi = a.steps.partition("-")
        sel = set(range(int(lo), int(hi or lo) + 1))

    def log(i, s, kind, ok, extra):
        r = dict(step=i, kind=kind, at=s.get("at"), puck=s.get("puck"), ok=int(ok.sum()), n=n, rate=round(float(ok.float().mean()), 3), **extra)
        rows.append(r)
        print(f"[{i:2d}] {kind:5s} {s.get('at', ''):5s} 퍽 {s.get('puck', ''):>2} → {int(ok.sum()):3d}/{n} | " + " · ".join(f"{k} {v}" for k, v in extra.items()), flush=True)

    def judge_L(L):
        """L 위 퍽 판정 (L 상자 기준, 계획 셀 = 상자 중심 + (fwd − 0.2665, lat))"""
        bq, bp = obj("boxL").data.root_quat_w.torch, obj("boxL").data.root_pos_w.torch
        out = {}
        for pid, cell in on_L.get(L, []):
            axv, bot = puck_bottom(pid)
            bb = quat_apply_inverse(bq, bot - bp)
            ref = torch.tensor([[cell["fwd"] - 0.2665, cell["lat"]]], device=dev)
            exy = torch.linalg.norm(bb[:, :2] - ref, dim=1)
            top = (bb[:, 2] - BOX_H / 2).abs() < 0.01
            stand = upright(pid) > 0.95
            out[pid] = dict(exy=exy, ok15=top & stand & (exy < 0.015), ok30=top & stand & (exy < 0.030))
        return out

    def leave_L():
        if cur_L is None:
            return
        for pid, r in judge_L(cur_L).items():
            final[pid] = r
        for k_, (pid, _) in enumerate(on_L.get(cur_L, [])):
            park(f"puckM{pid}", 24 + 6 * ("L1", "L2", "L3").index(cur_L) + k_)

    def tidy_lost():
        """잃은 퍽 (−1) 중 차체 위 (팔레트 · 몸통) 가 아닌 것은 치움 (상자를 옮길 때 겹치지 않게)"""
        for i_, pid in enumerate(PIDS):
            b = to_b(ppos(pid))
            on_robot = (b[:, 0].abs() < 0.30) & (b[:, 1].abs() < 0.25) & (ppos(pid)[:, 2] - org[:, 2] > 0.03)
            ids = torch.nonzero((status[pid] == -1) & ~on_robot).flatten()
            park(f"puckM{pid}", i_, ids)

    t_all = time.time()
    for i, s in enumerate(PLAN["steps"]):
        if sel is not None and i not in sel:
            continue
        t0 = time.time()
        if s["type"] == "pick":
            at, pid, box = s["at"], s["puck"], s["box"]
            if PUCKS[pid]["pose"] != "standing":
                status[pid][:] = -1
                log(i, s, "pick", torch.zeros(n, dtype=torch.bool, device=dev), {"건너뜀": "누운 퍽 (세우기 없음)"})
                continue
            leave_L()
            cur_L = None
            e = POSES[at]
            xr = e.get("geom", {}).get("center_x", -0.04035)
            bx = e.get("geom", {}).get("box_x", 0.3625)
            by0 = 0.0 if e.get("inline") else math.pi / 2   # 상자 축 0 = 퍽 두 개가 차체 앞뒤 (inline) · 옆
            mates = [q for q in PIDS if PUCKS[q]["box"] == box]
            if box != cur_box:   # 다른 상자: 지난 상자에 남은 퍽 치움 (자리는 box_rel 에 있음)
                for i_, q in enumerate(PIDS):
                    if PUCKS[q]["box"] != box:
                        park(f"puckM{q}", i_, torch.nonzero(status[q] == 0).flatten())
                cur_box = box
            # 도킹 (오차) → 회전 (계획 turn_deg + 오차) 뒤 상자 · 남은 퍽을 차체 기준 자리로
            err = sample_dock()
            th = torch.full((n,), math.radians(s.get("turn_deg", 0.0)), device=dev)
            if s.get("turn_deg"):
                th = th + gauss((n,), math.radians(a.turn_sigma))
            ctr = dock_tf(torch.tensor([[bx, 0.0]], device=dev).expand(n, 2), err, th, xr)
            yaw_box = by0 - err[:, 2] - th
            cy, sy = torch.cos(yaw_box), torch.sin(yaw_box)
            put("boxA", ctr, yaw_box, BOX_H / 2)
            for q in mates:
                if q in box_rel:
                    rel = box_rel[q]
                else:
                    pk = e["pucks"].get(PUCKS[q]["side"], e["pucks"].get(str(PUCKS[q]["side"])))
                    d = torch.tensor([[0.096 + pk["fwd"] - bx, pk["lat"]]], device=dev).expand(n, 2)
                    rel = torch.stack((math.cos(by0) * d[:, 0] + math.sin(by0) * d[:, 1], -math.sin(by0) * d[:, 0] + math.cos(by0) * d[:, 1], zeros), dim=1)
                xy = ctr + torch.stack((cy * rel[:, 0] - sy * rel[:, 1], sy * rel[:, 0] + cy * rel[:, 1]), dim=1)
                put(f"puckM{q}", xy, yaw_box + rel[:, 2], PUCK_Z, torch.nonzero(status[q] == 0).flatten())
            phys(u.arm_target, gripv(cfg.gripper_closed), 30)
            present = status[pid] == 0
            # 퍽 다시 재기 (target_look σ) → 팔을 퍽 쪽 기울임 시작 자세로 → 고정 동작 집기 (ROS toppick_tilt top = chain --scripted_pick)
            u.is_box[:] = PUCKS[pid]["shape"] == "box"
            u.tilted[:] = True
            meas = ppos(pid) + gauss((n, 3), a.look_sigma / 1000) * torch.tensor([1.0, 1.0, 0.0], device=dev)
            mb = to_b(meas)
            ready = torch.tensor([[READY_TILT[j] for j in ARM_JOINTS]], device=dev).repeat(n, 1)
            ready[:, 0] = torch.atan2(mb[:, 1], mb[:, 0] - cfg.joint1_x)
            jmove(ready, 0.8, cfg.gripper_open, 0.3)
            up = quat_apply(root()[1], ez)
            rad = meas - u.j1_w()
            rad = rad - up * (rad * up).sum(1, keepdim=True)
            u_r = rad / torch.linalg.norm(rad, dim=1, keepdim=True).clamp(min=1e-6)   # J1 에서 먼 쪽 (수평)
            gl = meas + up * cfg.grasp_dz + u_r * 0.002                               # ROS radial_offset 0.002
            ik_move(gl + up * 0.12 - u_r * 0.03, cfg.gripper_open, 2.5)               # approach_heights[0] · approach_inward
            ik_move(gl, cfg.gripper_open, 1.5)
            ik_move(gl, cfg.gripper_closed, 0.6)
            rad_h = gl - u.j1_w()
            rad_h = rad_h - up * (rad_h * up).sum(1, keepdim=True)
            r_h = torch.linalg.norm(rad_h, dim=1, keepdim=True)
            ik_move(gl + up * 0.085 - rad_h / r_h.clamp(min=1e-6) * (r_h - 0.32).clamp(min=0.0), cfg.gripper_closed, 1.7)   # lift_height · lift_max_radius
            e_h = in_hand_err(pid)
            held = present & (e_h < 0.015) & (ppos(pid)[:, 2] - org[:, 2] > PUCK_Z + 0.02) & (upright(pid) > 0.95)
            # 앞 운반 자세 (칸 q_front, 0.5 rad/s + 1.5 s) → 칸 궤적 (pallet_timeline = chain 과 같음)
            slot = SLOT[tuple(s["slot"])]
            jmove(slot.get("q_front", Q_FRONT), 0.5, cfg.gripper_closed, 1.5)
            e_f = in_hand_err(pid)
            tl = timeline(slot, dt, SP, TRAJ["transit"]["q_front"])
            stream([x[0] for x in tl], [FINGER_RELEASED if x[1] > 0 else cfg.gripper_closed for x in tl])
            # 판정 (칸 축 3 mm · 바닥 ±3 mm · 기울기 5° = pallet_place_check)
            axv, botw = puck_bottom(pid)
            bot = to_b(botw)
            dxy = torch.hypot(bot[:, 0] - slot["x"], bot[:, 1] - slot["y"])
            dz = bot[:, 2] - slot["floor_z"]
            tilt = torch.rad2deg(torch.acos(axv[:, 2].clamp(-1, 1)))
            seated = held & (dxy < 0.003) & (dz.abs() < 0.003) & (tilt < 5.0)
            loose = held & (dxy < 0.010) & (dz.abs() < 0.010) & (tilt < 10.0)    # 칸 안 (3 mm 는 못 넘어도 빼기는 될 수 있음)
            status[pid] = torch.where(present & loose, torch.ones_like(status[pid]), torch.where(present, torch.full_like(status[pid], -1), status[pid]))
            # 같은 상자 남은 퍽: 서 있고 상자 위인가 · 자리 기억 (상자 기준)
            bq, bp = obj("boxA").data.root_quat_w.torch, obj("boxA").data.root_pos_w.torch
            mate_bad = torch.zeros(n, dtype=torch.bool, device=dev)
            for q in mates:
                if q == pid:
                    continue
                rb = quat_apply_inverse(bq, ppos(q) - bp)
                qx = quat_apply_inverse(bq, quat_apply(pquat(q), ex))
                ok_q = (upright(q) > 0.95) & (rb[:, 2] > 0.0) & (rb[:, :2].abs().amax(1) < BOX_HALF)
                was = status[q] == 0
                rel_new = torch.stack((rb[:, 0], rb[:, 1], torch.atan2(qx[:, 1], qx[:, 0])), dim=1)
                box_rel[q] = torch.where(was[:, None], rel_new, box_rel.get(q, rel_new))
                mate_bad |= was & ~ok_q
                status[q] = torch.where(was & ~ok_q, torch.full_like(status[q], -1), status[q])
            tidy_lost()
            log(i, s, "pick", seated, {"있음": int(present.sum()), "쥠": int(held.sum()), "칸 3mm": int(seated.sum()), "칸 1cm": int(loose.sum()),
                                       "옆 퍽 쓰러짐": int(mate_bad.sum()), "손안 mm 중앙 (들기·앞)": f"{float(e_h.median()) * 1000:.1f}·{float(e_f.median()) * 1000:.1f}",
                                       "칸 mm 중앙": round(float(dxy[held].median()) * 1000, 1) if held.any() else None, "s": round(time.time() - t0, 1)})
        elif s["type"] == "place":
            at, pid = s["at"], s["puck"]
            if at != cur_L:
                leave_L()
                cur_L = at
                for i_, q in enumerate(PIDS):   # 상자 위 남은 퍽 치움 (다시 들르면 box_rel 로 되살림)
                    park(f"puckM{q}", i_, torch.nonzero(status[q] == 0).flatten())
                cur_box = None
                errL = sample_dock()
                ctrL = dock_tf(torch.tensor([[0.3625, 0.0]], device=dev).expand(n, 2), errL, zeros, 0.0)
                put("boxA", torch.tensor([[-0.60, 0.40]], device=dev).expand(n, 2), zeros, BOX_H / 2)
                put("boxL", ctrL, -errL[:, 2], BOX_H / 2)
                phys(u.arm_target, gripv(cfg.gripper_closed), 30)
            present = status[pid] == 1
            # 칸 빼기 (놓기 궤적 거꾸로 = pallet_place_check timeline_unload)
            slot = SLOT[tuple(s["from_slot"])]
            S_ = {sg["name"]: [list(q) for q in sg["q"]] for sg in slot["segments"]}
            qf = list(slot.get("q_front", Q_FRONT))
            jmove(qf, 0.5, cfg.gripper_closed, 0.5)
            qs, gs, prev = [], [], qf

            def go(qlist, held_):
                nonlocal prev
                sp_ = seg_speed(slot, held_, SP)
                for q in qlist:
                    m = max(1, int(math.ceil(max(abs(x - y) for x, y in zip(q, prev)) / (sp_ * dt))))
                    for k in range(1, m + 1):
                        qs.append([p + (x - p) * k / m for x, p in zip(q, prev)])
                        gs.append(cfg.gripper_closed if held_ else FINGER_RELEASED)
                    prev = q

            for nm in ("rotate_back", "return", "retract"):
                go(S_[nm][::-1], False)
            go(S_["release"], False)
            qs += [prev] * int(0.3 / dt)
            gs += [FINGER_RELEASED] * int(0.3 / dt)
            qs += [prev] * int(0.6 / dt)
            gs += [cfg.gripper_closed] * int(0.6 / dt)
            go(S_["place"][::-1], True)
            go(S_["carry"][::-1], True)
            go(S_["rotate"][::-1] + [qf], True)
            qs += [prev] * int(1.0 / dt)
            gs += [cfg.gripper_closed] * int(1.0 / dt)
            stream(qs, gs)
            e_u = in_hand_err(pid)
            unloaded = present & (e_u < 0.010) & (upright(pid) > 0.95)
            # L 칸 놓기 (ROS placeAt: 셀 = 계획 자리 (로봇이 믿는 도킹 자세 기준), 퍽 바닥 = L 윗면 + 1 mm, 45°, 앞뒤 4 cm, 벌림 36.4 mm)
            cell = POSES[at]["cells"][s["cell"]]
            rp, rq = root()
            tgt_b = torch.tensor([[0.096 + cell["fwd"], cell["lat"], 0.0]], device=dev).expand(n, 3)
            tip = rp + quat_apply(rq, tgt_b)
            tip[:, 2] = org[:, 2] + BOX_H + PUCK_H / 2 + 0.001 + cfg.grasp_dz
            u.is_box[:] = PUCKS[pid]["shape"] == "box"
            u.tilted[:] = True
            appr = u.appr_dir_w(u.close_dir_w(tip))
            pre = tip - appr * 0.04
            ik_move(pre, cfg.gripper_closed, 3.0)
            ik_move(tip, cfg.gripper_closed, 1.5)
            ik_move(tip, a.place_release, 1.0)
            ik_move(pre, a.place_release, 1.0)
            jmove(Q_FRONT, 0.5, cfg.gripper_closed, 0.5)
            on_L.setdefault(at, []).append((pid, cell))
            jl = judge_L(at)
            r_ = jl[pid]
            placed = present & unloaded & r_["ok15"]
            status[pid] = torch.where(present & unloaded & r_["ok30"], torch.full_like(status[pid], 2), torch.full_like(status[pid], -1))
            pushed = torch.zeros(n, dtype=torch.bool, device=dev)
            for q, _ in on_L[at][:-1]:   # 먼저 놓은 퍽이 이번에 셀 밖으로 밀렸나
                pushed |= (status[q] == 2) & ~jl[q]["ok30"]
            tidy_lost()
            log(i, s, "place", placed, {"있음": int(present.sum()), "빼기": int(unloaded.sum()), "셀 15mm": int(placed.sum()),
                                        "셀 30mm": int((present & unloaded & r_["ok30"]).sum()),
                                        "셀 mm 중앙": round(float(r_["exy"][unloaded].median()) * 1000, 1) if unloaded.any() else None,
                                        "앞 퍽 밀려남": int(pushed.sum()), "s": round(time.time() - t0, 1)})
    leave_L()

    # ---- 요약 ----
    ok15 = torch.stack([final[p]["ok15"] if p in final else torch.zeros(n, dtype=torch.bool, device=dev) for p in PIDS], dim=1)
    ok30 = torch.stack([final[p]["ok30"] if p in final else torch.zeros(n, dtype=torch.bool, device=dev) for p in PIDS], dim=1)
    per_puck = {str(p): dict(ok15=int(ok15[:, k].sum()), ok30=int(ok30[:, k].sum()),
                             exy_mm_median=round(float(final[p]["exy"].median()) * 1000, 1) if p in final else None) for k, p in enumerate(PIDS)}
    summary = dict(plan=os.path.relpath(a.plan, HERE), n=n, seed=a.seed, dock_sigma=a.dock_sigma, turn_sigma=a.turn_sigma, look_sigma=a.look_sigma,
                   grip_open=a.grip_open, place_release=a.place_release, ki=a.ki, steps=a.steps or "all", pucks=len(PIDS),
                   final_ok15_mean=round(float(ok15.sum(1).float().mean()), 2), final_ok30_mean=round(float(ok30.sum(1).float().mean()), 2),
                   mission_all15=int((ok15.sum(1) == len(PIDS)).sum()), mission_all30=int((ok30.sum(1) == len(PIDS)).sum()),
                   per_puck=per_puck, wall_s=round(time.time() - t_all, 1), date=time.strftime("%Y-%m-%d %H:%M"))
    json.dump(dict(summary=summary, steps=rows), open(os.path.join(OUT, "summary.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    with open(os.path.join(OUT, "steps.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
        w.writeheader()
        w.writerows(rows)
    print(f"\n미션 {n} 대: 끝까지 L 셀 15 mm 안 퍽 평균 {summary['final_ok15_mean']}/{len(PIDS)} (30 mm {summary['final_ok30_mean']}) · "
          f"12 개 모두 {summary['mission_all15']}/{n} (30 mm {summary['mission_all30']}) · {summary['wall_s']} s → {OUT}", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    app.close()
