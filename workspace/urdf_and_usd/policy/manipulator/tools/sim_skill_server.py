"""시뮬 스킬 서버 (2026-11-13, 실물 시험 연습용 — teleop 처럼 bash 한 줄로 스킬 하나씩).

Isaac Lab 장면 1 개 (차체 + 팔레트 + 상자 1 개 위 퍽 2 개) 를 창으로 띄우고, ROS 토픽으로 명령을 받아 한 스킬씩 실행한다.
  명령: /sim_skill/cmd (std_msgs/String) — 이름은 manipulator_node 스킬과 같게
    toppick_tilt                         상자 위 퍽 (J1 에서 가까운 것부터) 을 집기 정책 (기본 k9) 으로 45° 집기. 노드 조건:
                                         시작 READY_POSE_45, 퍽 좌표 한 번 잰 값 고정 (±perc_bias), 관절 변화 상한 4.0 rad/s, 행동 지연, 성공 = 쥠 + 3 cm 1 s
    pallet_load_<열>_<칸>_tilt           쥔 퍽을 팔레트 칸에 (노드 runPalletSlot: rotate · carry · place -> 머묾 -> 벌림 36.4 mm -> retract · return · rotate_back, 처짐 보정 ki 4)
    pallet_unload_<열>_<칸>_tilt         칸 퍽 꺼내기 (거꾸로 + q_front, 처짐 보정)
    place_at:base_link:<x>:<y>:<z>       쥔 퍽을 (x, y) 에, z = 놓을 윗면 높이 (노드 placeAt: 45°, 앞 4 cm 직선, 벌림 36.4 mm, 옆 보정 1.8 mm, 처짐 보정 스트리밍)
    시뮬 전용: transit (운반 자세) · reset (퍽 다시 놓기, 채운 퍽 비움) · cells (상자 위 3 x 3 칸 좌표) · status (퍽 자리)
               clear (상자 위 남은 퍽 치우기 — 같은 상자에 주사위 눈을 놓을 때)
               fill <열_칸> ... (팔레트 칸에 퍽 바로 채우기, 칸 모양대로 원기둥 · 직육면체, 예: fill 3_0 3_3 2_1)
  결과: /sim_skill/result (std_msgs/String, JSON 한 줄 {"cmd", "ok", "msg", "t"})
실행 (컨테이너 isaac-lab-base 안, tools/sim_skill.sh start 가 대신 함):
  ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib:$LD_LIBRARY_PATH \\
    /workspace/isaaclab/isaaclab.sh -p tools/sim_skill_server.py [--headless] [--policy policies/sidepick45_k9.pt]
"""
import argparse
import json
import math
import os
import sys
import time

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--policy", default="policies/sidepick45_k9.pt")
ap.add_argument("--act_delay", default="0,1", help="행동 지연 스텝 (실측 0 ~ 66 ms = 0,2 · horizon 1.0 이면 0,1)")
ap.add_argument("--perc_bias", type=float, default=2.0, help="[mm] target_look 잰 퍽 좌표 오차 ±")
ap.add_argument("--box_prob", type=float, default=0.5, help="집을 퍽 (첫째) 이 직육면체일 확률")
ap.add_argument("--max_joint_vel", type=float, default=4.0, help="[rad/s] 집기 정책 관절 변화 상한 (노드 sidepick45_k9.max_joint_velocity)")
ap.add_argument("--seed", type=int, default=-1)
AppLauncher.add_app_launcher_args(ap)
a = ap.parse_args()
app = AppLauncher(a).app

import omni.kit.app  # noqa: E402
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.ros2.core", True)
import rclpy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "tools_ik"))
import waffle_isaac  # noqa: E402,F401
from isaaclab.utils.math import quat_apply, quat_apply_inverse  # noqa: E402
from unload_traj import seeds_for, solve  # noqa: E402
from waffle_isaac.dice2_cfg import BOX_H, PUCK_H  # noqa: E402
from waffle_isaac.robot_cfg import FINGER_CLOSE  # noqa: E402
from waffle_isaac.sidepick_env_cfg import READY_POSE_45, WaffleSidePick45MissionEnvCfg  # noqa: E402

TRAJ = json.load(open(os.path.join(HERE, "pallet_place_traj.json")))
SLOTS = {(s["row"], s["index"]): s for s in TRAJ["slots"]}
TRANSIT = TRAJ["transit"]["q_front_tilt"]
J1X, PITCH = 0.096, 45
RELEASE_M = (36.4 - 41.6) / 2000.0       # 노드 pallet.release_position · place.release_position 0.0798 rad = 36.4 mm (벌림 mm = 41.6 + 2000 m)
OPEN_M = 0.010                           # 정책 벌림 (toppick_rl.open_m, 61.6 mm)
KI, BIAS_MAX, SETTLE, DWELL, GRIP_T = 4.0, 0.15, 1.5, 0.3, 1.0   # 노드 sag.* · pallet.place_dwell · gripper.move_time
PALLET_SPEED, ROW3_HOLD = 0.8, 0.4
LATERAL = 0.0018                         # place.lateral_offset_mm 1.8
GRIP_RANGE = (-0.010, -0.003)            # toppick_rl.grasp_finger_min · max


def ik(p_b, seed=None):
    h = np.array([p_b[0] - J1X, p_b[1], 0.0]); h /= np.linalg.norm(h)
    th = math.atan2(h[1], h[0])
    q = solve(np.asarray(p_b), h, PITCH, ([np.asarray(seed)] if seed is not None else []) + seeds_for(th))
    return q, h


def main():
    cfg = WaffleSidePick45MissionEnvCfg()
    cfg.scene.num_envs = 1
    cfg.box_shape_prob = a.box_prob
    cfg.near_start_prob = 0.0
    cfg.dock_second_prob = 0.0
    cfg.episode_length_s = 1e6
    cfg.perc_bias = a.perc_bias / 1000.0
    cfg.perc_jitter = 0.0
    cfg.ready_noise = 0.0
    cfg.act_delay_steps = tuple(int(v) for v in a.act_delay.split(","))
    cfg.gripper_closed = FINGER_CLOSE
    if a.seed >= 0:
        cfg.seed = a.seed
    from waffle_isaac.dice2_cfg import puck_cfg
    for j in range(6):  # fill 명령용 팔레트 퍽 (원기둥 · 직육면체 6 개씩, 처음엔 장면 밖)
        setattr(cfg.scene, f"fillC{j}", puck_cfg(f"FillC{j}", "cyl", (0.15, 0.7, 0.2) if j % 2 else (0.85, 0.15, 0.15), (-0.8, 0.40 + 0.07 * j)))
        setattr(cfg.scene, f"fillB{j}", puck_cfg(f"FillB{j}", "box", (0.85, 0.15, 0.15) if j % 2 else (0.15, 0.7, 0.2), (-0.9, 0.40 + 0.07 * j)))
    env = gym.make("Waffle-SidePick45Mission-Direct", cfg=cfg)
    u = env.unwrapped
    env.reset()
    r, dev, dt = u.robot, u.device, u.physics_dt
    arm, gl, gr = u._arm, u._gl, u._gr
    pol = torch.jit.load(os.path.join(HERE, a.policy), map_location=dev).eval()
    q_ready = torch.tensor([[READY_POSE_45[f"joint{i}"] for i in range(1, 6)]], device=dev)

    rclpy.init()
    node = rclpy.create_node("sim_skill_server")
    pub = node.create_publisher(String, "/sim_skill/result", 10)
    queue = []
    node.create_subscription(String, "/sim_skill/cmd", lambda m: queue.append(m.data.strip()), 10)

    ALL_PUCKS = ["puckT_cyl", "puckT_box", "puckN"] + [f"fillC{j}" for j in range(6)] + [f"fillB{j}" for j in range(6)]
    fill_used = {}

    def sk_fill(names):
        """팔레트 칸에 퍽 직접 채우기 (칸 모양대로, 계획기 #239 처럼). names = ["3_0", "2_1", ...]"""
        root = r.data.root_link_pos_w.torch[0]
        rq = r.data.root_link_quat_w.torch
        done = []
        for nm_ in names:
            row, idx = (int(v) for v in nm_.split("_"))
            sl = SLOTS.get((row, idx))
            if sl is None or nm_ in fill_used.values():
                continue
            pool = "fillB" if sl["shape"] == "box" else "fillC"
            obj = next((f"{pool}{j}" for j in range(6) if f"{pool}{j}" not in fill_used), None)
            if obj is None:
                continue
            pw = root + quat_apply(rq, torch.tensor([[sl["x"], sl["y"], sl["floor_z"] + PUCK_H / 2 + 0.001]], device=dev))[0]
            h = math.radians(sl["theta_deg"]) / 2 if sl["shape"] == "box" else 0.0
            pose = torch.tensor([[float(pw[0]), float(pw[1]), float(pw[2]), 0.0, 0.0, math.sin(h), math.cos(h)]], device=dev)   # (x, y, z, w) 순서 쿼터니언
            o = u._obj(obj)
            o.write_root_pose_to_sim_index(root_pose=pose, env_ids=torch.tensor([0], device=dev))
            o.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=dev), env_ids=torch.tensor([0], device=dev))
            fill_used[obj] = nm_; done.append(f"{nm_}({sl['shape']})")
        phys(st["hold_q"], st["hold_f"], int(1.0 / dt))
        return bool(done), "채움 " + " ".join(done) if done else "채울 칸 없음 (이미 찼거나 이름 틀림)"

    st = {"hold_q": r.data.joint_pos.torch[:, arm].clone(), "hold_f": float(OPEN_M), "cnt": 0, "bias": torch.zeros(1, 5, device=dev)}

    def phys(q, f, n=1):
        """팔 목표 q (1, 5) · 손가락 f (m) 로 물리 n 스텝 (창은 4 스텝마다)."""
        for _ in range(n):
            r.actuators.target_command.set_position_index(value=q.contiguous(), joint_ids=arm)
            r.actuators.target_command.set_position_index(value=torch.full((1, 2), float(f), device=dev), joint_ids=gl + gr)
            u.scene.write_data_to_sim()
            st["cnt"] += 1
            u.sim.step(render=(not a.headless) and st["cnt"] % 4 == 0)
            u.scene.update(dt=dt)
            if st["cnt"] % 30 == 0:
                rclpy.spin_once(node, timeout_sec=0.0)

    def held():
        """쥔 퍽이 있음 = 어느 퍽이든 잡는 점 3 cm 안 + 손가락 닫힘 쪽 (노드 쥠 판정 대신 시뮬은 위치로)."""
        g = u.grasp_w()[0]
        near = min(float(torch.linalg.norm(u._obj(nm).data.root_pos_w.torch[0] - g)) for nm in ALL_PUCKS)
        return near < 0.03 and finger() < GRIP_RANGE[1]

    def finger():
        return float(r.data.joint_pos.torch[0, gl[0]])

    def qnow():
        return r.data.joint_pos.torch[:, arm].clone()

    def pucks():
        """[(이름, base_link 위치, 직육면체?)] 상자 위 · 팔레트 · 손 안 구분 없이 지금 쓰는 퍽 둘."""
        out = []
        tgt = "puckT_box" if bool(u.is_box[0]) else "puckT_cyl"
        for nm, isb in ((tgt, bool(u.is_box[0])), ("puckN", bool(u.is_box[0]))):
            out.append((nm, u.to_base(u._obj(nm).data.root_pos_w.torch)[0], isb))
        return out

    def box_top():
        b = u._obj("boxA").data
        return u.to_base(b.root_pos_w.torch)[0], b.root_quat_w.torch

    def on_box(p_b):
        bc, _ = box_top()
        return abs(float(p_b[2] - (bc[2] + BOX_H / 2 + PUCK_H / 2))) < 0.012 and float(torch.linalg.norm(p_b[:2] - bc[:2])) < 0.12

    def stream(knots, grip_close_at=None):
        """노드 SagIntegrator 스트리밍: knots = [(q 5, 걸리는 s, 손가락 m)] 를 처짐 보정 (bias += ki dt (계획 − 측정)) 으로 따라감."""
        q0 = qnow()[0].cpu().numpy()
        pts = [(q0, 0.0, st["hold_f"])]
        T = [0.0]
        for q, d, f in knots:
            pts.append((np.asarray(q, dtype=np.float64), d, f)); T.append(T[-1] + max(d, 1e-3))
        bias = st["bias"]
        steps = int(T[-1] / dt) + 1
        for s_ in range(steps):
            t = s_ * dt
            j = max(0, min(len(T) - 2, int(np.searchsorted(T, t, side="right")) - 1))
            al = min(1.0, max(0.0, (t - T[j]) / max(T[j + 1] - T[j], 1e-6)))
            plan = pts[j][0] + (pts[j + 1][0] - pts[j][0]) * al
            f = pts[j][2] + (pts[j + 1][2] - pts[j][2]) * al
            plan_t = torch.tensor(plan, device=dev, dtype=torch.float32)[None]
            phys(plan_t + bias, f)
            bias = (bias + KI * dt * (plan_t - qnow())).clamp(-BIAS_MAX, BIAS_MAX)
        st["bias"] = bias
        st["hold_q"] = torch.tensor(pts[-1][0], device=dev, dtype=torch.float32)[None] + bias
        st["hold_f"] = pts[-1][2]

    def path_time(qs, speed, start):
        out, prev = [], np.asarray(start)
        for q in qs:
            q = np.asarray(q); out.append(max(0.02, float(np.max(np.abs(q - prev))) / speed)); prev = q
        return out

    # ---- 스킬 ----
    def sk_transit():
        q0 = qnow()[0].cpu().numpy()
        stream([(TRANSIT, max(1.0, float(np.max(np.abs(np.asarray(TRANSIT) - q0))) / 0.5), st["hold_f"])])
        return True, "transit"

    def sk_pick():
        cand = [(nm, p, isb) for nm, p, isb in pucks() if on_box(p)]
        if not cand:
            return False, "상자 위 퍽 없음"
        cand.sort(key=lambda c: float(torch.linalg.norm(c[1][:2] - torch.tensor([J1X, 0.0], device=dev))))
        nm, p0, isb = cand[0]
        u.tgt_n = torch.tensor([nm == "puckN"], device=dev)
        u.nb_present[:] = len(cand) > 1
        u.puck_z0[:] = u._obj(nm).data.root_pos_w.torch[0, 2]
        u.perc_bias[:] = (torch.rand(1, 3, device=dev) * 2 - 1) * a.perc_bias / 1000.0
        # 시작 자세 (READY_POSE_45) + 벌림, 처짐 보정 없이 sendArm 처럼
        q0 = qnow()
        T = max(1.0, float((q_ready - q0).abs().max()) / 0.5)
        for s_ in range(int(T / dt)):
            phys(q0 + (q_ready - q0) * min(1.0, (s_ + 1) * dt / T), OPEN_M)
        st["bias"].zero_()
        phys(q_ready, OPEN_M, int(0.5 / dt))
        u.actions[:] = 0.0; u.actions[:, -1] = 1.0; u.prev_actions[:] = u.actions
        u._act_buf = None
        if hasattr(u, "close_cnt"):
            u.close_cnt = torch.zeros_like(u.close_cnt)
        u.arm_target[:] = qnow()
        prev = u.arm_target.clone()
        hold, max_dq = 0, a.max_joint_vel / 30.0
        for k in range(int(12.0 * 30)):
            with torch.inference_mode():
                act = pol(u._get_observations()["policy"]).clamp(-1.0, 1.0)
            u._pre_physics_step(act.clone())
            u.arm_target[:] = torch.clamp(u.arm_target, prev - max_dq, prev + max_dq)
            prev = u.arm_target.clone()
            for _ in range(u.cfg.decimation):
                u._apply_action()
                u.scene.write_data_to_sim()
                st["cnt"] += 1
                u.sim.step(render=(not a.headless) and st["cnt"] % 4 == 0)
                u.scene.update(dt=dt)
            rclpy.spin_once(node, timeout_sec=0.0)
            f = finger()
            lift = float(u._obj(nm).data.root_pos_w.torch[0, 2] - u.puck_z0[0])
            held = GRIP_RANGE[0] < f < GRIP_RANGE[1] and lift > 0.03 and float(u.applied_actions[0, -1]) <= 0
            hold = hold + 1 if held else 0
            if hold >= 30:
                st["hold_q"] = qnow(); st["hold_f"] = FINGER_CLOSE; st["bias"].zero_()
                return True, f"{'직육면체' if isb else '원기둥'} {nm} 쥠 ({k / 30:.1f} s, 들림 {lift * 1000:.0f} mm, 손가락 {f:.4f})"
        st["hold_q"] = qnow(); st["hold_f"] = FINGER_CLOSE if float(u.applied_actions[0, -1]) <= 0 else OPEN_M
        return False, f"12 s 안에 못 쥠 (손가락 {finger():.4f})"

    def seg(sl, name):
        return next(g for g in sl["segments"] if g["name"] == name)["q"]

    def sk_pallet(row, idx, load):
        sl = SLOTS.get((row, idx))
        if sl is None:
            return False, f"칸 {row}_{idx} 없음"
        if load and not held():
            return False, "쥔 퍽 없음 (pallet_load)"
        q0 = qnow()[0].cpu().numpy()
        if load:
            qin = seg(sl, "rotate") + seg(sl, "carry") + seg(sl, "place")
            qout = seg(sl, "retract") + seg(sl, "return") + seg(sl, "rotate_back")
            sp_in, g_in, g_out = (ROW3_HOLD if row == 3 else PALLET_SPEED), FINGER_CLOSE, RELEASE_M
        else:
            qin = seg(sl, "rotate_back")[::-1] + seg(sl, "return")[::-1] + seg(sl, "retract")[::-1] + seg(sl, "release")
            qout = seg(sl, "place")[::-1] + seg(sl, "carry")[::-1] + seg(sl, "rotate")[::-1] + [sl.get("q_front", TRANSIT)]
            sp_in, g_in, g_out = PALLET_SPEED, RELEASE_M, FINGER_CLOSE
        knots = [(q0, SETTLE, st["hold_f"])]
        for q, d in zip(qin, path_time(qin, sp_in, q0)):
            knots.append((q, d, g_in))
        knots.append((qin[-1], DWELL, g_in))
        knots.append((qin[-1], GRIP_T, g_out))
        if not load:
            knots.append((qin[-1], 0.4, g_out))   # gripper.settle (쥠 확인)
        for q, d in zip(qout, path_time(qout, PALLET_SPEED, qin[-1])):
            knots.append((q, d, g_out))
        stream(knots)
        if not load and not held():
            return False, f"꺼냈지만 빈손 (손가락 {finger():.4f})"
        return True, f"{'넣음' if load else '꺼냄'} {row}_{idx}"

    def sk_place(x, y, z):
        if not held():
            return False, "쥔 퍽 없음 (place)"
        tip = np.array([x, y + LATERAL, z + PUCK_H / 2 + 0.015 + 0.002])
        _, h = ik(tip)
        d = math.cos(math.radians(PITCH)) * h - math.sin(math.radians(PITCH)) * np.array([0, 0, 1.0])
        pre, back = tip - d * 0.04, tip - d * 0.04
        q0 = qnow()[0].cpu().numpy()
        q_pre, _ = ik(pre, q0)
        if q_pre is None:
            return False, "IK 안 됨 (pre)"
        knots = [(q_pre, max(1.0, float(np.max(np.abs(q_pre - q0))) / 0.5), FINGER_CLOSE)]
        last = q_pre
        for p0, p1, g in ((pre, tip, FINGER_CLOSE), (tip, back, RELEASE_M)):
            n_ = max(1, math.ceil(np.linalg.norm(p1 - p0) / 0.01))
            for i in range(1, n_ + 1):
                q, _ = ik(p0 + (p1 - p0) * i / n_, last)
                if q is None:
                    return False, "IK 안 됨 (직선)"
                knots.append((q, 0.2, g)); last = q
            if p1 is tip:
                knots.append((last, GRIP_T, RELEASE_M)); knots.append((last, DWELL, RELEASE_M))
        knots.append((TRANSIT, max(1.0, float(np.max(np.abs(np.asarray(TRANSIT) - last))) / 0.5), RELEASE_M))
        stream(knots)
        return True, f"놓음 ({x:.3f}, {y:.3f}) 윗면 {z:.3f}"

    def cells():
        bc, bq = box_top()
        top = float(bc[2] + BOX_H / 2)
        ax = u.vec_base(quat_apply(bq, torch.tensor([[1.0, 0, 0]], device=dev)))[0]
        ay = u.vec_base(quat_apply(bq, torch.tensor([[0, 1.0, 0]], device=dev)))[0]
        out = {}
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                p = bc + 0.06 * (dx * ax + dy * ay)
                out[f"{dx},{dy}"] = {"x": round(float(p[0]), 4), "y": round(float(p[1]), 4), "z": round(top, 4)}
        return out

    def status():
        d = {nm: {"xyz_base": [round(float(v), 3) for v in p], "상자위": on_box(p), "직육면체": isb} for nm, p, isb in pucks()}
        for obj, sl_ in fill_used.items():
            p = u.to_base(u._obj(obj).data.root_pos_w.torch)[0]
            d[obj] = {"채운칸": sl_, "xyz_base": [round(float(v), 3) for v in p], "상자위": on_box(p)}
        return d | {"손가락": round(finger(), 4)}

    def run(cmd):
        t0 = time.time()
        try:
            if cmd == "toppick_tilt":
                ok, msg = sk_pick()
            elif cmd.startswith("pallet_load_") or cmd.startswith("pallet_unload_"):
                f = cmd.split("_")
                ok, msg = sk_pallet(int(f[2]), int(f[3]), f[1] == "load")
            elif cmd.startswith("place_at:"):
                f = cmd.split(":")
                ok, msg = sk_place(float(f[2]), float(f[3]), float(f[4]))
            elif cmd == "clear":  # 상자 위에 남은 집을 퍽을 치움 (같은 상자를 놓을 장소로 쓸 때)
                n_ = 0
                for nm_, p_, _ in pucks():
                    if on_box(p_):
                        o_ = u._obj(nm_); pose_ = o_.data.root_pose_w.torch.clone(); pose_[0, 0] -= 1.2; pose_[0, 2] = 0.05
                        o_.write_root_pose_to_sim_index(root_pose=pose_, env_ids=torch.tensor([0], device=dev)); n_ += 1
                phys(st["hold_q"], st["hold_f"], int(0.5 / dt))
                ok, msg = True, f"상자 위 퍽 {n_} 개 치움"
            elif cmd.startswith("fill"):
                ok, msg = sk_fill(cmd.split()[1:])
            elif cmd == "transit":
                ok, msg = sk_transit()
            elif cmd == "reset":
                env.reset(); fill_used.clear(); st["bias"].zero_(); st["hold_q"] = qnow(); st["hold_f"] = OPEN_M; u.tgt_n = None
                ok, msg = True, "다시 놓음"
            elif cmd == "cells":
                ok, msg = True, json.dumps(cells(), ensure_ascii=False)
            elif cmd == "status":
                ok, msg = True, json.dumps(status(), ensure_ascii=False)
            else:
                ok, msg = False, f"모르는 명령: {cmd}"
        except Exception as e:  # noqa: BLE001
            ok, msg = False, f"오류: {e}"
        res = json.dumps({"cmd": cmd, "ok": ok, "msg": msg, "t": round(time.time() - t0, 1)}, ensure_ascii=False)
        print(f"[SimSkill] {res}", flush=True)
        pub.publish(String(data=res))

    print("[SimSkill] 준비됨: /sim_skill/cmd 를 기다림", flush=True)
    while app.is_running():
        if queue:
            run(queue.pop(0))
        else:
            phys(st["hold_q"], st["hold_f"], 4)
            rclpy.spin_once(node, timeout_sec=0.0)


if __name__ == "__main__":
    try:
        main()
    finally:
        app.close()
