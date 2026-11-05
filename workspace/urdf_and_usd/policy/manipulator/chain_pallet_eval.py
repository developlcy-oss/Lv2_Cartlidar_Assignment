"""집기 -> 팔레트 칸 넣기 이어서 평가 (Isaac PhysX): 학습한 TopPick 정책으로 상자 위 퍽을 집은 뒤, 칸 고정 궤적으로 팔레트에 넣음.

toppick_eval.py (집기만) 와 pallet_place_check.py (퍽을 손에 넣어 주고 놓기만) 사이의 빈칸 = 정책이 쥔 그대로의 퍽 (손 안 어긋남) 이
칸에 들어가는지 확인 (정책_학습기록 4.15 절 "남은 것: 집기 -> 앞 운반 자세 -> 놓기 이어서").

대마다 (대 i -> 칸 i % 12, 칸 순서 = pallet_place_traj.json slots = 채우는 순서):
  1. 집기 (정책): TopPick 환경 그대로 에피소드 1 번 (6 s, 학습과 같은 배치 · 인식 · 무작위화). 성공 = 환경 판정 (1 s 세워 들기 + 옆 퍽 그대로)
     + 끝에 아직 쥐고 있음. 실패한 대는 여기서 끝 (집기 실패로 셈).
  2. 넘겨받기: 정책 끝 자세에서 칸의 앞 운반 자세 (q_front, 12칸 모두 45° 판) 까지 관절 직선 (--transit_speed), 쥔 채.
     처짐 보정 (목표 = 계획 + bias, bias += ki · dt · (계획 − 측정), ROS sag_comp 와 같은 식) 은 넘겨받는 순간부터 켬,
     q_front 에서 --settle 동안 머묾 (ROS manipulator_node 의 sag.settle_time 과 같음).
  3. 칸 궤적: pallet_timeline.timeline (pallet_place_check 와 같은 궤적 · 속도) -> 놓기 -> 벌림 -> 빼기 -> 1 s 기다림.
  4. 판정 (pallet_place_check 와 같음): 퍽 바닥 가운데 - 칸 축 3 mm 안, 바닥 높이 ±3 mm, 기울기 5° 안, 옮기는 중 미끄럼 15 mm 안.

손 안 어긋남 (쥔 퍽 중심 − 기대 중심 = TCP − 퍽 축 x 15 mm, 집게 좌표 성분 mm) 을 넘겨받을 때 · q_front · 놓기 직전에 잼:
  close = 손가락 닫는 축 (gripper y), appr = 집게 방향 (gripper x), side = 나머지 (gripper z), tilt = 퍽 축 − 기대 축 [°],
  face = 상자 퍽 면 − 닫는 축 [°] (상자 퍽만, 90° 주기). 운반 중 최대 미끄럼 (넘겨받을 때 대비 변화) 도.

    ./run_rl.sh chain_eval [대 수] [--seed N] [--policy 경로] ...      -> chain_pallet_eval.json (+ 학습기록_자동.md 한 줄)
    ./run_rl.sh chain_eval_play [대 수]                                 창으로 (12 대 = 칸마다 1 대)

물리 (놓기 확인과 같게): 퍽 마찰 --puck_mu 0.5 (결합 min, 퍽-팔레트 · 퍽-손가락 모두 0.5), 접촉 거리 --contact_offset 2 mm.
  그래서 무작위화 기본 --dr -friction (마찰 무작위화가 시작 때 재질을 덮어쓰므로 끔. 나머지 이득 · 질량 · 지연 · 관측 잡음은 켬).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--num_envs", type=int, default=192, help="대 수 (12 의 배수면 칸마다 같은 수)")
ap.add_argument("--policy", default=None, help="TorchScript 정책 (기본 policies/toppick_latest.pt)")
ap.add_argument("--seed", type=int, default=2, help="평가 시드 (체크포인트 고를 때 1, 보고 2 = #71)")
ap.add_argument("--layout", default="mix", choices=["mix", "rand", "A", "B", "C"], help="집기 배치 (toppick_eval 과 같음)")
ap.add_argument("--latch", type=float, default=None, help="인식 좌표 고정 비율 (기본 = 환경 설정 0.7)")
ap.add_argument("--dr", default="-friction", help="물리 무작위화 (toppick_eval --dr 과 같은 형식). 기본 = 마찰만 끔 (--puck_mu 를 쓰려고)")
ap.add_argument("--puck_mu", type=float, default=0.5, help="퍽 마찰 (정지 = 미끄럼, 결합 min). 0 이하 = 팀 재질 그대로")
ap.add_argument("--contact_offset", type=float, default=0.002, help="[m] 로봇 · 퍽 접촉 거리 (pallet_place_check 와 같음). 0 = 팀 값 그대로")
ap.add_argument("--transit_speed", type=float, default=0.5, help="[rad/s] 집은 자세 -> 앞 운반 자세 관절 속도")
ap.add_argument("--settle", type=float, default=1.5, help="[s] 앞 운반 자세에서 처짐 보정 머묾")
ap.add_argument("--ki", type=float, default=4.0, help="[1/s] 처짐 보정 적분 (0 = 끔)")
ap.add_argument("--bias_max", type=float, default=0.15, help="[rad] 처짐 보정 한도")
ap.add_argument("--speed", type=float, default=0.8, help="[rad/s] 칸 궤적 기본 속도 (pallet_place_check 와 같은 뜻)")
ap.add_argument("--tilt_speed", type=float, default=0.8)
ap.add_argument("--row_speed", default="3:0.4")
ap.add_argument("--empty_speed", type=float, default=0.8)
ap.add_argument("--grip_close", type=float, default=None, help="넘겨받은 뒤 손가락 목표 (기본 = 정책과 같은 환경 gripper_closed −0.0075)")
ap.add_argument("--gui", action="store_true")
ap.add_argument("--out", default=None, help="결과 JSON (기본 chain_pallet_eval.json)")
AppLauncher.add_app_launcher_args(ap)
a = ap.parse_args()
a.headless = not a.gui
app = AppLauncher(a).app
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.sensors import ContactSensorCfg  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_apply_inverse  # noqa: E402

import waffle_isaac  # noqa: E402,F401
from pallet_timeline import PHASES as TL_PHASES, Speeds, joint_line, parse_row_speed, timeline  # noqa: E402
from waffle_isaac.dice2_cfg import GRASP_OFFSET_A5, PUCK_H  # noqa: E402
from waffle_isaac.robot_cfg import ARM_LINKS_REGEX, BODY_LINK, FINGER_LINKS_REGEX, FINGER_RELEASED  # noqa: E402
from waffle_isaac.toppick_env import LAYOUTS  # noqa: E402
from waffle_isaac.toppick_env_cfg import WaffleTopPickEnvCfg, set_dr  # noqa: E402

TRAJ = json.load(open(os.path.join(HERE, "pallet_place_traj.json"), encoding="utf-8"))
SLOTS = TRAJ["slots"]
GRASP_DZ = TRAJ["grasp_dz"]
PHASES = ["pick", "transit"] + TL_PHASES[1:]  # 이어 평가 단계: 0 집기, 1 넘겨받기 (+ 머묾), 2 carry, 3 place, 4 release, 5 after
OUT = a.out or os.path.join(HERE, "chain_pallet_eval.json")
SP = Speeds(a.speed, a.tilt_speed, parse_row_speed(a.row_speed), a.empty_speed)
BODY = "{ENV_REGEX_NS}/Robot/" + BODY_LINK

# ---- 환경 설정 (TopPick 그대로 + 접촉 센서 · 퍽 재질) ----
cfg = WaffleTopPickEnvCfg()
cfg.scene.num_envs = a.num_envs
cfg.seed = a.seed
cfg.near_start_prob = 0.0
cfg.tilt_prob = 1.0 if all(s.get("tilt_deg") for s in SLOTS) else cfg.tilt_prob
assert all(s.get("tilt_deg") for s in SLOTS), "칸 궤적에 수직 칸이 있음: 대마다 집기 모드를 칸에 맞추는 코드가 필요 (지금은 12칸 모두 45°)"
set_dr(cfg, a.dr)
if a.latch is not None:
    cfg.perc_latch_prob = a.latch
if a.layout == "rand":
    cfg.layout_prob = 0.0
elif a.layout != "mix":
    cfg.layout_prob = 1.0
    cfg.layout_weights = tuple(1.0 if k == a.layout else 0.0 for k in LAYOUTS[1:])
for k in ("puckT_cyl", "puckT_box", "puckN"):
    o = getattr(cfg.scene, k)
    sp = o.spawn.replace(activate_contact_sensors=True)
    if a.contact_offset > 0:
        sp = sp.replace(collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=a.contact_offset, rest_offset=0.0))
    if a.puck_mu > 0:
        sp = sp.replace(physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=a.puck_mu, dynamic_friction=a.puck_mu,
                                                                        friction_combine_mode="min"))
    setattr(cfg.scene, k, o.replace(spawn=sp))
if a.contact_offset > 0:
    r = cfg.scene.robot
    cfg.scene.robot = r.replace(spawn=r.spawn.replace(collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=a.contact_offset, rest_offset=0.0)))


def _touch(prim, other=BODY):
    return ContactSensorCfg(prim_path=prim, filter_prim_paths_expr=[other], update_period=0.0, history_length=1)


cfg.scene.arm_body = _touch("{ENV_REGEX_NS}/Robot/" + ARM_LINKS_REGEX)
cfg.scene.finger_body = _touch("{ENV_REGEX_NS}/Robot/" + FINGER_LINKS_REGEX)
cfg.scene.puckC_body = _touch("{ENV_REGEX_NS}/PuckT_cyl")
cfg.scene.puckB_body = _touch("{ENV_REGEX_NS}/PuckT_box")
if a.gui:
    cfg.viewer.eye, cfg.viewer.lookat = (0.55, -0.65, 0.6), (0.1, 0.0, 0.15)


def main():
    env = gym.make("Waffle-TopPick-Direct", cfg=cfg)
    u = env.unwrapped
    n, dev = u.num_envs, u.device
    dt = u.physics_dt
    path = a.policy or os.path.join(HERE, "policies", "toppick_latest.pt")
    pol = torch.jit.load(path, map_location=dev).eval()
    grip_close = cfg.gripper_closed if a.grip_close is None else a.grip_close
    slot_of = [SLOTS[i % len(SLOTS)] for i in range(n)]
    ex_, ey_, ez_ = (torch.tensor([v], device=dev).expand(n, 3) for v in ([1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]))

    def force(k):
        return torch.linalg.norm(u.scene[k].data.normal_force_matrix_w.torch, dim=-1).amax(dim=(1, 2))

    def in_hand():
        """쥔 퍽 중심 − 기대 중심 (집게 좌표 mm: close · appr · side), 퍽 축 기울기 [°], 상자 면 − 닫는 축 [°]. 기울임 모드: 퍽 축 = 차체 위."""
        d = u.robot.data
        q = d.body_link_quat_w.torch[:, u._ee]
        gx, gy, gz = quat_apply(q, ex_), quat_apply(q, ey_), quat_apply(q, ez_)
        tcp = d.body_link_pos_w.torch[:, u._ee] + quat_apply(q, u._off)
        up = torch.where(u.tilted[:, None], quat_apply(d.root_link_quat_w.torch, ez_), -gx)
        pc, pq = u.puck_pos_w(), u.puck_quat_w()
        e = pc - (tcp - up * GRASP_DZ)
        pz = quat_apply(pq, ez_)
        pz = pz * torch.where((pz * up).sum(1, keepdim=True) < 0, -1.0, 1.0)
        tilt = torch.rad2deg(torch.acos((pz * up).sum(1).clamp(-1.0, 1.0)))
        px = quat_apply(pq, ex_)
        px = px - up * (px * up).sum(1, keepdim=True)  # 퍽 x 를 퍽 축에 수직인 면으로
        yv = gy - up * (gy * up).sum(1, keepdim=True)
        ang = torch.rad2deg(torch.atan2((torch.cross(yv, px, dim=1) * up).sum(1), (yv * px).sum(1)))
        face = torch.remainder(ang + 45.0, 90.0) - 45.0
        return torch.stack(((e * gy).sum(1), (e * gx).sum(1), (e * gz).sum(1)), dim=1) * 1000.0, tilt, face, e

    # ---- 1. 집기 (정책, toppick_eval 과 같은 판정) ----
    t0 = time.time()
    obs, _ = env.reset()
    tilted, box, lay, latched = u.tilted.clone(), u.is_box.clone(), u.layout.clone(), u.latched.clone()
    done = torch.zeros(n, dtype=torch.bool, device=dev)
    ok = torch.zeros_like(done)
    for _ in range(int(cfg.episode_length_s / u.step_dt) - 2):
        with torch.inference_mode():
            act = pol(obs["policy"])
        ok |= u.succeeded & ~done
        obs, _, term, trunc, _ = env.step(act)
        done |= term | trunc
    ok |= u.succeeded & ~done
    off_h, tilt_h, face_h, e_h = in_hand()
    holding = (~done) & (torch.linalg.norm(e_h, dim=1) < 0.015) & (u.lift() > cfg.lift_min)
    picked = ok & holding
    print(f"[Chain] 집기 {int(picked.sum())}/{n} (환경 성공 {int(ok.sum())}, 끝에 놓침 {int((ok & ~holding).sum())}) {time.time() - t0:.0f} s", flush=True)

    # ---- 2 ~ 3. 넘겨받기 + 칸 궤적 (계획을 대마다 만들어 (n, T, 5) 로) ----
    q_meas = u.robot.data.joint_pos.torch[:, u._arm].clone()
    plans, t_front, t_place = [], [], []
    n_settle = int(a.settle / dt)
    for i in range(n):
        s = slot_of[i]
        q0 = q_meas[i].tolist()
        qf = list(s.get("q_front", TRAJ["transit"]["q_front_tilt"]))
        tr = [(q, 0, 1, -1) for q in joint_line(q0, qf, a.transit_speed, dt)] + [(qf, 0, 1, -1)] * n_settle
        tl = [(q, g, ph + 1, sg) for q, g, ph, sg in timeline(s, dt, SP, TRAJ["transit"]["q_front"])]
        if not bool(picked[i]):  # 집기 실패: 제자리 (결과에서 뺌)
            tr, tl = [(q0, 1, 0, -1)], []
        plans.append(tr + tl)
        t_front.append(len(tr) - 1)
        pl = [k for k, x in enumerate(tl) if x[2] == 3]
        t_place.append(len(tr) + (pl[-1] if pl else len(tl) - 1))
    T = max(len(p) for p in plans)
    tgt = torch.tensor([[p[min(t, len(p) - 1)][0] for t in range(T)] for p in plans], device=dev)  # (n, T, 5)
    grp = torch.tensor([[p[min(t, len(p) - 1)][1] for t in range(T)] for p in plans], device=dev)
    phs = torch.tensor([[p[min(t, len(p) - 1)][2] for t in range(T)] for p in plans], device=dev)
    t_front = torch.tensor(t_front, device=dev)
    t_place = torch.tensor(t_place, device=dev)
    print(f"[Chain] 넘겨받기 + 칸 궤적 {T} 스텝 ({T * dt:.1f} s)", flush=True)

    cmd = u.robot.actuators.target_command
    wheels0 = torch.zeros(n, len(u._wheels), device=dev)
    bias = torch.zeros(n, 5, device=dev)
    keep = ~picked  # 집기 실패 대: 지금 관절 목표 · 손가락 그대로
    arm_keep, grip_keep = u.arm_target.clone(), torch.where(u.actions[:, -1:] > 0.0, cfg.gripper_open, cfg.gripper_closed)
    off_f, tilt_f, face_f = off_h.clone(), tilt_h.clone(), face_h.clone()
    off_p, tilt_p, face_p = off_h.clone(), tilt_h.clone(), face_h.clone()
    slip = torch.zeros(n, device=dev)
    fmax = {k: torch.zeros(n, len(PHASES), device=dev) for k in ("arm", "finger", "puck", "arm_box")}
    for t in range(T):
        if a.ki > 0:
            bias = torch.clamp(bias + a.ki * dt * (tgt[:, t] - u.robot.data.joint_pos.torch[:, u._arm]), -a.bias_max, a.bias_max)
        arm = torch.where(keep[:, None], arm_keep, tgt[:, t] + bias)
        g = torch.where(grp[:, t:t + 1] > 0, FINGER_RELEASED, grip_close)
        g = torch.where(keep[:, None], grip_keep, g)
        cmd.set_position_index(value=arm.contiguous(), joint_ids=u._arm)
        cmd.set_position_index(value=g.expand(-1, 2).contiguous(), joint_ids=u._gl + u._gr)
        cmd.set_velocity_index(value=wheels0, joint_ids=u._wheels)
        u.scene.write_data_to_sim()
        u.sim.step(render=a.gui)
        u.scene.update(dt)
        ph = phs[:, t]
        fp = torch.where(u.is_box, force("puckB_body"), force("puckC_body"))
        fab = torch.linalg.norm(u.scene["arm_box_contact"].data.normal_force_matrix_w.torch, dim=-1).amax(dim=(1, 2))
        for k, f in (("arm", force("arm_body")), ("finger", force("finger_body")), ("puck", fp), ("arm_box", fab)):
            fmax[k].scatter_reduce_(1, ph[:, None], f[:, None], reduce="amax")
        held = (ph >= 1) & (ph <= 3) & picked
        mf, mp = (t == t_front) & picked, (t == t_place) & picked
        if held.any() or mf.any() or mp.any():
            o_, ti_, fa_, e_ = in_hand()
            slip = torch.where(held, torch.maximum(slip, torch.linalg.norm(e_ - e_h, dim=1)), slip)
            off_f, tilt_f, face_f = (torch.where(mf[:, None], o_, off_f), torch.where(mf, ti_, tilt_f), torch.where(mf, fa_, face_f))
            off_p, tilt_p, face_p = (torch.where(mp[:, None], o_, off_p), torch.where(mp, ti_, tilt_p), torch.where(mp, fa_, face_p))
        if t % 480 == 0:
            print(f"[Chain] t={t * dt:5.1f} s 단계 {[PHASES[int(v)] for v in ph[:min(n, 12)].tolist()]}", flush=True)

    # ---- 4. 판정 (base_link 좌표, pallet_place_check 와 같은 기준) ----
    d = u.robot.data
    rp, rq = d.root_link_pos_w.torch, d.root_link_quat_w.torch
    pc, pq = u.puck_pos_w(), u.puck_quat_w()
    ax = quat_apply(pq, ez_)
    ax = ax * torch.where(ax[:, 2:3] < 0, -1.0, 1.0)
    bot = quat_apply_inverse(rq, pc - ax * (PUCK_H / 2) - rp)
    pxb = quat_apply_inverse(rq, quat_apply(pq, ex_))
    res = []
    for i in range(n):
        s = slot_of[i]
        r = dict(env=i, slot=f"{s['row']}:{s['index']}", shape="box" if bool(box[i]) else "cyl", layout=LAYOUTS[int(lay[i])],
                 latched=bool(latched[i]), picked=bool(picked[i]), pick_env_success=bool(ok[i]))
        if not r["picked"]:
            r["stage"] = "pick_fail"
            res.append(r)
            continue
        dxy = math.hypot(float(bot[i, 0]) - s["x"], float(bot[i, 1]) - s["y"])
        dz = float(bot[i, 2]) - s["floor_z"]
        tilt = math.degrees(math.acos(min(1.0, float(ax[i, 2]))))
        yaw = math.degrees(math.atan2(float(pxb[i, 1]), float(pxb[i, 0])))
        seated = dxy < 0.003 and abs(dz) < 0.003 and tilt < 5.0 and float(slip[i]) < 0.015
        lost_transit = float(torch.linalg.norm(off_f[i])) > 15.0
        r.update(seated=seated, stage="seated" if seated else ("transit_drop" if lost_transit else ("carry_slip" if float(slip[i]) >= 0.015 else "not_seated")),
                 xy_err_mm=round(dxy * 1000, 1), bottom_above_floor_mm=round(dz * 1000, 1), tilt_deg=round(tilt, 1),
                 box_face_err_deg=round(((yaw - s["theta_deg"] + 45.0) % 90.0) - 45.0, 1) if bool(box[i]) else None,
                 max_slip_mm=round(float(slip[i]) * 1000, 1),
                 in_hand={k: dict(close_mm=round(float(o[i, 0]), 2), appr_mm=round(float(o[i, 1]), 2), side_mm=round(float(o[i, 2]), 2),
                                  tilt_deg=round(float(tl[i]), 2), face_deg=round(float(fc[i]), 2) if bool(box[i]) else None)
                          for k, o, tl, fc in (("handover", off_h, tilt_h, face_h), ("q_front", off_f, tilt_f, face_f), ("place", off_p, tilt_p, face_p))},
                 max_force_N={k: {PHASES[j]: round(float(fmax[k][i, j]), 1) for j in range(1, len(PHASES))} for k in fmax})
        res.append(r)

    # ---- 요약 ----
    def rate(rows, key="seated"):
        m = len(rows)
        k = sum(1 for x in rows if x.get(key))
        return dict(ok=k, n=m, pct=round(100.0 * k / m, 1) if m else None)

    def stat(vals):
        if not vals:
            return None
        v = sorted(abs(x) for x in vals)
        return dict(median=round(v[len(v) // 2], 2), p95=round(v[min(len(v) - 1, int(0.95 * len(v)))], 2), max=round(v[-1], 2))

    pk = [x for x in res if x["picked"]]
    per_slot = {}
    for s in SLOTS:
        key = f"{s['row']}:{s['index']}"
        rows = [x for x in res if x["slot"] == key]
        per_slot[key] = dict(chain=rate(rows), pick=rate(rows, "picked"), place_given_pick=rate([x for x in rows if x["picked"]]),
                             cyl=rate([x for x in rows if x["shape"] == "cyl"]), box=rate([x for x in rows if x["shape"] == "box"]),
                             theta_deg=s["theta_deg"])
    hand = {stg: {c: stat([x["in_hand"][stg][c] for x in pk if x["in_hand"][stg][c] is not None])
                  for c in ("close_mm", "appr_mm", "side_mm", "tilt_deg", "face_deg")} for stg in ("handover", "q_front", "place")}
    def hand_bin(x):
        h = x["in_hand"]["handover"]
        e = math.hypot(h["close_mm"], h["appr_mm"], h["side_mm"])
        return "<1mm" if e < 1 else "1-3mm" if e < 3 else ">=3mm"
    by_hand = {b: rate([x for x in pk if hand_bin(x) == b]) for b in ("<1mm", "1-3mm", ">=3mm")}
    stages = {k: sum(1 for x in res if x["stage"] == k) for k in ("pick_fail", "transit_drop", "carry_slip", "not_seated", "seated")}
    summary = dict(policy=os.path.relpath(path, HERE), seed=a.seed, layout=a.layout, dr=a.dr, num_envs=n, puck_mu=a.puck_mu,
                   ki=a.ki, transit_speed=a.transit_speed, settle_s=a.settle, speeds=dict(speed=a.speed, tilt=a.tilt_speed, row=a.row_speed, empty=a.empty_speed),
                   chain=rate(res), pick=rate(res, "picked"), place_given_pick=rate(pk), stages=stages,
                   by_shape={sh: rate([x for x in res if x["shape"] == sh]) for sh in ("cyl", "box")},
                   by_layout={ly: rate([x for x in res if x["layout"] == ly]) for ly in LAYOUTS},
                   seated_by_handover_offset=by_hand, in_hand=hand,
                   arm_body_max_N=round(float(fmax["arm"][picked].max()) if picked.any() else 0.0, 1),
                   arm_box_transit_max_N=round(float(fmax["arm_box"][picked, 1].max()) if picked.any() else 0.0, 1),
                   wall_s=round(time.time() - t0, 1), date=time.strftime("%Y-%m-%d %H:%M"))
    c = summary["chain"]
    print(f"[Chain] 정책 {summary['policy']} | 시드 {a.seed} | 배치 {a.layout} | 무작위화 {a.dr}", flush=True)
    print(f"[Chain] 이어서 성공 {c['ok']}/{c['n']} ({c['pct']} %) = 집기 {summary['pick']['pct']} % x 넣기 {summary['place_given_pick']['pct']} % | 단계 {stages}", flush=True)
    print("[Chain] 칸별 (열:칸 이어서 / 집기 / 집은 것 중 넣기 | 원기둥 · 상자)", flush=True)
    for k, v in per_slot.items():
        f = lambda r: f"{r['ok']}/{r['n']}"
        print(f"[Chain]   {k} ({v['theta_deg']:+7.1f}°)  {f(v['chain']):>6s}  {f(v['pick']):>6s}  {f(v['place_given_pick']):>6s} | {f(v['cyl'])} · {f(v['box'])}", flush=True)
    for stg, v in hand.items():
        print(f"[Chain] 손 안 어긋남 {stg:8s} " + " ".join(f"{c_}: {v[c_]['median']}/{v[c_]['p95']}" for c_ in v if v[c_]) + " (중앙값/p95)", flush=True)
    print(f"[Chain] 넘겨받을 때 어긋남별 넣기 {by_hand} | 팔-몸체 최대 {summary['arm_body_max_N']} N, 넘겨받기 중 팔-상자 {summary['arm_box_transit_max_N']} N", flush=True)
    json.dump(dict(summary=summary, per_slot=per_slot, results=res), open(OUT, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"[Chain] 결과 {OUT}", flush=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        app.close()
