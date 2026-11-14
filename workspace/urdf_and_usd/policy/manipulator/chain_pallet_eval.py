"""집기 -> 팔레트 칸 넣기 이어서 평가 (Isaac PhysX): 학습한 TopPick 정책으로 상자 위 퍽을 집은 뒤, 칸 고정 궤적으로 팔레트에 넣음.

toppick_eval.py (집기만) 와 pallet_place_check.py (퍽을 손에 넣어 주고 놓기만) 사이의 빈칸 = 정책이 쥔 그대로의 퍽 (손 안 어긋남) 이
칸에 들어가는지 확인 (정책_학습기록 4.15 절 "남은 것: 집기 -> 앞 운반 자세 -> 놓기 이어서").

대마다 (대 i -> 칸 i % 12, 칸 순서 = pallet_place_traj.json slots = 채우는 순서):
  1. 집기 (정책): TopPick 환경 그대로 에피소드 1 번 (6 s, 학습과 같은 배치 · 인식 · 무작위화). 넘기는 대 = 물리 판정 (--gate physical, 기본):
     6 s 끝에 쥐고 (손 안 15 mm 안) 들고 (lift_min 위) 세워 (upright_min) 있고 옆 퍽 그대로 + 중간 종료 없음 — 환경 성공 기준과 무관해
     성공 기준이 다른 회차 (TP3 느슨 · TP4 손 안 4 mm) 를 같은 잣대로 비교. --gate env = 환경 성공 + 끝에 쥠 (2026-11-05 첫 판).
     넘기지 못한 대는 여기서 끝 (집기 실패로 셈).
  1'. 고정 동작 집기 (--scripted_pick σ, 2026-11-09): 정책 대신 ROS toppick_tilt (approach_mode top) 와 같은 경로. 잡는 점 = 잰 퍽
     (참값 + σ) + 잡는 높이, J1 에서 먼 쪽 −--regrasp_r. 그 위 5 cm · J1 쪽 3 cm 로 벌린 채 -> 직선으로 내려옴 -> 쥠 -> 5 cm 들기 (약 4.6 s).
     벌림 50 mm 시드 2: 171/192 (σ 0) · 144 (σ 2 mm), TP8 정책 57. --grip_open 0.0042 = 실물 벌림 50 mm (61.6 mm = 0.010 비례).
     2026-11-14: 들기 = ROS toppick.lift_height (--lift_h 0.085) · lift_max_radius (--lift_rmax 0.32, 그보다 먼 퍽은 들며 J1 쪽으로).
     예전 5 cm 는 옆 퍽 (60 mm) 보다 낮아 넘겨받기 관절 직선 중 쥔 퍽이 옆 퍽을 침. --lift_h 0.05 --lift_rmax 9 = 예전 노드.
  1b. 다시 쥐기 (--regrasp always | out, 기본 off, 2026-11-09): 처음 퍽 자리 위로 수평 이동 -> 내려놓기 -> 벌림 -> 잰 퍽 자리
     (참값 + --regrasp_noise σ) + 잡는 높이 − 목표 손 안 반경 (--regrasp_r) 로 옮기기 -> 쥠 -> 3 cm 들기 (약 4.7 s). 팔 이동은 환경 IK
     (_ik_step) 로 스텝마다 남은 오차의 --regrasp_gain 만. 놓친 대 = regrasp_drop. 대마다 regrasp_diag · drift_mm · in_hand.pick (다시 쥐기 전).
     TP8 시드 2: 이어서 57 -> 113/192, 측정 σ 1 · 2 · 3 mm 에서 107 · 94 · 84 (정책_학습기록 4.15 절).
  2. 넘겨받기: 정책 끝 자세에서 칸의 앞 운반 자세 (q_front, 12칸 모두 45° 판) 까지 관절 직선 (--transit_speed), 쥔 채.
     처짐 보정 (목표 = 계획 + bias, bias += ki · dt · (계획 − 측정), ROS sag_comp 와 같은 식) 은 넘겨받는 순간부터 켬,
     q_front 에서 --settle 동안 머묾 (ROS manipulator_node 의 sag.settle_time 과 같음).
  3. 칸 궤적: pallet_timeline.timeline (pallet_place_check 와 같은 궤적 · 속도) -> 놓기 -> 벌림 -> 빼기 -> 1 s 기다림.
  4. 판정 (pallet_place_check 와 같음): 퍽 바닥 가운데 - 칸 축 3 mm 안, 바닥 높이 ±3 mm, 기울기 5° 안, 옮기는 중 미끄럼 15 mm 안.

엄격 판정 (2026-11-14, 넣기 성공 + 아래 모두): 옆 퍽 (상자 위 남은 퍽) 이 처음 자리에서 3 mm 안 · 서 있음 · 팔이 닿지 않음 (집기부터 끝까지),
  --occ_all 이면 (--occ_rows planner: 목표 열 · 뒤 열의 다른 칸, all: 목표 칸 빼고 전부) 칸에 퍽 (실물은 색으로 칸을 정해 차례가 바뀜) -> 그 퍽들에 팔 · 쥔 퍽이 1 N 넘게 닿지 않고 3 mm 안.

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
ap.add_argument("--gate", default="physical", choices=["physical", "env"], help="칸 궤적으로 넘길 대: physical = 끝에 쥐고 들고 세움, env = 환경 성공 + 끝에 쥠")
ap.add_argument("--regrasp", default="off", choices=["off", "always", "out"], help="집은 뒤 다시 쥐기: 상자에 내려놓고 벌려 퍽 자리 (참값 + --regrasp_noise) 로 옮겨 다시 쥠. out = 손 안 반경이 --regrasp_win 밖일 때만")
ap.add_argument("--regrasp_r", type=float, default=-2.0, help="[mm] 손 안 반경 목표 (다시 쥐기 · 고정 집기 공용, + = J1 에서 먼 쪽, 놓기 확인에서 −4 ~ 0 이 24/24)")
ap.add_argument("--scripted_pick", type=float, default=None, help="[mm] 주면 정책 대신 고정 동작 집기 (ROS toppick_tilt approach_mode top 과 같은 경로), 값 = 퍽 자리 측정 오차 σ")
ap.add_argument("--grip_open", type=float, default=None, help="[m] 벌림 손가락 값 (기본 환경 gripper_open 0.010 = 61.6 mm, 실물 50 mm ≈ 0.0042)")
ap.add_argument("--regrasp_noise", type=float, default=0.0, help="[mm] 퍽 자리 · 손 안 반경 측정 오차 σ (카메라 가정)")
ap.add_argument("--regrasp_win", default="-5:1", help="[mm] out 모드: 잰 손 안 반경이 이 범위 밖이면 다시 쥠")
ap.add_argument("--regrasp_time", type=float, default=1.0, help="다시 쥐기 동작 시간 배율 (기본 내려놓기 1 · 벌림 0.5 · 옮기기 1 · 쥠 0.6 · 들기 1 s)")
ap.add_argument("--regrasp_gain", type=float, default=0.3, help="다시 쥐기 이동: 스텝마다 남은 위치 오차의 이 비율만 IK 로 (1 = 전부 -> 팔이 따라오기 전에 목표가 쌓여 ±4 mm 흔들림)")
ap.add_argument("--regrasp_home", type=int, default=1, help="1 = 처음 퍽 자리 위로 옮겨 내려놓음 (실물은 집을 때 인식 좌표), 0 = 그 자리에서 내림")
ap.add_argument("--lift_h", type=float, default=0.085, help="[m] 고정 동작 집기: 쥔 뒤 들기 (ROS toppick.lift_height)")
ap.add_argument("--lift_rmax", type=float, default=0.32, help="[m] 고정 동작 집기: J1 축에서 이보다 먼 퍽은 들며 J1 쪽으로 (거리 − 이 값) (ROS toppick.lift_max_radius)")
ap.add_argument("--occ_all", action="store_true", help="팔레트 칸에 퍽 (원기둥) 을 미리 앉힘 (어느 칸인지는 --occ_rows)")
ap.add_argument("--occ_rows", default="planner", choices=["planner", "all"],
                help="planner = 목표 열과 뒤 열 (번호 같거나 큰 열) 의 다른 칸만 (mission_planner 열 규칙: 앞 열에 퍽이 있으면 뒤 열에 안 실음), "
                     "all = 목표 칸 빼고 전부 (규칙상 없는 상태, 2 · 3 열 칸 바닥에서 집게가 1 열 퍽에 1 ~ 4 N 닿음)")
ap.add_argument("--traj", default=None, help="칸 궤적 파일 (기본 pallet_place_traj.json, 비교용)")
ap.add_argument("--slots", default="all", help="대 i 의 목표 칸 '열:칸,...' 을 돌아가며 (all = 궤적 파일 칸 전부)")
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
from waffle_isaac.dice2_cfg import GRASP_OFFSET_A5, PUCK_H, puck_cfg  # noqa: E402
from waffle_isaac.robot_cfg import ARM_LINKS_REGEX, BODY_LINK, FINGER_LINKS_REGEX, FINGER_RELEASED  # noqa: E402
from waffle_isaac.toppick_env import LAYOUTS  # noqa: E402
from waffle_isaac.toppick_env_cfg import WaffleTopPickEnvCfg, set_dr  # noqa: E402

TRAJ = json.load(open(a.traj or os.path.join(HERE, "pallet_place_traj.json"), encoding="utf-8"))
SLOTS = TRAJ["slots"]
GRASP_DZ = TRAJ["grasp_dz"]
PHASES = ["pick", "transit"] + TL_PHASES[1:]  # 이어 평가 단계: 0 집기, 1 넘겨받기 (+ 머묾), 2 carry, 3 place, 4 release, 5 after
OUT = a.out or os.path.join(HERE, "chain_pallet_eval.json")
SP = Speeds(a.speed, a.tilt_speed, parse_row_speed(a.row_speed), a.empty_speed)
BODY = "{ENV_REGEX_NS}/Robot/" + BODY_LINK

# ---- 환경 설정 (TopPick 그대로 + 접촉 센서 · 퍽 재질) ----
cfg = WaffleTopPickEnvCfg()
assert abs(cfg.grasp_dz - GRASP_DZ) < 1e-9, f"잡는 높이가 다름: 학습 grasp_dz {cfg.grasp_dz} vs 궤적 {GRASP_DZ} (pallet_traj.py --grasp_dz 와 toppick_env_cfg 를 같게)"
cfg.scene.num_envs = a.num_envs
if a.grip_open is not None:
    cfg.gripper_open = a.grip_open
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
ROBOT_LINKS = "{ENV_REGEX_NS}/Robot/(link[2-5]|gripper_roll_link|gripper_left_link|gripper_right_link)"
cfg.scene.arm_nb = _touch(ROBOT_LINKS, "{ENV_REGEX_NS}/PuckN")  # 2026-11-14 팔 · 손가락 - 옆 퍽
N_OCC = len(SLOTS)
OCC_PATHS = [f"{{ENV_REGEX_NS}}/PuckOcc{j}" for j in range(N_OCC)]
if a.occ_all:  # 칸 퍽 (치워 두는 곳 = 환경 사이 바닥) + 팔 · 쥔 퍽 - 칸 퍽 접촉
    for j in range(N_OCC):
        oc = puck_cfg(f"Occ{j}", "cyl", (0.55, 0.55, 0.6), (-0.55 + 0.1 * j, 0.75))
        sp_ = oc.spawn.replace(activate_contact_sensors=True)
        if a.contact_offset > 0:
            sp_ = sp_.replace(collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=a.contact_offset, rest_offset=0.0))
        if a.puck_mu > 0:
            sp_ = sp_.replace(physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=a.puck_mu, dynamic_friction=a.puck_mu, friction_combine_mode="min"))
        setattr(cfg.scene, f"occ{j}", oc.replace(spawn=sp_))
    cfg.scene.robot_occ = ContactSensorCfg(prim_path=ROBOT_LINKS, filter_prim_paths_expr=OCC_PATHS, update_period=0.0, history_length=1)
    cfg.scene.puckC_occ = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/PuckT_cyl", filter_prim_paths_expr=OCC_PATHS, update_period=0.0, history_length=1)
    cfg.scene.puckB_occ = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/PuckT_box", filter_prim_paths_expr=OCC_PATHS, update_period=0.0, history_length=1)
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
    if a.slots == "all":
        slot_of = [SLOTS[i % len(SLOTS)] for i in range(n)]
    else:
        by_key = {f"{s_['row']}:{s_['index']}": s_ for s_ in SLOTS}
        lst = [by_key[x.strip()] for x in a.slots.split(",")]
        slot_of = [lst[i % len(lst)] for i in range(n)]
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
    p_start = u.puck_pos_w().clone()  # 상자 위 처음 퍽 자리 (다시 쥘 때 여기 위로 돌아와 내려놓음)
    allid = torch.arange(n, device=dev)
    occ_pos0 = torch.zeros(n, N_OCC, 3, device=dev)
    occ_on = torch.zeros(n, N_OCC, dtype=torch.bool, device=dev)
    if a.occ_all:  # 목표 칸 빼고 모든 칸에 퍽 (칸 바닥 + 1 mm)
        rp_, rq_ = u.robot.data.root_link_pos_w.torch, u.robot.data.root_link_quat_w.torch
        for j, sj in enumerate(SLOTS):
            on = torch.tensor([slot_of[i] is not sj and (a.occ_rows == "all" or sj["row"] >= slot_of[i]["row"]) for i in range(n)], device=dev)
            pb_ = torch.tensor([[sj["x"], sj["y"], sj["floor_z"] + PUCK_H / 2 + 0.001]], device=dev).expand(n, 3)
            ob = u.scene[f"occ{j}"]
            pos = torch.where(on[:, None], rp_ + quat_apply(rq_, pb_), ob.data.root_pos_w.torch)
            ob.write_root_pose_to_sim_index(root_pose=torch.cat((pos, ob.data.default_root_pose.torch[:, 3:7]), dim=1), env_ids=allid)
            ob.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=allid)
            occ_on[:, j] = on
        for _ in range(10):  # 가라앉힘
            u.scene.write_data_to_sim()
            u.sim.step(render=False)
            u.scene.update(dt)
        for j in range(N_OCC):
            occ_pos0[:, j] = u.scene[f"occ{j}"].data.root_pos_w.torch
    nb_on = ~u.nb_absent.clone()
    nb_pos0 = u._obj("puckN").data.root_pos_w.torch.clone()
    nb_touch = torch.zeros(n, dtype=torch.bool, device=dev)
    occ_force = torch.zeros(n, len(PHASES), device=dev)

    def watch(ph_idx):  # 2026-11-14 엄격 판정 기록 (스텝마다): 팔 - 옆 퍽, 팔 · 쥔 퍽 - 칸 퍽 (단계별 최대 N)
        nonlocal nb_touch
        nb_touch |= nb_on & (force("arm_nb") > 1.0)
        if a.occ_all:
            fo_ = torch.maximum(force("robot_occ"), torch.where(u.is_box, force("puckB_occ"), force("puckC_occ")))
            occ_force.scatter_reduce_(1, ph_idx[:, None], fo_[:, None], reduce="amax")
    done = torch.zeros(n, dtype=torch.bool, device=dev)
    ok = torch.zeros_like(done)
    cmd = u.robot.actuators.target_command
    wheels0 = torch.zeros(n, len(u._wheels), device=dev)

    def ik_move(goal, grip, sec, mask, grip_other):  # 잡는 점 -> goal (30 Hz, 환경 IK · cart_scale · 앞섬 제한 그대로), mask 대만. 나머지 손가락 = grip_other
        for _ in range(int(sec * a.regrasp_time / u.step_dt)):
            dp = (goal - u.grasp_w()) * a.regrasp_gain
            dp = dp * torch.clamp(cfg.cart_scale / torch.linalg.norm(dp, dim=1, keepdim=True).clamp(min=1e-9), max=1.0)
            dq = u._ik_step(quat_apply_inverse(u.robot.data.root_link_quat_w.torch, dp))
            q = u.robot.data.joint_pos.torch[:, u._arm]
            qt = torch.clamp(torch.clamp(u.arm_target + dq, q - cfg.target_lead_max, q + cfg.target_lead_max), u._lo, u._hi)
            u.arm_target = torch.where(mask[:, None], qt, u.arm_target)
            g = torch.full((n, 1), grip, device=dev)
            if grip_other is not None:
                g = torch.where(mask[:, None], g, grip_other)
            for _ in range(cfg.decimation):
                cmd.set_position_index(value=u.arm_target.contiguous(), joint_ids=u._arm)
                cmd.set_position_index(value=g.expand(-1, 2).contiguous(), joint_ids=u._gl + u._gr)
                cmd.set_velocity_index(value=wheels0, joint_ids=u._wheels)
                u.scene.write_data_to_sim()
                u.sim.step(render=a.gui)
                u.scene.update(dt)
            watch(torch.zeros(n, dtype=torch.long, device=dev))

    if a.scripted_pick is None:
        for _ in range(int(cfg.episode_length_s / u.step_dt) - 2):
            with torch.inference_mode():
                act = pol(obs["policy"])
            ok |= u.succeeded & ~done
            obs, _, term, trunc, _ = env.step(act)
            done |= term | trunc
        ok |= u.succeeded & ~done
    else:  # 고정 동작 집기 = ROS manipulator_node toppick_tilt (approach_mode top) 와 같은 경로 (2026-11-09)
        allm = torch.ones(n, dtype=torch.bool, device=dev)
        up_w = quat_apply(u.robot.data.root_link_quat_w.torch, ez_)
        qe = u.robot.data.body_link_quat_w.torch[:, u._ee]
        u_r = (quat_apply(qe, ex_) + quat_apply(qe, ez_)) / math.sqrt(2.0)  # J1 에서 먼 쪽 (45° 에서 수평)
        meas = u.puck_pos_w() + torch.randn(n, 3, device=dev) * (a.scripted_pick / 1000) * torch.tensor([1.0, 1.0, 0.0], device=dev)
        gl = meas + up_w * cfg.grasp_dz - u_r * (a.regrasp_r / 1000)  # 잡는 점 (ROS radial_offset 0.002 = 반경 −2 mm)
        ik_move(gl + up_w * 0.05 - u_r * 0.03, cfg.gripper_open, 1.5, allm, None)  # 접근 위치 (approach_height · approach_inward)
        ik_move(gl, cfg.gripper_open, 1.5, allm, None)                            # 직선으로 내려옴
        ik_move(gl, grip_close, 0.6, allm, None)                                  # 쥠
        rad_h = gl - u.j1_w()
        rad_h = rad_h - up_w * (rad_h * up_w).sum(1, keepdim=True)
        r_h = torch.linalg.norm(rad_h, dim=1, keepdim=True)
        lift_v = up_w * a.lift_h - rad_h / r_h.clamp(min=1e-6) * (r_h - a.lift_rmax).clamp(min=0.0)  # ROS toppick lift_offset_ 과 같은 식
        ik_move(gl + lift_v, grip_close, max(1.0, a.lift_h / 0.05), allm, None)  # 들기 (ROS toppick.lift_height · lift_max_radius)
        u.actions[:, -1] = -1.0
    off_h, tilt_h, face_h, e_h = in_hand()
    holding = (~done) & (torch.linalg.norm(e_h, dim=1) < 0.015) & (u.lift() > cfg.lift_min) & (u.puck_up() > cfg.upright_min) & u.nb_ok()
    picked = (ok & holding) if a.gate == "env" else holding
    print(f"[Chain] 집기 {int(picked.sum())}/{n} (넘김 {a.gate} | 환경 성공 {int(ok.sum())}, 끝에 쥐고 듦 {int(holding.sum())}, "
          f"성공했다 놓침 {int((ok & ~holding).sum())}) {time.time() - t0:.0f} s", flush=True)
    grip_keep = torch.where(u.actions[:, -1:] > 0.0, cfg.gripper_open, cfg.gripper_closed)

    # ---- 1b. 다시 쥐기 (--regrasp): 상자에 내려놓고 벌려 퍽 자리로 옮겨 다시 쥠 ----
    off_pick, tilt_pick, face_pick = off_h.clone(), tilt_h.clone(), face_h.clone()
    regr = torch.zeros(n, dtype=torch.bool, device=dev)
    lost_rg = torch.zeros_like(regr)
    drift = torch.linalg.norm((u.puck_pos_w() - p_start)[:, :2], dim=1) * 1000  # 집은 뒤 퍽이 처음 자리에서 수평으로 옮겨진 거리 [mm]
    rg_diag = torch.zeros(n, 5, device=dev)  # 다시 쥐기 진단: 벌린 뒤 퍽 기울기 [°] · 퍽 이동 [mm] (내려놓은 자리에서), 쥐기 직전 잡는 점 − 목표 반경 · 높이 [mm], 끝 들림 [mm]
    if a.regrasp != "off":
        t1 = time.time()
        up_w = quat_apply(u.robot.data.root_link_quat_w.torch, ez_)
        qe = u.robot.data.body_link_quat_w.torch[:, u._ee]
        u_r = (quat_apply(qe, ex_) + quat_apply(qe, ez_)) / math.sqrt(2.0)  # 손 안 반경 방향 (+ = J1 에서 먼 쪽, 보상 rew_radial 과 같은 식)
        r_meas = (e_h * u_r).sum(1) * 1000 + torch.randn(n, device=dev) * a.regrasp_noise
        lo, hi = map(float, a.regrasp_win.split(":"))
        regr = picked & (torch.ones_like(picked) if a.regrasp == "always" else (r_meas < lo) | (r_meas > hi))

        def move(goal, grip, sec):  # 다시 쥐는 대만
            ik_move(goal, grip, sec, regr, grip_keep)

        if a.regrasp_home:  # 처음 퍽 자리 위로 수평 이동 -> 내려놓기 (정책이 든 채 옆으로 옮겨 두면 그대로 내리면 상자 모서리에 걸침)
            move(u.grasp_w() + (p_start - u.puck_pos_w()) * torch.tensor([1.0, 1.0, 0.0], device=dev), grip_close, 0.6)
            move(u.grasp_w() + (p_start - u.puck_pos_w()), grip_close, 1.0)
        else:
            move(u.grasp_w() - up_w * (u.lift() + 0.001)[:, None], grip_close, 1.0)  # 그 자리에서 내려놓기 (1 mm 더 누름)
        p_down = u.puck_pos_w().clone()
        move(u.grasp_w().clone(), cfg.gripper_open, 0.5)                          # 벌림
        rg_diag[:, 0] = torch.rad2deg(torch.acos(u.puck_up().clamp(-1.0, 1.0)))
        rg_diag[:, 1] = torch.linalg.norm(u.puck_pos_w() - p_down, dim=1) * 1000
        noise = torch.randn(n, 3, device=dev) * (a.regrasp_noise / 1000) * torch.tensor([1.0, 1.0, 0.0], device=dev)
        goal = u.puck_pos_w() + noise + up_w * cfg.grasp_dz - u_r * (a.regrasp_r / 1000)  # 잰 퍽 자리 + 잡는 높이, 손 안 반경 목표만큼
        move(goal, cfg.gripper_open, 1.0)                                          # 옮기기
        eg = goal - u.grasp_w()
        rg_diag[:, 2] = (eg * u_r).sum(1) * 1000
        rg_diag[:, 3] = (eg * up_w).sum(1) * 1000
        move(goal, grip_close, 0.6)                                                # 쥠
        move(goal + up_w * 0.03, grip_close, 1.0)                                  # 3 cm 들기
        rg_diag[:, 4] = u.lift() * 1000
        off_h, tilt_h, face_h, e_h = in_hand()
        held2 = (torch.linalg.norm(e_h, dim=1) < 0.015) & (u.lift() > cfg.lift_min) & (u.puck_up() > cfg.upright_min)
        lost_rg = regr & ~held2
        picked = picked & ~lost_rg
        print(f"[Chain] 다시 쥐기 {int(regr.sum())} 대 ({a.regrasp}, 잡음 {a.regrasp_noise} mm, 목표 반경 {a.regrasp_r} mm) | 놓침 {int(lost_rg.sum())} | {time.time() - t1:.0f} s", flush=True)

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

    bias = torch.zeros(n, 5, device=dev)
    keep = ~picked  # 집기 실패 대: 지금 관절 목표 · 손가락 그대로
    arm_keep = u.arm_target.clone()
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
        watch(ph)
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
    nb_now = u._obj("puckN").data.root_pos_w.torch
    nb_up_ok = u.nb_up() > cfg.upright_min
    occ_moved = torch.zeros(n, device=dev)
    for j in range(N_OCC if a.occ_all else 0):
        mv = torch.linalg.norm(u.scene[f"occ{j}"].data.root_pos_w.torch - occ_pos0[:, j], dim=1)
        occ_moved = torch.maximum(occ_moved, torch.where(occ_on[:, j], mv, torch.zeros_like(mv)))
    res = []
    for i in range(n):
        s = slot_of[i]
        r = dict(env=i, slot=f"{s['row']}:{s['index']}", shape="box" if bool(box[i]) else "cyl", layout=LAYOUTS[int(lay[i])],
                 latched=bool(latched[i]), picked=bool(picked[i]), pick_env_success=bool(ok[i]), regrasp=bool(regr[i]), drift_mm=round(float(drift[i]), 1))
        if bool(regr[i]):
            r["regrasp_diag"] = dict(zip(("puck_tilt_deg", "puck_moved_mm", "goal_radial_mm", "goal_height_mm", "lift_mm"), (round(float(v), 2) for v in rg_diag[i])))
        if not r["picked"]:
            r["stage"] = "regrasp_drop" if bool(lost_rg[i]) else "pick_fail"
            res.append(r)
            continue
        dxy = math.hypot(float(bot[i, 0]) - s["x"], float(bot[i, 1]) - s["y"])
        dz = float(bot[i, 2]) - s["floor_z"]
        tilt = math.degrees(math.acos(min(1.0, float(ax[i, 2]))))
        yaw = math.degrees(math.atan2(float(pxb[i, 1]), float(pxb[i, 0])))
        seated = dxy < 0.003 and abs(dz) < 0.003 and tilt < 5.0 and float(slip[i]) < 0.015
        nbm = float(torch.linalg.norm((nb_now[i] - nb_pos0[i])[:2])) if bool(nb_on[i]) else 0.0
        occm = float(occ_moved[i])
        strict_bad = [k_ for k_, bad_ in (("옆 퍽 닿음", bool(nb_touch[i])), ("옆 퍽 밀림", nbm > 0.003), ("옆 퍽 넘어짐", bool(nb_on[i]) and not bool(nb_up_ok[i])),
                                           ("칸 퍽 닿음", float(occ_force[i].max()) > 1.0), ("칸 퍽 밀림", occm > 0.003)) if bad_]
        r.update(strict=seated and not strict_bad, strict_fail=strict_bad, nb_moved_mm=round(nbm * 1000, 1), occ_moved_mm=round(occm * 1000, 1),
                 occ_force_N={PHASES[j]: round(float(occ_force[i, j]), 1) for j in range(len(PHASES))} if a.occ_all else None)
        lost_transit = float(torch.linalg.norm(off_f[i])) > 15.0
        r.update(seated=seated, stage="seated" if seated else ("transit_drop" if lost_transit else ("carry_slip" if float(slip[i]) >= 0.015 else "not_seated")),
                 xy_err_mm=round(dxy * 1000, 1), bottom_above_floor_mm=round(dz * 1000, 1), tilt_deg=round(tilt, 1),
                 box_face_err_deg=round(((yaw - s["theta_deg"] + 45.0) % 90.0) - 45.0, 1) if bool(box[i]) else None,
                 max_slip_mm=round(float(slip[i]) * 1000, 1),
                 radial_mm=round((float(off_h[i, 1]) + float(off_h[i, 2])) / math.sqrt(2.0), 2),
                 in_hand={k: dict(close_mm=round(float(o[i, 0]), 2), appr_mm=round(float(o[i, 1]), 2), side_mm=round(float(o[i, 2]), 2),
                                  tilt_deg=round(float(tl[i]), 2), face_deg=round(float(fc[i]), 2) if bool(box[i]) else None)
                          for k, o, tl, fc in (("pick", off_pick, tilt_pick, face_pick), ("handover", off_h, tilt_h, face_h), ("q_front", off_f, tilt_f, face_f), ("place", off_p, tilt_p, face_p))},
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
        per_slot[key] = dict(chain=rate(rows), strict=rate(rows, "strict"), pick=rate(rows, "picked"), place_given_pick=rate([x for x in rows if x["picked"]]),
                             cyl=rate([x for x in rows if x["shape"] == "cyl"]), box=rate([x for x in rows if x["shape"] == "box"]),
                             theta_deg=s["theta_deg"])
    hand = {stg: {c: stat([x["in_hand"][stg][c] for x in pk if x["in_hand"][stg][c] is not None])
                  for c in ("close_mm", "appr_mm", "side_mm", "tilt_deg", "face_deg")} for stg in ("handover", "q_front", "place")}
    def hand_bin(x):
        h = x["in_hand"]["handover"]
        e = math.hypot(h["close_mm"], h["appr_mm"], h["side_mm"])
        return "<1mm" if e < 1 else "1-3mm" if e < 3 else ">=3mm"
    by_hand = {b: rate([x for x in pk if hand_bin(x) == b]) for b in ("<1mm", "1-3mm", ">=3mm")}
    stages = {k: sum(1 for x in res if x["stage"] == k) for k in ("pick_fail", "regrasp_drop", "transit_drop", "carry_slip", "not_seated", "seated")}
    summary = dict(policy=os.path.relpath(path, HERE), gate=a.gate, pick_env_success=int(ok.sum()), seed=a.seed, layout=a.layout, dr=a.dr, num_envs=n, puck_mu=a.puck_mu,
                   scripted_pick_noise_mm=a.scripted_pick, gripper_open=cfg.gripper_open,
                   regrasp=dict(mode=a.regrasp, r_mm=a.regrasp_r, noise_mm=a.regrasp_noise, win=a.regrasp_win, n=int(regr.sum()), dropped=int(lost_rg.sum())),
                   ki=a.ki, transit_speed=a.transit_speed, settle_s=a.settle, speeds=dict(speed=a.speed, tilt=a.tilt_speed, row=a.row_speed, empty=a.empty_speed),
                   chain=rate(res), pick=rate(res, "picked"), place_given_pick=rate(pk), stages=stages,
                   strict=rate(res, "strict"), lift_h=a.lift_h, lift_rmax=a.lift_rmax, occ_all=a.occ_all,
                   strict_fail={k_: sum(1 for x in pk if x.get("seated") and k_ in x.get("strict_fail", [])) for k_ in ("옆 퍽 닿음", "옆 퍽 밀림", "옆 퍽 넘어짐", "칸 퍽 닿음", "칸 퍽 밀림")},
                   by_shape={sh: rate([x for x in res if x["shape"] == sh]) for sh in ("cyl", "box")},
                   by_layout={ly: rate([x for x in res if x["layout"] == ly]) for ly in LAYOUTS},
                   seated_by_handover_offset=by_hand, in_hand=hand,
                   arm_body_max_N=round(float(fmax["arm"][picked].max()) if picked.any() else 0.0, 1),
                   arm_box_transit_max_N=round(float(fmax["arm_box"][picked, 1].max()) if picked.any() else 0.0, 1),
                   wall_s=round(time.time() - t0, 1), date=time.strftime("%Y-%m-%d %H:%M"))
    c = summary["chain"]
    print(f"[Chain] 정책 {summary['policy']} | 시드 {a.seed} | 배치 {a.layout} | 무작위화 {a.dr}", flush=True)
    print(f"[Chain] 이어서 성공 {c['ok']}/{c['n']} ({c['pct']} %) = 집기 {summary['pick']['pct']} % x 넣기 {summary['place_given_pick']['pct']} % | 단계 {stages}", flush=True)
    st = summary["strict"]
    print(f"[Chain] **엄격 성공 {st['ok']}/{st['n']} ({st['pct']} %)** (넣기 성공 + 옆 퍽 · 칸 퍽 안 닿고 안 밀림) | 넣었는데 엄격 실패 {summary['strict_fail']}"
          f" | 들기 {a.lift_h} m (J1 축 {a.lift_rmax} m 밖은 당김), 칸 다 참 {a.occ_all}", flush=True)
    print("[Chain] 칸별 (열:칸 이어서 / 집기 / 집은 것 중 넣기 | 원기둥 · 상자 | 엄격)", flush=True)
    for k, v in per_slot.items():
        f = lambda r: f"{r['ok']}/{r['n']}"
        print(f"[Chain]   {k} ({v['theta_deg']:+7.1f}°)  {f(v['chain']):>6s}  {f(v['pick']):>6s}  {f(v['place_given_pick']):>6s} | {f(v['cyl'])} · {f(v['box'])} | {f(v['strict'])}", flush=True)
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
