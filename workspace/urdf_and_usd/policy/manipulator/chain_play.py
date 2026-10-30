"""매니퓰레이터 모델 이어 붙여 실행 (ROS 상태 머신의 시뮬레이션 판, 정책_학습기록 4.11 절):

  상자 위 퍽 --[1 집기 SidePick SP3]--> 쥐고 3 cm 듦 (0.5 s) --[2 옮기기·맞추기 Store ST3]--> 통 축 2.5 mm·입구 위 12 mm·직립
             --[3 넣기 Insert]--> 퍽 통 안 (입구 아래 60 mm) + 집게 위로 뺌 -> 끝 (팔 멈춤)

모델 파일은 그대로 (합치지 않음). 이 스크립트가 세 모델을 불러 단계마다 관측을 그 모델이 학습한 방식대로 만들어 넣고,
행동도 그 모델 방식대로 해석 (1·2 = 관절 += 0.04 rad, 3 = 잡는 점 1 mm + IK).
장면 = 정렬·집기 장면 (상자 + 퍽, 원기둥만) + 스토리지 접촉 거리 2 mm + 팔·손가락-몸체 접촉 센서. 목표 통 = P·N 번갈아.

    ./run_isaac.sh chain_play [환경 수]   창으로 (기본 4 대)
    ./run_isaac.sh chain_eval [환경 수]   창 없이 단계별 성공률 (기본 64 대)
"""

import argparse
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--gui", action="store_true")
parser.add_argument("--seconds", type=float, default=25.0)
parser.add_argument("--sidepick", default=None)
parser.add_argument("--store", default=None)
parser.add_argument("--insert", default=None)
parser.add_argument("--insert_ik", action="store_true", help="3 단계 넣기를 강화 학습 모델 대신 손 IK 로: 퍽 바닥을 통 축에 맞추며 1 mm / 스텝 내려 10 mm -> 열기 -> 위로 빼기 (store_hybrid 와 같음)")
parser.add_argument("--ik_depth", type=float, default=0.010)
parser.add_argument("--ik_gate_mm", type=float, default=50.0, help="손 IK 넣기 때 옮기기 -> 넣기 넘어가는 통 축 거리 [mm] (옮기기 정책은 통 근처 4 cm 까지 확실, 마지막 맞추기는 손 IK)")
parser.add_argument("--collect_handover", default=None, help="집기가 끝난 (쥐고 3 cm 들어 0.5 s) 실제 상태를 이 json 에 모음 -> store_starts.json 의 H (이어 받기) 시작 상태. 집기 모델만 씀")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = not args_cli.gui
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import glob  # noqa: E402
import json  # noqa: E402

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
import waffle_isaac  # noqa: E402,F401
from isaaclab.envs import ViewerCfg  # noqa: E402
from isaaclab.sensors import ContactSensorCfg  # noqa: E402
from isaaclab.utils.math import quat_apply, quat_apply_inverse, quat_box_minus  # noqa: E402
from waffle_isaac.dice2_cfg import PUCK_H  # noqa: E402
from waffle_isaac.insert_env_cfg import INSERT_POSE, WaffleInsertEnvCfg  # noqa: E402
from waffle_isaac.robot_cfg import ARM_JOINTS, ARM_LINKS_REGEX, FINGER_LINKS_REGEX, ROOT_TO_BASE_LINK  # noqa: E402
from waffle_isaac.sidepick_env_cfg import WaffleSidePickEnvCfg  # noqa: E402
from waffle_isaac.store_env_cfg import BODY, START_POSE, WaffleStoreEnvCfg  # noqa: E402

PHASES = ("1 집기", "2 옮기기·맞추기", "3 넣기", "끝")


def pick_policy(arg, fixed, pattern):
    if arg:
        return arg
    if os.path.exists(fixed):
        return fixed
    return sorted(glob.glob(os.path.join(HERE, pattern)))[-1]


def main():
    p_sp = pick_policy(args_cli.sidepick, os.path.join(HERE, "policies/sidepick_latest.pt"), "logs/rsl_rl/waffle_sidepick/*/exported/policy.pt")
    p_st = None if args_cli.collect_handover else pick_policy(args_cli.store, os.path.join(HERE, "policies/store_latest.pt"), "logs/rsl_rl/waffle_store/*/exported/policy.pt")
    p_in = None if (args_cli.collect_handover or args_cli.insert_ik) else pick_policy(args_cli.insert, os.path.join(HERE, "policies/insert_latest.pt"), "logs/rsl_rl/waffle_insert/*/exported/policy.pt")

    cfg = WaffleSidePickEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.box_shape_prob = 0.0  # 상자 모양 퍽은 통 (지름 약 32) 에 안 들어감
    cfg.near_start_prob = 0.0  # 실제 과제 배치 (학습용 역방향 커리큘럼 끔)
    cfg.episode_length_s = args_cli.seconds + 5.0
    cfg.viewer = ViewerCfg(eye=(0.85, -0.85, 0.75), lookat=(0.12, 0.0, 0.25), origin_type="env", env_index=0)
    col = sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0)  # 넣기 학습과 같게 (통 틈 1 mm)
    cfg.scene.robot = cfg.scene.robot.replace(spawn=cfg.scene.robot.spawn.replace(collision_props=col))
    cfg.scene.puckT_cyl = cfg.scene.puckT_cyl.replace(spawn=cfg.scene.puckT_cyl.spawn.replace(collision_props=col))
    cfg.scene.arm_body_contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/" + ARM_LINKS_REGEX, filter_prim_paths_expr=[BODY],
                                                  update_period=0.0, history_length=1)
    cfg.scene.finger_body_contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/" + FINGER_LINKS_REGEX,
                                                     filter_prim_paths_expr=[BODY], update_period=0.0, history_length=1)
    env = gym.make("Waffle-SidePick-Direct", cfg=cfg)
    u = env.unwrapped
    r, n, dev = u.robot, u.num_envs, u.device
    collect = args_cli.collect_handover
    pols = [torch.jit.load(p, map_location=dev).eval() for p in ((p_sp, p_sp, p_sp) if collect else (p_sp, p_st, p_in or p_st))]
    rows = []
    print(f"[Chain] 모델 1 집기 {p_sp}\n[Chain] 모델 2 옮기기·맞추기 {p_st}\n[Chain] 모델 3 넣기 {'손 IK (' + str(args_cli.ik_depth * 1000) + ' mm 넣고 놓기)' if args_cli.insert_ik else p_in}", flush=True)
    obs, _ = env.reset()

    S = json.load(open(os.path.join(HERE, "store_starts.json"), encoding="utf-8"))
    tubes = torch.tensor([[t["axis"][0], t["axis"][1], t["rim_z"]] for t in S["tubes"]], device=dev)
    tube = torch.tensor([1 + (i % 2) for i in range(n)], device=dev)  # P, N 번갈아
    st_cfg, in_cfg = WaffleStoreEnvCfg(), WaffleInsertEnvCfg()
    q_start = torch.tensor([[START_POSE[j] for j in ARM_JOINTS]], device=dev)
    q_ins = torch.tensor([[INSERT_POSE[j] for j in ARM_JOINTS]], device=dev)
    bl = torch.tensor([[0.0, 0.0, ROOT_TO_BASE_LINK]], device=dev)
    ex = torch.tensor([[1.0, 0.0, 0.0]], device=dev).repeat(n, 1)
    ez = torch.tensor([[0.0, 0.0, 1.0]], device=dev).repeat(n, 1)
    cols = torch.tensor([getattr(r, "num_base_dofs", 6) + j for j in u._arm], device=dev)
    off = u._off

    def to_bl(p_w):
        return quat_apply_inverse(r.data.root_link_quat_w.torch, p_w - r.data.root_link_pos_w.torch) - bl

    def vec_bl(v_w):
        return quat_apply_inverse(r.data.root_link_quat_w.torch, v_w)

    def state():
        pc = to_bl(u.puck_pos_w())
        uu = vec_bl(quat_apply(u.puck_quat_w(), ez))
        uu = uu * torch.where(uu[:, 2:3] < 0, -1.0, 1.0)
        pb = pc - uu * (PUCK_H / 2)
        t = tubes[tube]
        gb = to_bl(u.grasp_w())
        return dict(pc=pc, u=uu, pb=pb, t=t, gb=gb, dxy=torch.linalg.norm(pb[:, :2] - t[:, :2], dim=1), depth=t[:, 2] - pb[:, 2],
                    held=torch.linalg.norm(pc - gb, dim=1) < 0.03, fing=r.data.joint_pos.torch[:, u._gl[0]])

    def force(k):
        f = u.scene[k].data.normal_force_matrix_w.torch
        return torch.linalg.norm(f, dim=-1).amax(dim=(1, 2))

    phase = torch.zeros(n, dtype=torch.long, device=dev)
    cnt = torch.zeros(n, dtype=torch.long, device=dev)
    t_enter = torch.zeros(n, 4, device=dev)
    prev6 = torch.zeros(n, 6, device=dev)
    prev6[:, 5] = -1.0  # 옮기기 모델은 쥔 채로 시작하게 학습됨
    ph_steps = torch.zeros(n, dtype=torch.long, device=dev)
    ik_stage = torch.zeros(n, dtype=torch.long, device=dev)  # 손 IK 넣기: 0 내림, 1 열기, 2 빼기
    ik_cnt = torch.zeros(n, dtype=torch.long, device=dev)
    prev4 = torch.zeros(n, 4, device=dev)
    prev4[:, 3] = -1.0
    q5_hold = torch.zeros(n, 4, device=dev)
    f_arm_max = torch.zeros(n, device=dev)
    f_fing_max = torch.zeros(n, device=dev)
    body_steps = torch.zeros(n, device=dev)
    steps = int(args_cli.seconds / u.step_dt)
    arm_prev = u.arm_target.clone()
    for k in range(steps):
        s = state()
        ax, _, az = u.a5_axes_w()
        d = r.data
        # 관측 (모델마다 학습 때와 같게)
        o_sp = u._get_observations()["policy"]
        o_st = torch.cat((d.joint_pos.torch[:, u._arm] - q_start, d.joint_vel.torch[:, u._arm], d.joint_pos.torch[:, u._gl], s["gb"], s["pc"] - s["gb"],
                          s["u"], s["t"], s["pb"] - s["t"], vec_bl(-az), vec_bl(ax), torch.nn.functional.one_hot(tube, 3).float(), prev6), dim=-1)
        est_bottom = s["gb"] - torch.tensor([[0.0, 0.0, in_cfg.grasp_dz + PUCK_H / 2]], device=dev)
        o_in = torch.cat((d.joint_pos.torch[:, u._arm] - q_ins, d.joint_vel.torch[:, u._arm], d.applied_torque.torch[:, u._arm], d.joint_pos.torch[:, u._gl],
                          s["gb"], s["t"], est_bottom - s["t"], vec_bl(-az), torch.nn.functional.one_hot(tube, 3).float(), prev4), dim=-1)
        with torch.inference_mode():
            a_sp = pols[0](o_sp)
            a_st = pols[1](o_st) if not collect else torch.zeros(n, 6, device=dev)
            a_in = pols[2](o_in) if not (collect or args_cli.insert_ik) else torch.zeros(n, 4, device=dev)
        # 단계마다 행동 해석 (학습 때와 같게): 1 집기 = 잡는 점 5 mm 직교 + IK (집게 수평·바깥 방향), 2 옮기기 = 관절 += 0.04 rad,
        # 3 넣기 = 잡는 점 1 mm 직교 + IK (들어온 순간 자세 유지), 끝 = 멈춤·집게 벌림. 마지막 성분 = 집게 (> 0 벌림)
        q = d.joint_pos.torch[:, u._arm]
        lead = u.cfg.target_lead_max
        a_sp, a_st, a_in = a_sp.clamp(-1.0, 1.0), a_st.clamp(-1.0, 1.0), a_in.clamp(-1.0, 1.0)
        m0, m1, m3, m4 = phase == 0, phase == 1, phase == 2, phase == 3
        tgt = arm_prev.clone()
        grip = torch.ones(n, device=dev)
        if m0.any():
            dq0 = u._ik_step(a_sp[:, :3] * u.cfg.cart_scale)
            tgt[m0] = (arm_prev + dq0)[m0]
            grip[m0] = a_sp[m0, -1]
            u.prev_actions[m0] = u.actions[m0]
            u.actions[m0] = a_sp[m0]
        if m1.any():  # 옮기기 환경 _pre_physics_step 과 똑같이: 통 근처 정밀 모드 (x 0.25) + 손목 롤 0 고정
            fine = (s["dxy"] < st_cfg.fine_xy) & (-s["depth"] < st_cfg.zone_clear)
            sc = st_cfg.action_scale * torch.where(fine, st_cfg.fine_scale, 1.0).unsqueeze(1)
            t1 = arm_prev + sc * a_st[:, :5]
            if st_cfg.lock_roll:
                t1[:, 4] = 0.0
            tgt[m1] = t1[m1]
            grip[m1] = a_st[m1, 5]
            prev6[m1] = a_st[m1]
        if m3.any():
            q5 = d.body_link_quat_w.torch[:, u._a5]
            fresh = m3 & (ph_steps < 2)  # 넣기 단계 들어온 직후 자세를 유지 목표로
            q5_hold[fresh] = q5[fresh]
            J = d.body_link_jacobian_w.torch[:, u._a5][:, :, cols]
            rr = quat_apply(q5, off)
            Jl = J[:, 0:3] - torch.cross(rr.unsqueeze(-1).expand(-1, -1, 5), J[:, 3:6], dim=1)
            w = in_cfg.rot_hold_weight
            Jf = torch.cat((Jl, J[:, 3:6] * w), dim=1)
            if args_cli.insert_ik:  # 손 IK: 0 입구 위 8 mm 에서 축 맞춤 -> 1 1 mm/스텝 내림 (축 보정) -> 2 열기 0.5 s -> 3 위로 4 cm
                dpb = torch.zeros(n, 3, device=dev)
                xy = torch.clamp((s["t"][:, :2] - s["pb"][:, :2]) * 0.5, -0.004, 0.004)
                al, dn = m3 & (ik_stage == 0), m3 & (ik_stage == 1)
                dpb[:, :2] = torch.where((al | dn)[:, None], xy, dpb[:, :2])
                hz = torch.clamp(((s["t"][:, 2] + 0.008) - s["pb"][:, 2]) * 0.5, -0.003, 0.003)
                dpb[:, 2] = torch.where(al, hz, torch.where(dn, -0.001, torch.where(m3 & (ik_stage == 3), 0.002, 0.0)))
                a_in = torch.cat((dpb / in_cfg.cart_scale, torch.where(m3 & (ik_stage >= 2), 1.0, -1.0).unsqueeze(1)), dim=1)
            err = torch.cat((quat_apply(d.root_link_quat_w.torch, a_in[:, :3] * in_cfg.cart_scale), quat_box_minus(q5_hold, q5) * w), dim=1)
            dq = (Jf.transpose(1, 2) @ torch.linalg.solve(Jf @ Jf.transpose(1, 2) + in_cfg.ik_damping * torch.eye(6, device=dev), err.unsqueeze(-1))).squeeze(-1)
            tgt[m3] = (arm_prev + dq)[m3]
            grip[m3] = a_in[m3, 3]
            prev4[m3] = a_in[m3]
        u.arm_target[:] = torch.clamp(torch.clamp(tgt, q - lead, q + lead), u._lo, u._hi)
        u.arm_target[m4] = arm_prev[m4]
        u.actions[:, -1] = torch.where(m4, torch.ones_like(grip), grip)
        for _ in range(cfg.decimation):
            u._apply_action()
            u.scene.write_data_to_sim()
            u.sim.step(render=args_cli.gui)
            u.scene.update(u.physics_dt)
        u.episode_length_buf[:] = 0  # 정렬·집기 환경의 시간 끝 리셋 막기 (이 스크립트가 직접 돌림)
        arm_prev = u.arm_target.clone()
        fa, ff = force("arm_body_contact"), force("finger_body_contact")
        f_arm_max = torch.maximum(f_arm_max, fa)
        f_fing_max = torch.maximum(f_fing_max, ff)
        body_steps += (fa > 1.0).float()
        # 단계 전환
        s = state()
        if args_cli.insert_ik:
            m3n = phase == 2
            ik_cnt += 1
            to1 = m3n & (ik_stage == 0) & (((s["dxy"] < 0.0015) & ((s["depth"] + 0.008).abs() < 0.003)) | (ik_cnt > 90))
            to2 = m3n & (ik_stage == 1) & ((s["depth"] > args_cli.ik_depth) | (ik_cnt > 40))
            to3 = m3n & (ik_stage == 2) & (ik_cnt > 15)
            ik_stage[to1], ik_cnt[to1] = 1, 0
            ik_stage[to2], ik_cnt[to2] = 2, 0
            ik_stage[to3], ik_cnt[to3] = 3, 0
        lift = u.lift()
        c0 = (phase == 0) & (lift > 0.03) & s["held"] & (u.puck_up() > 0.9)
        if args_cli.insert_ik:  # 통 근처면 손 IK 가 맞춤까지
            c1 = (phase == 1) & s["held"] & (s["dxy"] < args_cli.ik_gate_mm / 1000) & (s["depth"] > -0.04) & (s["depth"] < 0.005) & (s["u"][:, 2] > 0.97)
        else:
            c1 = (phase == 1) & s["held"] & (s["dxy"] < 0.0025) & (s["depth"] > -0.012) & (s["depth"] < 0.002) & (s["u"][:, 2] > 0.985)
        # 끝 = 퍽이 통 축 6 mm 안·입구 아래 60 mm 넘게 + 집게 벌림 (집게를 6 cm 위로 빼는 조건은 새 팔 높이 한계 (통 위 약 4 cm) 로 못 채워 뺌)
        c2 = (phase == 2) & (s["depth"] > 0.06) & (s["dxy"] < 0.006) & (s["fing"] > -0.002)
        go = torch.where(phase == 0, c0, torch.where(phase == 1, c1, c2))
        cnt = torch.where(go, cnt + 1, torch.zeros_like(cnt))
        need = torch.where(phase == 0, 15, torch.where(phase == 1, 3, 5))
        adv = go & (cnt >= need) & (phase < 3)
        ph_steps += 1
        if adv.any():
            phase[adv] += 1
            cnt[adv] = 0
            ph_steps[adv] = 0
            ik_stage[adv], ik_cnt[adv] = 0, 0
            t_enter[adv, phase[adv]] = (k + 1) * u.step_dt
            if collect:  # 집기 -> 옮기기 넘어가는 순간의 실제 상태를 모으고 그 대는 멈춤 (끝 단계)
                jp = r.data.joint_pos.torch
                pq = u.puck_quat_w()
                for i in torch.nonzero(adv & (phase == 1)).flatten().tolist():
                    rows.append({"cat": "H", "tube": -1, "q": [round(float(v), 5) for v in jp[i, u._arm].tolist()],
                                 "finger": [float(jp[i, u._gl[0]]), float(jp[i, u._gr[0]])],
                                 "puck_b": [round(float(v), 5) for v in s["pc"][i].tolist()], "puck_quat_b": [float(v) for v in pq[i].tolist()]})
                phase[adv & (phase == 1)] = 3
            for i in torch.nonzero(adv).flatten().tolist():
                print(f"[Chain] t={(k + 1) * u.step_dt:5.1f}s env{i} (통 {'CPN'[int(tube[i])]}) -> {PHASES[int(phase[i])]}", flush=True)
        if k % 60 == 59:
            print(f"[Chain] t={(k + 1) * u.step_dt:5.1f}s 단계 {[int(v) + 1 for v in phase.tolist()][:16]} 퍽 들기 mm {[round(v * 1000) for v in lift.tolist()][:8]}"
                  f" 통 축 mm {[round(v * 1000, 1) for v in s['dxy'].tolist()][:8]} 입구 아래 mm {[round(v * 1000) for v in s['depth'].tolist()][:8]}", flush=True)
    if collect:
        json.dump({"note": "집기 (SidePick) 가 끝난 실제 상태 (Isaac): 팔 관절, 손가락, 퍽 중심 (base_link), 퍽 자세 (월드 = 루트 요 0, x,y,z,w)", "samples": rows},
                  open(collect, "w"))
        print(f"[Chain] 이어 받기 상태 {len(rows)} 개 -> {collect}", flush=True)
    s = state()
    done = phase == 3
    print(f"[Chain] ===== {args_cli.seconds:.0f} s 결과 ({n} 대, 목표 통 P·N 번갈아) =====", flush=True)
    print(f"[Chain] 1 집기 성공 {int((phase >= 1).sum())}/{n}, 2 통 위 맞춤 {int((phase >= 2).sum())}/{n}, 3 통 안 넣고 놓음 {int(done.sum())}/{n}", flush=True)
    stored = (s["depth"] > 0.06) & (s["dxy"] < 0.006)
    for ti, nm in ((1, "P"), (2, "N")):
        mt = tube == ti
        print(f"[Chain]   {nm} 통 목표 {int(mt.sum())} 대: 집기 {int(((phase >= 1) & mt).sum())}, 맞춤 단계 {int(((phase >= 2) & mt).sum())}, **최종 통 안 {int((stored & mt).sum())}**", flush=True)
    dropped = s["depth"] > 0.2
    print(f"[Chain]   최종: 통 안 {int(stored.sum())}/{n}, 퍽 떨어뜨림 (통 밖 낮은 곳) {int((dropped & ~stored).sum())}", flush=True)
    if done.any():
        print(f"[Chain] 끝까지 간 대의 단계 시작 시각 평균 s: 옮기기 {t_enter[done, 1].mean().item():.1f}, 넣기 {t_enter[done, 2].mean().item():.1f}, 끝 {t_enter[done, 3].mean().item():.1f}", flush=True)
    print(f"[Chain] 퍽 최종 입구 아래 mm {[round(v * 1000) for v in s['depth'].tolist()][:32]}", flush=True)
    print(f"[Chain] 팔 링크-몸체 최대 힘 N {[round(v, 1) for v in f_arm_max.tolist()][:32]} (1 N 넘은 스텝 합 {int(body_steps.sum())}) | 손가락-몸체 최대 N {[round(v, 1) for v in f_fing_max.tolist()][:32]}", flush=True)
    if args_cli.gui:
        while simulation_app.is_running():
            u.sim.step(render=True)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
