"""2026-11-10 (Onbeom, #192) P1 세우기 평가 (창 없이, 한 번 시도 기준): 대마다 첫 에피소드 결과를 셈.

  python lieupright_eval.py --policy <exported policy.pt> --num_envs 512 [--nb_prob 0.5] [--seed 777] [--starts 1,0,0] [--dump_end 파일]
판정 R1 (세워 놓은 대 / 대), R4 (옆 퍽 · 팔-몸체), R5 (성공 시각) 와 단계별 (쥠 · 잘 쥠 · 10 cm · 세움 · 놓음) · 종료 이유.
--dump_end: 성공한 대의 끝 상태 (세운 퍽 · 상자 · 옆 퍽, base) 를 JSON 으로 = P2 (다시 잡기) 시작 상태.
"""
import argparse
import json
import math
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=512)
parser.add_argument("--policy", required=True)
parser.add_argument("--nb_prob", type=float, default=0.5)
parser.add_argument("--seed", type=int, default=777)
parser.add_argument("--starts", default="1,0,0", help="시작 종류 확률 (처음부터, 쥐고 듦, 세움)")
parser.add_argument("--dump_end", default="")
parser.add_argument("--chain_phys", type=int, default=0, help="2026-11-11 E1a: 1 = chain45 와 같은 물리 (로봇 · 목표 퍽 contact_offset 2 mm · rest 0, 목표 퍽 마찰 0.5 min 결합)")
parser.add_argument("--nb_world", type=int, default=0, help="2026-11-11: 1 = 옆 퍽 밀림 판정을 월드 좌표로 (cfg.nb_move_world)")
parser.add_argument("--nb_diag", default="", help="2026-11-11 r2f 뒤 (계획 단계 회피안 근거): 옆 퍽 있는 대의 시작 틈 · 방향과 닿음 여부 json")
parser.add_argument("--cfg", default="", help="2026-11-11: 환경 설정 클래스 이름 끝 (예: B6 = WaffleLieUprightB6EnvCfg). 기본 = 기본 설정")
parser.add_argument("--obs_hold", default=None, help="2026-11-11 실물 쥠 판정 (RL1): 관측 쥠 칸만 바꿈 (truth = 학습 판정, finger = 손가락 범위 + 히스테리시스만). 판정 · 보상은 그대로")
parser.add_argument("--vmax", type=float, default=0.0, help="2026-11-11 (미션 파트 노드 모사): > 0 이면 팔 관절 목표 변화를 이 rad/s 로 자름 (cfg.arm_vmax)")
parser.add_argument("--vmode", default="clip", help="2026-11-11: 속도 제한 방식 clip (관절별 자름) | scale (노드 lieApplyActionLimited 비례 축소)")
parser.add_argument("--dump_e1", default="", help="2026-11-11 E1a: 접촉 없이 성공한 대의 끝 상태 (base 좌표, env 번호별) -> chain45 --lie_from_end")
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
from waffle_isaac.lieupright_env_cfg import WaffleLieUprightEnvCfg  # noqa: E402


def main():
    import waffle_isaac.lieupright_env_cfg as _C
    cfg = getattr(_C, f"WaffleLieUpright{args_cli.cfg}EnvCfg")() if args_cli.cfg else WaffleLieUprightEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.nb_prob = args_cli.nb_prob
    cfg.nb_move_world = bool(args_cli.nb_world) or cfg.nb_move_world
    if args_cli.obs_hold is not None:
        cfg.obs_hold_mode = args_cli.obs_hold
    cfg.arm_vmax = args_cli.vmax
    if args_cli.vmax > 0:
        cfg.arm_vmax_range = None  # 평가: 고정 상한
        cfg.arm_vlimit_mode = args_cli.vmode
    cfg.start_probs = tuple(float(v) for v in args_cli.starts.split(","))
    cfg.seed = args_cli.seed
    if args_cli.chain_phys:  # chain45_play.py 의 --col_offset 0.002 · --puck_mu 0.5 (기본값) 와 같은 변경 (REGRASP = 목표 퍽 둘 다)
        import isaaclab.sim as sim_utils
        col = sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0)
        mat = sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.5, friction_combine_mode="min")
        cfg.scene.robot = cfg.scene.robot.replace(spawn=cfg.scene.robot.spawn.replace(collision_props=col))
        for k_ in ("puckT_cyl", "puckT_box"):
            o_ = getattr(cfg.scene, k_)
            setattr(cfg.scene, k_, o_.replace(spawn=o_.spawn.replace(collision_props=col, physics_material=mat)))
    from phys_report import report
    report("E1a" if args_cli.chain_phys else "P1", cfg)
    env = gym.make("Waffle-LieUpright-Direct", cfg=cfg)
    u = env.unwrapped
    pol = torch.jit.load(args_cli.policy, map_location=u.device).eval()
    obs, _ = env.reset()
    n, dev = u.num_envs, u.device
    print(f"[LUEval] 정책 {args_cli.policy}, 대 {n}, 옆 퍽 확률 {args_cli.nb_prob}, seed {args_cli.seed}", flush=True)
    B = lambda: torch.zeros(n, dtype=torch.bool, device=dev)
    ended, succ = B(), B()
    st = {k: B() for k in ("grasp", "center", "lift10", "stood", "placed")}
    why = {k: B() for k in ("drop", "topple", "nb", "self", "arm_body", "body_box", "badspot", "nan", "open_away", "timeout")}
    t_succ = torch.full((n,), float("nan"), device=dev)
    nolift = B()  # RL1 검토: 쥠 판정 뒤 3 cm 못 든 채 끝난 대
    nb_on = u.nb_present.clone()
    nb_stage = {}
    end_state = [None] * n
    e1 = [None] * n
    j3_hi = torch.zeros(n, device=dev)  # 검토 7: 관절 3 > 1.5 rad 스텝 수 (첫 에피소드)
    steps = torch.zeros(n, device=dev)
    ho_fp = torch.zeros(n, device=dev); ho_fn = torch.zeros(n, device=dev); ho_st = torch.zeros(n, device=dev); ho_stage = torch.zeros(2, 4, device=dev)  # ho_stage [관측만 · 판정만] x [쥐기 전, 쥔 뒤 놓기 전, 놓은 뒤 벌리기 전, 벌림 명령 뒤] (RL1 승인 덧붙임)  # 2026-11-11 관측 쥠 대 판정 쥠: 관측만 1 · 판정만 1 · 산 스텝
    root_yaw_max = torch.zeros(n, device=dev)  # 2026-11-11 RL1: 로봇 root 요 최대 |값| (켤레 순서 잘못의 영향 확인, quat_apply 로만 잼)
    root_tilt_max = torch.zeros(n, device=dev)
    ab_hit = torch.zeros(n, device=dev)  # 2026-11-11 재확인: 팔-상자 접촉 스텝 수 (첫 에피소드, 벽 · 상자 접촉 금지 규칙)
    ml_end = torch.zeros(n, device=dev)  # 끝날 때 최대 든 높이
    gx_end = torch.full((n,), float("nan"), device=dev)  # 2026-11-11 성공 순간 잡는 점 base x
    open_b = torch.full((n,), float("nan"), device=dev); open_bfk = torch.full((n,), float("nan"), device=dev); open_st = torch.zeros(n, dtype=torch.bool, device=dev)  # 2026-11-11 원인 확인: 든 뒤 처음 벌림 명령 스텝의 관측 β · FK β · 그때 세움 여부
    sc_sum = torch.zeros(n, device=dev); sc_lt = torch.zeros(n, device=dev)
    drop_kind = torch.full((n,), -1, dtype=torch.long, device=dev)  # 2026-11-11 떨어뜨림 종료 순간: 0 퍽 xy 상자 위, 1 상자 밖 (바닥 낙하 가능), 2 이미 상자 윗면 3 cm 아래 (바닥)
    T = int(cfg.episode_length_s / u.step_dt)
    if args_cli.nb_diag:  # 시작 배치: 누운 퍽 직사각형 (60 x 30) 과 옆 퍽 원기둥 (반지름 15 mm 가정) 사이 수평 틈, 긴 축과 옆 퍽 방향 사이 각 (0 = 긴 축 끝 쪽, 90 = 긴 옆면 쪽)
        _e = u.vec_base(u.long_axis_w())[:, :2]
        _e = _e / torch.linalg.norm(_e, dim=1, keepdim=True).clamp(min=1e-6)
        _pp = u.to_base(u.puck_pos_w())[:, :2]
        _np = u.to_base(u._obj("puckN").data.root_pos_w.torch)[:, :2]
        _d = _np - _pp
        _lx = (_d * _e).sum(1).abs() - 0.03
        _ly = (_d[:, 0] * -_e[:, 1] + _d[:, 1] * _e[:, 0]).abs() - 0.015
        nb_gap0 = torch.sqrt(_lx.clamp(min=0) ** 2 + _ly.clamp(min=0) ** 2) - 0.015
        nb_ang0 = torch.rad2deg(torch.acos(((_d * _e).sum(1).abs() / torch.linalg.norm(_d, dim=1).clamp(min=1e-6)).clamp(max=1.0)))
        nb_dist0 = torch.linalg.norm(_d, dim=1)
        tp_move = torch.zeros(n, device=dev); tn_min = torch.full((n,), 9.0, device=dev); nbw_mv = torch.zeros(n, device=dev); box_mv = torch.zeros(n, device=dev); root_mv = torch.zeros(n, device=dev); ryaw_d = torch.zeros(n, device=dev); nbb_mv = torch.zeros(n, device=dev); g_nb_min = torch.full((n,), 9.0, device=dev); nb_cause = {}
        nb_side = torch.sign(_np[:, 1] - _pp[:, 1])  # 옆 퍽이 누운 퍽의 base y + 쪽 (1) / − 쪽 (−1)
    for t in range(T + 2):
        ml_ = u.max_lift.clone()
        snap = dict(grasp=u.grasp_seen.clone(), center=u.center_ok.clone(), lift10=u.lift10.clone(), stood=u.stood.clone(), placed=u.released.clone())
        if args_cli.dump_e1:
            rq_, rp_ = u.robot.data.root_link_quat_w.torch, u.robot.data.root_link_pos_w.torch
            from isaaclab.utils.math import quat_mul as _qm
            iq_ = torch.cat((-rq_[:, :3], rq_[:, 3:]), dim=1)  # 켤레 (x, y, z, w) (11-11 04:1x 고침, 전에는 (w, x, y, z) 로 봄 · root 거의 단위라 영향 작음)
            e1_now = {}
            for k_ in ("puckT_box", "boxA", "puckN"):
                o_ = u._obj(k_).data
                e1_now[k_] = (u.to_base(o_.root_pos_w.torch), _qm(iq_, o_.root_quat_w.torch))
            jq_ = u.robot.data.joint_pos.torch
            e1_q, e1_f = jq_[:, u._arm].clone(), torch.stack((jq_[:, u._gl[0]], jq_[:, u._gr[0]]), dim=1)
            e1_t = u.arm_target.clone()
            e1_g = u.to_base(u.grasp_w())  # 팔 관절 목표 (처짐 = 목표 − 측정, chain45 에서 같은 힘 균형으로 이어지게)
        if args_cli.dump_end:
            pb_, nb_ = u.to_base(u.puck_pos_w()), u.to_base(u._obj("puckN").data.root_pos_w.torch)[:, :2]
            rq = u.robot.data.root_link_quat_w.torch
            pq = torch.cat((-rq[:, :3], rq[:, 3:]), dim=1)  # 켤레 (x, y, z, w) (11-11 04:1x 고침)
            from isaaclab.utils.math import quat_mul
            pq = quat_mul(pq, u.puck_quat_w())
        live = ~ended
        j3_hi += (live & (u.robot.data.joint_pos.torch[:, u._arm[2]].abs() > 1.5)).float()
        steps += live.float()
        ab_hit += (live & u._contact("arm_box_contact")).float()
        from isaaclab.utils.math import quat_apply as _qa
        _rq = u.robot.data.root_link_quat_w.torch
        _bx = _qa(_rq, torch.tensor([[1.0, 0.0, 0.0]], device=dev).expand(n, 3))
        _bz = _qa(_rq, torch.tensor([[0.0, 0.0, 1.0]], device=dev).expand(n, 3))
        root_yaw_max = torch.maximum(root_yaw_max, torch.where(live, torch.atan2(_bx[:, 1], _bx[:, 0]).abs(), root_yaw_max))
        root_tilt_max = torch.maximum(root_tilt_max, torch.where(live, torch.acos(_bz[:, 2].clamp(-1, 1)), root_tilt_max))
        if args_cli.nb_diag:  # 누운 퍽 수평 이동 (시작에서), 누운 퍽 중심-옆 퍽 중심 최소 거리, 잡는 점-옆 퍽 중심 최소 수평 거리
            _pp_ = u.to_base(u.puck_pos_w())[:, :2]; _np_ = u.to_base(u._obj("puckN").data.root_pos_w.torch)[:, :2]; _g_ = u.to_base(u.grasp_w())[:, :2]
            if t == 0:
                tp0 = _pp_.clone()
            tp_move = torch.where(live, torch.linalg.norm(_pp_ - tp0, dim=1), tp_move)
            tn_min = torch.where(live, torch.minimum(tn_min, torch.linalg.norm(_pp_ - _np_, dim=1)), tn_min)
            g_nb_min = torch.where(live, torch.minimum(g_nb_min, torch.linalg.norm(_g_ - _np_, dim=1)), g_nb_min)
            _nw_ = u._obj("puckN").data.root_pos_w.torch[:, :2]; _bw_ = u._obj("boxA").data.root_pos_w.torch[:, :2]
            _rw_ = u.robot.data.root_link_pos_w.torch[:, :2]
            _rx_ = _qa(u.robot.data.root_link_quat_w.torch, torch.tensor([[1.0, 0.0, 0.0]], device=dev).expand(n, 3))
            _ry_ = torch.atan2(_rx_[:, 1], _rx_[:, 0])
            if t == 0:
                nw0, bw0, rw0, ry0 = _nw_.clone(), _bw_.clone(), _rw_.clone(), _ry_.clone()
            nbw_mv = torch.where(live, torch.linalg.norm(_nw_ - nw0, dim=1), nbw_mv)
            box_mv = torch.where(live, torch.linalg.norm(_bw_ - bw0, dim=1), box_mv)
            root_mv = torch.where(live, torch.linalg.norm(_rw_ - rw0, dim=1), root_mv)
            ryaw_d = torch.where(live, torch.rad2deg((_ry_ - ry0 + math.pi) % (2 * math.pi) - math.pi), ryaw_d)
            nbb_mv = torch.where(live, torch.linalg.norm(_np_ - u.nb_xy0, dim=1), nbb_mv)
        if args_cli.obs_hold != "truth":
            _ho, _ht = u.ho_state, u.hold_state
            ho_fp += (live & _ho & ~_ht).float(); ho_fn += (live & ~_ho & _ht).float(); ho_st += live.float()
            _sg = torch.where(u.released, 3, torch.where(u.placed, 2, torch.where(u.was_held, 1, 0)))
            for _k, _m in enumerate((live & _ho & ~_ht, live & ~_ho & _ht)):
                ho_stage[_k] += torch.bincount(_sg[_m], minlength=4)[:4].float()
        with torch.inference_mode():
            a = pol(obs["policy"])
        _aw, _ = u.ee_axes_b(); _h = u.h_base()
        _bfk = torch.atan2((_aw * _h).sum(1), -_aw[:, 2])
        _op = live & torch.isnan(open_b) & u.was_held & (u.max_lift > cfg.lift_ok) & (a[:, 5] > 0)
        open_b = torch.where(_op, u.beta, open_b); open_bfk = torch.where(_op, _bfk, open_bfk); open_st = torch.where(_op, u.stood, open_st)
        obs, _, term, trunc, _ = env.step(a)
        if hasattr(u, "vl_scale"):
            sc_sum += live.float() * u.vl_scale; sc_lt += (live & (u.vl_scale < 0.999)).float()
        done = (term | trunc) & ~ended
        if done.any():
            for k in st:
                st[k] |= done & snap[k]
            nolift |= done & snap["grasp"] & (ml_ < cfg.lift_ok)
            ml_end = torch.where(done, ml_, ml_end)
            s_ = done & u.t_succ
            succ |= s_
            t_succ = torch.where(s_, torch.full_like(t_succ, (t + 1) * u.step_dt), t_succ)
            for k, v in (("drop", u.t_drop), ("topple", u.t_topple), ("nb", u.t_nb), ("self", u.t_self), ("arm_body", u.t_ab), ("body_box", u.t_bb), ("badspot", u.t_badspot), ("nan", u.t_nan), ("open_away", u.t_openaway)):
                why[k] |= done & v
            why["timeout"] |= done & trunc & ~term
            gx_end = torch.where(s_, u.dbg_grasp_b[:, 0], gx_end)
            dk = torch.where(u.dbg_puck_z < 0.070 - 0.03, 2, torch.where(u.dbg_on_box, 0, 1))
            drop_kind = torch.where(done & u.t_drop, dk, drop_kind)
            # 2026-11-11 r2e 준비 (RL1): 옆 퍽 닿음 종료가 난 단계 = 다가가기 (쥐기 전) · 들기 · 세우기 (쥔 뒤 세우기 전) · 놓기 (세운 뒤 벌리기 전) · 손 빼기 (벌린 뒤)
            nbd = done & u.t_nb
            if args_cli.nb_diag and nbd.any():
                for i in torch.nonzero(nbd).flatten().tolist():
                    nb_cause[i] = [bool(x) for x in u.t_nb_parts[i].tolist()] + [round(float(tp_move[i]) * 1000, 1), round(float(tn_min[i]) * 1000, 1), round(float(g_nb_min[i]) * 1000, 1), round(float(nbb_mv[i]) * 1000, 1), round(float(nbw_mv[i]) * 1000, 1), round(float(box_mv[i]) * 1000, 1), round(float(root_mv[i]) * 1000, 1), round(float(ryaw_d[i]), 2)]
            if nbd.any():
                for i in torch.nonzero(nbd).flatten().tolist():
                    stg = "손 빼기" if bool(snap["placed"][i]) else ("놓기" if bool(snap["stood"][i]) else ("들기 · 세우기" if bool(snap["grasp"][i]) else "다가가기"))
                    nb_stage[stg] = nb_stage.get(stg, 0) + 1
            if args_cli.dump_end:
                for i in torch.nonzero(s_).flatten().tolist():
                    end_state[i] = dict(puck_p=pb_[i].tolist(), puck_q=pq[i].tolist(), box_c=u.box_c[i].tolist(), box_yaw=float(u.box_yaw[i]),
                                        nb_p=nb_[i].tolist(), nb_on=bool(nb_on[i]))
            if args_cli.dump_e1:
                for i in torch.nonzero(s_ & (ab_hit == 0)).flatten().tolist():
                    e1[i] = dict(env=i, q=e1_q[i].tolist(), q_tgt=e1_t[i].tolist(), g=e1_g[i].tolist(), fing=e1_f[i].tolist(), nb_on=bool(nb_on[i]),
                                 **{f"{k_}_p": v[0][i].tolist() for k_, v in e1_now.items()}, **{f"{k_}_q": v[1][i].tolist() for k_, v in e1_now.items()})
            ended |= done
        if bool(ended.all()):
            break
    N = n
    pct = lambda m: f"{int(m.sum())}/{N} ({m.float().mean().item() * 100:.1f} %)"
    print(f"[LUEval] **R1 세워 놓기 성공 {pct(succ)}**", flush=True)
    for nm in ("grasp", "center", "lift10", "stood", "placed"):
        print(f"[LUEval]   단계 {nm}: {pct(st[nm])}", flush=True)
    print(f"[LUEval]   옆 퍽 있음 {int(nb_on.sum())} 대 성공 {int((succ & nb_on).sum())}, 없음 {int((~nb_on).sum())} 대 성공 {int((succ & ~nb_on).sum())}", flush=True)
    print(f"[LUEval]   종료 이유: " + ", ".join(f"{k} {int(v.sum())}" for k, v in why.items()), flush=True)
    print(f"[LUEval]   쥠 판정 뒤 {cfg.lift_ok * 100:.0f} cm 못 든 채 끝난 대 {int(nolift.sum())} (쥠 판정 대 {int(st['grasp'].sum())} 중)", flush=True)
    print(f"[LUEval] 옆 퍽 닿음 종료 단계: {nb_stage} (옆 퍽 있는 대 {int(nb_on.sum())}, 그중 닿음 {int((why['nb'] & nb_on).sum())} = {(why['nb'] & nb_on).sum().item() / max(1, int(nb_on.sum())) * 100:.2f} %)", flush=True)
    print(f"[LUEval] R4 옆 퍽 닿음 · 밀림 {int(why['nb'].sum())} 대, 팔-몸체 {int(why['arm_body'].sum())} 대, 자기 충돌 {int(why['self'].sum())} 대", flush=True)
    print(f"[LUEval] 관절 3 > 1.5 rad 스텝 비율 {j3_hi.sum().item() / max(1.0, steps.sum().item()) * 100:.2f} % (그런 스텝이 있었던 대 {int((j3_hi > 0).sum())})", flush=True)
    print(f"[LUEval] 팔-상자 접촉 (> {cfg.contact_force} N) 있었던 대 {int((ab_hit > 0).sum())}, 그중 성공 {int(((ab_hit > 0) & succ).sum())}, 스텝 비율 {ab_hit.sum().item() / max(1.0, steps.sum().item()) * 100:.2f} %", flush=True)
    print(f"[LUEval] 로봇 root 요 최대 |°| {torch.rad2deg(root_yaw_max).max().item():.4f} (중앙 {torch.rad2deg(root_yaw_max).median().item():.4f}), 기울기 최대 ° {torch.rad2deg(root_tilt_max).max().item():.4f} | root 쿼터니언 (x, y, z, w) 대 0: {[round(v, 6) for v in u.robot.data.root_link_quat_w.torch[0].tolist()]}", flush=True)
    print(f"[LUEval] **접촉 없는 성공 (팔-상자 접촉 없음) {pct(succ & (ab_hit == 0))}**", flush=True)
    print(f"[LUEval] 관측 쥠 {cfg.obs_hold_mode}" + (f": 관측만 쥠 스텝 {ho_fp.sum().item() / max(1.0, ho_st.sum().item()) * 100:.2f} % (그런 대 {int((ho_fp > 0).sum())}, 그중 성공 {int(((ho_fp > 0) & succ).sum())}), 판정만 쥠 스텝 {ho_fn.sum().item() / max(1.0, ho_st.sum().item()) * 100:.2f} % (그런 대 {int((ho_fn > 0).sum())}, 그중 성공 {int(((ho_fn > 0) & succ).sum())}), 다른 스텝 대당 최대 {int((ho_fp + ho_fn).max().item())}" if cfg.obs_hold_mode != "truth" else ""), flush=True)
    if cfg.obs_hold_mode != "truth":
        print("[LUEval] 관측 쥠 다른 스텝 단계별 (쥐기 전 / 쥔 뒤 놓기 전 / 놓은 뒤 벌리기 전 / 벌림 명령 뒤): 관측만 " + " / ".join(str(int(v)) for v in ho_stage[0].tolist()) + ", 판정만 " + " / ".join(str(int(v)) for v in ho_stage[1].tolist()) + f" (산 스텝 합 {int(ho_st.sum().item())})", flush=True)
    if succ.any():
        m = ml_end[succ]
        print(f"[LUEval] 성공한 대 최대 든 높이 cm: 최소 {m.min().item() * 100:.1f} 중앙 {m.median().item() * 100:.1f} 최대 {m.max().item() * 100:.1f}, 10 cm 단계 통과 {int((succ & st['lift10']).sum())}, 가운데 쥠 {int((succ & st['center']).sum())}", flush=True)
    print(f"[LUEval] 떨어뜨림 종료 나눔 (종료 순간): 퍽 xy 상자 위 {int((drop_kind == 0).sum())}, 상자 밖 (바닥 낙하 가능) {int((drop_kind == 1).sum())}, 이미 바닥 쪽 (상자 윗면 3 cm 아래) {int((drop_kind == 2).sum())}", flush=True)
    gx = gx_end[~torch.isnan(gx_end)]
    if len(gx):
        q_ = lambda p: gx.quantile(p).item()
        print(f"[LUEval] 성공 순간 잡는 점 base x m: 중앙 {q_(0.5):.3f} 90 % {q_(0.9):.3f} 최대 {gx.max().item():.3f}, 0.38 넘음 {int((gx > 0.38).sum())}/{len(gx)}", flush=True)
    print(f"[LUEval] 팔 관절 속도 제한 v_max {cfg.arm_vmax} rad/s (0 = 없음), 방식 {cfg.arm_vlimit_mode}, 줄인 스텝 비율 {sc_lt.sum().item() / max(1.0, steps.sum().item()) * 100:.1f} %, 평균 배율 {sc_sum.sum().item() / max(1.0, steps.sum().item()):.3f}", flush=True)
    for _nm, _m in (("떨어뜨림 대", why["drop"]), ("성공 대", succ), ("그 밖 실패 대", ~succ & ~why["drop"])):
        _m = _m & ~torch.isnan(open_b)
        if _m.any():
            _d = (open_b - open_bfk)[_m]
            print(f"[LUEval] 원인 확인 {_nm} {int(_m.sum())} 대, 든 뒤 처음 벌림 명령 스텝: 관측 β 중앙 {open_b[_m].median().item():.2f} · FK β 중앙 {open_bfk[_m].median().item():.2f} rad, 차 (관측 − FK) 중앙 {_d.median().item():.2f} · 90 % {_d.quantile(0.9).item():.2f} · 최대 {_d.max().item():.2f}, 그때 이미 세움 (환경 판정) {int((open_st & _m).sum())}", flush=True)
    ts = t_succ[~torch.isnan(t_succ)]
    if len(ts):
        print(f"[LUEval] R5 성공 시각 s: 중앙 {ts.median().item():.2f} (최소 {ts.min().item():.2f} 최대 {ts.max().item():.2f})", flush=True)
    if args_cli.nb_diag:
        m = nb_on
        rows = [dict(env=i, gap=round(float(nb_gap0[i]), 4), ang=round(float(nb_ang0[i]), 1), dist=round(float(nb_dist0[i]), 4), side=int(nb_side[i]), nb=bool(why["nb"][i]), succ=bool(succ[i]), cause=nb_cause.get(i))
                for i in torch.nonzero(m).flatten().tolist()]
        with open(args_cli.nb_diag, "w") as f:
            json.dump(dict(seed=args_cli.seed, policy=args_cli.policy, rows=rows), f)
        g = nb_gap0[m]; hit = why["nb"][m]
        print(f"[LUEval] 옆 퍽 진단: 옆 퍽 있는 대 {int(m.sum())}, 시작 틈 mm 최소 {g.min().item()*1000:.1f} 중앙 {g.median().item()*1000:.1f} 최대 {g.max().item()*1000:.1f}; 닿은 대 (틈 mm, 각 °): "
              + ", ".join(f"({nb_gap0[i].item()*1000:.1f}, {nb_ang0[i].item():.0f})" for i in torch.nonzero(m & why["nb"]).flatten().tolist()), flush=True)
        print("[LUEval]   닿은 대 원인 (팔 접촉, 3 mm 밀림, 넘어짐, 누운 퍽 이동 mm, 누운-옆 퍽 중심 최소 mm, 잡는 점-옆 퍽 중심 최소 mm, 옆 퍽 base 이동 mm, 옆 퍽 월드 이동 mm, 상자 월드 이동 mm, 차체 월드 이동 mm, 차체 요 변화 °): " + "; ".join(f"{i}: {v}" for i, v in nb_cause.items()), flush=True)
        _cs = list(nb_cause.values())
        print(f"[LUEval]   원인별 (옆 퍽 닿음 종료 대): 팔 접촉 {sum(1 for v in _cs if v[0])}, 3 mm 밀림 {sum(1 for v in _cs if v[1] and not v[0])}, 넘어짐 {sum(1 for v in _cs if v[2] and not v[0] and not v[1])} (겹치면 앞 원인으로), 판정 좌표 {'월드' if cfg.nb_move_world else 'base'}", flush=True)
        _all = torch.ones(n, dtype=torch.bool, device=dev)
        print(f"[LUEval]   차체 (전체 대, 끝날 때 · 시작 대비): 월드 이동 mm 최대 {root_mv.max().item()*1000:.1f} (99 % {root_mv.quantile(0.99).item()*1000:.1f}), 요 변화 |°| 최대 {ryaw_d.abs().max().item():.2f} (99 % {ryaw_d.abs().quantile(0.99).item():.2f})", flush=True)
        ok_ = m & ~why["nb"]
        print(f"[LUEval]   비교 (안 닿은 옆 퍽 있는 대, 끝날 때): 옆 퍽 base 이동 mm 최대 {nbb_mv[ok_].max().item()*1000:.1f} (99 % {nbb_mv[ok_].quantile(0.99).item()*1000:.1f}), 월드 이동 최대 {nbw_mv[ok_].max().item()*1000:.1f}, 차체 요 변화 |°| 최대 {ryaw_d[ok_].abs().max().item():.2f} (99 % {ryaw_d[ok_].abs().quantile(0.99).item():.2f})", flush=True)
        print(f"[LUEval]   비교: 옆 퍽 있는 대 전체 잡는 점-옆 퍽 중심 최소 mm 중앙 {g_nb_min[m].median().item()*1000:.1f} (10 % {g_nb_min[m].quantile(0.1).item()*1000:.1f}), 누운 퍽 이동 mm 중앙 {tp_move[m].median().item()*1000:.1f} (90 % {tp_move[m].quantile(0.9).item()*1000:.1f})", flush=True)
        for th in (0.010, 0.015, 0.020, 0.025, 0.030):
            sm = g < th
            print(f"[LUEval]   틈 < {th*1000:.0f} mm: {int(sm.sum())} 대 (옆 퍽 있는 대의 {sm.float().mean().item()*100:.1f} %), 그중 닿음 {int((sm & hit).sum())}", flush=True)
    if args_cli.dump_end:
        L = [s for s in end_state if s is not None]
        with open(args_cli.dump_end, "w") as f:
            json.dump(dict(note="P1 세우기 성공 끝 상태 (base)", starts=L), f)
        print(f"[LUEval] 끝 상태 {len(L)} 개 -> {args_cli.dump_end}", flush=True)
    if args_cli.dump_e1:
        with open(args_cli.dump_e1, "w") as f:
            json.dump(dict(note="E1a 접촉 없는 P1 성공 끝 상태 (성공 판정 한 스텝 전, base 좌표 · base 기준 자세, env 번호별, 나머지 null)",
                           seed=args_cli.seed, nb_prob=args_cli.nb_prob, policy=args_cli.policy, num_envs=n, chain_phys=args_cli.chain_phys, starts=e1), f)
        print(f"[LUEval] E1 끝 상태 {sum(1 for v in e1 if v)} 개 -> {args_cli.dump_e1}", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
