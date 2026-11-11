"""누운 직육면체 세우기 `Waffle-LieUpright-Direct` 구현 (2026-11-10 Onbeom). 설정 · 설명은 lieupright_env_cfg.py.

집기 환경 (WaffleSidePickEnv) 의 장면 · 접촉 센서 · 자기 충돌 · 인식 지연 필터를 이어받고, 행동 · 관측 · 보상 · 초기화를 새로 둠.
"""

from __future__ import annotations

import json
import math
import os
from collections.abc import Sequence

import torch

from isaaclab.utils.math import quat_apply, quat_apply_inverse, quat_box_minus, quat_from_euler_xyz, quat_from_matrix, quat_mul

from .dice2_cfg import BOX_H, BOX_SIZE, PUCK_H
from .lie_base_env import WaffleSidePickEnv  # 릴리스 rl-20261110 판 복사본 (dev 에 들어오면 .sidepick_env 로)


class WaffleLieUprightEnv(WaffleSidePickEnv):

    def __init__(self, cfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        self.is_box[:] = True
        z = lambda: torch.zeros(n, device=dev)
        b = lambda: torch.zeros(n, dtype=torch.bool, device=dev)
        self.beta, self.rho = z(), z()
        self.kind = torch.zeros(n, dtype=torch.long, device=dev)  # 0 처음부터, 1 쥐고 든 상태, 2 세운 채
        self.was_held, self.grasp_seen, self.center_ok, self.lift10, self.stood, self.placed, self.released = b(), b(), b(), b(), b(), b(), b()
        self.succ_once = b()
        self.s_g = torch.ones(n, device=dev)  # 쥔 쪽 끝 (긴 축 부호)
        self.ax_hand0 = torch.zeros(n, 3, device=dev)  # 쥔 순간 손 (집게 몸) 좌표 퍽 긴 축
        self.D = torch.zeros(n, 3, device=dev)  # 세울 자리 (base, 퍽 중심)
        self.box_c = torch.zeros(n, 2, device=dev)
        self.box_yaw = z()
        self.nb_xy0 = torch.zeros(n, 2, device=dev)
        self.nb_w0 = torch.zeros(n, 2, device=dev)
        self.perc_out = torch.zeros(n, 3, device=dev)
        self.perc_frozen = b()
        self.box_dh = z()
        self.yaw_bias = z()
        self.phi_prev = z()
        self.hold_ok = torch.zeros(n, dtype=torch.long, device=dev)
        self.max_lift = z()
        self.hold_on = torch.zeros(n, dtype=torch.long, device=dev)
        self.hold_off = torch.zeros(n, dtype=torch.long, device=dev)
        self.hold_state = b()
        self.ho_on = torch.zeros(n, dtype=torch.long, device=dev)  # 2026-11-11 관측용 쥠 (obs_hold_mode)
        self.ho_off = torch.zeros(n, dtype=torch.long, device=dev)
        self.ho_state = b()
        self.vmax_ep = torch.zeros(n, device=dev)  # 2026-11-11 B6: 에피소드 관절 속도 상한 [rad/s] (0 = 없음)
        self.fix_p = torch.zeros(n, 3, device=dev)  # B6 obs_node: 시작 때 잰 퍽 자리 (base, 치우침 포함)
        self.box_c_obs = torch.zeros(n, 2, device=dev)  # 2026-11-11 B7: 관측용 잰 상자 (노드 place_box, ±3 mm · ±1°). 판정은 참 상자 (box_c)
        self.box_yaw_obs = torch.zeros(n, device=dev)
        self.fix_th = torch.zeros(n, device=dev)  # B6 obs_node: 시작 때 잰 긴 축 요 (치우침 포함)
        self.nb_f_prev = torch.full((n,), -1.0, device=dev)  # 2026-11-11 r2e 2 회차 (B5): 다가가기 옆 퍽 여유 퍼텐셜 이전 값 (−1 = 새 에피소드)
        self.starts = {"lifted": [], "stood": [], "above": [], "at": []}  # B2: above · at = 쥐기 전 (벌림)
        fn = cfg.starts_file
        if fn and os.path.exists(fn):
            with open(fn) as f:
                for s in json.load(f)["starts"]:
                    self.starts[s["kind"]].append(s)
        for k in self.starts:
            if self.starts[k]:
                L = self.starts[k]
                self.starts[k] = {key: torch.tensor([s[key] for s in L], dtype=torch.float32, device=dev)
                                  for key in ("q", "fing", "puck_p", "puck_q", "box_c", "box_yaw", "nb_p", "nb_on", "D", "ee_a", "ee_c", "g")}
            else:
                self.starts[k] = None
        print(f"[LieUpright] 시작 상태: " + ", ".join(f"{k} {0 if v is None else len(v['q'])}" for k, v in self.starts.items()) + f" ({fn})", flush=True)

    # ---- 상태 ----
    def long_axis_w(self):
        return quat_apply(self.puck_quat_w(), self._ez)

    def j1_w(self):
        d = self.robot.data
        return d.root_link_pos_w.torch + quat_apply(d.root_link_quat_w.torch, torch.tensor([[self.cfg.joint1_x, 0.0, 0.0]], device=self.device).expand(self.num_envs, 3))

    def ee_axes_b(self):
        q5 = self.robot.data.body_link_quat_w.torch[:, self._a5]
        return self.vec_base(quat_apply(q5, self._ex)), self.vec_base(quat_apply(q5, self._ey))

    def h_base(self):
        gb = self.to_base(self.grasp_w())
        h = torch.stack((gb[:, 0] - self.cfg.joint1_x, gb[:, 1], torch.zeros_like(gb[:, 0])), dim=1)
        return h / h.norm(dim=1, keepdim=True).clamp(min=1e-6)

    def target_R_base(self):
        """열 = (집게 방향 a, 닫는 축 c, 손가락 높이 방향 u). 설명은 cfg 머리말."""
        h = self.h_base()
        zb = self._ez
        t = torch.cross(zb, h, dim=1)
        cb, sb = torch.cos(self.beta)[:, None], torch.sin(self.beta)[:, None]
        a = -cb * zb + sb * h
        u0 = torch.cross(a, t, dim=1)
        cr, sr = torch.cos(self.rho)[:, None], torch.sin(self.rho)[:, None]
        c = cr * t + sr * u0
        u = -sr * t + cr * u0
        return torch.stack((a, c, u), dim=2)

    def set_beta_rho_from(self, ids, a, c, g):
        """현재 집게 방향 a · 닫는 축 c (base) 와 잡는 점 g -> β, ρ."""
        h = torch.stack((g[:, 0] - self.cfg.joint1_x, g[:, 1], torch.zeros_like(g[:, 0])), dim=1)
        h = h / h.norm(dim=1, keepdim=True).clamp(min=1e-6)
        zb = self._ez[: len(ids)]
        self.beta[ids] = torch.atan2((a * h).sum(1), -(a * zb).sum(1)).clamp(*self.cfg.beta_range)
        t = torch.cross(zb, h, dim=1)
        av = -torch.cos(self.beta[ids])[:, None] * zb + torch.sin(self.beta[ids])[:, None] * h
        u0 = torch.cross(av, t, dim=1)
        self.rho[ids] = torch.atan2((c * u0).sum(1), (c * t).sum(1)).clamp(-self.cfg.rho_max, self.cfg.rho_max)

    def holding(self):
        c = self.cfg
        fing = self.robot.data.joint_pos.torch[:, self._gl[0]]
        closed_ok = (fing > c.grasp_finger_range[0]) & (fing < c.grasp_finger_range[1])
        raw = closed_ok & (torch.linalg.norm(self.puck_pos_w() - self.grasp_w(), dim=1) < c.hold_dist)
        # 2026-11-10 B2 진단: 닫히는 도중 손가락이 범위를 지나가는 순간도 쥠으로 잡혀 바로 '떨어뜨림' 종료가 났음
        # -> 히스테리시스: 연속 hold_n 스텝 맞으면 쥠, 연속 hold_n 스텝 아니면 놓음 (스텝마다 _step_terms 에서 한 번 갱신)
        return self.hold_state if hasattr(self, "hold_state") else raw

    def _update_hold(self):
        c = self.cfg
        fing = self.robot.data.joint_pos.torch[:, self._gl[0]]
        closed_ok = (fing > c.grasp_finger_range[0]) & (fing < c.grasp_finger_range[1])
        raw = closed_ok & (torch.linalg.norm(self.puck_pos_w() - self.grasp_w(), dim=1) < c.hold_dist)
        self.hold_on = torch.where(raw, self.hold_on + 1, torch.zeros_like(self.hold_on))
        self.hold_off = torch.where(raw, torch.zeros_like(self.hold_off), self.hold_off + 1)
        self.hold_state = torch.where(self.hold_on >= c.hold_n_on, torch.ones_like(raw), torch.where(self.hold_off >= c.hold_n_off, torch.zeros_like(raw), self.hold_state))
        if c.obs_hold_mode != "truth":  # 2026-11-11 (RL1): 관측용 쥠 = 손가락 범위만 + 같은 히스테리시스 (실물 판정 모사)
            ro = closed_ok
            self.ho_on = torch.where(ro, self.ho_on + 1, torch.zeros_like(self.ho_on))
            self.ho_off = torch.where(ro, torch.zeros_like(self.ho_off), self.ho_off + 1)
            self.ho_state = torch.where(self.ho_on >= c.hold_n_on, torch.ones_like(ro), torch.where(self.ho_off >= c.hold_n_off, torch.zeros_like(ro), self.ho_state))

    def lift(self):
        return self.puck_pos_w()[:, 2] - self.puck_z0

    def box_top_w(self):
        return self.scene.env_origins[:, 2] + BOX_H

    def puck_state(self):
        """(세움 = 쥔 쪽 끝 · 위, 기울기 [rad], 바닥 - 상자 윗면 [m], 상자 위인지)."""
        e = self.long_axis_w()
        up = e[:, 2] * self.s_g  # 쥔 쪽 끝이 위면 +1
        tilt = torch.acos(e[:, 2].abs().clamp(max=1.0))
        bottom = self.puck_pos_w()[:, 2] - PUCK_H / 2 * e[:, 2].abs() - 0.015 * torch.sqrt((1 - e[:, 2] ** 2).clamp(min=0.0)) * 1.414
        pb = self.to_base(self.puck_pos_w())[:, :2] - self.box_c
        cy, sy = torch.cos(self.box_yaw), torch.sin(self.box_yaw)
        lx, ly = cy * pb[:, 0] + sy * pb[:, 1], -sy * pb[:, 0] + cy * pb[:, 1]
        on_box = (lx.abs() < BOX_SIZE / 2) & (ly.abs() < BOX_SIZE / 2)
        self.edge_d = BOX_SIZE / 2 - torch.maximum(lx.abs(), ly.abs())  # 퍽 중심 -> 가까운 상자 가장자리 [m]
        return up, tilt, bottom - self.box_top_w(), on_box

    def perc_puck_b(self):
        c, n, dev = self.cfg, self.num_envs, self.device
        p = self.puck_pos_w().clone()
        p[:, :2] += self.perc_bias[:, :2] + (torch.rand(n, 2, device=dev) * 2 - 1) * c.perc_jitter
        p[:, 2] = self.box_top_w() + self.box_dh + 0.015
        raw = self.to_base(p)
        f = self._perc_filter(raw, "lie")
        near = torch.linalg.norm(self.to_base(self.grasp_w()) - self.perc_out, dim=1) < c.perc_freeze_dist
        self.perc_frozen |= near | self.was_held
        self.perc_out = torch.where(self.perc_frozen[:, None], self.perc_out, f)
        return self.perc_out

    # ---- 행동 ----
    def _pre_physics_step(self, actions):
        c = self.cfg
        self.prev_actions = self.actions.clone()
        self.actions = actions.clamp(-1.0, 1.0)
        self.actions[:, -1] = torch.where(self.released, torch.ones_like(self.actions[:, -1]), self.actions[:, -1])
        q = self.robot.data.joint_pos.torch[:, self._arm]
        b0, r0, tp = self.beta.clone(), self.rho.clone(), self.arm_target.clone()

        def run(sc):  # 행동 앞 5 칸 x sc 로 β · ρ 갱신 + IK (sc: (n,))
            self.beta = (b0 + self.actions[:, 3] * sc * c.beta_scale).clamp(*c.beta_range)
            self.rho = (r0 + self.actions[:, 4] * sc * c.rho_scale).clamp(-c.rho_max, c.rho_max)
            dq = self._ik_pose(self.actions[:, :3] * sc[:, None] * c.cart_scale)
            t_ = torch.clamp(tp + dq, q - c.target_lead_max, q + c.target_lead_max)
            return torch.clamp(t_, self._lo, self._hi), self.beta.clone(), self.rho.clone()
        one = torch.ones(self.num_envs, device=self.device)
        t, bb, rr = run(one)
        self.vl_scale = one.clone()
        vm = self.vmax_ep if c.arm_vmax_range else torch.full_like(one, c.arm_vmax)  # B6: 에피소드마다 다른 상한
        if bool((vm > 0).any()):  # 2026-11-11 (RL 261111 1, 미션 파트 노드와 같게): 직전 보낸 목표 ± v_max × 제어 주기, 다음 스텝은 자른 값에서 이어감
            on = vm > 0
            dv = torch.where(on, vm, torch.full_like(vm, 1e6))[:, None] * (c.sim.dt * c.decimation)
            if c.arm_vlimit_mode == "scale":  # 노드 lieApplyActionLimited (PR #203): 넘으면 행동 앞 5 칸을 같은 비율로 줄여 다시 풂 (이분법 8 번), 남는 몫은 관절별로 자름
                need = ((t - tp).abs() > dv).any(1)
                if need.any():
                    lo, hi = torch.zeros_like(one), one.clone()
                    tl, bl, rl = run(lo)
                    for _ in range(8):
                        mid = 0.5 * (lo + hi)
                        tm, bm, rm = run(mid)
                        ok = ((tm - tp).abs() <= dv).all(1)
                        lo, hi = torch.where(ok, mid, lo), torch.where(ok, hi, mid)
                        tl, bl, rl = torch.where(ok[:, None], tm, tl), torch.where(ok, bm, bl), torch.where(ok, rm, rl)
                    t = torch.where(need[:, None], torch.clamp(tl, tp - dv, tp + dv), t)
                    bb, rr = torch.where(need, bl, bb), torch.where(need, rl, rr)
                    self.vl_scale = torch.where(need, lo, one)
            else:
                t = torch.clamp(t, tp - dv, tp + dv)
        self.beta, self.rho = bb, rr
        self.arm_target = t

    def _ik_pose(self, dp_b):
        c, d = self.cfg, self.robot.data
        rq = d.root_link_quat_w.torch
        J = d.body_link_jacobian_w.torch[:, self._a5][:, :, self._cols]
        q5 = d.body_link_quat_w.torch[:, self._a5]
        rr = quat_apply(q5, self._off)
        Jl = J[:, 0:3] - torch.cross(rr.unsqueeze(-1).expand(-1, -1, 5), J[:, 3:6], dim=1)
        w = c.rot_hold_weight
        Jf = torch.cat((Jl, J[:, 3:6] * w), dim=1)
        qt = quat_mul(rq, quat_from_matrix(self.target_R_base()))
        err = torch.cat((quat_apply(rq, dp_b), quat_box_minus(qt, q5) * w), dim=1)
        JJt = Jf @ Jf.transpose(1, 2) + c.ik_damping * torch.eye(6, device=self.device)
        return (Jf.transpose(1, 2) @ torch.linalg.solve(JJt, err.unsqueeze(-1))).squeeze(-1)

    # ---- 관측 ----
    def _get_observations(self):
        d = self.robot.data
        pb = self.perc_puck_b()
        e = self.vec_base(self.long_axis_w())
        th = torch.atan2(e[:, 1], e[:, 0]) + self.yaw_bias
        if self.cfg.obs_node:  # 2026-11-11 B6: 관측을 미션 노드와 같게 — 퍽 자리 · 긴 축 = 시작 때 잰 값 고정 (노드 lieStart, 쥔 퍽은 팔 카메라 밖 #128)
            pb, th = self.fix_p, self.fix_th
        gb = self.to_base(self.grasp_w())
        a, cax = self.ee_axes_b()
        nb = (self.nb_xy0 if self.cfg.obs_node else self.to_base(self._obj("puckN").data.root_pos_w.torch)[:, :2]) - pb[:, :2]  # B6: 옆 퍽 = 시작 값 (노드)
        nb = torch.where(self.nb_present[:, None], nb, torch.zeros_like(nb))
        held = (self.holding() if self.cfg.obs_hold_mode == "truth" else self.ho_state).float()  # 2026-11-11 obs_hold_mode
        obs = torch.cat((
            d.joint_pos.torch[:, self._arm] - d.default_joint_pos.torch[:, self._arm], d.joint_vel.torch[:, self._arm],
            d.joint_pos.torch[:, self._gl],
            pb, torch.stack((torch.cos(2 * th), torch.sin(2 * th)), dim=1),
            gb, a, cax, self.D,
            nb, self.nb_present.float()[:, None],
            *((self.box_c_obs, torch.stack((torch.cos(self.box_yaw_obs), torch.sin(self.box_yaw_obs)), dim=1)) if self.cfg.obs_box_noise[0] > 0 else
              (self.box_c, torch.stack((torch.cos(self.box_yaw), torch.sin(self.box_yaw)), dim=1))),
            held[:, None],
            torch.stack((torch.cos(self.rho), torch.sin(self.rho), self.beta), dim=1),
            self.actions,
        ), dim=-1)
        up, tilt, dz, _ = self.puck_state()
        priv = torch.cat((
            self.to_base(self.puck_pos_w()), e, self.to_base(self.puck_pos_w()) - gb, up[:, None], tilt[:, None], dz[:, None],
            torch.stack((self.was_held, self.stood, self.placed, self.released), dim=1).float(),
            torch.stack((self._contact("arm_nb_contact"), self._contact("body_box_contact"), self._contact("arm_box_contact")), dim=1).float(),
        ), dim=-1)
        return {"policy": obs, "critic": priv}

    # ---- 보상 ----
    def _get_rewards(self):
        return self._rew  # _step_terms 가 _get_dones 에서 이미 계산 (DirectRLEnv.step 은 종료를 보상보다 먼저 부름)

    def _step_terms(self):
        """이 스텝의 단계 · 보상 · 종료 사건을 한 번에 계산 (2026-11-10 도구 확인: _get_dones 가 먼저 불려 t_drop 이 없던 오류 고침)."""
        c, n, dev = self.cfg, self.num_envs, self.device
        g = self.grasp_w()
        pc = self.puck_pos_w()
        e = self.long_axis_w()
        a_w = quat_apply(self.robot.data.body_link_quat_w.torch[:, self._a5], self._ex)
        self._update_hold()
        hold = self.holding()
        lift = self.lift()
        along = ((g - pc) * e).sum(1)
        s_now = torch.where(along >= 0, 1.0, -1.0)
        s_use = torch.where(self.was_held, self.s_g, s_now)
        G = pc + e * (s_use * c.grasp_dx)[:, None] - a_w * c.grasp_dz_a
        dist = torch.linalg.norm(G - g, dim=1)
        # 쥔 순간 (처음, 상자 위에서)
        new_grasp = hold & ~self.was_held & (lift < c.lift_ok)
        off_err = (along.abs() - c.grasp_dx).abs()
        self.s_g = torch.where(new_grasp, s_now, self.s_g)
        q5 = self.robot.data.body_link_quat_w.torch[:, self._a5]
        ax_h = quat_apply_inverse(q5, e)
        self.ax_hand0 = torch.where(new_grasp[:, None], ax_h, self.ax_hand0)
        good = new_grasp & (off_err < c.grasp_tol) & (along * s_now > 0) & ~self.center_ok  # RL1 검토 2: 에피소드당 한 번
        bad = new_grasp & (off_err > c.grasp_bad)
        self.center_ok |= good
        self.grasp_seen |= new_grasp
        self.was_held |= hold
        # RL1 검토 1: 들기 전 (3 cm 아래) · 세우기 전에 놓으면 '쥔 적 없음' 으로 되돌림 -> f1 거리 shaping · new_grasp (s_g · ax_hand0 · 중심 쥠) 다시 살아남
        self.was_held &= ~(~hold & (self.max_lift < c.lift_ok) & ~self.stood)
        up, tilt, dz, on_box = self.puck_state()
        # 2026-11-11 평가 기록용 (리셋 전 값 보관, 학습 · 판정에는 안 씀): 퍽 상자 위 xy 여부, 퍽 높이 (환경 원점 기준), 잡는 점 (base)
        self.dbg_on_box, self.dbg_puck_z, self.dbg_grasp_b = on_box.clone(), (pc[:, 2] - self.scene.env_origins[:, 2]).clone(), self.to_base(g).clone()
        self.lift10 |=hold & (lift > c.lift_full - 0.005)
        self.stood |= hold & (up > c.stand_up_ok) & (lift > c.lift_ok)
        # 세울 자리까지 (퍽 중심 기준, 세운 뒤)
        Dw = self.robot.data.root_link_pos_w.torch + quat_apply(self.robot.data.root_link_quat_w.torch, self.D)
        dD = torch.linalg.norm(pc - Dw, dim=1)
        placed_now = on_box & (dz.abs() < c.place_dz) & (tilt < c.place_tilt) & self.stood
        rel_now = placed_now & (self.actions[:, -1] > 0) & ~self.released
        # 2026-11-10 검토 A: 다시 잡을 수 있는 자리 (세울 자리 3 cm 안 · 가장자리 25 mm 넘게 · 옆 퍽 7 cm 넘게)
        nbp = self.to_base(self._obj("puckN").data.root_pos_w.torch)[:, :2]
        d_nb = torch.where(self.nb_present, torch.linalg.norm(self.to_base(pc)[:, :2] - nbp, dim=1), torch.full_like(dD, 1.0))
        spot_ok = (dD <= c.spot_tol) & (self.edge_d >= c.spot_edge_min) & (d_nb >= c.spot_nb)
        # 2026-11-11 B7: 노드 놓음 규칙 (manipulator_node lieTick) — 든 뒤 처음 벌림이 (잡는 점 + a × 0.011) – D 수평 > 30 mm · 위 > 60 mm · β < 1.4 면 종료 (노드 'D 밖에서 벌림' 실패)
        self.t_openaway = torch.zeros_like(self.released)
        if c.node_release_rule:
            gb_ = self.to_base(g)
            ab_, _ = self.ee_axes_b()
            opening = self.was_held & (self.max_lift > c.lift_ok) & ~self.released & (self.actions[:, -1] > 0)
            dh_ = torch.linalg.norm(gb_[:, :2] + ab_[:, :2] * c.release_center_offset - self.D[:, :2], dim=1)
            self.t_openaway = opening & ((dh_ > c.release_xy) | (gb_[:, 2] - self.D[:, 2] > c.release_z) | (self.beta < c.release_beta))
        self.released |= rel_now
        self.placed |= placed_now
        # 퍼텐셜 (단계 4 개, 0 ~ 1)
        c_w = quat_apply(q5, self._ey)  # 닫는 축
        align = 1 - (c_w * e).sum(1).abs()  # B2-b: 닫는 축 ⟂ 긴 축 = 1
        f1 = torch.where(self.was_held, torch.ones_like(dist), (1 - (dist / c.reach_norm).clamp(0, 1)) * (1 - c.f1_align_w + c.f1_align_w * align))
        f2 = torch.where(self.stood | self.released, torch.ones_like(lift), hold.float() * (lift / c.lift_full).clamp(0, 1))
        f3 = torch.where(self.released, torch.ones_like(up), torch.where(self.stood, torch.ones_like(up), hold.float() * up.clamp(0, 1)))
        f4 = torch.where(self.released, torch.ones_like(dD), self.stood.float() * hold.float() * (1 - (dD / c.spot_norm).clamp(0, 1)))
        phi = f1 + f2 + f3 + f4
        first = self.episode_length_buf <= 1
        prog = torch.where(first, torch.zeros_like(phi), phi - self.phi_prev)
        self.phi_prev = phi
        # 성공: 놓은 뒤 섬 · 손 뺌, 1 s
        gd = torch.linalg.norm(pc - g, dim=1)
        ok = self.released & (tilt < c.place_tilt) & (gd > c.release_clear) & on_box
        self.hold_ok = torch.where(ok, self.hold_ok + 1, torch.zeros_like(self.hold_ok))
        stable = self.hold_ok >= int(c.success_hold_s / (c.sim.dt * c.decimation))
        succ = stable & spot_ok & ~self.succ_once
        self.t_badspot = stable & ~spot_ok  # 섰지만 다시 잡기 어려운 자리: 성공 없이 종료
        self.succ_once |= succ
        self.succeeded |= succ
        # 미끄러짐 (손에 대한 축이 쥔 순간보다 10° 넘게 돎)
        cosd = (ax_h * self.ax_hand0).sum(1).abs().clamp(max=1.0)
        slip = (torch.acos(cosd) - math.radians(c.slip_deg)).clamp(min=0.0) * (hold & self.was_held).float()
        # 종료 사건
        # 2026-11-10 B2 진단: 떨어뜨림 = 쥐고 3 cm 넘게 든 뒤 놓침 (상자 위에서 쥐다 놓친 것은 다시 쥘 수 있음) 또는 퍽이 상자 밖으로 떨어짐
        lifted_once = self.max_lift > c.lift_ok
        self.t_drop = lifted_once & self.was_held & ~hold & ~self.released & ~(placed_now & self.stood) & (lift > -0.01) & ((~on_box) | (tilt > c.place_tilt) | (dz > c.place_dz))
        self.t_drop |= ~self.released & (self.puck_pos_w()[:, 2] < self.puck_z0 - 0.03)
        self.t_topple = self.released & (tilt > c.topple_tilt)
        _nbd = torch.linalg.norm(self._obj("puckN").data.root_pos_w.torch[:, :2] - self.nb_w0, dim=1) if c.nb_move_world else torch.linalg.norm(nbp - self.nb_xy0, dim=1)
        # 2026-11-11 r2f 뒤 진단: base 좌표 밀림은 차체가 0.2 ~ 0.4° 돌면 옆 퍽이 가만히 있어도 2 ~ 3 mm 로 보임 → nb_move_world = True 면 월드 좌표 (시작 자리) 로 잼
        _nbc, _nbm, _nbt = self._contact("arm_nb_contact"), _nbd > c.nb_move_max, self.nb_toppled()
        self.t_nb_parts = torch.stack((_nbc, _nbm, _nbt), dim=1) & self.nb_present[:, None]  # 2026-11-11 진단 (평가만 읽음): 팔 접촉 · 3 mm 밀림 · 넘어짐
        self.t_nb = self.nb_present & (_nbc | _nbm | _nbt)
        self.t_self = self.self_bad()
        self.t_ab = self.arm_body_hit()
        self.t_bb = self._contact("body_box_contact")
        self.t_succ = succ
        arm_box = self._contact("arm_box_contact")
        # 2026-11-11 B5 (RL1 승인: r2d 옆 퍽 닿음은 거의 '다가가기' 단계): 쥐기 전 손가락 · 집게 몸 · 잡는 점과 옆 퍽 (원기둥) 의 수평 여유 퍼텐셜
        #   f = clamp((수평 거리 − 15 mm) / nb_margin, 0, 1), 보상 = rew_nb_pot × (f − 이전 f) (옆 퍽 있고 아직 안 쥔 동안만)
        rew_nbp = torch.zeros(n, device=dev)
        if c.rew_nb_pot != 0.0:
            pts = torch.cat((self.robot.data.body_link_pos_w.torch[:, self._fingers], self.grasp_w()[:, None]), dim=1)
            nbw = self._obj("puckN").data.root_pos_w.torch
            dxy = torch.linalg.norm(pts[:, :, :2] - nbw[:, None, :2], dim=-1).amin(dim=1) - 0.015
            f_nb = (dxy / c.nb_margin).clamp(0.0, 1.0)
            act = self.nb_present & ~self.was_held
            prev = torch.where(self.nb_f_prev < 0, f_nb, self.nb_f_prev)
            rew_nbp = torch.where(act, c.rew_nb_pot * (f_nb - prev), rew_nbp)
            self.nb_f_prev = f_nb
        self.t_abx = arm_box & bool(c.arm_box_term)  # 2026-11-11 r2d (RL1 승인): 팔-상자 접촉 1 N 넘으면 종료 (실패)
        rew = (c.rew_progress * prog + c.rew_center_grasp * good.float() + c.rew_off_grasp * bad.float() + torch.where(spot_ok, c.rew_placed, c.rew_placed_bad) * rel_now.float()
               + c.rew_success * succ.float() + c.rew_drop * self.t_drop.float() + c.rew_topple * self.t_topple.float() + c.rew_nb * self.t_nb.float()
               + c.rew_self * self.t_self.float() + c.rew_arm_body * self.t_ab.float() + c.rew_body_box * self.t_bb.float()
               + c.rew_slip * slip + c.rew_arm_box * arm_box.float() + c.rew_arm_box_term * self.t_abx.float() + rew_nbp + c.rew_time
               + c.rew_action_rate * torch.sum(torch.square(self.actions - self.prev_actions), dim=1) + c.rew_open_away * self.t_openaway.float())
        self.max_lift = torch.maximum(self.max_lift, lift * hold.float())
        # 2026-11-11 r2e 04:31 보상 NaN 으로 학습이 멈춤 (rsl_rl check_nan): 물리 상태가 깨진 대 (위치 · 관절 · 보상이 유한하지 않음) 는 보상 0 · 종료로 막고 셈
        fin = torch.isfinite(rew) & torch.isfinite(pc).all(1) & torch.isfinite(self.robot.data.joint_pos.torch).all(1) & torch.isfinite(g).all(1)
        self.t_nan = ~fin
        # RL1 11-11: 처음 난 t_nan 한 번만 기록 (대 · 에피소드 스텝 · 시작 종류 · 마지막 유한 상태)
        if not hasattr(self, "_nan_last_pc"):
            self._nan_last_pc, self._nan_last_q, self._nan_logged = pc.clone(), self.robot.data.joint_pos.torch.clone(), False
        if self.t_nan.any() and not self._nan_logged:
            i = int(torch.nonzero(self.t_nan)[0])
            print(f"[LieUpright] 처음 t_nan: 대 {i}, 에피소드 스텝 {int(self.episode_length_buf[i])}, 시작 종류 {int(self.kind[i])} (0 처음부터 · 1 쥐고 듦 · 2 세움 · 3 above · 4 at), "
                  f"보상 유한 {bool(torch.isfinite(rew[i]))}, 마지막 유한 퍽 위치 (월드) {[round(v, 4) for v in self._nan_last_pc[i].tolist()]}, "
                  f"마지막 유한 관절 {[round(v, 3) for v in self._nan_last_q[i].tolist()]}, 이 스텝 nan 대 {int(self.t_nan.sum())}", flush=True)
            self._nan_logged = True
        self._nan_last_pc = torch.where(fin[:, None], pc, self._nan_last_pc)
        self._nan_last_q = torch.where(fin[:, None], self.robot.data.joint_pos.torch, self._nan_last_q)
        rew = torch.where(fin, rew, torch.zeros_like(rew))
        self.max_lift = torch.nan_to_num(self.max_lift, nan=0.0)
        log = self.extras.setdefault("log", {})
        log["Metrics/holding_ratio"] = hold.float().mean().item()
        log["Metrics/beta"] = self.beta.mean().item()
        self._rew = rew

    def _get_dones(self):
        self._step_terms()
        tipped = self.robot.data.projected_gravity_b.torch[:, 2] > -0.6
        base_move = torch.linalg.norm(self.robot.data.root_link_pos_w.torch[:, :2] - self.root0[:, :2], dim=1) > self.cfg.base_move_max
        end = tipped | base_move | self.t_openaway | self.t_drop | self.t_topple | self.t_nb | self.t_self | self.t_ab | self.t_bb | self.t_succ | self.t_badspot | self.t_abx | self.t_nan
        return end, self.episode_length_buf >= self.max_episode_length - 1

    # ---- 초기화 ----
    def _log_episode(self, env_ids):
        log = self.extras.setdefault("log", {})
        ok = self.succeeded[env_ids].float()
        log["Metrics/success_rate"] = ok.mean().item()
        k = self.kind[env_ids]
        for kk, nm in ((0, "s0"), (1, "s1"), (2, "s2"), (3, "s3"), (4, "s4")):
            m = k == kk
            if m.any():
                log[f"Metrics/{nm}_success"] = ok[m].mean().item()
                if kk >= 3:
                    log[f"Metrics/{nm}_grasp"] = self.grasp_seen[env_ids[m]].float().mean().item()
        m = k == 0
        if m.any():
            ids = env_ids[m]
            log["Metrics/s0_grasp_center"] = self.center_ok[ids].float().mean().item()
            log["Metrics/s0_lift10"] = self.lift10[ids].float().mean().item()
            log["Metrics/s0_stood"] = self.stood[ids].float().mean().item()
            log["Metrics/s0_placed"] = self.released[ids].float().mean().item()
            log["Metrics/s0_grasp_any"] = self.grasp_seen[ids].float().mean().item()
        if hasattr(self, "t_drop"):
            for nm, t in (("drop", self.t_drop), ("topple", self.t_topple), ("nb", self.t_nb), ("self", self.t_self), ("arm_body", self.t_ab), ("badspot", self.t_badspot), ("arm_box", self.t_abx), ("nan", self.t_nan), ("open_away", self.t_openaway)):
                log[f"Episode_End/{nm}"] = t[env_ids].float().mean().item()
            log["Episode_End/timeout"] = (self.episode_length_buf[env_ids] >= self.max_episode_length - 1).float().mean().item()

    def _mission_layout(self, n):
        """설계 전제 A · B 배치 (chain45 --lie_layout mission 과 같은 식). 반환: 상자 중심 · 요, 누운 퍽 xy · 요, 옆 퍽 xy (base)."""
        c, dev = self.cfg, self.device
        U_ = lambda a, b, sh: a + (b - a) * torch.rand(sh, device=dev)
        hb = BOX_SIZE / 2
        s = torch.where(torch.rand(n, device=dev) < 0.5, 1.0, -1.0)
        TURN = math.asin(c.puck_dx / (c.joint1_x - c.center_x_m))
        byaw0 = U_(-c.yaw_noise, c.yaw_noise, (n,))
        bc0 = torch.stack((c.front_m + c.gap_m + U_(-c.gap_noise, c.gap_noise, (n,)) + hb, U_(-c.lat_noise, c.lat_noise, (n,))), dim=1)
        K = 64
        lp = torch.stack((s[:, None] * c.puck_dx + U_(-c.puck_jit, c.puck_jit, (n, K)), U_(-c.puck_jit, c.puck_jit, (n, K))), dim=-1)
        np_ = torch.stack((-s[:, None] * c.puck_dx + U_(-c.puck_jit, c.puck_jit, (n, K)), U_(-c.puck_jit, c.puck_jit, (n, K))), dim=-1)
        ly = U_(0.0, math.pi, (n, K))
        ca, sa = torch.cos(ly), torch.sin(ly)
        cs = torch.tensor([[1, 1], [1, -1], [-1, 1], [-1, -1]], device=dev, dtype=torch.float32) * torch.tensor([PUCK_H / 2, 0.015], device=dev)
        cxk = lp[..., None, 0] + ca[..., None] * cs[:, 0] - sa[..., None] * cs[:, 1]
        cyk = lp[..., None, 1] + sa[..., None] * cs[:, 0] + ca[..., None] * cs[:, 1]
        in_box = (cxk.abs() <= hb - c.edge_in).all(-1) & (cyk.abs() <= hb - c.edge_in).all(-1)
        d = np_ - lp
        lx = (d[..., 0] * ca + d[..., 1] * sa).abs() - PUCK_H / 2
        lyy = (-d[..., 0] * sa + d[..., 1] * ca).abs() - 0.015
        dist = torch.sqrt(lx.clamp(min=0) ** 2 + lyy.clamp(min=0) ** 2) - 0.015
        k = (in_box & (dist >= c.gap_min_pucks)).float().argmax(dim=1)
        ar = torch.arange(n, device=dev)
        lp_, np__, ly_ = lp[ar, k], np_[ar, k], ly[ar, k]
        cb, sb = torch.cos(byaw0), torch.sin(byaw0)

        def b2r(p):
            return torch.stack((bc0[:, 0] + cb * p[:, 1] - sb * p[:, 0], bc0[:, 1] + sb * p[:, 1] + cb * p[:, 0]), dim=1)
        th = -s * TURN
        ct, st = torch.cos(th), torch.sin(th)
        cen = torch.tensor([c.center_x_m, 0.0], device=dev)

        def rot(p):
            q = p - cen
            return torch.stack((ct * q[:, 0] - st * q[:, 1], st * q[:, 0] + ct * q[:, 1]), dim=1) + cen
        byaw = byaw0 + th
        return rot(bc0), byaw, rot(b2r(lp_)), byaw + ly_, rot(b2r(np__))

    def _stand_spot(self, bc, byaw, nbp, nb_on):
        """세울 자리 (팀 각본 규칙): 상자 가장자리 30 mm 안 격자, 관절1 축 0.27 ~ 0.32 m, 옆 퍽 7 cm 밖, 점수 |r − 0.285| − 0.2 min(옆 퍽 거리, 0.12)."""
        c, dev = self.cfg, self.device
        gu = torch.arange(-(BOX_SIZE / 2 - c.spot_edge), BOX_SIZE / 2 - c.spot_edge + 1e-6, 0.0145, device=dev)
        UU, VV = torch.meshgrid(gu, gu, indexing="xy")
        UU, VV = UU.reshape(-1), VV.reshape(-1)
        cy, sy = torch.cos(byaw)[:, None], torch.sin(byaw)[:, None]
        cx = bc[:, 0:1] + cy * UU - sy * VV
        cyy = bc[:, 1:2] + sy * UU + cy * VV
        rr = torch.hypot(cx - c.joint1_x, cyy)
        dn = torch.where(nb_on[:, None], torch.hypot(cx - nbp[:, 0:1], cyy - nbp[:, 1:2]), torch.full_like(rr, 1.0))
        okd = (rr >= c.spot_r[0]) & (rr <= c.spot_r[1]) & (dn >= c.spot_nb)
        sc = (rr - 0.285).abs() - 0.2 * dn.clamp(max=0.12)
        sc = torch.where(okd, sc, sc + 10.0)
        k = sc.argmin(dim=1)
        ar = torch.arange(len(bc), device=dev)
        z = torch.full((len(bc),), BOX_H + PUCK_H / 2 + 0.002, device=dev) - self.robot.data.default_root_pose.torch[0, 2]  # base 좌표 (차체 기본 높이 뺌)
        return torch.stack((cx[ar, k], cyy[ar, k], z), dim=1)

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        env_ids = torch.as_tensor(env_ids, device=self.device)
        if len(env_ids) > 0 and hasattr(self, "kind"):
            self._log_episode(env_ids)
        super(WaffleSidePickEnv, self)._reset_idx(env_ids)
        c, r, n, dev = self.cfg, self.robot, len(env_ids), self.device
        org = self.scene.env_origins[env_ids]
        root = r.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += org
        r.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        r.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        self.root0[env_ids] = root[:, :3]
        zero = torch.zeros(n, device=dev)

        # 시작 종류
        pr = torch.zeros(5, device=dev)
        pr[:len(c.start_probs)] = torch.tensor(c.start_probs, device=dev)
        for kk, nm in ((1, "lifted"), (2, "stood"), (3, "above"), (4, "at")):
            if self.starts[nm] is None:
                pr[kk] = 0.0
        kind = torch.multinomial(pr / pr.sum(), n, replacement=True)
        self.kind[env_ids] = kind

        # 처음부터 배치 (모두 뽑고 kind 0 에 씀)
        bc, byaw, lxy, lyaw, nxy = self._mission_layout(n)
        nb_on = torch.rand(n, device=dev) < c.nb_prob
        q_lie = quat_mul(quat_from_euler_xyz(zero, zero, lyaw), quat_from_euler_xyz(zero, torch.full((n,), math.pi / 2, device=dev), zero))
        p_puck = torch.cat((lxy, torch.full((n, 1), c.lie_z, device=dev)), dim=1)
        q_puck = q_lie
        q = r.data.default_joint_pos.torch[env_ids].clone()
        q[:, self._arm] = torch.clamp(q[:, self._arm] + (torch.rand(n, 5, device=dev) * 2 - 1) * c.ready_noise, self._lo, self._hi)
        fing = torch.full((n, 2), c.gripper_open, device=dev)
        # 각본 시작 상태
        a0 = torch.zeros(n, 3, device=dev)
        c0 = torch.zeros(n, 3, device=dev)
        g0 = torch.zeros(n, 3, device=dev)
        D = torch.zeros(n, 3, device=dev)
        for kk, nm in ((1, "lifted"), (2, "stood"), (3, "above"), (4, "at")):
            m = kind == kk
            if not m.any() or self.starts[nm] is None:
                continue
            S = self.starts[nm]
            j = torch.randint(0, len(S["q"]), (int(m.sum()),), device=dev)
            q[m.nonzero().flatten()[:, None], self._arm] = S["q"][j]
            fing[m] = S["fing"][j]
            bc[m], byaw[m] = S["box_c"][j], S["box_yaw"][j]
            p_puck[m], q_puck[m] = S["puck_p"][j], S["puck_q"][j]
            nxy[m] = S["nb_p"][j]
            nb_on[m] = S["nb_on"][j] > 0.5
            D[m], a0[m], c0[m], g0[m] = S["D"][j], S["ee_a"][j], S["ee_c"][j], S["g"][j]
        q[:, self._gl[0]] = fing[:, 0]
        q[:, self._gr[0]] = fing[:, 1]
        r.write_joint_position_to_sim_index(position=q, env_ids=env_ids)
        r.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=env_ids)
        self.arm_target[env_ids] = q[:, self._arm]
        self._place("boxA", env_ids, bc, byaw, torch.full((n,), BOX_H / 2, device=dev))
        o = self._obj("puckT_box")
        pose = o.data.default_root_pose.torch[env_ids].clone()
        pose[:, :3] = p_puck + org  # base = 환경 원점 (차체 기본 자세, 요 0) — chain45 와 같은 가정
        pose[:, 2] = p_puck[:, 2] + org[:, 2] + (root[:, 2] - org[:, 2]) * (kind > 0).float()  # 모은 상태는 base 좌표 z
        pose[:, 3:7] = q_puck
        o.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        o.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        park = torch.tensor([0.0, 0.7], device=dev).expand(n, 2)
        self._place("puckT_cyl", env_ids, park + torch.tensor([0.1, 0.0], device=dev), zero, torch.full((n,), 0.031, device=dev))
        nxy_v = nxy.clone()  # 2026-11-10 R0 검토: 옆 퍽 없어도 세울 자리는 '옆 퍽이 있었다면' 규칙으로 (없을 때 자리 쏠림 = 각본 R0b 저하 원인)
        nxy = torch.where(nb_on[:, None], nxy, torch.tensor([0.2, 0.7], device=dev).expand(n, 2))
        self._place("puckN", env_ids, nxy, zero, torch.where(nb_on, torch.full((n,), BOX_H + PUCK_H / 2 + 0.001, device=dev), torch.full((n,), PUCK_H / 2 + 0.001, device=dev)))
        self.nb_present[env_ids] = nb_on
        self.nb_prev[env_ids] = nxy + org[:, :2]
        self.nb_xy0[env_ids] = nxy
        self.nb_w0[env_ids] = nxy + org[:, :2]  # 옆 퍽 월드 시작 xy (밀림 월드 판정용)
        self.box_c[env_ids], self.box_yaw[env_ids] = bc, byaw
        Dn = self._stand_spot(bc, byaw, nxy_v, torch.ones_like(nb_on))
        self.D[env_ids] = torch.where((kind > 0)[:, None], D, Dn)
        self.puck_z0[env_ids] = c.lie_z + org[:, 2]
        # β · ρ: 처음부터 = 시작 자세 (45° 아래, 롤 0), 모은 상태 = 그때 집게 자세
        self.beta[env_ids] = math.pi / 4
        self.rho[env_ids] = 0.0
        m = kind > 0
        if m.any():
            self.set_beta_rho_from(env_ids[m], a0[m], c0[m], g0[m])
        # 단계 기록
        held0 = (kind == 1) | (kind == 2)  # B2: above · at 은 쥐기 전
        self.hold_state[env_ids] = held0
        self.nb_f_prev[env_ids] = -1.0
        self.hold_on[env_ids] = 0
        self.hold_off[env_ids] = 0
        self.ho_on[env_ids] = 0
        self.ho_off[env_ids] = 0
        self.ho_state[env_ids] = False
        for t_ in (self.was_held, self.grasp_seen):
            t_[env_ids] = held0
        self.center_ok[env_ids] = False
        self.lift10[env_ids] = held0
        self.stood[env_ids] = kind == 2
        self.placed[env_ids] = False
        self.released[env_ids] = False
        self.succ_once[env_ids] = False
        self.succeeded[env_ids] = False
        self.hold_ok[env_ids] = 0
        self.phi_prev[env_ids] = 0.0
        self.max_lift[env_ids] = torch.where(held0, torch.full_like(self.max_lift[env_ids], c.lift_full), torch.zeros_like(self.max_lift[env_ids]))  # 쥐고 든 · 세운 시작 = 이미 든 상태
        # 쥔 쪽 끝 · 손 안 축 (모은 상태): 잡는 점 - 퍽 중심 의 긴 축 부호
        e_b = quat_apply(q_puck, torch.tensor([[0.0, 0.0, 1.0]], device=dev).expand(n, 3))
        sg = torch.where(((g0 - p_puck) * e_b).sum(1) >= 0, 1.0, -1.0)
        self.s_g[env_ids] = torch.where(held0, sg, torch.ones_like(sg))
        A = torch.stack((a0, c0, torch.cross(a0, c0, dim=1)), dim=2)  # base 에서 집게 몸 축 (열)
        self.ax_hand0[env_ids] = torch.where(held0[:, None], (A.transpose(1, 2) @ e_b.unsqueeze(-1)).squeeze(-1), torch.zeros_like(e_b))
        # 인식
        self.perc_bias[env_ids] = (torch.rand(n, 3, device=dev) * 2 - 1) * c.perc_bias
        self.box_dh[env_ids] = (torch.rand(n, device=dev) * 2 - 1) * c.box_h_noise
        self.yaw_bias[env_ids] = (torch.rand(n, device=dev) * 2 - 1) * c.perc_yaw_noise
        self.perc_frozen[env_ids] = held0
        pw = torch.cat((p_puck[:, :2], torch.full((n, 1), BOX_H + 0.015, device=dev)), dim=1)
        pw[:, 2] = pw[:, 2] - (root[:, 2] - org[:, 2])
        self.perc_out[env_ids] = pw
        self.fix_p[env_ids] = pw + torch.cat((self.perc_bias[env_ids, :2], self.box_dh[env_ids, None]), dim=1)  # B6: 노드처럼 시작 때 한 번 잰 퍽 (참 + 치우침)
        self.fix_th[env_ids] = torch.atan2(e_b[:, 1], e_b[:, 0]) + self.yaw_bias[env_ids]
        if c.obs_box_noise[0] > 0:  # B7: 노드처럼 잰 상자 → 그 값과 노드 옆 퍽 (있으면 잰 옆 퍽, 없으면 상자 중심 대칭) 로 세울 자리 D
            bn_, yn_ = c.obs_box_noise
            self.box_c_obs[env_ids] = bc + (torch.rand(n, 2, device=dev) * 2 - 1) * bn_
            self.box_yaw_obs[env_ids] = byaw + (torch.rand(n, device=dev) * 2 - 1) * yn_
            nbm_ = nxy_v + (torch.rand(n, 2, device=dev) * 2 - 1) * 0.005
            nbv_ = torch.where(nb_on[:, None], nbm_, 2 * self.box_c_obs[env_ids] - self.fix_p[env_ids, :2])
            Dm_ = self._stand_spot(self.box_c_obs[env_ids], self.box_yaw_obs[env_ids], nbv_, torch.ones_like(nb_on))
            self.D[env_ids] = torch.cat((Dm_[:, :2], self.D[env_ids, 2:3]), dim=1)
        if c.arm_vmax_range:
            lo_, hi_ = c.arm_vmax_range
            self.vmax_ep[env_ids] = lo_ + (hi_ - lo_) * torch.rand(n, device=dev)
        self.actions[env_ids] = 0.0
        self.actions[env_ids, -1] = torch.where(held0, -1.0, 1.0)
        self.prev_actions[env_ids] = self.actions[env_ids]
        self.close_cnt[env_ids] = 0
