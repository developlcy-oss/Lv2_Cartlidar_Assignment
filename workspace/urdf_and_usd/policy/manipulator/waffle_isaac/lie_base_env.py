# [복사본 · 2026-11-11 Onbeom, #192] 원본: 팔 담당 LCY (developlcy-oss) 릴리스 rl-20261110 의 waffle_isaac/sidepick_env.py (md5 c4b52124423c).
# 누운 직육면체 세워 놓기 환경 (lieupright_env.py) 이 상속하는 릴리스 판이 dev 에 없어 새 이름으로 복사함. 원본에서 import 줄만 바꿈 (아래 '# [복사본]' 표시).
# dev 에 릴리스 판이 들어오면 이 파일을 지우고 lieupright_env* 의 import 를 원래 이름으로 되돌림. 기존 파일은 고치지 않음.
"""정렬·집기 `Waffle-SidePick-Direct` 구현. 설정과 설명은 sidepick_env_cfg.py."""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation, RigidObject
from isaaclab.envs import DirectRLEnv
from isaaclab.utils.math import quat_apply, quat_apply_inverse, quat_box_minus, quat_from_euler_xyz, quat_from_matrix, quat_mul, sample_uniform

from .dice2_cfg import BOX_H, BOX_SIZE, GRASP_OFFSET_A5, PUCK_H, PUCK_R, PUCK_Z
from .lie_arm_selfcol import ArmSelfCollision  # [복사본] 원본: from .arm_selfcol import ArmSelfCollision
from .robot_cfg import ARM_JOINTS, EE_BODY, GRIPPER_JOINT, GRIPPER_MIMIC, WHEEL_REGEX, ee_axes  # [복사본] 원본은 URDF_FULL 도 여기서 (dev robot_cfg 에 없음)
from waffle_4wd_isaac.robot_cfg import URDF_PATHS  # noqa: E402  [복사본] 릴리스 robot_cfg 의 URDF_FULL = URDF_PATHS["full"] 과 같은 값
URDF_FULL = URDF_PATHS["full"]  # [복사본]

if TYPE_CHECKING:
    from .lie_base_env_cfg import WaffleSidePickEnvCfg  # [복사본] 원본: from .sidepick_env_cfg import WaffleSidePickEnvCfg


def wrap90(a):
    return a - (math.pi / 2) * torch.round(a / (math.pi / 2))


class WaffleSidePickEnv(DirectRLEnv):
    cfg: WaffleSidePickEnvCfg

    def __init__(self, cfg: WaffleSidePickEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        r = self.robot
        self._arm, _ = r.find_joints(ARM_JOINTS, preserve_order=True)
        self._gl, _ = r.find_joints([GRIPPER_JOINT])
        self._gr, _ = r.find_joints([GRIPPER_MIMIC])
        self._wheels, _ = r.find_joints([WHEEL_REGEX])
        self._a5 = r.find_bodies(EE_BODY)[0][0]
        self._fingers = r.find_bodies(["gripper_left_link", "gripper_right_link", EE_BODY])[0]
        lim = r.data.soft_joint_pos_limits.torch[0, self._arm].clone()
        self._lo, self._hi = lim[:, 0], lim[:, 1]
        if cfg.grip_roll_pi:  # joint5 한계 ±3.3 rad (뒤집은 채 쥐기, 실물 EEPROM ±185° 필요)
            j5 = self._arm[4]
            lim_all = r.data.joint_pos_limits.torch.clone()
            lim_all[:, j5, 0], lim_all[:, j5, 1] = -3.3, 3.3
            r.write_joint_position_limit_to_sim_index(limits=lim_all[:, [j5]], joint_ids=[j5], warn_limit_violation=False)
            self._lo[4], self._hi[4] = -3.3, 3.3
        n, dev = self.num_envs, self.device
        self._off = torch.tensor([GRASP_OFFSET_A5], device=dev).repeat(n, 1)
        self._ex = torch.tensor([[1.0, 0.0, 0.0]], device=dev).repeat(n, 1)
        self._ey = torch.tensor([[0.0, 1.0, 0.0]], device=dev).repeat(n, 1)
        self._ez = torch.tensor([[0.0, 0.0, 1.0]], device=dev).repeat(n, 1)
        self.actions = torch.zeros(n, cfg.action_space, device=dev)  # 관절 공간 6 (팔 5 + 집게) 또는 직교 4 (잡는 점 xyz + 집게)
        self._cols = torch.tensor([getattr(r, "num_base_dofs", 6) + j for j in self._arm], device=dev)
        self.prev_actions = torch.zeros_like(self.actions)
        self.arm_target = r.data.default_joint_pos.torch[:, self._arm].clone()
        self.is_box = torch.zeros(n, dtype=torch.bool, device=dev)
        self.puck_z0 = torch.full((n,), PUCK_Z, device=dev)
        self.perc_bias = torch.zeros(n, 3, device=dev)
        self.nb_prev = torch.zeros(n, 2, device=dev)
        self.root0 = torch.zeros(n, 3, device=dev)
        self.hold = torch.zeros(n, dtype=torch.long, device=dev)
        self.succeeded = torch.zeros(n, dtype=torch.bool, device=dev)
        self.max_lift = torch.zeros(n, device=dev)
        self.arm_box_steps = torch.zeros(n, device=dev)
        self.target_xy = torch.zeros(n, 2, device=dev)
        self.nb_present = torch.ones(n, dtype=torch.bool, device=dev)
        self.puck_xy0 = torch.zeros(n, 2, device=dev)
        self.close_cnt = torch.zeros(n, dtype=torch.long, device=dev)  # 닫는 명령이 이어진 스텝 수 (hold_cart)  # 2026-11-06 집을 퍽 처음 자리 (월드) - 쥐기 전 밀기 벌점
        self.armcol = ArmSelfCollision(r, URDF_FULL, dev)  # 2026-11-06 팔끼리 충돌 상자 겹침 (팀 USD 는 팔끼리 충돌 뺌)  # 2026-11-05 상자당 2 개: 두 번째 퍽 집기 = 옆 퍽 없음 (관측 nb = 0)

    @property
    def robot(self) -> Articulation:
        return self.scene["robot"]

    def _obj(self, k) -> RigidObject:
        return self.scene[k]

    def puck_pos_w(self):
        return torch.where(self.is_box[:, None], self._obj("puckT_box").data.root_pos_w.torch, self._obj("puckT_cyl").data.root_pos_w.torch)

    def puck_quat_w(self):
        return torch.where(self.is_box[:, None], self._obj("puckT_box").data.root_quat_w.torch, self._obj("puckT_cyl").data.root_quat_w.torch)

    def to_base(self, p_w):
        d = self.robot.data
        return quat_apply_inverse(d.root_link_quat_w.torch, p_w - d.root_link_pos_w.torch)

    def vec_base(self, v_w):
        return quat_apply_inverse(self.robot.data.root_link_quat_w.torch, v_w)

    def grasp_w(self):
        d = self.robot.data
        return d.body_link_pos_w.torch[:, self._a5] + quat_apply(d.body_link_quat_w.torch[:, self._a5], self._off)

    def a5_axes_w(self):
        """(닫는 축, 아래쪽, −집게 방향) = 이전 팔 arm_link_5 의 (x, y, z) 축 뜻 (robot_cfg.ee_axes)."""
        return ee_axes(quat_apply, self.robot.data.body_link_quat_w.torch[:, self._a5], self._ex, self._ey, self._ez)

    def _perc_filter(self, raw, key="p"):
        """2026-11-10 #191 인식 결과의 지연 · 갱신 주기 · 끊김 (실물 팔 카메라). 설정이 끔이면 raw 그대로. key = 따로 쌓을 버퍼 이름."""
        c, n, dev = self.cfg, self.num_envs, self.device
        if c.perc_delay_steps[1] == 0 and c.perc_period_steps[1] <= 1 and c.perc_drop_p[1] == 0.0:
            return raw
        H = c.perc_delay_steps[1] + 1
        st = getattr(self, "_pcf", None)
        if st is None:
            st = self._pcf = {}
        if key not in st:
            st[key] = {"hist": raw[:, None].repeat(1, H, 1).clone(), "out": raw.clone(), "t": 0, "hold": torch.zeros(n, dtype=torch.long, device=dev),
                       "d": torch.randint(c.perc_delay_steps[0], c.perc_delay_steps[1] + 1, (n,), device=dev),
                       "per": torch.randint(c.perc_period_steps[0], c.perc_period_steps[1] + 1, (n,), device=dev)}
        b = st[key]
        new = self.episode_length_buf == 0
        if new.any():
            b["hist"][new] = raw[new, None]
            b["out"][new] = raw[new]
            b["hold"][new] = 0
            k = int(new.sum())
            b["d"][new] = torch.randint(c.perc_delay_steps[0], c.perc_delay_steps[1] + 1, (k,), device=dev)
            b["per"][new] = torch.randint(c.perc_period_steps[0], c.perc_period_steps[1] + 1, (k,), device=dev)
        b["t"] += 1
        slot = b["t"] % H
        b["hist"][:, slot] = raw
        cand = b["hist"][torch.arange(n, device=dev), (slot - b["d"]) % H]
        tick = (self.episode_length_buf % b["per"]) == 0
        v = self.robot.data.joint_vel.torch[:, self._arm[0]].abs()
        p = c.perc_drop_p[0] + (c.perc_drop_p[1] - c.perc_drop_p[0]) * torch.clamp(v / 0.5, 0.0, 1.0)
        drop = torch.rand(n, device=dev) < p
        upd = (tick & ~drop) | (b["hold"] >= b["per"] + c.perc_drop_max) | new
        b["out"] = torch.where(upd[:, None], cand, b["out"])
        b["hold"] = torch.where(upd, torch.zeros_like(b["hold"]), b["hold"] + 1)
        return b["out"]

    def perc_point_w(self):
        """정책 관측의 '인식 퍽 좌표' (기본 = 퍽 중심). LieEnd 는 잡을 점 − grasp_dz 를 줌 (U4 와 같은 뜻: 잡는 점 = 이 점 + grasp_dz)."""
        return self.puck_pos_w()

    def grasp_target_w(self):
        p = self.puck_pos_w().clone()
        p[:, 2] += self.cfg.grasp_dz
        return p

    def face_delta(self):
        px = quat_apply(self.puck_quat_w(), self._ex)
        ax, _, _ = self.a5_axes_w()
        return wrap90(torch.atan2(px[:, 1], px[:, 0]) - torch.atan2(ax[:, 1], ax[:, 0]))

    def lift(self):
        return self.puck_pos_w()[:, 2] - self.puck_z0

    def base_tilt(self):
        g = self.robot.data.projected_gravity_b.torch
        return torch.acos((-g[:, 2]).clamp(-1.0, 1.0))

    def puck_up(self):
        return quat_apply(self.puck_quat_w(), self._ez)[:, 2].abs()

    def _contact(self, k):
        f = self.scene[k].data.normal_force_matrix_w.torch  # (N, 몸, 상대, 3)
        return torch.linalg.norm(f, dim=-1).amax(dim=(1, 2)) > self.cfg.contact_force

    # ---- 행동 ----
    def _pre_physics_step(self, actions):
        self.prev_actions = self.actions.clone()
        self.actions = actions.clamp(-1.0, 1.0)
        q = self.robot.data.joint_pos.torch[:, self._arm]
        lead = self.cfg.target_lead_max
        if self.cfg.cartesian:
            dq = self._ik_step(self.hold_cart(self.actions) * self.cfg.cart_scale)
            t = torch.clamp(self.arm_target + dq, q - lead, q + lead)
        else:
            t = torch.clamp(self.arm_target + self.cfg.action_scale * self.actions[:, :5], q - lead, q + lead)
        self.arm_target = torch.clamp(t, self._lo, self._hi)
        if self.cfg.pick_j3_max is not None:  # SP-45c: 관절 3 상한 (팔을 접어 몸체에 부딪히지 않게)
            self.arm_target[:, 2] = torch.clamp(self.arm_target[:, 2], max=self.cfg.pick_j3_max)

    def hold_cart(self, actions, mask=None):
        """2026-11-06 '멈춘 뒤 닫기' 규칙: 닫는 명령이 이어진 스텝 수를 세고, 처음 close_hold_steps 동안 (퍽이 들리기 전) 잡는 점 이동 0. mask 밖 대는 셈을 0 으로."""
        closing = actions[:, -1] < 0
        if mask is not None:
            closing = closing & mask
        self.close_cnt = torch.where(closing, self.close_cnt + 1, torch.zeros_like(self.close_cnt))
        cart = actions[:, :3]
        if self.cfg.close_hold_steps > 0:
            hold = closing & (self.close_cnt <= self.cfg.close_hold_steps) & (self.lift() < self.cfg.lift_min)
            cart = torch.where(hold[:, None], torch.zeros_like(cart), cart)
        return cart

    def _ik_step(self, dp_b):
        """직교 행동 (261028 N7): 잡는 점 변위 (base 좌표) + 집게 자세 = 수평·joint1 축에서 바깥 방향·카메라 위 -> 감쇠 최소제곱 IK (넣기 전용 환경과 같은 방식)."""
        c, d = self.cfg, self.robot.data
        rq = d.root_link_quat_w.torch
        J = d.body_link_jacobian_w.torch[:, self._a5][:, :, self._cols]
        q5 = d.body_link_quat_w.torch[:, self._a5]
        rr = quat_apply(q5, self._off)
        Jl = J[:, 0:3] - torch.cross(rr.unsqueeze(-1).expand(-1, -1, 5), J[:, 3:6], dim=1)
        w = c.rot_hold_weight
        Jf = torch.cat((Jl, J[:, 3:6] * w), dim=1)
        j1 = d.root_link_pos_w.torch + quat_apply(rq, torch.tensor([[c.joint1_x, 0.0, 0.0]], device=self.device).expand(self.num_envs, 3))
        up = quat_apply(rq, self._ez)
        a = self.grasp_w() - j1
        a = a - (a * up).sum(1, keepdim=True) * up
        a = a / a.norm(dim=1, keepdim=True).clamp(min=1e-6)
        if c.grasp_pitch != 0.0:  # 2026-11-03: 집게를 바깥 방향에서 아래로 grasp_pitch 기울임 (손가락 닫는 축 y 는 수평 그대로)
            cp, sp = math.cos(c.grasp_pitch), math.sin(c.grasp_pitch)
            a, up = cp * a - sp * up, sp * a + cp * up
        R = torch.stack((a, torch.cross(up, a, dim=1), up), dim=2)
        if c.grip_roll_pi:  # 뒤집은 집게: 집게 방향 그대로, 닫는 축 · 카메라 축 반대
            R = R * torch.tensor([1.0, -1.0, -1.0], device=self.device).view(1, 1, 3)
        err = torch.cat((quat_apply(rq, dp_b), quat_box_minus(quat_from_matrix(R), q5) * w), dim=1)
        JJt = Jf @ Jf.transpose(1, 2) + c.ik_damping * torch.eye(6, device=self.device)
        return (Jf.transpose(1, 2) @ torch.linalg.solve(JJt, err.unsqueeze(-1))).squeeze(-1)

    def _apply_action(self):
        cmd = self.robot.actuators.target_command
        g = torch.where(self.actions[:, -1:] > 0.0, self.cfg.gripper_open, self.cfg.gripper_closed)
        cmd.set_position_index(value=self.arm_target.contiguous(), joint_ids=self._arm)
        cmd.set_position_index(value=g.expand(-1, 2).contiguous(), joint_ids=self._gl + self._gr)  # 두 손가락 같은 값
        cmd.set_velocity_index(value=torch.zeros(self.num_envs, len(self._wheels), device=self.device), joint_ids=self._wheels)

    # ---- 관측·보상·종료 ----
    def _get_observations(self):
        d = self.robot.data
        perc = self._perc_filter(self.perc_point_w() + self.perc_bias + (torch.rand(self.num_envs, 3, device=self.device) * 2 - 1) * self.cfg.perc_jitter)
        pb = self.to_base(perc)
        gb = self.to_base(self.grasp_w())
        ax, _, az = self.a5_axes_w()
        nb = self.to_base(self._obj("puckN").data.root_pos_w.torch)[:, :2] - pb[:, :2]
        nb = torch.where(self.nb_present[:, None], nb, torch.zeros_like(nb))
        obs = torch.cat((
            d.joint_pos.torch[:, self._arm] - d.default_joint_pos.torch[:, self._arm], d.joint_vel.torch[:, self._arm],
            d.joint_pos.torch[:, self._gl], pb, gb, pb - gb, self.vec_base(-az), self.vec_base(ax), nb,
            self.is_box.float().unsqueeze(1), self.actions,
        ), dim=-1)
        return {"policy": obs}

    def _get_rewards(self):
        c = self.cfg
        g = self.grasp_w()
        to_t = self.grasp_target_w() - g
        dist = torch.linalg.norm(to_t, dim=1)
        lift = self.lift()
        near = dist < c.near_dist
        close = (dist < 0.12).float()
        ax, ay, az = self.a5_axes_w()
        appr = -az
        sp_, cp_ = math.sin(c.grasp_pitch), math.cos(c.grasp_pitch)
        horiz = 1 - (appr[:, 2] + sp_).abs()  # 2026-11-03: 목표 피치 (0 이면 예전 '수평' 과 같음)
        if c.grip_roll_pi:  # 뒤집은 집게: 카메라 축이 아래가 '바른' 자세
            ay = -ay
        cam_up = (1 - ((-ay[:, 2]) - cp_).abs()).clamp(min=0.0) if c.grasp_pitch != 0.0 else (-ay[:, 2]).clamp(min=0.0)
        dir_h = to_t.clone()
        dir_h[:, 2] = 0
        dir_h = dir_h / dir_h.norm(dim=1, keepdim=True).clamp(min=1e-6)
        point = (appr * dir_h).sum(dim=1).clamp(min=0.0) * ((dist > 0.02) & (dist < 0.12)).float()
        align = self.is_box.float() * (1 + torch.cos(4 * self.face_delta())) / 2 * close
        up = self.puck_up()
        tilt = self.base_tilt()
        fing = self.robot.data.joint_pos.torch[:, self._gl[0]]
        dz_ok = (g[:, 2] - self.puck_pos_w()[:, 2] - c.grasp_dz).abs() < c.grasp_dz_tol  # 2026-11-07 잡는 높이 조건 (퍽 윗 모서리 쥐기 막기)
        grasped = (dist < c.grasp_dist) & (fing > c.grasp_finger_range[0]) & (fing < c.grasp_finger_range[1]) & (tilt < c.grasp_tilt_max) & (up > 0.9) & dz_ok
        lifted = (lift > c.lift_min) & near & (up > c.upright_min) & dz_ok
        height = (torch.clamp(lift, 0.0, c.lift_goal) - torch.clamp(lift - c.lift_goal, min=0.0)) / c.lift_goal * near.float() * dz_ok.float()
        xy = self._obj("puckN").data.root_pos_w.torch[:, :2]
        moved = torch.linalg.norm(xy - self.nb_prev, dim=1)
        self.nb_prev = xy.clone()
        toppled = (up < c.topple_cos) & (lift < c.lift_min)
        nb_touch = self._contact("arm_nb_contact")
        nb_topple = self.nb_toppled()
        arm_box = self._contact("arm_box_contact")
        body_box = self._contact("body_box_contact")
        base_move = torch.linalg.norm(self.robot.data.root_link_pos_w.torch[:, :2] - self.root0[:, :2], dim=1)
        extra = torch.zeros(self.num_envs, device=self.device)
        if c.rew_self_end != 0.0:  # 2026-11-04: 자기 충돌 · 관절 3 과도 = 강한 벌점 + 종료, 80° 넘으면 굽힌 만큼 벌점
            sb = self.self_bad()
            j3 = self.robot.data.joint_pos.torch[:, self._arm[2]].abs()
            extra += c.rew_self_end * sb.float() + c.rew_j3_bend * torch.clamp((j3 - c.j3_bend_start) / 0.1, 0.0, 3.0)
            lg = self.extras.setdefault("log", {})
            lg["Metrics/self_bad_ratio"] = sb.float().mean().item()
            lg["Metrics/j3_abs_max"] = j3.max().item()
        if c.rew_arm_body_end != 0.0:  # SP-45b
            arm_body = self.arm_body_hit()
            extra += c.rew_arm_body_end * arm_body.float()
            self.extras.setdefault("log", {})["Metrics/arm_body_ratio"] = arm_body.float().mean().item()
        if c.rew_held_tilt != 0.0 or c.rew_grasp_dz != 0.0:
            carried = (lift > c.lift_min) & near
            ang = torch.acos(up.clamp(max=1.0))
            gdz = (g[:, 2] - self.puck_pos_w()[:, 2] - c.grasp_dz).abs()
            extra += carried.float() * c.rew_held_tilt * torch.clamp(ang / c.held_tilt_ref, max=3.0)
            extra += grasped.float() * c.rew_grasp_dz * torch.clamp(gdz / 0.005, max=3.0)
            lg = self.extras.setdefault("log", {})
            if carried.any():
                lg["Metrics/held_tilt_deg"] = torch.rad2deg(ang[carried]).mean().item()
            if grasped.any():
                lg["Metrics/grasp_dz_err_mm"] = (gdz[grasped] * 1000).mean().item()
        if c.rew_pre_push != 0.0 or c.rew_close_still != 0.0:  # 2026-11-06 쥐기 전 밀기 · 닫으며 밀고 들어가기
            pre = lift < c.lift_min
            push = torch.linalg.norm(self.puck_pos_w()[:, :2] - self.puck_xy0, dim=1)
            extra += pre.float() * c.rew_pre_push * torch.clamp(push / 0.005, max=3.0)
            closing = pre & (self.actions[:, -1] < 0) & (dist < 0.03)
            gv = torch.linalg.norm(self.robot.data.body_link_lin_vel_w.torch[:, self._a5], dim=1)
            extra += closing.float() * c.rew_close_still * torch.clamp(gv / 0.02, max=3.0)
            lg = self.extras.setdefault("log", {})
            lg["Metrics/pre_push_mm"] = (push[pre] * 1000).mean().item() if pre.any() else 0.0
            if closing.any():
                lg["Metrics/close_speed_cms"] = (gv[closing] * 100).mean().item()
        fit_ok = torch.ones_like(near)
        if c.rew_grasp_fit != 0.0 or c.grasp_fit_tol < 1.0:  # 2026-11-06: 놓기 가정 (퍽 중심 = 잡는 점 − grasp_dz 수직) 과 실제 손 안 퍽 중심 차, 손가락 닫는 축 성분 뺌
            carried_f = (lift > c.lift_min) & near
            ef = self.puck_pos_w() - (g - self._ez * c.grasp_dz)
            ef = ef - (ef * ax).sum(1, keepdim=True) * ax
            fit = ef.norm(dim=1)
            fit_ok = fit < c.grasp_fit_tol
            extra += carried_f.float() * c.rew_grasp_fit * torch.clamp(fit / c.grasp_fit_ref, max=3.0)
            extra += (carried_f & fit_ok).float() * c.rew_fit_bonus  # 2026-11-09 칸에 앉을 만큼 맞게 쥔 채 든 스텝
            if carried_f.any():
                self.extras.setdefault("log", {})["Metrics/grasp_fit_mm"] = (fit[carried_f] * 1000).mean().item()
        rew = extra + (
            c.rew_reach * (1 - torch.tanh(dist / 0.1)) + c.rew_reach_fine * (1 - torch.tanh(dist / 0.02))
            + close * (c.rew_horizontal * horiz + c.rew_camera_up * cam_up) + c.rew_point * point + c.rew_align * align
            + c.rew_grasp * grasped.float() + c.rew_lifted * lifted.float() + c.rew_lift_height * height
            + c.rew_puck_tilt * (1 - up) * close
            + c.rew_arm_box * arm_box.float() + c.rew_body_box * body_box.float() + c.rew_base_move * (base_move > c.base_move_max).float()
            + c.rew_action_rate * torch.sum(torch.square(self.actions - self.prev_actions), dim=1)
            + c.rew_joint_vel * torch.sum(torch.square(self.robot.data.joint_vel.torch[:, self._arm]), dim=1)
            + c.rew_nb_touch * nb_touch.float() + c.rew_nb_topple * nb_topple.float()
            + c.rew_nb_touch_end * (nb_touch & torch.tensor(c.nb_touch_terminate, device=self.device)).float()
            + c.rew_nb_near * torch.clamp(1 - self.nb_near() / c.nb_near_m, 0.0, 1.0)
            + c.rew_topple_end * (toppled & torch.tensor(c.topple_terminate, device=self.device)).float()
            + c.rew_neighbor_moved * moved + c.rew_toppled * toppled.float() + c.rew_base_tilt * tilt + c.rew_fall * self._fell().float()
        )
        held = (lift > c.success_lift) & near & (up > c.upright_min) & dz_ok & fit_ok  # fit_ok: grasp_fit_tol 1.0 이면 늘 참
        self.hold = torch.where(held, self.hold + 1, torch.zeros_like(self.hold))
        self.succeeded |= self.hold >= int(c.success_hold_s / (c.sim.dt * c.decimation))
        self.max_lift = torch.maximum(self.max_lift, lift * near.float())
        self.arm_box_steps += arm_box.float()
        log = self.extras.setdefault("log", {})
        log["Metrics/grasp_distance"] = dist.mean().item()
        log["Metrics/horizontal"] = horiz.mean().item()
        log["Metrics/grasped_ratio"] = grasped.float().mean().item()
        log["Metrics/lifted_ratio"] = lifted.float().mean().item()
        log["Metrics/toppled_ratio"] = toppled.float().mean().item()
        log["Metrics/nb_touch_ratio"] = nb_touch.float().mean().item()
        log["Metrics/nb_topple_ratio"] = nb_topple.float().mean().item()
        log["Metrics/arm_box_contact_ratio"] = arm_box.float().mean().item()
        log["Metrics/body_box_contact_ratio"] = body_box.float().mean().item()
        return rew

    def self_dist(self):
        """집게 (손목 롤 링크 + 4 cm ~ 잡는 점 + 2 cm) · 쥔 퍽 (잡는 점 3 cm 안이면) 과 윗팔 (link3 -> link4 원점) · 아랫팔 (link4 -> link5 원점) 선분의 최소 거리 [m]."""
        d = self.robot.data
        P = d.body_link_pos_w.torch
        if not hasattr(self, "_seg_ids"):
            self._seg_ids = [self.robot.find_bodies(nm)[0][0] for nm in ("link3", "link4", "link5")]
        sh, el, wr = (P[:, i] for i in self._seg_ids)
        qa = d.body_link_quat_w.torch[:, self._a5]
        pa = P[:, self._a5]
        g0 = pa + quat_apply(qa, self._ex * 0.04)
        g1 = pa + quat_apply(qa, self._ex * 0.16)

        def seg(a, b, c, e):
            u_, v_, w_ = b - a, e - c, a - c
            A, B, C = (u_ * u_).sum(1), (u_ * v_).sum(1), (v_ * v_).sum(1)
            D, E = (u_ * w_).sum(1), (v_ * w_).sum(1)
            den = (A * C - B * B).clamp(min=1e-12)
            s_ = ((B * E - C * D) / den).clamp(0, 1)
            t_ = ((B * s_ + E) / C.clamp(min=1e-12)).clamp(0, 1)
            s_ = ((B * t_ - D) / A.clamp(min=1e-12)).clamp(0, 1)
            return torch.linalg.norm(a + s_[:, None] * u_ - (c + t_[:, None] * v_), dim=1)
        dist = torch.minimum(seg(g0, g1, sh, el), seg(g0, g1, el, wr))
        pc = self.puck_pos_w()
        ax = quat_apply(self.puck_quat_w(), self._ez) * (PUCK_H / 2)
        held = torch.linalg.norm(pc - self.grasp_w(), dim=1) < 0.03
        dp = torch.minimum(seg(pc - ax, pc + ax, sh, el), seg(pc - ax, pc + ax, el, wr))
        return torch.where(held, torch.minimum(dist, dp), dist)

    def self_bad(self):
        """자기 충돌 근접 또는 관절 3 과도 (j3_bend_end 넘음)."""
        c = self.cfg
        j3 = self.robot.data.joint_pos.torch[:, self._arm[2]].abs()
        return (self.self_dist() < c.self_margin) | (j3 > c.j3_bend_end) | (self.armcol.depth() > c.self_box_tol)

    def arm_body_hit(self):
        """SP-45b: 팔·손가락이 로봇 몸체에 contact_force 넘게 닿음 (scene 에 arm_body_contact 가 있을 때만)."""
        if "arm_body_contact" not in self.scene.keys():
            return torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        return self._contact("arm_body_contact")

    def nb_near(self):
        """손가락 (왼·오른 손가락 몸, 집게 몸) 원점과 옆 퍽 옆면 사이 수평 거리 최소 [m] (옆 퍽 높이 범위 안만, 밖이면 큰 값). SP-U3."""
        d = self.robot.data.body_link_pos_w.torch[:, self._fingers]  # (N, 3, 3)
        nb = self._obj("puckN").data.root_pos_w.torch
        dh = torch.linalg.norm(d[..., :2] - nb[:, None, :2], dim=-1) - 0.015
        inz = (d[..., 2] - nb[:, None, 2]).abs() < 0.045
        return torch.where(inz, dh, torch.full_like(dh, 1.0)).amin(dim=1)

    def target_toppled(self):
        return (self.puck_up() < self.cfg.topple_cos) & (self.lift() < self.cfg.lift_min)

    def nb_toppled(self):
        """옆 퍽 (목표 아님) 이 쓰러짐."""
        return quat_apply(self._obj("puckN").data.root_quat_w.torch, self._ez)[:, 2].abs() < self.cfg.topple_cos

    def _fell(self):
        return (self.puck_pos_w()[:, 2] < self.puck_z0 - self.cfg.fall_drop) & (self.lift() < self.cfg.lift_min)

    def _get_dones(self):
        tipped = self.robot.data.projected_gravity_b.torch[:, 2] > -0.6
        body_box = self._contact("body_box_contact")
        base_move = torch.linalg.norm(self.robot.data.root_link_pos_w.torch[:, :2] - self.root0[:, :2], dim=1) > self.cfg.base_move_max
        end = tipped | self._fell() | body_box | base_move | self.nb_toppled()
        self.term_arm_body = self.arm_body_hit() if self.cfg.rew_arm_body_end != 0.0 else torch.zeros_like(end)
        end = end | self.term_arm_body
        self.term_self = self.self_bad() if self.cfg.rew_self_end != 0.0 else torch.zeros_like(end)
        end = end | self.term_self
        self.term_nb = self._contact("arm_nb_contact") | self.nb_toppled()  # 진단용 (마지막 판정)
        self.term_topple = self.target_toppled()
        self.term_fell = self._fell()
        if self.cfg.nb_touch_terminate:  # SP-U3
            end = end | self.term_nb
        if self.cfg.topple_terminate:
            end = end | self.term_topple
        return end, self.episode_length_buf >= self.max_episode_length - 1

    # ---- 초기화 ----
    def _place(self, k, env_ids, xy, yaw, z):
        o = self._obj(k)
        pose = o.data.default_root_pose.torch[env_ids].clone()
        pose[:, :2] = xy + self.scene.env_origins[env_ids, :2]
        pose[:, 2] = z + self.scene.env_origins[env_ids, 2]
        zero = torch.zeros_like(yaw)
        pose[:, 3:7] = quat_from_euler_xyz(zero, zero, yaw)
        o.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        o.write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(env_ids), 6, device=self.device), env_ids=env_ids)

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        if len(env_ids) > 0:
            log = self.extras.setdefault("log", {})
            ok, bx = self.succeeded[env_ids].float(), self.is_box[env_ids]
            log["Metrics/success_rate"] = ok.mean().item()
            if bx.any():
                log["Metrics/success_box"] = ok[bx].mean().item()
            if (~bx).any():
                log["Metrics/success_cyl"] = ok[~bx].mean().item()
            log["Metrics/max_lift"] = self.max_lift[env_ids].mean().item()
            log["Metrics/arm_box_steps"] = self.arm_box_steps[env_ids].mean().item()
        super()._reset_idx(env_ids)
        c, r, n, dev = self.cfg, self.robot, len(env_ids), self.device
        org = self.scene.env_origins[env_ids]

        def U(lo, hi, shape):
            return sample_uniform(lo, hi, shape, dev)

        root = r.data.default_root_pose.torch[env_ids].clone()
        root[:, :3] += org
        r.write_root_pose_to_sim_index(root_pose=root, env_ids=env_ids)
        r.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        self.root0[env_ids] = root[:, :3]
        q = r.data.default_joint_pos.torch[env_ids].clone()
        near = (torch.rand(n, device=dev) < c.near_start_prob) & torch.tensor(not c.dock_layout, device=dev)  # 역방향 커리큘럼: 퍽을 벌린 손가락 사이 (시작 자세 앞 0 ~ 3.5 cm) 에 둠
        nz = torch.where(near, c.near_noise, c.ready_noise).unsqueeze(1)
        q[:, self._arm] = torch.clamp(q[:, self._arm] + U(-1.0, 1.0, (n, 5)) * nz, self._lo, self._hi)
        r.write_joint_position_to_sim_index(position=q, env_ids=env_ids)
        r.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=env_ids)
        self.arm_target[env_ids] = q[:, self._arm]

        # 상자: 로봇 앞 끝에서 gap, 가로 이동, 요 (가장 가까운 면·모서리 기준으로 gap)
        nb_on = torch.ones(n, dtype=torch.bool, device=dev)
        pyaw = None
        if c.dock_layout:  # 2026-11-05 주행 정지 기준: 상자 앞면이 로봇 앞면 (TFmini) 에서 0.17 ± 0.02 m, 요 ±5°, 두 퍽 간격 8 cm (주로 옆으로 나란히)
            half = BOX_SIZE / 2
            yaw = U(-c.dock_yaw, c.dock_yaw, (n,))
            pyaw = yaw.clone()
            bc = torch.stack((c.robot_front + U(*c.dock_gap, (n,)) + half, U(*c.dock_lat, (n,))), dim=1)
            side = torch.rand(n, device=dev) < c.dock_side_prob
            pa = yaw + side.float() * (math.pi / 2) + U(-c.dock_pair_noise, c.dock_pair_noise, (n,))  # 두 퍽 줄 방향
            pm = bc + U(-c.pair_jitter, c.pair_jitter, (n, 2))
            off = torch.stack((torch.cos(pa), torch.sin(pa)), dim=1) * (c.pair_spacing / 2)
            p1, p2 = pm - off, pm + off
            j1 = torch.tensor([c.joint1_x, 0.0], device=dev)
            first1 = (torch.linalg.norm(p1 - j1, dim=1) <= torch.linalg.norm(p2 - j1, dim=1))[:, None]
            pn, pf = torch.where(first1, p1, p2), torch.where(first1, p2, p1)  # 가까운 퍽 (첫째), 먼 퍽 (둘째)
            second = torch.rand(n, device=dev) < c.dock_second_prob  # 두 번째 퍽 집기: 첫 퍽은 이미 실음 -> 옆 퍽 없음
            tgt = torch.where(second[:, None], pf, pn)
            nbr = pf
            nb_on = ~second
            if c.dock_rand_prob > 0:  # 2026-11-06 두 퍽 자리 무작위 (상자 좌표에서 뽑아 닿는 곳만, 후보 64 개 중 처음 맞는 것)
                K = 64
                lim = BOX_SIZE / 2 - c.dock_rand_edge
                cy, sy = torch.cos(yaw)[:, None], torch.sin(yaw)[:, None]
                def cand():
                    uv = U(-lim, lim, (n, K, 2))
                    w = torch.stack((bc[:, None, 0] + cy * uv[..., 0] - sy * uv[..., 1], bc[:, None, 1] + sy * uv[..., 0] + cy * uv[..., 1]), dim=-1)
                    r_ = torch.linalg.norm(w - j1, dim=-1)
                    return w, (r_ >= c.dock_r_min) & (r_ <= c.dock_r_max)
                wa, oka = cand()
                wb, okb = cand()
                okp = oka & okb & (torch.linalg.norm(wa - wb, dim=-1) >= c.dock_rand_gap)
                has = okp.any(dim=1) & (torch.rand(n, device=dev) < c.dock_rand_prob)
                k_ = okp.float().argmax(dim=1)
                ar = torch.arange(n, device=dev)
                qa_, qb_ = wa[ar, k_], wb[ar, k_]
                fa_ = (torch.linalg.norm(qa_ - j1, dim=1) <= torch.linalg.norm(qb_ - j1, dim=1))[:, None]
                rn, rf = torch.where(fa_, qa_, qb_), torch.where(fa_, qb_, qa_)
                tgt = torch.where(has[:, None], torch.where(second[:, None], rf, rn), tgt)
                nbr = torch.where(has[:, None], rf, nbr)
                pyaw = torch.where(has, U(0.0, math.pi / 2, (n,)), pyaw)  # 직육면체 면 방향도 무작위
            rt = torch.linalg.norm(tgt - j1, dim=1)  # 닿는 거리 밖이면 차체를 앞 · 뒤로 조금 움직인 것과 같게 (상자째 x 로 옮김)
            sh = torch.clamp(rt - c.dock_r_max, min=0.0) - torch.clamp(c.dock_r_min - rt, min=0.0)
            shv = torch.stack((sh, torch.zeros_like(sh)), dim=1)
            bc, tgt, nbr = bc - shv, tgt - shv, nbr - shv
            self._place("boxA", env_ids, bc, yaw, torch.full((n,), BOX_H / 2, device=dev))
        else:
            yaw = U(0.0, math.pi, (n,))
            half = BOX_SIZE / 2
            half_proj = half * (torch.cos(yaw).abs() + torch.sin(yaw).abs())
            bc = torch.stack((c.robot_front + U(*c.gap_range, (n,)) + half_proj, U(*c.lat_range, (n,))), dim=1)
            self._place("boxA", env_ids, bc, yaw, torch.full((n,), BOX_H / 2, device=dev))
            off = torch.stack((torch.cos(yaw), torch.sin(yaw)), dim=1) * 0.04
            p1, p2 = bc - off, bc + off
            j1 = torch.tensor([c.joint1_x, 0.0], device=dev)
            d1 = (torch.linalg.norm(p1 - j1, dim=1) - c.sweet_radial).abs()
            d2 = (torch.linalg.norm(p2 - j1, dim=1) - c.sweet_radial).abs()
            pick1 = (d1 <= d2)[:, None]
            if c.front_first:  # SP-U4: 로봇 쪽 퍽 (joint1 축에 가까운 퍽) 을 집음, 너무 가까우면 상자째 바깥으로
                r1, r2 = torch.linalg.norm(p1 - j1, dim=1), torch.linalg.norm(p2 - j1, dim=1)
                pick1 = (r1 <= r2)[:, None]
                tgt0 = torch.where(pick1, p1, p2)
                rt = torch.linalg.norm(tgt0 - j1, dim=1)
                want_r = U(*c.front_r_range, (n,))
                push = torch.clamp(want_r - rt, min=0.0)
                radial = (tgt0 - j1) / rt.clamp(min=1e-6).unsqueeze(1)
                bc = bc + radial * push.unsqueeze(1)
                self._place("boxA", env_ids, bc, yaw, torch.full((n,), BOX_H / 2, device=dev))
                p1, p2 = bc - off, bc + off
            if near.any():  # 집을 퍽 = 시작 자세의 잡는 점 바로 앞, 상자는 그 아래로 (p1 = 집을 퍽)
                want = torch.stack((c.near_grasp_x + U(0.0, c.near_dx_max, (n,)), U(-c.near_dy_max, c.near_dy_max, (n,))), dim=1)
                if c.front_first:  # 옆 퍽 (want + 2 off) 이 바깥 쪽이 되게 off 방향을 고름
                    rad_w = (want - j1) / torch.linalg.norm(want - j1, dim=1, keepdim=True).clamp(min=1e-6)
                    off = torch.where(((off * rad_w).sum(1, keepdim=True) < 0) & near[:, None], -off, off)
                bc = torch.where(near[:, None], want + off, bc)
                self._place("boxA", env_ids, bc, yaw, torch.full((n,), BOX_H / 2, device=dev))
                p1, p2 = bc - off, bc + off
                pick1 = pick1 | near[:, None]
            tgt, nbr = torch.where(pick1, p1, p2), torch.where(pick1, p2, p1)
        if pyaw is None:
            pyaw = yaw
        is_box = torch.rand(n, device=dev) < c.box_shape_prob
        self.target_xy[env_ids] = tgt  # 집을 퍽 자리 (환경 원점 기준, LieEnd 가 씀)
        self.is_box[env_ids] = is_box
        park = torch.tensor([0.0, 0.7], device=dev).expand(n, 2)
        z = torch.full((n,), PUCK_Z, device=dev)
        self._place("puckT_box", env_ids, torch.where(is_box[:, None], tgt, park), pyaw, z)
        self._place("puckT_cyl", env_ids, torch.where(is_box[:, None], park + torch.tensor([0.1, 0.0], device=dev), tgt), pyaw, z)
        self.puck_xy0[env_ids] = tgt + org[:, :2]
        self.close_cnt[env_ids] = 0
        nbr = torch.where(nb_on[:, None], nbr, torch.tensor([0.2, 0.7], device=dev).expand(n, 2))  # 옆 퍽 없음 = 치워 둠
        self.nb_present[env_ids] = nb_on
        self._place("puckN", env_ids, nbr, pyaw, z)
        self.nb_prev[env_ids] = nbr + org[:, :2]
        self.puck_z0[env_ids] = PUCK_Z + org[:, 2]
        self.perc_bias[env_ids] = U(-c.perc_bias, c.perc_bias, (n, 3))
        self.actions[env_ids] = 0.0
        self.actions[env_ids, -1] = 1.0
        self.prev_actions[env_ids] = self.actions[env_ids]
        self.hold[env_ids] = 0
        self.succeeded[env_ids] = False
        self.max_lift[env_ids] = 0.0
        self.arm_box_steps[env_ids] = 0.0


class WaffleLieEndEnv(WaffleSidePickEnv):
    """누운 직육면체 해석 2 (2026-11-04): 반지름 방향으로 누운 퍽의 안쪽 끝을 수평 · 바깥 집게로 쥐고 들기.
    잡는 점 = 퍽 안쪽 끝 + end_dx (긴 축 따라 바깥으로), 높이 = 퍽 윗면 + (손가락 밑 29 mm 가 상자 위 2 mm). '세움' 판정은 '누운 채' (puck_up = 긴 축이 수평)."""

    def _long_axis_w(self):
        return quat_apply(self.puck_quat_w(), self._ez)

    def puck_up(self):
        az = self._long_axis_w()[:, 2]
        return torch.sqrt((1.0 - az * az).clamp(min=0.0))

    def grasp_target_w(self):
        c = self.cfg
        pc = self.puck_pos_w()
        L = self._long_axis_w()
        L = L - L[:, 2:3] * self._ez
        L = L / L.norm(dim=1, keepdim=True).clamp(min=1e-6)
        d = self.robot.data
        j1 = d.root_link_pos_w.torch + quat_apply(d.root_link_quat_w.torch, torch.tensor([[c.joint1_x, 0.0, 0.0]], device=self.device).expand(self.num_envs, 3))
        L = L * torch.where(((pc - j1) * L).sum(1, keepdim=True) >= 0, 1.0, -1.0)  # 바깥 방향
        t = pc - L * (PUCK_H / 2 - c.end_dx)
        t[:, 2] = pc[:, 2] + c.grasp_dz
        return t

    def perc_point_w(self):
        t = self.grasp_target_w()
        t[:, 2] -= self.cfg.grasp_dz
        return t

    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES
        c, n, dev = self.cfg, len(env_ids), self.device
        o = self._obj("puckT_box")
        org = self.scene.env_origins[env_ids]
        xy = self.target_xy[env_ids]  # 부모 리셋이 정한 집을 퍽 자리
        a = torch.atan2(xy[:, 1], xy[:, 0] - c.joint1_x) + (torch.rand(n, device=dev) * 2 - 1) * c.end_yaw_noise
        zero = torch.zeros(n, device=dev)
        q = quat_mul(quat_from_euler_xyz(zero, zero, a), quat_from_euler_xyz(zero, torch.full((n,), math.pi / 2, device=dev), zero))  # 몸 z (긴 축) -> 반지름 방향
        pose = o.data.default_root_pose.torch[env_ids].clone()
        pose[:, :2] = xy + org[:, :2]
        pose[:, 2] = BOX_H + PUCK_R + 0.001 + org[:, 2]
        pose[:, 3:7] = q
        o.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        o.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        self.puck_z0[env_ids] = BOX_H + PUCK_R + 0.001 + org[:, 2]
