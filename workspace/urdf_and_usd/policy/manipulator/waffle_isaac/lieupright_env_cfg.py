"""누운 직육면체 세우기 `Waffle-LieUpright-Direct` (2026-11-10 Onbeom, 팀 이슈 #192, 설계 `261110_1923_lie_box_rl/설계_누운직육면체_RL.md`).

상자 위 누운 직육면체 (30 x 30 x 60) 를 위에서 쥐고 -> 10 cm 들고 -> 손 안에서 세우고 -> 상자 위 세울 자리에 세워 놓고 -> 손을 뺌.
그 뒤 다시 잡기는 45° 집기 정책 (n5) 이 함 (이 환경 밖).

배치 = 미션 계획기 (mission_planner/poses.py) 정지 · 회전 접근: 앞 끝 (x 0.1328) -> 상자 면 0.1272 m (정렬 오차 앞뒤 · 좌우 ±1 cm, 요 ±3°),
  누운 직육면체 = 상자 옆 s x 40 mm, 옆 퍽 (세운 원기둥) = −s x 40 mm (각 ±10 mm, 깊이 0 ±10 mm), 누운 퍽 요 0 ~ 180°,
  바닥 모서리가 상자 가장자리 10 mm 안, 틈 ≥ 5 mm, 그 뒤 로봇 중심 (x −0.04035) 둘레 s x 17.06° 회전. 옆 퍽 있음 확률 nb_prob.

행동 6 = 잡는 점 변위 (base x, y, z) x 5 mm, 집게 숙임 β 변화 x 0.05 rad, 집게 롤 ρ 변화 x 0.06 rad, 집게 (> 0 벌림).
  이 팔은 관절 2 · 3 · 4 가 같은 축이라 집게 방향은 '팔 평면 (관절1 축 -> 잡는 점) 안 숙임' 과 '집게 축 둘레 롤' 만 자유롭다.
  집게 방향 a = −cos β z + sin β h (h = 관절1 축 -> 잡는 점 수평 방향), 닫는 축 · 손가락 높이 방향 = (t, a x t) 를 a 둘레로 ρ 돌림 (t = z x h).
  β = 0 아래, π/4 = 45° 집기 시작 자세, π/2 수평 (ρ = 0 이면 손가락 높이 방향 = 위 = 쥔 퍽이 섬). 팀 LiePick 의 IK (감쇠 최소제곱) 그대로.
  한 번 놓으면 (세워 놓고 벌림) 집게는 계속 벌림.
"""

from __future__ import annotations

import os

from isaaclab.utils import configclass

import isaaclab.sim as sim_utils

from .lie_base_env_cfg import WaffleSidePick45EnvCfg  # 릴리스 rl-20261110 판 복사본 (dev 에 들어오면 .sidepick_env_cfg 로)

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@configclass
class WaffleLieUprightEnvCfg(WaffleSidePick45EnvCfg):
    episode_length_s = 12.0
    action_space = 6
    observation_space = 45
    cart_scale = 0.005
    beta_scale = 0.05  # [rad / 스텝]
    rho_scale = 0.06
    beta_range = (-0.15, 1.75)  # [rad]
    rho_max = 2.6  # [rad] (joint5 ±2.8 안)
    rot_hold_weight = 1.0
    arm_vmax = 0.0  # [rad/s] 2026-11-11 평가용: > 0 이면 팔 관절 목표 변화를 이 속도로 자름 (실물 노드 제한 모사, 0 = 끔 = 학습 때)
    arm_vmax_range = None  # 2026-11-11 B6: (lo, hi) 면 에피소드마다 균등 무작위 상한 (arm_vmax 대신)
    obs_node = False  # 2026-11-11 B6: True = 관측을 미션 노드와 같게 (퍽 자리 · 긴 축 · 옆 퍽 = 시작 때 값 고정)
    obs_box_noise = (0.0, 0.0)  # 2026-11-11 B7: (m, rad) > 0 이면 관측 상자 · 세울 자리 D 를 노드처럼 잰 상자로 (판정은 참 상자)
    node_release_rule = False  # 2026-11-11 B7: True = 노드 놓음 규칙 (든 뒤 처음 벌림이 창 밖이면 종료 + rew_open_away)
    release_xy = 0.03
    release_z = 0.06
    release_beta = 1.4
    release_center_offset = 0.011
    rew_open_away = -30.0
    arm_vlimit_mode = "clip"  # 2026-11-11: "clip" = 관절별로 자름 (β · ρ 는 다 쌓임), "scale" = 노드 lieApplyActionLimited (행동 앞 5 칸 비례 축소)
    ready_noise = 0.05
    gripper_open = 0.010

    # 배치 (미션 정지 · 회전 접근)
    front_m = 0.1328
    gap_m = 0.1272
    gap_noise = 0.01
    lat_noise = 0.01
    yaw_noise = 0.0524  # ±3°
    center_x_m = -0.04035
    puck_dx = 0.040
    puck_jit = 0.010
    edge_in = 0.010
    gap_min_pucks = 0.005
    nb_prob = 0.5
    lie_z = 0.070 + 0.016  # 누운 퍽 중심 (상자 위 + 15 mm + 1 mm, chain45 와 같음)

    # 시작 상태 (역방향 커리큘럼): 처음부터 · 쥐고 든 상태 · 세운 채 세울 자리 위 (각본 chain45 --dump_starts 로 모음)
    start_probs = (0.4, 0.3, 0.3)  # (처음부터, 쥐고 듦, 세움[, above, at])
    starts_file = os.path.join(HERE, "lieupright_starts.json")
    f1_align_w = 0.0  # B2-b: f1 = 거리 x (1 − w + w x 롤 맞춤). 0 = 1 회차 (거리만)

    # 인식 (#191 실측 반영, 수평 x · y) + 높이는 상자 기하 (윗면 70 mm ±3 mm + 15 mm)
    perc_bias = 0.005
    perc_jitter = 0.001
    perc_delay_steps = (3, 9)
    perc_period_steps = (3, 10)
    perc_drop_p = (0.1, 0.4)
    box_h_noise = 0.003
    perc_yaw_noise = 0.052  # 긴 축 방향 에피소드 치우침 ±3°
    perc_freeze_dist = 0.06  # 잡는 점이 인식 퍽에서 이만큼 안이면 인식값 고정 (팔 카메라가 너무 가깝거나 손 안)

    # 쥐기 · 세우기
    grasp_dx = 0.010  # 잡는 점 = 가운데에서 긴 축 10 mm (세운 뒤 위쪽 끝 쪽, 팀 각본 lie_dx). 가운데면 세웠을 때 손가락 밑이 퍽 바닥 위 1 mm
    grasp_dz_a = 0.011  # 잡는 점 = 퍽 겉 위 (집게 방향 반대로 11 mm, 팀 각본)
    grasp_tol = 0.004  # 쥔 자리가 목표에서 긴 축 방향 4 mm 안 = 잘 쥠 (+10)
    grasp_bad = 0.008  # 8 mm 넘으면 −20
    grasp_finger_range = (-0.0108, -0.003)  # 30 mm 쪽을 쥠 (팀 LiePick)
    hold_dist = 0.03
    hold_n_on = 8  # [스텝] 2026-11-10 B2 진단 · RL1 검토 3: 쥠 = 연속 8 스텝 (빈손으로 닫힐 때 범위 지나는 데 약 6 스텝)
    hold_n_off = 3  # 놓음 = 연속 3 스텝
    obs_hold_mode = "truth"  # 2026-11-11 실물 쥠 판정 (RL1): 관측 35 칸 쥠만 바꿈. "truth" = 손가락 범위 + 퍽 중심-잡는 점 hold_dist (학습 판정), "finger" = 손가락 범위만 (실물에서 낼 수 있는 값). 보상 · 종료 · 성공 판정은 늘 truth
    lift_ok = 0.03
    lift_full = 0.10
    stand_up_ok = 0.95  # 쥔 쪽 끝 방향 · 위 (쥔 채 세움)
    place_dz = 0.003  # 퍽 바닥 - 상자 윗면 (잘 놓음)
    place_tilt = 0.0873  # 5°
    topple_tilt = 0.35  # 20° 넘게 기울면 넘어짐
    release_clear = 0.03  # 손 (잡는 점) 이 퍽에서 3 cm 넘게
    success_hold_s = 1.0
    nb_move_max = 0.003
    nb_move_world = False  # 2026-11-11: True = 옆 퍽 밀림을 월드 좌표 (시작 자리) 로 잼 (False = 옛 base 좌표, 차체가 돌면 거짓 밀림)
    slip_deg = 10.0
    spot_r = (0.27, 0.32)  # 세울 자리 관절1 축 거리 (팀 각본: 수평 집게는 0.27 m 이상)
    spot_edge = 0.030
    spot_nb = 0.07
    # 2026-11-10 검토 A (RL1 세션): 성공 = 다시 잡을 수 있는 자리. 세울 자리 D 에서 3 cm 안 (n5 집기 배치 범위 안으로),
    # 퍽 중심 -> 상자 가장자리 25 mm 넘게 (17.5 mm 면 세운 퍽이 떨어짐 = 팀 각본 기록, 설계 전제 B 의 '가장자리 10 mm 안쪽' = 중심 25 mm), 옆 퍽 7 cm 넘게 (세울 자리 규칙)
    spot_tol = 0.03
    spot_edge_min = 0.025

    # 보상 (설계 5 절)
    rew_progress = 20.0  # 퍼텐셜 차이 x 이것 (단계 4 개, 각 0 ~ 1)
    reach_norm = 0.15
    spot_norm = 0.15
    rew_center_grasp = 10.0
    rew_off_grasp = -20.0
    rew_placed = 30.0
    rew_placed_bad = 5.0  # 검토 A: 다시 잡기 어려운 자리에 놓음 (그 뒤 1 s 서면 성공 없이 종료)
    rew_success = 100.0
    rew_drop = -30.0
    rew_topple = -30.0
    rew_nb = -30.0
    rew_self = -100.0
    rew_arm_body = -50.0
    rew_body_box = -50.0
    rew_slip = -0.5
    rew_arm_box = -2.0
    arm_box_term = False  # 2026-11-11 r2d: True = 팔-상자 접촉 (> contact_force) 이면 종료
    rew_arm_box_term = -30.0
    rew_nb_pot = 0.0  # 2026-11-11 B5: 다가가기 옆 퍽 수평 여유 퍼텐셜 계수 (0 = 끔)
    nb_margin = 0.04  # [m] 여유 퍼텐셜이 다 차는 거리 (원기둥 겉 기준)
    rew_time = -0.05
    rew_action_rate = -0.01

    def __post_init__(self):
        super().__post_init__()
        self.pick_j3_max = None  # 세우기 · 놓기 자세는 집기와 다름. 몸체 · 자기 충돌은 종료 조건으로 막음
        # 퍽 질량 = 사용자 실측 (2026-11-13: 원기둥 13 g, 사각기둥 17 g = dice2_cfg.PUCK_MASS). 이전 #192 값 13.5 · 17.5 g
        for k_, m_ in (("puckT_cyl", 0.013), ("puckT_box", 0.017), ("puckN", 0.013)):
            o_ = getattr(self.scene, k_)
            setattr(self.scene, k_, o_.replace(spawn=o_.spawn.replace(mass_props=sim_utils.MassPropertiesCfg(mass=m_))))


@configclass
class WaffleLieUprightB2EnvCfg(WaffleLieUprightEnvCfg):
    """P1 2 회차 (B2, 2026-11-10): 쥐기 전 시작 상태 above (잡는 점 6 cm 위) · at (잡는 점, 벌림) 추가 + f1 롤 맞춤."""
    start_probs = (0.3, 0.2, 0.2, 0.15, 0.15)
    starts_file = os.path.join(HERE, "lieupright_starts_v2.json")
    f1_align_w = 0.5


@configclass
class WaffleLieUprightB3EnvCfg(WaffleLieUprightB2EnvCfg):
    """P1 r2d (2026-11-11, RL1 승인): B2 + 팔-상자 접촉 1 N 넘으면 종료 · −30 (사용자 기준: 상자 접촉 = 실패)."""
    arm_box_term = True


@configclass
class WaffleLieUprightB4EnvCfg(WaffleLieUprightB3EnvCfg):
    """P1 r2e (2026-11-11, RL1 지시: R4 옆 퍽 닿음 0 % 불통과 -> 설계 바꿔 반복): B3 + 옆 퍽 닿음 · 밀림 · 넘어짐 종료 벌점 −30 -> −60."""
    rew_nb = -60.0


@configclass
class WaffleLieUprightB5EnvCfg(WaffleLieUprightB4EnvCfg):
    """P1 r2e 2 회차 (2026-11-11, RL1 승인: 1 회차가 채택 기준 못 넘으면): B4 + 쥐기 전 옆 퍽 수평 여유 퍼텐셜 (계수 20, 4 cm 에서 다 참)."""
    rew_nb_pot = 20.0


@configclass
class WaffleLieUprightB6EnvCfg(WaffleLieUprightB4EnvCfg):
    """r2g (2026-11-11, RL 261110 1 조건부 승인 · 사용자 승인): B4 + 관측을 미션 노드와 같게 (퍽 자리 · 긴 축 · 옆 퍽 시작 값 고정, 쥠 = 손가락 범위만)
    + 관절 속도 상한 노드 비례 축소 (scale) 에피소드마다 U(2, 4) rad/s + 옆 퍽 밀림 월드 판정. 시작은 처음부터 · 쥐기 전 (above · at) 만 (쥔 · 세운 시작은 노드 관측과 맞지 않음).
    까닭: 미션 순서 모의에서 노드의 긴 축 고정 때문에 정책 단계 성공 90.2 % (긴 축만 살리면 97.1 %), 노드 FK 로는 못 품 (MD2)."""
    obs_node = True
    obs_hold_mode = "finger"
    arm_vlimit_mode = "scale"
    arm_vmax_range = (2.0, 4.0)
    nb_move_world = True
    start_probs = (0.5, 0.0, 0.0, 0.25, 0.25)


@configclass
class WaffleLieUprightB7EnvCfg(WaffleLieUprightB6EnvCfg):
    """r2g2 (2026-11-11, 미션 파트 261110 3 조건부 승인): B6 + 관측 상자 · D 를 노드처럼 잰 상자 (±3 mm · ±1°) 와 노드 옆 퍽 규칙으로 + 노드 놓음 규칙 (창 밖 벌림 종료 −30).
    까닭: r2g (B6) 미션 순서 M3 89.3 % — 노드 'D 밖에서 벌림' 125/1536 (학습 성공 판정과 노드 판정이 다름)."""
    obs_box_noise = (0.003, 0.01745)
    node_release_rule = True
