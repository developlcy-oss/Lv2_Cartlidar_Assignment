# [복사본 · 2026-11-11 Onbeom, #192] 원본: 팔 담당 LCY (developlcy-oss) 릴리스 rl-20261110 의 waffle_isaac/sidepick_env_cfg.py (md5 b9f68b46b459).
# 누운 직육면체 세워 놓기 환경 (lieupright_env.py) 이 상속하는 릴리스 판이 dev 에 없어 새 이름으로 복사함. 원본에서 import 줄만 바꿈 (아래 '# [복사본]' 표시).
# dev 에 릴리스 판이 들어오면 이 파일을 지우고 lieupright_env* 의 import 를 원래 이름으로 되돌림. 기존 파일은 고치지 않음.
"""정렬·집기 `Waffle-SidePick-Direct` (Direct RL): 상자 위 퍽을 **옆에서** (집게 수평) 집어 세운 채 들기.

사용자 단계 (sim/정책_학습기록.md 4 절): (1) 인식이 준 퍽 좌표로 퍽 정면에 자리 잡기 -> (2) 잡기 좋은 자세 (집게 수평, 카메라 위,
손가락 닫는 축 수평) -> (3) 집기. 옆에서 집어야 퍽이 세워진 채라 뒤 퍽 스토리지 통 위 입구에 넣을 수 있음 (4.4 절).
배치: 로봇 제자리 (주행 담당이 상자 앞에 세움). 상자 205 x 205 x 70 은 로봇 앞 끝 (라이다 받침 x 0.218) 에서 gap 5 ~ 11 cm,
가로 ±6 cm, 요 무작위. 퍽 2 개 = 상자 중심 ± 0.04 (상자 x 축, 미션 시작 상자). 집을 퍽 = joint1 축에서 0.33 m 에 가까운 쪽
(옆에서 집기가 되는 거리 0.29 ~ 0.34 m, 4.6 절), 다른 퍽은 옆 장애물.

행동 6 = 팔 5 관절 목표 += 0.04 rad x 행동 (실제 각 ±0.25 rad 안), 집게 (> 0 벌림 0.015 = 49.6 mm, ≤ 0 닫기).
관측 35 = 팔 위치 (시작 자세 기준) 5 · 속도 5, 왼손가락 1, 인식 퍽 좌표 3 (참값 + 에피소드마다 ±4 mm 치우침 + 스텝마다 ±1 mm),
          잡는 점 3, 퍽 − 잡는 점 3, 집게 방향 (arm_link_5 −z) 3, 손가락 닫는 축 (arm_link_5 x) 3, 옆 퍽 − 퍽 (x, y) 2,
          상자 모양 퍽 여부 1, 이전 행동 6. 위치·방향은 base_link 좌표.
상자 접촉: 접촉 센서 (팔 링크·손가락 vs 상자, 차체·바퀴 vs 상자). **차체가 상자에 닿으면 종료 + 큰 벌점** (사용자 조건).
"""

from __future__ import annotations

import math

from isaaclab_physx.physics import PhysxCfg

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import DirectRLEnvCfg, ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from .dice2_cfg import box_cfg, puck_cfg
from .robot_cfg import ARM_LINKS_REGEX, BODY_LINK, FINGER_CLOSE, FINGER_GRASP_RANGE, FINGER_LINKS_REGEX, FINGER_OPEN_TASK, FLOOR_MATERIAL, WAFFLE_CFG

READY_POSE = {"joint1": 0.0, "joint2": 1.07, "joint3": 0.60, "joint4": -1.66, "joint5": 0.0}  # 261028 N3: 잡는 높이 (퍽 중심 + 15 mm)·joint1 축 0.28 m, 집게 수평. N1 (퍽 4 cm 위) 은 손목 한계, N2 는 손가락이 상자에 닿음
"""시작 자세 = 옆에서 집기 경로의 '퍽 정면 6 cm 앞·4 cm 위' (MuJoCo side_grasp_check, gap 0.08 상자 90° 에서 성공한 자세).
집게 수평, 카메라 위, 손가락 닫는 축 수평. 학습 때 ±0.15 rad 잡음."""

_robot = WAFFLE_CFG.replace(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=WAFFLE_CFG.spawn.replace(
        activate_contact_sensors=True,
        articulation_props=WAFFLE_CFG.spawn.articulation_props.replace(enabled_self_collisions=True),
    ),
    init_state=WAFFLE_CFG.init_state.replace(joint_pos={**WAFFLE_CFG.init_state.joint_pos, **READY_POSE, "gripper_left_joint": 0.010,
                                                        "gripper_right_joint": 0.010}),
)
RED, GREEN = (0.9, 0.1, 0.1), (0.1, 0.7, 0.3)


@configclass
class SidePickSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg(physics_material=FLOOR_MATERIAL))
    robot: ArticulationCfg = _robot
    boxA: RigidObjectCfg = box_cfg("A", (0.38, 0.0))
    puckT_cyl: RigidObjectCfg = puck_cfg("T_cyl", "cyl", RED)  # 집을 퍽: 원기둥·상자 중 하나만 상자 위
    puckT_box: RigidObjectCfg = puck_cfg("T_box", "box", RED)
    puckN: RigidObjectCfg = puck_cfg("N", "cyl", GREEN, (0.42, 0.0))
    arm_box_contact = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/" + "(link[2-5]|gripper_roll_link|gripper_left_link|gripper_right_link)",
        filter_prim_paths_expr=["{ENV_REGEX_NS}/BoxA"], update_period=0.0, history_length=1,
    )
    body_box_contact = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/(" + BODY_LINK + "|wheel_.*)",
        filter_prim_paths_expr=["{ENV_REGEX_NS}/BoxA"], update_period=0.0, history_length=1,
    )
    arm_nb_contact = ContactSensorCfg(  # 2026-10-30 사용자 요청: 목표가 아닌 퍽 (옆 퍽) 을 건드림
        prim_path="{ENV_REGEX_NS}/Robot/" + "(link[2-5]|gripper_roll_link|gripper_left_link|gripper_right_link)",
        filter_prim_paths_expr=["{ENV_REGEX_NS}/PuckN"], update_period=0.0, history_length=1,
    )
    light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.85, 0.85, 0.85)))


@configclass
class WaffleSidePickEnvCfg(DirectRLEnvCfg):
    decimation = 4  # 30 Hz
    episode_length_s = 6.0
    action_space = 4  # 261028 N7: 직교 (잡는 점 x, y, z 변위 + 집게). N1 ~ N6 = 관절 공간 6 (팔 5 + 집게) 은 새 팔에서 수평 다가가기 (관절 2·3·4 같이) 를 못 배움
    observation_space = 33  # 29 + 이전 행동 4
    state_space = 0
    viewer: ViewerCfg = ViewerCfg(eye=(0.95, -0.7, 0.55), lookat=(0.33, 0.0, 0.1), origin_type="env", env_index=0)
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation, physics=PhysxCfg(), physics_material=FLOOR_MATERIAL)
    scene: SidePickSceneCfg = SidePickSceneCfg(num_envs=256, env_spacing=1.5, replicate_physics=True, clone_in_fabric=True)

    action_scale = 0.04  # 관절 공간일 때
    cartesian = True
    cart_scale = 0.005  # [m / 스텝] 잡는 점 변위 최대 (30 Hz -> 15 cm/s)
    rot_hold_weight = 0.3
    ik_damping = 1e-4
    target_lead_max = 0.25
    gripper_open = 0.010  # 261028 N6: 61.6 mm (N4·N5 49.6 mm 는 퍽 양옆 틈 10 mm 라 다가갈 때 퍽을 침, 넘어짐 20 %). 옆 퍽 (8 cm) 과는 안 닿음
    gripper_closed = FINGER_CLOSE

    # 배치 무작위 (로봇 앞 끝 = 라이다 받침 x 0.218)
    robot_front = 0.140  # 261028: TFmini 앞면 (이전 라이다 받침 0.218), geometry_check 1
    gap_range = (0.11, 0.19)  # 261028 N2: 퍽 높이에서 집게 수평은 joint1 축 0.27 m 이상 (Isaac 소프트 한계 joint4 −1.82). 이전 0.05 ~ 0.11
    lat_range = (-0.06, 0.06)
    front_first = True  # SP-U4 (2026-10-31 사용자 결정): 두 퍽 중 joint1 축에 가까운 (로봇 쪽) 퍽을 집음 -> 옆 퍽이 늘 바깥 쪽. 집을 퍽이 너무 가까우면 상자째 바깥으로 밀어 front_r_range 에 둠. 주행 결과에 따라 바뀔 수 있음 (False = 이전: sweet_radial 에 가까운 퍽)
    front_r_range = (0.32, 0.345)  # [m] 집을 (앞) 퍽의 joint1 축 거리. SP-U5 (2026-10-31): 0.30 ~ 0.34 -> 0.32 ~ 0.345 (연결 평가 집기 실패 11 대 중 8 대가 퍽 0.30 ~ 0.32 m)
    # 2026-11-05 주행 정지 기준 배치 (dock_layout, 45° 집기에서 켬): 상자 앞면 = 로봇 앞면 + dock_gap, 두 퍽 = 상자 가운데 ± pair_spacing / 2
    dock_layout = False
    dock_gap = (0.15, 0.19)  # [m] 로봇 앞면 (TFmini, x 0.140) -> 상자 앞면. 기준 0.17 ± 0.02 (주행 정밀도)
    dock_lat = (-0.04, 0.04)  # [m] 좌우 정지 오차
    dock_yaw = 0.087  # [rad] 상자 요 (±5°, 주행 요 오차)
    dock_side_prob = 0.8  # 두 퍽이 옆으로 나란히 (같은 거리) 인 경우. 나머지는 로봇 쪽으로 한 줄
    dock_pair_noise = 0.17  # [rad] 두 퍽 줄 방향 잡음 (±10°)
    pair_spacing = 0.08  # [m] 두 퍽 중심 간격
    pair_jitter = 0.01  # [m] 두 퍽 가운데가 상자 가운데에서 벗어난 정도
    dock_second_prob = 0.5  # 두 번째 퍽 집기 (첫 퍽은 실어서 옆 퍽 없음) 에피소드 비율
    dock_r_min, dock_r_max = 0.28, 0.365
    dock_rand_prob = 0.0  # 2026-11-06 사용자: 정지 거리는 그대로, 상자 위 두 퍽 자리 · 간격 · 방향 무작위 (이 확률로). 두 퍽 모두 45° 집기 닿는 곳 (관절1 축 dock_r_min ~ dock_r_max) 에서만 뽑음
    dock_rand_edge = 0.02  # [m] 퍽 중심과 상자 가장자리 최소 거리 (퍽 반지름 15 + 5 mm)
    dock_rand_gap = 0.045  # [m] 두 퍽 중심 최소 간격 (지름 30 + 15 mm)
    close_hold_steps = 0  # 2026-11-06 '멈춘 뒤 닫기' 규칙: 집게 닫는 명령이 시작된 뒤 이 스텝 동안 (퍽이 들리기 전) 잡는 점 이동을 0 으로 (행동 규칙, 연결 실행도 같게)
    rew_pre_push = 0.0  # 2026-11-06 쥐기 전 퍽 밀기: 퍽이 들리기 전 처음 자리에서 수평으로 움직인 거리 / 5 mm x 이것 (최대 3 배) / 스텝
    rew_close_still = 0.0  # 2026-11-06 닫는 동안 팔 멈춤: 집게 닫는 명령 중 · 퍽이 들리기 전 · 잡는 점 3 cm 안에서 잡는 점 속도 / 2 cm/s x 이것 (최대 3 배) / 스텝  # [m] 집을 퍽 관절1 축 거리를 이 안으로 (밖이면 차체를 조금 움직인 것처럼 상자째 x 로 옮김)
    sweet_radial = 0.33  # [m] 옆에서 집기 좋은 joint1 축 거리 (261028 N2)
    joint1_x = 0.096  # 261028 (J1 브라켓 +9 mm)
    ready_noise = 0.15
    near_start_prob = 0.3  # 261028 N4 0.4 -> N6 0.5 -> N10 0.3 (실제 과제 연습 늘림): 역방향 커리큘럼 (퍽을 시작 자세 잡는 점 바로 앞에 두고 시작). N1 ~ N3 은 한 번도 못 쥐어 퍽 위 4 ~ 7 cm 에 머묾
    near_dx_max = 0.08  # [m] N4 0.035 -> N6 0.08 (실제 과제 평가 3 % = 가까운 경우만 배움 -> 거리를 단계적으로)
    near_dy_max = 0.03  # [m] N4 0.008 -> N6 0.03
    near_noise = 0.03  # [rad] 그때 시작 자세 잡음
    near_grasp_x = 0.376  # [m] 시작 자세 (READY) 의 잡는 점 x (base_link, joint1 축 0.28 m)
    perc_bias = 0.004  # [m] 인식 좌표 에피소드 치우침
    perc_jitter = 0.001  # [m] 스텝마다
    # 2026-11-10 #191 팔 카메라 실측 (Jetson 태그): 결과 3.2 ~ 9 Hz, 지연 중앙 105 · p95 278 ms, 팔 움직일 때 끊김 (J1 0.5 rad/s 에서 17 ~ 37 %, 최대 간격 232 ms).
    # (0, 0) · (1, 1) · (0, 0) = 끔 (예전과 같음). 범위에서 에피소드마다 뽑음 (30 Hz 스텝 기준)
    perc_delay_steps = (0, 0)  # 지연 [스텝]
    perc_period_steps = (1, 1)  # 갱신 주기 [스텝] (사이에는 마지막 값 유지)
    perc_drop_p = (0.0, 0.0)  # 갱신 때 끊길 확률 (팔 정지 ~ 관절1 0.5 rad/s 이상, 사이는 비례)
    perc_drop_max = 7  # 끊김으로 더 기다리는 최대 스텝 (넘으면 강제 갱신)
    box_shape_prob = 0.0  # 2026-10-30 SP-U1: 직사각형 퍽은 누워 있음 -> 위에서 가운데 잡기 (Waffle-LiePick-Direct). 여기는 세운 원기둥만. 이전 0.5
    grasp_dz = 0.015  # 261028 N3: 새 손가락은 집게 축 아래로 29 mm (패드는 ±15) -> +6 이면 손가락 밑이 상자 위 2 ~ 7 mm 라 닿음 (N1·N2 실패). +15 = 16 mm 띄움, 패드가 퍽 위쪽 절반 쥠, 넣을 때 손가락이 통 위에 닿을 때 퍽 16 mm 들어감
    _old_grasp_dz = 0.006  # [m] 잡는 점 목표 = 퍽 중심 + 6 mm. SP1·SP2 는 +18 (상자 스침 방지) 이었으나 통 위 입구에 넣을 때 손가락이 카메라 가로대에
    #   2 ~ 3 mm 겹침 -> +4 ~ 8 이면 안 겹침, 상자 집기는 +8 에서 5/6 (4.9 절 정정, 2026-10-28) -> +6

    # 판정
    near_dist = 0.03
    grasp_dist = 0.02
    grasp_finger_range = FINGER_GRASP_RANGE  # 261028: 30 mm 퍽 쥐면 약 −0.006
    grasp_tilt_max = 0.05
    lift_min = 0.02
    lift_goal = 0.05
    success_lift = 0.03
    success_hold_s = 1.0
    upright_min = 0.95  # |퍽 축 z|
    topple_cos = 0.7
    fall_drop = 0.02
    contact_force = 1.0  # [N]
    base_move_max = 0.01  # [m] 차체가 이만큼 밀리면 종료 (팔이 상자를 밀어 로봇이 밀림)

    # 보상
    rew_reach = 1.0
    rew_reach_fine = 1.0
    rew_horizontal = 0.5  # 집게 수평 (12 cm 안)
    rew_camera_up = 0.3
    rew_point = 0.5  # 집게가 퍽을 향함 (2 ~ 12 cm)
    rew_align = 0.5  # 상자 퍽: 손가락과 면
    rew_grasp = 3.0
    rew_lifted = 6.0
    rew_lift_height = 10.0
    rew_puck_tilt = -2.0  # 가까울 때 (1 − |퍽 축 z|)
    rew_arm_box = -2.0  # 팔·손가락이 상자에 닿은 스텝
    rew_body_box = -20.0  # 차체가 상자에 닿음 (종료)
    rew_base_move = -10.0  # 차체 밀림 (종료)
    rew_action_rate = -0.02
    rew_joint_vel = -1e-3
    rew_neighbor_moved = -20.0  # [/m 옆 퍽 이동]
    rew_nb_touch = -10.0  # 2026-10-30 SP-U1 사용자 요청: 팔·손가락이 목표가 아닌 퍽 (옆 퍽) 에 닿은 스텝 (강한 음의 보상)
    rew_nb_topple = -50.0  # 옆 퍽을 쓰러뜨림 (축 z < topple_cos) + 종료
    rew_nb_touch_end = -30.0  # SP-U3 (2026-10-31): 옆 퍽에 닿으면 이것 + **종료** (U2 단독·연결에서 옆 퍽 닿은 대 41 % · 34 %, 스텝 벌점 −10 만으로는 안 피함)
    nb_touch_terminate = True
    rew_nb_near = -3.0  # SP-U3: 손가락이 옆 퍽 옆면에 nb_near_m 안으로 가까우면 (1 − 거리/nb_near_m) x 이것 = 미리 피하게
    nb_near_m = 0.03
    rew_topple_end = -30.0  # SP-U3: 목표 퍽을 넘어뜨리면 이것 + **종료** (연결 평가 한 번 시도 기준 넘어뜨림 27 %; 단독 평가는 리셋 재시도로 가려졌음)
    topple_terminate = True
    rew_toppled = -2.0  # 261028 N10: −0.5 -> −2 (N9 실제 과제 실패 대부분이 넘어뜨림 24/64)
    rew_base_tilt = -8.0
    rew_fall = -5.0
    grasp_pitch = 0.0  # 2026-11-03: 집게를 아래로 기울이는 각 [rad] (0 = 수평 옆 쥠). 손가락 닫는 축은 수평 그대로라 쥔 퍽은 세운 채
    # 2026-11-03 SP-45b (chain45 점검): 0 이면 끔 (수평 SidePick 은 그대로). 45° 판에서 켬
    rew_arm_body_end = 0.0  # 팔·손가락이 로봇 몸체 (base_link) 에 1 N 넘게 닿으면 이것 + 종료 (scene.arm_body_contact 필요). 연결에서 집기 중 248 · 801 N
    rew_held_tilt = 0.0  # 퍽을 든 동안 (lift > lift_min, 잡는 점 3 cm 안) 퍽 기울기 / held_tilt_ref x 이것 (최대 3 배) / 스텝. 쥔 퍽이 집기 끝에 1 ~ 11° 기울고 운반 중 최대 32° 까지 돎
    held_tilt_ref = 0.0873  # [rad] 5°
    # 2026-11-04 사용자 요청: 자기 충돌 (집게 · 손가락 · 쥔 퍽이 윗팔 · 아랫팔 · 모터를 뚫음) · 관절 3 과도하게 굽힘 금지 (0 = 끔)
    rew_self_end = 0.0  # 집게 · 쥔 퍽과 팔 링크 선분 거리가 self_margin 아래면 이것 + 종료 (시뮬레이션은 팔 링크끼리 충돌이 꺼져 있어 거리로 판정)
    self_margin = 0.055  # [m] 링크 · 모터 반폭 약 25 mm + 집게 · 손가락 반폭 약 30 mm
    rew_j3_bend = 0.0  # |관절 3| 이 j3_bend_start 를 넘은 만큼 / 0.1 rad x 이것 / 스텝 (최대 3 배)
    j3_bend_start = 1.40  # [rad] 80°
    self_box_tol = 0.005  # [m] 2026-11-06: 팔끼리 충돌 상자 (팀 URDF) 겹침이 이 깊이를 넘으면 self_bad (실물 전원 자세 stow 가 4.1 mm 겹침 = 상자가 실물보다 조금 큼)
    j3_bend_end = 1.55  # [rad] 89° 넘으면 rew_self_end 와 같은 벌점 + 종료
    grip_roll_pi = False  # 2026-11-04: 집게를 롤 180° 뒤집은 채 쥐기 (카메라 받침 아래, joint5 ≈ ±π, 시뮬레이션 joint5 한계 ±3.3 rad). LieRoll45: 쥔 뒤 롤 180° 로 정상 방향이 되며 퍽이 섬
    pick_j3_max = None  # [rad] 관절 3 목표 상한 (None = 관절 한계). SP-45c: 1.3 (못 쥐면 관절 3 1.54 ~ 1.58 로 접어 몸체에 809 N, 정상 집기는 −1.4 ~ −0.8)
    grasp_dz_tol = 1.0  # [m] 2026-11-07 쥠 · 듦 · 성공 판정에 잡는 높이 (잡는 점 − 퍽 중심 − grasp_dz) 조건. 1.0 = 끔
    rew_grasp_fit = 0.0  # 2026-11-06 쥐는 깊이 · 높이 일정하게: 퍽을 든 동안 (퍽 중심 − '잡는 점 − grasp_dz 수직') 의 손가락 축 뺀 크기 / grasp_fit_ref x 이것 (최대 3 배) / 스텝 = 놓기 가정 (--est) 오차
    grasp_fit_tol = 1.0  # [m] 2026-11-09 손 안 퍽 가정 오차가 이 안이어야 성공 (1.0 = 끔). rew_fit_bonus 도 이 안일 때
    rew_fit_bonus = 0.0  # 2026-11-09 든 동안 가정 오차 < grasp_fit_tol 인 스텝마다
    grasp_fit_ref = 0.002  # [m] 칸 틈 약 1 mm, 가정 오차 2 ~ 3 mm 넘으면 칸에 못 앉음 (run_seat.sh)
    rew_grasp_dz = 0.0  # 쥔 동안 (잡는 점 높이 − 퍽 중심) 이 grasp_dz 에서 벗어난 만큼 / 5 mm x 이것 (최대 3 배) / 스텝. 연결에서 2 ~ 24 mm 로 흩어짐


READY_POSE_45_OLD = {"joint1": 0.0, "joint2": 0.247, "joint3": 0.296, "joint4": 0.242, "joint5": 0.0}  # 잡는 점 관절1 축 0.28 m · 퍽 중심 + 33 mm (정지 15 ~ 19 cm 때)
READY_POSE_45 = {"joint1": 0.0, "joint2": -0.338, "joint3": 0.559, "joint4": 0.564, "joint5": 0.0}
"""2026-11-06 정지 거리 5 ~ 6 cm: 퍽이 관절1 축 0.18 ~ 0.28 m 에 오므로 예전 시작 자세 (잡는 점 0.28 m, 퍽 중심 + 33 mm) 면 시작부터 손가락이 퍽에 겹칠 수 있음
-> 잡는 점 관절1 축 0.22 m · 퍽 중심 + 80 mm (퍽 위로), 집게 45° 아래 (팀 URDF FK, 관절 2 · 3 · 4 = −19 · 32 · 32°). 연결 실행 (chain45) 의 시작 · 복귀 자세도 이것."""
"""2026-11-03 SP-45: READY_POSE 와 같은 잡는 점 (+20 mm 위) 에서 집게 45° 아래 (팀 URDF FK·IK, 관절 한계 여유 1.3 rad)."""


@configclass
class WaffleSidePick45EnvCfg(WaffleSidePickEnvCfg):
    """세운 퍽을 집게 45° 기울여 쥐기 (2026-11-03 사용자 결정): 팔레트 12 칸 모두 닿게 (수평 0° 는 바깥 열만, 관절1 ±172°).
    손가락 닫는 축 수평 -> 쥔 퍽은 세운 채, 잡는 점 = 퍽 중심 + 15 mm (팀 B안 팔레트 궤적 TCP 와 같은 점)."""

    def __post_init__(self):
        super().__post_init__()
        import math
        self.grasp_pitch = math.radians(45.0)
        self.box_shape_prob = 0.5  # 2026-11-04 사용자: 세운 직육면체도 롤 없이 원기둥처럼 45° 로 쥠 -> 학습에 반반 (이전 SP-45 · 45b 는 원기둥만)
        # SP-45b (2026-11-03): 팔-몸체 접촉 종료, 든 퍽 기울기·잡는 높이 벌점
        self.scene.arm_body_contact = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/" + "(link[2-5]|gripper_roll_link|gripper_left_link|gripper_right_link)",
            filter_prim_paths_expr=["{ENV_REGEX_NS}/Robot/" + BODY_LINK], update_period=0.0, history_length=1)
        self.rew_arm_body_end = -50.0
        self.rew_held_tilt = -4.0
        self.rew_grasp_dz = -2.0
        # SP-45c (2026-11-04): 퍽 마찰 0.5 (min 결합) = 연결 실행 (chain45, 칸 끼임 방지) 과 같게. 1.0 으로 학습한 정책이 0.5 에서 손 안 퍽이 15 ~ 22° 돎
        mat = sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.5, friction_combine_mode="min")
        for k in ("puckT_cyl", "puckT_box", "puckN"):
            o = getattr(self.scene, k)
            setattr(self.scene, k, o.replace(spawn=o.spawn.replace(physics_material=mat)))
        self.pick_j3_max = 1.3
        self.dock_rand_prob = 1.0  # 2026-11-06 사용자: 퍽 자리 무작위 (정지 거리 5 ~ 6 cm 부터는 전부 무작위, 닿는 곳에서만 = 상자 밀기 없음)
        self.dock_gap = (0.05, 0.06)  # 2026-11-06 사용자 확정: 차체는 항상 상자 앞면 5 ~ 6 cm 에 멈춤 (이전 0.15 ~ 0.19)
        self.dock_r_min = 0.18  # 관절1 축 18 cm 보다 가까우면 45° 로 쥘 때 관절3 68° 넘고 팔꿈치가 차체 위로 (상자 앞 약 6 cm 띠는 뺌)
        self.grasp_dz_tol = 0.006  # 2026-11-07 밤샘 near 직육면체가 퍽 윗 모서리 (잡는 높이 평균 25 mm, 목표 15) 를 쥐고 들어 보상을 받음 -> 6 mm 밖이면 쥠 · 듦 · 성공 아님
        self.ready_noise = 0.08  # 2026-11-07 시작 자세 (퍽 위 80 mm) 에 ±0.15 rad 면 시작부터 집게가 퍽에 겹쳐 튕겨 나가는 대 있음
        self.close_hold_steps = 0  # 2026-11-06 hold1 (12 = 0.4 s 멈춤) 실패: 직육면체 정책이 못 쥐게 무너짐 (0/64), 원기둥 18/48 -> 끔
        self.rew_pre_push = -3.0  # 2026-11-06 집다가 넘어뜨림 줄이기 (48 대 중 6 ~ 12)
        self.rew_close_still = -1.0  # 2026-11-06 쥐는 깊이 어긋남 (닫으며 밀고 들어감) 줄이기
        self.rew_grasp_fit = -3.0  # 2026-11-06 사용자: 쥐는 깊이 · 높이 일정하게 (가정 오차 못 앉은 대 중앙 3 ~ 4 mm -> 2 mm 안)
        self.dock_layout = True  # 2026-11-05 사용자: 주행 정지 기준 (상자 앞면 0.17 m) + 상자당 2 개 (두 번째 퍽 = 옆 퍽 없음) 로 학습 배치
        self.rew_self_end = -100.0  # 자기 충돌 · 관절 3 89° 넘음 -> 강한 벌점 + 종료
        self.rew_j3_bend = -5.0
        self.scene.robot = self.scene.robot.replace(init_state=self.scene.robot.init_state.replace(
            joint_pos={**self.scene.robot.init_state.joint_pos, **READY_POSE_45}))


@configclass
class WaffleSidePick45ChainEnvCfg(WaffleSidePick45EnvCfg):
    """2026-11-09 세운 퍽 45° 집기 다시 학습 (연결 기준). 원기둥 n5 (진단 60/64) 의 연결 상자당 2 개 = 원기둥 19 · 직육면체 22 /48,
    가장 큰 실패 '칸에 못 앉음' 14 ~ 18 = 손 안 퍽 가정 (잡는 점 − 15 mm) 오차. fit 학습은 4 mm 에서 멈췄고 (인식 치우침 ±4 mm 보다 정확히 못 쥠),
    연결에서 치우침 ±1 mm 면 원기둥 35/48 · 못 앉음 5 (2026-11-06). -> 사용자: 인식은 신경 쓰지 말 것 -> 치우침 ±1 mm 로 학습,
    성공 = 가정 오차 4 mm 안 (칸 틈 · 연결 기준), 맞게 쥔 채 든 스텝 보너스, 오차 벌점 −3 -> −5. 원기둥 · 직육면체 반반 그대로."""

    def __post_init__(self):
        super().__post_init__()
        self.perc_bias = 0.001
        self.perc_delay_steps = (3, 9)  # 2026-11-10 #191 실측 (지연 105 ~ 278 ms)
        self.perc_period_steps = (3, 10)  # 3.2 ~ 9 Hz
        self.perc_drop_p = (0.1, 0.4)
        self.grasp_fit_tol = 0.004
        self.rew_fit_bonus = 3.0
        self.rew_grasp_fit = -5.0


@configclass
class WaffleLieEndEnvCfg(WaffleSidePickEnvCfg):
    """누운 직육면체 해석 2 (2026-11-04 사용자 결정): 반지름 방향으로 누운 퍽 (긴 축이 관절1 쪽을 향함, 주행 조건) 의 **안쪽 끝**을
    수평 · 바깥 집게로 쥐고 3 cm 들기. 뒤 (각본): 손목을 아래로 숙여 퍽을 집게 아래로 세워 매달고 1 · 2열 칸에 수직으로 넣음.
    관측 · 행동은 SidePick (수평) 과 같음 -> U4 체크포인트에서 이어 학습. '인식 퍽 좌표' = 잡을 점 − grasp_dz (U4 와 같은 뜻)."""

    end_dx = 0.015  # [m] 잡는 점 = 퍽 안쪽 끝에서 긴 축 따라 15 mm 안
    end_yaw_noise = 0.087  # [rad] 긴 축 방향이 반지름 방향에서 벗어난 정도 (±5°, 주행 · 배치 오차)

    def __post_init__(self):
        super().__post_init__()
        self.box_shape_prob = 1.0  # 목표 = 직육면체 (부모 리셋이 세워 놓으면 LieEnd 리셋이 반지름 방향으로 눕힘)
        self.near_start_prob = 0.0  # 역방향 커리큘럼 끔 (세운 퍽용)
        self.grasp_dz = 0.016  # 잡는 점 = 퍽 축 + 16 mm (손가락 밑 29 mm 가 상자 위 2 mm)
        self.rew_align = 0.0  # 세운 상자용 면 맞춤 (누운 퍽은 몸 x 가 수직이라 뜻 없음)
        self.front_r_range = (0.31, 0.335)  # 퍽 중심 (안쪽 끝 + 15 mm = 잡는 점 0.295 ~ 0.32 m, IK: 퍽 중심 0.28 m 부터 됨)
        # SP-45b·c 와 같은 안전 · 품질 항목
        self.scene.arm_body_contact = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/" + "(link[2-5]|gripper_roll_link|gripper_left_link|gripper_right_link)",
            filter_prim_paths_expr=["{ENV_REGEX_NS}/Robot/" + BODY_LINK], update_period=0.0, history_length=1)
        self.rew_arm_body_end = -50.0
        self.rew_held_tilt = -4.0  # 든 퍽이 수평에서 기운 각 / 5°
        self.rew_grasp_dz = -2.0
        self.pick_j3_max = 1.3
        self.rew_self_end = -100.0  # 자기 충돌 · 관절 3 89° 넘음 -> 강한 벌점 + 종료
        self.rew_j3_bend = -5.0
        mat = sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.5, friction_combine_mode="min")
        for k in ("puckT_cyl", "puckT_box", "puckN"):
            o = getattr(self.scene, k)
            setattr(self.scene, k, o.replace(spawn=o.spawn.replace(physics_material=mat)))


@configclass
class WaffleLieRoll45EnvCfg(WaffleSidePick45EnvCfg):
    """누운 직육면체를 45° 로 쥐기 (2026-11-04 사용자 제안): 긴 축이 로봇 쪽 (반지름 방향) 으로 누운 퍽을 원기둥처럼 집게 45° 아래 · 바깥으로,
    가운데에서 로봇 쪽 끝으로 15 mm 옮긴 곳 (+ 축 위 8 mm, 손가락 밑이 상자 위) 을 쥐고 3 cm 들기. 뒤 (각본, chain45 --lie_roll): 5 cm 더 들고
    손목 롤 (joint5) 180° -> 퍽이 세로, 집게는 45° 그대로 = 세운 원기둥 45° 쥔 것과 같음 -> 원기둥 45° 경로로 12 칸.
    관측 · 행동 = 45° 집기와 같음 -> SP-45c 체크포인트에서 이어 학습. 자기 충돌 · 관절 3 · 팔-몸체 금지 조건 그대로."""

    end_dx = 0.015  # [m] 잡는 점 = 가운데에서 로봇 쪽 끝으로 15 mm (WaffleLieEndEnv: 퍽 − 긴 축 (0.03 − end_dx))
    end_yaw_noise = 0.087  # [rad] 긴 축이 반지름 방향에서 벗어난 정도 (±5°, 주행 · 배치 조건)

    def __post_init__(self):
        super().__post_init__()
        self.box_shape_prob = 1.0
        self.near_start_prob = 0.0
        self.grasp_dz = 0.008  # 잡는 점 = 누운 퍽 축 + 8 mm (45° 집게 손가락 아래 끝이 상자 위 약 3 mm)
        self.rew_align = 0.0  # 세운 상자용 면 맞춤 (누운 퍽은 뜻 없음)
        self.front_r_range = (0.30, 0.33)  # 퍽 중심 (IK: 0.28 ~ 0.34 m 에서 45° 쥐기 여유 0.88 ~ 1.12 rad)
        # 2026-11-04 v2: 정상 방향 (롤 0) 으로 쥐고 180° 돌리면 운반 중 집게가 뒤집혀 카메라 받침이 몸체 · 팔레트에 닿음 (16 대 중 12 대, 최대 668 N)
        # -> 뒤집은 채 (롤 180°) 쥐고, 쥔 뒤 롤 180° 로 정상 방향이 되며 퍽이 섬 = 원기둥 45° 와 같은 집게 방향
        self.grip_roll_pi = True
        self.rew_grasp_dz = -6.0  # v2 연결: 잡는 높이가 10 ~ 23 mm 로 흩어져 (목표 8) 롤 뒤 퍽 위치 가정이 어긋나고 높게 쥔 대는 운반 중 손 안에서 돎 -> 45° 집기 (−2) 보다 3 배
        self.scene.robot = self.scene.robot.replace(init_state=self.scene.robot.init_state.replace(
            joint_pos={**self.scene.robot.init_state.joint_pos, "joint5": 2.75}))  # 시작 자세 기본값은 URDF 한계 (±2.8) 안이어야 함 -> IK 가 ±π 로 돌림


@configclass
class WaffleSidePick45BoxEnvCfg(WaffleSidePick45EnvCfg):
    """세운 직육면체만 45° 로 쥐기 (2026-11-04, A 다시 집기용): 누운 직육면체를 세워 상자 위 0.32 ~ 0.34 m 에 내려놓은 뒤 다시 집을 때
    45° 집기 정책이 넘어뜨림 (실물 조건 16 대 중 7 대) -> 직육면체만으로 이어 학습. 나머지 (금지 조건 · 마찰 · 관절 3 상한) 는 45° 집기와 같음."""

    def __post_init__(self):
        super().__post_init__()
        self.box_shape_prob = 1.0
        self.front_r_range = (0.32, 0.345)


READY_POSE_NEAR70 = {"joint1": 0.0, "joint2": -0.346, "joint3": 0.249, "joint4": 1.319, "joint5": 0.0}
"""2026-11-06 상자 앞 띠 70° 집기 시작 자세: 잡는 점 관절1 축 0.16 m · 퍽 중심 + 80 mm, 집게 70° 아래 (관절 2 · 3 · 4 = −20 · 14 · 76°)."""


@configclass
class WaffleSidePickNear70EnvCfg(WaffleSidePick45EnvCfg):
    """2026-11-06 정지 거리 5 ~ 6 cm 에서 상자 앞 약 6 cm 띠 (퍽 중심 관절1 축 0.12 ~ 0.18 m) 의 세운 퍽: 45° 로 쥐면 관절3 76 ~ 84° (팔꿈치가 차체 위로)
    -> 집게 70° 아래로 쥠 (관절3 32 ~ 55°, 팀 URDF FK). 쥔 뒤 (연결 실행) 상자 위 45° 집기 범위로 옮겨 내려놓고 45° 정책으로 다시 집음.
    관측 · 행동 · 보상은 45° 집기와 같음 (목표 피치만 70°) -> 45° 체크포인트에서 이어 학습."""

    def __post_init__(self):
        super().__post_init__()
        import math
        self.grasp_pitch = math.radians(70.0)
        self.dock_r_min, self.dock_r_max = 0.12, 0.20  # 두 퍽 모두 상자 앞 띠 (조금 겹치게 0.20 까지)
        self.scene.robot = self.scene.robot.replace(init_state=self.scene.robot.init_state.replace(
            joint_pos={**self.scene.robot.init_state.joint_pos, **READY_POSE_NEAR70}))
