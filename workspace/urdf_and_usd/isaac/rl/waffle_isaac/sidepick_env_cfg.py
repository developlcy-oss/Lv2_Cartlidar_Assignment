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
    box_shape_prob = 0.5
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
    rew_neighbor_moved = -20.0
    rew_toppled = -2.0  # 261028 N10: −0.5 -> −2 (N9 실제 과제 실패 대부분이 넘어뜨림 24/64)
    rew_base_tilt = -8.0
    rew_fall = -5.0
