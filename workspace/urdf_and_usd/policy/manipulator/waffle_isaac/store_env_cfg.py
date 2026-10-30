"""스토리지 넣기 `Waffle-Store-Direct` (Direct RL): 옆에서 쥔 퍽을 **높이 들어** 뒤 퍽 스토리지 통 (3 개 중 하나) 위 입구로 가져가 세운 채 넣기.

사용자 지시 (sim/정책_학습기록.md 4 절): 퍽을 쥔 채 1 번 모터 (Base_Yaw) 를 180° 가까이 돌리고 3·4 번 (팔꿈치·손목 피치) 을 굽혀 통 위 입구로 넣음.
퍽을 집은 뒤 팔을 더 들어 올려 (몸체·스토리지·카메라 가로대 위로) 옮김. **팔·손가락이 몸체 (base_link = 차체·스토리지·카메라 가로대) 에 닿으면 강한 벌점**.
시작: sim/isaac/store_starts.json (MuJoCo IK, `sim/mujoco/store_starts.py`): 집은 직후 45 %, 가는 도중 30 %, 통 위 25 % (역방향 커리큘럼).
목표 통: 시작 상태가 정한 통 또는 (집은 직후면) 무작위. 퍽은 원기둥만 (상자 퍽 30 x 30 은 대각선 42 mm 라 통 (지름 약 32) 에 안 들어감).

행동 6 = 팔 5 관절 목표 += 0.04 rad x 행동 (실제 각 ±0.25 rad 안), 집게 (> 0 벌림 0.015, ≤ 0 닫기).
관측 41 (base_link 좌표) = 팔 위치 (시작 기준 자세) 5 · 속도 5, 왼손가락 1, 잡는 점 3, 퍽 중심 − 잡는 점 3, 퍽 축 3, 목표 통 입구 (축 xy, 입구 z) 3,
          퍽 바닥 중심 − 통 입구 3, 집게 방향 3, 손가락 닫는 축 3, 목표 통 one-hot 3, 이전 행동 6.
          (퍽 자세는 시뮬레이션 참값 = 실제 로봇에는 아직 센서 없음, 4.10 절)
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

from .dice2_cfg import puck_cfg
from .robot_cfg import ARM_LINKS_REGEX, BODY_LINK, FINGER_CLOSE, FINGER_HOLD_30, FINGER_LINKS_REGEX, FINGER_OPEN_TASK, FINGER_RELEASED, FLOOR_MATERIAL, WAFFLE_CFG

START_POSE = {"joint1": 0.0, "joint2": 0.72, "joint3": 0.61, "joint4": -1.33, "joint5": 0.0}
"""관측의 기준 자세 (집기 SP3 가 끝나는 자세 근처). 실제 시작 자세는 store_starts.json."""

_robot = WAFFLE_CFG.replace(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=WAFFLE_CFG.spawn.replace(
        activate_contact_sensors=True,
        collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),  # 통 틈 1.2 mm: 기본 접촉 거리면 입구에서 걸림 (store_check)
        articulation_props=WAFFLE_CFG.spawn.articulation_props.replace(enabled_self_collisions=True),
    ),
    init_state=WAFFLE_CFG.init_state.replace(joint_pos={**WAFFLE_CFG.init_state.joint_pos, **START_POSE, "gripper_left_joint": FINGER_HOLD_30,
                                                        "gripper_right_joint": FINGER_HOLD_30}),
)
_puck = puck_cfg("", "cyl", (0.9, 0.1, 0.1))
_puck = _puck.replace(spawn=_puck.spawn.replace(activate_contact_sensors=True,
                                                 collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0)))
BODY = "{ENV_REGEX_NS}/Robot/" + BODY_LINK  # 261028: 루트 base_link 에 스토리지·가로대·센서·보드가 합쳐짐 (이전 base_footprint)


@configclass
class StoreSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg(physics_material=FLOOR_MATERIAL))
    robot: ArticulationCfg = _robot
    puck: RigidObjectCfg = _puck
    arm_body_contact = ContactSensorCfg(  # ST4 부터 팔 링크만 (손가락은 따로)
        prim_path="{ENV_REGEX_NS}/Robot/" + ARM_LINKS_REGEX, filter_prim_paths_expr=[BODY], update_period=0.0, history_length=1,
    )
    finger_body_contact = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/" + FINGER_LINKS_REGEX, filter_prim_paths_expr=[BODY], update_period=0.0, history_length=1,
    )
    puck_body_contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Puck", filter_prim_paths_expr=[BODY], update_period=0.0, history_length=1)
    light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.85, 0.85, 0.85)))


@configclass
class WaffleStoreEnvCfg(DirectRLEnvCfg):
    decimation = 4  # 30 Hz
    episode_length_s = 10.0
    action_space = 6
    observation_space = 41
    state_space = 0
    viewer: ViewerCfg = ViewerCfg(eye=(0.55, -0.75, 0.75), lookat=(-0.05, 0.0, 0.35), origin_type="env", env_index=0)
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation, physics=PhysxCfg(), physics_material=FLOOR_MATERIAL)
    scene: StoreSceneCfg = StoreSceneCfg(num_envs=256, env_spacing=1.5, replicate_physics=True, clone_in_fabric=True)

    action_scale = 0.04
    fine_scale = 0.25  # ST5: 정밀 모드 = 통 축 15 mm·입구 위 3 cm 안에서 행동 크기 x 0.25 (0.01 rad). ST4 까지 탐색 흔들림 약 5 mm > 통 틈 1 mm
    fine_xy = 0.015
    target_lead_max = 0.25
    gripper_open = FINGER_OPEN_TASK
    gripper_closed = FINGER_CLOSE
    grasp_dz = 0.015  # 잡는 점 = 퍽 중심 + 15 mm (261028 정렬·집기와 같음)
    start_cats = ("A", "H", "B", "C")  # 쓰는 시작 종류 (A 집은 직후 (MuJoCo IK), H 집기 모델이 실제로 끝난 상태 (store_handover.json), B 가는 도중, C 통 위)
    cat_weight = (0.05, 0.60, 0.15, 0.20)  # 팀 공유본 T1 0.10·0.45·0.20·0.25 -> T2: H (실제 집기 끝 상태, 352 개) 60 % (T1 은 이어 붙이기에서 N 통 축 18 mm·입구 위 17 mm 에 머묾)  # start_cats 순서 비율. ST1 0.45·0.30·0.25 -> ST2 0.35·0.25·0.40 (A·B·C) -> 261028 H 추가 (이어 받기 차이, 4.12 절)
    tubes_used = (1, 2)  # 0 C 가운데, 1 P +y, 2 N −y. **C 는 Isaac 에서 통 안 10 mm 에서 놓아도 14 mm 에 걸림 (store_check, 4.10 절)** -> 우선 P·N

    # 판정 (base_link 좌표, 통 축·입구 = store_starts.json)
    held_dist = 0.03  # 퍽 중심-잡는 점 이 안이면 쥔 것
    clear_margin = 0.012  # 퍽 바닥이 입구 + 12 mm 위면 '높이 듦'
    high_band = 0.10  # 입구 아래 10 cm 부터 다가가기 보상이 켜짐 (높이 들어야 다가가기 보상 다 받음)
    align_xy = 0.005  # 넣기 시작 (퍽 바닥 중심-통 축). ST2: 0.004 -> 0.005
    inside_xy = 0.006
    inserted_depth = 0.015  # 퍽 바닥이 입구 아래 15 mm = 통 안
    stored_depth = 0.06  # 놓은 퍽 바닥이 입구 아래 60 mm = 넣기 끝 (양옆 통은 296 mm 에서 좁아져 303 에 멈춤, 입구 아래 96)
    released_finger = FINGER_RELEASED  # 261028 (이전 0.010)
    upright_min = 0.98  # ST2: 0.99 -> 0.98 (쥔 퍽 직립 평균 0.96 ~ 0.99)
    success_hold_s = 0.5
    contact_force = 1.0  # [N] 접촉으로 봄
    crash_force = 15.0  # [N] 세게 부딪힘 -> 종료
    base_move_max = 0.01

    # 보상
    rew_hold = 0.5  # 쥐고 있음 (넣기 전)
    rew_lift = 2.0  # 퍽 바닥 높이 (입구 + 12 mm 까지, 연속)
    rew_approach = 1.5  # 퍽 바닥 중심 -> 통 축 (수평, 1 − tanh(d/0.15)), 높이 들었을 때만
    rew_approach_mid = 3.0  # (1 − tanh(d/0.03)) ST2 추가: ST1 은 통 6 ~ 8 cm 앞에서 멈춤 (4 cm 로 가도 +0.3 / 스텝뿐)
    rew_approach_fine = 3.0  # (1 − tanh(d/0.008)) ST2: 2 -> 3
    rew_hover = 2.0  # ST2 추가: 통 5 cm 안에서 퍽 바닥 = 입구 + 8 mm (1 − tanh(|높이 차|/0.01))
    hover_clear = 0.008
    rew_precise = 4.0  # ST3 추가: (1 − tanh(d/0.002)) 통 틈 한쪽 1 mm 라 2 ~ 5 mm 맞춤으로는 안 들어감 (ST2 진단: 2 ~ 5 mm 위에서 머묾)
    rew_upright = 2.0  # 퍽 세움 (쥐고 있을 때) = 2 x (1 − tanh((1 − 축 z)/0.005)): 2° -> 1.8, 5° -> 0.7, 16° -> 0. 261028 ST-N4: 이전 1 x 축 z 는 0.96 (16°) 과 0.99 를 거의 구분 못 해 맞춤 (직립 0.98) 을 못 넘음
    upright_scale = 0.005
    rew_level = 1.5  # 261028 ST-N5: 집게 수평 (1 − tanh(|집게 방향 z|/0.03)). N4 진단: 통 위에서 집게 피치 약 11°
    rew_yaw = 1.5  # 팀 공유본 T1: 관절 1 을 목표 통 방향 각으로 (1 − |q1 − 목표|/π). ST-N4·N5 는 목표 P 통이어도 늘 − 로 돌아 −π 한계에 막힘 (P 축 68 ~ 70 mm)
    joint1_x = 0.096
    lock_roll = True  # 261028 ST-N5: 손목 롤 (관절 5) 0 고정. N4 진단: 롤 12 ~ 19° (관절 5 = 0.22 ~ 0.34) 로 퍽이 옆으로 기욺 (퍽 17 ~ 22°). 이 과제엔 롤이 필요 없음
    rew_insert = 6.0  # 내린 깊이 (입구 + 10 mm -> 입구 − 30 mm) x (1 − tanh(d/0.003)) x 직립. ST3: 4 -> 6, 맞춤 (5 mm) 켜고 끄기 -> 연속
    rew_inserted = 5.0
    rew_released = 5.0
    rew_stored = 10.0
    rew_retract = 2.0  # 놓은 뒤 집게를 통에서 뺌
    rew_puck_tilt = -2.0  # (1 − |퍽 축 z|)
    rew_body_contact = -15.0  # **팔 링크 (2 ~ 5) 가 몸체에 닿은 스텝 (1 N 넘으면), 손가락은 넣는 자리 밖에서** - 사용자 요청: 강한 음의 보상
    rew_body_crash = -50.0  # 세게 부딪힘 (15 N 넘으면) + 종료
    rew_finger_zone = -1.0  # ST4: 손가락이 넣는 자리 (통 축 8 mm·입구 위 3 cm 안) 에서 통 위에 닿음 = 약하게 (넣으려면 피할 수 없음, store_hybrid 4 ~ 10 N)
    zone_xy = 0.008
    zone_clear = 0.03
    rew_puck_body = -3.0  # 퍽이 통 밖에서 몸체 (가로대·통 테두리 위·레일) 에 닿음
    rew_drop = -20.0  # 넣기 전에 퍽 놓침 + 종료
    rew_base_move = -10.0
    rew_action_rate = -0.02
    rew_joint_vel = -1e-3
    rew_base_tilt = -8.0
