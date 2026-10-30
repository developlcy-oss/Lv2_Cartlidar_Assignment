"""주사위 2 만들기 과제 공통: 배치·상자·퍽 설정 (집기 P2·놓기 P1·재생 확인이 같이 씀).

배치는 sim/정책_학습기록.md 3.6 절 (MuJoCo 로 도달·물리 확인). base_link 좌표 = 지도 좌표, 로봇은 원점에서 +x, 지도 +Y = 로봇 왼쪽.
상자는 kinematic 강체 (움직이지 않지만 환경마다 리셋 때 위치를 바꿀 수 있음).
"""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg

BOX_SIZE, BOX_H = 0.205, 0.070  # [m] 미션 작은 상자 205 x 205 x 70
A_BOX = (0.335, 0.110)  # A 상자 중심 (앞 왼쪽)
B_BOX = (0.255, -0.120)  # B 상자 중심 (앞 오른쪽)
A_PUCK_DX = -0.040  # 집는 A 퍽 = A 상자 중심 + (−0.04, 0) (로봇 쪽). 옆 퍽 A2 = +0.04 (팔이 안 닿음)
B_PUCK_D = (0.060, -0.060)  # B 퍽 = 주사위 2 오른쪽 아래 칸
TARGET_D = (-0.060, 0.060)  # 목표 칸 = 주사위 2 왼쪽 위 칸
PUCK_R, PUCK_H = 0.015, 0.060  # 원기둥 지름 30, 상자 30 x 30, 높이 60
PUCK_MASS = {"cyl": 0.045, "box": 0.055}  # [kg] 실측 전 추정
GRASP_OFFSET_A5 = (0.14, 0.0, 0.0)  # 잡는 점 = 집게 몸 (261028: gripper_roll_link, 이름은 이전 그대로) 좌표의 손가락 패드 가운데. robot_cfg.GRASP_OFFSET_EE 와 같음
GRIPPER_OPEN_TASK = 0.015  # 이 과제의 집게 벌림 (49.6 mm). 다 벌리면 손가락 바깥 폭 약 130 mm 라 옆 퍽을 침
PUCK_Z = BOX_H + PUCK_H / 2 + 0.001  # 상자 위 퍽 중심 높이 (월드) + 1 mm


def box_cfg(name: str, pos_xy) -> RigidObjectCfg:
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Box{name}",
        spawn=sim_utils.CuboidCfg(
            size=(BOX_SIZE, BOX_SIZE, BOX_H),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.85, 0.78, 0.6)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(pos_xy[0], pos_xy[1], BOX_H / 2)),
    )


def puck_cfg(name: str, shape: str, rgb, pos_xy=(0.0, 0.6)) -> RigidObjectCfg:
    common = dict(
        rigid_props=sim_utils.RigidBodyPropertiesCfg(solver_position_iteration_count=16, solver_velocity_iteration_count=1,
                                                     max_depenetration_velocity=1.0),
        mass_props=sim_utils.MassPropertiesCfg(mass=PUCK_MASS[shape]),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=rgb),
    )
    spawn = (sim_utils.CylinderCfg(radius=PUCK_R, height=PUCK_H, axis="Z", **common) if shape == "cyl"
             else sim_utils.CuboidCfg(size=(2 * PUCK_R, 2 * PUCK_R, PUCK_H), **common))
    return RigidObjectCfg(prim_path=f"{{ENV_REGEX_NS}}/Puck{name}", spawn=spawn,
                          init_state=RigidObjectCfg.InitialStateCfg(pos=(pos_xy[0], pos_xy[1], PUCK_Z)))
