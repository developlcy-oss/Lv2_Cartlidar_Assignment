#!/usr/bin/env python3
"""받은 패키지 waffle_4wd_description_261027_1711 에서 이 패키지 (waffle_4wd_description_261028) 를 만든다.
받은 패키지와 arm_bench 는 읽기만 한다. 바뀐 내용 · 가정은 REVISION_261028.md.

    freecadcmd -c "exec(open('tools/make_part_meshes.py').read())"   # 센서 · 부품 메시 (치수 · 브라켓 · STEP 바꿀 때만)
    python3 tools/make_revision.py [--src 받은 패키지 폴더] [--arm arm_bench.urdf]

1. 팔 (2026-10-28 사용자 결정): RMX52_* 팔 (arm_link_1~5 · 손가락 · camera_link) 을 빼고 arm_bench.urdf 의 link2 아래를 붙인다.
   - J1 서보 (arm_bench link1) 는 넣지 않는다. 차체 메시의 J1 서보가 같은 크기 (46.4 x 28.4 x 40.4 mm) 라 그대로 쓴다.
   - 이름은 팀 Lv2 기준 (RENAME). 손가락 2 개와 그리퍼 모터 각 (gripper_joint) 은 따로 움직인다 (mimic 없음, arm_bench 그대로).
2. J1 브라켓 + 서보 + 나사 (차체 메시 덩어리 19 개, 충돌 상자 3 개) 를 앞으로 9 mm.
   브라켓 밑 구멍 4 개 (x 64 · 88, y ±29.85 mm) 가 상판 (TB3 와플판) 구멍 (73 · 97, ±30) 과 맞는 자리.
3. 옛 LiDAR (lidar_front/rear 링크 + 상판 밑 3 mm 브라켓 판 2 장 + 충돌 상자 2 개) 를 빼고 LDS-03 2 개를 상판 밑면에 뒤집어 단다.
   앞 · 뒤 가운데 (y 0), 몸체 바깥 면 = 상판 끝 (임시 - 귀 구멍 실측 뒤 다시). 이름은 팀 Lv2 기준 (lds_front · laser_front).
4. TFmini Plus + 사용자 브라켓 v2 (~/Desktop/CartRider/tfmini_plus_bracket_v2_print.stl) 를 하판 윗면 앞 가운데에 (테이프 · 케이블 타이 고정).
   벽이 앞, TFmini 는 벽 바깥 면에 (앞을 봄), 바닥판 · 받침 턱은 벽 뒤. 벽 뒷면 = 지금 OpenCR 앞 끝 + 1 mm. 측정 frame = tfmini_link (렌즈 가운데, +x 앞).
5. Jetson Orin Nano 개발자 키트: 차체 메시의 자리 상자 (103 x 90.5 x 34.8) 와 그 충돌 상자를 빼고 STEP 모델 링크 (jetson_link) 로.
   입출력 단자는 로봇 뒤 (사용자 2026-10-28).
6. 하판 배치 B안 (사용자 2026-10-28): OpenCR 2 장 · 배터리 2 개를 차체 메시에서 떼어 링크로 만들고, 모두 하판 위에 바로 (테이프 · 케이블 타이).
   Jetson 뒷면 = 판 뒤 끝 + 2 mm, OpenCR = Jetson 바로 앞, 배터리 = Jetson 양옆, TFmini 브라켓 = OpenCR 바로 앞 (사이 1 mm). layout() 참고.
7. 뜬 부품 지우기: 상판 · 하판 사이에 아무것에도 닿지 않고 떠 있던 기판 (USB-C + 8 핀, 30 x 18 x 7.7 mm) 과 그 충돌 상자,
   삼각형 2 개 이하 조각 35 개 (보관함 레일 근처, 부피 없음).
8. Gazebo: config/sensors.yaml (센서 사양 · 시뮬 값) 로 <gazebo> 센서 · 구동 태그와 config/gz_bridge.yaml 을 만든다 (gazebo_ext.py).
9. 퍽 보관 (사용자 2026-10-30): 차체 메시의 퍽 보관통 3 개 (통마다 위 · 아래 2 덩어리) 와 보관함 레일 2 개, 그 충돌 볼록 메시 묶음 8 개를 빼고
   퍽 플레이트 (meshes/puck_pallet.stl -> tools/make_puck_plate.py -> meshes/parts/puck_plate.*) 를 링크 puck_plate_link 로 단다.
   플레이트 원점 = 팔 joint1 축 (x, y) · 상판 윗면 (z 0.111), 회전 없음. 컵 12 개 = 팔 축 둘레 반지름 90 · 142 · 191 mm.
   충돌: collision:=full 은 볼록 조각 (바닥판 + 컵 면별), light 는 바닥판 상자 + 컵마다 꽉 찬 상자.
"""
import argparse
import hashlib
import json
import re
import shutil
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import numpy as np
from scipy.sparse import coo_matrix

import gazebo_ext
from scipy.sparse.csgraph import connected_components

PKG = 'waffle_4wd_description_261028'
SRC_PKG = 'waffle_4wd_description_261027_1711'
HERE = Path(__file__).resolve().parents[1]
DEFAULT_SRC = Path.home() / 'Downloads/urdf_261027_1711' / SRC_PKG
DEFAULT_ARM = Path.home() / 'GIT/lv2_CartLidar/arm_bench/urdf/arm_bench.urdf'   # 2026-10-29 ~/GIT 정리로 옮겨짐
GENERATED = ('meshes/main', 'meshes/collision', 'meshes/arm')   # 매번 지우고 다시 만드는 폴더 (meshes/parts 는 FreeCAD 스크립트 몫)

J1_SHIFT = np.array([0.009, 0.0, 0.0])   # [m] 앞 (+x)
# J1 묶음 = 외곽 상자가 이 범위 [m] 안에 다 들어오는 차체 메시 덩어리 (상판 윗면 z 0.111 위, 팔 장착 자리 주변)
J1_REGION = ((0.040, 0.115), (-0.040, 0.040), (0.1109, 0.170))
J1_PARTS = 19                            # 서보 1 + 브라켓 2 + 나사 16 (2026-10-28 분석)
J1_BOXES = 3                             # 충돌 상자: 브라켓 2 (static_431 · 432) + 서보 1 (static_435)
MOUNT_Z = 0.017                          # arm_bench: J1 서보 바닥 -> link1 원점 (= arm_base_link 높이)

# 상판 (TB3 와플판, base_link 메시): 밑면 z 0.102, 윗면 0.111, 앞 · 뒤 곧은 끝 x ±0.128 (y 가운데)
PLATE_BOTTOM_Z = 0.102
PLATE_EDGE_X = 0.128
# 옛 LiDAR 브라켓 판 = 상판 덩어리 안의 삼각형 중 이 범위 (앞; 뒤는 x · y 부호 반대) [m]. 밑면 z 0.099~0.102, 53 x 79 x 3 mm
OLD_BRACKET = dict(x=(0.0919, 0.1451), y=(0.0059, 0.0851), z=(0.0989, 0.10201))
OLD_BRACKET_TRIS = 1608                  # 앞 + 뒤 (2026-10-28 분석)
OLD_LIDAR_LINKS = {'lidar_front', 'lidar_rear', 'lidar_front_scan', 'lidar_rear_scan'}

PARTS = HERE / 'meshes/parts'           # tools/make_part_meshes.py (FreeCAD) 결과
LDS_JSON = PARTS / 'lds03.json'
# 임시 자리: 몸체 바깥 면 (+x 19.1 mm) = 상판 곧은 끝. 귀 구멍 3 개가 판 구멍과 동시에 맞는 자리가 f3d 치수로는 없어서 실측 뒤 다시 정한다
LDS_YAW = {'front': 0.0, 'rear': np.pi}  # 뒤집은 (roll π) 뒤 z 축 회전. 한쪽 귀 (-x) 가 차체 안쪽

# 하판 (TB3 와플판): 윗면 z 0.012. 앞 가운데에 반원 홈 (판 이음매), 가장 안쪽 판 끝 x 0.1075 (y ±2, 2026-10-28 메시에서 잼)
LOW_PLATE_TOP_Z = 0.012
SIDE_WALL_Y = 0.091                      # 옆벽 안쪽 면 |y| (x 0.0953 뒤쪽)
# 요청 6 하판 배치 B안 (사용자 2026-10-28): 모두 판 위에 바로 (테이프 · 케이블 타이, 추가 브라켓 없음), 부품 사이 LAYOUT_GAP.
#   Jetson 뒷면 = 판 뒤 끝 + JETSON_REAR_INSET, OpenCR 2 장 = Jetson 바로 앞 (y 는 받은 자리 그대로),
#   배터리 = Jetson 양옆 (앞 끝 = Jetson 앞 끝), TFmini 브라켓 바닥판 뒤 끝 = OpenCR 앞 끝 + LAYOUT_GAP (OpenCR 이 판 위라 그 밑으로 못 들어감)
LAYOUT_GAP = 0.001
JETSON_REAR_INSET = 0.002
# 받은 차체 메시에서 떼어 링크로 만드는 것 (외곽이 이 범위 [m] 안인 덩어리): OpenCR 2 장 (보드 2 + 작은 부품 2), 배터리 2 개 (상자, 삼각형 12)
OPENCR_REGION = ((0.0030, 0.1130), (-0.0790, 0.0790), (0.0169, 0.0361))
OPENCR_PARTS = 4
BATTERY_REGION = ((-0.1045, -0.0155), (-0.0850, 0.0850), (0.0129, 0.0391))

# Jetson: 받은 차체 메시의 자리 상자 (삼각형 12 개, 외곽이 이 범위 [m] 안) 와 같은 크기 충돌 상자 (static_440) 를 빼고 jetson_link 로
JETSON_PLACEHOLDER = ((-0.1120, -0.0080), (-0.0460, 0.0460), (0.0125, 0.0485))

# 무게중심 · 관성 다시 계산용 질량 가정 (REVISION_261028.md)
JETSON_KG_IN_BASE = 0.176                # 받은 base_link 질량에 자리 상자 몫으로 들어 있다고 본 Jetson 질량
OPENCR_KG, BATTERY_KG = 0.060, 0.106     # OpenCR e-Manual 60 g, LB-012 판매 페이지 106 g (1안 build_rover.py 기록). 받은 base_link 에 들어 있다고 봄
FR4_KG_M3 = 1850.0                       # 뜬 기판 질량 = 메시 부피 x FR-4 밀도 (가정)

# 요청 7: 뜬 기판 (외곽이 이 범위 [m] 안인 덩어리 하나, 상판 밑 14.5 mm 에 떠 있음) 과 조각 (삼각형 DEBRIS_MAX_TRIS 개 이하 덩어리)
FLOAT_BOARD_REGION = ((0.0340, 0.0660), (-0.0080, 0.0120), (0.0767, 0.0846))
DEBRIS_MAX_TRIS, DEBRIS_COUNT = 2, 35
# 요청 9 (사용자 2026-10-30): 퍽 보관통 · 보관함 레일 = 외곽이 이 범위 [m] 안인 덩어리 (삼각형 DEBRIS_MAX_TRIS 개 넘는 것)
STORAGE_REGIONS = (((-0.1225, -0.0545), (-0.1015, 0.1015), (0.1785, 0.4065)),    # 통 3 개 (z 0.18 ~ 0.40, 입구 위 테두리 포함)
                   ((-0.1340, -0.0430), (-0.1420, 0.1420), (0.1740, 0.2060)),    # 아래 레일
                   ((-0.1340, -0.0430), (-0.1420, 0.1420), (0.2840, 0.3160)))    # 위 레일
STORAGE_PARTS = 8                        # 통 3 x (위 · 아래) + 레일 2 (2026-10-30 분석)
STORAGE_HULL_GROUPS = ('base_link_part022', 'base_link_part025', 'base_link_part026', 'base_link_part029',
                       'base_link_part140', 'base_link_part141', 'base_link_rail_hi_new', 'base_link_rail_lo_new')
STORAGE_HULLS = 295                      # 위 묶음의 볼록 메시 수
PLATE_TOP_Z = 0.111                      # 상판 윗면 = 퍽 플레이트 밑면
PUCK_PLATE_SRC = HERE / 'meshes/puck_pallet.stl'
SERVO_KG = 0.094011                      # arm_bench link1 (J1 서보, ROBOTIS 표)
PLA_KG_M3, PLA_FILL = 1240.0, 0.55       # 출력 부품: PLA 밀도 x 채움 비율 (받은 패키지 REVISION_261027_1711 의 긴 브라켓 값)
STEEL_KG_M3 = 7850.0                     # 나사

OLD_ARM_LINKS = {'arm_link_1', 'arm_link_2', 'arm_link_3', 'arm_link_4', 'arm_link_5',
                 'gripper_left_link', 'gripper_right_link', 'camera_link', 'camera_optical_frame'}
ARM_SKIP_LINKS = {'arm_base', 'link1', 'arm_base_link'}           # arm_base_link 는 base_link 아래에 새로 만든다
# 팀 dev (controllers_4wd.yaml · opencr.ros2_control.xacro · robot.urdf.xacro) 이름 · 구조로 (사용자 2026-10-28 "이름 맞춤 4 개")
WHEEL_RENAME = {                                                 # 받은 바퀴 관절 -> (팀 관절, 팀 링크)
    'Wheel_Revolute_Front_Left': ('wheel_left_front_joint', 'wheel_left_front_link'),
    'Wheel_Revolute_Rear_Left': ('wheel_left_rear_joint', 'wheel_left_rear_link'),
    'Wheel_Revolute_Front_Right': ('wheel_right_front_joint', 'wheel_right_front_link'),
    'Wheel_Revolute_Rear_Right': ('wheel_right_rear_joint', 'wheel_right_rear_link'),
}
ROOT_JOINT = ('base_footprint_to_base_link', 'base_footprint_joint')   # 루트를 base_link 로 (팀 EKF 가 odom -> base_link 를 냄)
ARM_SKIP_JOINTS = {'arm_mount_joint', 'arm_base_link_joint'}
RENAME = {                                                       # arm_bench -> 팀 Lv2 이름
    'wrist_roll_joint': 'joint5',
    'gripper_motor_joint': 'gripper_roll',                       # 팀 dev arm_controller · ros2_control 칸 5 (그리퍼 개폐 모터 각)
    'gripper_link': 'gripper_roll_link',
    'gripper_camera_link': 'arm_camera_link',
    'gripper_camera_joint': 'arm_camera_joint',
    'gripper_camera_optical_frame': 'arm_camera_optical_frame',
    'gripper_camera_optical_joint': 'arm_camera_optical_joint',
    'end_effector_link': 'tool_link',
    'end_effector_joint': 'tool_joint',
}


def num(v):
    return f'{float(v):.9g}'


def vec(v):
    return ' '.join(num(x) for x in v)


def xyz_of(el):
    o = el.find('origin')
    return np.array([float(s) for s in o.get('xyz', '0 0 0').split()]) if o is not None else np.zeros(3)


def pa(m, r):
    """점 질량 m 이 r 에 있을 때 원점 기준 관성 (평행축)."""
    return m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))


def in_box(p, box):
    return all(box[i][0] <= p[..., i].min() and p[..., i].max() <= box[i][1] for i in range(3))


# ---------- 차체 메시 (base_link.dae) ----------

def read_dae(path):
    txt = path.read_text(encoding='utf-8')
    pos = re.search(r'<float_array count="\d+" id="verts-array-array">([^<]*)</float_array>', txt)
    V = np.array(pos.group(1).split(), dtype=float).reshape(-1, 3)
    tri = re.search(r'<triangles count="(\d+)"[^>]*>.*?<p>([^<]*)</p>', txt, re.S)
    n_in = len(set(re.findall(r'offset="(\d+)"', tri.group(0))))
    p = np.array(tri.group(2).split(), dtype=np.int64).reshape(-1, 3 * n_in)
    return dict(txt=txt, pos=pos, tri=tri, V=V, p=p, F=p.reshape(-1, 3, n_in)[:, :, 0])


def vertex_parts(V, F):
    """같은 자리 꼭짓점 (0.01 mm) 을 합쳐 이어진 덩어리 번호를 원래 꼭짓점마다 돌려준다."""
    key = np.round(V * 1e5).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    n = inv.max() + 1
    e = np.concatenate([inv[F][:, [0, 1]], inv[F][:, [1, 2]]])
    _, lab = connected_components(coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(n, n)), directed=False)
    return lab[inv]


def solid_props(T):
    """닫힌 삼각형 묶음 (n,3,3) 의 부피 (절댓값) 와 무게중심."""
    vol6 = np.einsum('ij,ij->i', T[:, 0], np.cross(T[:, 1], T[:, 2]))
    return abs(vol6.sum()) / 6, (vol6[:, None] * T.sum(1)).sum(0) / (4 * vol6.sum())


def edit_base_mesh(src_dae, out_dae):
    """J1 묶음을 J1_SHIFT 만큼 옮기고 옛 LiDAR 브라켓 판 · Jetson 자리 상자 · 퍽 보관통 · 레일을 지우고 OpenCR · 배터리를 떼어 내 쓴다.
    돌려주는 값: 옮긴 덩어리 [(kg, 무게중심)], 지운 것 [(kg, 무게중심)], J1 서보 바닥 중심 (옮기기 전),
    떼어 낸 것 {이름: 삼각형 (n,3,3) · 외곽 lo/hi · kg}."""
    d = read_dae(src_dae)
    V, F = d['V'], d['F']
    part = vertex_parts(V, F)
    tri_part = part[F[:, 0]]
    labs = np.unique(tri_part)

    # J1 묶음 (서보 · 브라켓 · 나사)
    j1 = [k for k in labs if in_box(V[F[tri_part == k]], J1_REGION)]
    assert len(j1) == J1_PARTS, f'J1 묶음 덩어리 {len(j1)} 개 (기대 {J1_PARTS}) - 받은 메시가 바뀌었는지 확인'
    props = [(k,) + solid_props(V[F[tri_part == k]]) for k in j1]
    servo = max(props, key=lambda p: p[1])[0]
    moved_parts = []
    for k, vol, cen in props:
        m = SERVO_KG if k == servo else vol * (PLA_KG_M3 * PLA_FILL if vol > 1e-6 else STEEL_KG_M3)   # 1 cm3 넘으면 브라켓
        moved_parts.append((m, cen))
    sv = V[F[tri_part == servo]].reshape(-1, 3)
    servo_bottom = np.array([(sv[:, 0].min() + sv[:, 0].max()) / 2, (sv[:, 1].min() + sv[:, 1].max()) / 2, sv[:, 2].min()])

    # Jetson 자리 상자 (덩어리 하나, 삼각형 12 개)
    ph = [k for k in labs if in_box(V[F[tri_part == k]], JETSON_PLACEHOLDER)]
    assert len(ph) == 1 and (tri_part == ph[0]).sum() == 12, f'Jetson 자리 상자 덩어리 {ph}'
    pv = V[F[tri_part == ph[0]]].reshape(-1, 3)
    drop_ph = tri_part == ph[0]

    # 옛 LiDAR 브라켓 판: 상판 덩어리 (z 0.102~0.111 삼각형이 가장 많은 덩어리) 안, 판 밑 + 판 밑면과 같은 높이의 위 향 면
    T = V[F]
    cen = T.mean(1)
    plate = np.bincount(tri_part[(cen[:, 2] > PLATE_BOTTOM_Z) & (cen[:, 2] < PLATE_BOTTOM_Z + 0.009)]).argmax()
    nz = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])[:, 2]
    removed_parts, drop = [], np.zeros(len(F), bool)
    for s in (1, -1):
        b = OLD_BRACKET
        m = ((tri_part == plate) & (b['x'][0] <= s * cen[:, 0]) & (s * cen[:, 0] <= b['x'][1])
             & (b['y'][0] <= s * cen[:, 1]) & (s * cen[:, 1] <= b['y'][1]) & (b['z'][0] <= cen[:, 2])
             & ((cen[:, 2] < PLATE_BOTTOM_Z - 1e-5) | ((np.abs(cen[:, 2] - PLATE_BOTTOM_Z) < 1e-5) & (nz > 0))))
        drop |= m
        top = m & (np.abs(cen[:, 2] - PLATE_BOTTOM_Z) < 1e-5)            # 판 윗면 (위 향) 넓이 x 두께 3 mm = 부피 (닫혀 있지 않아 근사)
        area = np.linalg.norm(np.cross(T[top, 1] - T[top, 0], T[top, 2] - T[top, 0]), axis=1) / 2
        c_xy = (area[:, None] * cen[top, :2]).sum(0) / area.sum()
        removed_parts.append((area.sum() * 0.003 * PLA_KG_M3 * PLA_FILL, np.array([c_xy[0], c_xy[1], PLATE_BOTTOM_Z - 0.0015])))
    assert drop.sum() == OLD_BRACKET_TRIS, f'옛 LiDAR 브라켓 판 삼각형 {drop.sum()} 개 (기대 {OLD_BRACKET_TRIS})'
    drop |= drop_ph
    removed_parts.append((JETSON_KG_IN_BASE, (pv.min(0) + pv.max(0)) / 2))

    # OpenCR 2 장 · 배터리 2 개: 떼어 내 링크로 (y 부호로 왼쪽 · 오른쪽)
    oc = [k for k in labs if in_box(V[F[tri_part == k]], OPENCR_REGION)]
    bt = [k for k in labs if in_box(V[F[tri_part == k]], BATTERY_REGION)]
    assert len(oc) == OPENCR_PARTS, f'OpenCR 덩어리 {len(oc)} 개 (기대 {OPENCR_PARTS})'
    assert len(bt) == 2 and all((tri_part == k).sum() == 12 for k in bt), f'배터리 덩어리 {bt}'
    extracted = {}
    for side, sg in (('left', 1), ('right', -1)):
        for name, ks, kg in (('opencr', oc, OPENCR_KG), ('battery', bt, BATTERY_KG)):
            m = np.isin(tri_part, [k for k in ks if sg * V[F[tri_part == k]][..., 1].mean() > 0])
            P = V[F[m]]
            lo, hi = P.reshape(-1, 3).min(0), P.reshape(-1, 3).max(0)
            extracted[f'{name}_{side}'] = dict(tris=P, lo=lo, hi=hi, kg=kg)
            drop |= m
            removed_parts.append((kg, (lo + hi) / 2))

    # 뜬 기판 · 조각 지우기
    fb = [k for k in labs if in_box(V[F[tri_part == k]], FLOAT_BOARD_REGION)]
    assert len(fb) == 1, f'뜬 기판 덩어리 {fb}'
    m = tri_part == fb[0]
    vol, c_fb = solid_props(V[F[m]])
    drop |= m
    removed_parts.append((vol * FR4_KG_M3, c_fb))
    counts = np.bincount(tri_part)
    small = np.flatnonzero((counts > 0) & (counts <= DEBRIS_MAX_TRIS))
    assert len(small) == DEBRIS_COUNT, f'조각 {len(small)} 개 (기대 {DEBRIS_COUNT})'
    drop |= np.isin(tri_part, small)
    print(f'뜬 기판 (삼각형 {m.sum()}, 부피 {vol * 1e6:.2f} cm3 -> {vol * FR4_KG_M3 * 1000:.1f} g) · 조각 {len(small)} 개 지움')

    # 요청 9: 퍽 보관통 · 보관함 레일 (덩어리마다 외곽 상자로 판정, 출력 부품 질량)
    lo = np.full((labs.max() + 1, 3), np.inf)
    hi = np.full((labs.max() + 1, 3), -np.inf)
    np.minimum.at(lo, tri_part, T.min(1))
    np.maximum.at(hi, tri_part, T.max(1))
    st = [k for k in labs if counts[k] > DEBRIS_MAX_TRIS
          and any(all(R[i][0] <= lo[k, i] and hi[k, i] <= R[i][1] for i in range(3)) for R in STORAGE_REGIONS)]
    assert len(st) == STORAGE_PARTS, f'퍽 보관통 · 레일 덩어리 {len(st)} 개 (기대 {STORAGE_PARTS})'
    st_kg = 0.0
    for k in st:
        vol, c = solid_props(V[F[tri_part == k]])
        removed_parts.append((vol * PLA_KG_M3 * PLA_FILL, c))
        st_kg += vol * PLA_KG_M3 * PLA_FILL
    m = np.isin(tri_part, st)
    drop |= m
    print(f'퍽 보관통 3 · 레일 2 (덩어리 {len(st)} 개, 삼각형 {m.sum()}, {st_kg * 1000:.1f} g) 지움')

    moved = np.isin(part, j1)
    V2 = V.copy()
    V2[moved] = np.round(V2[moved] + J1_SHIFT, 9)
    keep = d['p'][~drop]
    txt, pos, tri = d['txt'], d['pos'], d['tri']
    out = (txt[:pos.start(1)] + ' '.join(repr(v) for v in V2.ravel().tolist()) + txt[pos.end(1):tri.start(1)]
           + str(len(keep)) + txt[tri.end(1):tri.start(2)] + ' '.join(map(str, keep.ravel().tolist())) + txt[tri.end(2):])
    out_dae.write_text(out, encoding='utf-8')

    # 다시 읽어 확인: 옮긴 꼭짓점 · 지운 삼각형 말고는 원본과 같은지
    e = read_dae(out_dae)
    assert np.array_equal(e['V'][~moved], V[~moved]) and np.allclose(e['V'][moved] - V[moved], J1_SHIFT, atol=1e-9)
    assert np.array_equal(e['p'], keep)
    print(f'base_link.dae: 덩어리 {len(labs)} 개 중 J1 묶음 {len(j1)} 개 (꼭짓점 {moved.sum()}) x +{J1_SHIFT[0] * 1000:.0f} mm, '
          f'옛 LiDAR 브라켓 판 · Jetson 자리 상자 · 뜬 기판 · 조각 · 퍽 보관통 · 레일 지움, OpenCR 2 · 배터리 2 떼어 냄 (삼각형 {len(F)} -> {len(keep)})')
    return moved_parts, removed_parts, servo_bottom, extracted


def update_inertial(inertial, moves=(), removes=()):
    """moves: [(kg, 무게중심, 이동)], removes: [(kg, 무게중심)] 만큼 질량 · 무게중심 · 관성을 고친다."""
    M = float(inertial.find('mass').get('value'))
    C = xyz_of(inertial)
    a = inertial.find('inertia').attrib
    I = np.array([[float(a['ixx']), float(a['ixy']), float(a['ixz'])],
                  [float(a['ixy']), float(a['iyy']), float(a['iyz'])],
                  [float(a['ixz']), float(a['iyz']), float(a['izz'])]])
    I_o, MC = I + pa(M, C), M * C
    for m, c, dd in moves:
        I_o += pa(m, c + dd) - pa(m, c)
        MC = MC + m * dd
    M2 = M
    for m, c in removes:
        I_o -= pa(m, c)
        MC, M2 = MC - m * c, M2 - m
    C2 = MC / M2
    I2 = I_o - pa(M2, C2)
    inertial.find('mass').set('value', num(M2))
    inertial.find('origin').set('xyz', vec(C2))
    for key, (i, j) in {'ixx': (0, 0), 'ixy': (0, 1), 'ixz': (0, 2), 'iyy': (1, 1), 'iyz': (1, 2), 'izz': (2, 2)}.items():
        inertial.find('inertia').set(key, num(I2[i, j]))
    print(f'base_link 질량 {M * 1000:.1f} -> {M2 * 1000:.1f} g, 무게중심 {np.round(C * 1000, 2)} -> {np.round(C2 * 1000, 2)} mm')


# ---------- URDF ----------

FOOTPRINT_Z = 0.0067499816754           # base_footprint -> base_link (받은 URDF)
EXTRA_MATERIALS = {'lds03_dark': '0.12 0.12 0.13 1', 'tfmini_dark': '0.10 0.10 0.10 1', 'pla_print': '0.90 0.90 0.88 1',
                   'jetson': '0.20 0.24 0.22 1', 'opencr': '0.15 0.15 0.15 1', 'battery': '0.30 0.30 0.35 1',
                   'puck_plate': '0.95 0.60 0.15 1'}


def sensor_link(name, info, mesh, material, collisions):
    """질량 특성 (make_part_meshes.py JSON) + 겉모양 메시 + 충돌 [(이름, xyz, 모양, 속성)] 으로 링크를 만든다."""
    lk = ET.Element('link', name=name)
    inertial = ET.SubElement(lk, 'inertial')
    ET.SubElement(inertial, 'origin', xyz=vec(info['com_m']), rpy='0 0 0')
    ET.SubElement(inertial, 'mass', value=num(info['mass_kg']))
    ET.SubElement(inertial, 'inertia', **{k: num(v) for k, v in info['inertia_kg_m2'].items()})
    vis = ET.SubElement(lk, 'visual')
    ET.SubElement(vis, 'origin', xyz='0 0 0', rpy='0 0 0')
    ET.SubElement(ET.SubElement(vis, 'geometry'), 'mesh', filename=f'package://{PKG}/meshes/parts/{mesh}.stl')
    ET.SubElement(vis, 'material', name=material)
    for cname, xyz, geom, attr in collisions:
        col = ET.SubElement(lk, 'collision', name=cname)
        ET.SubElement(col, 'origin', xyz=vec(xyz), rpy='0 0 0')
        ET.SubElement(ET.SubElement(col, 'geometry'), geom, **attr)
    return lk


def fixed_joint(name, parent, child, xyz, rpy=(0, 0, 0)):
    j = ET.Element('joint', name=name, type='fixed')
    ET.SubElement(j, 'origin', xyz=vec(xyz), rpy=vec(rpy))
    ET.SubElement(j, 'parent', link=parent)
    ET.SubElement(j, 'child', link=child)
    return j


def lds_links(side, lds):
    """LDS-03 몸체 링크 (lds_<side>) + 스캔 frame (laser_<side>) 과 고정 관절."""
    s = 1 if side == 'front' else -1
    origin = np.array([s * (PLATE_EDGE_X - lds['bbox_mm'][1] / 1000), 0.0, PLATE_BOTTOM_Z + lds['mount_face_z_mm'] / 1000])
    bw, _, bz0, bz1 = lds['body_box_mm']
    hr, hz0, hz1 = lds['head_mm']
    body = sensor_link(f'lds_{side}', lds, 'lds03', 'lds03_dark', [
        (f'lds_{side}_body', (0, 0, (bz0 + bz1) / 2000), 'box', dict(size=vec((bw / 1000, bw / 1000, (bz1 - bz0) / 1000)))),
        (f'lds_{side}_head', (0, 0, (hz0 + hz1) / 2000), 'cylinder', dict(radius=num(hr / 1000), length=num((hz1 - hz0) / 1000)))])
    joints = [fixed_joint(f'lds_{side}_joint', 'base_link', f'lds_{side}', origin, (np.pi, 0, LDS_YAW[side])),
              fixed_joint(f'laser_{side}_joint', f'lds_{side}', f'laser_{side}', (0, 0, 0))]
    print(f'LDS-03 {side}: 광학 중심 (스캔면) base_link {np.round(origin * 1000, 1)} mm, 바닥 위 {(origin[2] + FOOTPRINT_Z) * 1000:.1f} mm')
    return [body, ET.Element('link', name=f'laser_{side}')], joints


def box_collisions(prefix, info):
    """JSON 의 collision_boxes_mm [{name, min, max}] -> sensor_link 충돌 목록."""
    return [(f'{prefix}_{b["name"]}', (np.array(b['min']) + b['max']) / 2000, 'box',
             dict(size=vec((np.array(b['max']) - b['min']) / 1000))) for b in info['collision_boxes_mm']]


def write_stl(path, T):
    """삼각형 (n,3,3) [m] 을 이진 STL 로."""
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-20
    rec = np.zeros(len(T), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['n'], rec['v'] = n, T
    path.write_bytes(b'make_revision.py'.ljust(80, b' ') + np.uint32(len(T)).tobytes() + rec.tobytes())


def layout(ext, jetson, bracket):
    """하판 배치 B안: 링크 이름 -> 원점 [m] (각 링크 밑면 가운데, TFmini 브라켓은 벽 바깥 면 가운데 밑면)."""
    jb = np.array(jetson['bbox_mm']) / 1000
    jx = -PLATE_EDGE_X + JETSON_REAR_INSET - jb[0]
    j_front = jx + jb[1]
    pos = {'jetson_link': np.array([jx, 0.0, LOW_PLATE_TOP_Z])}
    oc_front = -np.inf
    for side, sg in (('left', 1), ('right', -1)):
        o, b = ext[f'opencr_{side}'], ext[f'battery_{side}']
        ol, bl, bw = o['hi'][0] - o['lo'][0], b['hi'][0] - b['lo'][0], b['hi'][1] - b['lo'][1]
        pos[f'opencr_{side}_link'] = np.array([j_front + LAYOUT_GAP + ol / 2, (o['lo'][1] + o['hi'][1]) / 2, LOW_PLATE_TOP_Z])
        pos[f'battery_{side}_link'] = np.array([j_front - bl / 2, sg * (jb[3] + LAYOUT_GAP + bw / 2), LOW_PLATE_TOP_Z])
        oc_front = max(oc_front, pos[f'opencr_{side}_link'][0] + ol / 2)
        assert jb[3] + LAYOUT_GAP + bw <= SIDE_WALL_Y - LAYOUT_GAP, '배터리가 옆벽에 닿음'
    pos['tfmini_bracket'] = np.array([oc_front + LAYOUT_GAP - np.array(bracket['bbox_mm'])[0] / 1000, 0.0, LOW_PLATE_TOP_Z])
    return pos


def box_link(name, ext, material, out):
    """떼어 낸 부품 링크: 원점 = 밑면 가운데, 질량 = 외곽 상자에 고르게, 충돌 = 외곽 상자. 겉모양 = 떼어 낸 메시 (상자 12 삼각형이면 상자)."""
    size = ext['hi'] - ext['lo']
    org = np.array([(ext['lo'][0] + ext['hi'][0]) / 2, (ext['lo'][1] + ext['hi'][1]) / 2, ext['lo'][2]])
    kg, (sx, sy, sz) = ext['kg'], size
    lk = ET.Element('link', name=name)
    inertial = ET.SubElement(lk, 'inertial')
    ET.SubElement(inertial, 'origin', xyz=vec((0, 0, sz / 2)), rpy='0 0 0')
    ET.SubElement(inertial, 'mass', value=num(kg))
    ET.SubElement(inertial, 'inertia', ixx=num(kg * (sy ** 2 + sz ** 2) / 12), ixy='0', ixz='0',
                  iyy=num(kg * (sx ** 2 + sz ** 2) / 12), iyz='0', izz=num(kg * (sx ** 2 + sy ** 2) / 12))
    vis = ET.SubElement(lk, 'visual')
    if len(ext['tris']) == 12:
        ET.SubElement(vis, 'origin', xyz=vec((0, 0, sz / 2)), rpy='0 0 0')
        ET.SubElement(ET.SubElement(vis, 'geometry'), 'box', size=vec(size))
    else:
        mesh = f'{name.removesuffix("_link")}.stl'
        write_stl(out / 'meshes/main' / mesh, ext['tris'] - org)
        ET.SubElement(vis, 'origin', xyz='0 0 0', rpy='0 0 0')
        ET.SubElement(ET.SubElement(vis, 'geometry'), 'mesh', filename=f'package://{PKG}/meshes/main/{mesh}')
    ET.SubElement(vis, 'material', name=material)
    col = ET.SubElement(lk, 'collision', name=name.removesuffix('_link'))
    ET.SubElement(col, 'origin', xyz=vec((0, 0, sz / 2)), rpy='0 0 0')
    ET.SubElement(ET.SubElement(col, 'geometry'), 'box', size=vec(size))
    return lk


def lower_plate_links(ext, pos, out):
    """OpenCR 2 장 · 배터리 2 개 링크와 고정 관절 (B안 자리)."""
    links, joints = [], []
    for side in ('left', 'right'):
        for name, mat in (('opencr', 'opencr'), ('battery', 'battery')):
            ln = f'{name}_{side}_link'
            links.append(box_link(ln, ext[f'{name}_{side}'], mat, out))
            joints.append(fixed_joint(f'{name}_{side}_joint', 'base_link', ln, pos[ln]))
            e, p = ext[f'{name}_{side}'], pos[ln]
            h = (e['hi'] - e['lo']) / 2
            print(f'{ln}: base_link x {(p[0] - h[0]) * 1000:.1f}~{(p[0] + h[0]) * 1000:.1f} y {(p[1] - h[1]) * 1000:.1f}~{(p[1] + h[1]) * 1000:.1f} '
                  f'z {p[2] * 1000:.1f}~{(p[2] + 2 * h[2]) * 1000:.1f} mm (받은 자리 x {e["lo"][0] * 1000:.1f}~{e["hi"][0] * 1000:.1f}, z {e["lo"][2] * 1000:.1f}~)')
    return links, joints


def jetson_links(pos):
    """Jetson Orin Nano 개발자 키트 (jetson_link, 원점 = 받침 다리 바닥 가운데, 단자 변 -x)."""
    info = json.loads((PARTS / 'jetson_orin_nano.json').read_text())
    lk = sensor_link('jetson_link', info, 'jetson_orin_nano', 'jetson', box_collisions('jetson', info))
    origin = pos['jetson_link']
    bb = np.array(info['bbox_mm']) / 1000
    print(f'Jetson: base_link x {(origin[0] + bb[0]) * 1000:.1f}~{(origin[0] + bb[1]) * 1000:.1f} y {bb[2] * 1000:.1f}~{bb[3] * 1000:.1f} '
          f'z {(origin[2] + bb[4]) * 1000:.1f}~{(origin[2] + bb[5]) * 1000:.1f} mm, 단자 변 = 뒤 (-x)')
    return [lk], [fixed_joint('jetson_joint', 'base_link', 'jetson_link', origin)]


def tfmini_links(pos):
    """TFmini 브라켓 (tfmini_bracket, 원점 = 벽 바깥 면 가운데 밑면) + TFmini 몸체 · 측정 frame (tfmini_link, 렌즈 가운데, +x 앞)."""
    tf = json.loads((PARTS / 'tfmini_plus.json').read_text())
    br = json.loads((PARTS / 'tfmini_plus_bracket_v2.json').read_text())
    boxes = box_collisions('tfmini_bracket', br)
    bracket = sensor_link('tfmini_bracket', br, 'tfmini_plus_bracket_v2', 'pla_print', boxes)
    d, w, h = tf['box_mm']
    sensor = sensor_link('tfmini_link', tf, 'tfmini_plus', 'tfmini_dark',
                         [('tfmini_body', (-d / 2000, 0, 0), 'box', dict(size=vec((d / 1000, w / 1000, h / 1000))))])
    origin = pos['tfmini_bracket']                          # 링크 원점 = 벽 바깥 면
    lens = np.array(br['lens_mm']) / 1000
    joints = [fixed_joint('tfmini_bracket_joint', 'base_link', 'tfmini_bracket', origin),
              fixed_joint('tfmini_joint', 'tfmini_bracket', 'tfmini_link', lens)]
    p = origin + lens
    print(f'TFmini: 렌즈 가운데 base_link {np.round(p * 1000, 1)} mm, 바닥 위 {(p[2] + FOOTPRINT_Z) * 1000:.1f} mm')
    return [bracket, sensor], joints


def puck_plate_links(mount):
    """퍽 플레이트 (puck_plate_link, 원점 = 팔 joint1 축 · 플레이트 밑면). 질량 특성 · 충돌 조각 = tools/make_puck_plate.py 결과.
    충돌은 볼록 조각 메시 (collision:=full). light 상자는 write_xacro 가 JSON 의 light_boxes 로 바꿔 넣는다."""
    info = json.loads((PARTS / 'puck_plate.json').read_text())
    assert info['source_sha1'] == hashlib.sha1(PUCK_PLATE_SRC.read_bytes()).hexdigest(), \
        f'{PUCK_PLATE_SRC.name} 이 바뀜 - 먼저 tools/make_puck_plate.py 를 다시 돌릴 것'
    cols = [(f'puck_plate_{Path(f).stem}', (0, 0, 0), 'mesh', dict(filename=f'package://{PKG}/meshes/parts/{f}'))
            for f in info['collision_meshes']]
    lk = sensor_link('puck_plate_link', info, 'puck_plate', 'puck_plate', cols)
    origin = np.array([mount[0], mount[1], PLATE_TOP_Z])
    bb = np.array(info['bbox_mm']) / 1000
    print(f'퍽 플레이트: base_link x {(origin[0] + bb[0]) * 1000:.1f}~{(origin[0] + bb[1]) * 1000:.1f} y {bb[2] * 1000:.1f}~{bb[3] * 1000:.1f} '
          f'z {(origin[2] + bb[4]) * 1000:.1f}~{(origin[2] + bb[5]) * 1000:.1f} mm, {info["mass_kg"] * 1000:.1f} g, 컵 {len(info["cups"])} 개, '
          f'충돌 조각 {len(cols)} 개')
    return [lk], [fixed_joint('puck_plate_joint', 'base_link', 'puck_plate_link', origin)]


def build(src, arm_urdf, out):
    src_robot = ET.parse(src / 'urdf/waffle_4wd.urdf').getroot()
    robot = deepcopy(src_robot)
    arm = ET.parse(arm_urdf).getroot()
    assert LDS_JSON.exists(), f'{LDS_JSON} 없음 - 먼저 tools/make_part_meshes.py 를 FreeCAD 로 실행'
    lds = json.loads(LDS_JSON.read_text())

    # 옛 팔 · 옛 LiDAR 빼기
    gone = OLD_ARM_LINKS | OLD_LIDAR_LINKS
    for j in list(robot.findall('joint')):
        if j.find('child').get('link') in gone:
            robot.remove(j)
    for lk in list(robot.findall('link')):
        if lk.get('name') in gone:
            robot.remove(lk)

    # 바퀴 이름 · 루트 (팀 dev)
    links = {lk.get('name'): lk for lk in robot.findall('link')}
    for j in robot.findall('joint'):
        if j.get('name') in WHEEL_RENAME:
            jn, ln = WHEEL_RENAME[j.get('name')]
            links[j.find('child').get('link')].set('name', ln)
            j.set('name', jn)
            j.find('child').set('link', ln)
    rj = next(j for j in robot.findall('joint') if j.get('name') == ROOT_JOINT[0])
    assert rj.find('parent').get('link') == 'base_footprint' and rj.find('child').get('link') == 'base_link'
    assert np.allclose([float(v) for v in rj.find('origin').get('rpy').split()], 0), '루트 관절에 회전이 있음'
    rj.set('name', ROOT_JOINT[1])
    rj.find('parent').set('link', 'base_link')
    rj.find('child').set('link', 'base_footprint')
    rj.find('origin').set('xyz', vec(-xyz_of(rj)))

    # 차체 메시 · 충돌 상자 · 질량 분포
    for sub in GENERATED:
        shutil.rmtree(out / sub, ignore_errors=True)
    (out / 'meshes/main').mkdir(parents=True)
    moved_parts, removed_parts, servo_bottom, extracted = edit_base_mesh(src / 'meshes/main/base_link.dae', out / 'meshes/main/base_link.dae')
    base = next(lk for lk in robot.findall('link') if lk.get('name') == 'base_link')
    boxes = [c for c in base.findall('collision') if c.find('geometry/box') is not None]
    j1_boxes = [c for c in boxes if in_box(xyz_of(c), J1_REGION)]
    assert len(j1_boxes) == J1_BOXES, f'J1 충돌 상자 {len(j1_boxes)} 개 (기대 {J1_BOXES})'
    for c in j1_boxes:
        c.find('origin').set('xyz', vec(xyz_of(c) + J1_SHIFT))
    b = OLD_BRACKET
    old_boxes = [c for c in boxes if b['x'][0] <= abs(xyz_of(c)[0]) <= b['x'][1] and b['z'][0] <= xyz_of(c)[2] <= b['z'][1]]
    assert len(old_boxes) == 2, f'옛 LiDAR 브라켓 충돌 상자 {len(old_boxes)} 개 (기대 2)'
    jet_boxes = [c for c in boxes if in_box(xyz_of(c), JETSON_PLACEHOLDER)
                 and np.allclose([float(v) for v in c.find('geometry/box').get('size').split()], (0.103, 0.0905, 0.0348), atol=1e-3)]
    assert len(jet_boxes) == 1, f'Jetson 자리 충돌 상자 {len(jet_boxes)} 개 (기대 1)'
    box_size = lambda c: np.array([float(v) for v in c.find('geometry/box').get('size').split()])
    part_boxes = [c for c in boxes if any(in_box(xyz_of(c), R) and np.allclose(box_size(c), e['hi'] - e['lo'], atol=1e-3)
                                          for R, e in [(OPENCR_REGION, extracted['opencr_left']), (BATTERY_REGION, extracted['battery_left'])])]
    assert len(part_boxes) == 4, f'OpenCR · 배터리 충돌 상자 {len(part_boxes)} 개 (기대 4)'
    fb_boxes = [c for c in boxes if in_box(xyz_of(c), FLOAT_BOARD_REGION) and np.allclose(box_size(c), (0.0304, 0.018, 0.0077), atol=1e-3)]
    assert len(fb_boxes) == 1, f'뜬 기판 충돌 상자 {len(fb_boxes)} 개 (기대 1)'
    hull_group = lambda c: re.sub(r'_hull\d+(_free\d+)?\.stl$', '', Path(c.find('geometry/mesh').get('filename')).name)
    st_hulls = [c for c in base.findall('collision') if c.find('geometry/mesh') is not None and hull_group(c) in STORAGE_HULL_GROUPS]
    assert len(st_hulls) == STORAGE_HULLS, f'퍽 보관통 · 레일 충돌 볼록 메시 {len(st_hulls)} 개 (기대 {STORAGE_HULLS})'
    for c in old_boxes + jet_boxes + part_boxes + fb_boxes + st_hulls:
        base.remove(c)
    update_inertial(base.find('inertial'), [(m, c, J1_SHIFT) for m, c in moved_parts], removed_parts)

    # 팔 장착 자리 = 옮긴 J1 서보 바닥 중심 + (arm_bench link1 -> joint1 x, MOUNT_Z)
    old_yaw = xyz_of(next(j for j in src_robot.findall('joint') if j.get('name') == 'RMX52_Base_Yaw'))
    arm_j = {j.get('name'): j for j in arm.findall('joint')}
    horn = xyz_of(arm_j['joint1'])
    assert np.allclose(horn, xyz_of(arm_j['arm_base_link_joint'])), 'arm_bench joint1 과 arm_base_link 자리가 다름'
    mount = servo_bottom + J1_SHIFT + np.array([horn[0], horn[1], MOUNT_Z])
    assert abs(mount[0] - (old_yaw[0] + J1_SHIFT[0])) < 5e-4 and abs(mount[1]) < 5e-4, \
        f'J1 축 {mount} 이 받은 URDF 의 RMX52_Base_Yaw {old_yaw} + 이동과 다름'
    print(f'arm_base_link (J1 축) = base_link 에서 {np.round(mount * 1000, 2)} mm')

    # 팔 붙이기 (arm_bench link2 아래, 이름 바꿈)
    (out / 'meshes/arm').mkdir(parents=True)
    used_mats = set(EXTRA_MATERIALS)
    new_links = [ET.Element('link', name='arm_base_link')]
    for lk in arm.findall('link'):
        if lk.get('name') in ARM_SKIP_LINKS:
            continue
        lk = deepcopy(lk)
        lk.set('name', RENAME.get(lk.get('name'), lk.get('name')))
        for m in lk.iter('mesh'):
            f = Path(m.get('filename'))
            shutil.copy2(arm_urdf.parent / f, out / 'meshes/arm' / f.name)
            m.set('filename', f'package://{PKG}/meshes/arm/{f.name}')
        used_mats |= {m.get('name') for m in lk.iter('material')}
        new_links.append(lk)
    new_joints = [ET.Element('joint', name='arm_base_joint', type='fixed')]
    ET.SubElement(new_joints[0], 'origin', xyz=vec(mount), rpy='0 0 0')
    ET.SubElement(new_joints[0], 'parent', link='base_link')
    ET.SubElement(new_joints[0], 'child', link='arm_base_link')
    for j in arm.findall('joint'):
        if j.get('name') in ARM_SKIP_JOINTS:
            continue
        j = deepcopy(j)
        j.set('name', RENAME.get(j.get('name'), j.get('name')))
        for tag in ('parent', 'child'):
            j.find(tag).set('link', RENAME.get(j.find(tag).get('link'), j.find(tag).get('link')))
        if j.get('name') == 'joint1':                    # arm_base_link 가 곧 joint1 자리
            j.find('parent').set('link', 'arm_base_link')
            j.find('origin').set('xyz', '0 0 0')
        new_joints.append(j)

    # LDS-03 앞 · 뒤, TFmini
    pos = layout(extracted, json.loads((PARTS / 'jetson_orin_nano.json').read_text()),
                 json.loads((PARTS / 'tfmini_plus_bracket_v2.json').read_text()))
    for lk, jn in ([lds_links(side, lds) for side in ('front', 'rear')]
                   + [tfmini_links(pos), jetson_links(pos), lower_plate_links(extracted, pos, out), puck_plate_links(mount)]):
        new_links += lk
        new_joints += jn

    # 메시 경로를 이 패키지로, 받은 메시는 복사 (base_link.dae 는 위에서 새로 씀)
    for m in robot.iter('mesh'):
        rel = m.get('filename').replace(f'package://{SRC_PKG}/', '')
        m.set('filename', f'package://{PKG}/{rel}')
        if not (out / rel).exists():
            (out / rel).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src / rel, out / rel)

    # 재질: 남은 링크가 쓰는 것만 + arm_bench 재질 + LDS-03
    used_mats |= {m.get('name') for lk in robot.findall('link') for m in lk.iter('material')}
    for m in list(robot.findall('material')):
        if m.get('name') not in used_mats:
            robot.remove(m)
    mats = [deepcopy(mm) for mm in arm.findall('material') if mm.get('name') in used_mats]
    for name, rgba in EXTRA_MATERIALS.items():
        mats.append(ET.Element('material', name=name))
        ET.SubElement(mats[-1], 'color', rgba=rgba)
    first_link = list(robot).index(robot.find('link'))
    for i, m in enumerate(mats):
        robot.insert(first_link + i, m)
    last_link = max(i for i, e in enumerate(robot) if e.tag == 'link')
    for i, e in enumerate(new_links):
        robot.insert(last_link + 1 + i, e)
    for e in new_joints:
        robot.append(e)
    robot.insert(0, ET.Comment(
        f' 자동 생성: tools/make_revision.py. 받은 {SRC_PKG} + {arm_urdf.name} (link2 아래) + LDS-03 2 개 + TFmini Plus + Jetson, 하판 B안, 뜬 부품 지움, '
        f'퍽 보관통 · 레일 -> 퍽 플레이트. '
        f'J1 묶음 x +{J1_SHIFT[0] * 1000:.0f} mm, 이름 팀 Lv2 기준. 직접 고치지 말고 스크립트를 고칠 것 '))
    return robot


def light_collision_boxes(base, pkg_dir):
    """base_link 의 볼록 메시 충돌을 묶음 (partNNN · rail) 마다 외곽 상자 하나로 (Gazebo 가벼운 충돌, collision:=light)."""
    groups = {}
    for c in base.findall('collision'):
        m = c.find('geometry/mesh')
        if m is None:
            continue
        rel = m.get('filename').replace(f'package://{PKG}/', '')
        g = re.sub(r'_hull\d+(_free\d+)?\.stl$', '', Path(rel).name)
        b = Path(pkg_dir / rel).read_bytes()
        n = int(np.frombuffer(b[80:84], np.uint32)[0])
        v = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))['v'].reshape(-1, 3)
        lo, hi = groups.get(g, (np.full(3, np.inf), np.full(3, -np.inf)))
        groups[g] = (np.minimum(lo, v.min(0)), np.maximum(hi, v.max(0)))
    out = []
    for g, (lo, hi) in sorted(groups.items()):
        c = ET.Element('collision', name=f'light_{g.removeprefix("base_link_")}')
        ET.SubElement(c, 'origin', xyz=vec((lo + hi) / 2), rpy='0 0 0')
        ET.SubElement(ET.SubElement(c, 'geometry'), 'box', size=vec(hi - lo))
        out.append(c)
    return out


def write_xacro(robot, path, light_boxes):
    """prefix 인자를 받는 xacro (받은 패키지 launch 와 같은 사용법).
    collision 인자: full (기본, .urdf 와 같음) / light (base_link 볼록 메시 423 개 -> 묶음별 상자, Gazebo 를 빠르게).
    limit_margin 인자 (기본 0): 한계가 있는 관절의 lower · upper 를 이만큼 넓힘. Gazebo (DART) 에서 관절이 한계 끝에 닿으면
    속도 명령으로 빠져나오지 못하고 걸려서 (2026-10-28 손가락 시험), gazebo.launch.py 가 0.001 을 넘긴다."""
    x = deepcopy(robot)
    x.set('xmlns:xacro', 'http://www.ros.org/wiki/xacro')
    base = next(lk for lk in x.findall('link') if lk.get('name') == 'base_link')
    meshes = [c for c in base.findall('collision') if c.find('geometry/mesh') is not None]
    full = ET.Element('xacro:if', value="${collision == 'full'}")
    light = ET.Element('xacro:if', value="${collision == 'light'}")
    for c in meshes:
        base.remove(c)
        full.append(c)
    light.extend(deepcopy(light_boxes))
    base.extend([full, light])
    # 퍽 플레이트: full = 볼록 조각, light = 바닥판 상자 + 컵마다 꽉 찬 상자 (보관통 때와 같은 방식)
    plate = next(lk for lk in x.findall('link') if lk.get('name') == 'puck_plate_link')
    pfull = ET.Element('xacro:if', value="${collision == 'full'}")
    plight = ET.Element('xacro:if', value="${collision == 'light'}")
    for c in plate.findall('collision'):
        plate.remove(c)
        pfull.append(c)
    for bx in json.loads((PARTS / 'puck_plate.json').read_text())['light_boxes']:
        c = ET.SubElement(plight, 'collision', name=f'light_puck_plate_{bx["name"]}')
        ET.SubElement(c, 'origin', xyz=vec(bx['center_m']), rpy=vec((0, 0, bx['yaw_rad'])))
        ET.SubElement(ET.SubElement(c, 'geometry'), 'box', size=vec(bx['size_m']))
    plate.extend([pfull, plight])
    for e in x.iter():
        if e.tag in ('link', 'joint') and e.get('name'):
            e.set('name', '${prefix}' + e.get('name'))
        if e.tag == 'gazebo' and e.get('reference'):
            e.set('reference', '${prefix}' + e.get('reference'))
        if e.tag in ('gz_frame_id', 'left_joint', 'right_joint', 'joint_name', 'child_frame_id', 'robot_base_frame'):
            e.text = '${prefix}' + e.text
        if e.tag in ('parent', 'child'):
            e.set('link', '${prefix}' + e.get('link'))
        if e.tag == 'mimic':
            e.set('joint', '${prefix}' + e.get('joint'))
    x.insert(1, ET.Element('xacro:arg', name='prefix', default=''))
    x.insert(2, ET.Element('xacro:property', name='prefix', value='$(arg prefix)'))
    x.insert(3, ET.Element('xacro:arg', name='collision', default='full'))
    x.insert(4, ET.Element('xacro:property', name='collision', value='$(arg collision)'))
    x.insert(5, ET.Element('xacro:arg', name='limit_margin', default='0'))
    x.insert(6, ET.Element('xacro:property', name='limit_margin', value='$(arg limit_margin)'))
    for j in x.findall('joint'):
        lim = j.find('limit')
        if lim is not None and lim.get('lower') is not None:
            lim.set('lower', f"${{{lim.get('lower')} - limit_margin}}")
            lim.set('upper', f"${{{lim.get('upper')} + limit_margin}}")
    ET.indent(x, '  ')
    path.write_text("<?xml version='1.0' encoding='utf-8'?>\n" + ET.tostring(x, encoding='unicode') + '\n', encoding='utf-8')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', type=Path, default=DEFAULT_SRC)
    ap.add_argument('--arm', type=Path, default=DEFAULT_ARM)
    a = ap.parse_args()
    out = HERE
    robot = build(a.src, a.arm, out)
    S = gazebo_ext.add_gazebo(robot, out / 'config/sensors.yaml')
    n = gazebo_ext.write_bridge(S, out / 'config/gz_bridge.yaml')
    print(f'Gazebo: 센서 {len(robot.findall(".//sensor"))} 개 · 플러그인 {len(robot.findall(".//plugin"))} 개, 브리지 {n} 개 (config/gz_bridge.yaml)')

    (out / 'urdf').mkdir(exist_ok=True)
    ET.indent(robot, '  ')
    (out / 'urdf/waffle_4wd.urdf').write_text(
        "<?xml version='1.0' encoding='utf-8'?>\n" + ET.tostring(robot, encoding='unicode') + '\n', encoding='utf-8')
    base = next(lk for lk in robot.findall('link') if lk.get('name') == 'base_link')
    boxes = light_collision_boxes(base, out)
    write_xacro(robot, out / 'urdf/waffle_4wd.urdf.xacro', boxes)
    print(f'xacro collision:=light -> base_link 볼록 메시 {sum(c.find("geometry/mesh") is not None for c in base.findall("collision"))} 개를 상자 {len(boxes)} 개로, '
          f'퍽 플레이트 볼록 조각을 상자 {len(json.loads((PARTS / "puck_plate.json").read_text())["light_boxes"])} 개로')

    moving = [j.get('name') for j in robot.findall('joint') if j.get('type') != 'fixed']
    (out / 'config').mkdir(exist_ok=True)
    (out / 'config/joint_state.yaml').write_text(
        '# joint_state_publisher_gui 처음 값 (tools/make_revision.py 생성)\n\njoint_state_publisher:\n  ros__parameters:\n    zeros:\n'
        + ''.join(f'      {n}: 0.0\n' for n in sorted(moving)), encoding='utf-8')

    # launch · rviz 는 받은 패키지 것 (패키지 이름만 바꿈)
    for rel in ('launch/display.launch.py', 'rviz/display.rviz'):
        (out / rel).parent.mkdir(exist_ok=True)
        (out / rel).write_text((a.src / rel).read_text(encoding='utf-8').replace(SRC_PKG, PKG), encoding='utf-8')
    mass = sum(float(lk.find('inertial/mass').get('value')) for lk in robot.findall('link') if lk.find('inertial') is not None)
    print(f'링크 {len(robot.findall("link"))} · 관절 {len(robot.findall("joint"))} (움직이는 관절 {len(moving)}), 전체 질량 {mass:.4f} kg')


if __name__ == '__main__':
    main()
