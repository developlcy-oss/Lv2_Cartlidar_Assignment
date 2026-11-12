"""실기 URDF (urdf/robot.urdf.xacro, drive:=belt) <-> Isaac URDF (urdf_and_usd/isaac/urdf) 비교 시험 (#112).

두 모델은 링크를 나누는 방식이 다르다 (실기 joint1 원점 z 0.077 · link1 이 있고, Isaac 은 joint1 원점 0 · link1 이 없음,
같은 이름 link2 ~ link5 가 서로 다른 곳). 그래서 링크 단위가 아니라
  1) 관절 축 직선 (base_link 기준 방향 + 두 직선 사이 거리) 을 팔 자세 3^5 (0, ±0.3 rad) 마다,
  2) 뜻이 같은 프레임 (COMMON_FRAMES) 의 위치 · 자세를 같은 자세마다,
  3) 관절 종류 · 한계,
를 numpy 로 직접 계산한 FK 로 비교한다 (tf · KDL 을 쓰지 않음).
알려진 차이는 KNOWN_* 표에 이유와 함께 적고, 그 값 그대로일 때만 통과한다. 차이가 없어지면 표에서 지우라고 실패한다.
self-test: Isaac 모델을 일부러 틀리게 고쳐 시험이 그 차이를 잡는지 본다.
결과 JSON: 환경변수 MODEL_COMPARE_JSON 에 경로가 있으면 거기에 쓴다 (python3 test_model_compare.py 로 실행하면 표준 출력).
xacro 나 Isaac URDF 가 없으면 건너뛴다.
"""
import copy
import itertools
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

PKG_DIR = Path(__file__).resolve().parents[1]
REAL_XACRO = PKG_DIR / 'urdf' / 'robot.urdf.xacro'
ISAAC_DIR = PKG_DIR.parents[2] / 'urdf_and_usd' / 'isaac' / 'urdf'   # workspace/ros2_ws/src/<pkg> -> workspace/urdf_and_usd
ISAAC_URDFS = ['waffle_4wd_full.urdf', 'waffle_4wd_light.urdf']      # 기준 = full, light 는 기구학이 같아야 함

POS_TOL = 0.5e-3          # m
ANG_TOL = np.radians(0.1)  # rad
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
POSE_VALUES = (-0.3, 0.0, 0.3)

# 뜻이 같은 프레임 (실기 노드 · Isaac 센서가 쓰는 것). 이름이 같지만 뜻이 다른 link2 ~ link5 · 모델 한쪽에만 있는 프레임은 뺀다
COMMON_FRAMES = [
    'base_footprint', 'arm_base_link', 'tool_link', 'arm_camera_link', 'arm_camera_optical_frame',
    'tfmini_link', 'imu_link', 'laser_front', 'laser_rear', 'top_camera_front_link', 'top_camera_rear_link',
]
# 두 모델에 다 있지만 프레임으로 비교하지 않는 것 (이유)
SKIP_FRAMES = {
    'link2': '이름만 같음 (실기 = joint2 뒤, Isaac = joint1 뒤)',
    'link3': '이름만 같음', 'link4': '이름만 같음', 'link5': '이름만 같음',
    'gripper_roll_link': '실기 = 그리퍼 개폐 모터 뒤, Isaac = joint5 뒤 (tool_link 로 비교)',
    'wheel_left_front_link': '바퀴는 관절 축 직선 · 접지 원통으로 비교 (Isaac 링크 자세는 z 축 = 회전축)',
    'wheel_left_rear_link': '〃', 'wheel_right_front_link': '〃', 'wheel_right_rear_link': '〃',
    'base_link': '뿌리',
}

# 알려진 프레임 차이: 프레임 -> (Isaac - 실기 위치 [m] (base_link 기준, 팔 0 자세), 이유)
KNOWN_FRAME_DIFF = {
    'arm_base_link': ((0.0, 0.0, 0.0175),
                      '실기 = 팔 받침 아래 (joint1 원점 z +0.077), Isaac = 받침 위 (joint1 원점 0). joint1 축 직선은 같음'),
    'top_camera_front_link': ((0.0, -0.025024, 0.0),
                              '실기 = D435 infra1 (realsense 드라이버 기준), Isaac = 카메라 가운데 (infra1 은 optical x -0.025)'),
    'top_camera_rear_link': ((0.0, 0.025017, 0.0), '〃 (뒤 카메라, 180° 돌아 있어 부호 반대)'),
}
# 알려진 자세 차이: 프레임 -> (Isaac 과 실기 사이 각도 [deg], 이유). 이 각도 그대로일 때만 통과 (차이가 없어져도 실패)
KNOWN_FRAME_ROT = {
    'imu_link': (90.0, '실기 OpenCR 은 +y 가 차체 앞 (실기 yaw -90°, 2026-11-09 실물 확인), Isaac IMU 는 차체 축 그대로'),
    'top_camera_front_link': (2.588, '실물 실측 2026-11-12: 앞 D435 피치 0.6144098 -> 0.5692410 rad (2.588° 덜 숙임), Isaac 기준 모델은 그대로'),
}
# 알려진 관절 한계 차이: (관절, 항목) -> (실기, Isaac, 이유). 바퀴는 #15 에서 11.1 V 값으로 통일 (2026-11-09)
KNOWN_LIMIT_DIFF = {}
# 한쪽에만 있는 움직이는 관절
KNOWN_ONLY_ISAAC = {'gripper_left_joint', 'gripper_right_joint'}   # Isaac 은 손가락 직선 관절, 실기는 gripper_roll 모터 하나
KNOWN_ONLY_REAL = set()


# ---------------------------------------------------------------- 모델 읽기 · FK

def _vec(s, n=3):
    return np.array([float(v) for v in s.split()]) if s else np.zeros(n)


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle(axis, q):
    a = axis / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * k + (1 - np.cos(q)) * k @ k


def make_tf(rot, pos):
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = pos
    return t


class Model:
    def __init__(self, urdf_text, name):
        self.name = name
        self.root_xml = ET.fromstring(urdf_text)
        self.joints = {}
        for j in self.root_xml.findall('joint'):
            if j.get('type') is None:      # <ros2_control> 안의 <joint>
                continue
            o, a, lim = j.find('origin'), j.find('axis'), j.find('limit')
            self.joints[j.get('name')] = {
                'type': j.get('type'), 'parent': j.find('parent').get('link'), 'child': j.find('child').get('link'),
                'xyz': _vec(o.get('xyz')) if o is not None else np.zeros(3),
                'rpy': _vec(o.get('rpy')) if o is not None else np.zeros(3),
                'axis': _vec(a.get('xyz')) if a is not None else np.array([1.0, 0, 0]),
                'limit': {k: float(v) for k, v in lim.attrib.items()} if lim is not None else {},
            }
        self.links = {link.get('name'): link for link in self.root_xml.findall('link')}
        self.child_joint = {j['child']: n for n, j in self.joints.items()}
        roots = [n for n in self.links if n not in self.child_joint]
        assert roots == ['base_link'], f'{name}: 뿌리가 base_link 하나가 아님 {roots}'

    def movable(self):
        return {n for n, j in self.joints.items() if j['type'] != 'fixed'}

    def fk(self, q):
        """base_link 기준 링크 변환 {link: 4x4}, 관절 프레임 {joint: 4x4} (관절 원점, 회전 전)."""
        links, jframes = {'base_link': np.eye(4)}, {}

        def link_tf(link):
            if link not in links:
                jn = self.child_joint[link]
                j = self.joints[jn]
                parent = link_tf(j['parent'])
                jf = parent @ make_tf(rpy_matrix(j['rpy']), j['xyz'])
                jframes[jn] = jf
                v = q.get(jn, 0.0)
                if j['type'] in ('revolute', 'continuous'):
                    motion = make_tf(axis_angle(j['axis'], v), np.zeros(3))
                elif j['type'] == 'prismatic':
                    motion = make_tf(np.eye(3), j['axis'] / np.linalg.norm(j['axis']) * v)
                else:
                    motion = np.eye(4)
                links[link] = jf @ motion
            return links[link]

        for link in self.links:
            link_tf(link)
        return links, jframes

    def axis_line(self, jframes, jn):
        """관절 축 직선 (점, 단위 방향) base_link 기준."""
        jf = jframes[jn]
        d = jf[:3, :3] @ self.joints[jn]['axis']
        return jf[:3, 3], d / np.linalg.norm(d)

    def total_mass(self):
        return sum(float(m.get('value')) for m in self.root_xml.iter('mass'))

    def wheel_contact_y(self, link):
        """Isaac 바퀴 충돌 원통 중심의 base_link 기준 y (원통이 없으면 None)."""
        col = self.links[link].find('collision')
        if col is None or col.find('geometry/cylinder') is None:
            return None
        o = col.find('origin')
        local = _vec(o.get('xyz')) if o is not None else np.zeros(3)
        links, _ = self.fk({})
        return float((links[link] @ np.append(local, 1.0))[1])


def line_distance(p1, d1, p2, d2):
    """두 직선 사이 거리 (평행이면 점과 직선 거리)."""
    n = np.cross(d1, d2)
    if np.linalg.norm(n) < 1e-9:
        w = p2 - p1
        return float(np.linalg.norm(w - np.dot(w, d1) * d1))
    return float(abs(np.dot(p2 - p1, n)) / np.linalg.norm(n))


def rot_angle(r1, r2):
    c = (np.trace(r1.T @ r2) - 1.0) / 2.0
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


def arm_poses():
    for vals in itertools.product(POSE_VALUES, repeat=len(ARM_JOINTS)):
        yield dict(zip(ARM_JOINTS, vals))


# ---------------------------------------------------------------- 비교

def compare(real, isaac):
    """차이 목록 [(종류, 대상, 내용)] 와 요약 dict. 알려진 차이는 값이 같을 때만 통과."""
    errors, summary = [], {'poses': 0, 'max_frame_pos_mm': {}, 'max_frame_ang_deg': {}, 'max_axis_line_mm': {},
                           'max_axis_dir_deg': {}}

    # 관절 종류 · 연결
    rm, im = real.movable(), isaac.movable()
    if rm - im != KNOWN_ONLY_REAL:
        errors.append(('joint_set', 'real_only', sorted(rm - im)))
    if im - rm != KNOWN_ONLY_ISAAC:
        errors.append(('joint_set', 'isaac_only', sorted(im - rm)))
    common_joints = sorted(rm & im)
    for jn in common_joints:
        if real.joints[jn]['type'] != isaac.joints[jn]['type']:
            errors.append(('joint_type', jn, (real.joints[jn]['type'], isaac.joints[jn]['type'])))

    # 한계
    for jn in common_joints:
        lr, li = real.joints[jn]['limit'], isaac.joints[jn]['limit']
        for key in sorted(set(lr) | set(li)):
            a, b = lr.get(key), li.get(key)
            known = KNOWN_LIMIT_DIFF.get((jn, key))
            if known:
                if a is None or b is None or abs(a - known[0]) > 1e-6 or abs(b - known[1]) > 1e-6:
                    errors.append(('limit_known_changed', f'{jn}.{key}', {'real': a, 'isaac': b, 'table': known[:2]}))
            elif a is None or b is None or abs(a - b) > 1e-6:
                errors.append(('limit', f'{jn}.{key}', {'real': a, 'isaac': b}))

    # 프레임 목록 (새 프레임이 생기면 COMMON_FRAMES / SKIP_FRAMES 에 넣으라고 알림)
    shared = set(real.links) & set(isaac.links)
    unlisted = shared - set(COMMON_FRAMES) - set(SKIP_FRAMES)
    if unlisted:
        errors.append(('frame_list', 'unlisted_shared', sorted(unlisted)))
    for f in COMMON_FRAMES:
        if f not in shared:
            errors.append(('frame_list', 'missing', f))

    # 팔 0 자세에서 알려진 프레임 차이가 표 값 그대로인지
    rl0, _ = real.fk({})
    il0, _ = isaac.fk({})
    for f, (expect, _why) in KNOWN_FRAME_DIFF.items():
        if f in rl0 and f in il0:
            d = il0[f][:3, 3] - rl0[f][:3, 3]
            if np.linalg.norm(d - np.array(expect)) > POS_TOL:
                errors.append(('frame_known_changed', f, {'diff_m': d.round(6).tolist(), 'table': expect}))

    # 자세 3^5 마다 축 직선 · 프레임
    for q in arm_poses():
        summary['poses'] += 1
        rl, rj = real.fk(q)
        il, ij = isaac.fk(q)
        r0inv = {f: np.linalg.inv(rl0[f]) for f in KNOWN_FRAME_DIFF if f in rl0}
        for jn in common_joints:
            pr, dr = real.axis_line(rj, jn)
            pi, di = isaac.axis_line(ij, jn)
            dist = line_distance(pr, dr, pi, di) * 1e3
            ang = np.degrees(np.arccos(np.clip(np.dot(dr, di), -1, 1)))
            summary['max_axis_line_mm'][jn] = max(summary['max_axis_line_mm'].get(jn, 0.0), dist)
            summary['max_axis_dir_deg'][jn] = max(summary['max_axis_dir_deg'].get(jn, 0.0), ang)
            if dist > POS_TOL * 1e3 or np.radians(ang) > ANG_TOL:
                errors.append(('axis_line', jn, {'q': q, 'dist_mm': round(dist, 3), 'dir_deg': round(ang, 3)}))
        for f in COMMON_FRAMES:
            if f not in rl or f not in il:
                continue
            tr, ti = rl[f], il[f]
            if f in KNOWN_FRAME_DIFF:   # 표의 차이만큼 옮긴 뒤 비교 (자세에 따라 차이가 달라지면 잡힘)
                ti = ti.copy()
                ti[:3, 3] -= tr[:3, :3] @ (r0inv[f][:3, :3] @ np.array(KNOWN_FRAME_DIFF[f][0]))
            dp = float(np.linalg.norm(ti[:3, 3] - tr[:3, 3])) * 1e3
            da = np.degrees(rot_angle(tr[:3, :3], ti[:3, :3]))
            summary['max_frame_pos_mm'][f] = max(summary['max_frame_pos_mm'].get(f, 0.0), dp)
            summary['max_frame_ang_deg'][f] = max(summary['max_frame_ang_deg'].get(f, 0.0), da)
            if dp > POS_TOL * 1e3 or np.radians(abs(da - KNOWN_FRAME_ROT.get(f, (0.0,))[0])) > ANG_TOL:
                errors.append(('frame', f, {'q': q, 'pos_mm': round(dp, 3), 'ang_deg': round(da, 3)}))

    # 바퀴: Isaac 접지 원통 중심 |y| = 실기 바퀴 관절 |y|
    for jn in common_joints:
        if not jn.startswith('wheel_'):
            continue
        link = isaac.joints[jn]['child']
        y_isaac = isaac.wheel_contact_y(link)
        y_real = float(rl0[real.joints[jn]['child']][1, 3])
        if y_isaac is None or abs(y_isaac - y_real) > POS_TOL:
            errors.append(('wheel_contact_y', jn, {'real': y_real, 'isaac_cylinder': y_isaac}))

    for k in ('max_frame_pos_mm', 'max_frame_ang_deg', 'max_axis_line_mm', 'max_axis_dir_deg'):
        summary[k] = {n: round(v, 4) for n, v in summary[k].items()}
    # 총질량: 실기 xacro 에 <inertial> 이 없어 비교 못 함 -> 기록만
    summary['mass_kg'] = {'real': round(real.total_mass(), 4), 'isaac': round(isaac.total_mass(), 4)}
    return errors, summary


# ---------------------------------------------------------------- 읽기

def expand_real():
    if shutil.which('xacro') is None:
        return None
    text = REAL_XACRO.read_text().replace('$(find opencr_hardware)', str(PKG_DIR))
    with tempfile.NamedTemporaryFile('w', suffix='.xacro', delete=False) as f:
        f.write(text)
        tmp = f.name
    try:
        out = subprocess.run(['xacro', tmp, 'drive:=belt'], capture_output=True, text=True, check=True)
    finally:
        os.unlink(tmp)
    return Model(out.stdout, 'real')


@pytest.fixture(scope='module')
def real():
    m = expand_real()
    if m is None:
        pytest.skip('xacro 없음')
    return m


@pytest.fixture(scope='module')
def isaac_texts():
    paths = [ISAAC_DIR / n for n in ISAAC_URDFS]
    if not all(p.exists() for p in paths):
        pytest.skip(f'Isaac URDF 없음 ({ISAAC_DIR})')
    return {p.name: p.read_text() for p in paths}


@pytest.fixture(scope='module')
def isaac(isaac_texts):
    return Model(isaac_texts[ISAAC_URDFS[0]], 'isaac')


def _fmt(errors, n=15):
    lines = [f'{k} {t}: {v}' for k, t, v in errors[:n]]
    if len(errors) > n:
        lines.append(f'... 외 {len(errors) - n} 건')
    return '\n'.join(lines)


# ---------------------------------------------------------------- 시험

def test_real_vs_isaac(real, isaac):
    errors, summary = compare(real, isaac)
    out = os.environ.get('MODEL_COMPARE_JSON')
    if out:
        Path(out).write_text(json.dumps(report(errors, summary), ensure_ascii=False, indent=1))
    assert not errors, '실기 <-> Isaac 차이 (알려진 차이 표 밖):\n' + _fmt(errors)


def test_isaac_light_same_kinematics(isaac_texts, isaac):
    """light 판은 충돌만 다르고 관절은 full 과 같아야 한다."""
    light = Model(isaac_texts[ISAAC_URDFS[1]], 'isaac_light')
    assert light.joints.keys() == isaac.joints.keys()
    for jn, j in isaac.joints.items():
        k = light.joints[jn]
        assert (j['type'], j['parent'], j['child']) == (k['type'], k['parent'], k['child']), jn
        for key in ('xyz', 'rpy', 'axis'):
            assert np.allclose(j[key], k[key], atol=1e-9), f'{jn}.{key}'
        assert j['limit'] == k['limit'], jn


def _mutate(text, joint, fn):
    root = ET.fromstring(text)
    for j in root.findall('joint'):
        if j.get('name') == joint:
            fn(j)
    return ET.tostring(root, encoding='unicode')


def _shift_origin(axis_i, delta):
    def fn(j):
        o = j.find('origin')
        v = _vec(o.get('xyz'))
        v[axis_i] += delta
        o.set('xyz', ' '.join(map(str, v)))
    return fn


SELF_TESTS = {
    # 이름: (관절, 고치는 함수, 잡혀야 하는 차이 종류)
    'joint3_origin_z_+1mm': ('joint3', _shift_origin(2, 1e-3), {'axis_line', 'frame'}),
    'joint5_axis_flip': ('joint5', lambda j: j.find('axis').set('xyz', '-1 0 0'), {'axis_line', 'frame'}),
    'joint2_lower_changed': ('joint2', lambda j: j.find('limit').set('lower', '-2.0'), {'limit'}),
    'arm_camera_pitch_+1deg': ('arm_camera_joint',
                               lambda j: j.find('origin').set('rpy', f'0 {0.261788 + np.radians(1)} 0'), {'frame'}),
    'wheel_effort_changed': ('wheel_left_front_joint', lambda j: j.find('limit').set('effort', '2.0'), {'limit'}),
    'top_camera_front_y_fixed': ('top_camera_front_link_fixed', _shift_origin(1, 0.025024),
                                 {'frame_known_changed'}),
    'wheel_contact_+2mm': ('wheel_right_rear_joint', _shift_origin(1, -2e-3), {'wheel_contact_y'}),
    'imu_yaw_diff_gone': ('imu_joint', lambda j: j.find('origin').set('rpy', '0 0 -1.5707963267948966'), {'frame'}),
}


@pytest.mark.parametrize('name', sorted(SELF_TESTS))
def test_self_test_catches(real, isaac_texts, name):
    joint, fn, expect = SELF_TESTS[name]
    bad = Model(_mutate(isaac_texts[ISAAC_URDFS[0]], joint, fn), 'isaac_bad')
    errors, _ = compare(real, bad)
    kinds = {k for k, _, _ in errors}
    assert expect <= kinds, f'{name}: 잡혀야 할 {expect} 가 없음 (잡힌 것 {kinds})'


def report(errors, summary):
    return {
        'pass': not errors,
        'real': f'{REAL_XACRO.relative_to(PKG_DIR.parent)} drive:=belt',
        'isaac': str(ISAAC_DIR / ISAAC_URDFS[0]),
        'tol': {'pos_mm': POS_TOL * 1e3, 'ang_deg': float(np.degrees(ANG_TOL))},
        'errors': [{'kind': k, 'target': t, 'detail': v} for k, t, v in errors],
        'known_frame_diff': {f: {'isaac_minus_real_m': d, 'why': w} for f, (d, w) in KNOWN_FRAME_DIFF.items()},
        'known_frame_rot': {f: {'angle_deg': d, 'why': w} for f, (d, w) in KNOWN_FRAME_ROT.items()},
        'known_limit_diff': {f'{j}.{k}': {'real': a, 'isaac': b, 'why': w}
                             for (j, k), (a, b, w) in KNOWN_LIMIT_DIFF.items()},
        'summary': summary,
    }


if __name__ == '__main__':
    r = expand_real()
    if r is None:
        sys.exit('xacro 없음 (source /opt/ros/<배포판>/setup.bash)')
    i = Model((ISAAC_DIR / ISAAC_URDFS[0]).read_text(), 'isaac')
    errs, summ = compare(r, i)
    print(json.dumps(report(errs, summ), ensure_ascii=False, indent=1, default=float))
    sys.exit(1 if errs else 0)
