#!/usr/bin/env python3
"""DRIVE <-> STOW 접기 경로의 팔 ↔ 차체 · 팔레트 v5 · 퍽 최소 틈 (#277 1번, 계산만).

- 모델: 팀 URDF waffle_4wd_loaded.urdf (B안 · 팔레트 v5 · 퍽 10 개) 의 collision.
  팔 (link3 · link4 · link5 · gripper_roll_link · 손가락) = 상자, 장애물 (base_link 상자 · CoACD 볼록 조각, 퍽) = 겉면 점 (약 1.5 mm 간격).
- 틈 = 장애물 겉면 점 ↔ 팔 상자 부호 거리의 최소 (음수 = 겹침 깊이).
- 경로: 관절 묶음을 차례로 직선 보간 (노드 · meas_session 과 같은 순서, 최소 저크도 같은 길).
- 한계: 설계 URDF (영점 · 처짐 없음). 실물은 관절 2 · 3 이 1 ~ 3° 처지고 손으로 놓은 STOW 는 tick 범위만큼 다를 수 있음.

실행: python3 fold_clearance.py [--no-pucks]   (numpy · scipy 만, GPU 안 씀, 약 15 s)
"""
import argparse
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
DESC = os.path.normpath(os.path.join(HERE, '..', '..', 'urdf_and_usd', 'waffle_4wd_description'))
ARM_LINKS = ['link3', 'link4', 'link5', 'gripper_roll_link', 'gripper_left_link', 'gripper_right_link']
STEP = 0.0015


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin_T(el):
    T = np.eye(4)
    if el is None:
        return T
    xyz = [float(v) for v in el.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in el.get('rpy', '0 0 0').split()]
    T[:3, :3] = rpy_mat(*rpy)
    T[:3, 3] = xyz
    return T


def axis_T(axis, q, kind):
    T = np.eye(4)
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    if kind == 'prismatic':
        T[:3, 3] = a * q
        return T
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T[:3, :3] = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    return T


def read_stl(path):
    with open(path, 'rb') as f:
        data = f.read()
    if data[:5] == b'solid' and b'facet' in data[:300]:
        v = [list(map(float, ln.split()[1:4])) for ln in data.decode(errors='ignore').splitlines() if ln.strip().startswith('vertex')]
        return np.array(v).reshape(-1, 3, 3)
    n = struct.unpack('<I', data[80:84])[0]
    arr = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return arr['v'].reshape(-1, 3, 3).astype(float)


def sample_tris(tris, step=STEP):
    pts = [tris.reshape(-1, 3)]
    rng = np.random.default_rng(0)
    for t in tris:
        area = 0.5 * np.linalg.norm(np.cross(t[1] - t[0], t[2] - t[0]))
        n = int(math.ceil(area / (step * step) * 2))
        if n <= 0:
            continue
        u = rng.random((n, 2))
        m = u.sum(1) > 1
        u[m] = 1 - u[m]
        pts.append(t[0] + u[:, :1] * (t[1] - t[0]) + u[:, 1:] * (t[2] - t[0]))
    return np.vstack(pts)


def box_surface(size, step=STEP):
    sx, sy, sz = [s / 2 for s in size]
    out = []
    for ax in range(3):
        o = [a for a in range(3) if a != ax]
        h = [sx, sy, sz]
        g0 = np.linspace(-h[o[0]], h[o[0]], max(2, int(2 * h[o[0]] / step) + 1))
        g1 = np.linspace(-h[o[1]], h[o[1]], max(2, int(2 * h[o[1]] / step) + 1))
        A, B = np.meshgrid(g0, g1)
        for s in (-1, 1):
            p = np.zeros((A.size, 3))
            p[:, o[0]] = A.ravel()
            p[:, o[1]] = B.ravel()
            p[:, ax] = s * h[ax]
            out.append(p)
    return np.vstack(out)


def cyl_surface(r, L, step=STEP):
    na = max(12, int(2 * math.pi * r / step))
    nz = max(2, int(L / step) + 1)
    a = np.linspace(0, 2 * math.pi, na, endpoint=False)
    z = np.linspace(-L / 2, L / 2, nz)
    A, Z = np.meshgrid(a, z)
    side = np.c_[r * np.cos(A.ravel()), r * np.sin(A.ravel()), Z.ravel()]
    caps = []
    for rr in np.linspace(0, r, max(2, int(r / step) + 1)):
        nn = max(1, int(2 * math.pi * rr / step))
        aa = np.linspace(0, 2 * math.pi, nn, endpoint=False)
        for zz in (-L / 2, L / 2):
            caps.append(np.c_[rr * np.cos(aa), rr * np.sin(aa), np.full(nn, zz)])
    return np.vstack([side] + caps)


def apply(T, p):
    return p @ T[:3, :3].T + T[:3, 3]


class Model:
    def __init__(self, urdf):
        self.root = ET.parse(urdf).getroot()
        self.links = {l.get('name'): l for l in self.root.findall('link')}
        self.joints = {j.find('child').get('link'): j for j in self.root.findall('joint')}

    def chain_T(self, link, q):
        """base_link → link. q: 관절 이름 → 값"""
        T = np.eye(4)
        path = []
        cur = link
        while cur in self.joints:
            j = self.joints[cur]
            path.append(j)
            cur = j.find('parent').get('link')
        for j in reversed(path):
            T = T @ origin_T(j.find('origin'))
            if j.get('type') in ('revolute', 'continuous', 'prismatic'):
                ax = [float(v) for v in j.find('axis').get('xyz').split()]
                T = T @ axis_T(ax, q.get(j.get('name'), 0.0), j.get('type'))
        return T

    def obstacle_points(self, link, with_pucks=True):
        """base_link (와 퍽) 겉면 점 + 각 점의 출처 이름 (base_link 좌표)"""
        pts, tags = [], []
        # base_link 와 거기 고정된 모든 링크 (라이다 · TFmini · 젯슨 · OpenCR · 바퀴), 퍽은 선택
        arm_side = {'arm_base_link', 'link2'} | set(ARM_LINKS) | {'tool_link', 'arm_camera_link', 'arm_camera_optical_frame', 'gripper_motor_link'}
        names = [n for n in self.links if n not in arm_side and self.links[n].findall('collision')
                 and (with_pucks or not n.startswith('puck_'))]
        for name in names:
            Tl = self.chain_T(name, {})
            for c in self.links[name].findall('collision'):
                g = c.find('geometry')[0]
                Tc = Tl @ origin_T(c.find('origin'))
                if g.tag == 'mesh':
                    fn = g.get('filename').replace('package://waffle_4wd_description/', DESC + '/')
                    sc = np.array([float(v) for v in g.get('scale', '1 1 1').split()])
                    p = sample_tris(read_stl(fn) * sc)
                    tag = os.path.basename(fn).rsplit('_hull', 1)[0]
                elif g.tag == 'box':
                    p = box_surface([float(v) for v in g.get('size').split()])
                    tag = c.get('name') or 'box'
                elif g.tag == 'cylinder':
                    p = cyl_surface(float(g.get('radius')), float(g.get('length')))
                    tag = 'cyl'
                else:
                    continue
                if name != 'base_link':
                    tag = name
                pts.append(apply(Tc, p))
                tags += [tag] * len(p)
        return np.vstack(pts), np.array(tags)

    def arm_boxes(self):
        """(링크, 상자 원점 T (링크 기준), 반 크기)"""
        out = []
        for name in ARM_LINKS:
            for c in self.links[name].findall('collision'):
                g = c.find('geometry')[0]
                if g.tag != 'box':
                    continue
                out.append((name, origin_T(c.find('origin')), np.array([float(v) for v in g.get('size').split()]) / 2))
        return out


def box_sdf(Tw, half, P):
    """Tw: 상자 → base_link. P: (N,3) base_link. 반환 부호 거리 (N,)"""
    R, t = Tw[:3, :3], Tw[:3, 3]
    local = (P - t) @ R
    d = np.abs(local) - half
    outside = np.linalg.norm(np.maximum(d, 0), axis=1)
    inside = np.minimum(d.max(1), 0)
    return outside + inside


JN = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']


def grip_m(rad):
    # 집게 rad → 손가락 m (manipulator.yaml toppick_rl finger_m_at_open 0.0125 @ −0.764, at_close −0.0116 @ 0.923)
    return 0.0125 + (rad + 0.764) * (-0.0116 - 0.0125) / (0.923 + 0.764)


def qmap(q6):
    d = {JN[i]: q6[i] for i in range(5)}
    m = grip_m(q6[5])
    d['gripper_left_joint'] = m
    d['gripper_right_joint'] = m
    return d


def path(start, goal, order, n_per_rad=60):
    """관절 묶음 (0 기반) 차례로 직선 보간. 반환 [(단계 이름, q6)]"""
    q = np.array(start, float)
    out = [('start', q.copy())]
    for grp in order:
        tgt = q.copy()
        for j in grp:
            tgt[j] = goal[j]
        n = max(2, int(np.abs(tgt - q).max() * n_per_rad))
        for s in np.linspace(0, 1, n)[1:]:
            out.append(('J' + '·'.join(str(j + 1) for j in grp), q + (tgt - q) * s))
        q = tgt
    return out


def evaluate(model, obs, tags, boxes, waypoints, label):
    tree = cKDTree(obs)
    worst = []   # 단계별 최소
    per_step = {}
    rows = []
    for stage, q6 in waypoints:
        qm = qmap(q6)
        best = (1e9, None, None)
        Tlinks = {n: model.chain_T(n, qm) for n in ARM_LINKS}
        for (ln, To, half) in boxes:
            Tw = Tlinks[ln] @ To
            r = float(np.linalg.norm(half)) + 0.03
            idx = tree.query_ball_point(Tw[:3, 3], r)
            if not idx:
                continue
            idx = np.asarray(idx)
            d = box_sdf(Tw, half, obs[idx])
            k = int(np.argmin(d))
            if d[k] < best[0]:
                best = (float(d[k]), ln, str(tags[idx[k]]))
        rows.append((stage, q6.copy(), best))
        if stage not in per_step or best[0] < per_step[stage][0]:
            per_step[stage] = (best[0], best[1], best[2], q6.copy())
    print(f'\n== {label}')
    for st, (d, ln, tg, q) in per_step.items():
        flag = '  << 겹침' if d < 0 else ('  < 10 mm' if d < 0.010 else '')
        print(f'  {st:8s} 최소 {d * 1000:+6.1f} mm  {ln} ↔ {tg}  q=[{", ".join(f"{v:+.3f}" for v in q[:5])}]{flag}')
    return rows, per_step


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', default=os.path.join(DESC, 'urdf/waffle_4wd_loaded.urdf'))
    ap.add_argument('--no-pucks', action='store_true')
    args = ap.parse_args()
    m = Model(args.urdf)
    obs, tags = m.obstacle_points('base_link', with_pucks=not args.no_pucks)
    # 팔이 서 있는 자리 (조인트1 축 둘레 링크2 · 받침) 는 늘 닿아 있으므로 빼지 않고, 관절 2 위 링크 (link3 ~) 만 본다
    boxes = m.arm_boxes()
    print(f'장애물 점 {len(obs)} 개, 팔 상자 {len(boxes)} 개, 퍽 {"없음" if args.no_pucks else "10 개"}')

    STOW = [2.200, -2.088, 1.549, -1.868, 0.0, 0.923]        # manipulator.yaml (#277 J1, 11-16 관절 1 π 보정 펌웨어 뒤 모델 각도 · J5 0)
    STOW_1115 = [1.8715, -2.088, 1.549, -1.868, -0.1304, 0.923]   # 11-15 하루 (관절 1 모터가 돌아 설치된 상태의 실물 자리)
    STOW_OLD = [1.554, -2.088, 1.549, -1.868, 0.043, 0.923]   # 11-09 (팔레트 v3 때)
    E2 = [0.0, -1.37, -0.01, 1.85, 0.0, 0.923]                # FOLD_VIA = 11-13 ~ 11-17 DRIVE (ALIGN E2 + 집게 닫음)
    DRIVE = [0.0, -1.30, 0.60, 1.85, 0.0, 0.923]              # 2026-11-18 DRIVE (앞 RealSense 가림, drive_pose_check.py)
    OLD = [-0.0245, -1.6122, 1.2901, 1.1888, 0.0, 0.923]      # 11-13 전 DRIVE
    cases = [
        ('A. 접기: DRIVE E2 → STOW, 그리퍼·J5 → J1 → J4 → J2 → J3 (startup.stages 역순)', E2, STOW, [[4, 5], [0], [3], [1], [2]]),
        ('B. 펴기 (켜질 때 정리): STOW → DRIVE E2, J3 → J2 → J4 → J1 → J5·그리퍼', STOW, E2, [[2], [1], [3], [0], [4, 5]]),
        ('C. 예전 STOW (J1 1.554) 로 접기 — 칸 벽과 겹침 (#277)', E2, STOW_OLD, [[4, 5], [0], [3], [1], [2]]),
        ('D. #277 실물 경로 (확인용): 예전 DRIVE → 예전 STOW, J4 → J1 → J2 → J3', OLD, STOW_OLD, [[3], [0], [1], [2], [4, 5]]),
        ('E. 11-15 STOW (J1 1.8715) 로 접기 (비교용)', E2, STOW_1115, [[4, 5], [0], [3], [1], [2]]),
        ('F. 11-18 펴기 끝: FOLD_VIA (E2) → DRIVE, 관절 1 ~ 5 한 번에 (startup.unfold_via)', E2, DRIVE, [[0, 1, 2, 3, 4]]),
        ('G. 11-18 DRIVE → STOW 를 바로 묶음 순서로 (쓰지 않음: 퍽 R_1 겹침 — 그래서 FOLD_VIA 를 거침)', DRIVE, STOW, [[4, 5], [0], [3], [1], [2]]),
    ]
    for label, a, b, order in cases:
        evaluate(m, obs, tags, boxes, path(a, b, order), label)


if __name__ == '__main__':
    main()
