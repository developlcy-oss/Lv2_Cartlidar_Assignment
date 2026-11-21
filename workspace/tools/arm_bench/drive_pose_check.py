#!/usr/bin/env python3
"""주행 자세 (DRIVE) 검사 — 위 RealSense 가림 · 차체 · 퍽 틈 · 다른 자세로 가는 길 (2026-11-18, 계산만).

- 모델: fold_clearance.py 와 같은 팀 URDF (waffle_4wd_loaded.urdf, 팔레트 v5 · 퍽 10 개) 충돌 모양.
- 실물 팔 3 가지로 봄 (실제 관절 각 = 명령 − e − k·L, tools/arm_calib/arm_calib_fit.py 와 같은 식):
    URDF  = 설계 그대로
    real1 = 영점만 (실측 10 자세: IsaacLab .../기록/실측/6_보정자세.csv 6 줄 + 1_처짐_요약.md 4 줄 z, --joints 2,3,4) RMS 20 mm
    real2 = 영점 + 관절 2 · 3 처짐 (같은 자료, --joints 2,3,4 --sag 2,3) RMS 19 mm
  자료가 적고 거칠어 (하나씩 빼고 맞춘 오차 23 ~ 26 mm) 정답이 아니라 '실물이 이 근처' 범위로 씀.
  확인: 실물에서 문제없던 예전 DRIVE (E2) · STOW 가 세 모델 모두 틈 9 mm 이상.
- 가림: 앞 · 뒤 D435 컬러 시야 (69.4° × 42.5°) 에 팔 (link2 아래 관절 위 링크) 이 차지하는 비율. 자세 = robot.urdf.xacro (실물 피치).
- 길: 관절 공간 직선 (노드 이름 자세 이동과 같음). FOLD_VIA → DRIVE 는 startup.unfold_via.

실행: python3 drive_pose_check.py [--q 0,-1.25,0.60,1.80,0,0.923]   (numpy · scipy, 약 1 분)
"""
import argparse
import math
import os

import numpy as np
from scipy.spatial import cKDTree

import fold_clearance as fc

D = math.radians
MODELS = {
    'URDF': (np.zeros(5), np.zeros(5)),
    'real1': (np.array([0, D(-25.37), D(12.55), D(1.21), 0]), np.zeros(5)),
    'real2': (np.array([0, D(-13.68), D(13.59), D(12.16), 0]), np.array([0, D(-17.2), D(-55.6), 0, 0])),   # k: rad / m
}
TCP_X = 0.14
CAMS = {   # robot.urdf.xacro (실물): 위치 · 피치 · 요
    'front': (np.array([-0.0480903, 0.0, 0.4836226]), 0.5692410, 0.0),
    'rear': (np.array([-0.2079097, 0.0, 0.4836226]), 0.5926, math.pi),
}
HF, VF = math.radians(69.4) / 2, math.radians(42.5) / 2
NX, NY = 140, 84
STOW = [2.200, -2.088, 1.549, -1.868, 0.0, 0.923]
FOLD_VIA = [0.0, -1.37, -0.01, 1.85, 0.0, 0.923]
NAMED = {'ALIGN': [0, -1.37, -0.01, 1.85, 0, 0.0], 'BOX_VIEW': [0, -0.65, 0, 1.95, 0, 0],
         'TRANSIT_TILT': [0, -0.898, -0.108, 1.792, 0, 0], 'TAG_VIEW': [0, 0.9, -1.4, 0.2, 0, 0]}
CHILD = {1: 'link2', 2: 'link3', 3: 'link4', 4: 'link5', 5: 'gripper_roll_link'}


class Check:
    def __init__(self):
        self.m = fc.Model(os.path.join(fc.DESC, 'urdf/waffle_4wd_loaded.urdf'))
        kids = {}
        for c, j in self.m.joints.items():
            kids.setdefault(j.find('parent').get('link'), []).append(c)

        def desc(l):
            return [l] + [x for k in kids.get(l, []) for x in desc(k)]
        self.lp = {l: self._pts(l) for l in desc('link2') if self.m.links[l].findall('collision')}
        self.obs, self.tags = self.m.obstacle_points('base_link', with_pucks=True)
        self.tree = cKDTree(self.obs)
        self.boxes = self.m.arm_boxes()

    def _pts(self, name, step=0.003):
        out = []
        for c in self.m.links[name].findall('collision'):
            g = c.find('geometry')[0]
            if g.tag == 'box':
                p = fc.box_surface([float(v) for v in g.get('size').split()], step)
            elif g.tag == 'cylinder':
                p = fc.cyl_surface(float(g.get('radius')), float(g.get('length')), step)
            elif g.tag == 'mesh':
                fn = g.get('filename').replace('package://waffle_4wd_description/', fc.DESC + '/')
                p = fc.sample_tris(fc.read_stl(fn) * np.array([float(v) for v in g.get('scale', '1 1 1').split()]), step)
            else:
                continue
            out.append(fc.apply(fc.origin_T(c.find('origin')), p))
        return np.vstack(out) if out else np.zeros((0, 3))

    def real(self, q6, model):
        e, k = MODELS[model]
        q = np.array(q6, float)
        q[:5] -= e
        base = q.copy()
        qm = fc.qmap(base)
        T = self.m.chain_T('gripper_roll_link', qm)
        tcp = (T @ np.array([TCP_X, 0, 0, 1]))[:3]
        for j in (2, 3):
            if k[j - 1]:
                o = self.m.chain_T(CHILD[j], qm)[:3, 3]
                q[j - 1] -= k[j - 1] * math.hypot(tcp[0] - o[0], tcp[1] - o[1])
        return list(q)

    def occlusion(self, q6, cam):
        p0, pitch, yaw = CAMS[cam]
        cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
        R = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        qm = fc.qmap(q6)
        grid = np.zeros((NY, NX), bool)
        for l, p in self.lp.items():
            if not len(p):
                continue
            c = (fc.apply(self.m.chain_T(l, qm), p) - p0) @ R
            c = c[c[:, 0] > 0.02]
            u, v = np.arctan2(-c[:, 1], c[:, 0]), np.arctan2(c[:, 2], c[:, 0])
            k = (np.abs(u) < HF) & (np.abs(v) < VF)
            grid[((VF - v[k]) / (2 * VF) * NY).astype(int).clip(0, NY - 1), ((u[k] + HF) / (2 * HF) * NX).astype(int).clip(0, NX - 1)] = True
        g = grid.copy()
        g[1:] |= grid[:-1]
        g[:-1] |= grid[1:]
        g[:, 1:] |= grid[:, :-1]
        g[:, :-1] |= grid[:, 1:]
        return g.mean()

    def clearance(self, q6):
        qm = fc.qmap(q6)
        best = (1e9, '')
        for ln, To, half in self.boxes:
            Tw = self.m.chain_T(ln, qm) @ To
            idx = self.tree.query_ball_point(Tw[:3, 3], float(np.linalg.norm(half)) + 0.05)
            if idx:
                idx = np.asarray(idx)
                d = fc.box_sdf(Tw, half, self.obs[idx])
                k = int(np.argmin(d))
                if d[k] < best[0]:
                    best = (float(d[k]), str(self.tags[idx[k]]))
        return best

    def extent(self, q6):
        qm = fc.qmap(q6)
        P = np.vstack([fc.apply(self.m.chain_T(l, qm), p) for l, p in self.lp.items() if l != 'link2'])
        return P[:, 0].max(), P[:, 2].min()

    def path(self, a, b, model, order=None):
        best = (1e9, '')
        for _, q in fc.path(a, b, order or [[0, 1, 2, 3, 4, 5]], n_per_rad=20):
            c = self.clearance(self.real(q, model))
            if c[0] < best[0]:
                best = c
        return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--q', default='0,-1.25,0.60,1.80,0,0.923', help='검사할 DRIVE (관절 1 ~ 5, 그리퍼)')
    a = ap.parse_args()
    q = [float(v) for v in a.q.split(',')]
    ck = Check()
    print('xmax = 팔 가장 앞 (base_link x, 차체 앞 0.15) · zmin = 가장 낮은 점 (base_link z, 바닥 위 +0.007)')
    for label, pose in [('예전 DRIVE (E2, FOLD_VIA)', FOLD_VIA), ('검사 자세', q)]:
        print(f'\n== {label} {pose[:4]}')
        for mn in MODELS:
            r = ck.real(pose, mn)
            xm, zm = ck.extent(r)
            c = ck.clearance(r)
            print(f'  {mn:5s} 앞 가림 {ck.occlusion(r, "front") * 100:4.1f} % · 뒤 {ck.occlusion(r, "rear") * 100:4.1f} % | '
                  f'틈 {c[0] * 1000:5.1f} mm ({c[1]}) | xmax {xm:.3f} zmin {zm:.3f}')
    print('\n== 길 최소 틈 [mm] (관절 공간 직선)')
    for mn in MODELS:
        row = {'FOLD_VIA→': ck.path(FOLD_VIA, q, mn)}
        row.update({'→' + k: ck.path(q, v, mn) for k, v in NAMED.items()})
        print(f'  {mn:5s} ' + ' · '.join(f'{k} {v[0] * 1000:5.1f} ({v[1]})' for k, v in row.items()))


if __name__ == '__main__':
    main()
