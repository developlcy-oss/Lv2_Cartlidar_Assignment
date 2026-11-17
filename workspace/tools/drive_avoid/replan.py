#!/usr/bin/env python3
"""우회 경로 (2026-11-17): 규칙 v2 계획기 (plan_v2.py, 길 모음과 같은 설정) 에 장애물 점을 더해 지금 자리 → 정지점 길을 다시 계산.

  from replan import Replanner
  rp = Replanner()
  pieces, info = rp.plan((x, y, th), (gx, gy, gth|None), obstacle_xy)   # 실패면 (None, 이유)

- 계획기 · 길 모음 설정은 노트북 바탕화면 2026-11-14_규칙v2계획 (PLAN_DIR 환경 변수로 바꿈) 의 plan_v2.py · 길모음/make_legs.py 를 그대로 씀
  (계획 반경 ≥ 0.25 · 후진 ≤ 0.30 · 제자리 ≤ 75° · 실측 상자 6 + L 3). ★ Jetson 에서 쓰려면 이 두 파일과 2026-11-12_경로/seg_view.py 를 같이 복사
- 장애물 점: 라이다로 본 앞면 점 + 그 뒤 (로봇에서 멀어지는 쪽) 0.15 m 까지 그림자 점 (상자 20.5 cm · 사람 발 깊이를 몰라서), 반지름 OBST_R
- 벽 여유: 다시 계획은 0.03 m (원래 길은 0.05 ~ 0.07). 로봇이 이미 벽 가까이 멈춰 있을 수 있어서. 장애물은 OBST_R 이 여유를 겸함
- 펼침 상한 MAX_EXP (기본 100 000 ≈ 20 s): 통로가 막혀도 2D 로는 이어져 보이면 계획기가 오래 헤맴 (11-17 시험: 85 s 뒤 실패) → 넘으면 '우회 없음'
"""
from __future__ import annotations

import math
import os
import sys
import time

import numpy as np

PLAN_DIR = os.environ.get("PLAN_DIR", os.path.expanduser("~/Desktop/CartRider/2026-11-14_규칙v2계획"))
sys.path.insert(0, PLAN_DIR)
sys.path.insert(0, os.path.join(PLAN_DIR, "길모음"))
import plan_v2 as pv  # noqa: E402
import make_legs as ml  # noqa: E402

OBST_R = 0.06       # 장애물 점 반지름 [m]
SHADOW = 0.15       # 앞면 점 뒤로 그림자 길이 [m]
CLEAR_REPLAN = 0.03
MAX_EXP = int(os.environ.get("REPLAN_MAX_EXP", "100000"))


class Replanner:
    def __init__(self):
        ml.setup()                          # 길 모음과 같은 계획 설정 (반경 · 예산 · 실측 상자)
        pv.CLEAR = CLEAR_REPLAN
        pv.MAX_EXPAND = MAX_EXP
        self.F = pv.Field()
        P = ml.points()
        self.stops = [tuple(v["goal"][:2]) for k, v in P.items() if k != "HOME"] + [tuple(P["HOME"]["start"][:2])]
        self.points = P                     # 이름 → {"start", "goal"} (HOME goal = A1 칸 가운데, 방향 자유)

    def _with_obstacles(self, pts):
        if not len(pts):
            return self.F
        F = self.F
        d, dt = F.d.copy(), F.dt.copy()
        for px, py in pts:
            R = 0.6
            i0, i1 = max(0, int((px - R) / F.res)), min(F.nx, int((px + R) / F.res) + 2)
            j0, j1 = max(0, int((py - R) / F.res)), min(F.ny, int((py + R) / F.res) + 2)
            if i0 >= i1 or j0 >= j1:
                continue
            xs = (np.arange(i0, i1) * F.res)[:, None]
            ys = (np.arange(j0, j1) * F.res)[None, :]
            dd = (np.hypot(xs - px, ys - py) - OBST_R).astype(np.float32)
            np.minimum(d[i0:i1, j0:j1], dd, out=d[i0:i1, j0:j1])
            np.minimum(dt[i0:i1, j0:j1], dd, out=dt[i0:i1, j0:j1])   # 사람 · 키 큰 장애물: 팔 앞 끝도 피함
        G = pv.Field.__new__(pv.Field)
        G.__dict__.update(F.__dict__)
        G.d, G.dt = d, dt
        return G

    @staticmethod
    def obstacle_points(front_pts, robot_xy):
        """앞면 점 → 앞면 + 그림자 점 (로봇에서 멀어지는 쪽), 2 cm 칸으로 줄임"""
        if not len(front_pts):
            return np.zeros((0, 2))
        P = np.asarray(front_pts, dtype=float)
        v = P - np.asarray(robot_xy, dtype=float)[None]
        u = v / np.maximum(np.hypot(v[:, 0], v[:, 1]), 1e-6)[:, None]
        allp = [P + u * s for s in np.arange(0.0, SHADOW + 1e-9, 0.05)]
        Q = np.vstack(allp)
        key = np.round(Q / 0.02).astype(int)
        _, idx = np.unique(key, axis=0, return_index=True)
        return Q[np.sort(idx)]

    def plan(self, start, goal, obstacle_xy, goal_stop=None):
        """start (x, y, th) → goal (x, y, th|None). 반환: (조각 목록, 정보) 또는 (None, 이유)"""
        t0 = time.time()
        goal_stop = goal[2] is not None if goal_stop is None else goal_stop
        F = self._with_obstacles(obstacle_xy)
        if float(F.clear(np.array([start]))[0]) < 0.0:
            return None, "지금 자리가 장애물 · 벽과 겹침 (계획 시작 불가)"
        G0 = pv.Grid2D(F, goal, None, None)
        i, j = G0.cell(start[0], start[1])
        near_ok = np.isfinite(G0.h[max(0, i - 4):i + 5, max(0, j - 4):j + 5]).any()
        if not near_ok:
            return None, f"2D 로도 길 없음 (막힌 통로) · {time.time() - t0:.1f} s"
        tries = []
        for band in (0.70, None):
            poly = ml.guide(F, start, goal) if band is not None else None
            res = pv.plan_leg(F, tuple(start), tuple(goal), goal_stop, poly, band, (0.0, 0.0, 0.0, 0.0), self.stops)
            tries.append(f"띠 {band}: {'찾음' if res else '못 찾음'}")
            if res:
                path, (end_pose, _), n_exp = res
                finish = tuple(goal) if goal_stop else None
                rows = pv.timeline(tuple(start), path, finish, 0, 0)
                pieces = ml.to_pieces(rows, tuple(start))
                L = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(rows, rows[1:]))
                return pieces, {"desc": pv.describe(path), "len_m": round(L, 2), "plan_s": round(time.time() - t0, 1),
                                "expand": n_exp, "time_s": round(rows[-1][0], 1), "tries": tries}
        return None, f"규칙 v2 로 길 없음 ({', '.join(tries)}, 펼침 상한 {MAX_EXP}) · {time.time() - t0:.1f} s"


if __name__ == "__main__":   # 빠른 시험: python3 replan.py  (S2 box2→box6 중간에 상자)
    import json
    rp = Replanner()
    lib = json.load(open(os.path.join(PLAN_DIR, "길모음", "legs.json")))
    L = next(g for g in lib["legs"] if g["name"] == "box3>box5")
    allp = [p for pc in L["pieces"] if "poses" in pc for p in pc["poses"]]
    mid, st = allp[len(allp) // 2], allp[len(allp) // 4]
    front = [(mid[0] + dx, mid[1] + dy) for dx in np.linspace(-0.1, 0.1, 5) for dy in (-0.1, 0.1)]
    obs = rp.obstacle_points(front, st[:2])
    pcs, info = rp.plan(tuple(st), tuple(L["goal"]), obs)
    print(L["name"], "→", info if pcs is None else f"{info['desc']} · {info['plan_s']} s · 조각 {len(pcs)}")
