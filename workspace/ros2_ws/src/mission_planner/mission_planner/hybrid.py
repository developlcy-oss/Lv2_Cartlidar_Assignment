"""hybrid (병행) 운영 방식: 박스를 돌며 싣다가 어떤 L 의 퍽이 다 실리면 그 L 에 들러 한 번에 다 내림 (팀 결정 2026-11-12).

팔레트 = 열 스택 (뒤 열은 앞 열이 비어야 싣고 꺼냄) -> 퍽 단위 LIFO: 나중에 배달할 L 의 퍽을 먼저 배달할 L 의 퍽 위에 못 얹음.
L 배달 순서 (a, b, c) 를 정하면 가능한 경로는 꼭 이 꼴이다 (런 = 한 그룹 퍽들을 박스 순서 자유로 집는 구간):
    HOME → C1 → B1 → A → La → C2 → B2 → Lb → C3 → Lc → (verify · 복귀)
    A = a 그룹 전부, B = B1 ∪ B2, C = C1 ∪ C2 ∪ C3, C2 는 B1 이 비었을 때만 (B1 이 실려 있으면 C 를 그 위에 못 얹음)
  per_location (C1 = B1 = C2 = ∅) 과 박스 한 번 다 싣기 (C1 = C, B1 = B) 는 이 꼴의 특수한 경우.
빠르게 (Jetson): 런은 박스 단위 (같은 그룹 · 같은 박스 퍽은 한 정차에서 이어 집음).
  그룹마다 Held-Karp 표 HK[u, S, v] (u 에서 박스 집합 S 를 다 돌고 접근 자세 v 에서 끝나는 최소 비용) 를 numpy 로 한 번 만들고
  (그룹 퍽 구성이 같으면 할당이 달라도 재사용), (C1, B1) 모든 조합을 배열 연산으로 한 번에 셈. C2 만 부분집합 열거 (≤ 3^6).
비용 = 경로 비용 [m] + 3 열 칸 수 × ROW3_PENALTY_M (3 열은 싣기 · 꺼내기가 칸당 약 7 s 더 느림, #75. 0.2 m/s 로 바꾸면 약 2.8 m).
"""
from __future__ import annotations

import math
from itertools import permutations

import numpy as np

ROW3_PENALTY_M = 2.8   # [m] 3 열 칸 하나 (싣기 + 꺼내기 약 14 s × 0.2 m/s, 가정)
INF = math.inf


class Nodes:
    """비용표 (HOME · L · 박스 접근 자세) 를 한 번 만들어 둠 (World 마다 하나)."""

    def __init__(self, world, places):
        self.world = world
        self.keys = ["HOME", *places] + sorted({k for ks in world.box_opts.values() for k in ks})
        self.ix = {k: i for i, k in enumerate(self.keys)}
        n = len(self.keys)
        self.D = np.array([[world.cost(a, b) for b in self.keys] for a in self.keys])
        self.box_ix = {b: [self.ix[k] for k in ks] for b, ks in world.box_opts.items()}
        self.n = n
        self.hk_cache = {}


def _hk(nodes, boxes):
    """boxes (박스 번호 목록, 집합 비트 순서) -> HK[u, S, v] (S = 비트마스크). S = ∅ 은 u 에 머묾 (HK[u, 0, u] = 0)."""
    key = tuple(boxes)
    if key in nodes.hk_cache:
        return nodes.hk_cache[key]
    n, m = nodes.n, len(boxes)
    D = nodes.D
    hk = np.full((n, 1 << m, n), INF)
    hk[np.arange(n), 0, np.arange(n)] = 0.0
    for S in range(1 << m):
        cur = hk[:, S, :]                        # (u, v)
        if not np.isfinite(cur).any():
            continue
        for j in range(m):
            if S >> j & 1:
                continue
            T = S | 1 << j
            for k in nodes.box_ix[boxes[j]]:
                cand = (cur + D[:, k][None, :]).min(axis=1)    # u -> (S 끝 v) -> k
                hk[:, T, k] = np.minimum(hk[:, T, k], cand)
    nodes.hk_cache[key] = hk
    return hk


def _hk_path(nodes, boxes, hk, u, S, v):
    """HK 표에서 u → (S 박스들) → v 순서를 되찾음 -> [(박스, 접근 키), …]."""
    out, D = [], nodes.D
    while S:
        for j in range(len(boxes)):
            if not (S >> j & 1) or v not in nodes.box_ix[boxes[j]]:
                continue
            P = S & ~(1 << j)
            pv = int(np.argmin(hk[u, P, :] + D[:, v])) if P else u
            if abs(hk[u, P, pv] + D[pv, v] - hk[u, S, v]) < 1e-6:
                break
        else:
            raise RuntimeError("hybrid: 경로 되찾기 실패")
        out.append((boxes[j], nodes.keys[v]))
        S, v = P, pv
    return out[::-1]


def _group_boxes(ps):
    """그룹 퍽 -> (박스 목록, 박스별 퍽 수 배열, 박스 -> 퍽 목록)"""
    by = {}
    for p in ps:
        by.setdefault(p.box, []).append(p)
    boxes = sorted(by)
    return boxes, np.array([len(by[b]) for b in boxes]), by


def _subset_counts(cnt):
    """부분집합 S -> 퍽 수"""
    m = len(cnt)
    out = np.zeros(1 << m, dtype=int)
    for S in range(1, 1 << m):
        j = (S & -S).bit_length() - 1
        out[S] = out[S & (S - 1)] + cnt[j]
    return out


def plan_hybrid(world, alloc, group_of_L, nodes=None, row3_penalty=ROW3_PENALTY_M):
    """-> (비용, 사건 목록, verify 순서, 정보) 또는 None.
    사건 = ("pick", 접근 키, [퍽, …]) | ("place", L). 정보 = dict(perm, path_m, row3, peak)."""
    from .planner import _verify_and_return
    places = list(group_of_L)
    nodes = nodes or Nodes(world, places)
    D, ix = nodes.D, nodes.ix
    H = ix["HOME"]
    best = None
    for perm in permutations(places):
        La, Lb, Lc = (ix[L] for L in perm)
        (bA, cA, byA), (bB, cB, byB), (bC, cC, byC) = (_group_boxes(alloc[group_of_L[L]]) for L in perm)
        hA, hB, hC = _hk(nodes, bA), _hk(nodes, bB), _hk(nodes, bC)
        fA, fB, fC = (1 << len(bA)) - 1, (1 << len(bB)) - 1, (1 << len(bC)) - 1
        nA, nB, nC = int(cA.sum()), _subset_counts(cB), _subset_counts(cC)
        vr, vorder = _verify_and_return(world, perm[2], places)
        # 앞부분 F[C1, B1] = min HOME →C1→ v →B1→ w →A→ x → La
        tailA = (hA[:, fA, :] + D[:, La][None, :]).min(axis=1)                    # (w,)
        TB = (hB + tailA[None, None, :]).min(axis=2)                               # (v, B1)
        F = (hC[H, :, :][:, :, None] + TB[None, :, :]).min(axis=1)                 # (C1, B1)
        # 뒷부분: tailC[R] = Lb →R→ z → Lc
        tailC = (hC[ix[perm[1]], :, :] + D[:, Lc][None, :]).min(axis=1)            # (R,)
        gB = (hB[La, :, :] + D[:, Lb][None, :]).min(axis=1)                        # (B2,) La →B2→ Lb (C2 = ∅)
        T2 = (hB[:, fB, :] + D[:, Lb][None, :]).min(axis=1)                        # (x,) x →B→ Lb
        X = (hC[La, :, :] + T2[None, :]).min(axis=1)                               # (C2,) La →C2→ x →B→ Lb
        G0 = np.full(fC + 1, INF)
        G0c2 = np.zeros(fC + 1, dtype=int)
        for R in range(fC + 1):                                                    # B1 = ∅: C2 ⊆ R
            sub = R
            while True:
                c = X[sub] + tailC[R & ~sub]
                if c < G0[R]:
                    G0[R], G0c2[R] = c, sub
                if sub == 0:
                    break
                sub = (sub - 1) & R
        C1s = np.arange(fC + 1)
        B1s = np.arange(fB + 1)
        R = fC & ~C1s                                                              # 남은 C
        total = F + np.where(B1s[None, :] == 0, G0[R][:, None], gB[fB & ~B1s][None, :] + tailC[R][:, None])
        # 3 열: 동시에 실린 퍽 최대 − 8 (La 직전 C1 + B1 + A, Lb 직전 C1 + C2 + B, Lc 직전 C)
        h1 = nC[C1s][:, None] + nB[B1s][None, :] + nA
        c2n = np.where(B1s[None, :] == 0, nC[G0c2[R]][:, None], 0)
        h2 = nC[C1s][:, None] + c2n + nB[fB]
        peak = np.maximum(np.maximum(h1, h2), nC[fC])
        row3 = np.maximum(0, peak - 8)
        score = total + vr + row3 * row3_penalty
        i, j = np.unravel_index(int(np.argmin(score)), score.shape)
        if not np.isfinite(score[i, j]):
            continue
        if best is None or score[i, j] < best[0]:
            c2 = int(G0c2[fC & ~i]) if j == 0 else 0
            best = (float(score[i, j]), perm, int(i), int(j), c2, float(total[i, j] + vr), int(row3[i, j]), int(peak[i, j]), vorder)
    if best is None:
        return None
    score, perm, C1, B1, C2, path_m, row3, peak, vorder = best
    events = _rebuild(nodes, alloc, group_of_L, perm, C1, B1, C2)
    return score, events, vorder, dict(perm=list(perm), path_m=round(path_m, 3), row3=row3, peak=peak)


def _rebuild(nodes, alloc, group_of_L, perm, C1, B1, C2):
    """고른 런 집합으로 실제 순서 (런마다 시작 · 끝 접근 자세) 를 되찾음."""
    D, ix = nodes.D, nodes.ix
    gA, gB, gC = (_group_boxes(alloc[group_of_L[L]]) for L in perm)
    fA, fB, fC = ((1 << len(g[0])) - 1 for g in (gA, gB, gC))
    C3 = fC & ~C1 & ~C2
    B2 = fB & ~B1
    runs = [(gC, C1), (gB, B1), (gA, fA), ("L", perm[0]), (gC, C2), (gB, B2), ("L", perm[1]), (gC, C3), ("L", perm[2])]
    # 앞으로: 노드마다 최소 비용 (런 끝 노드 = v), 뒤로 되찾기
    n = nodes.n
    cost = np.full(n, INF)
    cost[ix["HOME"]] = 0.0
    back = []
    for g, S in runs:
        if g == "L":
            t = ix[S]
            arg = int(np.argmin(cost + D[:, t]))
            new = np.full(n, INF)
            new[t] = cost[arg] + D[arg, t]
            back.append(("L", S, np.full(n, arg)))
        else:
            hk = _hk(nodes, g[0])
            M = cost[:, None] + hk[:, S, :]                 # (u, v)
            arg = M.argmin(axis=0)
            new = M.min(axis=0)
            back.append((g, S, arg))
        cost = new
    v = int(np.argmin(cost))
    seq = []
    for g, S, arg in reversed(back):
        u = int(arg[v])
        if g == "L":
            seq.append(("place", S))
        else:
            boxes, _, by = g
            hk = _hk(nodes, boxes)
            for b, key in reversed(_hk_path(nodes, boxes, hk, u, S, v)):
                ext = nodes.world.approaches[key].extra["pucks"]
                seq.append(("pick", key, sorted(by[b], key=lambda p: ext[p.side]["dist"])))   # 싣는 순서 = 칸 배정 순서
        v = u
    seq.reverse()
    return seq


def assign_slots(events, owner, row_order):
    """LIFO 스택 위치로 팔레트 칸 정하기 -> {퍽 id: (열, 칸)}. owner = {퍽 id: L}.
    퍽 p 의 A_p = 실려 있는 동안 p 위에 놓인 퍽 수의 최대. 열 = A_p // 4 + 1 (위 = 앞 열), 열 안 칸 = row_order[3 − A_p % 4]
    (같은 열은 깊은 퍽부터 실음 = row_order 순서). 아래 퍽일수록 뒤 열이라 싣고 꺼낼 때 앞 열은 늘 비어 있음
    (위 퍽의 A 는 아래 퍽보다 작고, 같은 열에 동시에 있는 퍽은 A 가 서로 달라 4 개를 넘지 않음)."""
    stack, above = [], {}
    for ev in events:
        if ev[0] == "pick":
            for p in ev[2]:
                stack.append(p.id)
                above[p.id] = 0
                for k, q in enumerate(stack):
                    above[q] = max(above[q], len(stack) - 1 - k)
        else:
            mine = [q for q in stack if owner[q] == ev[1]]
            if stack[len(stack) - len(mine):] != mine:
                raise RuntimeError(f"hybrid: {ev[1]} 퍽이 스택 맨 위가 아님 {stack}")
            stack = stack[:len(stack) - len(mine)]
    if max(above.values(), default=0) >= 12:
        raise RuntimeError("hybrid: 팔레트 12 칸 초과")
    return {q: (a // 4 + 1, row_order[3 - a % 4]) for q, a in above.items()}
