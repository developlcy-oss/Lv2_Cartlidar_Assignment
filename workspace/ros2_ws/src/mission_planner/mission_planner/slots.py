"""팔레트 칸 배정 (#230): 싣기 · 내리기 차례는 그대로 두고, 퍽마다 칸을 고름. 세 방식 (hybrid · per_location · single_load) 공용.

지키는 것 (꼭):
  - 열 규칙: 앞 열 (번호 작은 열) 에 퍽이 있으면 뒤 열 칸을 싣거나 꺼낼 수 없음 (팔레트 문서 규칙).
  - row_rule="fixed" 면 같은 열 안도 ROW_ORDER 차례로 싣고 반대로 꺼냄 (예전 규칙). 기본 "free" 는 열 안 차례 자유.
될수록 (사전식 순서로 작게):
  1. 모양이 다른 칸 수 + 막힌 채 꺼내는 수 — 궤적 파일 칸 모양 (SLOT_SHAPE). 원기둥을 꺼낼 때 이웃 직육면체 칸 모서리가
     손가락을 막아 모양 무시면 꺼내기 → 놓기 79 %, 모양대로 + 막는 쌍 (BLOCKERS) 비우고 꺼내면 98 % (Isaac 모의, #230).
     막는 칸이 다른 장소 퍽 (나중에 내림) 으로 남는 배정을 피함. 같이 내리는 퍽끼리는 꺼내는 차례로 풂.
  2. 3 열 칸 수 (싣기 · 꺼내기 칸당 약 7 s 더 느림, #75).
  3. 예전 배정과 다른 정도 (스택 깊이로 정한 열과의 차이 + 열 안 ROW_ORDER 를 어긴 수).
꺼내는 차례 (같은 장소에서 함께 내리는 퍽): 막는 칸이 빈 퍽부터, 그다음 앞 열 · 열 안 UNLOAD_PREF (#230) — 규칙이 허락하는 한.
찾기: 분기 한정 깊이 우선 (예전 배정을 처음 상한으로), 노드 budget 에서 멈추고 그때까지 가장 좋은 것.
  열 안 차례 자유면 퍽마다 열 (3 가지) 만 분기하고 열 안 칸은 (모양 다름 + 이미 실린 퍽과 생길 막는 쌍) 이 가장 적은 빈 칸.
"""
from __future__ import annotations

ROWS = (1, 2, 3)
ROW_ORDER = [1, 2, 0, 3]   # 같은 열 안 싣는 차례 (pallet_traj.py: J1 각이 큰 칸부터)
# control/config/pallet_place_traj.json slots[].shape 와 같음 (test_slots 가 확인)
SLOT_SHAPE = {(1, 0): "cylinder", (1, 1): "box", (1, 2): "cylinder", (1, 3): "box",
              (2, 0): "box", (2, 1): "cylinder", (2, 2): "box", (2, 3): "cylinder",
              (3, 0): "cylinder", (3, 1): "box", (3, 2): "cylinder", (3, 3): "box"}
UNLOAD_PREF = {1: [3, 0, 2, 1], 2: [2, 0, 3, 1], 3: [3, 1, 0, 2]}   # 열 안 꺼내기 차례 (#230)
# 막는 쌍 (#230 팔 쪽 답, Isaac 모의): 칸 -> 그 칸을 꺼내기 전에 비어 있어야 하는 이웃 칸.
# 원기둥 칸은 집게 닫는 축 쪽 이웃 직육면체 모서리가 벌린 손가락을 막고, 1_1 (직육면체) 을 꺼낼 때는 1_0 을 침.
BLOCKERS = {(3, 0): ((3, 1),), (3, 2): ((3, 1), (3, 3)), (2, 1): ((2, 2), (2, 0)), (2, 3): ((2, 2),),
            (1, 2): ((1, 3),), (1, 1): ((1, 0),)}
BUDGET = 20000


def stack_slots(row_order, avoid=()):
    """스택 위치 A (0 = 맨 위) -> 칸: 앞 열부터, 열 안은 row_order 뒤 칸부터 (깊은 퍽이 row_order 앞 칸). avoid 칸은 뺌 (#219)"""
    return [(r, i) for r in ROWS for i in reversed(row_order) if (r, i) not in set(avoid)]


def stack_depth(events):
    """events = [("pick", 퍽) | ("place", L, [퍽, …])] -> {퍽 id: A} (실려 있는 동안 그 퍽 위에 놓인 퍽 수의 최대)."""
    stack, above = [], {}
    for ev in events:
        if ev[0] == "pick":
            stack.append(ev[1].id)
            above[ev[1].id] = 0
            for k, q in enumerate(stack):
                above[q] = max(above[q], len(stack) - 1 - k)
        else:
            gone = {p.id for p in ev[2]}
            stack = [q for q in stack if q not in gone]
    return above


def blocked_by(occ, slot):
    """slot 을 지금 꺼내면 막는 칸 중 차 있는 수"""
    return sum(b in occ for b in BLOCKERS.get(slot, ()))


class _Rules:
    def __init__(self, row_rule):
        if row_rule not in ("free", "fixed"):
            raise ValueError(f"row_rule 은 free | fixed ({row_rule})")
        self.fixed = row_rule == "fixed"
        self.rank = {i: k for k, i in enumerate(ROW_ORDER)}

    def can_load(self, occ, slot):
        r, i = slot
        for (rr, ii) in occ:
            if rr < r or (self.fixed and rr == r and self.rank[ii] > self.rank[i]):
                return False
        return True

    def can_take(self, occ, slot):
        r, i = slot
        for (rr, ii) in occ:
            if (rr, ii) == slot:
                continue
            if rr < r or (self.fixed and rr == r and self.rank[ii] > self.rank[i]):
                return False
        return True

    def unload_order(self, occ, slots):
        """slots (함께 내리는 칸들) 를 꺼낼 차례 -> (칸 목록, 막힌 채 꺼내는 수), 못 꺼내면 (None, 0). occ 는 바꾸지 않음.
        열 규칙이 허락하는 칸 중 막는 칸이 빈 것부터 (같으면 앞 열 · UNLOAD_PREF). 꺼내도 다른 칸이 새로 막히지는 않아 탐욕으로 충분."""
        occ, todo, out, blocked = dict(occ), set(slots), [], 0
        while todo:
            ok = [s for s in todo if self.can_take(occ, s)]
            if not ok:
                return None, 0
            s = min(ok, key=lambda s: (blocked_by(occ, s) > 0, s[0], UNLOAD_PREF[s[0]].index(s[1])))
            blocked += blocked_by(occ, s) > 0
            out.append(s)
            todo.discard(s)
            del occ[s]
        return out, blocked


def _cost(events, slot_of, pref_row, rules):
    """배정 하나의 값 (모양 다른 수 + 막힌 채 꺼내는 수, 3 열 수, 예전과 다른 정도), 규칙 어기면 None."""
    occ, mis, row3, dev = {}, 0, 0, 0
    for ev in events:
        if ev[0] == "pick":
            p, s = ev[1], slot_of[ev[1].id]
            if s in occ or not rules.can_load(occ, s):
                return None
            mis += SLOT_SHAPE[s] != p.shape
            row3 += s[0] == 3
            dev += abs(s[0] - pref_row[p.id]) + sum(1 for (rr, ii) in occ if rr == s[0] and rules.rank[ii] > rules.rank[s[1]])
            occ[s] = p.id
        else:
            order, b = rules.unload_order(occ, [slot_of[p.id] for p in ev[2]])
            if order is None:
                return None
            mis += b
            for s in order:
                del occ[s]
    return (mis, row3, dev)


def assign(events, avoid=(), row_rule="free", shape=True, budget=BUDGET, init=None):
    """-> (칸 {퍽 id: (열, 칸)}, 꺼내기 차례 {사건 번호: [퍽 id, …]}, 정보 dict(mismatch, row3, nodes, complete)).
    events = [("pick", 퍽) | ("place", L, [퍽, …])] (퍽 = allocation.Puck, .id · .shape). 칸이 모자라거나 규칙상 안 되면 RuntimeError.
    shape=False 면 모양을 안 봄 (예전처럼). init = 방식이 원래 정한 배정 {퍽 id: 칸} (처음 상한 · 열 기준),
    없으면 스택 깊이 배정 (LIFO 차례인 hybrid · per_location 에 맞음)."""
    rules = _Rules(row_rule)
    avoid = set(map(tuple, avoid))
    free = [s for s in SLOT_SHAPE if s not in avoid]
    if init is None:
        depth = stack_depth(events)
        seq = stack_slots(ROW_ORDER, avoid)
        if max(depth.values(), default=0) >= len(seq):
            raise RuntimeError(f"쓸 수 있는 팔레트 칸 {len(seq)} 개 초과")
        init = {q: seq[a] for q, a in depth.items()}          # 예전 (스택 깊이) 배정
    old = {q: tuple(s) for q, s in init.items()}              # 처음 상한
    pref_row = {q: s[0] for q, s in old.items()}

    def key(c):
        return c if shape else (0,) + c[1:]

    best_c = _cost(events, old, pref_row, rules)
    best = (key(best_c), dict(old)) if best_c is not None else (None, None)
    nodes = [0]
    out_at = {p.id: k for k, ev in enumerate(events) if ev[0] == "place" for p in ev[2]}   # 퍽을 내리는 사건 번호

    def conflicts(occ, s, q):
        """q 를 s 에 실으면 같은 열 이미 실린 퍽과 생길 막는 쌍 수 (다른 장소에서 내리는 것만, 같이 내리면 차례로 풂)"""
        n = 0
        for o, oq in occ.items():
            if o[0] != s[0] or out_at[oq] == out_at[q]:
                continue
            first, later = (o, s) if out_at[oq] < out_at[q] else (s, o)   # first 를 먼저 꺼냄, later 는 남아 있음
            n += later in BLOCKERS.get(first, ())
        return n

    def dfs(k, occ, where, mis, row3, dev):
        nonlocal best
        if nodes[0] >= budget:
            return
        nodes[0] += 1
        cur = (mis if shape else 0, row3, dev)
        if best[0] is not None and cur >= best[0]:
            return
        if k == len(events):
            best = (cur, dict(where))
            return
        ev = events[k]
        if ev[0] == "place":
            order, b = rules.unload_order(occ, [where[p.id] for p in ev[2]])
            if order is None:
                return
            nocc = dict(occ)
            for s in order:
                del nocc[s]
            dfs(k + 1, nocc, where, mis + (b if shape else 0), row3, dev)
            return
        p = ev[1]
        cands = []
        for s in free:
            if s in occ or not rules.can_load(occ, s):
                continue
            m = SLOT_SHAPE[s] != p.shape
            d = abs(s[0] - pref_row[p.id]) + sum(1 for (rr, ii) in occ if rr == s[0] and rules.rank[ii] > rules.rank[s[1]])
            c = conflicts(occ, s, p.id) if shape else 0
            cands.append(((m if shape else 0) + c, s[0] == 3, d, s, m if shape else 0))
        cands.sort()
        if not rules.fixed:   # 열 안 차례 자유면 열마다 가장 좋은 칸 하나만 (분기 3 개, 칸마다 분기하면 budget 에 걸려 더 나쁨)
            seen, one = set(), []
            for c in cands:
                if c[3][0] not in seen:
                    seen.add(c[3][0])
                    one.append(c)
            cands = one
        for _, r3, d, s, m in cands:
            occ[s] = p.id
            where[p.id] = s
            dfs(k + 1, occ, where, mis + m, row3 + r3, dev + d)
            del occ[s]
            del where[p.id]

    dfs(0, {}, {}, 0, 0, 0)
    if best[1] is None:
        raise RuntimeError(f"팔레트 칸 배정 못 함 (규칙 {row_rule}, 칸 {len(free)})")
    slot_of = best[1]
    unload, occ, blk = {}, {}, []
    for k, ev in enumerate(events):
        if ev[0] == "pick":
            occ[slot_of[ev[1].id]] = ev[1].id
        else:
            order, _ = rules.unload_order(occ, [slot_of[p.id] for p in ev[2]])
            unload[k] = []
            for s in order:
                if blocked_by(occ, s):
                    blk.append(occ[s])
                unload[k].append(occ.pop(s))
    shapes = {e[1].id: e[1].shape for e in events if e[0] == "pick"}
    mis = sum(SLOT_SHAPE[s] != shapes[q] for q, s in slot_of.items())
    return slot_of, unload, dict(mismatch=mis, blocked=len(blk), blocked_ids=sorted(blk), row3=sum(s[0] == 3 for s in slot_of.values()),
                                 nodes=nodes[0], complete=nodes[0] < budget)
