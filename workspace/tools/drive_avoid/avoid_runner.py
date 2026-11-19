#!/usr/bin/env python3
"""주행 관리자 (2026-11-17): 길 모음 (규칙 v2 로 미리 계산한 길) 을 차례로 따라가며 장애물이 나오면
  1. 바로 멈춤 (정지 관문이 바퀴 명령 0)  2. 그 자리에서 자기 위치 다시 찾기 (AMCL 제자리 갱신)
  3. WAIT_S (3 s) 동안 길 띠를 다시 봄 — CLEAR_S 동안 비어 있으면 '치워짐'
  4. 치워짐 → 남은 원래 길 그대로 · 그대로 있음 → 장애물을 넣고 다시 계획 (원래 길로 합류하는 짧은 우회 → 안 되면 정지점까지)
     → 그래도 길이 없으면 그 정지점을 건너뛰고 다음 정지점을 먼저 감 → 그 다음 건너뛴 정지점으로 돌아갔다가 계속
정지점마다 DWELL_S 동안 섬 ("확인" = 서기만, 2026-11-17 사용자).

  python3 avoid_runner.py --route "box1 box2 box6 box3 box5 box4 L3 L2 L1 HOME" --out <기록 폴더> [--gz] [--set-initial]

경기 도구 (../drive_demo/demo_follow.py) 의 조각 따라가기 (FollowPath · Spin) · 경로 이탈 멈춤 · 자기 위치 찾기를 그대로 불러 씀 (그 파일은 안 고침).
입력: /avoid/status (obstacle_guard) · TF map → base_link · /amcl_pose
출력: /avoid/path_ahead (지금 조각, 감시용) · /avoid/hold (멈춤 요청 → stop_gate) · /avoid/state (JSON, 상태 · 시뮬 시험이 들음)
기록: <out>/track.csv · legs.json · events.json · summary.txt
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "drive_demo"))
import demo_follow as df  # noqa: E402  (경기 도구 — 불러 쓰기만)

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav2_msgs.action import FollowPath, Spin  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

OBST = "장애물"
BLOCKED = "막힘 (우회 필요)"
WAIT_S, CLEAR_S, STOP_SETTLE = 3.0, 0.5, 0.5     # 판단 시간 (사용자 3 s) · 이만큼 비어 있으면 치워짐 · 멈춘 뒤 기다림 [s]
DWELL_S = 2.0                                    # 정지점에서 서는 시간 [s] ("확인")
MIN_PTS = 5                                      # obstacle_guard min_points 와 같게
MAX_REPLAN, MAX_SKIP = 2, 2                      # 구간 하나에서 다시 계획 횟수 · 정지점 하나를 건너뛸 수 있는 횟수
REJOIN_BACK = (0.45, 0.80)                       # 합류점 = 장애물 뒤 이만큼 [m] (차례로 시도)
NODE = None
ORIG_RUN_ACTION = df.run_action


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class AvoidNode(df.Follower):
    def __init__(self, sim_time):
        super().__init__(sim_time)
        self.sim = False   # demo_follow.relocalize 가 AMCL 제자리 갱신을 하도록 (Gazebo · 실물 모두 AMCL 있음). 시각은 위에서 이미 sim
        self.avoid = True
        self.pub_path = self.create_publisher(Path, "/avoid/path_ahead", 10)
        self.pub_hold = self.create_publisher(Bool, "/avoid/hold", 10)
        self.pub_state = self.create_publisher(String, "/avoid/state", 10)
        self.pub_leg = self.create_publisher(Path, "/avoid/leg_plan", 10)   # 구간 전체 길 (시뮬 시험이 장애물 놓을 자리를 고름)
        self.status, self.status_wall = None, 0.0
        self.hist = deque(maxlen=40)    # (벽시계, 띠 안 점 수, 둘레 모르는 점)
        self.hold = False
        self.leg_name, self.state_txt = "", "준비"
        self.spin_target = None
        self.events = []
        self.plans = []        # (모의 시각, 구간 번호, [[x, y], ...]) — 주행기록 HTML 이 커서 시각의 계획으로 그림
        self.legs_meta = []    # (구간 번호, 이름, 목표 이름, 목표 자세)
        self.create_subscription(String, "/avoid/status", self._status, 10)
        self.create_timer(0.1, lambda: self.pub_hold.publish(Bool(data=self.hold)))

    def pose(self):
        """map → base_link = 최신 map → odom (AMCL) ∘ 최신 odom → base_link (EKF). 경기 도구 pose() 는 한 번에 찾아서, AMCL 이 늦으면
        '가장 최근 공통 시각' (AMCL 마지막 갱신) 자세가 나옴 (11-17 밤 감시에서 1 ~ 2 s 묵은 자세 확인)"""
        from rclpy.time import Time
        try:
            ob = self.tf.lookup_transform("odom", "base_link", Time())
            mo = self.tf.lookup_transform("map", "odom", Time())
        except Exception:  # noqa: BLE001
            return super().pose()
        yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))  # noqa: E731
        a, b = yaw(mo.transform.rotation), yaw(ob.transform.rotation)
        bx, by = ob.transform.translation.x, ob.transform.translation.y
        c, s_ = math.cos(a), math.sin(a)
        return (mo.transform.translation.x + c * bx - s_ * by, mo.transform.translation.y + s_ * bx + c * by, wrap(a + b))

    def _status(self, m):
        try:
            st = json.loads(m.data)
        except ValueError:
            return
        self.status, self.status_wall = st, time.time()
        self.hist.append((self.status_wall, st["n"], st["near"]))

    def record_plan(self, k, pieces):
        pts = [[round(q[0], 3), round(q[1], 3)] for pc in pieces if pc["kind"] != "S" for q in pc["poses"][::3]]
        if pts:
            self.plans.append({"t": round(self.now_s(), 2), "step": k, "pts": pts})
            m = Path()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            for pc in pieces:
                if pc["kind"] == "S":
                    continue
                for q in pc["poses"]:
                    ps = PoseStamped()
                    ps.pose.position.x, ps.pose.position.y = q[0], q[1]
                    m.poses.append(ps)
            self.pub_leg.publish(m)

    def set_hold(self, v):
        self.hold = bool(v)
        self.pub_hold.publish(Bool(data=self.hold))

    def state(self, s, **kw):
        self.state_txt = s
        d = {"state": s, "leg": self.leg_name, "t": round(self.now_s(), 2), **kw}
        self.pub_state.publish(String(data=json.dumps(d, ensure_ascii=False)))
        print(f"    [{s}]" + (" " + " · ".join(f"{k} {v}" for k, v in kw.items()) if kw else ""), flush=True)

    def pub_poses(self, path_msg):
        self.pub_path.publish(path_msg)

    def pub_spin(self, dth):
        p = self.pose()
        m = Path()
        m.header.frame_id = "spin"
        m.header.stamp = self.get_clock().now().to_msg()
        ps = PoseStamped()
        if p:
            ps.pose.position.x, ps.pose.position.y = p[0], p[1]
            self.spin_target = p[2] + dth
        m.poses.append(ps)
        self.pub_path.publish(m)

    def guard_off(self):
        m = Path()
        m.header.frame_id = "map"
        self.pub_path.publish(m)

    def fresh_status(self, after):
        st = self.status
        return st if (st is not None and self.status_wall > after) else None


def run_action(client, goal, timeout, done_when=None, abort_when=None):
    """경기 도구 run_action 을 감싸 장애물 멈춤을 더함 (조각을 보낼 때 그 길을 감시에 알림)"""
    n = NODE
    if isinstance(goal, FollowPath.Goal):
        n.pub_poses(goal.path)
    elif isinstance(goal, Spin.Goal):
        n.pub_spin(goal.target_yaw)
    t_pub = time.time() + 0.15   # 새 길로 판단한 감시 결과만 (한 스캔 뒤)

    def ab():
        st = n.fresh_status(t_pub)
        if n.avoid and st and st.get("blocked") and st.get("mode") != "off":
            return f"{OBST} (띠 안 점 {st['n']} 개)"
        return abort_when() if abort_when else None
    return ORIG_RUN_ACTION(client, goal, timeout, done_when, ab)


df.run_action = run_action


def trim(pc, pose):
    """조각에서 이미 지난 부분을 뺀 나머지 (멈췄다 다시 갈 때) — 남은 게 3 cm 안이면 None"""
    if pc["kind"] == "S":
        if NODE.spin_target is None or pose is None:
            return pc
        rest = wrap(NODE.spin_target - pose[2])
        return None if abs(rest) < math.radians(2) else {"kind": "S", "dth": rest}
    P = pc["poses"]
    if pose is None or len(P) < 2:
        return pc
    k = min(range(len(P)), key=lambda i: math.hypot(P[i][0] - pose[0], P[i][1] - pose[1]))
    rest = P[k:]
    L = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(rest, rest[1:]))
    return None if (len(rest) < 2 or L < 0.03) else {"kind": pc["kind"], "poses": rest}


def obstacle_stop(node, why):
    """1 ~ 3 단계: 멈춤 → 자기 위치 다시 찾기 → WAIT_S 동안 판단. 반환 'cleared' | 'blocked'"""
    node.set_hold(True)
    p0 = node.pose()
    st0 = node.status or {}
    ev = {"t": round(node.now_s(), 2), "leg": node.leg_name, "why": why, "pose": [round(v, 3) for v in p0] if p0 else None,
          "front_pts": st0.get("pts", [])[:30], "guard_why": st0.get("why", "")}
    node.state("정지", 이유=why)
    df.sleep_s(node, STOP_SETTLE)
    node.state("위치확인")
    df.relocalize(node, "장애물 앞")
    am = node.amcl
    ev["amcl_sigma_cm"] = round(am[0] * 100, 1) if am else None
    node.state("판단", 기다림=f"{WAIT_S:.0f} s")
    t0, clear_since, verdict = node.now_s(), None, "blocked"
    t_after = time.time()
    while node.now_s() - t0 < WAIT_S and not df.STOP.is_set():
        st = node.fresh_status(t_after)
        if st is not None:
            t_after = node.status_wall
            if st["n"] < MIN_PTS:
                clear_since = clear_since if clear_since is not None else node.now_s()
                if node.now_s() - clear_since >= CLEAR_S:
                    verdict = "cleared"
                    break
            else:
                clear_since = None
        time.sleep(0.02)
    ev["verdict"] = verdict
    ev["judge_s"] = round(node.now_s() - t0, 2)
    ev["stop_s"] = round(node.now_s() - ev["t"], 2)
    node.events.append(ev)
    node.state("치워짐" if verdict == "cleared" else "그대로", 판단=f"{ev['judge_s']} s")
    return verdict


def follow_guarded(node, pc):
    """조각 하나 + 장애물 처리. 치워지면 남은 조각을 이어 감 (최대 3 번). 반환 (성공, 메시지) — 우회가 필요하면 (False, BLOCKED)"""
    cur = pc
    for _ in range(4):
        node.set_hold(False)
        r, msg = df.follow_piece(node, cur)
        if r or df.STOP.is_set() or not msg.startswith(OBST):
            return r, msg
        if obstacle_stop(node, msg) != "cleared":
            return False, BLOCKED
        cur = trim(cur, node.pose())
        if cur is None:
            return True, ""
        node.state("복귀", 남은=("제자리" if cur["kind"] == "S" else f"{len(cur['poses'])} 점"))
    return False, BLOCKED


def obstacle_cloud(node, robot_xy):
    """판단 동안 본 둘레 모르는 점 (마지막 1.5 s) → 앞면 점 (2 cm 칸으로 줄임)"""
    now = time.time()
    pts = [tuple(p) for (tw, _, near) in node.hist if now - tw < 1.5 for p in near]
    if not pts:
        return []
    seen, out = set(), []
    for x, y in pts:
        k = (round(x / 0.02), round(y / 0.02))
        if k not in seen and math.hypot(x - robot_xy[0], y - robot_xy[1]) < 1.6:
            seen.add(k)
            out.append((x, y))
    return out


def detour(node, rp, rest, goal, goal_stop):
    """장애물을 넣고 다시 계획. rest = 남은 원래 조각들 (지금 조각의 남은 부분 포함).
    1) 원래 길의 장애물 뒤 지점으로 합류하는 짧은 우회 → 2) 정지점까지 전체. 반환 새 조각 목록 또는 None"""
    p = node.pose()
    if p is None:
        return None, "자기 위치 없음"
    front = obstacle_cloud(node, p[:2])
    obs = rp.obstacle_points(front, p[:2])
    node.state("우회계획", 장애물점=len(front))
    # 1) 합류: rest 의 전진 · 후진 조각을 따라 걸으며 장애물에서 0.35 m 넘게 떨어지고, 장애물 가장 먼 투영보다 REJOIN_BACK 뒤인 점
    import numpy as np
    O = np.array(front) if front else np.zeros((0, 2))
    seq = []   # (조각 번호, 점 번호, 누적 길이)
    acc, last = 0.0, (p[0], p[1])
    for i, pc in enumerate(rest):
        if pc["kind"] == "S":
            continue
        for j, q in enumerate(pc["poses"]):
            acc += math.hypot(q[0] - last[0], q[1] - last[1])
            last = (q[0], q[1])
            seq.append((i, j, acc))
    if len(O) and seq:
        def odist(q):
            return float(np.min(np.hypot(O[:, 0] - q[0], O[:, 1] - q[1])))
        s_ob = max((s for (i, j, s) in seq if odist(rest[i]["poses"][j]) < 0.30), default=None)
        if s_ob is not None:
            for back in REJOIN_BACK:
                cand = next(((i, j) for (i, j, s) in seq if s >= s_ob + back and odist(rest[i]["poses"][j]) > 0.35), None)
                if cand is None:
                    continue
                i, j = cand
                g = tuple(rest[i]["poses"][j])
                pieces, info = rp.plan(p, g, obs, goal_stop=True)
                if pieces:
                    tail = [{"kind": rest[i]["kind"], "poses": rest[i]["poses"][j:]}] if len(rest[i]["poses"]) - j >= 2 else []
                    node.state("우회", 방식=f"합류 +{back} m", 길=info["desc"], 계획=f"{info['plan_s']} s")
                    return pieces + tail + rest[i + 1:], f"합류 +{back} m: {info['desc']} ({info['plan_s']} s)"
                node.state("우회계획", 합류=f"+{back} m 실패", 이유=info)
    # 2) 정지점까지 전체
    pieces, info = rp.plan(p, goal, obs, goal_stop=goal_stop)
    if pieces:
        node.state("우회", 방식="정지점까지", 길=info["desc"], 계획=f"{info['plan_s']} s")
        return pieces, f"정지점까지: {info['desc']} ({info['plan_s']} s)"
    return None, info


def run_leg(node, rp, k, name, pieces, goal, results):
    """구간 하나 (조각 → 장애물 처리 → 정지점 방향 맞추기). 반환 'ok' | 'skip' | 'fail'"""
    node.step = k
    node.leg_name = name
    tl = node.now_s()
    node.head_fix, node.head_skip, node.recover = [], [], []
    queue = deque(pieces)
    node.record_plan(k, pieces)
    goal_stop = goal[2] is not None
    notes, replans, out = [], 0, "ok"
    node.state("주행", 조각=len(queue))
    while queue and not df.STOP.is_set():
        pc = queue.popleft()
        r, msg = follow_guarded(node, pc)
        if r:
            continue
        if msg == BLOCKED:
            if replans >= MAX_REPLAN:
                out = "skip"
                notes.append(f"다시 계획 {replans} 번 뒤에도 막힘")
                break
            replans += 1
            rest = ([trim(pc, node.pose())] if trim(pc, node.pose()) else []) + list(queue)
            new, why = detour(node, rp, rest, goal, goal_stop)
            notes.append(f"다시 계획 {replans}: {why}")
            if new is None:
                out = "skip"
                break
            queue = deque(new)
            node.record_plan(k, new)
            continue
        # 장애물이 아닌 실패 (경로 이탈 · 시간 초과 등): 자기 위치를 찾고 한 번 더, 그래도 안 되면 다시 계획
        notes.append(f"실패: {msg}")
        df.relocalize(node, "조각 실패")
        r, msg = follow_guarded(node, trim(pc, node.pose()) or pc)
        if not r:
            out = "fail"
            notes.append(f"다시 해도 실패: {msg}")
            break
    node.guard_off()
    node.set_hold(False)
    p = node.pose()
    if out == "ok" and goal_stop and p:
        err = wrap(goal[2] - p[2])
        if abs(err) > df.YAW_FIX:
            df.follow_piece(node, {"kind": "S", "dth": err})
            node.guard_off()
        df.sleep_s(node, 0.3)
        p = node.pose()
    dpos = math.hypot(goal[0] - p[0], goal[1] - p[1]) if p else 9.0
    dyaw = abs(wrap(goal[2] - p[2])) if (p and goal_stop) else 0.0
    res = {"seg": f"S{k}", "name": name, "result": out, "ok": out == "ok", "sim_s": round(node.now_s() - tl, 1),
           "end_err_cm": round(dpos * 100, 1), "end_err_deg": round(math.degrees(dyaw), 1), "replans": replans, "notes": notes,
           "head_fix_deg": list(node.head_fix)}
    results.append(res)
    mark = {"ok": "✓", "skip": "↷", "fail": "✗"}[out]
    print(f"{mark} {name}: {res['sim_s']} s · 정지점 어긋남 {res['end_err_cm']} cm · {res['end_err_deg']}°"
          + (f" · 다시 계획 {replans}" if replans else "") + (f" · {notes[-1]}" if notes and out != "ok" else ""), flush=True)
    return out


def main():
    global NODE, WAIT_S, DWELL_S
    ap = argparse.ArgumentParser()
    ap.add_argument("--route", required=True, help='정지점 이름 차례 (예 "box1 box2 L1 HOME"), 출발은 늘 HOME')
    ap.add_argument("--lib", default=os.path.join(os.environ.get("PLAN_DIR", os.path.expanduser("~/Desktop/CartRider/2026-11-14_규칙v2계획")), "길모음", "legs.json"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--gz", action="store_true", help="Gazebo 시뮬 (모의 시각)")
    ap.add_argument("--set-initial", action="store_true", help="출발 자세를 /initialpose 로 (Gazebo 는 늘)")
    ap.add_argument("--wait", type=float, default=WAIT_S)
    ap.add_argument("--dwell", type=float, default=DWELL_S)
    ap.add_argument("--no-avoid", action="store_true", help="장애물 멈춤 끔 (어제 경기처럼, 비교용)")
    a = ap.parse_args()
    WAIT_S, DWELL_S = a.wait, a.dwell
    lib = json.load(open(a.lib, encoding="utf-8"))
    LIB = {g["name"]: g for g in lib["legs"] if g.get("ok")}
    PTS = lib["points"]
    route = [x for x in a.route.split() if x]
    bad = [x for x in route if x not in PTS]
    if bad:
        raise SystemExit(f"모르는 정지점 {bad} (쓸 수 있는 이름: {' '.join(PTS)})")

    signal.signal(signal.SIGINT, lambda *_: df.STOP.set())
    os.makedirs(a.out, exist_ok=True)
    df.LIVE = open(os.path.join(a.out, "track_live.csv"), "w", encoding="utf-8", buffering=1)
    df.LIVE.write("t,x,y,yaw,v,w,step,v_nav,w_nav,loc\n")
    rclpy.init()
    node = AvoidNode(a.gz)
    node.avoid = not a.no_avoid
    NODE = node
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=lambda: ex.spin(), daemon=True).start()
    from replan import Replanner
    rp = Replanner()
    start = PTS["HOME"]["start"]
    print(f"경로 HOME → {' → '.join(route)} · 길 모음 {os.path.basename(a.lib)} ({len(LIB)} 개) · 장애물 멈춤 {'켬' if node.avoid else '끔'} · 판단 {WAIT_S} s · 정지점 서기 {DWELL_S} s", flush=True)

    # 출발: 위치 추정이 출발 자세에 맞을 때까지 (Gazebo 는 출발 자세로 놓였으므로 /initialpose 를 줌)
    t0 = time.time()
    if a.gz or a.set_initial:
        time.sleep(1.0)
        node.set_initial(*start)
    while not df.STOP.is_set():
        p, am = node.pose(), node.amcl
        if p and am and math.hypot(p[0] - start[0], p[1] - start[1]) < 0.08 and abs(wrap(p[2] - start[2])) < math.radians(8) \
                and am[0] <= df.AMCL_XY_OK and am[1] <= df.AMCL_YAW_OK:
            break
        if time.time() - t0 > 60:
            print("✗ 60 s 안에 출발 자세 · AMCL 이 안 맞음", flush=True)
            df.STOP.set()
        if a.gz and time.time() - t0 > 3 and int(time.time() - t0) % 5 == 0:
            node.set_initial(*start)
        time.sleep(0.5)
    for c, nm in ((node.fp, "follow_path"), (node.spin, "spin")):
        if not df.STOP.is_set() and not c.wait_for_server(timeout_sec=30.0):
            print(f"✗ {nm} action server 없음", flush=True)
            df.STOP.set()
    print(f"출발 자세 확인 ({time.time() - t0:.1f} s)", flush=True)

    results, visited, gave_up = [], ["HOME"], []
    todo = list(route)
    cur, at_stop, k = "HOME", True, 0
    skips = {}
    t_run = node.now_s()
    while todo and not df.STOP.is_set():
        nxt = todo[0]
        if nxt == cur and at_stop:
            todo.pop(0)
            continue
        goal = tuple(PTS[nxt]["goal"])
        key = f"{cur}>{nxt}"
        if at_stop and key in LIB:
            pieces, name = LIB[key]["pieces"], key
        else:   # 정지점이 아닌 자리 (건너뛴 뒤) → 지금 자리에서 다시 계획
            p = node.pose()
            node.leg_name = f"(여기)>{nxt}"
            node.state("계획", 목표=nxt)
            pieces, info = rp.plan(p, goal, rp.obstacle_points(obstacle_cloud(node, p[:2]), p[:2]))
            name = f"(여기)>{nxt}"
            if pieces is None:
                print(f"  ⚠ 지금 자리 → {nxt} 길 없음 ({info})", flush=True)
                out = "skip"
                results.append({"seg": f"S{k}", "name": name, "result": "skip", "ok": False, "notes": [f"계획 실패: {info}"]})
            else:
                print(f"  · 지금 자리 → {nxt}: {info['desc']} ({info['plan_s']} s)", flush=True)
        node.legs_meta.append((k, name, nxt, list(goal)))
        if pieces is not None:
            out = run_leg(node, rp, k, name, pieces, goal, results)
        k += 1
        if out == "ok":
            todo.pop(0)
            visited.append(nxt)
            cur, at_stop = nxt, True
            if nxt == "HOME":
                break
            node.state("정지점", 이름=nxt, 서기=f"{DWELL_S} s")
            df.sleep_s(node, DWELL_S)
            node.arrived.publish(String(data=nxt))
            continue
        # 건너뛰기: 다음 정지점을 먼저, 그 다음 다시 (최대 MAX_SKIP 번)
        at_stop = False
        skips[nxt] = skips.get(nxt, 0) + 1
        todo.pop(0)
        if skips[nxt] > MAX_SKIP or not todo:
            if nxt == "HOME" or not todo:
                print(f"  ✗ {nxt} 로 갈 길이 없음 — 여기서 끝", flush=True)
                gave_up.append(nxt)
                break
            print(f"  ✗ {nxt} 포기 (건너뛰기 {skips[nxt] - 1} 번 뒤에도 막힘)", flush=True)
            gave_up.append(nxt)
            continue
        todo.insert(1, nxt)
        node.state("건너뜀", 정지점=nxt, 다음=todo[0])
        print(f"  ↷ {nxt} 건너뜀 → {todo[0]} 먼저, 그 다음 {nxt}", flush=True)
    node.guard_off()
    node.set_hold(False)
    node.step = -1
    node.state("끝", 간곳=" → ".join(visited), 포기=",".join(gave_up) or "없음", 걸린시간=f"{node.now_s() - t_run:.1f} s")
    time.sleep(1.0)
    with node.lock:
        rows = list(node.rows)
    save(a, results, rows, node.events, visited, gave_up, node.now_s() - t_run, node, PTS)
    ex.shutdown()
    rclpy.try_shutdown()


def save(a, results, rows, events, visited, gave_up, total_s, node=None, PTS=None):
    if node is not None:   # 주행기록 HTML (2026-11-13_주행기록/make_html.py) 이 읽는 형식
        s0 = PTS["HOME"]["start"]
        lines = [f"# 장애물 주행 (avoid_runner, {time.strftime('%Y-%m-%d %H:%M')}): 경로 {a.route} · 실제 간 곳 {' → '.join(visited)}", "route:",
                 f"  - {{stop: signal, x: {s0[0]:.4f}, y: {s0[1]:.4f}, face: {math.degrees(s0[2]):.1f}}}"]
        for (k, name, tgt, g) in node.legs_meta:
            lines.append("  - A1   # 출발 구역" if tgt == "HOME" else
                         f"  - {{stop: {tgt}, x: {g[0]:.4f}, y: {g[1]:.4f}, face: {math.degrees(g[2]):.1f}}}" + ("   # 다시 감 (건너뛴 뒤)" if name.startswith("(") else ""))
        open(os.path.join(a.out, "avoid_route.yaml"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
        cr = [{"seg": r["seg"], "name": r["name"], "points": [i, i + 1], "ok": r["ok"],
               "tries": [{"status": "ok" if r["ok"] else r["result"], "msg": " / ".join(r.get("notes", []))}]} for i, r in enumerate(results)]
        json.dump(cr, open(os.path.join(a.out, "course_result.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        moves = []
        for i, r in enumerate(results):
            R = [x for x in rows if x[6] == i]
            if len(R) < 2:
                continue
            L = rev = 0.0
            for p, q in zip(R, R[1:]):
                ds = math.hypot(q[1] - p[1], q[2] - p[2]); L += ds; rev += ds if q[4] < -0.02 else 0.0
            g = node.legs_meta[i][3]
            n_stop = sum(1 for e in events if e["leg"] == r["name"])
            moves.append({"단계번호": i, "목표이름": r["name"], "단계": r["name"], "결과": "성공" if r["ok"] else ("건너뜀" if r["result"] == "skip" else "실패"),
                          "시간_s": round(R[-1][0] - R[0][0], 1), "길이_m": round(L, 2), "후진_m": round(rev, 2), "제자리_번": 0, "제자리_deg": 0,
                          "재계획_바뀜": r.get("replans", 0), "복구": n_stop, "멈춤_s": round(sum(e["stop_s"] for e in events if e["leg"] == r["name"]), 1),
                          "최소여유_cm": "", "목표_x": g[0], "목표_y": g[1], "도착_x": round(R[-1][1], 3), "도착_y": round(R[-1][2], 3)})
        json.dump(moves, open(os.path.join(a.out, "moves.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        with open(os.path.join(a.out, "plans.jsonl"), "w", encoding="utf-8") as f:
            for p in node.plans:
                f.write(json.dumps(p) + "\n")
    with open(os.path.join(a.out, "track.csv"), "w", encoding="utf-8") as f:
        f.write("t,x,y,yaw,v,w,step,v_nav,w_nav,loc\n")
        for r in rows:
            f.write(f"{r[0]:.2f},{r[1]:.4f},{r[2]:.4f},{r[3]:.1f},{r[4]:.3f},{r[5]:.3f},{r[6]},{r[7]:.3f},{r[8]:.3f},{r[9]}\n")
    json.dump(results, open(os.path.join(a.out, "legs.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    json.dump(events, open(os.path.join(a.out, "events.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    n_ok = sum(1 for r in results if r["ok"])
    with open(os.path.join(a.out, "summary.txt"), "w", encoding="utf-8") as f:
        f.write(f"장애물 주행 ({'Gazebo' if a.gz else '실물'}): 구간 {n_ok}/{len(results)} 성공 · {total_s:.1f} s · 간 곳 {' → '.join(visited)}"
                + (f" · 포기 {','.join(gave_up)}" if gave_up else "") + f" · 장애물 멈춤 {len(events)} 번\n")
        for r in results:
            f.write(f"{r['name']}: {r['result']} · {r.get('sim_s', '-')} s · 정지점 어긋남 {r.get('end_err_cm', '-')} cm"
                    + (f" · {' / '.join(r['notes'])}" if r.get("notes") else "") + "\n")
        for e in events:
            f.write(f"  장애물 {e['leg']} t {e['t']} · {e['verdict']} · 멈춤 {e['stop_s']} s · AMCL σ {e.get('amcl_sigma_cm')} cm\n")
    print(f"\n구간 {n_ok}/{len(results)} 성공 · {total_s:.1f} s → 기록 {a.out}", flush=True)


if __name__ == "__main__":
    main()
