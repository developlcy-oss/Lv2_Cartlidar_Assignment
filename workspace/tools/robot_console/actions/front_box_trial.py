#!/usr/bin/env python3
"""출발 상자 재기 (콘솔 모드 5): Nav2 로 출발 상자 대기 자리 (접근 자세표 stage) 를 차례로 가서, 도착마다 앞 D435
/perception/place_box/measure_front 를 3 번 부름. 정답 = 발제 좌표 (상자 중심 · 요 0). 미로 · 로봇만으로 (따로 준비 없음).
계획: data/261120_tonight/PLAN.md 4 절 (2026-11-20 검토 승인).

호출마다 trials.jsonl 한 줄: 위치 감시 상태 · AMCL 자세 · 공분산, 결과 (map · base_footprint), 퍽 두 개 자리, 발제와의 퍽 자리 오차,
max_shift (0.08) 거부 · sign_warn (|요| ≥ 40°) 여부. 위치 감시가 OK 가 아닐 때 값은 분석 때 치우침에서 뺌.

쓰기: front_box_trial.py --out <폴더> [--calls 3] [--skip B1:S,B5:N] [--step]
  --step  상자마다 Enter (TUI) 를 기다림 · s 건너뜀 · q 끝 (기본은 차례로 자동)
"""
import argparse
import json
import math
import os
import select
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.srv import SetPerceptionMode
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_goals import REPO  # noqa: E402
from start_check import wait_and_check  # noqa: E402

PUCK_DX = 0.040          # 퍽 = 상자 중심 x ±0.040 (mission_planner poses.PUCK_DX, 발제)
MAX_SHIFT = 0.08         # mission_plan.yaml start_box.max_shift
SIGN_WARN_DEG = 40.0     # mission_plan.yaml start_box.sign_warn_deg
# 발제 시작 박스 중심 (mm → m, ref/mission.pdf 251 행), 요 0 (퍽 두 개가 X 축)
BOXES = {1: (1.468, 0.950), 2: (3.670, 3.152), 3: (2.936, 5.354), 4: (0.734, 5.354), 5: (2.202, 5.354), 6: (3.670, 3.886)}
ORDER = [1, 2, 6, 3, 5, 4]   # 모드 1 과 같은 돌기 차례


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def read_key(timeout=None):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return None
    line = sys.stdin.readline()
    return None if line == '' else line.strip().lower()


def stages(skip):
    """박스마다 접근 자세 (계획기와 같은 규칙: ok · 제자리 회전 되는 쪽 먼저 · 틈 큰 쪽)"""
    d = yaml.safe_load(open(REPO / 'workspace/ros2_ws/src/mission_planner/config/approach_poses.yaml', encoding='utf-8'))
    out = []
    for b in ORDER:
        c = [(k, v) for k, v in d.items() if isinstance(v, dict) and v.get('kind') == 'box' and v.get('box') == b and v.get('ok')]
        c = [kv for kv in c if kv[0] not in skip]
        if not c:
            out.append({'box': b, 'key': None})
            continue
        k, v = max(c, key=lambda kv: (bool(kv[1].get('stage_rotate')), kv[1].get('stage_clearance') or 0.0))
        out.append({'box': b, 'key': k, 'stage': v['stage']})
    return out


class Runner:
    def __init__(self, node):
        self.n = node
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_prior = node.create_publisher(PoseStamped, '/perception/place_box/prior', 10)
        self.status = self.box = self.amcl = None
        self.loc = ''
        node.create_subscription(String, '/perception/place_box/status', lambda m: setattr(self, 'status', (time.time(), m.data)), latched)
        node.create_subscription(PoseStamped, '/perception/place_box', lambda m: setattr(self, 'box', (time.time(), m)), latched)
        node.create_subscription(String, '/localization/status', lambda m: setattr(self, 'loc', m.data), latched)
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: setattr(self, 'amcl', m), latched)
        self.cli = node.create_client(Trigger, '/perception/place_box/measure_front')
        self.mode = node.create_client(SetPerceptionMode, '/perception/set_mode')
        self.nav = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.tf = Buffer(cache_time=Duration(seconds=30))
        self.tl = TransformListener(self.tf, node)

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)

    def call(self, client, req, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        f = client.call_async(req)
        end = time.time() + timeout
        while not f.done() and time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)
        return f.result() if f.done() else None

    def set_mode(self):
        req = SetPerceptionMode.Request()
        req.mode = SetPerceptionMode.Request.SEARCH_FRONT
        r = self.call(self.mode, req, 30.0)
        print(f'인지 모드 SEARCH_FRONT: {r.message if r else "응답 없음"}', flush=True)
        return bool(r and r.success)

    def goto(self, x, y, yaw):
        if not self.nav.wait_for_server(timeout_sec=60.0):
            return False, '/navigate_to_pose 없음'
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.n.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        f = self.nav.send_goal_async(g)
        while not f.done():
            rclpy.spin_once(self.n, timeout_sec=0.1)
        h = f.result()
        if not h.accepted:
            return False, '목표 거절'
        r = h.get_result_async()
        t0 = time.time()
        while not r.done():
            rclpy.spin_once(self.n, timeout_sec=0.1)
            if read_key(0) == 'q':
                h.cancel_goal_async()
                return False, '사람이 끝냄 (q)'
            if time.time() - t0 > 180:
                h.cancel_goal_async()
                return False, '180 s 초과'
        ok = r.result().status == GoalStatus.STATUS_SUCCEEDED
        return ok, '도착' if ok else f'Nav2 실패 (상태 {r.result().status})'

    def to_frame(self, frame, src, x, y, yaw, stamp):
        T = self.tf.lookup_transform(frame, src, stamp, timeout=Duration(seconds=1.0))
        a = yaw_q(T.transform.rotation)
        t = T.transform.translation
        return (t.x + math.cos(a) * x - math.sin(a) * y, t.y + math.sin(a) * x + math.cos(a) * y, wrap(yaw + a))

    def measure(self, box, k):
        bx, by = BOXES[box]
        prior = PoseStamped()
        prior.header.frame_id = 'map'
        prior.header.stamp = self.n.get_clock().now().to_msg()
        prior.pose.position.x, prior.pose.position.y = bx, by
        prior.pose.orientation.w = 1.0
        self.pub_prior.publish(prior)
        self.spin(0.3)
        t0 = time.time()
        row = {'box': box, 'call': k, 't': t0, 'truth_map': [bx, by, 0.0], 'loc_status': self.loc, 'ok': False}
        if self.amcl:
            p = self.amcl.pose.pose
            c = self.amcl.pose.covariance
            row['amcl'] = [round(p.position.x, 4), round(p.position.y, 4), round(math.degrees(yaw_q(p.orientation)), 2)]
            row['amcl_sd'] = [round(math.sqrt(max(c[0], 0)), 4), round(math.sqrt(max(c[7], 0)), 4), round(math.degrees(math.sqrt(max(c[35], 0))), 2)]
        r = self.call(self.cli, Trigger.Request(), 5.0)
        row['request'] = r.message if r else '응답 없음'
        if not (r and r.success):
            return row
        while time.time() - t0 < 8.0 and not (self.status and self.status[0] > t0):
            rclpy.spin_once(self.n, timeout_sec=0.05)
        if not (self.status and self.status[0] > t0):
            row['why'] = 'status 시간 초과'
            return row
        st = json.loads(self.status[1])
        row['status'] = st
        self.spin(0.2)
        row['loc_status_after'] = self.loc
        if not st.get('ok') or not (self.box and self.box[0] > t0):
            row['why'] = st.get('why', 'ok false (흩어짐)')
            return row
        m = self.box[1]
        try:
            mx, my, myaw = self.to_frame('map', m.header.frame_id, m.pose.position.x, m.pose.position.y, yaw_q(m.pose.orientation), m.header.stamp)
            fx, fy, fyaw = self.to_frame('base_footprint', m.header.frame_id, m.pose.position.x, m.pose.position.y, yaw_q(m.pose.orientation),
                                         m.header.stamp)
        except Exception as e:  # noqa: BLE001
            row['why'] = f'TF {e}'
            return row
        # 정사각형이라 요는 발제 요 (0) ±45° 안으로
        myaw = wrap(myaw - round(myaw / (math.pi / 2)) * (math.pi / 2))
        pucks = [(mx + s * PUCK_DX * math.cos(myaw), my + s * PUCK_DX * math.sin(myaw)) for s in (-1, 1)]
        err = max(math.hypot(px - (bx + s * PUCK_DX), py - by) for (px, py), s in zip(pucks, (-1, 1)))
        shift = math.hypot(mx - bx, my - by)
        row.update(ok=True, map=[round(mx, 4), round(my, 4), round(math.degrees(myaw), 2)],
                   base_footprint=[round(fx, 4), round(fy, 4), round(math.degrees(fyaw), 2)],
                   pucks_map=[[round(a, 4), round(b, 4)] for a, b in pucks], puck_err=round(err, 4),
                   center_err=[round(mx - bx, 4), round(my - by, 4)], yaw_err_deg=round(math.degrees(myaw), 2),
                   max_shift_reject=shift > MAX_SHIFT, sign_warn=abs(math.degrees(myaw)) >= SIGN_WARN_DEG)
        return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--calls', type=int, default=3)
    ap.add_argument('--skip', default='', help='뺄 접근 자세 (쉼표), 예 B1:S,B5:N (Gazebo 판정 실패 자세)')
    ap.add_argument('--step', action='store_true')
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('console_start_box_drive')
    R = Runner(node)
    say = lambda t: print(t, flush=True)  # noqa: E731
    say('출발 상자 재기 — 출발 자리에서 Nav2 로 상자 대기 자리를 돌며 앞 D435 로 3 번씩 잼 (q = 끝)')
    from lifecycle_msgs.srv import GetState   # Nav2 active 기다림 (nav_goals.py 와 같음)
    gs = node.create_client(GetState, '/bt_navigator/get_state')
    end, active = time.time() + 180.0, False
    while time.time() < end and not active:
        if gs.wait_for_service(timeout_sec=1.0):
            f = gs.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, f, timeout_sec=3.0)
            active = bool(f.result() and f.result().current_state.label == 'active')
        if not active:
            time.sleep(2.0)
    if not active:
        say('✗ bt_navigator 가 180 s 안에 active 가 안 됨 — 끝')
        return 1
    if not wait_and_check(node, say):
        return 1
    if not R.set_mode():
        say('✗ 인지 모드를 못 켬 — 끝')
        return 1
    plan = stages({s.strip() for s in a.skip.split(',') if s.strip()})
    say('차례: ' + ' → '.join(f'B{p["box"]}:{(p["key"] or "없음 (뺌)").split(":")[-1]}' for p in plan))
    path = os.path.join(a.out, 'trials.jsonl')
    with open(path, 'a', encoding='utf-8') as f:
        for p in plan:
            if not p['key']:
                say(f'▶ B{p["box"]}: 쓸 접근 자세 없음 (뺌) — 건너뜀')
                continue
            if a.step:
                say(f'▶ B{p["box"]} ({p["key"]}) — Enter 출발 · s 건너뜀 · q 끝')
                key = None
                while key is None:
                    key = read_key(0.2)
                    rclpy.spin_once(node, timeout_sec=0.0)
                if key == 'q':
                    break
                if key == 's':
                    continue
            x, y, yaw = p['stage']
            say(f'→ B{p["box"]} 대기 자리 {p["key"]} ({x:.2f}, {y:.2f}) 로')
            ok, why = R.goto(x, y, yaw)
            say(f'  {why}')
            if why.startswith('사람이'):
                break
            if not ok:
                f.write(json.dumps({'box': p['box'], 'key': p['key'], 'nav': why}, ensure_ascii=False) + '\n')
                continue
            R.spin(1.0)   # 멈춘 뒤 영상 · TF 안정
            for k in range(1, a.calls + 1):
                row = R.measure(p['box'], k)
                row['key'] = p['key']
                f.write(json.dumps(row, ensure_ascii=False) + '\n')
                f.flush()
                if row['ok']:
                    say(f'  {k}회: 중심 오차 ({row["center_err"][0] * 1000:+.0f}, {row["center_err"][1] * 1000:+.0f}) mm · '
                        f'요 {row["yaw_err_deg"]:+.1f}° · 퍽 자리 오차 {row["puck_err"] * 1000:.0f} mm · 위치 감시 {row["loc_status"]}'
                        f'{" · max_shift 거부" if row["max_shift_reject"] else ""}{" · sign_warn" if row["sign_warn"] else ""}')
                else:
                    say(f'  {k}회: ✗ {row.get("why") or row["request"]}')
        hx, hy, hyaw = yaml.safe_load(open(REPO / 'workspace/ros2_ws/src/mission_planner/config/approach_poses.yaml',
                                           encoding='utf-8')).get('HOME', {}).get('stage', [0.69365, 0.95, math.pi])
        say(f'→ 출발 자리로 ({hx:.2f}, {hy:.2f})')
        say('  ' + R.goto(hx, hy, hyaw)[1])
    say(f'끝 → {path}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
