#!/usr/bin/env python3
"""자율주행 시연: Nav2 (/navigate_to_pose) 에 목표를 차례로 보냄. 길은 Nav2 가 지도 · 라이다로 스스로 짜고 장애물을 피함
(정해진 길을 따라가는 drive_demo 와 다름). 위치는 AMCL (발제 SDF 지도).

쓰기: nav_goals.py [--step] [--goals <yaml>] [--out <폴더>]
  --step   목표마다 Enter 를 기다림 (TUI Enter) · s = 이 목표 건너뜀 · q = 끝
  --goals  [{name, x, y, yaw_deg}] 목록 (발제 좌표). 없으면 미션 접근 자세 (approach_poses.yaml stage): 상자 6 곳 → L3 · L2 · L1 → 출발
결과: --out 폴더 nav_goals.json (목표마다 성공 · 걸린 시간 · 끝 자리 오차)
"""
import argparse
import json
import math
import os
import select
import sys
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
STATUS = {GoalStatus.STATUS_SUCCEEDED: '성공', GoalStatus.STATUS_ABORTED: '실패 (Nav2 포기)', GoalStatus.STATUS_CANCELED: '취소'}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


ORDER = ['B1', 'B2', 'B6', 'B3', 'B5', 'B4', 'L3', 'L2', 'L1', 'HOME']   # 11-16 주행 시연과 같은 차례


def default_goals():
    """미션이 Nav2 목표로 쓰는 접근 자세 (mission_planner/config/approach_poses.yaml 의 stage = 상자 면에서 standoff 떨어진 자리).
    상자는 ok 인 쪽 중 장애물과 틈 (stage_clearance) 이 가장 큰 쪽."""
    d = yaml.safe_load(open(REPO / 'workspace/ros2_ws/src/mission_planner/config/approach_poses.yaml', encoding='utf-8'))
    out = []
    for name in ORDER:
        cands = [(k, v) for k, v in d.items() if isinstance(v, dict) and 'stage' in v and v.get('ok')
                 and (k == name or k.startswith(name + ':'))]
        if not cands:
            continue
        k, v = max(cands, key=lambda kv: kv[1].get('stage_clearance') or 0.0)
        x, y, yaw = v['stage']
        out.append({'name': '출발' if k == 'HOME' else k, 'x': x, 'y': y, 'yaw_deg': math.degrees(yaw),
                    'clearance_m': v.get('stage_clearance')})
    home = d.get('HOME', {}).get('stage', [0.69365, 0.95, math.pi])
    return out, home


def read_line(timeout=None):
    """TUI 가 표준 입력으로 보낸 한 줄 (Enter = ''), 없으면 None"""
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return None
    line = sys.stdin.readline()
    return line.strip().lower() if line else 'q'


def go(node, ac, buf, g, tgt, robot_pose):
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = node.get_clock().now().to_msg()
    ps.pose.position.x, ps.pose.position.y = tgt[0], tgt[1]
    ps.pose.orientation.z, ps.pose.orientation.w = math.sin(tgt[2] / 2), math.cos(tgt[2] / 2)
    t0 = time.time()
    fb = {'d': None, 'rec': 0}

    def on_fb(m):
        fb['d'] = m.feedback.distance_remaining
        fb['rec'] = m.feedback.number_of_recoveries
    f = ac.send_goal_async(NavigateToPose.Goal(pose=ps), feedback_callback=on_fb)
    rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
    gh = f.result()
    status = None
    if not gh or not gh.accepted:
        print(f'  ✗ {g["name"]} 목표 거절 (비상정지 · 위치 잠금 · Nav2 상태 확인)', flush=True)
    else:
        print(f'  → {g["name"]} 출발', flush=True)
        rf = gh.get_result_async()
        last = 0.0
        try:
            while not rf.done():
                rclpy.spin_once(node, timeout_sec=0.2)
                if time.time() - last > 3.0 and fb['d'] is not None:
                    print(f'    남은 거리 {fb["d"]:.2f} m · 복구 {fb["rec"]} 번 · {time.time() - t0:.0f} s', flush=True)
                    last = time.time()
        except KeyboardInterrupt:
            gh.cancel_goal_async()
            raise
        status = rf.result().status
    pose = robot_pose()
    err = None
    if pose:
        err = {'cm': round(100 * math.hypot(pose[0] - tgt[0], pose[1] - tgt[1]), 1),
               'deg': round(math.degrees(abs(wrap(pose[2] - tgt[2]))), 1)}
    res = STATUS.get(status, '거절' if status is None else f'상태 {status}')
    print(f'  {"✓" if status == GoalStatus.STATUS_SUCCEEDED else "✗"} {g["name"]} {res} · {time.time() - t0:.1f} s'
          + (f' · 끝 자리 오차 {err["cm"]} cm · {err["deg"]}°' if err else ''), flush=True)
    return {'name': g['name'], 'result': res, 'time_s': round(time.time() - t0, 1), 'end_error': err, 'recoveries': fb['rec']}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--step', action='store_true')
    ap.add_argument('--goals', default='')
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    goals, start = default_goals()
    if a.goals:
        goals = yaml.safe_load(open(a.goals, encoding='utf-8'))
    rclpy.init()
    node = rclpy.create_node('console_nav_goals')
    buf = Buffer()
    TransformListener(buf, node)
    ac = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    print('Nav2 /navigate_to_pose 기다림 (최대 180 s) …', flush=True)
    if not ac.wait_for_server(timeout_sec=180.0):
        print('✗ /navigate_to_pose 없음 — Nav2 가 다 안 켜짐 (#276: ros2 lifecycle get /bt_navigator)', flush=True)
        return 1

    def robot_pose():
        end = time.time() + 2.0
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                t = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
                q = t.transform.rotation
                return [t.transform.translation.x, t.transform.translation.y,
                        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))]
            except Exception:
                pass
        return None

    results = []
    print(f'목표 {len(goals)} 곳: ' + ' → '.join(g['name'] for g in goals), flush=True)
    quit_all = False
    for i, g in enumerate(goals):
        tgt = [g['x'], g['y'], math.radians(g.get('yaw_deg', 0.0))]
        if a.step:
            print(f'▶ {i + 1}/{len(goals)} {g["name"]} ({g["x"]:.2f}, {g["y"]:.2f}) — Enter 출발 · s 건너뜀 · q 끝', flush=True)
            k = read_line()
            if k == 'q':
                break
            if k == 's':
                results.append({'name': g['name'], 'result': '건너뜀'})
                continue
        while True:   # 실패하면 (step) Enter = 다시
            r = go(node, ac, buf, g, tgt, robot_pose)
            results.append(r)
            if r['result'] == '성공' or not a.step:
                break
            print('  Enter = 다시 · s = 다음 목표 · q = 끝', flush=True)
            k = read_line()
            if k == 'q':
                quit_all = True
                break
            if k == 's':
                break
        if quit_all:
            break
    ok = sum(r.get('result') == '성공' for r in results)
    print(f'끝: 성공 {ok}/{len(results)}', flush=True)
    if a.out:
        Path(a.out).mkdir(parents=True, exist_ok=True)
        (Path(a.out) / 'nav_goals.json').write_text(json.dumps(results, ensure_ascii=False, indent=1), encoding='utf-8')
    return 0 if ok == len(results) else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
