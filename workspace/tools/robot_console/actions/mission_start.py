#!/usr/bin/env python3
"""통합 시연: /mission/start 를 받아들여질 때까지 3 s 마다 부름 (준비 확인 14 개가 모두 OK 일 때만 받음). 그 뒤 미션 상태를 찍음."""
import sys
import time

import rclpy
from robot_interfaces.msg import MissionStatus
from std_srvs.srv import Trigger

STATE = {0: 'IDLE', 1: 'RUNNING', 2: 'PAUSED', 3: 'DONE', 4: 'FAILED'}


def main():
    rclpy.init()
    node = rclpy.create_node('console_mission_start')
    from start_check import wait_and_check   # 출발 자리 확인 (검토 C, 2026-11-20)
    if not wait_and_check(node, lambda t: print(t, flush=True)):
        return 1
    cli = node.create_client(Trigger, '/mission/start')
    if not cli.wait_for_service(timeout_sec=120.0):
        print('✗ /mission/start 서비스 없음', flush=True)
        return 1
    for i in range(1, 201):
        f = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
        r = f.result()
        print(f'시도 {i}: {"★ 시작됨" if r and r.success else (r.message if r else "응답 없음")}', flush=True)
        if r and r.success:
            break
        time.sleep(3.0)
    else:
        return 1
    last = [None]

    def on(m):
        k = (m.step_index, m.state)
        if k != last[0]:
            last[0] = k
            print(f'미션 단계 {m.step_index} {m.step_name} {STATE.get(m.state, m.state)} {m.reason}', flush=True)
    node.create_subscription(MissionStatus, '/mission/status', on, 10)
    while rclpy.ok() and (last[0] is None or last[0][1] not in (3, 4)):
        rclpy.spin_once(node, timeout_sec=0.5)
    return 0 if last[0] and last[0][1] == 3 else 1


if __name__ == '__main__':
    sys.exit(main())
