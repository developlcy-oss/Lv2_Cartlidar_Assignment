#!/usr/bin/env python3
"""팔 시연: 이름 자세를 차례로 (/manipulator/pose). 자세마다 Enter 를 기다림 (TUI Enter), q = 끝.
쓰기: arm_demo.py DRIVE ALIGN TAG_VIEW BOX_VIEW DRIVE
켜질 때 정리 (STOW → DRIVE) 가 끝나기 전에는 manipulator 가 거절하므로 성공할 때까지 5 s 마다 다시 보냄 (최대 120 s)."""
import sys
import time

import rclpy
from rclpy.action import ActionClient
from robot_interfaces.action import ArmPose


def main():
    poses = sys.argv[1:] or ['DRIVE']
    rclpy.init()
    node = rclpy.create_node('console_arm_demo')
    ac = ActionClient(node, ArmPose, '/manipulator/pose')
    if not ac.wait_for_server(timeout_sec=60.0):
        print('✗ /manipulator/pose 서버 없음', flush=True)
        return 1
    for i, name in enumerate(poses):
        print(f'▶ {i + 1}/{len(poses)} {name} — Enter 로 출발, q 끝', flush=True)
        line = sys.stdin.readline()
        if not line or line.strip().lower() == 'q':
            print('끝 (사용자)', flush=True)
            return 0
        end = time.time() + 120
        while True:
            f = ac.send_goal_async(ArmPose.Goal(name=name))
            rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
            gh = f.result()
            if gh and gh.accepted:
                r = gh.get_result_async()
                rclpy.spin_until_future_complete(node, r, timeout_sec=120.0)
                res = r.result()
                ok = res is not None and res.result.success
                print(f'  {"✓" if ok else "✗"} {name} {"" if ok else (res.result.message if res else "시간 초과")}', flush=True)
                break
            if time.time() > end:
                print(f'  ✗ {name} 계속 거절됨 (켜질 때 정리 · 비상정지 · 팔 HOLD 확인)', flush=True)
                return 1
            print('  … 거절됨 (팔 정리 중이거나 HOLD) — 5 s 뒤 다시', flush=True)
            time.sleep(5.0)
    print('팔 시연 끝', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
