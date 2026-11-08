#!/usr/bin/env python3
"""지금 카메라에 보이는 퍽을 집는다 (실물 시험용, manipulator_node 의 target_look → toppick_tilt).

  1. /perception/set_mode 로 퍽 인식을 켬 (기본 SEARCH_FRONT | READ_TAG = 5)
  2. /perception/pucks 에서 집을 후보 (target_index, 없으면 --any 일 때 가장 가까운 퍽) 를 고름
  3. execute_skill target_look:<frame>:x:y:z → 팔 카메라로 내려다보며 위치를 다시 재서 붙잡음
  4. 성공하면 execute_skill toppick_tilt → 45° 로 집고 그리퍼 막힘으로 쥠 판정

사용 (bringup · ~/rearm.sh · 자동 STOW 가 끝난 뒤):
  ros2 run control pick_visible_puck.py --dry-run      # 고른 퍽만 보여줌 (팔 안 움직임)
  ros2 run control pick_visible_puck.py                # 확인 후 집기
  ros2 run control pick_visible_puck.py --look-only    # target_look 까지만
Ctrl-C = 진행 중인 skill 취소.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from robot_interfaces.action import ExecuteSkill
from robot_interfaces.msg import PuckArray
from robot_interfaces.srv import SetPerceptionMode

COLORS = {0: '모름', 1: '빨강', 2: '초록', 3: '파랑'}
SHAPES = {0: '모름', 1: '원통', 2: '상자'}
CAMERAS = {0: '앞 D435', 1: '뒤 D435', 2: '팔 Arducam'}


class PickVisiblePuck(Node):
    def __init__(self):
        super().__init__('pick_visible_puck')
        self.pucks = None
        self.create_subscription(PuckArray, '/perception/pucks', self.on_pucks,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.mode_client = self.create_client(SetPerceptionMode, '/perception/set_mode')
        self.skill_client = ActionClient(self, ExecuteSkill, '/manipulator/execute_skill')
        self.goal_handle = None

    def on_pucks(self, msg):
        self.pucks = msg

    def spin_until(self, done, timeout):
        end = time.monotonic() + timeout
        while rclpy.ok() and not done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return done()

    def set_mode(self, mode):
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('/perception/set_mode 없음 — bringup 을 use_perception:=true 로 띄웠는지 확인')
        req = SetPerceptionMode.Request()
        req.mode = mode
        fut = self.mode_client.call_async(req)
        self.spin_until(fut.done, 5.0)
        if not fut.done() or not fut.result().success:
            raise RuntimeError(f'set_mode({mode}) 실패: {fut.result().message if fut.done() else "응답 없음"}')
        self.get_logger().info(f'perception: {fut.result().message}')

    def pick_target(self, wait, any_puck):
        """퍽 목록이 비지 않을 때까지 wait 초 기다린 뒤 (puck, index) 또는 None"""
        end = time.monotonic() + wait
        while rclpy.ok() and time.monotonic() < end:
            self.pucks = None
            self.spin_until(lambda: self.pucks is not None, 1.0)
            msg = self.pucks
            if msg is None or not msg.pucks:
                continue
            if 0 <= msg.target_index < len(msg.pucks):
                return msg.pucks[msg.target_index], msg.target_index, msg
            if any_puck:
                return msg.pucks[0], 0, msg   # 수평 거리가 가까운 것부터 정렬돼 옴
        return None

    def run_skill(self, policy, timeout):
        if not self.skill_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('/manipulator/execute_skill 없음 — manipulator_node 가 떴는지 확인')
        goal = ExecuteSkill.Goal()
        goal.policy = policy
        goal.timeout = float(timeout)
        self.get_logger().info(f'skill 보냄: {policy}')
        send = self.skill_client.send_goal_async(goal)
        self.spin_until(send.done, 5.0)
        if not send.done() or not send.result().accepted:
            return False, '거절됨 (자동 STOW 정리 중이거나 이름이 틀림 — bringup 로그 확인)'
        self.goal_handle = send.result()
        res = self.goal_handle.get_result_async()
        self.spin_until(res.done, timeout + 10.0)
        self.goal_handle = None
        if not res.done():
            return False, '결과 시간 초과'
        r = res.result().result
        return r.success, r.message

    def cancel(self):
        if self.goal_handle is not None:
            fut = self.goal_handle.cancel_goal_async()
            self.spin_until(fut.done, 2.0)
            self.get_logger().warn('skill 취소 요청함')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mode', type=int, default=5, help='perception 모드 (기본 5 = SEARCH_FRONT | READ_TAG, 0 = 안 바꿈)')
    ap.add_argument('--wait', type=float, default=10.0, help='퍽이 보일 때까지 기다릴 시간 s')
    ap.add_argument('--any', action='store_true', help='집을 후보(target_index)가 없으면 가장 가까운 퍽')
    ap.add_argument('--frame', default='base_link', help='target_look 좌표 frame (/perception/pucks 는 base_link)')
    ap.add_argument('--z-offset', type=float, default=0.0, help='퍽 중심 z 보정 m (인식 높이가 틀릴 때)')
    ap.add_argument('--look-only', action='store_true', help='target_look 까지만')
    ap.add_argument('--dry-run', action='store_true', help='고른 퍽만 보여주고 끝 (팔 안 움직임)')
    ap.add_argument('--yes', action='store_true', help='움직이기 전에 묻지 않음')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = PickVisiblePuck()
    try:
        code = run(args, node)
    except KeyboardInterrupt:
        node.cancel()
        code = 1
    except RuntimeError as e:
        print(f'오류: {e}')
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


def run(args, node):
    if args.mode:
        node.set_mode(args.mode)
    found = node.pick_target(args.wait, args.any)
    if found is None:
        n = len(node.pucks.pucks) if node.pucks else 0
        print(f'퍽을 못 찾음 ({args.wait:.0f} s). 보이는 퍽 {n} 개, 집을 후보 없음'
              + ('' if n == 0 else ' → --any 로 가장 가까운 퍽'))
        print('  확인: ros2 topic hz /top_camera_front/color/image_raw · /arm_camera/image_raw, 퍽 윗면 AprilTag')
        return 1
    puck, idx, msg = found
    p = puck.pose.position
    z = p.z + args.z_offset
    print(f'퍽 #{idx}/{len(msg.pucks)}: 번호 {puck.tag_id} · {COLORS.get(puck.color, "?")} · '
          f'{SHAPES.get(puck.shape, "?")} · {CAMERAS.get(puck.camera, "?")} · 신뢰도 {puck.confidence:.2f}')
    print(f'  위치 ({msg.header.frame_id}) x {p.x:.3f} · y {p.y:.3f} · z {p.z:.3f} m, '
          f'수평 거리 {math.hypot(p.x, p.y):.3f} m')
    look = f'target_look:{args.frame}:{p.x:.4f}:{p.y:.4f}:{z:.4f}'
    if args.dry_run:
        print(f'(dry-run) 보낼 skill: {look} → toppick_tilt')
        return 0
    if not args.yes and input('팔이 움직입니다. 주변을 비웠으면 y: ').strip().lower() != 'y':
        print('취소')
        return 1
    ok, why = node.run_skill(look, 20.0)
    print(f'target_look: {"성공" if ok else "실패"} {why}')
    if not ok or args.look_only:
        return 0 if ok else 1
    ok, why = node.run_skill('toppick_tilt', 30.0)
    print(f'toppick_tilt: {"성공 (쥠)" if ok else "실패"} {why}')
    return 0 if ok else 1


if __name__ == '__main__':
    main()
