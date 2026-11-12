#!/usr/bin/env python3
"""키보드 조이스틱 (2026-11-12, 젯슨에서 실행): 조이스틱 없이 /joy 를 보내 브링업의 REARM · E-stop · teleop 주행을 키보드로.

    source /opt/ros/lyrical/setup.bash && source ~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/setup.bash
    python3 ~/kbd_joy.py

- 팀 opencr_hardware: 리셋 버튼 = /joy buttons[7] (REARM), E-stop = buttons[6], 조이스틱 무수신이면 잠김 -> 20 Hz 로 계속 보냄.
- 팀 teleop_node (TeleopTwistJoy): enable 버튼 4 를 누른 동안만 axes[1] 앞뒤 · axes[0] 회전 (scale 0.2 m/s · 1.0 rad/s).
- 주행 키는 누른 뒤 0.3 s 만 유지 (키를 떼면 멈춤, 터미널 키 반복으로 계속 감).
"""
import select
import sys
import termios
import time
import tty

import rclpy
from sensor_msgs.msg import Joy

HELP = """키보드 조이스틱
  r = 리셋 (REARM, 팔 HOLD 풀기 / 시작 정리 시작)     x = 비상정지 (E-stop, 다시 풀려면 r)
  w / s = 앞 / 뒤 (누르고 있는 동안)   a / d = 왼쪽 / 오른쪽 회전   스페이스 = 멈춤
  q = 끝 (끝나면 /joy 가 끊겨 브링업이 잠김 = 안전)"""


def main():
    rclpy.init()
    node = rclpy.create_node("kbd_joy")
    pub = node.create_publisher(Joy, "/joy", 10)
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print(HELP, flush=True)
    btn_until = {6: 0.0, 7: 0.0}
    move, move_until = (0.0, 0.0), 0.0
    try:
        while True:
            now = time.time()
            if select.select([sys.stdin], [], [], 0.05)[0]:
                k = sys.stdin.read(1)
                if k == "q":
                    break
                if k == "r":
                    btn_until[7] = now + 0.3
                    print("  리셋 (REARM) 보냄", flush=True)
                elif k == "x":
                    btn_until[6] = now + 0.3
                    print("  비상정지 보냄 (풀려면 r)", flush=True)
                elif k in "wsad":
                    move = {"w": (1.0, 0.0), "s": (-1.0, 0.0), "a": (0.0, 1.0), "d": (0.0, -1.0)}[k]
                    move_until = now + 0.3
                elif k == " ":
                    move_until = 0.0
                    print("  멈춤", flush=True)
            m = Joy()
            m.header.stamp = node.get_clock().now().to_msg()
            m.axes = [0.0] * 8
            m.buttons = [0] * 11
            if now < move_until:
                m.buttons[4] = 1                    # teleop enable (누른 동안만 주행)
                m.axes[1], m.axes[0] = move
            for b, t in btn_until.items():
                if now < t:
                    m.buttons[b] = 1
            pub.publish(m)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()
        print("끝 (/joy 끊김 -> 브링업 잠김)")


if __name__ == "__main__":
    main()
