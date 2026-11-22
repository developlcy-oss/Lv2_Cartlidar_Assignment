#!/usr/bin/env python3
"""Gazebo 전용 (2026-11-21): 실물처럼 덜 도는 로봇 흉내 — 입력 (기본 /cmd_vel_turn) 의 회전만 줄여 → 출력 (기본 /cmd_vel, Gazebo 브리지).
  tools/drive_demo/sim_turn_scale.py (11-16, loopback 시뮬용) 의 사본. 실물 11-15 (벨트 v7 1 : 2.0) 기록:
  움직이며 꺾기 = 명령 회전의 약 79 %, 제자리 회전 = 66 ~ 80 %. 회전만 이 비율로 줄여 Gazebo 에 넣으면 IMU · EKF 도 줄어든 회전을 봄.
  근사일 뿐 (미끄럼 · 지연 · 잡음 없음). 실물 검증이 아님. navigation gazebo_sim.launch.py sim_turn:=0.79,0.70 일 때만 뜸."""
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class TurnScale(Node):
    def __init__(self):
        super().__init__("sim_turn_scale")
        self.k_curve = float(self.declare_parameter("k_curve", 0.79).value)
        self.k_spin = float(self.declare_parameter("k_spin", 0.70).value)
        self.spin_v = float(self.declare_parameter("spin_v", 0.03).value)   # |vx| 이 이보다 작으면 제자리 회전으로 봄
        self.pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.create_subscription(TwistStamped, "cmd_vel_turn", self._in, 20)
        self.get_logger().info(f"실물형 회전: 꺾기 × {self.k_curve} · 제자리 × {self.k_spin} (cmd_vel_turn → cmd_vel)")

    def _in(self, m):
        k = self.k_spin if abs(m.twist.linear.x) < self.spin_v else self.k_curve
        m.twist.angular.z *= k
        self.pub.publish(m)


def main():
    rclpy.init()
    n = TurnScale()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):   # Ctrl-C · launch 종료 (SIGTERM 뒤 rclpy 내부 예외) — 원본과 같게 조용히 끝냄
        pass


if __name__ == "__main__":
    main()
