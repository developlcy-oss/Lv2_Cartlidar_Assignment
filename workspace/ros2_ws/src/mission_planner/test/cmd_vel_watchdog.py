#!/usr/bin/env python3
"""Gazebo 시험용 (#151): twist_mux 출력 /cmd_vel_mux -> /cmd_vel. timeout (기본 0.2 s) 동안 명령이 없으면 0 을 한 번 냄
(실물 OpenCR ERR_CMD_TIMEOUT 200 ms 와 같게). Gazebo DiffDrive 는 시간 초과가 없어 localization 잠금 중에도
마지막 Nav2 명령으로 계속 달림 → 바퀴 헛돌기 · odom 흐름 · 다시 넣은 자세 틀어짐 (2026-11-12)."""
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class Watchdog(Node):
    def __init__(self):
        super().__init__("cmd_vel_watchdog")
        self.timeout = float(self.declare_parameter("timeout", 0.2).value)
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(TwistStamped, "/cmd_vel_mux", self.on_cmd, 10)
        self.last, self.zeroed, self.n_zero = None, True, 0
        self.create_timer(0.05, self.tick)

    def on_cmd(self, m):
        self.last, self.zeroed = self.get_clock().now(), False
        self.pub.publish(m)

    def tick(self):
        if self.zeroed or self.last is None:
            return
        if (self.get_clock().now() - self.last).nanoseconds * 1e-9 > self.timeout:
            z = TwistStamped()
            z.header.stamp = self.get_clock().now().to_msg()
            z.header.frame_id = "base_link"
            self.pub.publish(z)
            self.zeroed = True
            self.n_zero += 1
            self.get_logger().info(f"명령 {self.timeout:.2f} s 없음 -> 0 ({self.n_zero} 번째)")


def main():
    rclpy.init()
    n = Watchdog()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):   # 종료 신호 · 컨텍스트 종료 (stop_all INT)
        pass


if __name__ == "__main__":
    main()
