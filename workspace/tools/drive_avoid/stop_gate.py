#!/usr/bin/env python3
"""정지 관문 (2026-11-17): 장애물 감시가 막힘을 내거나 주행 관리자가 멈춤을 요청하면 바퀴 명령을 바로 0 으로.

  python3 stop_gate.py --ros-args -p in_topic:=cmd_vel_gate_in -p out_topic:=cmd_vel [-p use_sim_time:=true]

  in_topic (TwistStamped, 규칙 v2 필터 출력) → out_topic (시뮬 /cmd_vel · 실물 cmd_vel_nav = twist_mux nav 입력)
  0 으로 바꾸는 때: /avoid/blocked = true · /avoid/hold = true · 감시 소식이 guard_timeout 넘게 끊김 (감시가 죽으면 서도록)
  막힌 동안은 0 을 20 Hz 로 계속 냄 (앞 명령이 남지 않게). 소프트웨어 정지 (물리 비상정지 아님).
"""
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String


class Gate(Node):
    def __init__(self):
        super().__init__("stop_gate")
        P = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.timeout = P("guard_timeout", 0.6)
        self.pub = self.create_publisher(TwistStamped, P("out_topic", "cmd_vel"), 10)
        self.ev = self.create_publisher(String, "/avoid/gate", 10)
        self.blocked, self.hold, self.t_guard = False, False, 0.0
        self.closed_prev = None
        self.create_subscription(TwistStamped, P("in_topic", "cmd_vel_gate_in"), self._in, 20)
        self.create_subscription(Bool, "/avoid/blocked", self._blocked, 10)
        self.create_subscription(Bool, "/avoid/hold", lambda m: self._set("hold", m.data), 10)
        self.create_timer(0.05, self._tick)

    def _blocked(self, m):
        self.t_guard = time.monotonic()
        self._set("blocked", m.data)

    def _set(self, k, v):
        setattr(self, k, bool(v))
        self._tick()

    def closed(self):
        stale = time.monotonic() - self.t_guard > self.timeout
        return self.blocked or self.hold or stale, ("감시 끊김" if stale else "막힘" if self.blocked else "멈춤 요청" if self.hold else "")

    def _zero(self):
        z = TwistStamped()
        z.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(z)

    def _tick(self):
        c, why = self.closed()
        if c:
            self._zero()
        if c != self.closed_prev:
            self.ev.publish(String(data=f"{'닫힘' if c else '열림'} {why}".strip()))
            self.closed_prev = c

    def _in(self, m):
        if self.closed()[0]:
            self._zero()
        else:
            self.pub.publish(m)


def main():
    rclpy.init()
    n = Gate()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
