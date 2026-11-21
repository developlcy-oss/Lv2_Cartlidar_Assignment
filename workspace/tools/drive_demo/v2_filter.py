#!/usr/bin/env python3
"""규칙 v2 cmd_vel 필터 (시연용, 저장소 밖): Nav2 명령 cmd_vel_raw → 한계 적용 → out_topic (실물 cmd_vel_nav = twist_mux nav 입력, 시뮬 cmd_vel)
drive_rl base_cmd.py 규칙 v2 와 같은 한계 (가속은 diff_drive 가 맡음):
  - 전진 ≤ vx_fwd · 후진 ≤ vx_rev [m/s] — 넘으면 v · ω 를 같은 비율로 줄임 (반경 그대로)
  - 이어진 후진 ≤ rev_max (기본 0.40 m, 0.15 m 전진해야 다시 셈) — 넘으면 뒤로 가는 명령을 0
    11-15 21:2x 사용자 결정: 실물만 0.50 m (계획은 0.40 그대로). 실물이 덜 꺾어 (79 %) 꺾으며 하는 후진이 계획보다 길어짐 (S4 막힘, 1115_2104)
  - 움직이는 중 (|v| ≥ 0.03) |ω| ≤ |v| / 0.20 (반경 ≥ 0.20 m)
  - 멈춘 채 제자리 회전 |ω| ≤ spin_w, 돈 각도 합 ≤ 90° (0.15 m 움직여야 다시 셈)
    11-15 21:1x: odom_topic 을 주면 돈 각도를 명령이 아니라 실제 (오도메트리 yaw 변화) 로 셈 — 실물은 명령보다 느리게 돌아
    (제자리 66 ~ 80 %) 명령으로 세면 예산이 일찍 차서 S1 시작 방향 맞추기가 0 으로 잘림 (1115_2055)
  - 꺾는 중 가장 빠른 모터 ≤ turn_motor (기본 3.0) rad/s (곡률 0.5 /m 부터 줄어 1 /m 부터 상한), v · ω 같은 비율로 줄임
    모터 = 바퀴 둘레 속도 / (바퀴 반지름 0.0335 × belt). belt 기본 1.5 (11-14 까지 계획 · 시뮬 기준), v7 벨트 = 2.0 (11-15)
  - 11-18 속도 곡선: demo_follow 가 내는 /drive_demo/stop_dist (다음 정지까지 남은 경로 길이 [m], 음수 = 없음) 로
    |v| ≤ √(stop_creep_v² + 2·stop_decel·max(d − stop_creep_d, 0)) — 정지 앞에서만 감속 (그 밖은 전진 상한 · 모터 상한까지).
    0.3 s 넘게 안 오면 이 상한은 끔. 직진 모터 상한 = vx_fwd 기준 (11-16 젯슨 판과 같게, 기록으로 확인)
입력이 0.25 s 끊기면 0 을 0.5 s 동안 내고 조용해짐 (twist_mux nav 입력이 시간 초과로 풀리게 — 리모컨 · drive 를 막지 않음).
같은 값을 cmd_vel_v2 로도 냄 (기록용). ⚠ 소프트웨어 한계일 뿐 물리 비상정지가 아님.
"""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rclpy.node import Node

REV_MAX, RESET, R_MIN, SPIN_V, SPIN_MAX = 0.40, 0.15, 0.20, 0.03, math.radians(90.0)
K0, K1, R_WHEEL = 0.5, 1.0, 0.0335
SEP = 0.317 * 1.29   # 실효 바퀴 간격 = 기하 0.317 × controllers_4wd.yaml wheel_separation_multiplier (2026-11-18, 실물 회전 0.775 배)
DT = 0.02


class V2Filter(Node):
    def __init__(self):
        super().__init__("v2_filter")
        self.vx_fwd = self.declare_parameter("vx_fwd", 0.30).value
        self.vx_rev = self.declare_parameter("vx_rev", 0.15).value
        self.spin_w = self.declare_parameter("spin_w", 0.5).value
        out = self.declare_parameter("out_topic", "cmd_vel_nav").value
        self.belt = self.declare_parameter("belt", 1.5).value   # 바퀴 ω / 모터 ω (v7 = 2.0)
        self.turn_motor = self.declare_parameter("turn_motor", 5.5).value   # 꺾는 중 모터 상한 [rad/s]. 11-18 arc_test: 바퀴 12.7 rad/s (모터 6.35) 에서 포화 → 13 % 여유
        self.m_per = R_WHEEL * self.belt   # 모터 1 rad/s → 바퀴 둘레 속도 [m/s]
        self.rev_max = self.declare_parameter("rev_max", REV_MAX).value   # 이어진 후진 한도 [m] (실물 0.50, run_demo)
        self.odom_topic = self.declare_parameter("odom_topic", "").value   # 있으면 제자리 회전 예산 = 실제 돈 각도
        self.yaw_last, self.t_spin, self.spin_peak = None, -1e9, 0.0
        self.m_top = self.vx_fwd * (1 + K0 * SEP / 2) / self.m_per   # 거의 직진일 때 모터 상한 = 전진 상한으로 갈 수 있게 (11-16 젯슨 판)
        self.stop_decel = self.declare_parameter("stop_decel", 0.3).value     # 정지 앞 감속도 [m/s²]
        self.stop_creep_v = self.declare_parameter("stop_creep_v", 0.05).value   # 마지막 구간 속도 [m/s]
        self.stop_creep_d = self.declare_parameter("stop_creep_d", 0.05).value   # 마지막 구간 길이 [m] (demo_follow 가 중간 정지에는 이만큼 더해 보냄)
        self.stop_d, self.t_stop = -1.0, -1e9
        self.create_subscription(Float32, "/drive_demo/stop_dist", self._stop, 10)
        self.pub = self.create_publisher(TwistStamped, out, 10)
        self.mon = self.create_publisher(TwistStamped, "cmd_vel_v2", 10)
        self.create_subscription(TwistStamped, "cmd_vel_raw", self._in, 20)
        self.raw, self.t_raw = (0.0, 0.0), -1e9
        self.spin_used = self.moved = self.rev_run = self.fwd = 0.0
        self.clips = {"speed": 0, "curv": 0, "spin": 0, "rev": 0, "motor": 0, "stop": 0}
        if self.odom_topic:
            self.create_subscription(Odometry, self.odom_topic, self._odom, 20)
        self.create_timer(DT, self._tick)
        self.get_logger().info(f"규칙 v2 필터: cmd_vel_raw → {out} · 전진 ≤ {self.vx_fwd} · 후진 ≤ {self.vx_rev} m/s · 제자리 ≤ {self.spin_w} rad/s · 벨트 {self.belt} · 꺾는 중 모터 ≤ {self.turn_motor} · 제자리 예산 = {"실제 " + self.odom_topic if self.odom_topic else "명령"} · 이어진 후진 ≤ {self.rev_max} m · 정지 앞 감속 {self.stop_decel} m/s² → {self.stop_creep_v} m/s (마지막 {self.stop_creep_d} m)")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.yaw_last is not None and self.now() - self.t_spin < 0.5:   # 제자리 회전 명령 중 · 끝난 뒤 0.5 s (미끄러져 더 도는 것)
            self.spin_used += abs((yaw - self.yaw_last + math.pi) % (2 * math.pi) - math.pi)
            self.spin_peak = max(self.spin_peak, self.spin_used)
        self.yaw_last = yaw

    def _stop(self, m):
        self.stop_d, self.t_stop = m.data, self.now()

    def _in(self, m):
        self.raw, self.t_raw = (m.twist.linear.x, m.twist.angular.z), self.now()

    def _tick(self):
        age = self.now() - self.t_raw
        if age > 0.75:
            return   # 조용 (twist_mux nav 시간 초과)
        v, w = self.raw if age < 0.25 else (0.0, 0.0)
        cap = self.vx_fwd if v >= 0 else self.vx_rev
        if abs(v) > cap:
            s = abs(v) / cap
            v, w = v / s, w / s
            self.clips["speed"] += 1
        if v < 0 and self.rev_run + abs(v) * DT > self.rev_max:
            v = 0.0
            self.clips["rev"] += 1
        if abs(v) >= SPIN_V:
            lim = abs(v) / R_MIN
            if abs(w) > lim + 1e-9:
                w = math.copysign(lim, w)
                self.clips["curv"] += 1
        else:
            left = max(SPIN_MAX - self.spin_used, 0.0) / DT
            lim = min(max(self.spin_w, abs(v) / R_MIN), left)
            if abs(w) > lim + 1e-9:
                w = math.copysign(lim, w)
                self.clips["spin"] += 1
        sp = abs(v)
        if sp < SPIN_V:
            allow = self.turn_motor
        else:
            f = min(max((abs(w) / sp - K0) / (K1 - K0), 0.0), 1.0)
            allow = self.m_top - (self.m_top - self.turn_motor) * f
        s = max((sp + abs(w) * SEP / 2) / self.m_per / allow, 1.0)
        if s > 1.0 + 1e-6:
            v, w = v / s, w / s
            self.clips["motor"] += 1
        if self.stop_d >= 0.0 and self.now() - self.t_stop < 0.3 and abs(v) > 1e-6:   # 정지 앞 감속 (속도 곡선)
            cap = math.sqrt(self.stop_creep_v ** 2 + 2 * self.stop_decel * max(self.stop_d - self.stop_creep_d, 0.0))
            if abs(v) > cap + 1e-6:
                s = abs(v) / cap
                v, w = v / s, w / s
                self.clips["stop"] += 1
        sp = abs(v)
        if sp < SPIN_V and abs(w) > 0.05:
            self.t_spin = self.now()
        if sp < SPIN_V and not self.odom_topic:
            self.spin_used += abs(w) * DT
            self.spin_peak = max(self.spin_peak, self.spin_used)
        self.moved = 0.0 if (sp < SPIN_V and abs(w) > 0.05) else self.moved + sp * DT
        if self.moved >= RESET:
            self.spin_used = 0.0
        if v < 0:
            self.rev_run += -v * DT
            self.fwd = 0.0
        else:
            self.fwd += v * DT
            if self.fwd >= RESET:
                self.rev_run = 0.0
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        out.twist.linear.x, out.twist.angular.z = v, w
        self.pub.publish(out)
        self.mon.publish(out)


def main():
    rclpy.init()
    n = V2Filter()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    print(f"v2_filter 잘림 횟수 (50 Hz 틱): {n.clips} · 제자리 예산 최대 {math.degrees(n.spin_peak):.0f}° ({'실제' if n.odom_topic else '명령'})", flush=True)


if __name__ == "__main__":
    main()
