#!/usr/bin/env python3
"""11-21 상자 점만 지운 정지용 스캔 (run_demo.sh 가 SAFETY=1 일 때 띄움, collision_monitor · HOLD 의 소스).

  merged_in (실물 /scan_merged · 시뮬 /scan) → /scan_guard : 지금 구간 상자 (반 변 + GUARD_MARGIN, map) 안의 점만 inf
  front_in  (실물 /scan_front, 원본)          → /scan_front_guard : 상자 구간 (zone) 일 때만 차체 (BODY_BOX) · 상자 점을 뺀 원본,
                                                 zone 이 아니면 모두 inf (collision_monitor 소스 시간 초과를 피하려고 늘 냄)
  zone = demo_follow 가 상자 정지점 0.40 m 안 (끝 0.42 m) 에서 0.1 s 마다 내는 /drive_demo/box_zone (std_msgs/String JSON {"name","cx","cy","half"},
         "" = 끝). 1 s 넘게 안 오면 끝으로 봄.
배경 (11-21 사용자 결정): 상자 앞에서도 앞 라이다 정지를 남기되 상자 자체는 장애물로 보지 않음.
  /scan_merged 는 scan_body_mask 가 base_link x ≤ 0.285 · |y| ≤ 0.19 를 지워 차체 앞 3.5 cm 안을 거의 못 봄
  → 상자 구간에서는 앞 라이다 원본으로 가까운 곳을 봄 (차체 상자만 지움, 11-21 실물 차체 반사 미확인 → FRONT_GUARD=0 으로 끔).
⚠ 상자 사각형 안에 든 손 · 발은 상자로 보고 지움. AMCL 이 틀리면 상자가 안 지워져 멈추거나 (안전 쪽) 엉뚱한 곳을 지움.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from box_quiet import BODY_BOX, BOX_HALF, GUARD_MARGIN, in_body, in_box, scan_points

ZONE_STALE = 1.0


class Guard(Node):
    def __init__(self):
        super().__init__("box_scan_guard")
        self.merged_in = self.declare_parameter("merged_in", "/scan_merged").value
        self.front_in = self.declare_parameter("front_in", "/scan_front").value   # "" = 앞 원본 소스 안 씀 (모두 inf 만 냄)
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base = self.declare_parameter("base_frame", "base_link").value
        self.margin = self.declare_parameter("margin", GUARD_MARGIN).value
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.zone, self.t_zone = None, -1e9
        self.n_mask = self.n_front = 0
        self.warned = set()
        self.pub_m = self.create_publisher(LaserScan, "/scan_guard", qos_profile_sensor_data)
        self.pub_f = self.create_publisher(LaserScan, "/scan_front_guard", qos_profile_sensor_data)
        self.create_subscription(String, "/drive_demo/box_zone", self._zone, 10)
        self.create_subscription(LaserScan, self.merged_in, self._merged, qos_profile_sensor_data)
        if self.front_in:
            self.create_subscription(LaserScan, self.front_in, self._front, qos_profile_sensor_data)
        else:   # 앞 원본이 없으면 /scan_guard 를 받을 때마다 빈 앞 스캔을 냄 (_merged 에서)
            self.get_logger().info("front_in 없음 — /scan_front_guard 는 늘 비어 있음")
        self.get_logger().info(f"상자 점 지우기: {self.merged_in} → /scan_guard · {self.front_in or '(없음)'} → /scan_front_guard "
                               f"(상자 구간만) · 여유 {self.margin:.2f} m · 차체 {BODY_BOX}")

    def _zone(self, m):
        if not m.data:
            self.zone = None
            return
        try:
            z = json.loads(m.data)
            self.zone, self.t_zone = ((float(z["cx"]), float(z["cy"])), float(z.get("half", BOX_HALF)), z.get("name", "")), time.time()
        except (ValueError, KeyError, TypeError):
            self.zone = None

    def zone_now(self):
        if self.zone is None or time.time() - self.t_zone > ZONE_STALE:
            return None
        return self.zone

    def _tf(self, target, frame):
        try:
            t = self.tf.lookup_transform(target, frame, Time())
        except Exception:  # noqa: BLE001
            if (target, frame) not in self.warned:
                self.warned.add((target, frame))
                self.get_logger().warning(f"TF {target} ← {frame} 없음 — 상자 점을 지우지 않음 (멈출 수 있음)")
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w)

    @staticmethod
    def pts(m, tf):
        return scan_points(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max, tf)

    def _copy(self, m, ranges):
        o = LaserScan()
        o.header, o.angle_min, o.angle_max, o.angle_increment = m.header, m.angle_min, m.angle_max, m.angle_increment
        o.time_increment, o.scan_time, o.range_min, o.range_max = m.time_increment, m.scan_time, m.range_min, m.range_max
        o.ranges = ranges
        return o

    def _merged(self, m):
        z = self.zone_now()
        ranges = list(m.ranges)
        if z is not None:
            tf = self._tf(self.map_frame, m.header.frame_id)
            if tf is not None:
                h = z[1] + self.margin
                for i, p in enumerate(self.pts(m, tf)):
                    if p is not None and in_box(p, z[0], h):
                        ranges[i] = math.inf
                        self.n_mask += 1
        self.pub_m.publish(self._copy(m, ranges))
        if not self.front_in:
            self.pub_f.publish(self._copy(m, [math.inf] * len(m.ranges)))

    def _front(self, m):
        z = self.zone_now()
        if z is None:
            self.pub_f.publish(self._copy(m, [math.inf] * len(m.ranges)))
            return
        tb = self._tf(self.base, m.header.frame_id)
        tm = self._tf(self.map_frame, m.header.frame_id)
        ranges = [math.inf] * len(m.ranges)
        if tb is not None:
            pb = self.pts(m, tb)
            pm = self.pts(m, tm) if tm is not None else [None] * len(pb)
            h = z[1] + self.margin
            for i, (b, mp) in enumerate(zip(pb, pm)):
                if b is None or in_body(b):
                    continue
                if tm is not None and mp is not None and in_box(mp, z[0], h):
                    continue
                ranges[i] = m.ranges[i]
                self.n_front += 1
        self.pub_f.publish(self._copy(m, ranges))


def main():
    rclpy.init()
    n = Guard()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    print(f"box_scan_guard: 지운 상자 점 {n.n_mask} · 상자 구간 앞 원본 점 {n.n_front}", flush=True)


if __name__ == "__main__":
    main()
