#!/usr/bin/env python3
"""Jetson 에서 ssh 로 실행 (odom_view.sh 가 표준 입력으로 보냄). 읽기만 — 아무것도 발행하지 않음.
10 Hz 로 한 줄 JSON: 바퀴 odom · EKF · AMCL 자세, IMU yaw 적분, map→odom, 명령 속도."""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Stream(Node):
    def __init__(self):
        super().__init__('pc_odom_view_stream')
        self.s = {'wheel': None, 'ekf': None, 'amcl': None, 'imu_yaw': 0.0, 'map_odom': None, 'cmd': None,
                  'wheel_w': None, 'imu_w': None}
        self.imu_t = None
        self.create_subscription(Odometry, '/odom', lambda m: self.odo('wheel', m), 20)
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: self.odo('ekf', m), 20)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/cmd_vel', lambda m: self.s.__setitem__('cmd', [m.twist.linear.x, m.twist.angular.z]), 10)
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.create_timer(0.1, self.out)

    def odo(self, k, m):
        p = m.pose.pose
        self.s[k] = [p.position.x, p.position.y, yaw_of(p.orientation)]
        if k == 'wheel':
            self.s['wheel_w'] = m.twist.twist.angular.z

    def on_amcl(self, m):
        p = m.pose.pose
        self.s['amcl'] = [p.position.x, p.position.y, yaw_of(p.orientation)]

    def on_imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        w = m.angular_velocity.z
        if self.imu_t is not None and 0 < t - self.imu_t < 0.5:
            self.s['imu_yaw'] += w * (t - self.imu_t)
        self.imu_t = t
        self.s['imu_w'] = w

    def out(self):
        try:
            tf = self.buf.lookup_transform('map', 'odom', rclpy.time.Time())
            tr = tf.transform
            self.s['map_odom'] = [tr.translation.x, tr.translation.y, yaw_of(tr.rotation)]
        except Exception:
            pass
        self.s['t'] = time.time()
        sys.stdout.write(json.dumps(self.s) + '\n')
        sys.stdout.flush()


def main():
    rclpy.init()
    n = Stream()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, BrokenPipeError, Exception):
        pass


if __name__ == '__main__':
    main()
