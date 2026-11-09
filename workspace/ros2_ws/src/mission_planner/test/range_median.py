"""시험 전용: /tfmini/range (Gazebo 잡음 ±3 cm) → 최근 7 개 중앙값 → /tfmini/range_med (보정된 TFmini 흉내)."""
import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range


class Med(Node):
    def __init__(self):
        super().__init__("range_median")
        self.q = deque(maxlen=7)
        self.pub = self.create_publisher(Range, "/tfmini/range_med", qos_profile_sensor_data)
        self.create_subscription(Range, "/tfmini/range", self.cb, qos_profile_sensor_data)

    def cb(self, m):
        self.q.append(m.range)
        m.range = float(statistics.median(self.q))
        self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(Med())


if __name__ == "__main__":
    main()
