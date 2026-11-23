#!/usr/bin/env python3
"""Jetson 라이다 값을 ssh 로 받아 노트북 ROS (localhost) 로 다시 냄 → RViz2 로 지도 위에 보기 (Wi-Fi DDS 안 씀). 읽기만.

  scan_view.sh 가 부름. Jetson 쪽: /scan_merged · /scan_front_filtered · /scan_rear_filtered 를 받아 JSON 줄로 stdout (5 Hz).
  노트북 쪽: /view/scan_merged (base_link) · /view/scan_front (laser_front) · /view/scan_rear (laser_rear) LaserScan,
            TF map → base_link (처음 = 계획 출발 자세 0.694, 0.950, 180°, RViz "2D Pose Estimate" 로 옮김),
            base_link → laser_front · laser_rear (URDF 값, roll π), /view/footprint (차체 외곽)
  amcl (11-15) 이면 TF map → base_link 를 Jetson 위치 추정 (AMCL) 값으로 따라 그림 = 위치 추정이 맞는지 · 주행 감시.
  mirror:=front|rear|both 로 그 라이다를 좌우 뒤집어 보기, rot 이면 앞 · 뒤 라이다를 자기 축으로 180° 돌려 보기 (장착 방향 확인용, 로봇은 안 바꿈)
"""
import json
import math
import re
import subprocess
import sys
import threading

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

REMOTE = r'''
source /opt/ros/lyrical/setup.bash >/dev/null 2>&1
source ~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/setup.bash >/dev/null 2>&1
exec python3 -u - <<"EOF"
import json, math, time, rclpy, tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
rclpy.init(); n = Node("scan_view_dump"); last = {}
def cb(t, m):
    now = time.time()
    if now - last.get(t, 0) < 0.2: return
    last[t] = now
    f = lambda x: round(x, 3) if x == x and x not in (float("inf"), float("-inf")) else None
    print(json.dumps({"t": t, "amin": m.angle_min, "inc": m.angle_increment, "rmin": m.range_min, "rmax": m.range_max,
                      "r": [f(x) for x in m.ranges]}), flush=True)
for t in ("/scan_merged", "/scan_front_filtered", "/scan_rear_filtered"):
    n.create_subscription(LaserScan, t, lambda m, t=t: cb(t, m), qos_profile_sensor_data)
buf = tf2_ros.Buffer(); tl = tf2_ros.TransformListener(buf, n)
def pose_tick():   # 11-15: 로봇이 추정한 자리 (AMCL map -> base_link) 도 5 Hz 로
    try: tr = buf.lookup_transform("map", "base_link", Time())
    except Exception: return
    q = tr.transform.rotation
    print(json.dumps({"t": "pose", "x": tr.transform.translation.x, "y": tr.transform.translation.y,
                      "yaw": math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))}), flush=True)
n.create_timer(0.2, pose_tick)
rclpy.spin(n)
EOF
'''
# 11-23: 출발 구역 (mission_plan.yaml return_check zone_center (0.734, 0.95) · zone_half 0.30 = 0.6 m 사각, poses.py START_ZONE_HALF) → /view/start_zone (map)
START_ZONE = (0.734, 0.95, 0.30)
# 11-23: 신호등 · 바코드 관찰 자리 (mission_plan.yaml observe.view_pose) → /view/observe_pose (map, 화살표)
OBSERVE_POSE = (0.575, 0.95, math.pi)
OUT = {"/scan_merged": ("/view/scan_merged", "base_link"), "/scan_front_filtered": ("/view/scan_front", "laser_front"),
       "/scan_rear_filtered": ("/view/scan_rear", "laser_rear")}


def q_yaw(y):
    return math.sin(y / 2), math.cos(y / 2)


def jetson_urdf_fixed(host):
    """Jetson 에 설치된 robot.urdf.xacro 의 laser_front yaw 가 0 이 아니면 True (PR #304, 11-15) — 라이다별 점을 Jetson 과 같은 TF 로 그림"""
    cmd = ("grep -o 'name=\"laser_front\"[^>]*' ~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/"
           "opencr_hardware/share/opencr_hardware/urdf/robot.urdf.xacro")
    try:
        out = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", host, cmd],
                             capture_output=True, text=True, timeout=15).stdout
    except Exception:
        return None
    m = re.search(r'rpy="([^"]*)"', out)
    if not m:
        return None
    parts = m.group(1).split(" ", 2)
    return len(parts) == 3 and parts[2].strip() not in ("0", "0.0", "0.000000")


class Relay(Node):
    def __init__(self, host, mirror):
        super().__init__("scan_relay")
        self.mirror = mirror
        self.follow = mirror == "amcl"   # 11-15: Jetson 위치 추정 자세를 따라 그림 (2D Pose Estimate 는 무시)
        self.got_pose = False
        self.pubs = {k: self.create_publisher(LaserScan, v[0], 10) for k, v in OUT.items()}
        self.fp = self.create_publisher(PolygonStamped, "/view/footprint", 10)
        self.zone = self.create_publisher(PolygonStamped, "/view/start_zone", 10)
        self.obs = self.create_publisher(PoseStamped, "/view/observe_pose", 10)
        self.tfb = TransformBroadcaster(self)
        self.stb = StaticTransformBroadcaster(self)
        self.pose = [0.6936, 0.95, math.pi]
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.on_pose, 10)
        st = []
        fixed = jetson_urdf_fixed(host)   # 11-15: 새 URDF (yaw 보정) 면 앞 yaw π · 뒤 0, 옛 URDF 면 앞 0 · 뒤 π
        yf, yr = (math.pi, 0.0) if fixed else (0.0, math.pi)
        self.get_logger().info("Jetson URDF 라이다 yaw 보정: " + {True: "있음 (PR #304)", False: "없음 (옛 판)", None: "못 읽음 → 옛 판으로 그림"}[fixed])
        # 11-20: 실물 앞 LDS-03 만 1.5 cm 낮춤 (laser_front z 0.0675 → 0.0525, 높이 70 mm 상자를 봄). 뒤는 그대로
        for child, x, z, roll, yaw in (("laser_front", 0.1318, 0.0525, math.pi, yf), ("laser_rear", -0.1318, 0.0675, math.pi, yr)):
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = "base_link", child
            t.transform.translation.x, t.transform.translation.z = x, z
            cr, sr, cy, sy = math.cos(roll / 2), math.sin(roll / 2), math.cos(yaw / 2), math.sin(yaw / 2)
            t.transform.rotation.x, t.transform.rotation.y = sr * cy, sr * sy   # q = Rz(yaw) · Rx(roll)
            t.transform.rotation.z, t.transform.rotation.w = cr * sy, cr * cy
            st.append(t)
        self.stb.sendTransform(st)
        self.create_timer(0.1, self.tick)
        self.n = 0
        self.proc = subprocess.Popen(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", host, "bash -s"],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        self.proc.stdin.write(REMOTE)
        self.proc.stdin.close()
        threading.Thread(target=self.reader, daemon=True).start()
        self.get_logger().info(f"{host} 라이다 받는 중 · 좌우 뒤집어 보기: {mirror or '없음'} · RViz 2D Pose Estimate 로 로봇 자리 옮기기")

    def on_pose(self, m):
        if self.follow:
            self.get_logger().warn("amcl 모드: 로봇 위치 추정 자세를 따라 그림 (2D Pose Estimate 는 노트북에서만, 로봇에는 안 감 — 무시)")
            return
        q = m.pose.pose.orientation
        self.pose = [m.pose.pose.position.x, m.pose.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))]
        self.get_logger().info(f"로봇 자리 → ({self.pose[0]:.3f}, {self.pose[1]:.3f}, {math.degrees(self.pose[2]):.1f}°)")

    def tick(self):
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, "map", "base_link"
        t.transform.translation.x, t.transform.translation.y = self.pose[0], self.pose[1]
        t.transform.rotation.z, t.transform.rotation.w = q_yaw(self.pose[2])
        self.tfb.sendTransform(t)
        p = PolygonStamped()
        p.header.stamp, p.header.frame_id = now, "base_link"
        for x, y in ((0.25, 0.07), (0.25, -0.07), (0.1328, -0.1676), (-0.2135, -0.1676), (-0.2135, 0.1676), (0.1328, 0.1676)):
            p.polygon.points.append(Point32(x=x, y=y))
        self.fp.publish(p)
        z = PolygonStamped()
        z.header.stamp, z.header.frame_id = now, "map"
        cx, cy, h = START_ZONE
        for x, y in ((cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)):
            z.polygon.points.append(Point32(x=x, y=y))
        self.zone.publish(z)
        o = PoseStamped()
        o.header.stamp, o.header.frame_id = now, "map"
        o.pose.position.x, o.pose.position.y = OBSERVE_POSE[0], OBSERVE_POSE[1]
        o.pose.orientation.z, o.pose.orientation.w = q_yaw(OBSERVE_POSE[2])
        self.obs.publish(o)

    def reader(self):
        for line in self.proc.stdout:
            try:
                d = json.loads(line)
            except ValueError:
                continue
            if d["t"] == "pose":
                if self.follow:
                    self.pose = [d["x"], d["y"], d["yaw"]]
                    if not self.got_pose:
                        self.got_pose = True
                        self.get_logger().info(f"로봇 위치 추정 자세 받음 ({d['x']:.3f}, {d['y']:.3f}, {math.degrees(d['yaw']):.1f}°) — 빨간 점이 벽에 겹치면 위치 추정 OK")
                continue
            topic, frame = OUT[d["t"]]
            r = [float("inf") if v is None else v for v in d["r"]]
            flip = (self.mirror in ("front", "both") and d["t"] == "/scan_front_filtered") or \
                   (self.mirror in ("rear", "both") and d["t"] == "/scan_rear_filtered")
            m = LaserScan()
            m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), frame
            m.angle_min, m.angle_increment = d["amin"], d["inc"]
            m.angle_max = d["amin"] + d["inc"] * (len(r) - 1)
            if flip:   # 각 θ → −θ (좌우 뒤집기)
                m.angle_min, m.angle_max, m.angle_increment = -m.angle_max, -d["amin"], d["inc"]
                r = r[::-1]
            if self.mirror == "rot" and d["t"] != "/scan_merged":   # 라이다 자기 축으로 180° 돌려 보기 (장착 방향 확인용)
                m.angle_min += math.pi
                m.angle_max += math.pi
            m.range_min, m.range_max = d["rmin"], d["rmax"]
            m.ranges = r
            self.pubs[d["t"]].publish(m)
            self.n += 1
            if self.n == 3:
                self.get_logger().info("라이다 값 들어옴")
        self.get_logger().error("ssh 끊김 — Jetson 연결 · 로봇 쪽 라이다 launch 확인")


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "cart11@cart11.local"
    mirror = sys.argv[2] if len(sys.argv) > 2 else ""
    rclpy.init()
    n = Relay(host, mirror)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    n.proc.terminate()


if __name__ == "__main__":
    main()
