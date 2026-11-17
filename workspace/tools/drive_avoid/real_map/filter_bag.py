"""bag 에서 SLAM 에 필요한 토픽만 새 bag 으로. ROTATE_MERGED=1 이면 /scan_merged 를 base_link 둘레로 180° 돌림 (11-16 07:35 녹화는 스캔 앞뒤가 반대 — 진단 diag_trans.py). /tf 의 map → odom (녹화 때 AMCL) 은 빼고 따로 CSV 로 남김 (정렬 첫 추정용).
   python3 filter_bag.py <원본 bag 폴더> <새 bag 폴더>"""
import sys, csv, math
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
import os
ROT = os.environ.get("ROTATE_MERGED") == "1"

src, dst = sys.argv[1:3]
KEEP = {"/tf", "/tf_static", "/scan_merged", "/scan_front_filtered", "/scan_rear_filtered", "/odometry/filtered", "/imu/data"}
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=src, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
w = rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=dst, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
for m in r.get_all_topics_and_types():
    if m.name in KEEP:
        w.create_topic(m)
amcl = open(dst.rstrip("/") + "_amcl_map_odom.csv", "w", newline="")
cw = csv.writer(amcl); cw.writerow(["t", "x", "y", "yaw"])
n = {k: 0 for k in KEEP}; dropped = 0; parents = {}
while r.has_next():
    topic, data, t = r.read_next()
    if topic not in KEEP:
        continue
    if topic == "/tf":
        msg = deserialize_message(data, TFMessage)
        keep = []
        for tr in msg.transforms:
            parents[(tr.header.frame_id, tr.child_frame_id)] = parents.get((tr.header.frame_id, tr.child_frame_id), 0) + 1
            if tr.header.frame_id == "map":
                q = tr.transform.rotation
                cw.writerow([tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9, tr.transform.translation.x, tr.transform.translation.y,
                             math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))])
                dropped += 1
            else:
                keep.append(tr)
        if not keep:
            continue
        msg.transforms = keep
        data = serialize_message(msg)
    if ROT and topic == "/scan_merged":
        sm = deserialize_message(data, LaserScan); nr = len(sm.ranges); assert nr == 720, nr
        sm.ranges = list(sm.ranges[nr // 2:]) + list(sm.ranges[:nr // 2])   # 새 각 a = 옛 각 a + π
        if len(sm.intensities) == nr: sm.intensities = list(sm.intensities[nr // 2:]) + list(sm.intensities[:nr // 2])
        data = serialize_message(sm)
    w.write(topic, data, t)
    n[topic] += 1
del w
print("쓴 메시지:", n)
print("뺀 map→* TF:", dropped)
print("/tf 부모→자식:", parents)
