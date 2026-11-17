#!/usr/bin/env python3
"""진단 (11-17): REC=1 녹화에서 감시가 막힘을 낸 순간의 로봇 기울기 (IMU pitch) · 점 거리 · 출처.
   python3 diag_floor.py <기록 폴더>/bag"""
import json, math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from std_msgs.msg import String
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
imu, st = [], []
while r.has_next():
    tp, d, t = r.read_next()
    if tp == "/imu/data":
        m = deserialize_message(d, Imu); q = m.orientation
        pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        imu.append((t * 1e-9, math.degrees(pitch), math.degrees(roll), m.linear_acceleration.x))
    elif tp == "/avoid/status":
        s = json.loads(deserialize_message(d, String).data); st.append((t * 1e-9, s))
imu = np.array(imu)
print(f"IMU {len(imu)} · 감시 소식 {len(st)} · pitch 범위 {imu[:, 1].min():.2f} ~ {imu[:, 1].max():.2f}° · roll {imu[:, 2].min():.2f} ~ {imu[:, 2].max():.2f}°")
bl = [(t, s) for t, s in st if s["blocked"] or s["n"] >= 5]
print(f"띠 안 점 ≥ 5 또는 막힘: {len(bl)} 번")
for t, s in bl[:25]:
    k = int(np.argmin(np.abs(imu[:, 0] - t)))
    rx, ry, yaw = s["robot"]
    ds = [math.hypot(px - rx, py - ry) for px, py in s["pts"]] or [float("nan")]
    print(f"  t {t - st[0][0]:6.2f} · {s['src']:<12} n {s['n']:3d} · {s['why'] or '-':<16} · 점 거리 {min(ds):.2f}~{max(ds):.2f} m · pitch {imu[k, 1]:+.2f}° · ax {imu[k, 3]:+.2f}")
