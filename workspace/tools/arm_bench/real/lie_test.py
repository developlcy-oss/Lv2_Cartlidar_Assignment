#!/usr/bin/env python3
"""누운 퍽 세우기 시험 한 번 + 기록 (2026-11-12, 젯슨, 브링업 · kbd_joy 가 떠 있을 때).
인식 켜기 -> 누운 퍽 위치 읽기 -> target_look -> lie_upright:base_link:<상자 x>:<상자 y>:<상자 요 deg>. 영상 · 관절 · 결과 기록.

    python3 ~/lie_test.py <상자 x m> <상자 y m> <상자 요 deg> [메모]      예: python3 ~/lie_test.py 0.29 0.0 0 첫시도

기록: ~/실측/lie_<MMDD_HHMMSS>/  arm.mp4 (팔 카메라 10 fps, 640x360) · joints.csv (관절 1~5 도 · 집게 rad) · log.txt
가져오기 (PC): scp -r cart11@cart11.local:~/실측/lie_* <PC 폴더>/
"""
import csv
import datetime
import math
import os
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces.msg import PuckArray
from sensor_msgs.msg import Image, JointState

NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper_roll"]
OUT = os.path.expanduser(f"~/실측/lie_{datetime.datetime.now():%m%d_%H%M%S}")
os.makedirs(OUT, exist_ok=True)
LOG = open(os.path.join(OUT, "log.txt"), "w", encoding="utf-8")
T0 = time.time()


def P(s):
    s = f"[{time.time() - T0:6.1f}s] {s}"
    print(s, flush=True)
    LOG.write(s + "\n")
    LOG.flush()


class Recorder:
    def __init__(self, node):
        self.vw, self.n_img, self.pucks = None, 0, []
        self.jf = open(os.path.join(OUT, "joints.csv"), "w", newline="")
        self.jw = csv.writer(self.jf)
        self.jw.writerow(["t"] + [f"{k}_deg" for k in NAMES[:5]] + ["gripper_rad"])
        node.create_subscription(Image, "/arm_camera/image_raw", self.on_img, qos_profile_sensor_data)
        node.create_subscription(JointState, "/joint_states", self.on_js, 10)
        node.create_subscription(PuckArray, "/perception/pucks", lambda m: self.pucks.append(m), 1)

    def on_img(self, m):
        self.n_img += 1
        if self.n_img % 3:                       # 30 -> 10 fps
            return
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        a = cv2.resize(a, (640, 360))
        cv2.putText(a, f"{time.time() - T0:5.1f}s", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        if self.vw is None:
            self.vw = cv2.VideoWriter(os.path.join(OUT, "arm.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (640, 360))
        self.vw.write(a)

    def on_js(self, m):
        if "joint1" not in m.name:
            return
        d = dict(zip(m.name, m.position))
        self.jw.writerow([f"{time.time() - T0:.3f}"] + [f"{math.degrees(d.get(k, float('nan'))):.2f}" for k in NAMES[:5]]
                         + [f"{d.get('gripper_roll', float('nan')):.3f}"])

    def close(self):
        if self.vw is not None:
            self.vw.release()
        self.jf.close()


def run(args):
    P("$ ros2 " + " ".join(args))
    out = subprocess.run(["ros2"] + args, capture_output=True, text=True).stdout
    keep = [l.strip() for l in out.splitlines() if l.strip().startswith(("success", "message", "response", "robot_interfaces.srv"))]
    P("  " + (" | ".join(keep) or out[-400:].replace("\n", " ")))
    return "success: true" in out or "success=True" in out


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        return
    bx, by, byaw = (float(v) for v in sys.argv[1:4])
    P(f"누운 퍽 세우기 시험 {OUT}  상자 ({bx}, {by}, {byaw}°)  메모: {' '.join(sys.argv[4:])}")
    rclpy.init()
    node = rclpy.create_node("lie_test")
    rec = Recorder(node)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        run(["service", "call", "/perception/set_mode", "robot_interfaces/srv/SetPerceptionMode", "{mode: 4}"])
        t = time.time()
        while time.time() - t < 12 and not (rec.pucks and rec.pucks[-1].pucks):
            time.sleep(0.1)
        if not (rec.pucks and rec.pucks[-1].pucks):
            P("!! /perception/pucks 에 퍽이 없음 (팔 카메라 화면에 퍽이 보이는지 확인)")
            return
        pk = rec.pucks[-1].pucks[0]
        p = pk.pose.position
        P(f"인식한 퍽 (base_link): x {p.x:.3f} y {p.y:.3f} z {p.z:.3f}  tag {pk.tag_id} shape {pk.shape} "
          f"standing {pk.standing} (orientation_valid {pk.orientation_valid})")
        if not run(["action", "send_goal", "/manipulator/execute_skill", "robot_interfaces/action/ExecuteSkill",
                    f"{{policy: 'target_look:base_link:{p.x:.3f}:{p.y:.3f}:{p.z:.3f}', timeout: 30.0}}"]):
            P("!! target_look 실패 -> 집기 안 함")
            return
        run(["action", "send_goal", "/manipulator/execute_skill", "robot_interfaces/action/ExecuteSkill",
             f"{{policy: 'lie_upright:base_link:{bx:.3f}:{by:.3f}:{byaw:.1f}', timeout: 60.0}}"])
        time.sleep(2.0)                          # 끝난 뒤 모습도 조금 더 기록
    finally:
        ex.shutdown()
        rec.close()
        P(f"기록 끝: {OUT} (영상 {rec.n_img // 3} 장)")


if __name__ == "__main__":
    main()
