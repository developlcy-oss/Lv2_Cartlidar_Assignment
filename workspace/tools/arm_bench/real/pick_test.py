#!/usr/bin/env python3
"""집기 시험 한 번 + 기록 (2026-11-12, 젯슨, 브링업 · kbd_joy 가 떠 있을 때).
인식 켜기 -> 퍽 위치 읽기 -> target_look -> toppick_tilt. 그동안 팔 카메라 영상 · 관절 · 결과를 기록.
2026-11-14: 미션 차례 그대로 이어서 (Isaac chain_pallet_eval --scripted_pick 과 같은 흐름):
  집을 퍽 = J1 축에 가장 가까운 퍽 (mission 은 가까운 퍽부터, 나머지 = 옆 퍽) -> --load 면 pallet_load_<열>_<칸>_tilt ->
  --recheck 면 처음 target_look 자리로 다시 가서 옆 퍽을 다시 잼 (옆 퍽 밀림 = 쥔 뒤 들기 · 운반 중 침).

    python3 ~/pick_test.py [메모]                          # 집기만 (예전과 같음)
    python3 ~/pick_test.py --load 1 1 --recheck [메모]      # 집기 -> 1 열 1 칸 넣기 -> 옆 퍽 다시 잼

기록: ~/실측/pick_<MMDD_HHMMSS>/  arm.mp4 (팔 카메라 10 fps, 640x360) · joints.csv (관절 1~5 도 · 집게 rad) · log.txt
가져오기 (PC): scp -r cart11@cart11.local:~/실측/pick_* <PC 폴더>/
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
OUT = os.path.expanduser(f"~/실측/pick_{datetime.datetime.now():%m%d_%H%M%S}")
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


def parse():
    """--load <열> <칸> · --recheck · 나머지 = 메모."""
    av, load, recheck, memo = sys.argv[1:], None, False, []
    i = 0
    while i < len(av):
        if av[i] == "--load" and i + 2 < len(av):
            load = (int(av[i + 1]), int(av[i + 2]))
            i += 3
        elif av[i] == "--recheck":
            recheck = True
            i += 1
        else:
            memo.append(av[i])
            i += 1
    return load, recheck, " ".join(memo)


J1_X = 0.096  # base_link 에서 관절 1 축 x (manipulator.yaml ik · 팀 URDF)


def look_and_read(rec, p, wait=3.0):
    """target_look (p) 로 가서 머문 뒤 들어온 퍽 목록 (base_link xyz)."""
    if not run(["action", "send_goal", "/manipulator/execute_skill", "robot_interfaces/action/ExecuteSkill",
                f"{{policy: 'target_look:base_link:{p[0]:.3f}:{p[1]:.3f}:{p[2]:.3f}', timeout: 30.0}}"]):
        return None
    rec.pucks.clear()
    t = time.time()
    while time.time() - t < wait:
        time.sleep(0.1)
    arr = [m for m in rec.pucks if m.pucks]
    return [(q.pose.position.x, q.pose.position.y, q.pose.position.z) for q in arr[-1].pucks] if arr else []


def main():
    load, recheck, memo = parse()
    P(f"집기 시험 {OUT}  넣기 {load}  다시 잼 {recheck}  메모: {memo}")
    rclpy.init()
    node = rclpy.create_node("pick_test")
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
        all_pk = rec.pucks[-1].pucks
        pk = min(all_pk, key=lambda q: math.hypot(q.pose.position.x - J1_X, q.pose.position.y))  # 가까운 퍽부터 (mission 과 같음)
        p = pk.pose.position
        P(f"인식한 퍽 {len(all_pk)} 개, 집을 퍽 (J1 에 가장 가까움, base_link): x {p.x:.3f} y {p.y:.3f} z {p.z:.3f}  tag {pk.tag_id} shape {pk.shape}"
          f"  J1 축 {math.hypot(p.x - J1_X, p.y):.3f} m")
        look = (p.x, p.y, p.z)
        before = look_and_read(rec, look)
        if before is None:
            P("!! target_look 실패 -> 집기 안 함")
            return
        nbs = [q for q in before if math.hypot(q[0] - p.x, q[1] - p.y) > 0.03]  # 옆 퍽 (집을 퍽에서 3 cm 밖)
        P("옆 퍽 (집기 전): " + (", ".join(f"({q[0]:.3f}, {q[1]:.3f})" for q in nbs) or "없음"))
        ok = run(["action", "send_goal", "/manipulator/execute_skill", "robot_interfaces/action/ExecuteSkill",
                  "{policy: toppick_tilt, timeout: 40.0}"])
        P(f"집기 {'성공' if ok else '실패'}")
        if ok and load:
            ok = run(["action", "send_goal", "/manipulator/execute_skill", "robot_interfaces/action/ExecuteSkill",
                      f"{{policy: pallet_load_{load[0]}_{load[1]}_tilt, timeout: 90.0}}"])
            P(f"넣기 {load[0]}열 {load[1]} {'성공' if ok else '실패'}")
        time.sleep(2.0)                          # 끝난 뒤 모습도 조금 더 기록
        if recheck and nbs:
            after = look_and_read(rec, look)
            for q in nbs:
                d = min((math.hypot(q[0] - w[0], q[1] - w[1]) for w in (after or [])), default=float("nan"))
                P(f"옆 퍽 ({q[0]:.3f}, {q[1]:.3f}) 다시 잼: 가장 가까운 퍽까지 {d * 1000:.0f} mm "
                  + ("(그대로)" if d < 0.008 else "!! 밀림 의심 (인식 오차 수 mm 감안, 영상 확인)" if d == d else "!! 안 보임 (넘어짐 · 밀림?)"))
    finally:
        ex.shutdown()
        rec.close()
        P(f"기록 끝: {OUT} (영상 {rec.n_img // 3} 장)")


if __name__ == "__main__":
    main()
