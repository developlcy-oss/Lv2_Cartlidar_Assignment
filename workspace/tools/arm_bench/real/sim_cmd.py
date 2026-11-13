#!/usr/bin/env python3
"""시뮬 스킬 서버에 명령 하나 보내고 결과를 기다림 (호스트 ROS Lyrical). arm_cmd.sh sim 이 부름.
  python3 sim_cmd.py <명령> [timeout s]"""
import json
import sys
import time

import rclpy
from std_msgs.msg import String

cmd = sys.argv[1]
timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0
rclpy.init()
n = rclpy.create_node("sim_cmd")
got = []
n.create_subscription(String, "/sim_skill/result", lambda m: got.append(m.data), 10)
pub = n.create_publisher(String, "/sim_skill/cmd", 10)
t0 = time.time()
while pub.get_subscription_count() == 0 and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.2)
if pub.get_subscription_count() == 0:
    print("시뮬 스킬 서버가 안 보임 (tools/sim_skill.sh start 했는지, 컨테이너 · 호스트 ROS_DOMAIN_ID 같은지)")
    sys.exit(2)
time.sleep(0.3)
pub.publish(String(data=cmd))
while time.time() - t0 < timeout:
    rclpy.spin_once(n, timeout_sec=0.2)
    for g in got:
        r = json.loads(g)
        if r.get("cmd") == cmd:
            print(("성공  " if r["ok"] else "실패  ") + f"{cmd}: {r['msg']} ({r['t']} s)")
            sys.exit(0 if r["ok"] else 1)
print(f"시간 초과 {timeout:.0f} s: {cmd}")
sys.exit(3)
