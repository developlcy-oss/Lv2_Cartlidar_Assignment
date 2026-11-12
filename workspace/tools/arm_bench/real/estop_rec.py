#!/usr/bin/env python3
"""E-stop 시험 기록 (2026-11-12): /joint_states 실제 · arm_controller 원하는 위치 · /e_stop 을 CSV 로. 600 s 또는 Ctrl+C."""
import datetime, math, os, sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
N = ["joint1", "joint2", "joint3", "joint4", "joint5"]
out = os.path.expanduser(f"~/실측/estop_{datetime.datetime.now():%m%d_%H%M%S}")
os.makedirs(out, exist_ok=True)
f = open(os.path.join(out, "data.csv"), "w")
f.write("t,estop," + ",".join(f"q{i+1}" for i in range(5)) + "," + ",".join(f"ref{i+1}" for i in range(5)) + "\n")
st = {"q": [math.nan]*5, "ref": [math.nan]*5, "e": -1}
T0 = time.time()
rclpy.init(); n = rclpy.create_node("estop_rec")
def js(m):
    d = dict(zip(m.name, m.position))
    if "joint1" not in d: return
    st["q"] = [math.degrees(d[k]) for k in N]
    f.write(f"{time.time()-T0:.3f},{st['e']}," + ",".join(f"{v:.3f}" for v in st["q"] + st["ref"]) + "\n")
def cs(m):
    d = dict(zip(m.joint_names, m.reference.positions))
    st["ref"] = [math.degrees(d.get(k, math.nan)) for k in N]
def es(m): st["e"] = int(m.data)
n.create_subscription(JointState, "/joint_states", js, 50)
n.create_subscription(JointTrajectoryControllerState, "/arm_controller/controller_state", cs, 50)
n.create_subscription(Bool, "/e_stop", es, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
print("기록", out, flush=True)
try:
    while rclpy.ok() and time.time() - T0 < 600:
        rclpy.spin_once(n, timeout_sec=0.1)
except KeyboardInterrupt:
    pass
f.close(); print("끝", out)
