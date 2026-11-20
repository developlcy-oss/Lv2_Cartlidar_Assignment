"""#157 Gazebo 시험 시나리오 (test/gazebo_box_park.sh 가 부름): 박스 앞 FINE 정렬 → turn_p17 → turn_0 → navigate (출발 구역).
--place (#160): 빈 장소 상자 앞 박스 접근 (box_only, 인식 결과를 내지 않음) → navigate. turn 은 하지 않음.
--center (11-20 상자 중앙 맞춤): FINE 뒤 /drive/align goal.center (center_box = Gazebo 실제 상자 --box x,y,yaw (world = map) 를
  그 순간 odom 으로 바꾼 값: odom 자세 ∘ (Gazebo 실제 자세)⁻¹ ∘ 상자) → turn_p17 → turn_0 → navigate. 로봇 실제 자세는 `gz model -m waffle_4wd -p`.
  통과 = FINE · center 성공 + 실제 옆 어긋남 < --lat_tol (1 cm) + 요 < --yaw_tol_deg (1.5°) + TFmini (중앙값) 0.13 ± 0.01 + turn_p17 성공

가짜 인식 결과 (ARM_ALIGN, 화면 가운데) 만 내고 drive_node · Nav2 · obstacle_stop · twist_mux 는 진짜.
결과: 단계별 성공 · 걸린 시간 [sim s], 박스 앞 이후 비상 정지가 켜진 횟수, TFmini (중앙값) 최소 · 최대 -> JSON 한 줄 (stdout 마지막 줄).
종료 코드 0 = FINE · turn_p17 · turn_0 · navigate 모두 성공.
"""
import json
import math
import re
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces.action import AlignBase, ExecuteSkill
from robot_interfaces.msg import PerceptionResult
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String

ARM_ALIGN = 2
FINE_TARGET = 0.13


def gz_pose(model="waffle_4wd"):
    """Gazebo 실제 모델 자세 (world x, y, yaw). 못 읽으면 None."""
    try:
        out = subprocess.run(["gz", "model", "-m", model, "-p"], capture_output=True, text=True, timeout=20).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    i = out.find("Pose")
    vecs = re.findall(r"\[\s*([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s*\]", out[i:] if i >= 0 else "")
    if len(vecs) < 2:
        return None
    return float(vecs[0][0]), float(vecs[0][1]), float(vecs[1][2])


def compose(a, b):
    """2D 자세 a ∘ b (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return -c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def center_error(robot, box):
    """상자 법선 (요 90° 마다 같음, 로봇 요에 가장 가까운 것) 기준 로봇 옆 어긋남 [m] · 요 오차 [rad] (control/box_center.hpp 와 같은 식)."""
    heading = wrap(robot[2] + math.remainder(box[2] - robot[2], math.pi / 2))
    c, s = math.cos(heading), math.sin(heading)
    dx, dy = robot[0] - box[0], robot[1] - box[1]
    return -s * dx + c * dy, wrap(heading - robot[2])


class Scenario(Node):
    def __init__(self, goal_xy_yaw, place=False, center=None):
        super().__init__("park_scenario")
        self.goal = goal_xy_yaw
        self.place = place
        self.center = center   # (상자 world x, y, yaw, lat_tol, yaw_tol_rad) 또는 None
        self.odom = None
        self.create_subscription(Odometry, "/odom", lambda m: setattr(self, "odom", m), 10)
        self.pub = self.create_publisher(PerceptionResult, "/perception/result", 1)
        if not place:   # 장소 상자는 정렬할 퍽이 없음 -> 인식 결과 없이도 되는지 본다
            self.create_timer(0.05, self.pub_result)
        self.stop = None
        self.stop_log = []      # (sim t, bool, reason)
        self.reason = ""
        self.rng = []
        self.create_subscription(Bool, "/perception/obstacle_stop", self.on_stop, 10)
        self.create_subscription(String, "/perception/obstacle_stop/reason", lambda m: setattr(self, "reason", m.data), 10)
        self.create_subscription(Range, "/tfmini/range_med", lambda m: self.rng.append((self.t(), m.range)), qos_profile_sensor_data)
        self.align = ActionClient(self, AlignBase, "/drive/align")
        self.skill = ActionClient(self, ExecuteSkill, "/drive/execute_skill")
        self.nav = ActionClient(self, NavigateToPose, "/drive/navigate")

    def t(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pub_result(self):
        m = PerceptionResult()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.mode = ARM_ALIGN
        m.offset_valid = True
        m.confidence = 1.0
        self.pub.publish(m)

    def on_stop(self, m):
        if self.stop != m.data:
            self.stop_log.append((round(self.t(), 2), m.data, self.reason))
        self.stop = m.data

    def run_goal(self, client, goal, timeout):
        t0 = self.t()
        if not client.wait_for_server(timeout_sec=20.0):
            return dict(ok=False, msg="server 없음", s=0.0)
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return dict(ok=False, msg="거절", s=0.0)
        rf = gh.get_result_async()
        end = time.time() + timeout
        while not rf.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not rf.done():
            gh.cancel_goal_async()
            return dict(ok=False, msg="시나리오 시간 초과", s=round(self.t() - t0, 1))
        r = rf.result()
        ok = r.status == 4  # SUCCEEDED
        msg = getattr(r.result, "message", "") if hasattr(r.result, "message") else ""
        return dict(ok=ok, msg=msg, status=r.status, s=round(self.t() - t0, 1))

    def truth(self):
        """Gazebo 실제 자세 + 옆 어긋남 · 요 오차 (상자 기준)."""
        p = gz_pose()
        if p is None:
            return None
        lat, yerr = center_error(p, self.center[:3])
        return dict(pose=[round(v, 4) for v in p], lateral_m=round(lat, 4), yaw_err_deg=round(math.degrees(yerr), 2))

    def range_now(self, sec=1.5):
        """최근 sec 동안 TFmini 중앙값 (/tfmini/range_med) 의 중앙값."""
        now = self.t()
        r = sorted(v for t, v in self.rng if t >= now - sec and math.isfinite(v))
        return round(r[len(r) // 2], 4) if r else None

    def run_center(self):
        """FINE 뒤: 실제 상자를 odom 으로 바꿔 goal.center → 실제 옆 · 요 · TFmini 판정."""
        res = dict(before=self.truth())
        if res["before"] is None or self.odom is None:
            res.update(ok=False, msg="Gazebo 실제 자세 (gz model) 또는 /odom 없음")
            return res
        o = self.odom.pose.pose
        odom_robot = (o.position.x, o.position.y, 2 * math.atan2(o.orientation.z, o.orientation.w))
        box_odom = compose(compose(odom_robot, inverse(tuple(res["before"]["pose"]))), self.center[:3])
        g = AlignBase.Goal()
        g.target_distance = FINE_TARGET
        g.tolerance = 0.02
        g.center = True
        g.center_box.header.frame_id = self.odom.header.frame_id or "odom"
        g.center_box.header.stamp = self.get_clock().now().to_msg()
        g.center_box.pose.position.x, g.center_box.pose.position.y = box_odom[0], box_odom[1]
        g.center_box.pose.position.z = 0.07
        g.center_box.pose.orientation.z, g.center_box.pose.orientation.w = math.sin(box_odom[2] / 2), math.cos(box_odom[2] / 2)
        res["box_odom"] = [round(v, 4) for v in box_odom]
        res["goal"] = self.run_goal(self.align, g, 300)
        self.wait(2.0)
        res["after"] = self.truth()
        res["tfmini"] = self.range_now()
        a = res["after"]
        lat_tol, yaw_tol = self.center[3], self.center[4]
        res["pass"] = dict(
            goal=res["goal"]["ok"],
            lateral=a is not None and abs(a["lateral_m"]) < lat_tol,
            yaw=a is not None and abs(math.radians(a["yaw_err_deg"])) < yaw_tol,
            tfmini=res["tfmini"] is not None and abs(res["tfmini"] - FINE_TARGET) <= 0.01)
        res["ok"] = all(res["pass"].values())
        return res

    def wait(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def main(self):
        self.wait(3.0)
        out = {}
        g = AlignBase.Goal(); g.target_distance = 0.13   # mission approach 0.13 과 같음 (float32, drive_node 는 1e-4 여유로 비교)
        g.tolerance = 0.02
        g.box_only = self.place
        out["align_fine"] = self.run_goal(self.align, g, 240)
        t_park = self.t()
        self.wait(3.0)
        out["stop_after_align"] = dict(stop=self.stop, reason=self.reason)
        if self.center:
            out["center"] = self.run_center()
            t_park = self.t()
        for name in () if self.place else ("turn_p17", "turn_0"):
            s = ExecuteSkill.Goal(); s.policy = name; s.timeout = 15.0
            out[name] = dict(self.run_goal(self.skill, s, 90), stop_now=self.stop)
        n = NavigateToPose.Goal()
        p = PoseStamped(); p.header.frame_id = "map"
        p.pose.position.x, p.pose.position.y = self.goal[0], self.goal[1]
        p.pose.orientation.z, p.pose.orientation.w = math.sin(self.goal[2] / 2), math.cos(self.goal[2] / 2)
        n.pose = p
        out["navigate"] = dict(self.run_goal(self.nav, n, 600), stop_now=self.stop)
        self.wait(2.0)
        after = [r for t, r in self.rng if t >= t_park]
        out["tfmini_after_align"] = dict(min=round(min(after), 3) if after else None, max=round(max(after), 3) if after else None)
        log = [x for x in self.stop_log if x[0] >= t_park]
        out["stop_after_park"] = dict(turn_on=sum(1 for x in log if x[1]), first=log[:4])
        return out


def opt(args, name, default):
    """--name value 를 꺼내고 나머지를 돌려줌."""
    if name in args:
        i = args.index(name)
        v = args[i + 1]
        del args[i:i + 2]
        return v
    return default


def main():
    args = sys.argv[1:]
    box = [float(v) for v in opt(args, "--box", "1.468,0.95,0").split(",")]   # SDF start_box_1 (world = map)
    lat_tol = float(opt(args, "--lat_tol", "0.01"))
    yaw_tol = math.radians(float(opt(args, "--yaw_tol_deg", "1.5")))
    place = "--place" in args
    center = "--center" in args
    argv = [a for a in args if a not in ("--place", "--center")]
    goal = [float(v) for v in argv[:3]] if len(argv) >= 3 else [0.69365, 0.95, 3.14159265]
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    node = Scenario(goal, place, (box[0], box[1], box[2], lat_tol, yaw_tol) if center else None)
    try:
        out = node.main()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(out, ensure_ascii=False))
    if place:
        need = ("align_fine", "navigate")
    elif center:
        need = ("align_fine", "center", "turn_p17")   # turn_0 · navigate 는 기록만
    else:
        need = ("align_fine", "turn_p17", "turn_0", "navigate")
    sys.exit(0 if all(out[k]["ok"] for k in need) else 1)


if __name__ == "__main__":
    main()
