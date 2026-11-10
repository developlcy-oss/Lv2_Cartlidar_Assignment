"""#157 Gazebo 시험 시나리오 (test/gazebo_box_park.sh 가 부름): 박스 앞 FINE 정렬 → turn_p17 → turn_0 → navigate (출발 구역).

가짜 인식 결과 (ARM_ALIGN, 화면 가운데) 만 내고 drive_node · Nav2 · obstacle_stop · twist_mux 는 진짜.
결과: 단계별 성공 · 걸린 시간 [sim s], 박스 앞 이후 비상 정지가 켜진 횟수, TFmini (중앙값) 최소 · 최대 -> JSON 한 줄 (stdout 마지막 줄).
종료 코드 0 = FINE · turn_p17 · turn_0 · navigate 모두 성공.
"""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces.action import AlignBase, ExecuteSkill
from robot_interfaces.msg import PerceptionResult
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String

ARM_ALIGN = 2


class Scenario(Node):
    def __init__(self, goal_xy_yaw):
        super().__init__("park_scenario")
        self.goal = goal_xy_yaw
        self.pub = self.create_publisher(PerceptionResult, "/perception/result", 1)
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

    def wait(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def main(self):
        self.wait(3.0)
        out = {}
        g = AlignBase.Goal(); g.target_distance = 0.13   # mission approach 0.13 과 같음 (float32, drive_node 는 1e-4 여유로 비교)
        g.tolerance = 0.02
        out["align_fine"] = self.run_goal(self.align, g, 240)
        t_park = self.t()
        self.wait(3.0)
        out["stop_after_align"] = dict(stop=self.stop, reason=self.reason)
        for name in ("turn_p17", "turn_0"):
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


def main():
    goal = [float(v) for v in sys.argv[1:4]] if len(sys.argv) >= 4 else [0.69365, 0.95, 3.14159265]
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    node = Scenario(goal)
    try:
        out = node.main()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(out, ensure_ascii=False))
    sys.exit(0 if all(out[k]["ok"] for k in ("align_fine", "turn_p17", "turn_0", "navigate")) else 1)


if __name__ == "__main__":
    main()
