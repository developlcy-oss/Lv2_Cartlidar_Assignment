#!/usr/bin/env python3
"""실물 팔 실측 도우미 (2026-11-11, 젯슨에서 실행). 기록/실측_보정_가이드.md 의 1 ~ 5 번을 메뉴로.

    source ~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/setup.bash
    python3 ~/meas_session.py                          브링업 모드 (arm_controller 액션)
    python3 ~/meas_session.py --direct /dev/ttyACM1    직접 모드 (브링업 끄고 팔 OpenCR 에 바로, 팀 arm_bench)

- bringup 은 떠 있고 미션은 시작하지 않은 상태. 시작 정리 (STOW -> DRIVE) 가 끝난 뒤 실행.
- 이동마다 계획을 보여 주고 Enter 를 눌러야 보냄. 이동 중 Ctrl+C = 목표 취소 + 그 자리 정지.
- 관절 속도 0.2 rad/s 이하. 관절 1 을 0.3 rad 넘게 돌릴 때는 자동으로 높은 자세를 거쳐 돌림.
- 결과: ~/실측/*.csv, 화면 출력 전체는 ~/실측/log.txt. 기록 줄은 [기록] 으로 시작.
"""
import csv
import datetime
import math
import os
import subprocess
import sys
import time

try:   # 브링업 모드만 ROS 필요 (직접 모드는 없어도 됨)
    import rclpy
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from trajectory_msgs.msg import JointTrajectoryPoint
except ImportError:
    rclpy = None

NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper_roll"]
OPEN, CLOSE = -0.764, 0.923
VEL = 0.2          # rad/s
J1_TURN = 0.3      # rad, 이보다 크게 돌리면 높은 자세 경유
HIGH = [-0.6875, -0.1959, 1.9306, 0.0]   # 관절 2 ~ 5, 앞 높은 자세 (관절 1 회전용)
POSES = {          # 관절 1 ~ 5 (rad), 설계 높이 · 수평 거리 (mm, 관절 1 축 기준)
    "0": ([0, 0, 0, 0, 0], 325, 316),
    "A": ([0, -0.2454, 0.2999, 0.9927, 0], 175, 200),
    "B": ([0, -0.338, 0.559, 0.564, 0], 180, 220),
    "C": ([2.9998, -0.1708, -0.3884, 1.6063, 0], 248, 191),
}
SLOTS = {}         # 칸: (관절 1, 칸 위 6 cm 관절 2 ~ 5, 잡는 자리 관절 2 ~ 5), unload_traj.json 60°
for row, cells in {
    "3": [("1", 2.9998, [-0.1708, -0.3884, 1.6063, 0], [-0.3147, 0.2594, 1.1025, 0]),
          ("0", 2.7163, [-0.1708, -0.3883, 1.6063, 0], [-0.3147, 0.2594, 1.1025, 0])],
    "2": [("1", 2.9503, [-0.5685, 0.0337, 1.5821, 0], [-0.7122, 0.6200, 1.1394, 0]),
          ("0", 2.5675, [-0.5687, 0.0338, 1.5822, 0], [-0.7125, 0.6201, 1.1396, 0])],
    "1": [("1", 2.8019, [-0.9413, 0.2481, 1.7404, 0], [-1.1893, 0.8260, 1.4105, 0]),
          ("0", 2.1215, [-0.9415, 0.2481, 1.7406, 0], [-1.1896, 0.8260, 1.4108, 0])],
}.items():
    for idx, j1, above, grasp in cells:
        mirror = {"1": "2", "0": "3"}[idx]
        SLOTS[f"{row}-{idx}"] = (j1, above, grasp)          # 왼쪽 칸 = 관절 1 +
        SLOTS[f"{row}-{mirror}"] = (-j1, above, grasp)      # 오른쪽 칸 = 관절 1 −
Q_FRONT = [0, -0.2923, -0.1141, 1.9772, 0]   # 팀 운반 시작 자세
STOW = [1.554, -2.088, 1.549, -1.868, 0.043, 0.923]      # 팀 manipulator.yaml (전원 끄기 전 / 켤 때 자세)
DRIVE = [-0.0245, -1.6122, 1.2901, 1.1888, 0.0, 0.923]
BENCH = os.path.expanduser("~/Lv2_Cartlidar_Assignment/workspace/tools/arm_bench")

OUT = os.path.expanduser("~/실측")
os.makedirs(OUT, exist_ok=True)
LOG = open(os.path.join(OUT, "log.txt"), "a", encoding="utf-8")


class Back(Exception):
    """q 입력 -> 메뉴로."""


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s, flush=True)
    LOG.write(s + "\n")
    LOG.flush()


def ask(prompt, kind=str, empty_ok=False):
    while True:
        s = input(f"  > {prompt}: ").strip()
        LOG.write(f"  > {prompt}: {s}\n")
        if s == "q":
            raise Back
        if s == "" and empty_ok:
            return None
        try:
            return kind(s)
        except ValueError:
            print("    숫자로 다시 (q = 메뉴)")


def deg(v):
    return " ".join(f"{math.degrees(x):6.1f}" for x in v)


def record(fname, header, row):
    path = os.path.join(OUT, fname)
    new = not os.path.exists(path)
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if new:
            w.writerow(header)
        w.writerow(row)
    P(f"[기록] {fname}: " + ",".join(str(x) for x in row))


class Arm:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node("meas_session")
        self.js = None
        self.n.create_subscription(JointState, "/joint_states", self._cb, 10)
        self.estop = False                               # /e_stop: 잠기면 이동 취소, 풀리면 남은 이동을 천천히 다시
        self.n.create_subscription(Bool, "/e_stop", lambda m: setattr(self, "estop", m.data), QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.ac = ActionClient(self.n, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        P("arm_controller 기다리는 중 ...")
        if not self.ac.wait_for_server(timeout_sec=10):
            P("arm_controller 액션 서버 없음. bringup 확인.")
            sys.exit(1)
        q, g = self.read()
        self.cmd, self.grip = q, min(max(g, OPEN), CLOSE)

    rec = None                                           # 떨림 측정 중이면 [(t, 각 6, 전류 6, 명령 6)]

    def _cb(self, m):
        if "joint1" in m.name:
            self.js = m
            if self.rec is not None:
                d = dict(zip(m.name, m.position))
                e = dict(zip(m.name, m.effort)) if len(m.effort) == len(m.name) else {}
                self.rec.append((time.time(), [d.get(k, float("nan")) for k in NAMES],
                                 [e.get(k, float("nan")) for k in NAMES], list(self.cmd) + [self.grip]))

    def read(self, avg=0.5):
        """관절 1 ~ 5 (rad) 와 집게 값, avg 초 평균."""
        acc, t0 = [], time.time()
        while time.time() - t0 < avg or not acc:
            self.js = None
            rclpy.spin_once(self.n, timeout_sec=0.2)
            if self.js:
                d = dict(zip(self.js.name, self.js.position))
                acc.append([d.get(k, float("nan")) for k in NAMES])
            if time.time() - t0 > 5 and not acc:
                P("/joint_states 가 안 옴")
                sys.exit(1)
        v = [sum(a[i] for a in acc) / len(acc) for i in range(6)]
        return v[:5], v[5]

    def show(self, tag="지금"):
        q, g = self.read()
        P(f"  {tag} 관절 1~5 (도): {deg(q)} | 집게 {g:+.3f}")
        return q, g

    def _send(self, q, grip, T):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q] + [float(grip)]
        pt.time_from_start = Duration(sec=int(T), nanosec=int((T % 1) * 1e9))
        goal.trajectory.points = [pt]
        f = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=5)
        gh = f.result()
        if gh is None or not gh.accepted:
            P("  !! 목표 거부됨 (관절 범위 · HOLD 상태 확인)")
            return False
        r = gh.get_result_async()
        try:
            t_end = time.time() + T + 10
            while not r.done() and time.time() < t_end:
                rclpy.spin_once(self.n, timeout_sec=0.02)
                if self.estop:                           # E-stop: 궤적이 계속 나아가지 않게 취소 -> 풀린 뒤 남은 이동
                    gh.cancel_goal_async()
                    P("  !! E-stop: 이동 멈춤. 리셋 (r) 하면 남은 이동을 이어서 함")
                    while self.estop:
                        rclpy.spin_once(self.n, timeout_sec=0.05)
                    self.wait(1.0)
                    cur, _ = self.read(0.2)
                    dq = max(abs(a - b) for a, b in zip(q, cur))
                    P("  리셋됨: 남은 이동 이어서")
                    return self._send(q, grip, max(2.0, dq / VEL))
        except KeyboardInterrupt:
            gh.cancel_goal_async()
            cur, g = self.read(0.2)
            self._send(cur, g, 1.0)
            P("  !! 취소: 그 자리 정지")
            raise Back
        res = r.result()
        if res is None:
            P("  !! 결과 시간 초과")
            return False
        if res.result.error_code != 0:
            P(f"  !! 컨트롤러 오류 {res.result.error_code} {res.result.error_string}")
        self.cmd, self.grip = list(q), grip
        return True

    def move(self, q, grip=None, confirm=True, T=None, label="", route=True):
        grip = self.grip if grip is None else grip
        cur, g = self.read(0.2)
        if any(math.isnan(x) for x in cur + [g, grip] + list(q)):
            P("  !! 관절값에 nan: 보드에서 관절 데이터가 안 옴 (모터 전원 · REARM 확인). 이동 안 함")
            raise Back
        if route and abs(q[0] - cur[0]) > J1_TURN:          # 관절 1 큰 회전은 높은 자세에서
            if any(abs(a - b) > 0.05 for a, b in zip(cur[1:], HIGH)):
                self.move([cur[0]] + HIGH, grip, confirm, label="높은 자세 (회전 전)", route=False)
            self.move([q[0]] + HIGH, grip, confirm, label="관절 1 회전", route=False)
            cur, _ = self.read(0.2)
        dq = max([abs(a - b) for a, b in zip(q, cur)] + [abs(grip - self.grip) / 4])
        T = T or max(3.0, dq / VEL)
        P(f"  이동 {label}: 목표 (도) {deg(q)} | 집게 {grip:+.3f} | {T:.1f} 초")
        if confirm:
            s = input("    Enter = 실행, s = 건너뜀, q = 메뉴: ").strip()
            if s == "q":
                raise Back
            if s == "s":
                return False
        ok = self._send(q, grip, T)
        time.sleep(0.5)
        self.show("도착")
        return ok

    def close(self):
        pass

    def wait(self, sec):
        """기다리는 동안도 /joint_states 를 받음 (떨림 측정 기록용)."""
        t_end = time.time() + sec
        while time.time() < t_end:
            rclpy.spin_once(self.n, timeout_sec=0.005)

    def hold_puck(self):
        self.move(self.cmd, OPEN, label="집게 열기")
        input("    손가락 사이에 퍽을 대고 Enter (닫음): ")
        self.move(self.cmd, CLOSE, confirm=False, label="집게 닫기")


class DirectArm(Arm):
    """브링업 없이 팔 OpenCR 에 바로 (팀 arm_bench ArmController, 바퀴 보드가 FAULT 일 때). 스킬 (메뉴 5) 은 못 씀."""

    def _on_state(self, st):                             # 100 Hz STATE 마다 (제어 스레드)
        if self.rec is not None:
            c = self.c.cmd
            self.rec.append((time.time(), list(st["pos"][6:12]), list(st["effort"][6:12]),
                             list(c) if c else [float("nan")] * 6))

    def __init__(self, port):
        sys.path.insert(0, os.path.join(BENCH, "slider"))
        from arm_control import ArmController
        self.c = ArmController(port, speed=0.5, max_track=0.25, on_state=self._on_state)
        P(f"팔 보드 {port} 직접 연결 (브링업 없음)")
        time.sleep(1.0)
        st = self.c.snapshot()["last"]
        if st is None or st.get("role") != 2:
            other = "/dev/ttyACM0" if port.endswith("1") else "/dev/ttyACM1"
            P(f"!! {port} 는 팔 보드가 아님 (role={None if st is None else st.get('role')}, 1 = 바퀴). "
              f"전원을 다시 켜면 포트 번호가 바뀜 -> --direct {other} 로 다시 실행")
            sys.exit(1)
        q, g = self.read()
        self.cmd, self.grip = q, min(max(g, OPEN), CLOSE)

    def read(self, avg=0.5):
        acc, t0 = [], time.time()
        while time.time() - t0 < avg or not acc:
            s = self.c.snapshot()["last"]
            if s:
                acc.append(list(s["pos"][6:12]))
            if time.time() - t0 > 5 and not acc:
                P("팔 보드 STATE 가 안 옴 (포트 · 전원)")
                sys.exit(1)
            time.sleep(0.02)
        v = [sum(a[i] for a in acc) / len(acc) for i in range(6)]
        return v[:5], v[5]

    def _run(self):
        """토크 ON · REARM -> RUN. 이미 RUN 이면 그대로."""
        if self.c.snapshot()["state"] == "RUN":
            return True
        self.c.request("start")
        t0 = time.time()
        while time.time() - t0 < 4:
            sn = self.c.snapshot()
            if sn["state"] == "RUN":
                return True
            if sn["state"] == "STOPPED" or sn["msg"].startswith("시작 안 함"):
                break
            time.sleep(0.05)
        P(f"  !! 팔 시작 실패: {self.c.snapshot()['msg']}")
        return False

    def _send(self, q, grip, T):
        if not self._run():
            raise Back                                # 시작 못 하면 이어지는 동작도 멈춤
        # 직선 보간은 '마지막 명령' 에서 시작 (처진 실측 위치에서 시작하면 명령이 처진 쪽으로 튀었다 돌아와 덜컹, 11-12 떨림 측정)
        c0 = self.c.snapshot()["cmd"]
        a, b = (list(c0) if c0 else self.cmd + [self.grip]), list(q) + [grip]
        t0 = time.time()
        try:
            while True:                                   # 관절 공간 직선 보간 (궤적 컨트롤러와 같게)
                r = min(1.0, (time.time() - t0) / T)
                for i in range(6):
                    self.c.set_target(i, a[i] + (b[i] - a[i]) * r)
                sn = self.c.snapshot()
                if sn["state"] != "RUN":
                    P(f"  !! 정지: {sn['msg']}")
                    raise Back
                if r >= 1.0:
                    now, _ = self.read(0.1)
                    if max(abs(x - y) for x, y in zip(now, q)) < 0.02 or time.time() - t0 > T + 3:
                        break
                time.sleep(0.02)
        except KeyboardInterrupt:
            self.c.request("stop")
            time.sleep(0.5)
            P("  !! 취소: 그 자리 정지 (HOLD)")
            raise Back
        self.cmd, self.grip = list(q), grip
        return True

    def wait(self, sec):
        time.sleep(sec)                                  # 직접 모드는 제어 스레드가 100 Hz 로 기록

    def close(self):
        self.c.close()


def fold(a, to_drive):
    """STOW <-> DRIVE, 팀 시작 정리와 같은 순서 (J3 -> J2 -> J1 -> J4 -> J5 · 집게), 0.1 rad/s."""
    tgt = DRIVE if to_drive else STOW
    order = [[2], [1], [0], [3], [4, 5]]
    if not to_drive:
        order = order[::-1]
    for grp in order:
        q, g = list(a.cmd), a.grip
        for j in grp:
            if j == 5:
                g = tgt[5]
            else:
                q[j] = tgt[j]
        dq = max([abs(q[j] - a.cmd[j]) for j in range(5)] + [abs(g - a.grip) / 4])
        if dq < 0.02:
            continue
        a.move(q, g, T=max(3.0, dq / 0.1), route=False, label=f"{'펴기' if to_drive else '접기'} 관절 {'·'.join(str(j + 1) for j in grp)}")


def m1_zero(a):
    P("\n[1] 부호 확인 + 영점")
    a.move(POSES["0"][0], label="0 자세")
    a.move(POSES["A"][0], label="A 자세 (앞으로 내려가야 정상)")
    if ask("팔이 앞으로 내려갔나? (y/n)") != "y":
        P("  !! 부호 이상: 여기서 멈추고 강화학습 세션에 알림")
        raise Back
    a.move(POSES["0"][0], label="0 자세 (영점)")
    q, _ = a.read()
    P("  수평계: 윗팔 (설계 90°), 아랫팔 · 집게 (설계 0°), 관절 1 = 집게가 차체 중앙선에서 좌우 mm (왼 +)")
    up = ask("윗팔 각도 (°, Enter = 안 잼)", float, empty_ok=True)
    fore = ask("아랫팔 각도 (°, Enter = 안 잼)", float, empty_ok=True)
    hand = ask("집게 각도 (°, Enter = 안 잼)", float, empty_ok=True)
    lat = ask("관절 1 좌우 어긋남 (mm, 왼 +, Enter = 안 잼)", float, empty_ok=True)
    record("1_영점.csv", ["읽은각1", "읽은각2", "읽은각3", "읽은각4", "읽은각5", "윗팔_deg", "아랫팔_deg", "집게_deg", "좌우_mm"],
           [f"{math.degrees(x):.2f}" for x in q] + [up, fore, hand, lat])


def m2_sag(a):
    P("\n[2] 처짐: 자세 0 · A · B · C × 3 회. 잡는 점 (패드 사이 가운데) 의 바닥 위 높이, 관절 1 축에서 수평 거리 (C 는 뒤쪽)")
    reps = ask("회차 수 (Enter = 1)", int, empty_ok=True) or 1
    names = [x for x in (ask("잴 자세 (예 'C', Enter = 0 A B C)", empty_ok=True) or "0 A B C").upper().split() if x in POSES]
    for puck in ("빈손", "쥠"):
        if ask(f"{puck} 상태로 잴까? (y/n)") != "y":
            continue
        if puck == "쥠":
            a.hold_puck()
        else:
            a.move(a.cmd, OPEN, label="집게 열기")
        for name in names:
            q, h0, r0 = POSES[name]
            for rep in range(1, reps + 1):
                a.move([q[0]] + HIGH, label=f"{name} 전 높은 자세")
                a.move(q, label=f"{name} 자세 {rep}/{reps} (설계 높이 {h0} · 수평 {r0} mm)")
                rd, g = a.read()
                h = ask("높이 (mm, Enter = 건너뜀)", float, empty_ok=True)
                if h is None:
                    continue
                r = ask("수평 거리 (mm, Enter = 안 잼)", float, empty_ok=True)
                record("1_처짐.csv", ["자세", "회차", "퍽", "명령1", "명령2", "명령3", "명령4", "명령5",
                                    "읽은1", "읽은2", "읽은3", "읽은4", "읽은5", "집게", "높이_mm", "수평_mm"],
                       [name, rep, puck] + [f"{math.degrees(x):.2f}" for x in q] + [f"{math.degrees(x):.2f}" for x in rd]
                       + [f"{g:.3f}", h, r])
        a.move([a.cmd[0]] + HIGH, label="높은 자세")
        if abs(a.cmd[0]) > J1_TURN:
            a.move([0] + HIGH, label="정면으로")


def m3_grip(a):
    P("\n[3] 집게 폭: 패드 안쪽 사이 거리 (캘리퍼스). 팔은 A 자세")
    a.move(POSES["A"][0], label="A 자세")
    for state in ("열림", "닫힘", "원기둥", "직육면체"):
        if state == "열림":
            a.move(a.cmd, OPEN, label="열기")
        elif state == "닫힘":
            a.move(a.cmd, CLOSE, label="빈손 닫기")
        else:
            a.move(a.cmd, OPEN, label="열기")
            input(f"    {state} 퍽을 손가락 사이에 대고 Enter (닫음): ")
            a.move(a.cmd, CLOSE, confirm=False, label="닫기")
        _, g = a.read()
        w = ask(f"{state} 폭 (mm, Enter = 건너뜀)", float, empty_ok=True)
        if w is not None:
            record("2_집게폭.csv", ["상태", "집게값_rad", "폭_mm"], [state, f"{g:.3f}", w])
    a.move(a.cmd, OPEN, label="열기")


def fk(q):
    """팀 URDF FK (td_ik.fk 와 같음): 관절 1 ~ 5 (rad) -> 잡는 점 base_link (m), 회전."""
    import numpy as np

    def Ry(t):
        c, s_ = math.cos(t), math.sin(t)
        return np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]])

    def Rz(t):
        c, s_ = math.cos(t), math.sin(t)
        return np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])

    def Rx(t):
        c, s_ = math.cos(t), math.sin(t)
        return np.array([[1, 0, 0], [0, c, -s_], [0, s_, c]])
    p, R = np.array([0.096, 0, 0.131]), Rz(q[0])
    p = p + R @ [0, 0, 0.0595]; R = R @ Ry(q[1])
    p = p + R @ [0.024, 0, 0.128]; R = R @ Ry(q[2])
    p = p + R @ [0.124, 0, 0]; R = R @ Ry(q[3])
    p = p + R @ [0.028, 0, 0]; R = R @ Rx(q[4])
    return p + R @ [0.14, 0, 0], R


def cart_step(q, d):
    """잡는 점을 base_link 방향 d (m) 만큼, 집게 숙임 각은 그대로. 관절 1 ~ 4 수치 IK."""
    import numpy as np
    q = np.array(q, float)
    p0, R0 = fk(q)
    pt, pitch0 = p0 + np.array(d), -R0[2, 0]

    def res(x):
        qq = q.copy(); qq[:4] = x
        p, R = fk(qq)
        return np.r_[(p - pt) * 1000, (-R[2, 0] - pitch0) * 300]
    x = q[:4].copy()
    for _ in range(8):
        r = res(x)
        J = np.array([(res(x + e) - r) / 1e-5 for e in np.eye(4) * 1e-5]).T
        x = x - np.linalg.lstsq(J, r, rcond=None)[0]
    out = q.copy(); out[:4] = x
    return list(out), float(np.abs(res(x)[:3]).max())


DIRS = {"위": (0, 0, 1), "아래": (0, 0, -1), "앞": (1, 0, 0), "뒤": (-1, 0, 0), "왼": (0, 1, 0), "오른": (0, -1, 0)}


def jog(a, home, home_label):
    """퍽에 집게를 손으로 맞춤. ok = 읽은 관절 (rad 5 개) 돌려줌, b = None."""
    P("  움직이기: '아래 5' = 5 mm 아래. 위 · 아래 · 멀리 · 가까이 (로봇 몸에서 멀어짐 / 가까워짐) · 왼 · 오른 (로봇 기준)")
    P("            앞 · 뒤 = 로봇 앞 / 뒤 방향. 한 번에 20 mm 까지. 처음 = 시작 자리로 되돌아감")
    P("            집게 42 = 손가락 사이를 42 mm 로. ok = 맞춤 끝 · 기록, b = 기록 없이 빠져나옴")
    while True:
        s = input("    조정: ").strip()
        LOG.write(f"    조정: {s}\n")
        if s == "ok":
            q, _ = a.read()
            return q
        if s in ("b", "q"):
            return None
        if s.startswith("집게"):
            try:
                w = float(s.split()[1])
                assert 24 <= w <= 66
            except (ValueError, AssertionError, IndexError):
                print("    예: '집게 42' (손가락 사이 mm, 24 ~ 66)")
                continue
            g = -0.764 + (66.68 - w) / 25.4          # 실측 2_집게폭: 폭 = 66.68 − 25.4 × (집게 + 0.764)
            a.move(a.cmd, min(max(g, OPEN), CLOSE), confirm=False, T=1.5, label=f"집게 {w:g} mm")
            continue
        if s == "처음":
            a.move(home, confirm=False, label=f"{home_label} (처음)")
            continue
        parts = s.split()
        if parts and parts[0] in ("멀리", "가까이"):
            p, _ = fk(a.cmd)
            ux, uy = p[0] - 0.096, p[1]
            n = math.hypot(ux, uy) or 1.0
            sgn = 1 if parts[0] == "멀리" else -1
            DIRS[parts[0]] = (sgn * ux / n, sgn * uy / n, 0)
        try:
            if parts and parts[0] in DIRS:
                mm = float(parts[1]) if len(parts) > 1 else 2.0
                assert 0 < mm <= 20
                q, err = cart_step(a.cmd, [v * mm / 1000 for v in DIRS[parts[0]]])
                if err > 1.0 or max(abs(x - y) for x, y in zip(q, a.cmd)) > math.radians(15):
                    print("    그 방향으로는 못 감 (관절 한계 근처)")
                    continue
                label = f"{parts[0]} {mm:g} mm"
            else:
                i, d = int(parts[0]), float(parts[1])
                assert 1 <= i <= 5 and abs(d) <= 3
                q = list(a.cmd); q[i - 1] += math.radians(d)
                label = f"관절 {i} {d:+.1f}°"
        except (ValueError, AssertionError, IndexError):
            print("    예: '아래 5', '왼 2', '앞 1', ok, b")
            continue
        a.move(q, confirm=False, T=1.5, label=label)


CALIB_HDR = ["이름", "j1", "j2", "j3", "j4", "j5", "x", "y", "z"]


def calib_row(name, q, p_j1_mm):
    """6_보정자세.csv 한 줄: 읽은 관절 (도) + 실제 잡는 점 (관절 1 축 기준 x · y, 바닥 위 z, mm)."""
    record("6_보정자세.csv", CALIB_HDR, [name] + [f"{math.degrees(v):.2f}" for v in q] + [f"{v:.1f}" for v in p_j1_mm])


def m4_slot(a):
    P("\n[4] 칸 좌표: 칸에 퍽을 넣어 두고, 퍽이 두 손가락 정가운데 · 손가락 가운데가 퍽 윗면보다 15 mm 아래에 오게 맞춤")
    slot = ask(f"칸 ({' '.join(sorted(SLOTS))})")
    if slot not in SLOTS:
        P("  없는 칸")
        return
    j1, above, grasp = SLOTS[slot]
    a.move(a.cmd, OPEN, label="집게 열기")
    a.move([j1] + above, label=f"{slot} 칸 위 6 cm")
    q = jog(a, [j1] + above, f"{slot} 칸 위 6 cm")
    if q is not None:
        p, _ = fk(q)
        note = input("    비고 (Enter = 없음): ").strip()
        record("3_칸좌표.csv", ["칸", "방법", "읽은1", "읽은2", "읽은3", "읽은4", "읽은5", "설계x_mm", "설계y_mm", "설계z_mm", "비고"],
               [slot, "A"] + [f"{math.degrees(x):.2f}" for x in q] + [f"{v * 1000:.1f}" for v in p] + [note])
        g, _ = fk([j1] + grasp + [0.0])            # 칸 설계 잡는 점 (unload_traj 잡는 자리 = 퍽 윗면 15 mm 아래)
        calib_row(f"칸{slot}", q, [(g[0] - 0.096) * 1000, g[1] * 1000, (g[2] + 0.00675) * 1000])
    a.move([j1] + above, label="칸 위로")
    a.move([j1] + HIGH, label="높은 자세")
    a.move([0] + HIGH, label="정면으로")


BOX_SPOTS = [("가운데", 0.0), ("왼쪽 6 cm", 0.06), ("오른쪽 6 cm", -0.06)]


def m12_box(a):
    P("\n[12] 상자 위 퍽 맞추기 (영점 보정용): 상자 (205 × 205 × 70) 를 로봇 정면에 똑바로, 가운데선이 로봇 가운데선과 같게")
    gap = ask("로봇 앞 끝 (맨 앞 판) 에서 상자 앞면까지 (mm)", float)
    front = ask("관절 1 모터 중심에서 로봇 앞 끝까지 수평 거리 (mm, 모르면 Enter = 34 설계)", float, empty_ok=True) or 34.0
    cx = front + gap + 102.5                         # 관절 1 축에서 상자 가운데까지 (mm)
    z_grasp = 70 + 60 - 15                           # 바닥 위: 상자 70 + 퍽 60, 퍽 윗면 15 mm 아래
    for name, y in BOX_SPOTS:
        if ask(f"퍽을 상자 {name} 에 세웠나? (y = 시작 / s = 건너뜀)") != "y":
            continue
        # 실물 손은 모델보다 앞쪽에서 약 7 cm 낮음 (11-12 상자 가운데 실측) -> 모델로 퍽 윗면 위 15 cm 에 띄움 (실물 약 8 cm 위)
        tgt = [(cx / 1000) + 0.096, y, (z_grasp + 165) / 1000 - 0.00675]
        ready = [0, -0.338, 0.559, 0.564, 0]          # 팀 READY_POSE_45 (집게 45° 아래): 계산 시작점으로만 씀 (실물로 가면 상자 위 퍽에 닿음)
        p0, _ = fk(ready)
        q, err = cart_step(ready, [t - c for t, c in zip(tgt, p0)])
        a.move([0] + HIGH, label="높은 자세")
        if err > 2.0:
            P("  !! 그 자리로 가는 자세를 못 찾음 (너무 멀거나 가까움)")
            continue
        a.move(a.cmd, OPEN, label="집게 열기")
        a.move(q, label=f"상자 {name} 퍽 위")
        qr = jog(a, q, f"상자 {name} 퍽 위")
        if qr is not None:
            calib_row(f"상자{name.split()[0]}", qr, [cx, y * 1000, z_grasp])
        a.move(q, label="퍽 위로 (띄움)")
    a.move([0] + HIGH, label="높은 자세")


def skill(name):
    cmd = ["ros2", "action", "send_goal", "/manipulator/execute_skill", "robot_interfaces/action/ExecuteSkill",
           f"{{policy: {name}, timeout: 40.0}}"]
    P(f"  스킬 {name} 실행 ...")
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=90).stdout
    tail = " ".join(l.strip() for l in out.splitlines() if "success" in l or "message" in l or "rejected" in l.lower())
    P(f"  스킬 결과: {tail or out[-300:]}")
    return tail


def m5_insert(a):
    P("\n[5] 넣기 시험: q_front 에서 퍽 쥠 -> pallet_load -> 결과 기록 -> pallet_unload")
    slot = ask(f"칸 ({' '.join(sorted(SLOTS))})")
    if slot not in SLOTS:
        P("  없는 칸")
        return
    n = ask("횟수 (Enter = 5)", int, empty_ok=True) or 5
    r, c = slot.split("-")
    P("  결과: 1 성공 / 2 끼임 / 3 기울어짐 / 4 튕겨 나감 / 5 옆 칸 건드림 / 6 스킬 거부 · 기타")
    labels = {"1": "성공", "2": "끼임", "3": "기울어짐", "4": "튕겨 나감", "5": "옆 칸 건드림", "6": "기타"}
    for k in range(1, n + 1):
        P(f"\n  -- {slot} {k}/{n}")
        a.move(Q_FRONT, label="q_front")
        a.hold_puck()
        msg = skill(f"pallet_load_{r}_{c}_tilt")
        res = labels.get(ask("결과 번호"), "기타")
        tilt = ask("기울기 (°, Enter = 0)", float, empty_ok=True) or 0
        dirn = "" if res == "성공" else (input("    어긋난 방향 (앞/뒤/왼/오른, Enter = 없음): ").strip())
        amt = "" if res == "성공" else (ask("어긋난 양 (mm, Enter = 없음)", float, empty_ok=True) or "")
        note = input("    비고: ").strip()
        record("4_넣기시험.csv", ["칸", "회차", "결과", "기울기_deg", "어긋난방향", "어긋난양_mm", "비고", "스킬메시지"],
               [slot, k, res, tilt, dirn, amt, note, msg.replace(",", ";")])
        if ask("꺼내기 스킬 돌릴까? (y = 스킬 / n = 손으로 꺼냄)") == "y":
            skill(f"pallet_unload_{r}_{c}_tilt")
        else:
            input("    손으로 퍽을 꺼내고 Enter: ")


def m6_mass(a):
    P("\n[6] 무게 · 마찰 (로봇 안 움직임). 마찰 = 판을 기울여 퍽이 미끄러지기 시작하는 각 5 회")
    for item in ("원기둥1_g", "원기둥2_g", "원기둥3_g", "직육면체1_g", "직육면체2_g", "직육면체3_g"):
        v = ask(f"{item} (Enter = 건너뜀)", float, empty_ok=True)
        if v is not None:
            record("5_무게_마찰.csv", ["항목", "값1", "값2", "값3", "값4", "값5", "평균", "마찰계수"], [item, v, "", "", "", "", v, ""])
    for item in ("퍽-상자", "퍽-패드", "퍽-칸바닥"):
        vals = []
        for i in range(5):
            v = ask(f"{item} 각도 {i + 1}/5 (°, Enter = 끝)", float, empty_ok=True)
            if v is None:
                break
            vals.append(v)
        if vals:
            m = sum(vals) / len(vals)
            record("5_무게_마찰.csv", ["항목", "값1", "값2", "값3", "값4", "값5", "평균", "마찰계수"],
                   [item] + vals + [""] * (5 - len(vals)) + [f"{m:.2f}", f"{math.tan(math.radians(m)):.3f}"])


CALIB = [   # 6_보정자세_측정안내.md (강화학습 세션 11-12), 관절 1 ~ 4 (도), 관절 5 = 0
    ("P1", "앞으로 낮게 뻗음", [0, -15, 30, 30]), ("P2", "앞으로 높게", [0, 0, 0, 0]),
    ("P3", "앞, 팔꿈치 많이 굽힘", [0, -40, 60, 20]), ("P4", "앞, 손목만 아래", [0, -20, 20, 60]),
    ("P5", "왼쪽 45°", [45, -15, 30, 30]), ("P6", "오른쪽 45°", [-45, -15, 30, 30]),
    ("P7", "왼쪽 90°, 높게", [90, -10, 10, 40]), ("P8", "뒤 팔레트 위", [172, -9, -20, 92]),
    ("P9", "상자 위 집기 근처", [0, -30, 50, 50]), ("P10", "몸 쪽으로 당김", [0, -50, 90, 0]),
]


def m10_calib(a):
    P("\n[10] 영점 보정 자세: 자세마다 손끝 (두 손가락 사이 가운데) 의 x (관절 1 축 점에서 앞 +) · y (왼쪽 +) · z (바닥에서 높이), mm")
    P("     x · y 는 손끝에서 실 + 추를 내려 종이에 찍은 점을 관절 1 축 점에서 잼. 못 재면 Enter")
    start = ask("시작 자세 (예 P3, Enter = P1)", empty_ok=True) or "P1"
    names = [c[0] for c in CALIB]
    if start not in names:
        P("  없는 자세")
        return
    a.move(a.cmd, OPEN, label="집게 열기")
    for name, desc, q4 in CALIB[names.index(start):]:
        q = [math.radians(v) for v in q4] + [0.0]
        if abs(q[0] - a.cmd[0]) > J1_TURN:
            a.move([a.cmd[0]] + HIGH, label="높은 자세 (회전 전)")
        rep = 1
        while True:
            if a.move(q, label=f"{name} {desc} ({rep} 회)") is False:
                break
            rd, _ = a.read()
            x = ask("x 앞 (mm, Enter = 안 잼)", float, empty_ok=True)
            y = ask("y 왼 (mm, Enter = 안 잼)", float, empty_ok=True)
            z = ask("z 높이 (mm, Enter = 안 잼)", float, empty_ok=True)
            record("6_보정자세.csv", ["이름", "j1", "j2", "j3", "j4", "j5", "x", "y", "z"],
                   [name] + [f"{math.degrees(v):.2f}" for v in rd] + ["" if v is None else v for v in (x, y, z)])
            if ask("같은 자세 한 번 더? (y/n)") != "y":
                break
            a.move([q[0]] + HIGH, label="높은 자세 (다시 재기 전)")
            rep += 1
    a.move([a.cmd[0]] + HIGH, label="높은 자세")
    if abs(a.cmd[0]) > J1_TURN:
        a.move([0] + HIGH, label="정면으로")
    P("  끝. ~/실측/6_보정자세.csv 를 PC 기록/실측/ 로 가져오기")


GRIP_PTS = [-0.764, -0.5, -0.3, -0.1, 0.0, 0.08, 0.2, 0.3, 0.4, 0.6, 0.922]   # 6_보정자세_측정안내.md ②


def m11_grip_pts(a):
    P("\n[11] 집게 빈손 폭 11 점: 집게 값마다 패드 안쪽 폭 (캘리퍼스, mm). 손가락 사이에 아무것도 넣지 말 것")
    for g in GRIP_PTS:
        a.move(a.cmd, g, confirm=False, T=2.0, label=f"집게 {g:+.3f}")
        time.sleep(0.5)
        _, rg = a.read()
        w = ask(f"폭 (mm, 명령 {g:+.3f} · 읽은 {rg:+.3f}, Enter = 안 잼)", float, empty_ok=True)
        record("7_집게폭점.csv", ["명령", "읽은값", "폭mm"], [f"{g:.3f}", f"{rg:.3f}", "" if w is None else w])
    a.move(a.cmd, OPEN, confirm=False, label="집게 열기")
    P("  끝. ~/실측/7_집게폭점.csv 를 PC 기록/실측/ 로 가져오기")


SHAKE_POSES = {"1": ("지금 자세", None), "2": ("앞 45° 집기 자세 (READY_POSE_45)", [0, -0.338, 0.559, 0.564, 0]),
               "3": ("앞으로 쭉 뻗음 (0 자세)", [0, 0, 0, 0, 0]), "4": ("DRIVE", DRIVE[:5])}


def shake_report(rows, steps, out):
    """단계마다 넘침 · 자리 잡는 시간 · 남은 흔들림 · 주파수 · 전류."""
    import numpy as np
    t = np.array([r[0] for r in rows]); pos = np.degrees(np.array([r[1][:5] for r in rows]))
    cur = np.array([r[2][:5] for r in rows], float)
    lines = []
    for name, j, t0, t1, target in steps:              # 명령이 끝난 순간 t0 부터 t1 까지 머묾
        m = (t >= t0) & (t <= t1)
        if m.sum() < 20:
            continue
        tt = t[m] - t0
        fin = float(np.mean(pos[m, j][-50:]))           # 마지막 0.5 s 평균 = 멈춘 자리
        err = fin - np.degrees(target)                  # 남는 오차 (처짐 · 유격)
        x = pos[m, j] - fin
        over = float(np.max(np.abs(x)))                 # 멈춘 자리 기준 넘침 · 출렁임
        inb = np.abs(x) <= 0.1
        settle = next((float(tt[i]) for i in range(len(x)) if inb[i:].all()), float("nan"))
        late = tt >= 0.5
        resid = float(np.std(x[late])) if late.sum() > 10 else float("nan")
        freq = float("nan")
        if late.sum() > 32:
            y = x[late] - x[late].mean()
            f = np.abs(np.fft.rfft(y)); fr = np.fft.rfftfreq(len(y), float(np.median(np.diff(tt))))
            freq = float(fr[1:][np.argmax(f[1:])])
        c = cur[m, j]
        lines.append(f"{name:<12} 남는 오차 {err:+5.2f}° | 출렁임 {over:5.2f}° | 0.1° 안 정착 {settle:4.2f} s | 0.5 s 뒤 흔들림 {resid:5.3f}° "
                     f"(주파수 {freq:4.1f} Hz) | 전류 최대 {np.nanmax(np.abs(c)):5.2f} · 평균 {np.nanmean(np.abs(c)):5.2f}")
    open(os.path.join(out, "summary.txt"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    for l in lines:
        P("  " + l)


def m13_shake(a):
    P("\n[13] 떨림 측정: 관절을 하나씩 조금 (±step°) 빠르게 움직였다 멈추고 머무는 동안 각 · 전류를 100 Hz 로 기록")
    P("     휴대폰 슬로모션으로 팔 끝을 같이 찍으면 좋음 (모터 센서는 링크 휨 · 기어 유격은 못 봄)")
    k = ask("자세 (1 지금 / 2 앞 45° 집기 / 3 앞으로 뻗음 / 4 DRIVE, Enter = 1)", empty_ok=True) or "1"
    if k not in SHAKE_POSES:
        return
    label, q0 = SHAKE_POSES[k]
    if q0 is not None:
        a.move(list(q0), label=label)
    q0 = list(a.cmd)
    step = math.radians(ask("움직일 크기 (도, Enter = 5)", float, empty_ok=True) or 5.0)
    T = ask("움직이는 시간 (s, Enter = 0.4 = 빠르게)", float, empty_ok=True) or 0.4
    hold = 2.0
    out = os.path.join(OUT, f"shake_{datetime.datetime.now():%m%d_%H%M%S}")
    os.makedirs(out, exist_ok=True)
    P(f"  자세 {label}: {deg(q0)} | 관절 1 ~ 4 를 +{math.degrees(step):.0f}° -> 원래 -> −{math.degrees(step):.0f}° -> 원래, 매번 {hold:.0f} s 머묾")
    if input("    Enter = 시작 (팔 주변 비우기), q = 취소: ").strip() == "q":
        return
    steps = []
    a.rec = []
    try:
        a.wait(hold)
        steps.append(("가만히", 1, time.time() - hold, time.time(), q0[1]))
        for j in range(4):
            for sgn in (1, 0, -1, 0):
                q = list(q0); q[j] += sgn * step
                a.move(q, confirm=False, T=T, label=f"관절 {j + 1} {'+' if sgn > 0 else '−' if sgn < 0 else '원래'}")
                t0 = time.time(); a.wait(hold)
                steps.append((f"관절{j + 1} {'+' if sgn > 0 else '-' if sgn < 0 else '0'}", j, t0, time.time(), q[j]))
    finally:
        rows, a.rec = a.rec, None
    with open(os.path.join(out, "data.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t"] + [f"각{i}_deg" for i in range(1, 7)] + [f"전류{i}" for i in range(1, 7)] + [f"명령{i}_deg" for i in range(1, 7)])
        for r in rows:
            w.writerow([f"{r[0] - rows[0][0]:.3f}"] + [f"{math.degrees(v):.3f}" for v in r[1]] + [f"{v:.3f}" for v in r[2]]
                       + [f"{math.degrees(v):.3f}" for v in r[3]])
    P(f"  기록 {len(rows)} 줄 ({len(rows) / max(rows[-1][0] - rows[0][0], 1e-3):.0f} Hz) -> {out}")
    shake_report(rows, steps, out)
    P("  가져오기 (PC): scp -r cart11@cart11.local:" + out.replace(os.path.expanduser("~"), "~") + " <PC>/기록/실측/")


def m8_manual(a):
    s = ask("관절 1 ~ 5 (도, 공백) [집게 rad]")
    v = [float(x) for x in s.split()]
    a.move([math.radians(x) for x in v[:5]], v[5] if len(v) > 5 else None, label="수동")


def m9_fold(a):
    if ask("1 = STOW -> DRIVE 펴기 / 2 = DRIVE -> STOW 접기 (전원 끄기 전)") == "1":
        fold(a, True)
    else:
        if abs(a.cmd[0]) > J1_TURN:
            a.move([a.cmd[0]] + HIGH, label="높은 자세")
            a.move([0] + HIGH, label="정면으로")
        a.move(DRIVE[:5], DRIVE[5], label="DRIVE")
        fold(a, False)


def main():
    direct = sys.argv[2] if len(sys.argv) > 2 and sys.argv[1] == "--direct" else None
    P(f"\n===== 실측 세션 {datetime.datetime.now():%Y-%m-%d %H:%M} ({'직접 ' + direct if direct else '브링업'}) =====")
    a = DirectArm(direct) if direct else Arm()
    q, g = a.show()
    if any(math.isnan(x) for x in q + [g]):
        P("  !! 관절값 nan: 보드에서 관절 데이터가 안 옴. 모터 전원 · 리셋 버튼 (REARM) 확인 뒤 다시 실행")
        return
    if q[1] < -1.9:
        if not direct:
            P("  !! STOW 자세로 보임. bringup 시작 정리 (DRIVE) 가 끝난 뒤 다시 실행")
            return
        P("  STOW 자세: 팀 순서 (관절 3 -> 2 -> 1 -> 4 -> 5) 로 DRIVE 까지 폄 (0.1 rad/s)")
        try:
            fold(a, True)
        except Back:
            P("  펴기 중단. 메뉴 9 로 다시")
    menu = {"1": ("부호 확인 + 영점", m1_zero), "2": ("처짐", m2_sag), "3": ("집게 폭", m3_grip),
            "4": ("칸 좌표 (방법 A)", m4_slot), "5": ("넣기 시험", m5_insert), "6": ("무게 · 마찰", m6_mass),
            "7": ("지금 관절각", lambda a: a.show()), "8": ("자세 직접 보내기 (도)", m8_manual),
            "9": ("STOW <-> DRIVE", m9_fold), "10": ("영점 보정 자세 (6_보정자세)", m10_calib),
            "11": ("집게 빈손 폭 11 점 (7_집게폭점)", m11_grip_pts), "12": ("상자 위 퍽 맞추기 (영점 보정)", m12_box),
            "13": ("떨림 측정 (각 · 전류 100 Hz)", m13_shake)}
    if direct:
        menu["5"] = ("넣기 시험 (직접 모드는 못 씀)", lambda a: P("  스킬은 브링업 (manipulator_node) 필요"))
    while True:
        P("\n메뉴: " + " / ".join(f"{k} {v[0]}" for k, v in menu.items()) + " / 0 끝")
        k = input("선택: ").strip()
        LOG.write(f"선택: {k}\n")
        if k == "0":
            break
        if k in menu:
            try:
                menu[k][1](a)
            except Back:
                P("  (메뉴로)")
            except KeyboardInterrupt:
                P("  (중단, 메뉴로)")
    a.close()
    P(f"끝. 결과 폴더: {OUT}" + ("  (전원 끄기 전에는 메뉴 9 -> 2 로 STOW)" if direct else ""))


if __name__ == "__main__":
    main()
