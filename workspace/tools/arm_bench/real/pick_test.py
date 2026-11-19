#!/usr/bin/env python3
"""집기 시험 한 번 + 기록 (2026-11-12, 젯슨, 브링업 · kbd_joy 가 떠 있을 때).
인식 켜기 -> 퍽 위치 읽기 -> target_look -> toppick_tilt. 그동안 팔 카메라 영상 · 관절 · 결과를 기록.
2026-11-14: 미션 차례 그대로 이어서 (Isaac chain_pallet_eval --scripted_pick 과 같은 흐름):
  집을 퍽 = J1 축에 가장 가까운 퍽 (mission 은 가까운 퍽부터, 나머지 = 옆 퍽) -> --load 면 pallet_load_<열>_<칸>_tilt ->
  --recheck 면 처음 target_look 자리로 다시 가서 옆 퍽을 다시 잼 (옆 퍽 밀림 = 쥔 뒤 들기 · 운반 중 침).

    python3 ~/pick_test.py [메모]                          # 집기만 (예전과 같음)
    python3 ~/pick_test.py --load 1 1 --recheck [메모]      # 집기 -> 1 열 1 칸 넣기 -> 옆 퍽 다시 잼
    python3 ~/pick_test.py --load auto --occupied "3_0" [메모]  # 칸 자동 (아래), load.sh 가 여러 번 이어 부름
2026-11-15 (load.sh 용):
  --load auto   인식한 퍽 모양 (Puck.shape) 에 맞는 빈 칸을 계획기 규칙으로 고름: 열 규칙 (앞 열에 퍽이 있으면 뒤 열에 못 실음,
                mission_planner slots.py can_load) 이 허락하는 칸 중 모양 같은 칸 -> 이미 실린 퍽과 막는 쌍 (BLOCKERS) 안 생기는 칸
                -> 뒤 열부터 (3 -> 2 -> 1) -> 열 안 ROW_ORDER. 모양 같은 칸이 없으면 다른 모양 칸 (경고).
  --occupied "r_i ..."  이미 퍽이 있는 칸 · --removed "r_i ..."  쓰지 않는 칸 (기본 PALLET_REMOVED)
  칸 위치 = 노드 궤적 (control/config/pallet_place_traj.json, #293 실측 칸 맞춤). 설치본이 실측 (pallet_slot_teach.json) 과 다르면 경고.
  노드가 거부하면 먼저 pose DRIVE (팔이 STOW 면 노드가 폄, 2026-11-18) 뒤, 그래도 거부면 10 s 뒤 다시 (dice.sh 와 같음). 마지막 줄 'RESULT ok|pick_fail|load_fail|no_puck|no_slot <칸>' (load.sh 가 읽음), 종료 코드 0 = 성공.

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


def unfold():
    """팔이 STOW 면 노드가 pick · skill 을 거절 (2026-11-18) -> pose DRIVE (노드가 단계 순서로 폄). 이미 DRIVE 면 바로 끝."""
    out = subprocess.run(["ros2", "action", "send_goal", "/manipulator/pose", "robot_interfaces/action/ArmPose", "{name: DRIVE}"],
                         capture_output=True, text=True).stdout
    P("  pose DRIVE: " + ("ok" if "success: true" in out else "실패 (" + out[-200:].replace("\n", " ") + ")"))


def run(args):
    unfolded = False
    for k in range(9):                       # 노드가 켜질 때 정리 중이면 거부 -> 10 s 뒤 다시 (최대 90 s)
        P("$ ros2 " + " ".join(args))
        out = subprocess.run(["ros2"] + args, capture_output=True, text=True).stdout
        if "Goal was rejected" not in out:
            break
        if not unfolded:                     # 첫 거부: STOW 일 수 있음 -> 먼저 펴 보고 바로 다시
            P("  거부됨 -> 팔이 STOW 일 수 있어 pose DRIVE 먼저")
            unfold()
            unfolded = True
            continue
        P(f"  거부됨 (켜질 때 정리 중이거나 다른 스킬 중) -> 10 s 뒤 다시 ({k + 1}/9)")
        time.sleep(10.0)
    keep = [l.strip() for l in out.splitlines() if l.strip().startswith(("success", "message", "response", "robot_interfaces.srv"))]
    P("  " + (" | ".join(keep) or out[-400:].replace("\n", " ")))
    return "success: true" in out or "success=True" in out


def parse():
    """--load <열> <칸> | --load auto · --occupied · --removed · --recheck · 나머지 = 메모."""
    global OCC, REMOVED
    av, load, recheck, memo = sys.argv[1:], None, False, []
    i = 0
    while i < len(av):
        if av[i] == "--load" and i + 1 < len(av) and av[i + 1] == "auto":
            load = "auto"
            i += 2
        elif av[i] == "--load" and i + 2 < len(av):
            load = (int(av[i + 1]), int(av[i + 2]))
            i += 3
        elif av[i] in ("--occupied", "--removed") and i + 1 < len(av):
            v = {(int(a), int(b)) for a, _, b in (t.partition("_") for t in av[i + 1].split())}
            OCC, REMOVED = (v, REMOVED) if av[i] == "--occupied" else (OCC, v)
            i += 2
        elif av[i] == "--recheck":
            recheck = True
            i += 1
        else:
            memo.append(av[i])
            i += 1
    return load, recheck, " ".join(memo)


J1_X = 0.096  # base_link 에서 관절 1 축 x (manipulator.yaml ik · 팀 URDF)
OCC = set()
REMOVED = {(int(a), int(b)) for a, _, b in (t.partition("_") for t in os.environ.get("PALLET_REMOVED", "").split())}
WS = os.path.expanduser("~/Lv2_Cartlidar_Assignment/workspace/ros2_ws")


def slot_rules():
    """계획기 slots.py (설치본 -> 저장소 소스 -> 같은 규칙 사본 v5) -> (SLOT_SHAPE, ROW_ORDER, BLOCKERS, can_load, 출처)."""
    sys.path.insert(0, os.path.join(WS, "src/mission_planner"))
    try:
        from mission_planner.slots import BLOCKERS, ROW_ORDER, SLOT_SHAPE, _Rules
        return SLOT_SHAPE, ROW_ORDER, BLOCKERS, _Rules("free").can_load, "mission_planner.slots"
    except Exception as e:
        shp = {(1, 0): "cylinder", (1, 1): "box", (1, 2): "cylinder", (1, 3): "box", (2, 0): "box", (2, 1): "cylinder",
               (2, 2): "box", (2, 3): "cylinder", (3, 0): "cylinder", (3, 1): "box"}
        bl = {(1, 0): ((1, 1),), (1, 1): ((1, 0),), (1, 2): ((1, 3),)}
        return shp, [1, 2, 0, 3], bl, (lambda occ, s: not any(r < s[0] for r, _ in occ)), f"사본 ({e.__class__.__name__})"


def pick_slot(shape):
    """퍽 모양 (Puck.shape 1 원기둥 · 2 직육면체 · 0 모름) -> 실을 칸 (열, 칸) 과 설명, 없으면 (None, 이유)."""
    shp, row_order, bl, can_load, src = slot_rules()
    want = {1: "cylinder", 2: "box"}.get(shape)
    free = [s for s in shp if s not in OCC and s not in REMOVED and can_load(OCC, s)]
    if not free:
        return None, f"실을 수 있는 칸 없음 (차 있음 {sorted(OCC)}, 앞 열에 퍽이 있으면 뒤 열 못 실음)"
    pairs = lambda s: sum(b in OCC for b in bl.get(s, ())) + sum(s in v and k in OCC for k, v in bl.items())
    s = min(free, key=lambda s: (want is not None and shp[s] != want, pairs(s), -s[0], row_order.index(s[1])))
    note = f"{src}, 퍽 {want or '모양 모름'} -> 칸 {shp[s]}"
    if want is not None and shp[s] != want:
        note += " !! 모양 다른 칸 (같은 모양 빈 칸 없음 — 꺼낼 때 이웃 칸 모서리에 걸릴 수 있음)"
    return s, note


def check_traj():
    """젯슨 설치본 노드 궤적의 칸 놓는 끝 자세가 실측 칸 맞춤 (pallet_slot_teach.json) 과 같은지. 다르면 경고만."""
    import json
    try:
        pre = subprocess.run(["ros2", "pkg", "prefix", "control"], capture_output=True, text=True).stdout.strip()
        t = json.load(open(os.path.join(pre, "share/control/config/pallet_place_traj.json"), encoding="utf-8"))
        m = json.load(open(os.path.join(WS, "src/control/config/pallet_slot_teach.json"), encoding="utf-8"))
        m = m.get("slots", m)
    except Exception as e:
        P(f"!! 칸 위치 확인 못 함 ({e.__class__.__name__}: {e}) — 저장소 · 빌드 확인")
        return
    bad = []
    for s in t["slots"]:
        k = f"{s['row']}_{s['index']}"
        q = next(g for g in s["segments"] if g["name"] == "place")["q"][-1]
        if k in m and max(abs(a - b) for a, b in zip(q, m[k]["q_meas"])) > 1e-3:
            bad.append(k)
    missing = [k for k in m if k not in {f"{s['row']}_{s['index']}" for s in t["slots"]}]
    if bad or missing or not m:
        P(f"!! 설치된 노드 궤적이 실측 칸 맞춤과 다름 (칸 {bad + missing}) -> git pull origin dev 후 colcon build --packages-select control, 노드 다시 띄우기")
    else:
        P(f"칸 위치: 설치된 노드 궤적 = 실측 칸 맞춤 {len(m)} 칸 (#293)")


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
    P(f"집기 시험 {OUT}  넣기 {load}  다시 잼 {recheck}  차 있는 칸 {sorted(OCC)}  메모: {memo}")
    if load:
        check_traj()
    result = "no_puck"
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
            return 2
        all_pk = rec.pucks[-1].pucks
        pk = min(all_pk, key=lambda q: math.hypot(q.pose.position.x - J1_X, q.pose.position.y))  # 가까운 퍽부터 (mission 과 같음)
        p = pk.pose.position
        P(f"인식한 퍽 {len(all_pk)} 개, 집을 퍽 (J1 에 가장 가까움, base_link): x {p.x:.3f} y {p.y:.3f} z {p.z:.3f}  tag {pk.tag_id} shape {pk.shape}"
          f"  J1 축 {math.hypot(p.x - J1_X, p.y):.3f} m")
        if load == "auto":
            load, why = pick_slot(pk.shape)
            if load is None:
                result = f"no_slot -"
                P("!! " + why)
                return 1
            P(f"실을 칸 {load[0]}_{load[1]} ({why})")
        look = (p.x, p.y, p.z)
        before = look_and_read(rec, look)
        if before is None:
            P("!! target_look 실패 -> 집기 안 함")
            result = "pick_fail -"
            return 1
        nbs = [q for q in before if math.hypot(q[0] - p.x, q[1] - p.y) > 0.03]  # 옆 퍽 (집을 퍽에서 3 cm 밖)
        P("옆 퍽 (집기 전): " + (", ".join(f"({q[0]:.3f}, {q[1]:.3f})" for q in nbs) or "없음"))
        ok = run(["action", "send_goal", "/manipulator/execute_skill", "robot_interfaces/action/ExecuteSkill",
                  "{policy: toppick_tilt, timeout: 40.0}"])
        P(f"집기 {'성공' if ok else '실패'}")
        result = "ok -" if ok else "pick_fail -"
        if ok and load:
            ok = run(["action", "send_goal", "/manipulator/execute_skill", "robot_interfaces/action/ExecuteSkill",
                      f"{{policy: pallet_load_{load[0]}_{load[1]}_tilt, timeout: 90.0}}"])
            P(f"넣기 {load[0]}열 {load[1]} {'성공' if ok else '실패'}")
            result = f"{'ok' if ok else 'load_fail'} {load[0]}_{load[1]}"
        time.sleep(2.0)                          # 끝난 뒤 모습도 조금 더 기록
        if recheck and nbs:
            after = look_and_read(rec, look)
            for q in nbs:
                d = min((math.hypot(q[0] - w[0], q[1] - w[1]) for w in (after or [])), default=float("nan"))
                P(f"옆 퍽 ({q[0]:.3f}, {q[1]:.3f}) 다시 잼: 가장 가까운 퍽까지 {d * 1000:.0f} mm "
                  + ("(그대로)" if d < 0.008 else "!! 밀림 의심 (인식 오차 수 mm 감안, 영상 확인)" if d == d else "!! 안 보임 (넘어짐 · 밀림?)"))
        return 0 if ok else 1
    finally:
        ex.shutdown()
        rec.close()
        P(f"기록 끝: {OUT} (영상 {rec.n_img // 3} 장)")
        P(f"RESULT {result}")


if __name__ == "__main__":
    sys.exit(main())
