#!/usr/bin/env python3
"""놓기 실측 기록 (2026-11-11, 젯슨에서 실행). 칸마다 노드 각본을 부르고, 노드 로그 · 집게 값은 자동으로, 오차는 자로 재서 한 줄 입력.
결과 = 짧은 CSV 한 파일 (~/place_measure/<시각>_<L>.csv) -> PC 로 scp 하거나 마지막에 나오는 요약만 붙여 넣으면 됨.

  source /opt/ros/<배포판>/setup.bash; source ~/<ws>/install/setup.bash   (manipulator_node 가 떠 있어야 함)
  python3 place_measure.py --place L1                              # 9 칸, 퍽은 손으로 쥐여 줌 (칸마다 쥐여 달라고 물음)
  python3 place_measure.py --place L1 --unload 1_0,1_1,1_2,1_3,2_0,2_1,2_2,2_3,3_0   # 칸마다 팔레트 <열>_<칸> 에서 꺼낸 뒤 놓기
  python3 place_measure.py --place L2 --cells m1_m1,0_0,p1_p1 --note "초안 패치 적용"

칸마다 입력 (Enter 로 넘어감):  앞 왼 섬 밀림 [폭]
  앞  = 칸 중심 -> 퍽 중심, 로봇에서 멀어지면 + (mm)      왼 = 로봇 기준 왼쪽 + (mm)
  섬  = y (서 있음) / n (넘어짐)                         밀림 = 먼저 놓은 퍽이 밀린 최대 mm (없으면 0)
  폭  = (선택) 놓은 직후 벌어진 손가락 사이 mm (#201 집게 폭 ↔ 값)
  예) "3 -2 y 0"   "x" = 넘어짐 (나머지 비움)   "s" = 이 칸 건너뜀   "q" = 끝
"""
import argparse
import csv
import datetime
import os
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from robot_interfaces.action import ExecuteSkill

PARAMS = ["place.sag", "place.radial_offset_mm", "place.radial_offset_slope_mm", "place.clearance", "place.release_position",
          "place.pitch", "sag.enabled", "sag.ki", "pallet.speed", "gripper.close_position"]
ALL9 = [f"{a}_{b}" for a in ("m1", "0", "p1") for b in ("m1", "0", "p1")]
KEEP = ("place:", "sag bias", "stream", "pallet", "IK", "fail", "error")  # 저장할 노드 로그 줄


class M(Node):
    def __init__(self, node_name):
        super().__init__("place_measure")
        self.ac = ActionClient(self, ExecuteSkill, "/manipulator/execute_skill")
        self.pc = self.create_client(GetParameters, f"/{node_name}/get_parameters")
        self.node_name = node_name
        self.logs, self.grip = [], None
        self.create_subscription(Log, "/rosout", self._log, 100)
        self.create_subscription(JointState, "/joint_states", self._js, 10)

    def _log(self, m):
        if m.name.endswith(self.node_name) and any(k in m.msg for k in KEEP):
            self.logs.append(m.msg)

    def _js(self, m):
        if "gripper_roll" in m.name:
            self.grip = m.position[m.name.index("gripper_roll")]

    def params(self):
        if not self.pc.wait_for_service(timeout_sec=3.0):
            return {p: "?" for p in PARAMS}
        f = self.pc.call_async(GetParameters.Request(names=PARAMS))
        rclpy.spin_until_future_complete(self, f, timeout_sec=3.0)
        out = {}
        for p, v in zip(PARAMS, f.result().values if f.result() else []):
            out[p] = {1: v.bool_value, 2: v.integer_value, 3: v.double_value, 4: v.string_value}.get(v.type, "-")
        return out

    def skill(self, name, timeout=90.0):
        """(성공, 메시지, 걸린 초, 이 동작 동안 노드 로그 줄)"""
        if not self.ac.wait_for_server(timeout_sec=5.0):
            return False, "execute_skill 서버 없음", 0.0, []
        self.logs.clear()
        t0 = time.time()
        g = ExecuteSkill.Goal(policy=name, timeout=float(timeout))
        f = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        h = f.result()
        if h is None or not h.accepted:
            return False, "거절됨", time.time() - t0, list(self.logs)
        r = h.get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=timeout + 10.0)
        if r.result() is None:
            return False, "시간 초과", time.time() - t0, list(self.logs)
        res = r.result().result
        for _ in range(20):  # 끝난 뒤 로그 · 관절 값 조금 더 받음
            rclpy.spin_once(self, timeout_sec=0.05)
        return res.success, res.message, time.time() - t0, list(self.logs)

    def grip_now(self):
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.grip


def git_info(repo):
    try:
        c = subprocess.run(["git", "-C", repo, "rev-parse", "--short", "HEAD"], capture_output=True, text=True, timeout=5).stdout.strip()
        b = subprocess.run(["git", "-C", repo, "rev-parse", "--abbrev-ref", "HEAD"], capture_output=True, text=True, timeout=5).stdout.strip()
        d = subprocess.run(["git", "-C", repo, "status", "--short", "--untracked-files=no"], capture_output=True, text=True, timeout=5).stdout.strip()
        return f"{b}@{c} 고친 파일 {len(d.splitlines()) if d else 0}"
    except Exception as e:  # noqa: BLE001
        return f"? ({e})"


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return "q"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--place", required=True, choices=["L1", "L2", "L3"])
    ap.add_argument("--cells", default=",".join(ALL9), help="m1_m1,0_0,... (명령 이름의 dx_dy, 지도 기준)")
    ap.add_argument("--unload", default="", help="칸마다 먼저 꺼낼 팔레트 <열>_<칸> (cells 와 같은 개수), 비우면 손으로 쥐여 줌")
    ap.add_argument("--repeat", type=int, default=1, help="칸마다 반복 수")
    ap.add_argument("--stop_mm", default="", help="상자 앞면 ~ 로봇 앞 거리 (mm, 잰 값)")
    ap.add_argument("--yaw_deg", default="", help="상자에 대해 로봇 틀어짐 (°, 대략)")
    ap.add_argument("--note", default="")
    ap.add_argument("--repo", default=os.path.expanduser("~/Lv2_Cartlidar_Assignment"))
    ap.add_argument("--node", default="manipulator_node")
    ap.add_argument("--out_dir", default=os.path.expanduser("~/place_measure"))
    a = ap.parse_args()

    cells = [c for c in a.cells.split(",") if c]
    slots = [s for s in a.unload.split(",") if s]
    if slots and len(slots) != len(cells) * a.repeat:
        sys.exit(f"--unload 개수 {len(slots)} != 칸 수 x 반복 {len(cells) * a.repeat}")
    rclpy.init()
    n = M(a.node)
    prm = n.params()
    os.makedirs(a.out_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime("%m%d_%H%M")
    path = os.path.join(a.out_dir, f"{ts}_{a.place}.csv")
    fh = open(path, "w", newline="")
    fh.write(f"# {ts} {a.place} git {git_info(a.repo)} 정지 {a.stop_mm} mm 틀어짐 {a.yaw_deg}° 쥐기 {'꺼내기' if slots else '손'} 메모 {a.note}\n")
    fh.write("# " + " ".join(f"{k}={v}" for k, v in prm.items()) + "\n")
    w = csv.writer(fh)
    w.writerow(["순서", "칸", "꺼낸칸", "꺼내기", "놓기", "초", "앞mm", "왼mm", "섬", "밀림mm", "폭mm", "집게쥠", "집게놓음", "tip", "보정"])
    print(f"기록 파일 {path}\n설정 {prm}")
    rows, k = [], 0
    for rep in range(a.repeat):
        for c in cells:
            sl = slots[k] if slots else ""
            k += 1
            un = ""
            if sl:
                ok, msg, _, lg = n.skill(f"pallet_unload_{sl}_tilt")
                un = "ok" if ok else f"실패:{msg}"
                bias = next((x.split("bias")[-1].strip() for x in lg if "sag bias" in x), "")
                print(f"[{k}] 꺼내기 {sl}: {un} {bias}")
                if not ok:
                    w.writerow([k, c, sl, un, "", "", "", "", "", "", "", "", "", "", bias]); fh.flush()
                    if ask("  꺼내기 실패. Enter = 다음 칸, q = 끝: ") == "q":
                        break
                    continue
            else:
                v0 = ask(f"[{k}] {c}: 퍽을 쥐여 주고 Enter (s 건너뜀, q 끝): ")
                if v0 == "q":
                    break
                if v0 == "s":
                    continue
            g_hold = n.grip_now()
            ok, msg, dt, lg = n.skill(f"place_{a.place}_{c}_tilt")
            g_rel = n.grip_now()
            tip = next((x.split("tip")[-1].split(" in ")[0].strip() for x in lg if "place: tip" in x), "")
            st = next((x.split("stream")[-1].strip() for x in lg if "place: stream" in x), "")
            print(f"[{k}] 놓기 {c}: {'ok' if ok else '실패 ' + msg} {dt:.1f}s tip {tip} {st}")
            v = ask("  앞 왼 섬 밀림 [폭] (x 넘어짐, s 건너뜀, q 끝): ")
            if v == "q":
                break
            f = v.split() if v not in ("s", "x") else (["", "", "n"] if v == "x" else [])
            f += [""] * (5 - len(f))
            row = [k, c, sl, un, "ok" if ok else f"실패:{msg}", f"{dt:.1f}", f[0], f[1], f[2], f[3], f[4],
                   f"{g_hold:.4f}" if g_hold is not None else "", f"{g_rel:.4f}" if g_rel is not None else "", tip, st]
            w.writerow(row); fh.flush(); rows.append(row)
            if ask("  다음 칸 준비 (먼저 놓은 퍽은 그대로) 되면 Enter: ") == "q":
                break
        else:
            continue
        break
    fh.close()
    # 요약 (이것만 붙여 넣어도 됨)
    print(f"\n===== 요약 {path} =====")
    print(open(path).read().splitlines()[0]); print(open(path).read().splitlines()[1])
    for r in rows:
        print(f"{r[1]:>6} 꺼냄{r[2] or '-':>4} 놓기 {r[4]:<4} 앞 {r[6] or '-':>5} 왼 {r[7] or '-':>5} 섬 {r[8] or '-'} 밀림 {r[9] or '-'} 폭 {r[10] or '-'} 집게 {r[11]}/{r[12]}")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
