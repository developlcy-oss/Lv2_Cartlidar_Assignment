#!/usr/bin/env python3
"""Gazebo 장애물 시험 (2026-11-17): 주행 중 로봇 앞 길 위에 장애물을 놓고 (치우고), Gazebo 실제 자세로 충돌 여유를 잼.

  python3 scenario.py --scenario <시나리오.json|none> --known <알려진 물체 지도 yaml> --out <기록 폴더> [--move-boxes]

시나리오 json: {"obstacles": [{"id": "o1", "leg": "box3>box5", "after_m": 0.3, "ahead_m": 1.0, "lateral": 0.0,
                               "shape": "box|person|wall", "remove_after_stop_s": 1.0 또는 null}]}
  - leg 구간에서 주행을 시작해 after_m 만큼 간 뒤, 지금 따라가는 조각 (/avoid/path_ahead) 을 따라 ahead_m 앞 (옆으로 lateral) 에 놓음
  - remove_after_stop_s: 로봇이 그 장애물 때문에 멈춘 ('정지') 뒤 이만큼 지나 치움 (null = 그대로 둠)
  - shape: box = 퍽 상자 크기 20.5 × 20.5 cm (높이는 시뮬 라이다 스캔면 7.5 cm 보다 높게 10 cm ★ 실물 상자 7 cm 는 실물 라이다가 봤음)
           person = 선생님 발 두 개 (10 × 26 × 12 cm, 옆으로 0.22 m 사이) + 다리 · 몸 · wall = 길을 가로막는 판 55 × 10 × 30 cm
--move-boxes: Gazebo 상자 · L 9 개를 길 모음 실측 자리로 옮김 (set_pose)
기록: truth.csv (Gazebo 실제 자세 · 지도 물체까지 여유 · 장애물까지 여유) · scenario_events.json · truth_summary.txt
여유 = 차체 + 팔 끝 6 각형 (Nav2 footprint) 둘레에서 가장 가까운 물체까지 [cm], 음수 = 겹침 (충돌)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import threading

import numpy as np
import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from obstacle_guard import load_known  # noqa: E402

WORLD = "dice_mission"
ROBOT = "waffle_4wd"
FP = np.array([[0.25, 0.07], [0.25, -0.07], [0.1328, -0.1676], [-0.2135, -0.1676], [-0.2135, 0.1676], [0.1328, 0.1676]])


def boundary(poly, step=0.01):
    out = []
    for a, b in zip(poly, np.roll(poly, -1, 0)):
        n = max(2, int(np.linalg.norm(b - a) / step))
        out.append(a + (b - a) * np.linspace(0, 1, n, endpoint=False)[:, None])
    return np.vstack(out)


FPB = boundary(FP)


def rect_dist(P, cx, cy, yaw, a, b):
    """점들 → 회전 직사각형 (가운데 · 방향 · 변 a × b) 까지 부호 거리 (안 = 음수)"""
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = P[:, 0] - cx, P[:, 1] - cy
    u, v = c * dx + s * dy, -s * dx + c * dy
    qx, qy = np.abs(u) - a / 2, np.abs(v) - b / 2
    return np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0)


def shape_parts(shape):
    """(로컬 x, 로컬 y, 크기 x, 크기 y, 크기 z, 바닥 z) 상자들 — 로컬 x = 길 방향"""
    if shape == "box":
        return [(0, 0, 0.205, 0.205, 0.10, 0.0)]
    if shape == "wall":
        return [(0, 0, 0.10, 0.55, 0.30, 0.0)]
    if shape == "person":   # 발 두 개 (라이다 높이) + 다리 두 개 + 몸 (보이기만)
        return [(0, 0.11, 0.26, 0.10, 0.12, 0.0), (0, -0.11, 0.26, 0.10, 0.12, 0.0),
                (0, 0.11, 0.12, 0.12, 0.75, 0.12), (0, -0.11, 0.12, 0.12, 0.75, 0.12), (0, 0, 0.25, 0.40, 0.65, 0.87)]
    raise ValueError(shape)


def sdf_of(name, shape):
    links = []
    for i, (lx, ly, sx, sy, sz, z0) in enumerate(shape_parts(shape)):
        g = f"<box><size>{sx} {sy} {sz}</size></box>"
        col = f"<collision name='c'><geometry>{g}</geometry></collision>" if z0 < 0.3 else ""
        links.append(f"<link name='l{i}'><pose>{lx} {ly} {z0 + sz / 2} 0 0 0</pose>{col}"
                     f"<visual name='v'><geometry>{g}</geometry><material><ambient>0.85 0.15 0.15 1</ambient><diffuse>0.85 0.15 0.15 1</diffuse></material></visual></link>")
    return f"<?xml version='1.0'?><sdf version='1.9'><model name='{name}'><static>true</static>{''.join(links)}</model></sdf>"


def gz_service(service, reqtype, req):
    cmd = ["gz", "service", "-s", f"/world/{WORLD}/{service}", "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean", "--timeout", "4000", "--req", req]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    return r.returncode == 0 and "true" in r.stdout


class Scenario(Node):
    def __init__(self, a):
        super().__init__("avoid_scenario", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.a = a
        self.obs = json.load(open(a.scenario))["obstacles"] if a.scenario != "none" else []
        for o in self.obs:
            o.update({"spawned": None, "removed": None, "stop_t": None, "pose": None})
        self.D, self.ox, self.oy, self.res = load_known(a.known)
        self.truth = None
        self.state, self.leg, self.leg_t0, self.leg_pos0 = "", "", None, None
        self.path = None
        self.path_leg_fixed = None   # 장애물을 놓은 뒤에는 우회 길로 바뀌지 않게 (기록용)
        self.rows = []
        self.log = []
        self.f = open(os.path.join(a.out, "truth.csv"), "w", buffering=1)
        self.f.write("t,x,y,yaw,clear_static_cm,clear_obs_cm,state,leg\n")
        # Gazebo 실제 자세: gz-transport 로 바로 받음 (ros_gz_bridge 의 Pose_V → TF 는 모델 이름을 빠뜨림, 11-17)
        from gz.msgs.pose_v_pb2 import Pose_V
        from gz.transport import Node as GzNode
        self.gz = GzNode()
        if not self.gz.subscribe(Pose_V, f"/world/{WORLD}/dynamic_pose/info", self._truth):
            print("[시나리오] ✗ Gazebo 자세 구독 실패", flush=True)
        self.create_subscription(String, "/avoid/state", self._state, 50)
        self.create_subscription(Path, "/avoid/leg_plan", self._path, 10)   # 구간 전체 길 (조각 하나가 아니라)
        self.create_timer(0.1, self._tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _truth(self, m):
        for p in m.pose:
            if p.name == ROBOT:
                q = p.orientation
                self.truth = (p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
                return

    def _path(self, m):
        if m.poses and m.header.frame_id == "map" and self.path_leg_fixed is None:
            self.path = np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses])

    def _state(self, m):
        d = json.loads(m.data)
        if d.get("leg") != self.leg or d["state"] == "주행" and self.state != "주행":
            if d.get("leg") != self.leg:
                self.leg, self.leg_t0, self.leg_pos0 = d.get("leg"), self.now(), self.truth
        self.state = d["state"]
        if self.state == "정지":
            for o in self.obs:
                if o["spawned"] is not None and o["removed"] is None and o["stop_t"] is None:
                    o["stop_t"] = self.now()
        self.log.append({"t": round(self.now(), 2), **d})

    def footprint(self, pose):
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        return np.c_[x + c * FPB[:, 0] - s * FPB[:, 1], y + s * FPB[:, 0] + c * FPB[:, 1]]

    def clear_static(self, P):
        i = np.clip(((P[:, 0] - self.ox) / self.res).astype(int), 0, self.D.shape[1] - 1)
        j = np.clip(((P[:, 1] - self.oy) / self.res).astype(int), 0, self.D.shape[0] - 1)
        return float(self.D[j, i].min()) - self.res / 2

    def clear_obs(self, P):
        best = 9.0
        for o in self.obs:
            if o["pose"] is None or o["removed"] is not None:
                continue
            ox, oy, oyaw = o["pose"]
            c, s = math.cos(oyaw), math.sin(oyaw)
            for lx, ly, sx, sy, sz, z0 in shape_parts(o["shape"]):
                if z0 > 0.3:
                    continue
                best = min(best, float(rect_dist(P, ox + c * lx - s * ly, oy + s * lx + c * ly, oyaw, sx, sy).min()))
                # 장애물이 차체 안에 통째로 들어간 경우 (점들이 밖) 대비: 장애물 가운데가 6 각형 안이면 겹침
        return best

    def _tick(self):
        if self.truth is None:
            return
        P = self.footprint(self.truth)
        cs, co = self.clear_static(P), self.clear_obs(P)
        t = self.now()
        self.rows.append((t, cs, co, self.truth[0], self.truth[1]))
        self.f.write(f"{t:.2f},{self.truth[0]:.4f},{self.truth[1]:.4f},{math.degrees(self.truth[2]):.1f},{cs * 100:.1f},{co * 100:.1f},{self.state},{self.leg}\n")
        for o in self.obs:
            if o["spawned"] is None and self.leg == o["leg"] and self.state == "주행" and self.leg_pos0 is not None:
                moved = math.hypot(self.truth[0] - self.leg_pos0[0], self.truth[1] - self.leg_pos0[1])
                if moved >= o.get("after_m", 0.3) and self.path is not None:
                    o["spawned"] = t   # 한 번만
                    threading.Thread(target=self.spawn, args=(o,), daemon=True).start()
            force = o.get("remove_after_s") is not None and o["spawned"] is not None and t - o["spawned"] >= o["remove_after_s"]
            if o["spawned"] is not None and o["removed"] is None and o.get("pose") is not None and (force or (o.get("remove_after_stop_s") is not None
                    and o["stop_t"] is not None and t - o["stop_t"] >= o["remove_after_stop_s"])):
                o["removed"] = t
                threading.Thread(target=self.remove, args=(o,), daemon=True).start()

    def spawn(self, o):
        if "gap_m" in o:   # 차체 앞면 (base_link x 0.1328) 에서 장애물 앞면까지 o["gap_m"] → 가운데까지 길 거리
            depth = max(abs(lx) + sx / 2 for lx, ly, sx, sy, sz, z0 in shape_parts(o["shape"]) if z0 < 0.3)
            o["ahead_m"] = 0.1328 + o["gap_m"] + depth
        self.spawn_truth = self.truth
        P, (rx, ry) = self.path, self.truth[:2]
        k = int(np.argmin(np.hypot(P[:, 0] - rx, P[:, 1] - ry)))
        acc, j = 0.0, k
        while j + 1 < len(P) and acc < o["ahead_m"]:
            acc += float(np.hypot(*(P[j + 1] - P[j])))
            j += 1
        if acc < o["ahead_m"] - 0.05:
            print(f"[시나리오] {o['id']} 못 놓음: 구간 남은 길 {acc:.2f} m < {o['ahead_m']} m", flush=True)
            o["skipped"] = True
            return
        j2 = min(j + 1, len(P) - 1)
        yaw = math.atan2(P[j2, 1] - P[j - 1 if j2 == j else j, 1], P[j2, 0] - P[j - 1 if j2 == j else j, 0])
        lat = o.get("lateral", 0.0)
        x, y = P[j, 0] - math.sin(yaw) * lat, P[j, 1] + math.cos(yaw) * lat
        o["pose"] = (x, y, yaw)
        o["ahead_real_m"] = round(acc, 2)
        sdf = sdf_of(o["id"], o["shape"])
        r = subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-world", WORLD, "-name", o["id"], "-string", sdf,
                            "-x", f"{x:.4f}", "-y", f"{y:.4f}", "-z", "0.0", "-Y", f"{yaw:.4f}"], capture_output=True, text=True, timeout=30)
        print(f"[시나리오] {o['id']} ({o['shape']}) 놓음 ({x:.2f}, {y:.2f}) · 로봇 앞 길 {acc:.2f} m · {'성공' if r.returncode == 0 else '실패 ' + r.stderr[-200:]}", flush=True)

    def remove(self, o):
        ok = gz_service("remove", "gz.msgs.Entity", f'name: "{o["id"]}" type: MODEL')
        print(f"[시나리오] {o['id']} 치움 (멈춘 뒤 {o['remove_after_stop_s']} s) · {'성공' if ok else '실패'}", flush=True)

    def finish(self):
        self.f.close()
        R = np.array(self.rows) if self.rows else np.zeros((0, 5))
        cs = float(R[:, 1].min()) if len(R) else float("nan")
        co = float(R[:, 2].min()) if len(R) else float("nan")
        hit_s = int((R[:, 1] < 0).sum()) if len(R) else 0
        hit_o = int((R[:, 2] < 0).sum()) if len(R) else 0
        lines = [f"Gazebo 실제 자세 기준: 지도 물체 (벽 · 상자) 최소 여유 {cs * 100:.1f} cm (겹침 {hit_s / 10:.1f} s) · 장애물 최소 여유 {co * 100 if co < 9 else float('nan'):.1f} cm (겹침 {hit_o / 10:.1f} s)"]
        for o in self.obs:
            lines.append(f"  {o['id']} {o['shape']} {o['leg']}: 놓음 {'예' if o['pose'] else '아니오'}" + (f" ({o['pose'][0]:.2f}, {o['pose'][1]:.2f}) 로봇 앞 {o.get('ahead_real_m')} m" if o['pose'] else "")
                         + (f" · 멈춤 t {o['stop_t']:.1f}" if o["stop_t"] else " · 그 장애물로 멈추지 않음") + (f" · 치움 t {o['removed']:.1f}" if o["removed"] else ""))
        for o in self.obs:
            if o.get("pose") is None or not len(R):
                continue
            t0 = o["spawned"]
            k0 = int(np.searchsorted(R[:, 0], t0))
            if k0 < 3 or k0 >= len(R) - 3:
                continue
            v0 = float(np.hypot(R[k0, 3] - R[k0 - 3, 3], R[k0, 4] - R[k0 - 3, 4]) / max(R[k0, 0] - R[k0 - 3, 0], 1e-3))
            k1 = k0
            while k1 + 3 < len(R) and np.hypot(R[k1 + 3, 3] - R[k1, 3], R[k1 + 3, 4] - R[k1, 4]) / max(R[k1 + 3, 0] - R[k1, 0], 1e-3) > 0.02:
                k1 += 1
            travel = float(np.hypot(R[k1, 3] - R[k0, 3], R[k1, 4] - R[k0, 4]))
            t_end = o["removed"] if o["removed"] else R[-1, 0]
            seg = R[(R[:, 0] >= t0) & (R[:, 0] <= t_end)]
            cmin = float(seg[:, 2].min()) if len(seg) else float("nan")
            o.update({"v_at_spawn": round(v0, 3), "stop_travel_m": round(travel, 3), "stop_after_s": round(R[k1, 0] - t0, 2), "min_clear_m": round(cmin, 3)})
            lines.append(f"  [갑자기] {o['id']} 간격 {o.get('gap_m', '-')} m · 놓을 때 속도 {v0:.2f} m/s · 멈출 때까지 {travel * 100:.1f} cm ({R[k1, 0] - t0:.2f} s) · "
                         f"장애물 최소 여유 {cmin * 100:.1f} cm · {'충돌' if cmin < 0.005 else '안 닿음'}")
        open(os.path.join(self.a.out, "truth_summary.txt"), "w").write("\n".join(lines) + "\n")
        json.dump({"obstacles": [{k: v for k, v in o.items()} for o in self.obs], "states": self.log},
                  open(os.path.join(self.a.out, "scenario_events.json"), "w"), ensure_ascii=False, indent=1, default=str)
        print("\n".join(lines), flush=True)


def move_boxes():
    """Gazebo 상자 · L 9 개를 길 모음 실측 자리로. 11-17 밤: 고정 (static) 모델은 set_pose 가 '성공' 만 답하고 안 옮김 (TFmini 가 원래 자리 상자를 봄)
    → 지우고 실측 자리에 같은 크기 상자로 다시 만듦. L 앞 태그판 (place_tag_*, 카메라 표시) 은 주행 시험에 필요 없어 지움"""
    import replan  # noqa: F401
    import make_legs as ml
    ok = 0
    for name, (x, y) in ml.BOX.items():
        gz_service("remove", "gz.msgs.Entity", f'name: "{name}" type: MODEL')
        col = "0.55 0.45 0.3 1" if name.startswith("place") else "0.8 0.68 0.48 1"
        sdf = (f"<?xml version='1.0'?><sdf version='1.9'><model name='{name}'><static>true</static><link name='link'>"
               f"<visual name='box'><pose>0 0 0.035 0 0 0</pose><geometry><box><size>0.205 0.205 0.07</size></box></geometry>"
               f"<material><ambient>{col}</ambient><diffuse>{col}</diffuse></material></visual>"
               f"<collision name='box_collision'><pose>0 0 0.035 0 0 0</pose><geometry><box><size>0.205 0.205 0.07</size></box></geometry></collision>"
               f"</link></model></sdf>")
        r = subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-world", WORLD, "-name", name, "-string", sdf, "-x", f"{x}", "-y", f"{y}", "-z", "0"],
                           capture_output=True, text=True, timeout=30)
        ok += r.returncode == 0
    tags = sum(gz_service("remove", "gz.msgs.Entity", f'name: "place_tag_{i}" type: MODEL') for i in (12, 13, 14))
    print(f"[시나리오] Gazebo 상자 · L {ok}/{len(ml.BOX)} 개를 실측 자리에 다시 만듦 · 태그판 {tags} 개 지움", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="none")
    ap.add_argument("--known", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--move-boxes", action="store_true")
    a = ap.parse_args()
    if a.move_boxes:
        move_boxes()
        return
    rclpy.init()
    n = Scenario(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:  # noqa: BLE001  (끝낼 때 rclpy 종료와 겹침 — 기록은 남김)
        print(f"[시나리오] 종료 중 {type(e).__name__}", flush=True)
    n.finish()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
