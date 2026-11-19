#!/usr/bin/env python3
"""장애물 감시 (2026-11-17): 앞으로 갈 경로 띠 안에 '지도에 없는' 점이 있으면 막힘.

  python3 obstacle_guard.py --ros-args -p known_map:=<지도 yaml> [-p use_sim_time:=true]

입력
  /scan_front · /scan_rear  LDS-03 원래 스캔 (11-17 밤 b: 팀 scan_body_mask · scan_merge 를 거치지 않음 — 그 상자는 앞 x ≤ 0.285 m 를 지워
                            차체 앞 15 cm 가 안 보였음). 차체 상자 body_box 안 점 · 출발 전 정지 구간에 배운 '차체에 가린 칸' 을 직접 지움
  /tfmini/range             정면 TFmini (c): 맞은 자리가 알려진 물체 밖이면 그 자체로 장애물 (발 · 낮은 물체 · 라이다가 못 보는 차체 앞 10 cm)
  /avoid/path_ahead         주행 관리자가 지금 따라가는 조각 (nav_msgs/Path, map). frame_id "spin" = 제자리 회전 (poses[0] = 가운데) · 빈 경로 = 감시 끔
  TF map → base_link (AMCL) · base_link → laser_* · tfmini_link (고정)
판단 (스캔마다)
  1. 스캔 점 → base_link (차체 상자 · 가린 칸 지움) → map
  2. 알려진 물체 (known_map 점유 칸) 에서 known_tol 넘게 떨어진 점만 '모르는 점'
  3. 경로 띠: 로봇이 경로에 가장 가까운 곳부터 경로를 따라 lookahead m 까지, 경로에서 half_width 안 (제자리 회전이면 가운데에서 spin_radius 안)
  4. 띠 안 모르는 점이 min_points 개 이상 →
       그중 하나라도 로봇 가운데에서 close_range 안이면 그 스캔에서 바로 막힘 (a: 확인 스캔을 기다리지 않음 — 0.45 m/s 에서 4.5 cm 덜 감)
       아니면 같은 라이다의 스캔이 persist 번 이어질 때 막힘
     TFmini: 경로가 앞으로 가는 조각이고 맞은 자리가 모르는 자리 · 띠 안이면 바로 막힘
출력  /avoid/blocked (Bool) · /avoid/status (JSON: n · blocked · why · pts · near)
소프트웨어 정지 (물리 비상정지 아님). 라이다 스캔면 (바닥 위 약 7.5 cm) 보다 낮은 물체는 라이다로 못 봄 (정면 한 줄기는 TFmini 2.4 cm).
★ 가린 칸 학습: 노드가 뜬 뒤 첫 스캔부터 learn_window (0.5 ~ 1.9 s) 동안 로봇이 서 있어야 함.
"""
from __future__ import annotations

import json
import math
import os

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Path
from PIL import Image
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Duration, Time
from scipy import ndimage
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


def load_known(path):
    """점유 지도 (map_server yaml) → (점유에서 거리 [m] 배열, 원점 x, y, 해상도). 행 0 = y 최소"""
    m = yaml.safe_load(open(path))
    img_path = m["image"] if os.path.isabs(m["image"]) else os.path.join(os.path.dirname(os.path.abspath(path)), m["image"])
    img = np.array(Image.open(img_path))
    if img.ndim == 3:
        img = img[..., 0]
    img = img[::-1].astype(np.float32)
    p = img / 255.0 if not m.get("negate", 0) else 1.0 - img / 255.0
    occ = (1.0 - p) > m.get("occupied_thresh", 0.65)
    res = float(m["resolution"])
    dist = ndimage.distance_transform_edt(~occ) * res
    return dist.astype(np.float32), float(m["origin"][0]), float(m["origin"][1]), res


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rot_of(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Guard(Node):
    def __init__(self):
        super().__init__("obstacle_guard")
        P = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.known_tol = P("known_tol", 0.08)       # 지도 물체에서 이만큼 넘게 떨어진 점만 '모르는 점' [m]
        self.half_w = P("half_width", 0.22)          # 경로 띠 반폭 [m] = 차체 반폭 0.1676 + 0.05
        self.look = P("lookahead", 1.0)              # 경로 따라 앞 [m]
        self.spin_r = P("spin_radius", 0.32)         # 제자리 회전: 가운데에서 [m]
        self.min_pts = int(P("min_points", 5))
        self.persist = int(P("persist", 2))
        self.close_r = P("close_range", 0.75)        # (a) 이 안의 점은 한 스캔으로 바로 [m, 로봇 가운데에서] = 차체 앞 약 0.6 m
        self.near_r = P("near_radius", 2.0)
        self.box = [float(v) for v in P("body_box", [-0.235, 0.16, 0.19])]   # (b) 차체 상자 x 뒤 · x 앞 · |y| [m, base_link] (팀 상자 앞 0.285 → 0.16)
        self.learn = [float(v) for v in P("learn_window", [0.5, 1.9])]
        self.tf_stop = P("tfmini_range", 0.45)       # (c) TFmini 가 이보다 가까우면 [m, 렌즈에서]
        self.tf_min = P("tfmini_min", 0.03)
        self.tf_known_tol = P("tfmini_known_tol", 0.15)   # TFmini 는 한 줄기라 위치 오차에 약함 → 알려진 물체 허용을 넓게 (정지점 앞 상자)
        self.tf_end_skip = P("tfmini_end_skip", 0.30)     # 조각 남은 길이 이 안 (정지점에 다가가는 끝) 에서는 TFmini 안 씀 [m]
        self.tf_self = P("tfmini_self_max", 0.17)         # 이보다 가까우면 자기 몸 (주행 자세 집게 손가락, x 0.23 ~ 0.29) 으로 보고 무시 [m, 렌즈에서]
        self.tf_persist = int(P("tfmini_persist", 5))     # TFmini 판정이 이만큼 이어져야 막힘 (약 100 Hz → 50 ms, 흔들리는 손가락 걸침 거름)
        self.odom = P("odom_frame", "odom")
        known = P("known_map", "")
        if not known:
            raise SystemExit("known_map (알려진 물체 지도 yaml) 이 필요함")
        self.D, self.ox, self.oy, self.res = load_known(known)
        self.Gy, self.Gx = np.gradient(self.D, self.res)     # 거리장 기울기 (행 = y, 열 = x)
        self.align_on = bool(P("align", True))               # 스캔을 알려진 물체 지도에 맞춰 위치 오차를 바로잡은 뒤 판단 (11-17 밤)
        self.corr = (0.0, 0.0, 0.0)
        self.tf_dbg = None
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.path, self.spin_c = None, None
        self.hits = {}
        self.extr = {}               # frame → (R 3×3, t 3) base_link 기준
        self.t0, self.learning, self.fixed = None, {}, {}
        self.last_pts = {}           # 토픽 → (스캔 시각, 모르는 점 map) — 둘레 점 합치기
        self.src = {}                # 소스 (라이다 토픽 · /tfmini) → (시각, 막힘, 띠 안 점 수, 띠 안 점, 이유) — 하나라도 막히면 막힘
        self.tf_hit = 0
        self.pub_b = self.create_publisher(Bool, "/avoid/blocked", 10)
        self.pub_s = self.create_publisher(String, "/avoid/status", 10)
        self.create_subscription(Path, "/avoid/path_ahead", self._path, 10)
        for t in P("scan_topics", ["/scan_front", "/scan_rear"]):
            self.create_subscription(LaserScan, t, lambda m, t=t: self._scan(t, m), qos_profile_sensor_data)
        tft = P("tfmini_topic", "/tfmini/range")
        if tft and tft != "none":
            self.create_subscription(Range, tft, self._tfmini, qos_profile_sensor_data)
        self.get_logger().info(f"알려진 물체 지도 {known} · 모르는 점 = 물체에서 {self.known_tol} m 넘게 · 띠 반폭 {self.half_w} · 앞 {self.look} m · "
                               f"점 {self.min_pts} 개 ({self.close_r} m 안 = 한 스캔, 그 밖 = {self.persist} 스캔) · 차체 상자 {self.box} · TFmini < {self.tf_stop} m")

    # ── 경로 ──
    def _path(self, m: Path):
        if not m.poses:
            self.path, self.spin_c = None, None
        elif m.header.frame_id == "spin":
            p = m.poses[0].pose.position
            self.path, self.spin_c = None, (p.x, p.y)
        else:
            self.path = np.array([[q.pose.position.x, q.pose.position.y] for q in m.poses], dtype=np.float64)
            self.spin_c = None
        self.hits, self.tf_hit, self.src = {}, 0, {}

    def _known_dist(self, X, Y):
        i = ((X - self.ox) / self.res).astype(int)
        j = ((Y - self.oy) / self.res).astype(int)
        inside = (i >= 0) & (i < self.D.shape[1]) & (j >= 0) & (j < self.D.shape[0])
        d = np.zeros(np.shape(X), np.float32)   # 지도 밖 = 알려진 것
        d[inside] = self.D[j[inside], i[inside]]
        return d

    def _sample(self, X, Y):
        i = np.clip(((X - self.ox) / self.res).astype(int), 0, self.D.shape[1] - 1)
        j = np.clip(((Y - self.oy) / self.res).astype(int), 0, self.D.shape[0] - 1)
        return self.D[j, i], self.Gx[j, i], self.Gy[j, i]

    def _align(self, X, Y, rx, ry):
        """점들을 로봇 자리 둘레로 돌리고 옮겨 알려진 물체 거리장에 맞춤 (가우스-뉴턴 6 번, 벽 0.20 → 0.10 m 안 점만).
        AMCL 이 몇 cm · 몇 ° 어긋나면 벽 점이 '모르는 점' 으로 나와 거짓 장애물이 됐음 (11-17 밤: σ 2.7 cm · 1.7° 에서 모르는 점 425 개).
        장애물 점은 소수라 거의 안 끌려감. 반환 (dx, dy, dθ) — 15 cm · 5° 넘거나 맞출 점이 모자라면 (0, 0, 0)"""
        dx = dy = dth = 0.0
        for it in range(6):
            c, s_ = math.cos(dth), math.sin(dth)
            qx = rx + c * (X - rx) - s_ * (Y - ry) + dx
            qy = ry + s_ * (X - rx) + c * (Y - ry) + dy
            d, gx, gy = self._sample(qx, qy)
            m = d < (0.20 if it < 2 else 0.10)
            if m.sum() < 40:
                return (0.0, 0.0, 0.0)
            J = np.c_[gx[m], gy[m], -gx[m] * (qy[m] - ry - dy) + gy[m] * (qx[m] - rx - dx)]
            H = J.T @ J + np.diag([1e-3, 1e-3, 1e-3])
            step = -np.linalg.solve(H, J.T @ d[m])
            dx, dy, dth = dx + step[0], dy + step[1], dth + step[2]
        if abs(dx) > 0.15 or abs(dy) > 0.15 or abs(dth) > math.radians(5):
            return (0.0, 0.0, 0.0)
        return (dx, dy, dth)

    def _corridor(self, rx, ry):
        P = self.path
        if len(P) == 1:
            return P
        k0 = int(np.argmin(np.hypot(P[:, 0] - rx, P[:, 1] - ry)))
        seg, acc = [P[k0]], 0.0
        for k in range(k0 + 1, len(P)):
            acc += float(np.hypot(*(P[k] - P[k - 1])))
            seg.append(P[k])
            if acc >= self.look:
                break
        return np.array(seg)

    @staticmethod
    def _dist_to_polyline(Q, L):
        if len(L) == 1:
            return np.hypot(Q[:, 0] - L[0, 0], Q[:, 1] - L[0, 1])
        A, B = L[:-1], L[1:]
        V = B - A
        l2 = np.maximum((V ** 2).sum(1), 1e-12)
        W = Q[:, None, :] - A[None, :, :]
        t = np.clip((W * V[None]).sum(2) / l2[None], 0.0, 1.0)
        C = A[None] + t[..., None] * V[None]
        return np.hypot(Q[:, None, 0] - C[..., 0], Q[:, None, 1] - C[..., 1]).min(1)

    def _in_band(self, U, rx, ry):
        if self.spin_c is not None:
            return np.hypot(U[:, 0] - self.spin_c[0], U[:, 1] - self.spin_c[1]) <= self.spin_r
        if self.path is not None:
            return self._dist_to_polyline(U, self._corridor(rx, ry)) <= self.half_w
        return np.zeros(len(U), bool)

    def _extr(self, frame):
        if frame not in self.extr:
            try:
                t = self.tf.lookup_transform("base_link", frame, Time())
            except Exception:  # noqa: BLE001
                return None
            self.extr[frame] = (rot_of(t.transform.rotation), np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]))
        return self.extr[frame]

    def _robot(self, stamp):
        """스캔 시각의 map → base_link = (최신 map → odom, AMCL) ∘ (그 시각 odom → base_link, EKF).
        11-17 밤: map → base_link 를 한 번에 찾으면 AMCL 이 아직 그 시각까지 안 와서 '가장 최근 공통 시각' (AMCL 마지막 갱신, 1 ~ 2 s 전) 자세가
        나와 벽 점이 엉뚱한 자리로 가 거짓 장애물이 됐음"""
        try:
            ob = self.tf.lookup_transform(self.odom, "base_link", Time.from_msg(stamp), timeout=Duration(seconds=0.03))
        except Exception:  # noqa: BLE001
            try:
                ob = self.tf.lookup_transform(self.odom, "base_link", Time())
            except Exception:  # noqa: BLE001
                return None
        try:
            mo = self.tf.lookup_transform("map", self.odom, Time())
        except Exception:  # noqa: BLE001
            return None
        a = yaw_of(mo.transform.rotation)
        bx, by, bth = ob.transform.translation.x, ob.transform.translation.y, yaw_of(ob.transform.rotation)
        c, s_ = math.cos(a), math.sin(a)
        return (mo.transform.translation.x + c * bx - s_ * by, mo.transform.translation.y + s_ * bx + c * by, math.atan2(math.sin(a + bth), math.cos(a + bth)))

    # ── 라이다 ──
    def _scan(self, topic, s: LaserScan):
        ex = self._extr(s.header.frame_id)
        if ex is None:
            return
        R, tr = ex
        r = np.asarray(s.ranges, dtype=np.float64)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r >= max(s.range_min, 0.02)) & (r <= s.range_max)
        L = np.c_[r * np.cos(a), r * np.sin(a), np.zeros(len(r))]
        B = L @ R.T + tr
        inside = ok & (B[:, 0] >= self.box[0]) & (B[:, 0] <= self.box[1]) & (np.abs(B[:, 1]) <= self.box[2])
        ts = s.header.stamp.sec + s.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = ts
        if self.learn[0] <= ts - self.t0 <= self.learn[1]:      # 출발 전 정지: 차체 (상자 + 1 cm) 에 맞은 칸 배우기
            grow = ok & (B[:, 0] >= self.box[0] - 0.01) & (B[:, 0] <= self.box[1] + 0.01) & (np.abs(B[:, 1]) <= self.box[2] + 0.01)
            prev = self.learning.get(topic)
            self.learning[topic] = grow if prev is None or len(prev) != len(grow) else (prev | grow)
        elif ts - self.t0 > self.learn[1] and topic in self.learning:
            self.fixed[topic] = self.learning.pop(topic)
            self.get_logger().info(f"{topic}: 차체에 가린 칸 {int(self.fixed[topic].sum())} 개 배움 ({len(r)} 칸 중)")
        keep = ok & ~inside
        fx = self.fixed.get(topic)
        if fx is not None and len(fx) == len(keep):
            keep &= ~fx
        rp = self._robot(s.header.stamp)
        if rp is None:
            return
        rx, ry, yaw = rp
        bx, by = B[keep, 0], B[keep, 1]
        if self.align_on:
            c, sn = math.cos(yaw), math.sin(yaw)
            X0, Y0 = rx + c * bx - sn * by, ry + sn * bx + c * by
            self.corr = self._align(X0, Y0, rx, ry)
            rx, ry, yaw = rx + self.corr[0], ry + self.corr[1], yaw + self.corr[2]
        c, sn = math.cos(yaw), math.sin(yaw)
        X, Y = rx + c * bx - sn * by, ry + sn * bx + c * by
        unk = self._known_dist(X, Y) > self.known_tol
        U = np.c_[X[unk], Y[unk]]
        Ub = np.c_[bx[unk], by[unk]]                # base_link 기준 (가까움 판단)
        m = self._in_band(U, rx, ry) if len(U) else np.zeros(0, bool)
        pts, n = U[m], int(m.sum())
        close = n >= self.min_pts and bool((np.hypot(Ub[m, 0], Ub[m, 1]) <= self.close_r).any())
        self.hits[topic] = self.hits.get(topic, 0) + 1 if n >= self.min_pts else 0
        why = "가까움 (한 스캔)" if close else (f"{self.persist} 스캔" if self.hits[topic] >= self.persist else "")
        self.last_pts[topic] = (ts, U)
        self._publish(ts, rx, ry, yaw, n, pts, why, topic)

    # ── TFmini ──
    def _tfmini(self, m: Range):
        ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        # 11-17 밤 진단: Gazebo 주행 자세 (0, −1.35, 1.15, 0.81) 에서 집게 손가락이 TFmini 빔에 걸려 30 초에 850 번 0.3 m 안 (대부분 0.10 ~ 0.16 m)
        #   → L 정지점 앞 거짓 멈춤. 실물 DRIVE 자세에서도 가리는지 확인 필요 ★
        if self.path is None or self.spin_c is not None or not (max(self.tf_min, self.tf_self) < m.range <= min(self.tf_stop, m.max_range)):
            self._tf_clear(ts)
            return
        ex = self._extr(m.header.frame_id)
        rp = self._robot(m.header.stamp)
        if ex is None or rp is None:
            return
        R, tr = ex
        hit_b = tr + R @ np.array([m.range, 0.0, 0.0])
        rx, ry, yaw = rp[0] + self.corr[0], rp[1] + self.corr[1], rp[2] + self.corr[2]   # 라이다 맞추기의 최근 보정
        P = self._corridor(rx, ry)
        if len(P) < 2 or (P[-1, 0] - rx) * math.cos(yaw) + (P[-1, 1] - ry) * math.sin(yaw) <= 0:
            self._tf_clear(ts)   # 앞으로 가는 조각이 아님 (후진)
            return
        k0 = int(np.argmin(np.hypot(self.path[:, 0] - rx, self.path[:, 1] - ry)))
        rest = float(np.hypot(np.diff(self.path[k0:, 0]), np.diff(self.path[k0:, 1])).sum()) if k0 < len(self.path) - 1 else 0.0
        if rest < self.tf_end_skip:
            self._tf_clear(ts)   # 조각 끝 (정지점 앞 상자) — 11-17 밤 시뮬: 위치 오차 몇 cm 로 상자 앞면이 '모르는 자리' 로 나와 거짓 멈춤 3 번
            return
        c, sn = math.cos(yaw), math.sin(yaw)
        hx, hy = rx + c * hit_b[0] - sn * hit_b[1], ry + sn * hit_b[0] + c * hit_b[1]
        if self._known_dist(np.array([hx]), np.array([hy]))[0] <= self.tf_known_tol or not self._in_band(np.array([[hx, hy]]), rx, ry)[0]:
            self._tf_clear(ts)
            return
        self.tf_hit += 1
        if self.tf_hit >= self.tf_persist:
            self.tf_dbg = {"range": round(m.range, 3), "hit": [round(hx, 3), round(hy, 3)], "rest": round(rest, 3), "path_n": int(len(self.path)),
                           "k0": k0, "known_d": round(float(self._known_dist(np.array([hx]), np.array([hy]))[0]), 3), "robot": [round(rx, 3), round(ry, 3), round(yaw, 3)],
                           "lens_b": [round(float(tr[0]), 3), round(float(tr[1]), 3)], "frame": m.header.frame_id}
            self._publish(ts, rx, ry, yaw, self.min_pts, np.array([[hx, hy]]), f"TFmini {m.range:.2f} m", "/tfmini")

    def _tf_clear(self, ts):
        if self.tf_hit or "/tfmini" in self.src:
            self.tf_hit = 0
            self.src.pop("/tfmini", None)

    def _publish(self, ts, rx, ry, yaw, n, pts, why, src):
        self.src[src] = (ts, bool(why), int(n), pts, why)
        live = {k: v for k, v in self.src.items() if ts - v[0] <= 0.35}   # 0.35 s 넘게 소식 없는 소스는 뺌 (라이다 10 Hz)
        self.src = live
        blocked = any(v[1] for v in live.values())
        n = max((v[2] for v in live.values()), default=0)
        b = [v for v in live.values() if v[1]]
        why = " · ".join(f"{v[4]}" for v in b)
        pts = np.vstack([v[3] for v in live.values() if len(v[3])]) if any(len(v[3]) for v in live.values()) else np.zeros((0, 2))
        self.pub_b.publish(Bool(data=blocked))
        nears = [U for (tw, U) in self.last_pts.values() if abs(ts - tw) < 0.3 and len(U)]
        near = np.vstack(nears) if nears else np.zeros((0, 2))
        near = near[np.hypot(near[:, 0] - rx, near[:, 1] - ry) <= self.near_r] if len(near) else near
        st = {"t": round(ts, 3), "n": int(n), "blocked": blocked, "why": why, "src": src,
              "mode": "spin" if self.spin_c is not None else ("path" if self.path is not None else "off"),
              "robot": [round(rx, 3), round(ry, 3), round(yaw, 4)],
              "corr": [round(self.corr[0], 3), round(self.corr[1], 3), round(math.degrees(self.corr[2]), 2)],
              "tf_dbg": self.tf_dbg if src == "/tfmini" else None,
              "pts": np.round(pts[:60], 3).tolist(), "near": np.round(near[:400], 3).tolist()}
        self.pub_s.publish(String(data=json.dumps(st, ensure_ascii=False)))


def main():
    rclpy.init()
    n = Guard()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
