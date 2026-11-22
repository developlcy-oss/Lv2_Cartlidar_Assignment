#!/usr/bin/env python3
"""11-21 상자 앞면 측정 bag 회귀 (replay approach) — 실물 rosbag 의 정지 순간 /scan_front 로 box_face.find_box_face 를 다시 돌림.

쓰기: python3 replay_ap.py runs/1121_1800_*/bag runs/1121_1809_*/bag ...   (bag 디렉터리 또는 .mcap/.db3 파일)
  --n 5        정지 구간마다 쓸 스캔 장수 (ap_measure 와 같이 평균, 기본 5)
  --csv out    표를 CSV 로도 저장

무엇을 봄 (상자 구간 = /drive_demo/box_zone 이 이름을 낸 동안):
  · 정지 구간 = /odometry/filtered |v| < 5 mm/s · |w| < 0.01 rad/s 인 스캔이 3 장 이상 이어진 곳
  · 예상 앞면 = box_zone 상자 중심 (map) 을 그때 /amcl_pose 로 base_link 에 옮긴 뒤 x − 0.1025 (로봇 정면 기준 근사, ap_expect_goal 대용)
  · 세 방식: 예전 (expect ±0.10, 맞춤 여유 0 = 11-21 실물 때 동작) · 새 expect (±0.20 + 맞춤 여유) · 새 expect 없음 (정면 탐색)
  · 칸: 찾음 ✓/✗, 옆 [cm] (+ = 상자 중앙이 왼쪽), 틈 [cm], 기울기 [°], 예상 ↔ 측정 앞면 중점 차 [cm], 상자 영역 점 수
의존: rosbag2_py (+ rclpy) 가 있으면 그것 (mcap · sqlite3 모두). 없으면 .db3 (sqlite3) 만 직접 읽음 (CDR 최소 해석).
ROS 없이 import 되는 box_face.py 만 씀. 저장소 파일을 고치지 않음.
"""
from __future__ import annotations

import argparse
import bisect
import glob
import json
import math
import os
import sqlite3
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import box_face as bf  # noqa: E402

TOPICS = ("/drive_demo/box_zone", "/scan_front", "/odometry/filtered", "/amcl_pose")
OLD_CFG = {"expect_win": 0.10, "fit_margin": 0.0, "refine_adapt": 0.0, "x_max": 0.60, "y_abs": 0.30, "seg_per_cluster": 2}
STILL_V, STILL_W = 0.005, 0.01


# ---------------------------------------------------------------- CDR 최소 해석 (rosbag2_py 없을 때 sqlite3 용)
class _Cdr:
    """XCDR1 little endian (rosbag2 기본). 정렬은 4 바이트 캡슐 헤더 뒤부터"""

    def __init__(self, data):
        if data[1] not in (0x01, 0x03):
            raise ValueError("big endian CDR 은 안 읽음")
        self.b, self.o = data[4:], 0

    def _al(self, n):
        self.o += (-self.o) % n

    def u32(self):
        self._al(4)
        v = struct.unpack_from("<I", self.b, self.o)[0]
        self.o += 4
        return v

    def i32(self):
        self._al(4)
        v = struct.unpack_from("<i", self.b, self.o)[0]
        self.o += 4
        return v

    def f32(self):
        self._al(4)
        v = struct.unpack_from("<f", self.b, self.o)[0]
        self.o += 4
        return v

    def f64(self, n=1):
        self._al(8)
        v = struct.unpack_from("<%dd" % n, self.b, self.o)
        self.o += 8 * n
        return v if n > 1 else v[0]

    def string(self):
        n = self.u32()
        s = self.b[self.o:self.o + n]
        self.o += n
        return s.rstrip(b"\0").decode("utf-8", "replace")

    def f32seq(self):
        n = self.u32()
        self._al(4)
        v = struct.unpack_from("<%df" % n, self.b, self.o)
        self.o += 4 * n
        return list(v)

    def header(self):
        sec, nsec = self.i32(), self.u32()
        return sec, nsec, self.string()


def cdr_laserscan(data):
    r = _Cdr(data)
    _, _, frame = r.header()
    amin, _amax, ainc, _ti, _st, rmin, rmax = (r.f32() for _ in range(7))
    return frame, amin, ainc, rmin, rmax, r.f32seq()


def cdr_odom_twist(data):
    r = _Cdr(data)
    r.header()
    r.string()                 # child_frame_id
    r.f64(7)                   # pose
    r.f64(36)                  # covariance
    tw = r.f64(6)              # linear xyz, angular xyz
    return tw[0], tw[5]


def cdr_pose_cov(data):
    r = _Cdr(data)
    r.header()
    x, y, _z, qx, qy, qz, qw = r.f64(7)
    return x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def cdr_string(data):
    return _Cdr(data).string()


# ---------------------------------------------------------------- bag 읽기
def _bag_files(path):
    if os.path.isdir(path):
        fs = sorted(glob.glob(os.path.join(path, "*.mcap")) + glob.glob(os.path.join(path, "*.db3")))
        return path, fs
    return os.path.dirname(path), [path]


def read_bag(path):
    """→ dict(zones=[{name, cx, cy, t0, t1}], scans=[(t, frame, amin, ainc, rmin, rmax, ranges)], odom=[(t, v, w)], amcl=[(t, x, y, yaw)]), 읽은 방법"""
    out = {"scans": [], "odom": [], "amcl": [], "zone_msgs": []}
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        how = "rosbag2_py"
    except ImportError:
        rosbag2_py = None
        how = "sqlite3 직접"
    root, files = _bag_files(path)
    if rosbag2_py is not None:
        sid = "mcap" if any(f.endswith(".mcap") for f in files) else "sqlite3"
        uri = root if os.path.isdir(path) else path
        rd = rosbag2_py.SequentialReader()
        rd.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid), rosbag2_py.ConverterOptions("", ""))
        types = {t.name: t.type for t in rd.get_all_topics_and_types()}
        rd.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
        cls = {t: get_message(types[t]) for t in TOPICS if t in types}
        while rd.has_next():
            tp, data, t = rd.read_next()
            m = deserialize_message(data, cls[tp])
            ts = t / 1e9
            if tp == "/scan_front":
                out["scans"].append((ts, m.header.frame_id, m.angle_min, m.angle_increment, m.range_min, m.range_max,
                                     list(m.ranges)))
            elif tp == "/odometry/filtered":
                out["odom"].append((ts, m.twist.twist.linear.x, m.twist.twist.angular.z))
            elif tp == "/amcl_pose":
                q = m.pose.pose.orientation
                out["amcl"].append((ts, m.pose.pose.position.x, m.pose.pose.position.y,
                                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
            else:
                out["zone_msgs"].append((ts, m.data))
    else:
        dbs = [f for f in files if f.endswith(".db3")]
        if not dbs:
            raise SystemExit(f"rosbag2_py 없음 — .mcap ({path}) 은 직접 못 읽음 (ROS 환경을 source 하거나 .db3 로 변환)")
        for db in dbs:
            con = sqlite3.connect(db)
            tid = {name: i for i, name in con.execute("SELECT id, name FROM topics")}
            q = "SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN (%s) ORDER BY timestamp" % \
                ",".join(str(tid[t]) for t in TOPICS if t in tid)
            inv = {v: k for k, v in tid.items()}
            for i, t, data in con.execute(q):
                tp, ts = inv[i], t / 1e9
                if tp == "/scan_front":
                    out["scans"].append((ts,) + cdr_laserscan(data))
                elif tp == "/odometry/filtered":
                    out["odom"].append((ts,) + cdr_odom_twist(data))
                elif tp == "/amcl_pose":
                    out["amcl"].append((ts,) + cdr_pose_cov(data))
                else:
                    out["zone_msgs"].append((ts, cdr_string(data)))
            con.close()
    zones, cur = [], None
    for ts, s in out.pop("zone_msgs"):
        if s and cur is None:
            cur = dict(json.loads(s), t0=ts)
        elif not s and cur is not None:
            cur["t1"] = ts
            zones.append(cur)
            cur = None
    if cur is not None:
        cur["t1"] = out["scans"][-1][0] if out["scans"] else cur["t0"]
        zones.append(cur)
    out["zones"] = zones
    return out, how


# ---------------------------------------------------------------- 회귀
def still_blocks(D, z, min_len=3):
    ot = [o[0] for o in D["odom"]]

    def still(t):
        i = bisect.bisect(ot, t) - 1
        if i < 0:
            return False
        return abs(D["odom"][i][1]) < STILL_V and abs(D["odom"][i][2]) < STILL_W

    blocks, cur = [], []
    for s in D["scans"]:
        if not (z["t0"] <= s[0] <= z["t1"]):
            continue
        if still(s[0]):
            cur.append(s)
        elif cur:
            blocks.append(cur)
            cur = []
    if cur:
        blocks.append(cur)
    return [b for b in blocks if len(b) >= min_len]


def expect_face(D, z, t):
    """box_zone 상자 중심 → 그 순간 AMCL 로 base_link → 로봇 쪽 앞면 근사 (x − half)"""
    at = [a[0] for a in D["amcl"]]
    i = bisect.bisect(at, t) - 1
    if i < 0:
        return None
    _, x, y, th = D["amcl"][i]
    dx, dy = z["cx"] - x, z["cy"] - y
    c, s = math.cos(th), math.sin(th)
    bx, by = c * dx + s * dy, -s * dx + c * dy
    return bx - z.get("half", bf.BOX_SIDE / 2), by, 0.0


def measure(scans, expect, cfg):
    faces = []
    for s in scans:
        pts = bf.scan_to_base(s[6], s[2], s[3], s[4], s[5])
        faces.append(bf.find_box_face(pts, expect=expect, cfg=cfg))
    return bf.average_faces(faces), faces


def box_region_pts(scans, ex):
    """예상 앞면 주변 (법선 −0.05 ~ +0.30, 접선 ±0.20) 점 수 중앙값 — 상자가 라이다에 얼마나 보이는지"""
    ns = []
    for s in scans:
        n = 0
        for x, y in bf.scan_to_base(s[6], s[2], s[3], s[4], s[5]):
            if x >= 0.14 and -0.05 <= x - ex[0] <= 0.30 and abs(y - ex[1]) <= 0.20:
                n += 1
        ns.append(n)
    ns.sort()
    return ns[len(ns) // 2] if ns else 0


def _cell(f, ex):
    if not f.ok:
        return "✗", "", "", "", ""
    d = math.hypot(f.cx - ex[0], f.cy - ex[1]) if ex else float("nan")
    return "✓", f"{f.lateral * 100:+.1f}", f"{f.gap * 100:.1f}", f"{math.degrees(f.yaw_err):+.1f}", f"{d * 100:.1f}"


def run(paths, n, csv_path=None):
    rows, summ = [], {}
    for p in paths:
        D, how = read_bag(p)
        tag = os.path.basename(os.path.dirname(os.path.abspath(p)) if os.path.basename(p) == "bag" else p)
        print(f"\n### {tag}  ({how}, 스캔 {len(D['scans'])} · 상자 구간 {len(D['zones'])})")
        print(f"{'상자':5} {'t[s]':>5} {'장':>2} {'예상(x,y)':>14} {'점':>3} | {'예전':2} | {'새expect':8} {'옆':>6} {'틈':>5} {'°':>5} {'차':>5} |"
              f" {'새None':6} {'옆':>6} {'틈':>5} {'°':>5} | 새 실패 이유")
        for z in D["zones"]:
            bl = still_blocks(D, z)
            if not bl:
                print(f"{z['name']:5}  정지 구간 없음")
            for b in bl:
                mid = b[len(b) // 2]
                ex = expect_face(D, z, mid[0])
                use = b[:n] if len(b) <= n else b[len(b) // 2 - n // 2: len(b) // 2 - n // 2 + n]
                old, _ = measure(use, ex, OLD_CFG)
                new, nf = measure(use, ex, None)
                non, nn = measure(use, None, None)
                npts = box_region_pts(use, ex) if ex else 0
                c_old, c_new, c_non = _cell(old, ex), _cell(new, ex), _cell(non, ex)
                why = "" if new.ok else (next((f.why for f in nf if not f.ok), new.why))[:60]
                exs = f"({ex[0]:.2f},{ex[1]:+.2f})" if ex else "-"
                print(f"{z['name']:5} {mid[0] - z['t0']:5.1f} {len(use):2d} {exs:>14} {npts:3d} | {c_old[0]:2} | {c_new[0]:8} {c_new[1]:>6} {c_new[2]:>5}"
                      f" {c_new[3]:>5} {c_new[4]:>5} | {c_non[0]:6} {c_non[1]:>6} {c_non[2]:>5} {c_non[3]:>5} | {why}")
                rows.append(dict(bag=tag, box=z["name"], t=round(mid[0] - z["t0"], 1), n=len(use),
                                 ex_x=ex[0] if ex else None, ex_y=ex[1] if ex else None, region_pts=npts,
                                 old_ok=old.ok, new_ok=new.ok, new_lat_cm=c_new[1], new_gap_cm=c_new[2], new_yaw=c_new[3],
                                 new_d_cm=c_new[4], none_ok=non.ok, none_lat_cm=c_non[1], none_gap_cm=c_non[2], why=why))
                s = summ.setdefault(tag, [0, 0, 0, 0])
                s[0] += 1
                s[1] += old.ok
                s[2] += new.ok
                s[3] += non.ok
    print("\n### 보이는 정도: 예상 앞면 x 구간별 상자 영역 점 수 중앙값 · 새 expect 찾음")
    bins = [(0.0, 0.22), (0.22, 0.30), (0.30, 0.40), (0.40, 0.50), (0.50, 9.0)]
    for lo, hi in bins:
        rr = [r for r in rows if r["ex_x"] is not None and lo <= r["ex_x"] < hi]
        if rr:
            ps = sorted(r["region_pts"] for r in rr)
            print(f"  앞면 x {lo:.2f}~{hi:.2f} m (틈 ≈ {max(lo - bf.FRONT_OFFSET, 0):.2f}~{hi - bf.FRONT_OFFSET:.2f}): "
                  f"{len(rr)} 곳 · 점 중앙 {ps[len(ps) // 2]} · 찾음 {sum(r['new_ok'] for r in rr)}")
    print("\n### 요약 (정지 구간 수 · 예전 / 새 expect / 새 None 찾음)")
    for tag, (k, a, b, c) in summ.items():
        print(f"  {tag}: {k} 곳 · 예전 {a} · 새 expect {b} · 새 None {c}")
    if csv_path:
        import csv
        with open(csv_path, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()) if rows else ["bag"])
            w.writeheader()
            w.writerows(rows)
        print(f"CSV: {csv_path}")
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--n", type=int, default=5)
    ap.add_argument("--csv")
    a = ap.parse_args(argv)
    run(a.bags, a.n, a.csv)


if __name__ == "__main__":
    main()
