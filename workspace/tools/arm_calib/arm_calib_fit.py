#!/usr/bin/env python3
"""팔 관절 영점 보정 맞추기 — 실측 자세 CSV → `control/config/manipulator.yaml` `ik.joint_offsets`.

입력 CSV (팔 벤치 `tools/arm_bench/real/meas_session.py` 메뉴 10 · 4 · 12 가 남기는 `6_보정자세.csv` 와 같은 꼴):
    이름, j1, j2, j3, j4, j5, x, y, z       (j = 그때 읽은 관절 [도, 모터 값 그대로], x · y = 관절 1 축 점에서 앞 · 왼 [mm], z = 바닥 위 [mm], 못 잰 칸은 비움)
  - 잰 점: 이름이 `칸` · `상자` 로 시작하면 **잡는 점** (gripper_roll_link 앞 0.140 m, 퍽 중심 + 15 mm), 아니면 **손끝** (두 손가락 끝 가운데).
    열 `점` (tcp | tip | 숫자 m) 이 있으면 그것을 씀.
  - 열 `쓰임` 이 `확인` 이면 맞추는 데 안 쓰고 확인만.

모델: 실제 관절 각 = 읽은 각 − e_j (− k_j · L_j, `--sag`) → 팀 URDF FK → 잰 점.
  e_j = 관절 영점 오차 → 노드 명령 = 모델 각 + e_j 이므로 **ik.joint_offsets = e** (관절 5 는 위치에 영향 없음 → 0).
  `--sag 2,3`: 관절 2 · 3 의 처짐 (토크 ∝ 잡는 점까지 수평 거리 L) 도 맞춤 — 자세마다 다른 처짐을 따로 떼어 냄.
  `--z0`: 바닥 높이 (base_link 지면 위, `--base_h`) 오차를 상수로 같이 맞춤.

출력: 자세별 오차 (보정 전 · 지금 manipulator.yaml 값 · 새 값), 하나씩 빼고 맞춘 확인 오차, yaml 한 줄,
      놓기 궤적 (pallet_place_traj.json) 칸 놓는 자세 · 상자 위 집기 자세에서 지금 값 ↔ 새 값 끝 위치 차이.

  python3 arm_calib_fit.py 6_보정자세.csv [--joints 1,2,3,4] [--sag 2,3] [--z0] [--base_h 7.25]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.normpath(os.path.join(HERE, "..", ".."))
URDF = os.path.join(WS, "urdf_and_usd", "waffle_4wd_description", "urdf", "waffle_4wd.urdf")
YAML = os.path.join(WS, "ros2_ws", "src", "control", "config", "manipulator.yaml")
TRAJ = os.path.join(WS, "urdf_and_usd", "policy", "manipulator", "pallet_place_traj.json")
ARM = ["joint1", "joint2", "joint3", "joint4", "joint5"]
TCP_X = 0.140   # 잡는 점 (pallet_traj.py TCP, meas_session.fk 와 같음)


# ---------------------------------------------------------------- FK (팀 URDF)
def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axang(a, t):
    a = np.asarray(a, float) / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


class Arm:
    """base_link → gripper_roll_link 사슬 (관절 1 ~ 5). 점 = gripper_roll_link 앞 x (m)."""

    def __init__(self, urdf=URDF):
        root = ET.parse(urdf).getroot()
        J = {j.find("child").get("link"): j for j in root.findall("joint")}
        chain, link = [], "gripper_roll_link"
        while link in J:
            chain.append(J[link]); link = J[link].find("parent").get("link")
        self.chain = []
        for j in reversed(chain):
            o = j.find("origin")
            xyz = np.array([float(v) for v in o.get("xyz").split()])
            R = _rpy(*[float(v) for v in (o.get("rpy") or "0 0 0").split()])
            ax = j.find("axis")
            self.chain.append((j.get("name"), xyz, R, None if ax is None else np.array([float(v) for v in ax.get("xyz").split()])))
        names = [c[0] for c in self.chain]
        assert all(n in names for n in ARM), f"URDF 팔 관절 없음: {names}"
        # 손끝 = 손가락 충돌 상자의 가장 앞 (gripper_roll_link x)
        tip = 0.0
        for side in ("gripper_left_joint",):
            jj = next(j for j in root.findall("joint") if j.get("name") == side)
            x0 = float(jj.find("origin").get("xyz").split()[0])
            for c in root.find(f"link[@name='{jj.find('child').get('link')}']").findall("collision"):
                b = c.find("geometry/box")
                if b is not None:
                    tip = max(tip, x0 + float(c.find("origin").get("xyz").split()[0]) + float(b.get("size").split()[0]) / 2)
        self.tip_x = round(tip, 4)
        self.j2 = None

    def frames(self, q):
        """관절 1 ~ 5 각 (rad) → [(관절 이름, 원점, 회전)] (관절 축이 돌기 전 자리) 와 gripper_roll_link (p, R)."""
        p, R, out = np.zeros(3), np.eye(3), {}
        for name, xyz, R0, ax in self.chain:
            p = p + R @ xyz; R = R @ R0
            out[name] = (p.copy(), R.copy())
            if name in ARM:
                R = R @ _axang(ax, q[ARM.index(name)])
        return out, p, R

    def point(self, q, x):
        _, p, R = self.frames(q)
        return p + R @ np.array([x, 0.0, 0.0])


# ---------------------------------------------------------------- 자료
def read_csv(path, arm, args):
    rows = []
    for r in csv.DictReader(open(path, encoding="utf-8-sig")):
        name = (r.get("이름") or "").strip()
        if not name:
            continue
        q = [math.radians(float(r[f"j{i}"])) for i in range(1, 6)]
        meas = [None if (r.get(k) or "").strip() == "" else float(r[k]) for k in ("x", "y", "z")]
        if all(m is None for m in meas):
            continue
        pt = (r.get("점") or "").strip()
        if pt == "tcp" or (not pt and re.match(r"^(칸|상자)", name)):
            px = TCP_X
        elif pt in ("", "tip"):
            px = args.tip if args.tip else arm.tip_x
        else:
            px = float(pt)
        use = (r.get("쓰임") or "").strip() != "확인"
        rows.append(dict(name=name, q=np.array(q), meas=meas, px=px, use=use))
    if not rows:
        raise SystemExit(f"{path}: 쓸 줄 없음")
    return rows


def to_mm(p_base, base_h_mm):
    """base_link (m) → CSV 좌표 (관절 1 축 점 기준 x · y, 바닥 위 z, mm)"""
    return np.array([(p_base[0] - 0.096) * 1000, p_base[1] * 1000, p_base[2] * 1000 + base_h_mm])


# ---------------------------------------------------------------- 맞추기
class Model:
    def __init__(self, arm, joints, sag, z0, base_h):
        self.arm, self.joints, self.sag, self.z0, self.base_h = arm, joints, sag, z0, base_h
        self.n = len(joints) + len(sag) + (1 if z0 else 0)

    def unpack(self, x):
        e = np.zeros(5); k = np.zeros(5)
        for i, j in enumerate(self.joints):
            e[j - 1] = x[i]
        for i, j in enumerate(self.sag):
            k[j - 1] = x[len(self.joints) + i]
        dz = x[-1] if self.z0 else 0.0
        return e, k, dz

    def lever(self, q, j):
        """관절 j 축에서 잡는 점까지 수평 거리 (m) — 처짐 토크 비례"""
        fr, p, R = self.arm.frames(q)
        tcp = p + R @ np.array([TCP_X, 0, 0])
        o = fr[ARM[j - 1]][0]
        return math.hypot(tcp[0] - o[0], tcp[1] - o[1])

    def real_q(self, q_read, e, k):
        q = q_read - e
        for j in self.sag:
            q[j - 1] -= k[j - 1] * self.lever(q_read - e, j)
        return q

    def predict(self, row, x):
        e, k, dz = self.unpack(x)
        return to_mm(self.arm.point(self.real_q(row["q"], e, k), row["px"]), self.base_h + dz)

    def resid(self, x, rows):
        out = []
        for r in rows:
            d = self.predict(r, x) - np.array([m if m is not None else 0.0 for m in r["meas"]])
            out += [d[i] for i in range(3) if r["meas"][i] is not None]
        return np.array(out)

    def fit(self, rows):
        return least_squares(self.resid, np.zeros(self.n), args=(rows,), method="lm" if sum(
            sum(m is not None for m in r["meas"]) for r in rows) > self.n else "trf").x


def err3(model, r, x):
    d = model.predict(r, x) - np.array([m if m is not None else np.nan for m in r["meas"]])
    return d, float(np.sqrt(np.nansum(d ** 2)))


def current_offsets():
    try:
        t = open(YAML, encoding="utf-8").read()
        m = re.search(r"joint_offsets:\s*\[([^\]]+)\]", t)
        return np.array([float(v) for v in m.group(1).split(",")])
    except Exception:
        return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--joints", default="1,2,3,4", help="맞출 관절 영점 (기본 1,2,3,4. 관절 5 는 끝 위치에 영향 없음)")
    ap.add_argument("--sag", default="", help="처짐도 맞출 관절 (예 2,3). 자세 10 개 이상일 때")
    ap.add_argument("--z0", action="store_true", help="바닥 높이 오차를 상수로 같이 맞춤")
    ap.add_argument("--base_h", type=float, default=7.25, help="base_link 지면 위 [mm] (바퀴 반지름 0.0335 모델 7.25, 실측값이 있으면 그것)")
    ap.add_argument("--tip", type=float, default=None, help="손끝 x [m, gripper_roll_link] (기본 URDF 손가락 앞 끝)")
    ap.add_argument("--urdf", default=URDF)
    ap.add_argument("--traj", default=TRAJ)
    ap.add_argument("--json", default=None, help="결과를 JSON 으로도 저장")
    a = ap.parse_args(argv)
    arm = Arm(a.urdf)
    joints = [int(v) for v in a.joints.split(",") if v]
    sag = [int(v) for v in a.sag.split(",") if v]
    rows = read_csv(a.csv, arm, a)
    fit_rows = [r for r in rows if r["use"]]
    model = Model(arm, joints, sag, a.z0, a.base_h)
    nobs = sum(sum(m is not None for m in r["meas"]) for r in fit_rows)
    print(f"자세 {len(rows)} 개 (맞춤 {len(fit_rows)} · 확인 {len(rows) - len(fit_rows)}), 잰 값 {nobs} 개, 맞출 값 {model.n} 개"
          f" | 잡는 점 x {TCP_X} · 손끝 x {a.tip or arm.tip_x} m, 바닥 높이 {a.base_h} mm")
    if nobs < model.n + 3:
        raise SystemExit("잰 값이 너무 적음 (맞출 값 + 3 개 이상 필요)")
    x = model.fit(fit_rows)
    e, k, dz = model.unpack(x)
    cur = current_offsets()
    x_cur = None
    if cur is not None:   # 지금 manipulator.yaml 값으로 본 오차 (영점만, 처짐 없음)
        mc = Model(arm, [1, 2, 3, 4, 5], [], False, a.base_h)
        x_cur = cur.copy()

    print("\n| 자세 | 쓰임 | 점 | 보정 없음 [mm] | 지금 yaml [mm] | 새 값 [mm] | 새 값 dx dy dz |\n|---|---|---|---|---|---|---|")
    m0 = Model(arm, [], [], False, a.base_h)
    stats = {"none": [], "cur": [], "new": [], "new_check": []}
    for r in rows:
        _, en = err3(m0, r, np.zeros(0))
        ec = err3(mc, r, x_cur)[1] if x_cur is not None else float("nan")
        d, ew = err3(model, r, x)
        stats["none"].append(en); stats["cur"].append(ec); stats["new" if r["use"] else "new_check"].append(ew)
        print(f"| {r['name']} | {'맞춤' if r['use'] else '확인'} | {'잡는 점' if r['px'] == TCP_X else '손끝'} | {en:.1f} | {ec:.1f} | {ew:.1f} | "
              + " ".join("-" if np.isnan(v) else f"{v:+.1f}" for v in d) + " |")
    rms = lambda v: float(np.sqrt(np.mean(np.square(v)))) if v else float("nan")
    print(f"\nRMS: 보정 없음 {rms(stats['none']):.1f} · 지금 yaml {rms(stats['cur']):.1f} · 새 값 (맞춘 자세) {rms(stats['new']):.1f}"
          + (f" · 새 값 (확인 자세) {rms(stats['new_check']):.1f}" if stats["new_check"] else "") + " mm")

    # 하나씩 빼고 맞춘 확인 오차 (자료가 적을 때 과적합 보기)
    loo = []
    if len(fit_rows) >= model.n // 2 + 3:
        for i, r in enumerate(fit_rows):
            rest = fit_rows[:i] + fit_rows[i + 1:]
            if sum(sum(m is not None for m in t["meas"]) for t in rest) < model.n + 2:
                continue
            loo.append(err3(model, r, model.fit(rest))[1])
        print(f"하나씩 빼고 맞춘 오차: RMS {rms(loo):.1f} · 최대 {max(loo):.1f} mm ({len(loo)} 자세) — 맞춘 자세 RMS 보다 많이 크면 자세를 더 재야 함")

    e_out = e.copy()
    print("\n새 영점 오차 e (도): " + " ".join(f"j{i + 1} {math.degrees(v):+.2f}" for i, v in enumerate(e))
          + ("" if cur is None else "  | 지금 yaml (도): " + " ".join(f"{math.degrees(v):+.2f}" for v in cur)))
    if sag:
        print("처짐 k (도 / 100 mm 팔 길이): " + " ".join(f"j{j} {math.degrees(k[j - 1]) * 0.1:+.2f}" for j in sag))
    if a.z0:
        print(f"바닥 높이 보정 {dz:+.1f} mm (base_h {a.base_h} → {a.base_h + dz:.1f})")

    # 놓기 궤적 · 상자 위 자세에서 지금 값 ↔ 새 값 (처짐 포함) 끝 차이 → 처짐을 넣었으면 그 자세들의 평균 실효 영점을 권함
    poses = []
    try:
        T = json.load(open(a.traj, encoding="utf-8"))
        for s in T["slots"]:
            seg = next(g for g in s["segments"] if g["name"] == "place")
            poses.append((f"칸 {s['row']}:{s['index']} 놓기", np.array(seg["q"][-1][:5])))
    except Exception as ex:
        print(f"(궤적 {a.traj} 못 읽음: {ex})")
    for nm, qd in (("상자 위 집기 (P9)", [0, -30, 50, 50, 0]),):   # 미션 자세만 (평균 실효 영점에 들어감)
        poses.append((nm, np.radians(qd)))
    if poses:
        def real(qm, off):   # 명령 = 모델 + offset, 실제 = 명령 − e − k·L
            return arm.point(model.real_q(qm + off, e, k), TCP_X)
        if sag:   # 자세마다 처짐까지 지우는 영점 = e + k·L → yaml 에는 하나만 들어가므로 이 자세들의 평균 (실효 영점)
            eff = []
            for _, qm in poses:
                e_eff = e.copy()
                for j in sag:
                    e_eff[j - 1] += k[j - 1] * model.lever(qm, j)
                eff.append(e_eff)
            e_out = np.mean(eff, axis=0)
        print("\n| 자세 (모델 각) | 지금 yaml 로 명령하면 끝 오차 [mm] | 새 값으로 [mm] |\n|---|---|---|")
        for nm, qm in poses:
            target = arm.point(qm, TCP_X)
            ecur = np.linalg.norm(real(qm, cur) - target) * 1000 if cur is not None else float("nan")
            enew = np.linalg.norm(real(qm, e_out) - target) * 1000
            print(f"| {nm} | {ecur:.1f} | {enew:.1f} |")
        if sag:
            print("처짐을 넣어 맞췄으므로 yaml 값 = 위 자세들의 평균 실효 영점. 남는 오차는 자세마다 다른 처짐 (줄이려면 놓기 처짐 보정 스트리밍 `pallet.ki`)")
    e_out[4] = 0.0 if 5 not in joints else e_out[4]
    line = "joint_offsets: [" + ", ".join(f"{v:.4f}" for v in e_out) + "]"
    print(f"\n→ {os.path.relpath(YAML, WS)} `ik.{line}`")
    if a.json:
        json.dump(dict(e_rad=e.tolist(), k=k.tolist(), dz_mm=dz, joint_offsets=e_out.tolist(), rms=dict(none=rms(stats["none"]), cur=rms(stats["cur"]),
                       new=rms(stats["new"]), check=rms(stats["new_check"]), loo=rms(loo)), n_rows=len(rows)), open(a.json, "w"), indent=1)
    return e_out


if __name__ == "__main__":
    sys.exit(0 if main() is not None else 1)
