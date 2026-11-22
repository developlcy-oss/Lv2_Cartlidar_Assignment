"""팔레트 (퍽 스토리지) 수직 넣기 궤적 — 처음부터 끝까지 집게 거의 수직 (2026-11-22, 사용자: 45° 쓰지 말고 수직으로, 부딪히지 않게).

pallet_traj_direct.py --vertical 판의 문제 (실물):
  - 45° · 55° 로 옮기는 자세 (반경 0.10 · 퍽 바닥 0.23) 가 팔 윗부분을 뒤로 눕혀 (관절 2 명령 −110°) 3 번 관절이 스토리지에 박힘.
    그 생성기는 쥔 퍽만 검사하고 팔 몸체는 검사하지 않음
이 판:
  rotate      : 높은 자세 HIGH_V (meas_session HIGH 관절 2 · 3 = 손 맞춤 · 측정 도구에서 실물로 수십 번 돈 자세) 에서 관절 1 만.
                관절 4 만 바꿔 집게를 넣을 때 기울기 (5 ~ 15°) 로 세움 -> 퍽은 처음부터 수직에 가깝게 쥔 채
  carry       : HIGH_V -> 칸 입구 위 (above) 관절 공간 직선
  place       : above -> 입구 -> 칸 바닥 (칸 축 수직 직선, 2 mm 간격 IK, 기울기 그대로)
  release · retract (place 거꾸로) · return (carry 거꾸로) · rotate_back
모델: URDF + 관절 영점 보정 (11-14 45° 손 맞춤 10 칸, J2 −29.04° · J4 +22.89°) + 칸 목표 이동 (11-22 실물 1_3: 바깥 5.2 · 옆 −3.2 · 위 −4.0 mm).
관절 4 명령 상한 2.21 rad (실물 약 2.24 rad 에서 기구적으로 막힘).
검사 (하나라도 걸리면 그 칸은 만들지 않음, 0 이 아니면 종료 코드 1):
  - 팔 몸체: 관절 2 -> 3 -> 4 -> 5 링크 중심선 위 점 (각 5 등분) 이 스토리지 위 (J1 에서 반경 0.06 ~ 0.25, 뒤쪽 |yaw| ≥ 100°) 면 높이 ≥ ARM_MIN_Z
  - 쥔 퍽 (바닥 · 가운데 · 윗면): 다른 칸 (꽉 찬 팔레트 가정) 퍽 기둥 안이면 그 퍽 윗면 + 5 mm 위, 자기 칸 벽 윗면 아래면 칸 축에서 4 mm 안
  - 경유점 사이는 관절 공간 직선 10 등분으로 같이 검사. 집고 든 자세 (11-22 실물) -> 출발 자세도 검사
  (11-22 실물에서 부딪힌 PR 360 궤적은 스토리지 위 링크 최저 218 mm -> 이 검사에 걸림, 이 판은 296 mm 이상)
    python3 pallet_traj_vertical.py            # -> pallet_place_traj_vertical.json (+ control/config 사본) + 검사표
"""

from __future__ import annotations

import argparse
import json
import math
import os

import numpy as np

import pallet_traj as PT

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "pallet_place_traj.json")
OUT = os.path.join(HERE, "pallet_place_traj_vertical.json")
CONTROL = os.path.join(HERE, "..", "..", "..", "ros2_ws", "src", "control", "config", "pallet_place_traj_vertical.json")
JOFF = np.array([0.0, -0.50685, -0.00192, 0.39951, 0.0])
SHIFT_MM = (5.2, -3.2, -4.0)
J4_MAX = 2.21
HIGH_CMD = (-0.6875, -0.1959)          # meas_session HIGH 관절 2 · 3 (명령)
HAND_DZ = 0.010                         # tool_link = 퍽 중심 + 이만큼 (toppick)
D_AXIS = HAND_DZ + PT.PUCK_H / 2        # 손끝 -> 퍽 바닥 (집게 축)
ENTER_GAP = 0.002
Z_TOP = 0.200                           # 칸 바로 위에서 시작하는 퍽 바닥 높이 = 퍽 윗면 최대 0.192 + 8 mm
ROT_R, ROT_Z, ROT_TILT = 0.13, 0.21, 32.0   # 관절 1 을 돌리는 자세: 퍽 바닥 반경 0.13 · 높이 0.21 · 집게 32°
#   (11-22: 집게가 수직에 가까우면 관절 4 ≤ 2.21 로는 퍽 바닥을 0.17 ~ 0.18 위로 못 듦 -> 그 높이로 돌면 2 열 벽 · 퍽을 쓸음.
#    0.21 이면 기운 퍽 가장 낮은 모서리도 칸 퍽 윗면 최대 0.192 보다 1.6 cm 위)
MAX_TOP_TILT = 35.0
LIFT_CMD = np.radians([-4.83, -21.62, -14.94, 108.98, 0.0])   # 11-22 실물 집고 든 자세 (노드는 여기서 출발 자세로 바로 감, 그 길도 검사)                     # 입구 위에서 관절 4 때문에 기울일 수 있는 최대 (퍽 바닥은 칸 축 위)
STEP_DOWN = 0.002
ARM_MIN_Z = 0.250                       # 스토리지 위 팔 링크 중심선 최저 높이 = 퍽 윗면 최대 0.192 + 모터 몸통 반폭 · 여유 약 58 mm.
#   11-22 실물 충돌 자세 (관절 2 명령 −109.6°) 는 관절 3 원점 z 0.235 — 0.215 였으면 못 잡음 -> 0.250 (그 자세 걸림 확인)
STORE_RMIN, STORE_R = 0.06, 0.25        # 스토리지 = J1 에서 반경 0.06 ~ 0.25 · 뒤쪽 (각 |yaw| ≥ 100°, 칸은 116 ~ 168°)
STORE_YAW = math.radians(100.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--control", default=CONTROL)
    a = ap.parse_args()
    src = json.load(open(SRC, encoding="utf-8"))
    arm = PT.Arm(PT.URDF)
    j1 = arm.j1
    hi_cmd = arm.hi.copy()
    hi_cmd[3] = min(hi_cmd[3], J4_MAX)
    lo_cmd = arm.lo.copy()
    arm.lo, arm.hi = lo_cmd - JOFF, hi_cmd - JOFF          # ik_down 은 모델 관절로 풂
    d_model = PT.GRASP_DZ + PT.PUCK_H / 2
    S = src["slots"]
    tgt = {}
    for s in S:   # 칸 목표 (이동 반영)
        yaw = math.atan2(s["y"] - j1[1], s["x"] - j1[0])
        r = math.hypot(s["x"] - j1[0], s["y"] - j1[1]) + SHIFT_MM[0] / 1000.0
        yaw += SHIFT_MM[1] / 1000.0 / r
        tgt[f"{s['row']}_{s['index']}"] = dict(yaw=yaw, r=r, floor=s["floor_z"] + SHIFT_MM[2] / 1000.0,
                                             wall=s["wall_top_z"] + SHIFT_MM[2] / 1000.0, s=s)

    def at(r, yaw, z):
        return np.array([j1[0] + r * math.cos(yaw), j1[1] + r * math.sin(yaw), z])

    def solve(p_bottom, yaw, seed, tilt_deg):
        t = math.radians(tilt_deg)
        ax = np.array([math.sin(t) * math.cos(yaw), math.sin(t) * math.sin(yaw), -math.cos(t)])
        q, e, tt = arm.ik_down(p_bottom - ax * D_AXIS - np.array([0, 0, d_model]), yaw, seed, off=PT.BOTTOM, tilt=t)
        return np.array(q), (e < 1e-3 and tt < math.radians(1.0))

    def model(qc):
        return np.asarray(qc, float) - JOFF

    def points(qc):
        """명령 관절 -> 몸체 점 (관절 3 · 4 · 5 원점) · 퍽 점 (바닥 · 가운데 · 윗면)."""
        qm = model(qc)
        js = [arm.fk(qm, until=n)[:3, 3] for n in ("joint2", "joint3", "joint4", "joint5")]
        body = [a0 + (a1 - a0) * f for a0, a1 in zip(js[:-1], js[1:]) for f in np.linspace(0.0, 1.0, 6)[1:]]
        t, R = arm.tcp(qm)
        ax = R[:, 0]
        puck = [t + ax * D_AXIS, t + ax * (D_AXIS - PT.PUCK_H / 2), t + ax * (D_AXIS - PT.PUCK_H)]
        return body, puck, math.degrees(math.acos(max(-1.0, min(1.0, -R[2, 0]))))

    def check_q(qc, own, holding):
        body, puck, _ = points(qc)
        bad = []
        for i, b in enumerate(body):
            rb = math.hypot(b[0] - j1[0], b[1] - j1[1])
            over = STORE_RMIN <= rb <= STORE_R and abs(math.atan2(b[1] - j1[1], b[0] - j1[0])) >= STORE_YAW
            if over and b[2] < ARM_MIN_Z:
                bad.append(f"링크{i // 5 + 2}-{i // 5 + 3} z {b[2] * 1000:.0f}")
        if holding:
            o_own = tgt[own]
            ax_own = at(o_own["r"], o_own["yaw"], 0)[:2]
            _, _, tl_q = points(qc)
            for i_p, p in enumerate(puck):   # 바닥 · 가운데 · 윗면: 자기 칸 벽 아래면 축에서 4 mm + 기울기만큼 (바닥 0, 가운데 30 mm x sin)
                lim = 0.004 + (0.0, PT.PUCK_H / 2, PT.PUCK_H)[i_p] * math.sin(math.radians(tl_q)) + 0.0005
                d_own = np.hypot(*(p[:2] - ax_own))
                if p[2] < o_own["wall"] + 0.003 and lim < d_own < PT.CUP_HALF + PT.PUCK_R + 0.010:   # 자기 칸 근처에서만
                    bad.append(f"퍽 자기 칸 벽 아래 축에서 {np.hypot(*(p[:2] - ax_own)) * 1000:.1f} mm")
                for k, o in tgt.items():   # 빈 칸이라도 칸 벽: 칸 축에서 컵 반폭 + 퍽 반지름 안이면 벽 윗면 + 3 mm 위
                    if k == own and p[2] < o["wall"] + 0.003:
                        continue
                    ox = at(o["r"], o["yaw"], 0.0)[:2]   # 모델 오차 이동을 모든 칸에 같게
                    if np.hypot(*(p[:2] - ox)) < PT.CUP_HALF + PT.PUCK_R and p[2] < o["wall"] + 0.003:
                        bad.append(f"퍽 {k} 칸 벽")
                for k, o in tgt.items():
                    # 계획기 열 규칙 (앞 열에 퍽이 있으면 뒤 열에 못 실음) -> 넣을 때 앞 열 (열 번호 작음) 은 비어 있음
                    if k == own or o["s"]["row"] < tgt[own]["s"]["row"]:
                        continue
                    ox = at(o["r"], o["yaw"], 0.0)[:2]
                    if np.hypot(*(p[:2] - ox)) < 0.033 and p[2] < o["floor"] + PT.PUCK_H + 0.005:
                        bad.append(f"퍽 {k} 칸 퍽과 겹침")
        return bad

    def check_path(path, own, holding):
        bad = []
        for qa, qb in zip(path[:-1], path[1:]):
            for f in np.linspace(0.0, 1.0, 11)[1:]:
                bad += check_q(qa + (qb - qa) * f, own, holding)
        return sorted(set(bad))

    def high_v(yaw, tilt_deg):
        """HIGH 관절 2 · 3 그대로, 관절 4 를 바꿔 집게 기울기 = tilt_deg (안 되면 가장 가까이)."""
        best = None
        for j4 in np.linspace(1.6, J4_MAX, 62):
            qc = np.array([yaw, HIGH_CMD[0], HIGH_CMD[1], j4, 0.0])
            _, _, tl = points(qc)
            if best is None or abs(tl - tilt_deg) < best[0]:
                best = (abs(tl - tilt_deg), qc, tl)
        return best[1], best[2]

    out, report, fail = [], [], 0
    for s in S:
        k = f"{s['row']}_{s['index']}"
        T = tgt[k]
        yaw, r = T["yaw"], T["r"]
        z_p, z_in = T["floor"] + PT.PLACE_GAP, T["wall"] + ENTER_GAP
        tilt = None
        for td in range(5, 16):
            qb, ok1 = solve(at(r, yaw, z_p), yaw, None, td)
            qe, ok2 = solve(at(r, yaw, z_in), yaw, qb[1:4], td)
            if ok1 and ok2:
                tilt = td
                break
        if tilt is None:
            report.append(f"{k}: 입구 · 바닥을 15° 안으로 못 세움 -> 만들지 않음")
            fail += 1
            continue
        # 바닥 -> 입구는 기울기 그대로 (칸 축 수직 직선), 입구 위 -> Z_TOP 은 퍽 바닥을 칸 축에 둔 채 관절 4 가 허락하는 만큼만 기울임
        up, q, tl_now, z_top = [qb], qb, float(tilt), z_p
        zs = list(PT.lin(z_p, Z_TOP, STEP_DOWN))
        for zz in zs:
            q2, ok = solve(at(r, yaw, zz), yaw, q[1:4], tl_now)
            if not ok and zz > z_in + 1e-9:
                for td in np.arange(tl_now + 0.5, MAX_TOP_TILT + 0.01, 0.5):
                    q2, ok = solve(at(r, yaw, zz), yaw, q[1:4], td)
                    if ok:
                        tl_now = float(td)
                        break
            if not ok:
                if zz <= z_in + 0.010:
                    q = None
                break                             # 올릴 수 있는 만큼만 (그 위는 carry 가 높은 자세에서 바로 옴, 검사가 확인)
            up.append(q2)
            q = q2
            z_top = zz
        if q is None:
            report.append(f"{k}: 칸 입구 위 10 mm 도 안 풀림 (기울기 {tl_now:.1f}°) -> 만들지 않음")
            fail += 1
            continue
        place_m = up[::-1]                                  # 위 -> 바닥 (모델)
        place = [qq + JOFF for qq in place_m]               # 명령
        above_h = z_top - z_in
        hv0, ok0 = solve(at(ROT_R, 0.0, ROT_Z), 0.0, None, ROT_TILT)
        hv, ok1 = solve(at(ROT_R, yaw, ROT_Z), yaw, hv0[1:4], ROT_TILT)
        if not (ok0 and ok1):
            report.append(f"{k}: 회전 자세 안 풀림 -> 만들지 않음")
            fail += 1
            continue
        hv0, hv = hv0 + JOFF, hv + JOFF
        tl0 = ROT_TILT
        rot = [np.array([th, *hv[1:4], 0.0]) for th in [0.0] + list(PT.lin(0.0, yaw, 0.05))]
        # carry: 높은 자세의 퍽 바닥 높이 그대로 칸 축 위까지 (반경만) -> 칸 축 따라 z_top 까지 내림 (같은 yaw 평면, 5 mm 간격 IK)
        _, pk_hv, _ = points(hv)
        b0 = pk_hv[0]
        r0 = (b0[0] - j1[0]) * math.cos(yaw) + (b0[1] - j1[1]) * math.sin(yaw)
        z0 = max(b0[2], z_top)   # = ROT_Z
        if r >= r0:   # 2 · 3 열: 회전 높이 그대로 칸 축 위까지 -> 칸 축 따라 내림
            legs = [(rr, z0) for rr in PT.lin(r0, r, 0.005)] + [(r, zz) for zz in PT.lin(z0, z_top, 0.005)]
        else:         # 1 열: 안쪽으로 오며 비스듬히 내림 (2 열 퍽 위인지는 검사가 봄)
            n = max(2, int(math.ceil(max(abs(r0 - r), abs(z0 - z_top)) / 0.005)))
            legs = [(r0 + (r - r0) * i / n, z0 + (z_top - z0) * i / n) for i in range(1, n + 1)]
        carry_m, qq_m, ok_c = [], model(hv), True
        for rr, zz in legs:
            for td in np.arange(float(tilt), 45.01, 0.5):
                q2, ok = solve(at(rr, yaw, zz), yaw, qq_m[1:4], td)
                if ok:
                    break
            if not ok:
                ok_c = False
                break
            carry_m.append(q2)
            qq_m = q2
        if not ok_c:
            report.append(f"{k}: 높은 자세 -> 칸 위 경로가 안 풀림 (반경 {rr * 1000:.0f} · 높이 {zz * 1000:.0f} mm) -> 만들지 않음")
            fail += 1
            continue
        carry = [qq + JOFF for qq in carry_m]
        hold_path = [np.array([LIFT_CMD[0] * 0 + rot[0][0], *LIFT_CMD[1:4], 0.0])] + rot + carry + place
        bad = check_path([np.asarray(x, float) for x in hold_path], k, True)
        bad += check_path([np.asarray(x, float) for x in (place[::-1] + carry[::-1] + rot[::-1])], k, False)
        j4max = max(x[3] for x in hold_path)
        allq = np.array(hold_path)
        margin = float(min((allq - lo_cmd).min(), (hi_cmd - allq).min())) + PT.LIMIT_MARGIN   # hi/lo 는 이미 여유 빼 둔 값
        line = (f"{k}: 입구 ~ 바닥 집게 {tilt}° · 칸 위 퍽 바닥 {z_top * 1000:.0f} mm (입구 +{above_h * 1000:.0f}) 에서 {tl_now:.1f}° · 높은 자세 기울기 {tl0:.1f}° · "
                f"관절 4 최대 {math.degrees(j4max):.1f}° · 관절 한계 여유 {margin:+.3f} rad")
        if bad:
            line += " | !! 검사 걸림: " + ", ".join(bad[:4])
            fail += 1
        report.append(line)
        if bad or margin < 0:
            continue
        R5 = lambda L: [[round(float(v), 5) for v in x] for x in L]
        segs = [dict(name="rotate", gripper="hold", q=R5(rot[1:])), dict(name="carry", gripper="hold", q=R5(carry)),
                dict(name="place", gripper="hold", q=R5(place)), dict(name="release", gripper="open", q=R5(place[-1:])),
                dict(name="retract", gripper="open", q=R5(place[::-1][1:])), dict(name="return", gripper="open", q=R5(carry[::-1][1:] + [hv])),
                dict(name="rotate_back", gripper="open", q=R5(rot[::-1][1:]))]
        o = {kk: v for kk, v in s.items() if kk not in ("segments", "direct_path")}
        o.update(q_front=R5([hv0])[0], tilt_deg=float(tilt), enter_tilt_deg=float(tilt), segments=segs, vertical_high=True,
                 target=dict(yaw=round(yaw, 5), r=round(r, 5), floor_z=round(T["floor"], 5), wall_top_z=round(T["wall"], 5)))
        out.append(o)
    for l in report:
        print(l)
    if fail:
        print(f"!! {fail} 칸 실패 -> 파일 안 씀")
        raise SystemExit(1)
    hv0 = np.array(out[0]["q_front"])
    doc = {kk: v for kk, v in src.items() if kk != "slots"}
    doc["note"] = ("2026-11-22 수직 넣기 (pallet_traj_vertical.py, 직접 고치지 말 것): 높은 자세 HIGH_V (관절 2 · 3 = meas_session HIGH, "
                   "관절 4 로 집게를 세움) 에서 관절 1 회전 -> 칸 입구 위 -> 칸 축 수직 직선. 모델 = URDF + 관절 보정 " + ",".join(map(str, JOFF))
                   + f" + 칸 이동 {SHIFT_MM} mm, 관절 4 ≤ {J4_MAX}. 팔 몸체 · 쥔 퍽 충돌 검사 통과 칸만.")
    doc["transit"] = dict(src["transit"], q_front_tilt=[round(float(v), 5) for v in hv0], tilt_deg=out[0]["tilt_deg"])
    doc["slots"] = out
    for path in [a.out] + ([a.control] if a.control else []):
        json.dump(doc, open(path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        print("->", os.path.normpath(path))


if __name__ == "__main__":
    main()
