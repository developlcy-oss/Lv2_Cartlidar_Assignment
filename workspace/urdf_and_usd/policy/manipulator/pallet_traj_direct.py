"""팔레트 (퍽 스토리지) 놓기 궤적 '높이 이동 -> 칸 바로 위 -> 수직 넣기' 판 (2026-11-14, 사용자: 칸 위로 이동한 뒤 바로 넣기).

예전 두 판의 문제 (FK 로 쥔 퍽 바닥 경로를 계산해 확인, 이 파일 맨 아래 check 출력과 같음):
  - pallet_place_traj.json (노드 기본): rotate 를 퍽 바닥 0.156 m 에서 함 -> 1 열 칸 퍽 윗면 (0.180) 보다 24 mm 아래를 지나감.
    칸에 퍽이 있으면 그대로 침. carry 는 지나는 열 벽마다 계단식으로 올라가며 칸 반경까지 기어감 ('뱀처럼').
  - pallet_place_traj_high.json (11-13): 높이는 충분 (+50 mm) 하지만 place 가 11 mm 간격 경유점 10 개를 관절 직선으로 이어
    내려가는 동안 퍽이 반경 방향으로 13 mm 부풀고, 칸 입구 (벽 윗면) 아래에서도 9 mm 옆으로 미끄러지며 들어감.
이 판:
  rotate      : 앞 운반 자세 (J1 0, 퍽 바닥 반경 PIVOT_R, 높이 ZH) 에서 J1 만 칸 방향으로 (높이 그대로, 모든 칸 퍽 윗면 + 38 mm 위)
  carry       : 높이 ZH 그대로 칸 반경까지 직선 (5 mm 간격 IK) = 칸 바로 위
  place       : 칸 축을 따라 수직 직선으로 칸 바닥 + PLACE_GAP 까지 (2 mm 간격 IK, 반경 · 접선 어긋남 < 0.5 mm)
  release · retract (place 거꾸로) · return (carry 거꾸로) · rotate_back
segments 이름 · 형식은 pallet_place_traj.json 과 같음 (노드 runPalletSlot 그대로 읽음, 꺼내기는 거꾸로).
집게는 모든 칸 45° (팀 pallet_traj.py 와 같은 이유), 관절 5 = 0. 놓는 자세 (place 마지막) 는 pallet_traj.py 판과 관절값이 같음.
Isaac (chain45, 2026-11-14 512 대 x 퍽 2 개): 목표 칸 빼고 다 찬 팔레트에서 예전 궤적 8.0 % -> 이 궤적 운반 중 칸 퍽 · 몸체 닿음 0.
칸 순서 · 좌표는 입력 파일 (기본 pallet_place_traj.json) 그대로라 다시 돌려도 같은 결과.

    python3 pallet_traj_direct.py      # pallet_place_traj.json 을 이 판으로 바꾸고 control/config 사본도 같이 + 경로 검사표
    python3 pallet_traj_direct.py --check other.json   # 다른 궤적 파일의 쥔 퍽 경로 검사표만
    python3 pallet_traj_direct.py --vertical            # 수직 집기 (toppick.pitch −85°) 용 판 -> pallet_place_traj_vertical.json (+ control 사본)
      = --puck_axis --hand_dz 0.010 --enter_tilt_deg 5 (2026-11-21)
      · 퍽은 집게 축을 따라 쥠 (tool_link = 퍽 중심 + hand_dz, toppick 수직 집기와 같음)
      · 수직 집게로는 퍽 바닥을 base_link z 0.17 위로 못 올림 (관절 4 한계) -> 다른 칸 퍽 윗면 (0.180 ~ 0.192) 위를 못 지나감.
        그래서 rotate · carry 는 예전처럼 45° (높이 ZH), 칸 바로 위에서 내려가며 칸 입구 (벽 윗면 + ENTER_GAP) 까지 집게를 세우고
        (기울기를 높이에 따라 45° -> enter_tilt 로), 입구부터 칸 바닥까지는 enter_tilt (안 풀리면 1° 씩 키워 최대 15°) 로 수직 직선
      · 칸 좌표는 명목 (pallet_slots v5). 45° 손 맞춤 (pallet_slot_teach.json) 은 자세가 달라 옮기지 않음 (--taught 로 옮겨 보면
        1 열이 모델에서 J1 브라켓 안쪽이라 안 풀림) -> 실물에서 slot_teach.py (TEACH_TRAJ=이 파일) 로 칸마다 다시 맞춤
    (예전 낮은 판으로 돌리려면 python3 pallet_traj.py — control/config 사본은 손으로 복사)
"""

from __future__ import annotations

import argparse
import json
import math
import os

import numpy as np

import pallet_traj as PT

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "pallet_place_traj.json")  # 칸 순서 · 좌표 (v5) 는 이 파일 그대로
OUT = SRC
CONTROL = os.path.join(HERE, "..", "..", "..", "ros2_ws", "src", "control", "config", "pallet_place_traj.json")  # 노드 기본 궤적 (사본)
PIVOT_R = 0.10  # [m] J1 을 돌리는 퍽 바닥 반경 (팀 · high 판과 같음)
ZH = 0.230  # [m] 이동 높이 (퍽 바닥, base_link). 칸 퍽 윗면 최대 0.192 + 38 mm (처짐 6 ~ 13 mm 빼도 25 mm)
STEP = 0.005
STEP_DOWN = 0.002
ENTER_GAP = 0.006  # [m] 수직 판: 집게를 다 세우는 높이 = 칸 벽 윗면 + 이만큼 (퍽 바닥)
PUCK_AXIS_D = None  # 수직 판 (--puck_axis): 퍽 바닥 = TCP + 집게 축 x 이 값. None = 퍽 세운 채 (45° 판)


def bottom_path(q):
    """관절 -> 쥔 퍽 바닥 가운데 (base_link). 45° 집게: TCP = 퍽 바닥 + (GRASP_DZ + PUCK_H / 2) 수직 위."""
    p, R = _ARM.tcp(np.asarray(q, dtype=float))
    if PUCK_AXIS_D is not None:
        return p + R[:, 0] * PUCK_AXIS_D, R
    return p - np.array([0.0, 0.0, PT.GRASP_DZ + PT.PUCK_H / 2]), R


_ARM = PT.Arm(PT.URDF)


def check(doc, label):
    """쥔 퍽 바닥 경로: 다른 칸 퍽 (꽉 찬 팔레트) 윗면과의 높이 여유 · place 구간 수평 어긋남 (칸 입구 아래 / 전체)."""
    S = doc["slots"]
    print(f"== {label}: 퍽 바닥이 다른 칸 퍽 위를 지날 때 높이 여유 (꽉 찬 팔레트 가정, 음수 = 침) · 넣기 수평 어긋남")
    worst = 1.0
    for s in S:
        row = []
        for g in s["segments"]:
            if g["gripper"] != "hold":
                continue
            pts = np.array([bottom_path(q)[0] for q in g["q"]])
            cl, who = 1.0, ""
            for o in S:
                if o is s:
                    continue
                m = np.hypot(pts[:, 0] - o["x"], pts[:, 1] - o["y"]) < 2 * 0.015 + 0.004
                if m.any() and (pts[m, 2] - (o["floor_z"] + PT.PUCK_H)).min() < cl:
                    cl, who = (pts[m, 2] - (o["floor_z"] + PT.PUCK_H)).min(), f"{o['row']}_{o['index']}"
            worst = min(worst, cl)
            txt = f"{g['name']} z {pts[:, 2].min():.3f}~{pts[:, 2].max():.3f}"
            if who:
                txt += f" 여유 {cl * 1000:+.0f} mm ({who})"
            if g["name"] == "place":
                dxy = np.hypot(pts[:, 0] - s["x"], pts[:, 1] - s["y"])
                cup = pts[:, 2] < s["wall_top_z"] + 0.005
                txt += f" | 수평 어긋남 전체 {dxy.max() * 1000:.1f} mm, 칸 입구 아래 {dxy[cup].max() * 1000 if cup.any() else 0:.1f} mm"
            row.append(txt)
        print(f"  {s['row']}_{s['index']}: " + " | ".join(row))
    print(f"  -> 다른 칸 퍽 위 최소 여유 {worst * 1000:+.0f} mm")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--zh", type=float, default=ZH)
    ap.add_argument("--check", default=None)
    ap.add_argument("--src", default=SRC)
    ap.add_argument("--control", default=CONTROL, help="노드 사본 경로 ('' = 안 씀)")
    ap.add_argument("--tilt_deg", type=float, default=45.0, help="[°] 집게 축의 수직에서 기울기 (손끝 바깥 +). 45 = 기본 판, 5 = 수직 집기 (pitch −85°)")
    ap.add_argument("--puck_axis", action="store_true", help="퍽이 집게 축을 따라 쥐어짐 (수직 집기). 없으면 퍽은 세운 채 (45° 판)")
    ap.add_argument("--hand_dz", type=float, default=None, help="[m] --puck_axis 일 때 TCP = 퍽 중심 + 집게 축 따라 이만큼 (기본 PT.GRASP_DZ)")
    ap.add_argument("--taught", default=None, help="칸 손 맞춤 기록 (pallet_slot_teach.json): 칸 목표 = 맞춘 관절의 모델 FK 퍽 바닥")
    ap.add_argument("--enter_tilt_deg", type=float, default=None, help="[°] 칸 입구부터 바닥까지 집게 기울기 (없으면 --tilt_deg 그대로, 높이에 따라 바꾸지 않음)")
    ap.add_argument("--vertical", action="store_true", help="수직 집기 판 기본값 묶음 (위 설명)")
    a = ap.parse_args()
    if a.vertical:
        a.puck_axis = True
        a.hand_dz = 0.010 if a.hand_dz is None else a.hand_dz
        a.enter_tilt_deg = 5.0 if a.enter_tilt_deg is None else a.enter_tilt_deg
        if a.out == OUT:
            a.out = os.path.join(HERE, "pallet_place_traj_vertical.json")
        if a.control == CONTROL:
            a.control = os.path.join(os.path.dirname(CONTROL), "pallet_place_traj_vertical.json")
    if a.check:
        check(json.load(open(a.check, encoding="utf-8")), a.check)
        return
    src = json.load(open(a.src, encoding="utf-8"))
    arm = _ARM
    j1 = arm.j1
    global PUCK_AXIS_D
    d_model = PT.GRASP_DZ + PT.PUCK_H / 2   # ik_down 이 퍽 바닥에서 수직 위로 더하는 TCP 높이
    d_axis = (PT.GRASP_DZ if a.hand_dz is None else a.hand_dz) + PT.PUCK_H / 2
    PUCK_AXIS_D = d_axis if a.puck_axis else None
    taught = json.load(open(a.taught, encoding="utf-8"))["slots"] if a.taught else {}

    def at(r, yaw, z):
        return np.array([j1[0] + r * math.cos(yaw), j1[1] + r * math.sin(yaw), z])

    def solve(p, yaw, q0, tilt_deg=a.tilt_deg):
        tilt = math.radians(tilt_deg)
        if a.puck_axis:   # 퍽 바닥 p -> TCP = p − 집게 축 · d_axis (ik_down 의 수직 d_model 을 빼 둠)
            a_dir = np.array([math.sin(tilt) * math.cos(yaw), math.sin(tilt) * math.sin(yaw), -math.cos(tilt)])
            p = p - a_dir * d_axis - np.array([0.0, 0.0, d_model])
        q, e, t = arm.ik_down(p, yaw, q0, off=PT.BOTTOM, tilt=tilt)
        return q, e, t

    q_front, e, t = solve(at(PIVOT_R, 0.0, a.zh), 0.0, None)
    assert e < 1e-3 and t < math.radians(1.0), ("앞 운반 자세", e, t)
    out, worst = [], dict(err=0.0, tilt=0.0, margin=9.0)
    for s in src["slots"]:
        s = dict(s)
        key = f"{s['row']}_{s['index']}"
        if key in taught:   # 45° 손 맞춤 관절의 모델 FK: TCP = 퍽 중심 + 0.015 (맞춤 기준), 퍽 세운 채 -> 퍽 바닥
            t_, _ = arm.tcp(np.asarray(taught[key]["q_meas"], dtype=float))
            b = t_ - np.array([0.0, 0.0, 0.015 + PT.PUCK_H / 2])
            print(f"  {key}: 칸 목표 = 손 맞춤 FK (x {(b[0] - s['x']) * 1000:+.1f} y {(b[1] - s['y']) * 1000:+.1f} z {(b[2] - s['floor_z']) * 1000:+.1f} mm)")
            s.update(x_nominal=s["x"], y_nominal=s["y"], floor_z_nominal=s["floor_z"], x=round(float(b[0]), 5), y=round(float(b[1]), 5),
                     floor_z=round(float(b[2]), 5), wall_top_z=round(float(s["wall_top_z"] + b[2] - s["floor_z"]), 5), taught_target=True)
        yaw = math.atan2(s["y"] - j1[1], s["x"] - j1[0])
        r = math.hypot(s["x"] - j1[0], s["y"] - j1[1])
        z_p = s["floor_z"] + PT.PLACE_GAP
        z_in = s["wall_top_z"] + ENTER_GAP
        ok = lambda e_, t_: e_ < 1e-3 and t_ < math.radians(1.0)
        tilt_in = a.tilt_deg if a.enter_tilt_deg is None else a.enter_tilt_deg
        if a.enter_tilt_deg is not None:   # 입구 · 바닥이 모두 풀리는 가장 작은 기울기
            while tilt_in <= 15.0 and not all(ok(*solve(at(r, yaw, zz), yaw, None, tilt_in)[1:]) for zz in (z_in, z_p)):
                tilt_in += 1.0
            assert tilt_in <= 15.0, (s["row"], s["index"], "입구 · 바닥을 15° 안으로 못 세움")

        def tilt_at(zz):   # 입구 아래 = tilt_in, 위 = ZH 로 갈수록 a.tilt_deg
            if a.enter_tilt_deg is None or zz <= z_in:
                return tilt_in
            return tilt_in + (a.tilt_deg - tilt_in) * min(1.0, (zz - z_in) / (a.zh - z_in))

        # 놓는 자세를 전체 탐색으로 풀고 위로 이어 풂 (같은 해 가지)
        q_pl, e0, t0 = solve(at(r, yaw, z_p), yaw, None, tilt_at(z_p))
        down, q = [], q_pl
        zs = list(PT.lin(z_p, a.zh, STEP_DOWN))  # 아래 -> 위
        for zz in zs[1:]:
            q, e_, t_ = solve(at(r, yaw, zz), yaw, q[1:4], tilt_at(zz))
            if not ok(e_, t_):   # 기울기를 바꾸는 중 이어 풀기가 다른 가지로 새면 전체 탐색
                q2, e2, t2 = solve(at(r, yaw, zz), yaw, None, tilt_at(zz))
                if (e2, t2) < (e_, t_) and np.abs(np.array(q2) - np.array(down[-1][0] if down else q_pl)).max() < 0.3:
                    q, e_, t_ = q2, e2, t2
            down.append((q, e_, t_))
        down = down[::-1] + [(q_pl, e0, t0)]  # 위 -> 아래
        q_above = down[0][0]
        carry, q = [], q_above
        rs = list(PT.lin(r, PIVOT_R, STEP))  # 칸 반경 -> 피벗
        for rr in rs[1:]:
            q, e_, t_ = solve(at(rr, yaw, a.zh), yaw, q[1:4])
            carry.append((q, e_, t_))
        carry = carry[::-1] + [(q_above, down[0][1], down[0][2])]  # 피벗 쪽 -> 칸 위
        q_piv = [yaw, *q_front[1:4], 0.0]
        segs = []

        def seg(name, grip, sol):
            segs.append(dict(name=name, gripper=grip, q=[[round(float(v), 5) for v in x[0]] for x in sol],
                             max_err_mm=round(max(x[1] for x in sol) * 1000, 2), max_tilt_deg=round(math.degrees(max(x[2] for x in sol)), 2)))

        rot = [([th, *q_front[1:4], 0.0], 0.0, 0.0) for th in PT.lin(0.0, yaw, 0.05)]
        seg("rotate", "hold", rot)
        seg("carry", "hold", ([(q_piv, 0.0, 0.0)] if np.abs(np.array(carry[0][0]) - q_piv).max() > 0.02 else []) + carry)
        seg("place", "hold", down)
        segs.append(dict(name="release", gripper="open", q=[[round(float(v), 5) for v in q_pl]], max_err_mm=0.0, max_tilt_deg=0.0))
        seg("retract", "open", down[::-1][1:])
        seg("return", "open", carry[::-1][1:] + [(q_piv, 0.0, 0.0)])
        seg("rotate_back", "open", rot[::-1][1:])
        allq = np.array([x for sg in segs for x in sg["q"]])
        margin = float(min((allq - arm.lo).min(), (arm.hi - allq).min()))
        em = max(sg["max_err_mm"] for sg in segs)
        tm = max(sg["max_tilt_deg"] for sg in segs)
        worst["err"], worst["tilt"], worst["margin"] = max(worst["err"], em), max(worst["tilt"], tm), min(worst["margin"], margin)
        if a.puck_axis:
            print(f"   입구 z {z_in:.3f} 부터 집게 {tilt_in:.0f}°")
        print(f"{s['row']}열 {s['index']} 반경 {r * 1000:5.1f} mm J1 {math.degrees(yaw):+7.1f}° | 경유점 rotate {len(rot)} carry {len(segs[1]['q'])} place {len(down)}"
              f" | 오차 최대 {em:.2f} mm 집게 각 오차 최대 {tm:.2f}° | 관절 한계 여유 {margin:+.3f} rad")
        o = {k: v for k, v in s.items() if k not in ("segments", "direct_path")}
        o.update(tilt_deg=a.tilt_deg, enter_tilt_deg=tilt_in, q_front=[round(float(v), 5) for v in q_front], joint_margin_rad=round(margin, 4), segments=segs, direct_path=True)
        out.append(o)
    doc = {k: v for k, v in src.items() if k != "slots"}
    doc["note"] = ((f"2026-11-21 수직 집기 판 (--tilt_deg {a.tilt_deg} --enter_tilt_deg {a.enter_tilt_deg} --puck_axis --hand_dz {a.hand_dz}"
                    + (f" --taught {os.path.basename(a.taught)}" if a.taught else "") + "). ") if a.puck_axis else "") + ("2026-11-14 '높이 이동 -> 칸 바로 위 -> 수직 넣기' 판 (pallet_traj_direct.py 가 만듦, 직접 고치지 말 것). "
                   f"rotate = 퍽 바닥 높이 {a.zh:.3f} · 반경 {PIVOT_R} 에서 J1 만, carry = 같은 높이로 칸 반경까지 직선, place = 칸 축 수직 직선 (2 mm). "
                   "segments 형식은 pallet_traj.py 판과 같음 (gripper hold = 쥔 채, open = 벌림, 경유점 사이는 관절 공간 직선). "
                   "TCP = gripper_roll_link + (0.14, 0, 0) = 퍽 중심 + 0.015, joint5 0, base_link 좌표 [m] · 관절 [rad]. 칸 순서 · 좌표 = pallet_slots.yaml v5.")
    doc["transit"] = dict(src["transit"], r=PIVOT_R, puck_bottom_z=a.zh, q_front_tilt=[round(float(v), 5) for v in q_front], tilt_deg=a.tilt_deg)
    doc["max_err_mm"], doc["max_tilt_deg"] = round(worst["err"], 3), round(worst["tilt"], 3)
    doc["slots"] = out
    for path in [a.out] + ([a.control] if a.control else []):
        json.dump(doc, open(path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        print(f"-> {os.path.normpath(path)}")
    print(f"-> {a.out} (칸 {len(out)}, 오차 최대 {worst['err']:.2f} mm, 집게 각 오차 최대 {worst['tilt']:.2f}°, 관절 한계 여유 최소 {worst['margin']:+.3f} rad)"
          f" | 앞 운반 자세 {[round(float(v), 3) for v in q_front]}")
    check(doc, os.path.basename(a.out))


if __name__ == "__main__":
    main()
