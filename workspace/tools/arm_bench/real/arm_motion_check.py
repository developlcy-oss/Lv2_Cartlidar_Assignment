#!/usr/bin/env python3
"""arm_motion_check.py — 팔 자세 이동 (켜질 때 정리 STOW → DRIVE · /manipulator/pose) 기록을 읽어 속도 · 쏠림 · 떨림을 표로 (2026-11-14).

  ros2 bag record -o ~/arm_check/<이름> /arm_controller/controller_state /diagnostics     # 이동 전에 켜고, 끝나면 Ctrl-C
  python3 arm_motion_check.py ~/arm_check/<이름>

읽는 토픽: /arm_controller/controller_state (reference = 명령 궤적, feedback = 엔코더, error = reference − feedback).
구간 = reference 속도가 0.01 rad/s 넘는 동안 (관절 묶음마다 따로). 표:
  - 시간 [s] · 가장 큰 관절 속도 [rad/s]                       → 속도
  - 추종 오차 최대 [deg] (관절별, 움직이는 동안)                  → 늦게 따라감 · 넘침
  - 떨림: (엔코더 속도 − 명령 속도) 의 RMS [deg/s] · 부호 바뀜 [회/s] → 클수록 덜덜거림
  - 멈춘 뒤 1 s: 남은 오차 [deg] · 흔들림 (최대 − 최소) [tick]
  - 집게 숙임 (엔코더 기준) = 관절 2 + 3 + 4 의 남은 오차 합 [deg], − = 명령보다 더 숙임 (집게 방향 = −(q2 + q3 + q4))
엔코더 바깥 (기어 유격 · 링크 휨) 은 안 보임 → 집게 각도는 수평계로 따로 (DRIVE 설계 −26.9°).
"""
import sys
import warnings

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPIC = "/arm_controller/controller_state"
ARM = ["joint1", "joint2", "joint3", "joint4", "joint5"]
TICK = 360.0 / 4096   # deg
DEG = 180.0 / np.pi


def read(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path), rosbag2_py.ConverterOptions("", ""))
    msg_t = get_message("control_msgs/msg/JointTrajectoryControllerState")
    t, ref, rv, fb, fv = [], [], [], [], []
    while r.has_next():
        topic, data, stamp = r.read_next()
        if topic != TOPIC:
            continue
        m = deserialize_message(data, msg_t)
        idx = [list(m.joint_names).index(j) for j in ARM]
        pick = lambda seq: [seq[i] if i < len(seq) else 0.0 for i in idx]
        t.append(stamp * 1e-9)
        ref.append(pick(m.reference.positions)); rv.append(pick(m.reference.velocities))
        fb.append(pick(m.feedback.positions)); fv.append(pick(m.feedback.velocities))
    if not t:
        sys.exit(f"{TOPIC} 없음 — bag 에 이 토픽을 넣었는지 확인")
    return np.array(t) - t[0], *(np.array(a) for a in (ref, rv, fb, fv))


def segments(t, rv, thr=0.01, gap=0.3):
    moving = np.abs(rv).max(axis=1) > thr
    segs, start = [], None
    for i, m in enumerate(moving):
        if m and start is None:
            start = i
        if not m and start is not None:
            if i + 1 < len(t) and np.any(moving[i:i + int(gap * 100) + 1]):
                continue   # 짧은 끊김은 같은 구간
            segs.append((start, i)); start = None
    if start is not None:
        segs.append((start, len(t) - 1))
    return segs


def main():
    warnings.filterwarnings("ignore", category=DeprecationWarning)
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    t, ref, rv, fb, fv = read(sys.argv[1])
    err = ref - fb
    segs = segments(t, rv)
    print(f"기록 {t[-1]:.1f} s, 이동 구간 {len(segs)} 개\n")
    for n, (a, b) in enumerate(segs, 1):
        moved = np.abs(ref[b] - ref[a])
        j = [k + 1 for k in range(5) if moved[k] > 0.02]
        dt = t[b] - t[a]
        vmax = np.abs(rv[a:b + 1]).max()
        print(f"[{n}] {t[a]:6.1f} ~ {t[b]:6.1f} s  관절 {j}  시간 {dt:5.1f} s  가장 빠른 명령 속도 {vmax:.2f} rad/s")
        emax = np.abs(err[a:b + 1]).max(axis=0) * DEG
        res = (fv[a:b + 1] - rv[a:b + 1]) * DEG
        rms = np.sqrt((res ** 2).mean(axis=0))
        flips = (np.diff(np.sign(np.diff(fb[a:b + 1], axis=0)), axis=0) != 0).sum(axis=0) / max(dt, 1e-3)
        print("     관절            " + "  ".join(f"{'J' + str(k + 1):>6s}" for k in range(5)))
        print("     추종 오차 최대° " + "  ".join(f"{v:6.2f}" for v in emax))
        print("     떨림 RMS °/s    " + "  ".join(f"{v:6.2f}" for v in rms))
        print("     부호 바뀜 /s    " + "  ".join(f"{v:6.1f}" for v in flips))
        settle = (t > t[b]) & (t <= t[b] + 1.0)
        if settle.sum() > 5:
            e_end = err[settle].mean(axis=0) * DEG
            pp = (fb[settle].max(axis=0) - fb[settle].min(axis=0)) * DEG / TICK
            print("     멈춘 뒤 오차°   " + "  ".join(f"{v:6.2f}" for v in e_end))
            print("     멈춘 뒤 흔들림  " + "  ".join(f"{v:6.0f}" for v in pp) + "  (tick, 1 = 0.088°)")
            print(f"     집게 숙임 (엔코더): {e_end[1] + e_end[2] + e_end[3]:+.2f}°  (− = 명령보다 더 숙임)")
        print()
    print("기준 (2026-11-13 opencr_arm_tool 0.2 rad/s): 추종 오차 최대 1.6° · 멈춘 뒤 오차 0.5° 안 · 흔들림 1 ~ 3 tick")


if __name__ == "__main__":
    main()
