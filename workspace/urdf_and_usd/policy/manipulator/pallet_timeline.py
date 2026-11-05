"""팔레트 칸 궤적 (pallet_place_traj.json) -> 물리 스텝별 관절 목표 시간표. Isaac 없이 import 됨 (순수 파이썬).

pallet_place_check.py (놓기만 확인) 와 chain_pallet_eval.py (집기 -> 놓기 이어서 평가) 가 같이 씀 -> 두 평가가 같은 궤적 · 속도를 따름.
속도: 쥔 구간 = row_speed (그 열) > tilt_speed (기울인 칸) > speed, 빈 집게 구간 = empty_speed (0 이면 쥔 구간과 같음).
ROS 쪽 같은 일은 control/manipulator_node (pallet_load_<열>_<칸>_tilt, sag_comp.hpp).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

PHASES = ["grip", "carry", "place", "release", "after"]
PHASE_OF = {"rotate": 1, "carry": 1, "place": 2, "release": 3, "retract": 4, "return": 4, "rotate_back": 4}
SEGS = list(PHASE_OF)
DWELL = {"place": 0.3, "release": 0.6}  # [s] 구간 끝에서 머묾 (놓는 높이 · 벌린 뒤)
AFTER_WAIT = 1.0  # [s] 궤적 끝에서 기다림 (퍽이 가라앉는지)


@dataclass
class Speeds:
    speed: float = 0.8  # [rad/s] 기본
    tilt_speed: float = 0.8  # 기울인 칸 (tilt_deg) 의 쥔 구간
    row_speed: dict = field(default_factory=lambda: {3: 0.4})  # 열 -> 쥔 구간 속도 (2026-11-03: 0.8 이면 3열 상자 퍽이 칸 벽에 걸림)
    empty_speed: float = 0.8  # 빈 집게 구간, 0 = 쥔 구간과 같음


def parse_row_speed(s: str) -> dict:
    """'3:0.4,2:0.6' -> {3: 0.4, 2: 0.6}."""
    return {int(x.split(":")[0]): float(x.split(":")[1]) for x in s.split(",") if x}


def seg_speed(slot, held, sp: Speeds):
    if not held and sp.empty_speed:
        return sp.empty_speed
    if slot["row"] in sp.row_speed:
        return sp.row_speed[slot["row"]]
    if slot.get("tilt_deg") and sp.tilt_speed:
        return sp.tilt_speed
    return sp.speed


def timeline(slot, dt, sp: Speeds, q_front_default, roll_pi=False):
    """칸 궤적 -> 물리 스텝마다 (팔 목표 5, 손가락 0 쥠 · 1 벌림, 단계 PHASES 번호, 구간 SEGS 번호). 관절 공간 직선, 속도 한도.
    roll_pi: 손목 롤 (joint5) 을 rotate 동안 0 -> π, rotate_back 동안 π -> 0 (pallet_place_check --roll_pi 시험용)."""
    roll = math.copysign(math.pi, slot["j1_yaw"]) if roll_pi else 0.0
    out, prev = [], list(slot.get("q_front", q_front_default))
    for sg in slot["segments"]:
        ph = PHASE_OF[sg["name"]]
        g = 0 if sg["gripper"] == "hold" else 1
        speed = seg_speed(slot, g == 0, sp)
        nq = len(sg["q"])
        for iq, q in enumerate(sg["q"]):
            f = (iq + 1) / nq
            q = list(q[:4]) + [roll * (f if sg["name"] == "rotate" else (1 - f) if sg["name"] == "rotate_back" else 1.0)]
            n = max(1, int(math.ceil(max(abs(a - b) for a, b in zip(q, prev)) / (speed * dt))))
            for k in range(1, n + 1):
                out.append(([p + (a - p) * k / n for a, p in zip(q, prev)], g, ph, SEGS.index(sg["name"])))
            prev = q
        out += [(list(prev), g, ph, SEGS.index(sg["name"]))] * int(DWELL.get(sg["name"], 0.0) / dt)
    out += [(list(prev), 1, 4, len(SEGS) - 1)] * int(AFTER_WAIT / dt)
    return out


def joint_line(q_from, q_to, speed, dt):
    """관절 공간 직선 (가장 많이 도는 관절이 speed [rad/s]) -> 스텝별 목표 목록 (끝점 포함, 시작점 뺌)."""
    n = max(1, int(math.ceil(max(abs(a - b) for a, b in zip(q_to, q_from)) / (speed * dt))))
    return [[p + (a - p) * k / n for a, p in zip(q_to, q_from)] for k in range(1, n + 1)]
