#!/usr/bin/env python3
"""수직 넣기 높이 확인 (2026-11-22, 젯슨, 브링업 + kbd_joy 떠 있을 때).
손으로 맞춘 수직 칸 자세 (~/실측/slot_teach_vertical.json, 퍽을 쥔 바닥 자세) 에서 집게 기울기 그대로 위로 조금씩 올리며
관절 4 가 몇 도까지 쓰이는지 실물로 잼. 칸 입구 (퍽 바닥이 벽 윗면 + 6 mm = 바닥에서 36 mm) 까지 관절 4 가 막힘 (약 128°) 전에
가면 수직 넣기 가능.

    python3 ~/vertical_lift_check.py            1_3 (기록된 칸)
    python3 ~/vertical_lift_check.py 1_3 48     48 mm 까지 (기본 42)

순서: 지금 자세 -> (높은 자세 · 관절 1 회전) -> 맞춘 자세 60 mm 위 -> 맞춘 자세 (퍽 쥔 자리) -> 6 mm 씩 위로, 매번 Enter.
칸에 퍽을 넣어 두면 실제로 퍽을 쥐었다 들어 올리며 확인 (집게는 벌린 채로 움직이고, 바닥에서 쥘지 물어봄).
이동마다 Enter (s = 건너뜀, q = 끝). 위험하면 kbd_joy 에서 x.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.expanduser("~/Lv2_Cartlidar_Assignment/workspace/tools/arm_bench/real"))
import meas_session as m  # noqa: E402

REC = os.environ.get("TEACH_REC", os.path.expanduser("~/실측/slot_teach_vertical.json"))
OPEN = -0.30      # 약 47 mm
CLOSE = 0.923
J4_STOP = math.radians(128.0)   # 11-21 · 11-22 실물에서 관절 4 가 멈춘 자리


def main():
    slot = sys.argv[1] if len(sys.argv) > 1 else "1_3"
    top_mm = float(sys.argv[2]) if len(sys.argv) > 2 else 42.0
    rec = json.load(open(REC, encoding="utf-8"))
    if slot not in rec:
        sys.exit(f"{REC} 에 {slot} 기록 없음 (있는 칸: {list(rec)})")
    q0 = list(rec[slot]["q_meas"][:5])
    a = m.Arm()
    q, _ = a.show()
    if q[1] < -1.9:
        if not m.node_unfold():
            sys.exit("STOW 에서 pose DRIVE 실패")
        q, _ = a.show()
    a.cmd = list(q)
    m.P(f"\n[{slot}] 맞춘 수직 자세 (도) {m.deg(q0)} | 관절 4 막힘 약 {math.degrees(J4_STOP):.0f}°")
    try:
        above, err = m.cart_step(q0, [0.0, 0.0, 0.06])
        if err > 1.0:
            sys.exit("맞춘 자세 60 mm 위를 못 풂")
        a.move(above, OPEN, label=f"{slot} 맞춘 자세 60 mm 위 (집게 벌림)")
        a.move(q0, OPEN, label=f"{slot} 맞춘 자세 (퍽 쥐는 자리)")
        if m.ask("여기서 퍽을 쥘까? (y = 쥐고 들어 올리며 잼 / n = 빈손으로 잼)") == "y":
            a.move(a.cmd, CLOSE, confirm=False, T=1.5, label="쥐기")
        rows = []
        cur = list(q0)
        for k in range(1, int(top_mm // 6) + 1):
            nxt, err = m.cart_step(cur, [0.0, 0.0, 0.006])
            if err > 1.0:
                m.P(f"  {6 * k} mm: 못 풂 -> 멈춤")
                break
            if nxt[3] > J4_STOP:
                m.P(f"  {6 * k} mm: 관절 4 목표 {math.degrees(nxt[3]):.1f}° > 막힘 {math.degrees(J4_STOP):.0f}° -> 보내지 않고 멈춤")
                break
            if not a.move(nxt, label=f"위로 {6 * k} mm"):
                break
            meas, g = a.read()
            rows.append((6 * k, math.degrees(nxt[3]), math.degrees(meas[3]), g))
            cur = nxt
        m.P("\n높이 (mm) | 관절 4 명령 (도) | 관절 4 측정 (도) | 집게")
        for h, c, me, g in rows:
            m.P(f"  {h:4d}     | {c:7.1f}          | {me:7.1f}          | {g:+.3f}" + ("   ← 명령보다 2° 넘게 덜 감 (막힘?)" if c - me > 2.0 else ""))
        m.P("칸 입구 = 36 mm. 36 mm 까지 측정이 명령을 따라가면 수직 넣기 가능.")
        a.move(a.cmd, OPEN, confirm=False, T=1.5, label="집게 벌림 (퍽 놓음)")
        a.move(above, label="60 mm 위")
    except m.Back:
        m.P("  (끝냄)")
    a.move([0] + m.HIGH, label="정면 높은 자세")


if __name__ == "__main__":
    main()
