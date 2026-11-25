#!/usr/bin/env python3
"""SpinBudget 시험 (ROS 없이: python3 test_v2_filter.py). v2_filter 는 rclpy 를 불러오므로 없으면 건너뜀."""
import math
import sys
import types

try:
    import rclpy  # noqa: F401
except ImportError:   # rclpy 없는 PC: 가짜 모듈로 SpinBudget 만 불러옴
    for n in ("rclpy", "rclpy.node", "geometry_msgs", "geometry_msgs.msg", "nav_msgs", "nav_msgs.msg", "std_msgs", "std_msgs.msg"):
        sys.modules[n] = types.ModuleType(n)
    sys.modules["rclpy.node"].Node = object
    for n, a in (("geometry_msgs.msg", ["TwistStamped"]), ("nav_msgs.msg", ["Odometry"]), ("std_msgs.msg", ["Float32"])):
        for x in a:
            setattr(sys.modules[n], x, object)
from v2_filter import SpinBudget

D = math.radians


def turn(b, deg, step=1.0):
    n = int(abs(deg) / step)
    for _ in range(n):
        b.add(D(step) * (1 if deg > 0 else -1))


def test_same_direction_blocked():   # 90° + 90° 이어 돌기는 두 모드 모두 막음
    for m in ("sum", "span"):
        b = SpinBudget(D(90), m)
        turn(b, 90)
        assert b.remaining(1) < D(0.5), m
        assert b.remaining(-1) >= (0 if m == "sum" else D(89)), m


def test_back_and_forth_span():   # 1851: −49° 뒤 +41° — span 은 예산 안 먹음, sum 은 90° 가 참
    s, t = SpinBudget(D(90), "span"), SpinBudget(D(90), "sum")
    for b in (s, t):
        turn(b, -49)
        turn(b, 41)
    assert abs(s.used - D(49)) < 1e-6 and s.remaining(1) > D(40) + D(8) - 1e-6
    assert t.remaining(1) < D(1)


def test_span_never_exceeds_90():   # 아무렇게나 돌아도 방향 범위 ≤ 90° (180° · 360° 불가)
    import random
    random.seed(1)
    b = SpinBudget(D(90), "span_cap", D(400))
    for _ in range(20000):
        d = random.choice((-1, 1)) * D(0.5)
        if b.remaining(1 if d > 0 else -1) >= abs(d):
            b.add(d)
        assert b.used <= D(90) + 1e-9


def test_span_cap():   # 범위는 넉넉해도 왕복 총량이 상한을 넘으면 막힘
    b = SpinBudget(D(90), "span_cap", D(180))
    for _ in range(3):
        turn(b, 50); turn(b, -50)
    assert b.remaining(1) < D(1) and b.remaining(-1) < D(1)
    b2 = SpinBudget(D(90), "span")
    for _ in range(3):
        turn(b2, 50); turn(b2, -50)
    assert b2.remaining(1) > D(30)


def test_default_is_sum():
    assert SpinBudget().mode == "sum"


def test_reset():
    b = SpinBudget(D(90), "span")
    turn(b, 80)
    b.reset()
    assert b.used == 0 and b.remaining(1) == D(90)


if __name__ == "__main__":
    for k, f in sorted(globals().items()):
        if k.startswith("test_"):
            f(); print("ok", k)
