"""arm_calib_fit.py 시험: 알고 넣은 영점 오차 (· 처짐) 로 가짜 실측 CSV 를 만들어 되찾는지.  python3 -m pytest -q test_arm_calib_fit.py"""
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_calib_fit as A  # noqa: E402

# meas_session.py CALIB (P1 ~ P10, 도) + 칸 · 상자 자세
POSES = [("P1", [0, -15, 30, 30]), ("P2", [0, 0, 0, 0]), ("P3", [0, -40, 60, 20]), ("P4", [0, -20, 20, 60]),
         ("P5", [45, -15, 30, 30]), ("P6", [-45, -15, 30, 30]), ("P7", [90, -10, 10, 40]), ("P8", [172, -9, -20, 92]),
         ("P9", [0, -30, 50, 50]), ("P10", [0, -50, 90, 0]), ("칸3:0", [168, -15.7, 39.5, 21.2]), ("칸1:1", [157, -77.7, 69.9, 52.8]),
         ("상자가운데", [0, -35, 60, 45]), ("상자왼", [12, -35, 60, 45])]


def make_csv(path, e_true, k_true=None, sag=(), noise=0.3, seed=0, base_h=7.25, check=("P10", "상자왼")):
    arm = A.Arm()
    m = A.Model(arm, [1, 2, 3, 4], list(sag), False, base_h)
    rng = np.random.default_rng(seed)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["이름", "j1", "j2", "j3", "j4", "j5", "x", "y", "z", "쓰임"])
        for name, qd in POSES:
            q_read = np.radians(qd + [0])
            px = A.TCP_X if name.startswith(("칸", "상자")) else arm.tip_x
            k = np.zeros(5) if k_true is None else np.asarray(k_true)
            p = A.to_mm(arm.point(m.real_q(q_read, np.asarray(e_true), k), px), base_h) + rng.normal(0, noise, 3)
            w.writerow([name] + [f"{v:.3f}" for v in qd + [0]] + [f"{v:.2f}" for v in p] + ["확인" if name in check else ""])


def test_recovers_offsets(tmp_path):
    e_true = [-0.03, 0.10, -0.06, 0.05, 0.0]
    p = tmp_path / "c.csv"
    make_csv(p, e_true)
    e = A.main([str(p)])
    assert np.allclose(e[:4], e_true[:4], atol=math.radians(0.3)), (np.degrees(e), np.degrees(e_true))


def test_large_j2_offset(tmp_path):
    """지난번 실물처럼 관절 2 가 크게 (27°) 어긋나도 찾음"""
    e_true = [-0.028, -0.4788, 0.1641, 0.0955, 0.0]
    p = tmp_path / "c.csv"
    make_csv(p, e_true, noise=0.5)
    e = A.main([str(p)])
    assert np.allclose(e[:4], e_true[:4], atol=math.radians(0.5))


def test_sag_fit_separates(tmp_path):
    """관절 2 처짐 (팔 길이에 비례) 을 넣은 자료: --sag 2 로 맞추면 영점은 본래 값, 처짐 없이 맞추면 남는 오차가 큼"""
    e_true = [0.0, 0.05, -0.03, 0.02, 0.0]
    k_true = [0, 0.4, 0, 0, 0]       # rad / m (팔 길이 0.25 m 면 0.1 rad)
    p = tmp_path / "c.csv"
    make_csv(p, e_true, k_true=k_true, sag=(2,), noise=0.2)
    arm = A.Arm()
    rows = A.read_csv(str(p), arm, type("a", (), {"tip": None})())
    m1 = A.Model(arm, [1, 2, 3, 4], [2], False, 7.25)
    x1 = m1.fit([r for r in rows if r["use"]])
    e1, k1, _ = m1.unpack(x1)
    assert abs(k1[1] - 0.4) < 0.05 and np.allclose(e1[:4], e_true[:4], atol=math.radians(0.5))
    m0 = A.Model(arm, [1, 2, 3, 4], [], False, 7.25)
    x0 = m0.fit([r for r in rows if r["use"]])
    r0 = np.sqrt(np.mean(m0.resid(x0, rows) ** 2))
    r1 = np.sqrt(np.mean(m1.resid(x1, rows) ** 2))
    assert r1 < 0.5 and r0 > 2 * r1


def test_tip_from_urdf():
    assert 0.155 < A.Arm().tip_x < 0.165   # 손가락 앞 끝 (gripper_roll_link 앞 약 0.160 m)
