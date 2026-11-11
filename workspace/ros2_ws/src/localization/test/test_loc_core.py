"""loc_core 단위 시험(ROS 없음): 거리장 · 잔차 · 탐색이 알려진 자세를 되찾는지."""
import math

import cv2
import numpy as np
import pytest

from localization import loc_core as lc


@pytest.fixture
def field(tmp_path):
    img = np.full((120, 160), 254, np.uint8)          # 0.025 m 칸, 4 m × 3 m 방 + 가운데 벽 하나(대칭 깨기)
    img[0, :] = img[-1, :] = 0
    img[:, 0] = img[:, -1] = 0
    img[40:80, 100] = 0
    cv2.imwrite(str(tmp_path / 'm.pgm'), img)
    (tmp_path / 'm.yaml').write_text('image: m.pgm\nresolution: 0.025\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return lc.DistField(str(tmp_path / 'm.yaml'))


def scan_from(field, pose, n=180, r_max=3.5):
    """pose 에서 광선을 쏴 벽에 닿는 점(base_link 기준)"""
    pts = []
    for a in np.linspace(-math.pi, math.pi, n, endpoint=False):
        for r in np.arange(0.05, r_max, 0.005):
            x, y = pose[0] + r * math.cos(pose[2] + a), pose[1] + r * math.sin(pose[2] + a)
            if field.lookup(np.array([x, y]))[()] < 0.0126:
                pts.append((r * math.cos(a), r * math.sin(a)))
                break
    return np.array(pts)


def test_residual_small_at_truth_large_when_shifted(field):
    pose = (1.2, 1.5, 0.3)
    pts = scan_from(field, pose)
    assert lc.residual_multi(field, [(pose, pts)], 'interp_p75') < 0.02
    assert lc.residual_multi(field, [((1.4, 1.5, 0.3), pts)], 'interp_p75') > 0.05


def test_local_search_recovers_pose(field):
    pose = (1.2, 1.5, 0.3)
    pts = scan_from(field, pose)
    r = lc.search(field, pts, center=(1.4, 1.4, 0.4), lin=0.5, ang=math.radians(30))
    assert math.hypot(r['best'][0] - pose[0], r['best'][1] - pose[1]) < 0.03
    assert abs(r['best'][2] - pose[2]) < math.radians(2)


def test_upside_down_lidar_mirrors_y():
    T = lc.T2d_from_tf(0.1, 0.0, 1.0, 0.0, 0.0, 0.0)  # roll π
    p = lc.scan_points([1.0], math.pi / 2, 0.0, T, 0.1, 3.5)
    assert np.allclose(p, [[0.1, -1.0]], atol=1e-9)


def test_rear_scan_moved_into_front_frame(field):
    """움직이는 중: 뒤 스캔 시각 자세의 점을 odom 이동만큼 옮기면 앞 스캔 시각 자세에서 맞음 (#151)"""
    p_front, p_rear = (1.2, 1.5, 0.3), (1.25, 1.52, 0.38)      # 0.05 s 사이 5 cm · 4.6° 움직임
    pts = scan_from(field, p_rear)
    raw = lc.residual_multi(field, [(p_front, pts)], 'interp_p75')
    moved = lc.move_points(pts, lc.se2_compose(lc.se2_inverse(p_front), p_rear))
    assert lc.residual_multi(field, [(p_front, moved)], 'interp_p75') < 0.02 < raw


def test_to_now_compensation():
    """스캔 시각 자세 ∘ (odom 스캔 시각 → 지금) = 지금 자세 (odom 이 map 과 틀어져 있어도)"""
    map_odom = (0.4, -0.2, 0.5)
    o_st, o_now = (1.0, 2.0, 0.1), (1.3, 2.1, 0.6)
    pose_st, pose_now = lc.se2_compose(map_odom, o_st), lc.se2_compose(map_odom, o_now)
    got = lc.se2_compose(pose_st, lc.se2_compose(lc.se2_inverse(o_st), o_now))
    assert np.allclose(got, pose_now, atol=1e-9)
