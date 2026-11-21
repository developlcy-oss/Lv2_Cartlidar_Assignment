"""loc_guard_core 단위 시험(ROS 없음): AMCL 자세 순간 튐 거부 · 국소 정합 · 포기 · 예열 · 섞기 속도 제한 · 각도 감싸기.
실물 수치: 1115_2055(18 cm 한 칸 스파이크) · 1116_1225(37 cm 튐 뒤 정지 표류) · 1116_1215(시작 수렴 급변)."""
import math

import cv2
import numpy as np
import pytest

from localization import loc_core as lc
from localization import loc_guard_core as g

DT = 0.1


@pytest.fixture(scope='module')
def field(tmp_path_factory):
    """L자 복도 지도(0.025 m 칸, 5 m × 4 m) + 기둥 하나(대칭 깨기)"""
    d = tmp_path_factory.mktemp('map')
    img = np.full((160, 200), 254, np.uint8)
    img[0, :] = img[-1, :] = 0
    img[:, 0] = img[:, -1] = 0
    img[:100, 80:] = 0            # 오른쪽 위를 막아 L자: 세로 복도(x 0~2 m) + 가로 복도(y 0~1.5 m)
    img[120:130, 150:155] = 0     # 가로 복도 안 기둥
    img[30:34, 20:40] = 0         # 세로 복도 안 턱
    cv2.imwrite(str(d / 'm.pgm'), img)
    (d / 'm.yaml').write_text('image: m.pgm\nresolution: 0.025\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return lc.DistField(str(d / 'm.yaml'))


def scan_from(field, pose, n=180, r_max=3.5):
    """pose 에서 광선을 쏴 벽에 처음 닿는 점(base_link 기준)"""
    r = np.arange(0.05, r_max, 0.005)
    pts = []
    for a in np.linspace(-math.pi, math.pi, n, endpoint=False):
        xy = np.stack([pose[0] + r * math.cos(pose[2] + a), pose[1] + r * math.sin(pose[2] + a)], 1)
        hit = np.nonzero(field.lookup(xy) < 0.0126)[0]
        if len(hit):
            pts.append((r[hit[0]] * math.cos(a), r[hit[0]] * math.sin(a)))
    return np.array(pts)


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def ang(a, b):
    return abs(g.wrap(a[2] - b[2]))


def deg(p):
    """실물 표기 (x, y, yaw[deg]) → (x, y, yaw[rad])"""
    return (p[0], p[1], math.radians(p[2]))


def started(core, amcl, odom, pts=None, t0=10.0):
    """예열이 끝난 상태에서 첫 자세를 받아 last_good 을 잡음"""
    core.reset(0.0)
    d = core.update(t0, amcl, odom, pts)
    assert d.action in ('init', 'accept', 'accept_better')
    return d


# ---------- 순수 함수 ----------

def test_wrap_range():
    for a in (math.pi, -math.pi, 3 * math.pi, -3 * math.pi + 1e-6, 7.0, -7.0, 0.0):
        w = g.wrap(a)
        assert -math.pi - 1e-12 <= w <= math.pi + 1e-12    # atan2 는 −π 를 돌려줄 수 있음(판정엔 영향 없음)
        assert math.isclose(math.cos(w), math.cos(a), abs_tol=1e-9) and math.isclose(math.sin(w), math.sin(a), abs_tol=1e-9)


def test_predict_and_map_odom_consistent():
    """map→odom 이 고정이면 예측 = 실제 map 자세, map_odom_from 은 그 고정값"""
    T = (0.4, -0.2, 2.9)
    o0, o1 = (1.0, 2.0, 3.0), (1.3, 2.1, -3.0)
    m0, m1 = lc.se2_compose(T, o0), lc.se2_compose(T, o1)
    assert np.allclose(g.predict(m0, o0, o1), m1, atol=1e-9)
    mo = g.map_odom_from(m1, o1)
    assert np.allclose(mo[:2], T[:2], atol=1e-9) and abs(g.wrap(mo[2] - T[2])) < 1e-9


# ---------- 1. 정상 이동 받아들임 ----------

def test_normal_motion_accepted_out_equals_amcl():
    core = g.GuardCore(g.Params())
    T = (0.3, -0.1, 0.2)                       # map→odom 고정
    odom = [(0.08 * k * DT * math.cos(0.36 * k * DT), 0.08 * k * DT * math.sin(0.36 * k * DT), 0.36 * k * DT) for k in range(30)]
    started(core, lc.se2_compose(T, odom[0]), odom[0])
    for k in range(1, 30):
        amcl = lc.se2_compose(T, odom[k])
        d = core.update(10.0 + k * DT, amcl, odom[k])
        assert d.action == 'accept', (k, d)
        assert np.allclose(d.out, amcl, atol=1e-9)
        assert d.reinit is False


# ---------- 2. 한 칸 18 cm 스파이크(1115_2055) ----------

def test_single_spike_held_then_reaccepted():
    tr = [deg(p) for p in ((0.3749, 0.4912, -62.8), (0.3766, 0.4849, -61.2), (0.2949, 0.6243, -59.8),
                           (0.3919, 0.4688, -58.3), (0.3967, 0.4613, -57.3))]
    good = [tr[0], tr[1], None, tr[3], tr[4]]
    good[2] = (0.5 * (tr[1][0] + tr[3][0]), 0.5 * (tr[1][1] + tr[3][1]), 0.5 * (tr[1][2] + tr[3][2]))   # 참 궤적(보간)
    T = (0.05, 0.02, 0.1)                      # odom = T⁻¹ ∘ 참 자세 (odom 증분 = 참 증분)
    odom = [lc.se2_compose(lc.se2_inverse(T), p) for p in good]
    core = g.GuardCore(g.Params())             # field 없음 → 잔차 None → 튐이면 hold
    started(core, tr[0], odom[0])
    assert core.update(10.1, tr[1], odom[1]).action == 'accept'
    d = core.update(10.2, tr[2], odom[2])
    assert g.is_jump(d.err_xy, d.err_yaw, g.Params())
    assert d.action == 'hold'
    assert dist(d.out, d.pred) < 1e-9 and dist(d.out, good[2]) < 0.02 and ang(d.out, good[2]) < math.radians(2)
    d = core.update(10.3, tr[3], odom[3])
    assert d.action == 'accept' and np.allclose(d.out, tr[3], atol=1e-9)
    assert core.update(10.4, tr[4], odom[4]).action == 'accept'


# ---------- 3. 37 cm 튐 + 국소 정합(합성 L자 지도) ----------

def test_big_jump_refined_to_truth(field):
    truth0, truth1 = (1.00, 2.50, 1.2), (1.03, 2.56, 1.24)
    odom0, odom1 = (5.0, 5.0, 0.0), lc.se2_compose((5.0, 5.0, 0.0), lc.se2_compose(lc.se2_inverse(truth0), truth1))
    core = g.GuardCore(g.Params(), field)
    amcl0 = (truth0[0] + 0.025, truth0[1] - 0.02, truth0[2] + math.radians(1.0))   # 받아들인 AMCL 이 3 cm 남짓 어긋남
    started(core, amcl0, odom0, scan_from(field, truth0))
    pts = scan_from(field, truth1)
    amcl1 = (truth1[0] - 0.30, truth1[1] + 0.22, truth1[2] - 0.05)                 # 37 cm 튐
    d = core.update(10.1, amcl1, odom1, pts)
    assert d.action == 'refine', d
    assert d.r_amcl > 0.03 and d.r_ref <= 0.03
    assert dist(d.out, truth1) < 0.02 and ang(d.out, truth1) < math.radians(1.5)
    # 다음 칸: AMCL 이 참 자세로 돌아오면 정합 자세 기준 예측과 맞아 받아들임
    odom2 = lc.se2_compose(odom1, (0.01, 0.0, 0.0))
    truth2 = lc.se2_compose(truth1, (0.01, 0.0, 0.0))
    d = core.update(10.2, truth2, odom2, scan_from(field, truth2))
    assert d.action == 'accept', d


# ---------- 4. odom 이 틀림(실제 kidnapping) → AMCL 받아들임 ----------

def test_odom_wrong_amcl_good_accepted(field):
    truth0 = (1.00, 2.50, 1.2)
    odom0 = (0.0, 0.0, 0.0)
    core = g.GuardCore(g.Params(), field)
    started(core, truth0, odom0, scan_from(field, truth0))
    truth1 = (0.85, 2.20, 1.5)                 # 실제로는 들려서 옮겨짐
    odom1 = (0.02, 0.0, 0.0)                   # odom 은 거의 안 움직였다고 함
    pts = scan_from(field, truth1)
    d = core.update(10.1, truth1, odom1, pts)
    assert g.is_jump(d.err_xy, d.err_yaw, g.Params())
    assert d.action == 'accept_better', d
    assert d.r_amcl <= 0.03 and d.r_pred > d.r_amcl
    assert np.allclose(d.out, truth1, atol=1e-9)


# ---------- 5. 둘 다 나쁨이 계속 → reinit · give_up(LOST) ----------

def test_both_bad_gives_up_after_limit(field):
    p = g.Params()
    truth0, odom0 = (1.00, 2.50, 1.2), (0.0, 0.0, 0.0)
    core = g.GuardCore(p, field)
    started(core, truth0, odom0, scan_from(field, truth0))
    rng = np.random.default_rng(0)
    junk = rng.uniform(-2, 2, (120, 2))        # 지도 어디와도 안 맞는 스캔
    amcl = (1.40, 2.20, 0.9)                   # 50 cm 튐
    acts, reinits = [], []
    for k in range(1, 40):
        d = core.update(10.0 + k * DT, amcl, odom0, junk)
        acts.append(d.action)
        reinits.append(d.reinit)
        if d.action == 'give_up':
            assert np.allclose(d.out, amcl, atol=1e-9)
            break
    assert acts[-1] == 'give_up', acts
    assert all(a == 'hold' for a in acts[:-1]), acts
    # 첫 거부 10.1 s → 10.1 + 2.0 s 에서 포기(부동소수 여유 한 칸)
    assert p.max_reject_s / DT - 1 <= len(acts) - 1 <= p.max_reject_s / DT + 1
    assert reinits.count(True) == 1 and reinits.index(True) == p.reinit_after_n - 1
    # 포기 뒤에는 AMCL 을 새 기준으로 받아들임
    d = core.update(10.0 + 40 * DT, amcl, odom0, junk)
    assert d.action == 'accept'


def test_real_1116_drift_no_scan_gives_up():
    """1116_1225: 37 cm 튐 뒤 정지 중 표류 — 스캔 없으면 hold 가 이어지다 2 s 뒤 give_up"""
    p = g.Params()
    pre = [deg((1.5514, 4.8604, 120.4)), deg((1.5886, 4.7968, 118.6))]
    T = (0.0, 0.0, 0.0)
    core = g.GuardCore(p)
    started(core, pre[0], pre[0])
    assert core.update(10.1, pre[1], pre[1]).action == 'accept'
    stop_odom = lc.se2_compose(pre[1], (0.0, 0.0, math.radians(-1.4)))   # 꺾다가 멈춤
    jumped = deg((1.4136, 5.1293, 117.2))
    seq = [jumped] + [(1.4008 + (1.5162 - 1.4008) * i / 19, 5.1317 + (5.1550 - 5.1317) * i / 19, jumped[2]) for i in range(20)]
    acts, reinit_at = [], None
    for k, a in enumerate(seq):
        d = core.update(10.2 + k * DT, a, stop_odom)
        acts.append(d.action)
        if d.reinit:
            reinit_at = k
        if d.action == 'hold':
            assert dist(d.out, d.pred) < 1e-9
        if d.action == 'give_up':
            assert np.allclose(d.out, a, atol=1e-9)
            break
    assert acts[0] == 'hold' and acts[-1] == 'give_up', acts
    assert reinit_at == p.reinit_after_n - 1


def test_reject_guard_timing():
    p = g.Params()
    rg = g.RejectGuard(p)
    out = [rg.on_reject(5.0 + k * DT) for k in range(25)]
    assert out[p.reinit_after_n - 1] == 'reinit' and out.count('reinit') == 1
    assert 'give_up' in out and out.index('give_up') in (19, 20, 21)
    rg.on_accept()
    assert rg.on_reject(100.0) == 'reject'


# ---------- 6. 예열 안 큰 변화 통과(1116_1215) ----------

def test_warmup_passes_convergence_jumps():
    core = g.GuardCore(g.Params())
    core.reset(0.0)
    odom = (0.0, 0.0, 0.0)
    a4, a5 = deg((0.7085, 0.8980, -169.9)), deg((0.6907, 0.9474, -179.8))
    for t, a in ((0.4, a4), (0.5, a5), (1.0, (a5[0] + 0.06, a5[1], a5[2] + 0.17)), (1.9, (0.4, 0.4, 0.0))):
        d = core.update(t, a, odom)
        assert d.action in ('warmup', 'init'), (t, d)
        assert np.allclose(d.out, a, atol=1e-9)
    # 예열 끝: 같은 큰 튐은 이제 거부
    d = core.update(2.5, (0.9, 0.9, 1.0), odom)
    assert d.action == 'hold'


def test_warmup_class():
    w = g.Warmup(2.0)
    assert w.active(0.0)
    w.start(1.0)
    assert w.active(2.9) and not w.active(3.1)


# ---------- 7. 섞기 속도 제한 ----------

def test_rate_limit_step_bounds():
    cur, tgt = (0.0, 0.0, 0.0), (0.06, 0.08, math.radians(20))     # 10 cm · 20°
    dt = 0.05
    nxt = g.rate_limit_step(cur, tgt, dt, 0.15, math.radians(15))
    assert dist(cur, nxt) <= 0.15 * dt + 1e-9
    assert ang(cur, nxt) <= math.radians(15) * dt + 1e-9
    assert dist(cur, nxt) > 0.15 * dt * 0.99                      # 멈춰 있지 않음
    # 방향이 목표 쪽
    assert np.allclose(np.array(nxt[:2]) / np.linalg.norm(nxt[:2]), [0.6, 0.8], atol=1e-6)
    # 가까우면 목표 그대로
    near = (0.001, 0.0, 0.001)
    assert np.allclose(g.rate_limit_step(cur, near, dt, 0.15, math.radians(15)), near, atol=1e-12)


def test_rate_limit_step_yaw_wraps_short_way():
    cur, tgt = (0.0, 0.0, math.radians(179)), (0.0, 0.0, math.radians(-179))
    nxt = g.rate_limit_step(cur, tgt, 1.0, 0.15, math.radians(15))
    assert np.allclose(g.wrap(nxt[2]), tgt[2], atol=1e-9)         # 2° 차이 → 한 번에 도달(358° 돌지 않음)


def test_blender_limits_each_step():
    p = g.Params()
    b = g.Blender(p)
    b.set_target((0.0, 0.0, 0.0))                                  # 첫 목표는 snap
    assert np.allclose(b.cur, (0.0, 0.0, 0.0))
    b.set_target((0.10, 0.0, 0.0))
    dt, prev = 0.05, b.cur
    for _ in range(30):
        cur = b.step(dt)
        assert dist(prev, cur) <= p.blend_v * dt + 1e-9
        prev = cur
    assert dist(prev, (0.10, 0.0, 0.0)) < 1e-9                     # 결국 도달
    b.set_target((0.20, 0.0, 0.0))
    cur = b.step(dt, still=True)
    assert dist((0.10, 0.0, 0.0), cur) <= p.blend_v * p.still_scale * dt + 1e-9
    b.set_target((5.0, 5.0, 1.0), snap=True)
    assert np.allclose(b.cur, (5.0, 5.0, 1.0))


# ---------- 8. 각도 감싸기 ----------

def test_jump_check_near_pi():
    p = g.Params()
    pred, meas = (1.0, 1.0, math.radians(179)), (1.0, 1.0, math.radians(-179))
    e_xy, e_yaw = g.pose_error(pred, meas)
    assert abs(e_yaw - math.radians(2)) < 1e-9 and e_xy < 1e-9
    assert not g.is_jump(e_xy, e_yaw, p)
    e_xy, e_yaw = g.pose_error(pred, (1.0, 1.0, math.radians(-175)))
    assert g.is_jump(e_xy, e_yaw, p)


def test_guard_accepts_across_pi():
    core = g.GuardCore(g.Params())
    started(core, (2.0, 1.0, math.radians(178)), (0.0, 0.0, math.radians(178)))
    for k in range(1, 8):                                          # 0.5°씩 돌며 ±π 를 넘음
        yaw = g.wrap(math.radians(178 + 0.5 * k))
        d = core.update(10.0 + k * DT, (2.0, 1.0, yaw), (0.0, 0.0, yaw))
        assert d.action == 'accept', (k, d)
        assert d.err_yaw < math.radians(0.1)


def test_decide_jump_rules():
    p = g.Params()
    assert g.decide_jump(0.01, 0.05, 0.01, p) == 'accept_better'
    assert g.decide_jump(0.01, 0.012, 0.01, p) == 'refine'          # AMCL 이 조금(margin 이하)만 나으면 정합
    assert g.decide_jump(0.06, 0.05, 0.02, p) == 'refine'
    assert g.decide_jump(0.06, 0.05, 0.05, p) == 'hold'
    assert g.decide_jump(0.06, 0.05, None, p) == 'hold'
    assert g.decide_jump(None, None, None, p) == 'hold'


# ---------- 9. refine 성공은 give_up 시계를 멈춤(11-16 재현) ----------

def test_1116_wrong_amcl_3s_refine_keeps_truth(field):
    """AMCL 이 37 cm 틀린 채 3 s 머물며 떠다녀도 스캔이 참 자세를 받쳐 주면 give_up 없이 참 자세 유지"""
    p = g.Params()
    truth = (1.00, 2.50, 1.2)
    odom = (3.0, -1.0, 0.4)                    # 정지
    core = g.GuardCore(p, field)
    started(core, truth, odom, scan_from(field, truth))
    pts = scan_from(field, truth)
    acts, reinits = [], []
    for k in range(1, 31):                     # 3 s
        drift = 0.12 * k / 30                  # 정지 중 12 cm 표류(실물)
        amcl = (truth[0] - 0.30 + drift, truth[1] + 0.22, truth[2] - 0.03)
        d = core.update(10.0 + k * DT, amcl, odom, pts)
        acts.append(d.action)
        reinits.append(d.reinit)
        assert dist(d.out, truth) < 0.02 and ang(d.out, truth) < math.radians(1.5), (k, d)
    assert all(a == 'refine' for a in acts), acts
    assert reinits.count(True) == 1 and reinits.index(True) == p.reinit_after_n - 1


def test_reject_guard_confirmed_never_gives_up():
    p = g.Params()
    rg = g.RejectGuard(p)
    out = [rg.on_reject(5.0 + k * DT, confirmed=True) for k in range(50)]   # 5 s 내내 refine
    assert 'give_up' not in out
    assert out.count('reinit') == 1 and out[p.reinit_after_n - 1] == 'reinit'


def test_reject_guard_hold_only_gives_up_at_limit():
    p = g.Params()
    rg = g.RejectGuard(p)
    out = [rg.on_reject(5.0 + k * DT) for k in range(25)]
    assert out.index('give_up') in (19, 20, 21)


def test_reject_guard_hold_after_refine_restarts_clock():
    """refine 1.5 s → hold 1.5 s → refine 한 칸 → hold: 시계는 마지막 hold 시작부터 2 s"""
    p = g.Params()
    rg = g.RejectGuard(p)
    t = 5.0
    seq = [True] * 15 + [False] * 15 + [True] + [False] * 25
    res = []
    for c in seq:
        res.append((round(t, 3), rg.on_reject(t, confirmed=c)))
        t += DT
    gu = [tt for tt, r in res if r == 'give_up']
    assert len(gu) == 1, res
    hold2_start = 5.0 + 31 * DT
    assert hold2_start + p.max_reject_s - DT - 1e-6 <= gu[0] <= hold2_start + p.max_reject_s + DT + 1e-6
    # 첫 hold 구간(1.5 s)에서는 포기하지 않음
    assert all(r != 'give_up' for tt, r in res if tt < hold2_start)


def test_guard_mixed_refine_hold_restarts_clock(field):
    """GuardCore 통합: refine 성공 2.5 s 뒤 스캔이 없어 hold 로 바뀌면 그 시점부터 2 s 뒤 give_up"""
    p = g.Params()
    truth, odom = (1.00, 2.50, 1.2), (0.0, 0.0, 0.0)
    core = g.GuardCore(p, field)
    started(core, truth, odom, scan_from(field, truth))
    pts = scan_from(field, truth)
    amcl = (truth[0] - 0.30, truth[1] + 0.22, truth[2])
    for k in range(1, 26):
        assert core.update(10.0 + k * DT, amcl, odom, pts).action == 'refine'
    t_hold = 10.0 + 26 * DT
    k = 26
    while True:
        d = core.update(10.0 + k * DT, amcl, odom, None)
        if d.action == 'give_up':
            break
        assert d.action == 'hold' and dist(d.out, truth) < 0.02
        k += 1
        assert k < 80
    assert abs((10.0 + k * DT) - (t_hold + p.max_reject_s)) <= DT + 1e-6


# ---------- 10. refine 이 오래 이어지면 refine_long 경고 한 번(동작은 그대로) ----------

def test_refine_long_warns_once_behaviour_unchanged(field):
    p = g.Params()
    truth, odom = (1.00, 2.50, 1.2), (0.0, 0.0, 0.0)
    core = g.GuardCore(p, field)
    started(core, truth, odom, scan_from(field, truth))
    pts = scan_from(field, truth)
    amcl = (truth[0] - 0.30, truth[1] + 0.22, truth[2])
    longs = []
    for k in range(1, 131):                    # 13 s
        d = core.update(10.0 + k * DT, amcl, odom, pts)
        assert d.action == 'refine' and dist(d.out, truth) < 0.02, (k, d)
        if d.refine_long:
            longs.append(k)
    assert len(longs) == 1, longs
    assert abs((longs[0] - 1) * DT - p.max_refine_s) <= DT + 1e-6     # 첫 refine(k=1)부터 10 s
    # 받아들임으로 연속이 끊기면 경고 상태도 초기화 → 다음 연속에서 다시 한 번
    assert core.update(23.2, truth, odom, pts).action == 'accept'
    longs2 = [k for k in range(1, 121) if core.update(23.2 + k * DT, amcl, odom, pts).refine_long]
    assert len(longs2) == 1


# ---------- 11. 스캔 끊김 → active 는 map→odom 발행 멈춤(노드 tick) ----------

def _fake_node(mode, stale_s=1.0):
    rclpy = pytest.importorskip('rclpy')
    lg = pytest.importorskip('localization.loc_guard')
    from rclpy.time import Time
    n = object.__new__(lg.LocGuard)
    clock = {'t': 0.0}
    sent, events = [], []
    params = {'scan_stale_s': stale_s, 'scan_topic': '/scan_merged', 'pose_topic': '/amcl_pose', 'future_dt': 0.1}

    class Log:
        def error(self, *a, **k): pass
        def warning(self, *a, **k): pass
        def info(self, *a, **k): pass

    class Clock:
        def now(self):
            return Time(nanoseconds=int(clock['t'] * 1e9))

    class Br:
        def sendTransform(self, t): sent.append((clock['t'], t.transform.translation.x))

    n.g = params.__getitem__
    n.now_s = lambda: clock['t']
    n.lookup = lambda *a: None
    n.get_logger = lambda: Log()
    n.get_clock = lambda: Clock()
    n.event = lambda kind, **kw: events.append((clock['t'], kind, kw))
    n.mf, n.of, n.bf = 'map', 'odom', 'base_link'
    n.t_tick, n.last_odom, n.still = None, None, False
    n.last_pose_rx = None
    n.last_scan_rx, n.scan_stale = None, False
    n.blend = g.Blender(g.Params())
    n.blend.set_target((0.5, 0.0, 0.0))
    n.br = Br() if mode == 'active' else None
    return n, clock, sent, events


def _tick_at(n, clock, t):
    clock['t'] = t
    n.tick()


def test_scan_stale_stops_tf_in_active_and_resumes():
    n, clock, sent, events = _fake_node('active')
    n.last_scan_rx = 0.0
    for t in (0.05, 0.5, 0.95, 1.0):          # 1 s 이하 → 발행
        _tick_at(n, clock, t)
    assert [s[0] for s in sent] == [0.05, 0.5, 0.95, 1.0]
    for t in (1.05, 1.5, 2.5):                # 1 s 넘게 끊김 → 발행 멈춤, 사건 한 번
        _tick_at(n, clock, t)
    assert len(sent) == 4
    st = [e for e in events if e[1] == 'scan_stale']
    assert len(st) == 1 and st[0][0] == 1.05 and st[0][2]['stop_tf'] is True
    n.last_scan_rx = 2.6                      # 스캔 다시 옴
    _tick_at(n, clock, 2.65)
    assert sent[-1][0] == 2.65
    assert [e[1] for e in events].count('scan_resume') == 1


def test_scan_stale_shadow_only_logs():
    n, clock, sent, events = _fake_node('shadow')
    n.last_scan_rx = 0.0
    _tick_at(n, clock, 0.5)
    _tick_at(n, clock, 1.5)
    assert sent == []
    st = [e for e in events if e[1] == 'scan_stale']
    assert len(st) == 1 and st[0][2]['stop_tf'] is False


def test_no_scan_yet_publishes_nothing():
    """시작 뒤 스캔이 한 번도 안 왔으면 active 도 발행하지 않음"""
    n, clock, sent, events = _fake_node('active')
    for t in (0.05, 0.1, 3.0):
        _tick_at(n, clock, t)
    assert sent == []
