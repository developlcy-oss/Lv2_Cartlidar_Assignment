"""위치 보호(loc_guard) 계산 — ROS 없음. loc_guard 노드가 씀. 잔차 · 국소 정합은 loc_core 를 그대로 씀.
자세는 모두 평면 (x, y, yaw[rad]), 시간 t 는 초.
판정(AMCL 자세가 올 때마다, GuardCore.update):
  예측 = 마지막 정상 map 자세 ∘ (그 뒤 EKF odom 이동). AMCL 과 예측 차이가 jump_xy · jump_yaw_deg 이하면 받아들임.
  넘으면(튐 후보) 스캔–지도 잔차를 비교: AMCL 쪽이 res_thr 이하이고 예측보다 res_margin 넘게 좋으면 받아들임(실제 밀림을 바로잡은 것).
  아니면 거부하고 예측에서 라이다 국소 정합(loc_core._refine) → 잔차 res_thr 이하면 정합 자세(refine), 아니면 예측 그대로(hold).
  거부가 이어질 때의 규칙(RejectGuard · GuardCore):
  - give_up 시계 = hold(AMCL · 정합 둘 다 나쁨)가 끊김 없이 이어진 시간만. max_reject_s 넘으면 AMCL 을 따름(give_up — 진짜로 미끄러진 경우 영원히 거부하지 않게).
  - refine 성공(정합 잔차 ≤ res_thr) 한 번이면 hold 시계를 0 으로. refine 이 이어지는 동안은 정합 자세를 계속 쓰고 give_up 하지 않음
    (실물 11-16: AMCL 이 37 cm 어긋난 채 머묾 — 정합이 받쳐 주면 계속 거부).
  - reinit_after_n: 연속 거부 수(hold · refine 모두)가 이 값이 될 때 한 번 다시 맞추기 요청(active + reinit_enabled 면 /initialpose).
  - refine_long: 연속 거부 안에서 첫 refine 부터 max_refine_s 넘게 지나면 경고 한 번(Decision.refine_long). 동작은 그대로 — AMCL 이 계속 다른 곳에 있다는 신호.
  - accept 계열 · give_up · reset 은 위 시계와 수를 모두 0 으로.
  wrap 은 [-π, π] (atan2 가 -π 를 돌려줄 수 있음).
  시작 · /initialpose 뒤 warmup_s 동안은 그대로 통과(초기 수렴 중 몇 cm · 10° 급변은 정상, 실물 11-15 · 11-16 기록).
섞기(Blender): 발행하는 map→odom 을 목표 쪽으로 blend_v · blend_w_deg 속도 제한으로 옮김(멈춤 중엔 still_scale 배)."""
import math
from dataclasses import dataclass

from localization import loc_core as lc


@dataclass
class Params:
    jump_xy: float = 0.05          # [m] AMCL 한 갱신당 odom 예측과의 차이 한계
    jump_yaw_deg: float = 3.0
    res_thr: float = 0.03          # [m] 잔차(거리장 중앙값) 정상 한계 — loc_monitor res_thr 와 같음
    res_margin: float = 0.005      # [m] 튐 후보 AMCL 을 받아들이려면 예측보다 이만큼 좋아야 함
    warmup_s: float = 2.0
    max_reject_s: float = 2.0      # [s] hold 만 이어진 시간 한계
    max_refine_s: float = 10.0     # [s] refine 이 이만큼 이어지면 refine_long 경고 한 번
    reinit_after_n: int = 10
    min_points: int = 30
    blend_v: float = 0.15          # [m/s]
    blend_w_deg: float = 15.0      # [deg/s]
    still_scale: float = 3.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def predict(last_map, last_odom, odom_now):
    """last_map ∘ (last_odom⁻¹ ∘ odom_now)"""
    return lc.se2_compose(last_map, lc.se2_compose(lc.se2_inverse(last_odom), odom_now))


def pose_error(pred, meas):
    """pred 기준 meas 의 차이 pred⁻¹ ∘ meas → (거리 [m], yaw [rad])"""
    d = lc.se2_compose(lc.se2_inverse(pred), meas)
    return math.hypot(d[0], d[1]), wrap(d[2])


def is_jump(err_xy, err_yaw, p):
    return err_xy > p.jump_xy or abs(err_yaw) > math.radians(p.jump_yaw_deg)


def map_odom_from(map_pose, odom_pose):
    """map→base 와 odom→base 로 map→odom = map_pose ∘ odom_pose⁻¹"""
    return lc.se2_compose(map_pose, lc.se2_inverse(odom_pose))


def rate_limit_step(cur, target, dt, max_v, max_w):
    """cur 에서 target 쪽으로 xy 는 max_v·dt, yaw 는 max_w·dt 까지만 (넘지 않으면 target 그대로)"""
    dx, dy = target[0] - cur[0], target[1] - cur[1]
    d = math.hypot(dx, dy)
    lim = max(0.0, max_v * dt)
    k = 1.0 if d <= lim or d == 0.0 else lim / d
    dyaw = wrap(target[2] - cur[2])
    alim = max(0.0, max_w * dt)
    dyaw = max(-alim, min(alim, dyaw))
    return (cur[0] + k * dx, cur[1] + k * dy, wrap(cur[2] + dyaw))


def decide_jump(r_amcl, r_pred, r_ref, p):
    """튐 후보 처리 → 'accept_better' | 'refine' | 'hold'"""
    if r_amcl is not None and r_pred is not None and r_amcl <= p.res_thr and r_amcl + p.res_margin < r_pred:
        return 'accept_better'
    if r_ref is not None and r_ref <= p.res_thr:
        return 'refine'
    return 'hold'


class Warmup:
    def __init__(self, warmup_s):
        self.warmup_s, self.t0 = warmup_s, None

    def start(self, t):
        self.t0 = t

    def active(self, t):
        return self.t0 is None or t - self.t0 < self.warmup_s


class RejectGuard:
    """연속 거부 상태기계"""
    def __init__(self, p):
        self.p = p
        self.on_accept()

    def on_accept(self):
        self.since, self.n = None, 0

    def on_reject(self, t, confirmed=False):
        """confirmed = 라이다 국소 정합이 잔차로 확인됨(refine). 확인된 거부는 give_up 시계를 멈춤 —
        AMCL 이 37 cm 어긋난 채 머물러도(실물 11-16) 스캔이 예측 쪽을 받쳐 주면 계속 거부. 확인 안 된 거부(hold)만 max_reject_s 를 셈."""
        self.n += 1
        if confirmed:
            self.since = None
        else:
            if self.since is None:
                self.since = t
            if t - self.since >= self.p.max_reject_s:
                self.on_accept()
                return 'give_up'
        return 'reinit' if self.n == self.p.reinit_after_n else 'reject'


class Blender:
    """발행할 map→odom 을 목표 쪽으로 속도 제한해 옮김"""
    def __init__(self, p):
        self.p, self.cur, self.target = p, None, None

    def set_target(self, T, snap=False):
        self.target = tuple(T)
        if snap or self.cur is None:
            self.cur = self.target

    def step(self, dt, still=False):
        if self.cur is None:
            return None
        k = self.p.still_scale if still else 1.0
        self.cur = rate_limit_step(self.cur, self.target, dt, self.p.blend_v * k, math.radians(self.p.blend_w_deg) * k)
        return self.cur


@dataclass
class Decision:
    action: str                    # init | warmup | accept | accept_better | refine | hold | give_up
    out: tuple                     # 내보낼 map 자세 (AMCL stamp 기준)
    pred: tuple = None
    err_xy: float = None
    err_yaw: float = None
    r_amcl: float = None
    r_pred: float = None
    r_ref: float = None
    reinit: bool = False
    refine_long: bool = False      # 이번 갱신에서 refine 이 max_refine_s 를 처음 넘음(경고 한 번)

    @property
    def rejected(self):
        return self.action in ('refine', 'hold')


class GuardCore:
    def __init__(self, p=None, field=None):
        self.p = p or Params()
        self.field = field
        self.warm = Warmup(self.p.warmup_s)
        self.rej = RejectGuard(self.p)
        self.last_good = None      # (map 자세, odom 자세)
        self.refine_since, self.refine_warned = None, False

    def _clear_refine(self):
        self.refine_since, self.refine_warned = None, False

    def reset(self, t):
        """시작 · /initialpose — 예열 다시, 마지막 정상 자세 지움"""
        self.warm.start(t)
        self.last_good = None
        self.rej.on_accept()
        self._clear_refine()

    def _res(self, pose, pts):
        return lc.residual(self.field, pose, pts)

    def _accept(self, action, amcl, odom, **kw):
        self.last_good = (tuple(amcl), tuple(odom))
        self.rej.on_accept()
        self._clear_refine()
        return Decision(action, tuple(amcl), **kw)

    def update(self, t, amcl, odom, pts=None):
        if self.warm.t0 is None:
            self.warm.start(t)
        if self.last_good is None:
            return self._accept('warmup' if self.warm.active(t) else 'init', amcl, odom)
        pred = predict(*self.last_good, odom)
        exy, eyaw = pose_error(pred, amcl)
        kw = dict(pred=pred, err_xy=exy, err_yaw=eyaw)
        if self.warm.active(t):
            return self._accept('warmup', amcl, odom, **kw)
        if not is_jump(exy, eyaw, self.p):
            return self._accept('accept', amcl, odom, **kw)
        # 튐 후보 — 잔차 · 국소 정합은 이때만(젯슨 CPU)
        r_amcl = r_pred = r_ref = None
        ref = None
        if self.field is not None and pts is not None and len(pts) >= self.p.min_points:
            r_amcl, r_pred = self._res(amcl, pts), self._res(pred, pts)
            kw.update(r_amcl=r_amcl, r_pred=r_pred)
            if decide_jump(r_amcl, r_pred, None, self.p) == 'accept_better':
                return self._accept('accept_better', amcl, odom, **kw)
            best, _ = lc._refine(self.field, pts, pred)
            ref = tuple(float(v) for v in best)
            r_ref = self._res(ref, pts)
            kw['r_ref'] = r_ref
        action = decide_jump(r_amcl, r_pred, r_ref, self.p)
        st = self.rej.on_reject(t, confirmed=(action == 'refine'))
        if st == 'give_up':
            self.last_good = (tuple(amcl), tuple(odom))
            self._clear_refine()
            return Decision('give_up', tuple(amcl), **kw)
        long_ = False
        if action == 'refine':
            out = ref
            self.last_good = (ref, tuple(odom))
            if self.refine_since is None:
                self.refine_since = t
            if not self.refine_warned and t - self.refine_since >= self.p.max_refine_s:
                self.refine_warned = long_ = True
        else:
            out = pred
        return Decision(action, out, reinit=(st == 'reinit'), refine_long=long_, **kw)
