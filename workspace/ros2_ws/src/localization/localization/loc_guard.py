"""위치 보호 노드(loc_guard). 계산은 loc_guard_core.py · loc_core.py.
AMCL 자세(/amcl_pose)가 순간적으로 튀면 EKF odom 예측 · 스캔–지도 잔차 · 라이다 국소 정합으로 거르고 보정한다.
mode:
  shadow(기본) — 판정 · 보정값 계산 · 기록만. /loc_guard/pose(보정된 map 자세), /loc_guard/event(jsonl), event_log 파일. TF 는 내지 않음(AMCL tf_broadcast true 그대로).
  active      — 위에 더해 map→odom 을 rate Hz · stamp now+future_dt 로 냄(AMCL tf_broadcast false 는 launch 가 맡음).
                reinit_enabled 면 연속 거부 reinit_after_n 번째에 /initialpose(보정 자세)로 AMCL 입자를 다시 뿌림.
  (off 는 launch 가 노드를 띄우지 않는 것.)
/initialpose 를 받으면(남이 보낸 것만) 예열을 다시 시작한다. 멈춤 중 AMCL 이 /amcl_pose 를 안 내도(update_min_d > 0) active 는 마지막 map→odom 을 계속 낸다
(pose 끊김은 경고만). 대신 scan_topic 이 scan_stale_s 넘게 안 오면 active 는 map→odom 발행을 멈춤(AMCL 도 스캔마다 TF 를 내므로 스캔이 끊기면
TF 가 낡아 Nav2 · loc_monitor 가 알아챔 — 같은 동작). 사건 scan_stale · scan_resume 은 shadow 에서도 기록."""
import collections
import json
import math
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from localization import loc_core as lc
from localization import loc_guard_core as gc


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def tf_pose(t):
    tr = t.transform
    return (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))


def r3(v):
    return None if v is None else round(float(v), 4)


class LocGuard(Node):
    def __init__(self):
        super().__init__('loc_guard')
        P = [('mode', 'shadow'), ('map', ''), ('pose_topic', '/amcl_pose'), ('scan_topic', '/scan_merged'),
             ('map_frame', 'map'), ('odom_frame', 'odom'), ('base_frame', 'base_link'),
             ('r_min', 0.15), ('r_max', 2.5), ('n_points', 180), ('min_points', 30),
             ('jump_xy', 0.05), ('jump_yaw_deg', 3.0), ('res_thr', 0.03), ('res_margin', 0.005),
             ('warmup_s', 2.0), ('max_reject_s', 2.0), ('max_refine_s', 10.0), ('scan_stale_s', 1.0),
             ('reinit_enabled', False), ('reinit_after_n', 10), ('reinit_xy_sd', 0.05), ('reinit_yaw_sd_deg', 5.0),
             ('blend_v', 0.15), ('blend_w_deg', 15.0), ('still_scale', 3.0),
             ('rate', 20.0), ('future_dt', 0.1), ('scan_max_dt', 0.06), ('event_log', '')]
        for k, v in P:
            self.declare_parameter(k, v)
        g = self.g = lambda k: self.get_parameter(k).value
        self.mode = g('mode')
        if self.mode not in ('shadow', 'active'):
            raise ValueError(f'mode {self.mode} — shadow | active (off 는 launch 에서 노드를 띄우지 않음)')
        self.p = gc.Params(jump_xy=g('jump_xy'), jump_yaw_deg=g('jump_yaw_deg'), res_thr=g('res_thr'), res_margin=g('res_margin'),
                           warmup_s=g('warmup_s'), max_reject_s=g('max_reject_s'), max_refine_s=g('max_refine_s'), reinit_after_n=g('reinit_after_n'),
                           min_points=g('min_points'), blend_v=g('blend_v'), blend_w_deg=g('blend_w_deg'), still_scale=g('still_scale'))
        field = lc.DistField(g('map')) if g('map') else None
        if field is None:
            self.get_logger().warning('map 이 비어 있음 — 잔차 · 국소 정합 없이 odom 예측만으로 판정 (튐은 hold → give_up)')
        self.core = gc.GuardCore(self.p, field)
        self.blend = gc.Blender(self.p)
        self.mf, self.of, self.bf = g('map_frame'), g('odom_frame'), g('base_frame')
        self.buf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.listener = tf2_ros.TransformListener(self.buf, self, spin_thread=True)
        self.scans = collections.deque(maxlen=10)      # (stamp ns, base_link 점)
        self.last_cov = None
        self.last_odom = None                          # (노드 시각 s, odom 자세) — 멈춤 판단
        self.still = False
        self.last_pose_rx = None
        self.last_scan_rx = None                       # 노드 시각 s — 스캔 끊김(scan_stale)
        self.scan_stale = False
        self.reinit_sent = False                       # 우리가 낸 /initialpose 는 예열을 다시 하지 않게
        self.n = collections.Counter()
        self.log_f = open(g('event_log'), 'a') if g('event_log') not in ('', 'none') else None
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/loc_guard/pose', 10)
        self.pub_ev = self.create_publisher(String, '/loc_guard/event', 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(PoseWithCovarianceStamped, g('pose_topic'), self.on_pose, 10)
        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)
        self.br = tf2_ros.TransformBroadcaster(self) if self.mode == 'active' else None
        self.t_tick = None
        self.create_timer(1.0 / g('rate'), self.tick)
        self.create_timer(10.0, self.report)
        self.event('start', mode=self.mode, reinit_enabled=g('reinit_enabled'), map=g('map'), jump_xy=self.p.jump_xy,
                   jump_yaw_deg=self.p.jump_yaw_deg, res_thr=self.p.res_thr)

    # ---- 공통 ----
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, kind, **kw):
        rec = {'t': round(self.now_s(), 3), 'event': kind, 'mode': self.mode} | kw
        s = json.dumps(rec, ensure_ascii=False)
        self.pub_ev.publish(String(data=s))
        if kind != 'accept':
            self.get_logger().info(s)
        if self.log_f:
            self.log_f.write(s + '\n')
            self.log_f.flush()

    def lookup(self, parent, child, stamp):
        """콜백 안이라 기다리지 않음(loc_monitor 와 같은 이유 — 젯슨에서 실행기가 막힘)"""
        try:
            return tf_pose(self.buf.lookup_transform(parent, child, stamp))
        except tf2_ros.TransformException:
            return None

    # ---- 입력 ----
    def on_scan(self, m):
        self.last_scan_rx = self.now_s()
        if m.header.frame_id != self.bf:       # /scan_merged 는 base_link 기준
            self.n['scan_frame'] += 1
            return
        pts = lc.scan_points(m.ranges, m.angle_min, m.angle_increment, [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
                             self.g('r_min'), self.g('r_max'), self.g('n_points'))
        self.scans.append((Time.from_msg(m.header.stamp).nanoseconds, pts))

    def pts_at(self, st, odom_st):
        """AMCL stamp 에 가장 가까운 스캔 점을 AMCL stamp base_link 로 옮김(odom 이동만큼)"""
        if not self.scans:
            return None
        ns, pts = min(self.scans, key=lambda s: abs(s[0] - st.nanoseconds))
        if abs(ns - st.nanoseconds) * 1e-9 > self.g('scan_max_dt'):
            return None
        if ns == st.nanoseconds:
            return pts
        o = self.lookup(self.of, self.bf, Time(nanoseconds=ns))
        if o is None:
            return None
        return lc.move_points(pts, lc.se2_compose(lc.se2_inverse(odom_st), o))

    def on_initialpose(self, m):
        if self.reinit_sent:
            self.reinit_sent = False
            return
        self.core.reset(self.now_s())
        self.event('reset', why='initialpose', pose=[r3(m.pose.pose.position.x), r3(m.pose.pose.position.y), r3(yaw_of(m.pose.pose.orientation))])

    def on_pose(self, m):
        self.last_pose_rx = self.now_s()
        st = Time.from_msg(m.header.stamp)
        odom = self.lookup(self.of, self.bf, st)
        if odom is None:
            self.n['tf_fail'] += 1
            return
        q = m.pose.pose
        amcl = (q.position.x, q.position.y, yaw_of(q.orientation))
        t0 = time.monotonic()
        d = self.core.update(st.nanoseconds * 1e-9, amcl, odom, self.pts_at(st, odom))
        wall_ms = (time.monotonic() - t0) * 1e3
        self.n[d.action] += 1
        self.blend.set_target(gc.map_odom_from(d.out, odom), snap=d.action in ('init', 'warmup', 'give_up'))
        out_mo = self.blend.cur
        out = lc.se2_compose(out_mo, odom)          # 섞은 map→odom 으로 본 map 자세(active 가 실제로 내는 값)
        self.publish_pose(m, out, d)
        if d.action != 'accept' or wall_ms > 50:
            self.event(d.action, amcl=[r3(v) for v in amcl], out=[r3(v) for v in d.out], blended=[r3(v) for v in out],
                       pred=None if d.pred is None else [r3(v) for v in d.pred],
                       err_xy=r3(d.err_xy), err_yaw_deg=None if d.err_yaw is None else round(math.degrees(d.err_yaw), 2),
                       r_amcl=r3(d.r_amcl), r_pred=r3(d.r_pred), r_ref=r3(d.r_ref), reinit=d.reinit, wall_ms=round(wall_ms, 1))
        if d.refine_long:
            self.event('refine_long', refine_s=self.p.max_refine_s, amcl=[r3(v) for v in amcl], out=[r3(v) for v in d.out], err_xy=r3(d.err_xy))
        if d.reinit and self.mode == 'active' and self.g('reinit_enabled'):
            self.send_initialpose(d.out, m.header.stamp)

    # ---- 출력 ----
    def publish_pose(self, m, out, d):
        o = PoseWithCovarianceStamped()
        o.header = m.header
        o.header.frame_id = self.mf
        o.pose.pose.position.x, o.pose.pose.position.y = out[0], out[1]
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(out[2] / 2), math.cos(out[2] / 2)
        cov = list(m.pose.covariance)
        if d.rejected and d.err_xy is not None:      # 거부 중이면 AMCL 과 벌어진 만큼 불확실하다고 알림
            cov[0] += d.err_xy ** 2
            cov[7] += d.err_xy ** 2
            cov[35] += d.err_yaw ** 2
        o.pose.covariance = cov
        self.pub_pose.publish(o)

    def send_initialpose(self, pose, stamp):
        m = PoseWithCovarianceStamped()
        m.header.stamp, m.header.frame_id = stamp, self.mf
        m.pose.pose.position.x, m.pose.pose.position.y = pose[0], pose[1]
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        sxy, syaw = self.g('reinit_xy_sd'), math.radians(self.g('reinit_yaw_sd_deg'))
        m.pose.covariance[0] = m.pose.covariance[7] = sxy * sxy
        m.pose.covariance[35] = syaw * syaw
        self.reinit_sent = True
        self.pub_init.publish(m)
        self.event('initialpose', pose=[r3(v) for v in pose])

    def tick(self):
        now = self.now_s()
        dt = 0.0 if self.t_tick is None else min(now - self.t_tick, 0.5)
        self.t_tick = now
        o = self.lookup(self.of, self.bf, Time())
        if o is not None:
            if self.last_odom is not None and now > self.last_odom[0]:
                ex, ey = o[0] - self.last_odom[1][0], o[1] - self.last_odom[1][1]
                w = abs(gc.wrap(o[2] - self.last_odom[1][2]))
                el = now - self.last_odom[0]
                self.still = math.hypot(ex, ey) / el < 0.01 and w / el < math.radians(1.0)
            self.last_odom = (now, o)
        cur = self.blend.step(dt, self.still)
        age = None if self.last_scan_rx is None else now - self.last_scan_rx
        stale = age is None or age > self.g('scan_stale_s')
        if stale != self.scan_stale and (age is not None or cur is not None):   # 시작 직후(스캔 · 자세 모두 없음)는 사건 없이
            self.scan_stale = stale
            if stale:
                self.event('scan_stale', age_s=None if age is None else round(age, 2), stop_tf=self.br is not None)
                self.get_logger().error(f'{self.g("scan_topic")} 끊김' + (' — map→odom 발행 멈춤' if self.br is not None else ''))
            else:
                self.event('scan_resume')
        if self.br is None or cur is None or stale:
            return
        if self.last_pose_rx is not None and now - self.last_pose_rx > 5.0:
            self.get_logger().warning(f'{self.g("pose_topic")} 가 {now - self.last_pose_rx:.0f} s 없음 — 마지막 map→odom 을 계속 냄',
                                      throttle_duration_sec=10.0)
        t = TransformStamped()
        t.header.stamp = (self.get_clock().now() + Duration(seconds=self.g('future_dt'))).to_msg()
        t.header.frame_id, t.child_frame_id = self.mf, self.of
        t.transform.translation.x, t.transform.translation.y = cur[0], cur[1]
        t.transform.rotation.z, t.transform.rotation.w = math.sin(cur[2] / 2), math.cos(cur[2] / 2)
        self.br.sendTransform(t)

    def report(self):
        if self.n:
            self.event('report', counts=dict(self.n))
            self.n.clear()


def main():
    rclpy.init()
    n = LocGuard()
    ex = SingleThreadedExecutor()
    ex.add_node(n)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():
            raise


if __name__ == '__main__':
    main()
