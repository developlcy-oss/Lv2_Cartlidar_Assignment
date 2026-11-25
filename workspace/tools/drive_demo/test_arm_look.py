"""arm_look.py 시험: cd workspace/tools/drive_demo && python3 -m pytest -q test_arm_look.py
순수 함수 (ROS 없이) + ROS 가 있으면 (source install/setup.bash) 가짜 팔 · 인식 노드로 ArmLook 한 번 돌려 봄 (없으면 건너뜀).
"""
import json
import math
import os
import threading
import time

import pytest

import arm_look as al
import box_face as bf

DEG = math.pi / 180.0


def status(x, y, yaw, ok=True, **kw):
    return al.parse_status(json.dumps(dict({"ok": ok, "frames": 3, "x": x, "y": y, "yaw": yaw, "spread_m": 0.001, "spread_deg": 0.2,
                                            "frame": "odom", "camera": "arm"}, **kw)))


# ---------- parse_status ----------
def test_parse_ok_and_fail():
    assert status(1, 2, 0)["ok"] is True
    f = al.parse_status('{"ok": false, "why": "윗면 못 찾음"}')
    assert f["ok"] is False and "윗면" in f["why"]
    assert al.parse_status("not json") is None and al.parse_status("[1]") is None and al.parse_status(None) is None


# ---------- 좌표 변환 ----------
def test_top_to_face_straight():
    """로봇 odom (2, 1) yaw 0 · 상자 윗면 중심 앞 0.380 (앞 끝 0.1328 + 틈 0.145 + 반 변 0.1025) → 앞면 중앙 x 0.2778 · 틈 0.145 · 옆 0 · 방향 0"""
    f, why = al.top_to_face(status(2.0 + 0.3803, 1.0, 0.0), (2.0, 1.0, 0.0), bf=bf)
    assert f is not None, why
    assert f.cx == pytest.approx(0.2778, abs=1e-3) and f.cy == pytest.approx(0.0, abs=1e-6)
    assert f.gap == pytest.approx(0.145, abs=1e-3) and f.lateral == pytest.approx(0.0, abs=1e-6) and abs(f.nyaw) < 1e-6
    assert f.ok and f.length == pytest.approx(0.205)


def test_top_to_face_rotated_robot_and_lateral():
    """로봇이 odom 에서 +90° 보고 있고 상자는 왼쪽 (로봇 기준 y +0.02) · 요는 odom 에서 90° 마다 같음 (여기선 0° = 로봇 기준 −90° ≡ 0°)"""
    oy = 90 * DEG
    # 로봇 기준 윗면 중심 (0.380, 0.02) → odom: (ox + c*0.380 - s*0.02, oy + s*0.380 + c*0.02)
    c, s = math.cos(oy), math.sin(oy)
    ox_, oy_ = 5.0, 3.0
    x, y = ox_ + c * 0.380 - s * 0.02, oy_ + s * 0.380 + c * 0.02
    f, why = al.top_to_face(status(x, y, 0.0), (ox_, oy_, oy), bf=bf)
    assert f is not None, why
    assert f.cy == pytest.approx(0.02, abs=1e-6) and f.cx == pytest.approx(0.380 - 0.1025, abs=1e-6)
    assert abs(f.nyaw) < 1e-6      # odom 요 0 = 로봇 기준 −90° → 90° 마다 같으니 0°
    assert f.lateral == pytest.approx(0.02, abs=1e-6)


@pytest.mark.parametrize("yaw_deg", [-40, -20, 5, 18, 40, 50, 95, 130, -100])
def test_normal_within_45(yaw_deg):
    ny = al.front_face_of_top(0.38, 0.0, yaw_deg * DEG)[2]
    assert abs(ny) <= math.pi / 4 + 1e-9
    assert abs(math.sin((ny - yaw_deg * DEG) * 2)) < 1e-9   # 90° 배수만 차이 (sin 2Δ = 0)


def test_front_face_is_half_side_toward_robot():
    fx, fy, ny = al.front_face_of_top(0.40, 0.03, 20 * DEG)
    # 앞면 중앙 → 윗면 중심 = 반 변 × 법선
    assert math.hypot(0.40 - fx, 0.03 - fy) == pytest.approx(0.1025, abs=1e-9)
    assert (0.40 - fx) / 0.1025 == pytest.approx(math.cos(ny)) and (0.03 - fy) / 0.1025 == pytest.approx(math.sin(ny))


def test_reject_far_from_expect():
    st = status(0.40, 0.0, 0.0)   # 앞면 중앙 x 0.2975
    f, why = al.top_to_face(st, (0, 0, 0), expect=(0.20, 0.0), bf=bf)   # 9.75 cm 차이 > 8
    assert f is None and "예상" in why
    f, why = al.top_to_face(st, (0, 0, 0), expect=(0.30, 0.01), bf=bf)
    assert f is not None


def test_reject_failed_status_and_nan():
    assert al.top_to_face(status(0, 0, 0, ok=False, why="흩어짐"), (0, 0, 0), bf=bf)[0] is None
    assert al.top_to_face(al.parse_status('{"ok": true, "x": 1}'), (0, 0, 0), bf=bf)[0] is None
    assert al.top_to_face(al.parse_status('{"ok": true, "x": NaN, "y": 0, "yaw": 0}'), (0, 0, 0), bf=bf)[0] is None
    assert al.top_to_face(None, (0, 0, 0), bf=bf)[0] is None
    assert al.top_to_face(status(0.4, 0, 0), None, bf=bf)[0] is None   # 오도메트리 없음


# ---------- 교차 확인 ----------
def mk(gap, lat, yaw_deg):
    f = bf.FaceResult(True, "", bf.FRONT_OFFSET + gap, lat, yaw_deg * DEG, 0.205)
    return bf._fill(f)


def test_cross_check_agree_and_disagree():
    c = al.cross_check(mk(0.145, 0.00, 0.0), mk(0.150, 0.01, 0.0))
    assert c["agree"] and c["dlat_cm"] == pytest.approx(1.0, abs=0.05)
    c = al.cross_check(mk(0.145, 0.00, 0.0), mk(0.145, 0.04, 0.0))
    assert not c["agree"] and "옆" in c["why"]
    c = al.cross_check(mk(0.145, 0.00, 0.0), mk(0.145, 0.0, 5.0))
    assert not c["agree"] and "방향" in c["why"]
    c = al.cross_check(mk(0.145, 0.00, 0.0), mk(0.200, 0.0, 0.0))
    assert not c["agree"] and "틈" in c["why"]


def test_prior_and_gap():
    p = al.prior_from_expect((0.2778, 0.0), 0.0)
    assert p == pytest.approx((0.3803, 0.0, 0.0))
    assert al.expected_gap((0.2778, 0)) == pytest.approx(0.145, abs=1e-3) and al.expected_gap(None) is None


# ---------- 자세별 측정 자리 · 허용 틈 · prior (11-22 밤: 기본 BOX_VIEW) ----------
def test_default_pose_is_box_view():
    assert al.ARM_POSE == "BOX_VIEW"


def test_pose_at():
    assert al.pose_at("BOX_VIEW") == "measure"   # 라이다 측정 자리 (틈 0.145) — 1 차 정지 없음
    assert al.pose_at("ALIGN") == "pre"          # 틈 0.30 1 차 정지
    with pytest.raises(ValueError):
        al.pose_at("NOPE")


def test_gap_ranges():
    assert al.pose_gap_range("BOX_VIEW") == (0.08, 0.20)
    assert al.pose_gap_range("ALIGN") == (0.20, 0.65)
    assert al.ARM_MIN_GAP == 0.20
    ok = lambda p, g: al.gap_skip_reason(p, g) == ""   # noqa: E731
    assert ok("BOX_VIEW", 0.145) and ok("BOX_VIEW", 0.107) and ok("BOX_VIEW", 0.08) and ok("BOX_VIEW", 0.20)
    assert not ok("BOX_VIEW", 0.07) and not ok("BOX_VIEW", 0.30)   # 0.30 = ALIGN 자리 — BOX_VIEW 에서는 안 보임
    assert ok("ALIGN", 0.30) and ok("ALIGN", 0.20) and ok("ALIGN", 0.65)
    assert not ok("ALIGN", 0.145) and not ok("ALIGN", 0.70)
    assert "모름" in al.gap_skip_reason("BOX_VIEW", None)
    assert "<" in al.gap_skip_reason("BOX_VIEW", 0.05) and ">" in al.gap_skip_reason("BOX_VIEW", 0.4)


def test_box_view_prior_at_measure_gap():
    # 틈 0.145 → 앞면 x = 0.1328 + 0.145 = 0.2778, 윗면 중심 x = + 0.1025 ≈ 0.38 (perception 기본 prior 0.3625 ± 0.08 안)
    e = (al.FRONT_X + 0.145, 0.0)
    p = al.prior_from_expect(e, 0.0)
    assert p[0] == pytest.approx(0.1328 + 0.145 + 0.1025, abs=1e-3) and abs(p[0] - 0.3625) <= 0.08
    # back 모드 정지점 틈 0.107 · 하한 0.08 에서도 기본 prior 0.3625 ± 0.08 안
    for g in (0.107, 0.08):
        assert abs(al.prior_from_expect((al.FRONT_X + g, 0.0), 0.0)[0] - 0.3625) <= 0.08
    # ALIGN 틈 0.30 이면 기본 prior 밖 (꼭 내야 함)
    assert abs(al.prior_from_expect((al.FRONT_X + 0.30, 0.0), 0.0)[0] - 0.3625) > 0.08


def test_armlook_rejects_unknown_pose():
    with pytest.raises(ValueError):
        al.ArmLook(None, threading.Event(), pose="NOPE")


# ---------- ROS: 가짜 팔 · 인식 ----------
def _ros():
    try:
        import rclpy  # noqa: F401
        from robot_interfaces.action import ArmPose  # noqa: F401
        return True
    except ImportError:
        return False


@pytest.mark.skipif(not _ros(), reason="ROS (rclpy · robot_interfaces) 없음 — source install/setup.bash")
@pytest.mark.parametrize("pose", ["BOX_VIEW", "ALIGN"])
@pytest.mark.parametrize("scenario", ["ok", "measure_fail", "pose_reject", "drive_stuck", "no_status"])
def test_ros_look(scenario, pose):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.action import ActionServer
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from robot_interfaces.action import ArmPose
    from robot_interfaces.srv import SetPerceptionMode
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    os.environ["ROS_DOMAIN_ID"] = "77"
    rclpy.init()
    log, prior_seen = [], []

    class Mock(Node):
        def __init__(self):
            super().__init__("mock_arm_perc")
            ActionServer(self, ArmPose, "/manipulator/pose", self.on_pose)
            self.create_service(SetPerceptionMode, "/perception/set_mode", self.on_mode)
            self.create_service(Trigger, "/perception/place_box/measure", self.on_meas)
            self.create_subscription(PoseStamped, "/perception/place_box/prior", lambda m: prior_seen.append(m), 10)
            self.pub = self.create_publisher(String, "/perception/place_box/status",
                                             QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            # 예전 (latched) 결과가 남아 있어도 안 쓰는지: 호출 전에 가짜 옛 결과를 내 둠
            self.pub.publish(String(data=json.dumps({"ok": True, "x": 99.0, "y": 99.0, "yaw": 0.0})))

        def on_pose(self, gh):
            name = gh.request.name
            log.append(name)
            res = ArmPose.Result()
            if scenario == "pose_reject" and name == pose:
                res.success, res.message = False, "start_tolerance"
                gh.abort()
                return res
            if scenario == "drive_stuck" and name == "DRIVE":
                res.success, res.message = False, "blocked"
                gh.abort()
                return res
            time.sleep(0.2)
            res.success = True
            gh.succeed()
            return res

        def on_mode(self, req, res):
            log.append(f"mode{req.mode}")
            res.success, res.message = True, "ok"
            return res

        def on_meas(self, req, res):
            log.append("measure")
            res.success, res.message = True, "시작"
            if scenario != "no_status":
                threading.Timer(0.5, self.reply).start()
            return res

        def reply(self):
            d = {"ok": False, "why": "윗면 못 찾음"} if scenario == "measure_fail" else \
                {"ok": True, "frames": 3, "x": 1.0 + 0.3803, "y": 2.0, "yaw": 0.0, "spread_m": 0.001, "spread_deg": 0.1, "frame": "odom", "camera": "arm"}
            self.pub.publish(String(data=json.dumps(d)))

    mock, client = Mock(), Node("test_client")
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(mock)
    ex.add_node(client)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        stop = threading.Event()
        holds = []
        if scenario == "no_status":
            al.MEASURE_T = 1.5
        a = al.ArmLook(client, stop, hold=lambda: holds.append(1), log=lambda m: None, pose=pose)
        ok, why = a.available(wait=5.0)
        assert ok, why
        odom = (1.0, 2.0, 0.0, time.time())
        if scenario == "drive_stuck":
            al.BACK_TRIES = 2
            with pytest.raises(al.ArmStuck):
                a.look(prior=(0.38, 0.0, 0.0), odom_fn=lambda: odom, expect=(0.2778, 0.0), bf=bf)
            assert log.count("DRIVE") == 2
            return
        rec, face = a.look(prior=(0.38, 0.0, 0.0), odom_fn=lambda: odom, expect=(0.2778, 0.0), bf=bf)
        assert log[0] == pose
        assert log[-1] == "DRIVE", log          # 어떤 경우에도 마지막은 DRIVE
        assert holds, "팔이 움직이는 동안 hold (바퀴 0 명령) 를 불러야 함"
        if scenario == "ok":
            assert face is not None and rec["ok"], rec
            assert face.gap == pytest.approx(0.145, abs=2e-3) and abs(face.lateral) < 2e-3
            assert "mode4" in log and "mode0" in log and log.index("mode4") < log.index("measure") < log.index("mode0")   # 켜고 → 재고 → 끔
            assert prior_seen, "사전값을 냈어야 함"
        else:
            assert face is None and not rec["ok"] and rec["why"], rec
    finally:
        ex.shutdown()
        rclpy.shutdown()


# ---------- 주행 내내 BOX_VIEW 유지 (ROS 없이: 팔 go 만 가짜) ----------
class _FakeArm(al.ArmLook):
    """ArmLook 의 raise_hold · fold · look · back_to_drive 는 진짜, go · set_mode · measure 만 가짜"""

    def __init__(self, pose="BOX_VIEW", fail=()):
        self.node, self.stop, self.hold, self.log, self.pose = None, threading.Event(), (lambda: None), (lambda m: None), pose
        self.err, self.status, self.up = "", None, False
        self.calls, self.fail = [], set(fail)

    def go(self, name, timeout=12.0):
        self.calls.append(name)
        return (name not in self.fail), ("blocked" if name in self.fail else "")

    def set_mode(self, mode, timeout=6.0):
        self.calls.append(f"mode{mode}")
        return True, "ok"

    def measure(self, prior=None, timeout=14.0):
        self.calls.append("measure")
        return {"ok": True, "frames": 3, "x": 1.3803, "y": 2.0, "yaw": 0.0, "spread_m": 0.001, "spread_deg": 0.1, "frame": "odom", "camera": "arm"}, ""


def test_pose_hold_flags():
    assert al.pose_hold("BOX_VIEW") is True and al.pose_hold("ALIGN") is False


def test_hold_raise_once_measure_without_pose_calls_fold_at_end(monkeypatch):
    a = _FakeArm()
    assert a.raise_hold()[0] and a.up and a.calls == ["DRIVE", "BOX_VIEW"]   # 11-23: STOW 에서는 DRIVE 먼저
    odom = (1.0, 2.0, 0.0, time.time())
    for _ in range(3):   # 상자 세 개 — 정지점마다 팔 왕복 없음
        rec, face = a.look(prior=(0.38, 0.0, 0.0), odom_fn=lambda: odom, expect=(0.2778, 0.0), bf=bf, move=False)
        assert rec["ok"] and face is not None
    assert a.calls.count("DRIVE") == 1 and a.calls.count("BOX_VIEW") == 1, a.calls   # 측정 때 자세 호출 없음 (DRIVE 는 펴기 전 한 번)
    assert a.calls.count("measure") == 3 and a.up
    assert a.fold() is True and a.calls[-1] == "DRIVE" and not a.up
    assert a.fold() is True and a.calls.count("DRIVE") == 2   # 이미 접힘 → 또 안 부름


def test_hold_raise_fail_marks_up_and_fold_returns_to_drive():
    a = _FakeArm(fail={"BOX_VIEW"})
    ok, msg = a.raise_hold()
    assert not ok and "blocked" in msg and a.up   # 움직였을 수 있어 접기 시도 대상
    assert a.fold() is True and a.calls == ["DRIVE", "BOX_VIEW", "DRIVE"]


def test_hold_fold_fail_three_tries(monkeypatch):
    monkeypatch.setattr(al, "BACK_TRIES", 3)
    monkeypatch.setattr(time, "sleep", lambda s: None)
    a = _FakeArm(fail={"DRIVE"})
    a.raise_hold()
    assert a.fold() is False and a.calls.count("DRIVE") == 4 and a.up   # 펴기 전 DRIVE 1 + 접기 3


def test_hold_raise_drive_fail_does_not_send_pose():
    a = _FakeArm(fail={"DRIVE"})
    ok, msg = a.raise_hold()
    assert not ok and "DRIVE" in msg and a.up and a.calls == ["DRIVE"]   # 11-23: DRIVE 못 가면 BOX_VIEW 안 보냄


def test_fold_works_even_if_ctrl_c_set():
    a = _FakeArm()
    a.raise_hold()
    a.stop.set()   # Ctrl+C
    assert a.fold() is True and a.calls[-1] == "DRIVE"
    assert a.stop.is_set()   # 원래 Event 복원


# ---------- demo_follow 쪽 흐름: arm_hold_start / arm_hold_end (가짜 노드) ----------
@pytest.fixture
def df(monkeypatch):
    import demo_follow as d
    monkeypatch.setattr(d, "ap_zero", lambda n: None)
    monkeypatch.setattr(d, "ap_cmd", lambda n, v, w: None)
    monkeypatch.setattr(d, "STEP_MODE", False)
    monkeypatch.setattr(d, "ARM", {"mode": "on", "ok": True, "why": "", "solo": False})
    monkeypatch.setattr(d, "ARM_POSE", "BOX_VIEW")
    monkeypatch.setattr(d, "al", al)
    d.STOP.clear()
    yield d
    d.STOP.clear()


class _N:
    pass


def _node(arm):
    n = _N()
    n.arm = arm
    return n


def test_df_start_raises_once_and_end_folds(df):
    a = _FakeArm()
    n = _node(a)
    df.arm_hold_start(n)
    assert a.calls == ["DRIVE", "BOX_VIEW"] and n.arm_held and df.ARM["mode"] == "on"
    df.arm_hold_end(n)
    assert a.calls == ["DRIVE", "BOX_VIEW", "DRIVE"] and not a.up


def test_df_end_folds_on_ctrl_c_but_not_on_safe_stop(df):
    a = _FakeArm()
    n = _node(a)
    df.arm_hold_start(n)
    df.STOP.set()   # Ctrl+C → 접음
    df.arm_hold_end(n)
    assert a.calls[-1] == "DRIVE"
    b = _FakeArm()
    n2 = _node(b)
    df.STOP.clear()
    df.arm_hold_start(n2)
    df.safe_stop(n2, "시험 접촉")   # 안전 정지 → 팔 안 움직임
    df.arm_hold_end(n2)
    assert b.calls == ["DRIVE", "BOX_VIEW"] and b.up


def test_df_raise_fail_turns_arm_look_off(df):
    a = _FakeArm(fail={"BOX_VIEW"})
    n = _node(a)
    df.arm_hold_start(n)
    assert df.ARM["mode"] == "off" and "못 폄" in df.ARM["why"]
    assert a.calls == ["DRIVE", "BOX_VIEW", "DRIVE"] and not a.up   # DRIVE 로 되돌림 시도
    df.arm_hold_end(n)
    assert a.calls == ["DRIVE", "BOX_VIEW", "DRIVE"]


def test_df_raise_fail_and_drive_fail_safe_stops(df, monkeypatch):
    monkeypatch.setattr(time, "sleep", lambda s: None)
    a = _FakeArm(fail={"BOX_VIEW", "DRIVE"})
    n = _node(a)
    df.arm_hold_start(n)
    assert df.ARM["mode"] == "off" and df.STOP.is_set() and n.safe_msg


def test_df_align_and_off_do_not_hold(df):
    df.ARM_POSE = "ALIGN"
    a = _FakeArm(pose="ALIGN")
    df.arm_hold_start(_node(a))
    assert a.calls == []
    df.ARM_POSE = "BOX_VIEW"
    df.ARM["mode"] = "off"
    df.arm_hold_start(_node(a))
    assert a.calls == []
    df.arm_hold_end(_node(a))   # 안 폈으니 접기도 없음
    assert a.calls == []
