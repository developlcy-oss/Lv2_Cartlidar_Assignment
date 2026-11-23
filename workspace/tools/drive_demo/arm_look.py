#!/usr/bin/env python3
"""11-22 (밤) 팔 카메라 (Arducam) 로 상자 재기 — 주행 모드에서 상자 앞 1 차 정지 자리 (틈 약 0.30 m) 에 멈췄을 때 팔을 ALIGN 으로 펴
상자 윗면을 보고, 그 값을 앞 라이다 상자 앞면 (box_face.py) 과 같은 꼴 (FaceResult: base_link 틈 · 옆 · 방향) 로 바꿈. demo_follow 가 부름.
11-22 밤 (사용자 결정 2): BOX_VIEW 는 **주행 내내 유지** — 자율 시작 전 DRIVE → BOX_VIEW 한 번 (raise_hold), 끝에 접음 (fold). 측정 자리에서는 팔을 안 움직이고 카메라만 켜서 잼 (look(move=False)).
11-23 (사용자 결정 3, 기본): **상자 구간에서만 BOX_VIEW** — demo_follow ArmZone 이 정지점 0.40 m 안에 들어가면 주행 중 (바퀴 안 멈춤) BOX_VIEW, 벗어나면 DRIVE_LOW 로 접음.
  이 모듈의 ArmLook 을 hold=None 으로 따로 만들어 씀 (바퀴 0 명령 안 냄). --no-arm-zone = 위 11-22 방식 (주행 내내 유지).
11-22 밤 (사용자 결정): 측정 자세를 고를 수 있음 (ARM_LOOK_POSE) — 기본 **BOX_VIEW** (바로 아래 약 89.5° 를 봄, 미션 box_look 과 같은 자세, perception 기본 prior 도 이 자세용).
  · BOX_VIEW = 1 차 정지 없이 **라이다 측정 자리 (틈 0.145)** 에서 라이다 측정 뒤 팔 BOX_VIEW → 재기 → DRIVE → S 자. 허용 틈 0.10~0.20.
    윗면이 보이는 범위 base x 0.16~0.33 (윗면 **일부만**, 전체는 안 들어옴 → 부정확할 수 있음), 팔 앞 끝 x ≈ 0.318 → 틈 0.145 에서 상자 위로 약 4 cm 걸침, 팔 최저 z ≈ 0.18 (상자 윗면 0.07 위 ·
    상자 위 퍽 높이 여유는 **미확인**) — 설계값, 실물 미확인. 벤치 확인 먼저 (README).
  · ALIGN = 틈 0.30 1 차 정지에서 (윗면 전체가 보임, 허용 틈 0.20~0.65). 예전 동작 그대로.

기존 인터페이스 재사용 (새 인터페이스 없음. 미션 center_box 와 같은 흐름 ALIGN → measure):
  /manipulator/pose        (robot_interfaces/action/ArmPose, name = ALIGN · DRIVE)   — 팔 이름 자세
  /perception/set_mode     (robot_interfaces/srv/SetPerceptionMode, READ_TAG = 4)       — 팔 카메라 켬 (perception 은 READ_TAG · ARM_ALIGN 일 때만 잼)
  /perception/place_box/prior   (PoseStamped, 선택)                                     — 사전값 (상자 윗면 중심 · 요). 없으면 perception.yaml prior [0.3625, 0] ± 0.08
  /perception/place_box/measure (std_srvs/Trigger)                                      — 측정 시작 (success = 시작함, 결과는 아래 status)
  /perception/place_box/status  (std_msgs/String JSON, latched)                         — {"ok": true, "x", "y", "yaw", "spread_m", "spread_deg", "frame": "odom", ...}
                                                                                          또는 {"ok": false, "why": ...}. x · y = 상자 **윗면** 중심 (odom), yaw = 윗면 요 (90° 마다 같음)
  → 상자 앞면 = 윗면 중심에서 법선 반대로 반 변 (0.1025) 안쪽. 법선 = 윗면 요 + k × 90° 중 로봇 정면에 가장 가까운 것 (|nyaw| ≤ 45°)

이 모듈의 순수 함수 (ROS 없이 시험: test_arm_look.py) = parse_status · top_to_face · cross_check · prior_from_expect.
ArmLook 클래스가 ROS 호출 (ROS 모듈은 늦게 불러서 없는 PC 에서도 import 됨).

안전 규칙 (실물 미시험 — 팔은 시뮬 · 노트북에서 못 움직여 봄):
  · hold 콜백을 주면 팔이 움직이는 동안 바퀴 0 명령을 계속 냄 (부르는 쪽이 정지 확인 — ALIGN · 11-22 BOX_VIEW 방식).
    hold=None 이면 바퀴를 안 멈춤 = 주행 중 팔 이동 (11-23 ArmZone, "바퀴가 서 있을 때만" 규칙은 사용자 결정으로 없앰).
    실물 왕복 (11-23, 바퀴 정지 상태): DRIVE ↔ BOX_VIEW · DRIVE_LOW ↔ BOX_VIEW 각 20 회 성공, 이동당 1.0~1.07 s. 주행 중 이동은 실물 미시험.
  · 팔을 편 뒤에는 **반드시** DRIVE 로 되돌림 (3 번 시도). 못 되돌리면 ArmStuck → 부르는 쪽이 전체를 안전 정지 (바퀴 안 움직임).
  · 예상 틈이 자세별 허용 범위 (POSE_CFG min_gap ~ max_gap) 밖이면 팔을 올리지 않음.
"""
import json
import math
import time

SIDE = 0.205          # 상자 한 변 [m] (perception arm.place_box.box)
HEIGHT = 0.07         # 윗면 높이 (참고)
MAX_SHIFT = 0.08      # 윗면 중심이 예상 (계획 · 라이다) 에서 이보다 멀면 믿지 않음 [m] (mission_plan center.max_shift)
YAW_MAX = math.radians(25.0)   # 앞면 법선이 정면에서 이보다 틀면 믿지 않음 (box_approach AP_YAW_MAX 와 같게)
# 라이다 ↔ 팔 카메라 교차 확인 허용 (drive.yaml sensor_disagree 0.02 · fuse yaw σ: 라이다 5 장 1.5° / Arducam 0.4°)
#   팔 기구학 RMS 19~20 mm (manipulator.yaml) 라 옆 · 틈은 3 cm, 방향은 3° — 큰 오검출 (엉뚱한 사각형) 만 거르는 용도. 가중 평균 · 칼만 결합은 안 함
AGREE_LAT, AGREE_GAP, AGREE_YAW = 0.03, 0.03, math.radians(3.0)
ARM_POSE = "BOX_VIEW"   # 기본 측정 자세 (사용자 결정 11-22 밤). manipulator.yaml poses.BOX_VIEW = [0, -0.65, 0, 1.95, 0, 0]
# 자세별: at = 측정 자리 ("measure" = 라이다 측정 자리 틈 0.145 · "pre" = 틈 0.30 1 차 정지), 팔을 펴도 되는 예상 틈 범위 [m] (밖이면 안 폄), 설명
#   ALIGN: 윗면을 앞 · 아래 41.9° 로 봄 (= FOLD_VIA, 팔 최저 z 0.178 m, 앞으로 2~7 cm, /scan_front 안 가림 — 계산). 윗면 전체가 보이는 틈 ≈ 0.19 ~ 0.65
#   BOX_VIEW: 바로 아래 · 윗면 base x 0.16~0.33 ↔ 틈 0.03~0.20 (윗면 일부만). 팔 앞 끝 x ≈ 0.318
POSE_CFG = {
    "ALIGN": {"at": "pre", "min_gap": 0.20, "max_gap": 0.65, "hold": False},
    "BOX_VIEW": {"at": "measure", "min_gap": 0.08, "max_gap": 0.20, "hold": True},   # hold = 측정 때 팔을 안 움직임 (이미 이 자세 — 11-22 주행 내내 유지 · 11-23 ArmZone 이 상자 구간에서 폄)
}
ARM_MIN_GAP = POSE_CFG["ALIGN"]["min_gap"]   # (예전 이름 유지)
FRONT_X = 0.1328      # 상판 앞 끝 base_link x [m]
POSE_T = 12.0         # 팔 자세 한 번 상한 [s] (mission_plan center.arm_pose_s 10)
MODE_T = 6.0          # set_mode 응답 상한 [s]
MEASURE_T = 14.0      # 측정 결과 (영상 3 장, Jetson 장당 약 3 s) 상한 [s] (center.measure_timeout_s 12 + 여유)
SETTLE_T = 0.8        # 팔 자세 뒤 재기 전 기다림 [s] (center.settle_s)
READ_TAG = 4          # SetPerceptionMode 비트
BACK_TRIES = 3        # DRIVE 복귀 시도 수


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ArmStuck(Exception):
    """측정 자세로 올린 팔을 DRIVE 로 못 되돌림 — 부르는 쪽이 전체를 멈춰야 함"""


# ───────────────── 순수 함수 (ROS 없음) ─────────────────

def parse_status(text):
    """/perception/place_box/status JSON → dict (ok 키 보장) 또는 None (JSON 아님)"""
    try:
        d = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(d, dict):
        return None
    d["ok"] = bool(d.get("ok", False))
    return d


def to_base(odom_pose, x, y, yaw):
    """odom 의 (x, y, yaw) → base_link. odom_pose = (ox, oy, oyaw) — 로봇이 서 있는 동안의 odom 자세"""
    ox, oy, oyaw = odom_pose[0], odom_pose[1], odom_pose[2]
    c, s = math.cos(oyaw), math.sin(oyaw)
    dx, dy = x - ox, y - oy
    return c * dx + s * dy, -s * dx + c * dy, wrap(yaw - oyaw)


def front_face_of_top(cx, cy, yaw, side=SIDE):
    """상자 윗면 중심 (cx, cy) · 요 yaw (모두 base_link) → 앞면 중앙 (x, y) 과 법선 nyaw (로봇 → 상자, |nyaw| ≤ 45°)"""
    ny = wrap(((yaw + math.pi / 4) % (math.pi / 2)) - math.pi / 4)   # 정사각형 윗면 요는 90° 마다 같음 → 정면에 가장 가까운 것
    return cx - 0.5 * side * math.cos(ny), cy - 0.5 * side * math.sin(ny), ny


def top_to_face(status, odom_pose, expect=None, side=SIDE, bf=None):
    """측정 상태 (parse_status 결과) + 지금 odom 자세 → (FaceResult 꼴, 이유 "").  못 쓰면 (None, 이유).
    expect = 계획 · 라이다 예상 앞면 중앙 (base_link x, y) — 윗면 중심과 이 값 차이가 MAX_SHIFT 넘으면 거부 (None = 안 봄).
    bf = box_face 모듈 (FaceResult · _fill 을 씀. None 이면 import)"""
    if status is None:
        return None, "상태를 못 읽음"
    if not status.get("ok"):
        return None, str(status.get("why") or "팔 카메라 측정 실패 (ok false)")
    try:
        x, y, yaw = float(status["x"]), float(status["y"]), float(status["yaw"])
    except (KeyError, TypeError, ValueError):
        return None, "상태에 x · y · yaw 없음"
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        return None, "상태 값이 유한하지 않음"
    if odom_pose is None:
        return None, "오도메트리 없음"
    if bf is None:
        import box_face as bf
    bx, by, byaw = to_base(odom_pose, x, y, yaw)
    fx, fy, ny = front_face_of_top(bx, by, byaw, side)
    if expect is not None and math.hypot(fx - expect[0], fy - expect[1]) > MAX_SHIFT:
        return None, (f"윗면에서 낸 앞면 중앙 ({fx:.3f}, {fy:.3f}) 이 예상 ({expect[0]:.3f}, {expect[1]:.3f}) 에서 "
                      f"{math.hypot(fx - expect[0], fy - expect[1]) * 100:.1f} cm > {MAX_SHIFT * 100:.0f} cm")
    f = bf.FaceResult(True, "", fx, fy, ny, side)   # n_pts · rms 는 0 (라이다 점이 아님)
    f = bf._fill(f)
    f.why = "팔 카메라"
    return f, ""


def cross_check(lidar, arm):
    """라이다 앞면 ↔ 팔 카메라 앞면 (둘 다 base_link FaceResult 꼴) → dict(gap, lat, yaw 차 · agree · why).  gap · lat = arm − lidar [m], yaw [rad]"""
    dg, dl = arm.gap - lidar.gap, arm.lateral - lidar.lateral
    dy = wrap(arm.nyaw - lidar.nyaw)
    bad = []
    if abs(dl) > AGREE_LAT:
        bad.append(f"옆 {dl * 100:+.1f} cm (> {AGREE_LAT * 100:.0f})")
    if abs(dg) > AGREE_GAP:
        bad.append(f"틈 {dg * 100:+.1f} cm (> {AGREE_GAP * 100:.0f})")
    if abs(dy) > AGREE_YAW:
        bad.append(f"방향 {math.degrees(dy):+.1f}° (> {math.degrees(AGREE_YAW):.0f}°)")
    return {"dgap_cm": round(dg * 100, 1), "dlat_cm": round(dl * 100, 1), "dyaw_deg": round(math.degrees(dy), 1),
            "agree": not bad, "why": " · ".join(bad)}


def prior_from_expect(expect_face, rel_yaw=0.0, side=SIDE):
    """예상 앞면 중앙 (base_link x, y) → 사전값 윗면 중심 (x, y, yaw). rel_yaw = 예상 상자 방향 − 로봇 방향 (계획 정지점 방향 기준)"""
    return (expect_face[0] + 0.5 * side * math.cos(rel_yaw), expect_face[1] + 0.5 * side * math.sin(rel_yaw), rel_yaw)


def expected_gap(expect_face, front=FRONT_X):
    """예상 앞면 중앙 x → 앞 끝 ↔ 상자 틈 (정면 가정)"""
    return None if expect_face is None else expect_face[0] - front


def pose_gap_range(pose):
    """자세 → (최소 틈, 최대 틈) [m]. 모르는 자세는 ValueError"""
    c = POSE_CFG.get(pose)
    if c is None:
        raise ValueError(f"모르는 팔 측정 자세 {pose!r} (ALIGN · BOX_VIEW)")
    return c["min_gap"], c["max_gap"]


def pose_at(pose):
    """자세 → 측정 자리 ("measure" = 라이다 측정 자리 틈 0.145 · "pre" = 틈 0.30 1 차 정지)"""
    pose_gap_range(pose)
    return POSE_CFG[pose]["at"]


def pose_hold(pose):
    """자세 → 주행 중 계속 유지하나 (BOX_VIEW True = 자율 시작 전 DRIVE → 자세 한 번 · 끝에 접음 / ALIGN False = 측정 때만 폄)"""
    pose_gap_range(pose)
    return POSE_CFG[pose]["hold"]


def gap_skip_reason(pose, gap):
    """예상 틈 gap 에서 이 자세로 팔을 펴도 되나 → 안 되면 이유 문자열, 되면 "" """
    if gap is None:
        return "예상 틈을 모름"
    lo, hi = pose_gap_range(pose)
    if gap < lo:
        return f"예상 틈 {gap * 100:.1f} cm < {lo * 100:.0f} cm ({pose} 에서 윗면이 안 보임 · 상자에 너무 가까움)"
    if gap > hi:
        return f"예상 틈 {gap * 100:.1f} cm > {hi * 100:.0f} cm ({pose} 에서 윗면이 안 보임 · 상자가 멀음)"
    return ""


# ───────────────── ROS ─────────────────

class ArmLook:
    """팔 BOX_VIEW (또는 ALIGN) → 측정 → DRIVE.  node = rclpy 노드 (이미 다른 스레드에서 spin 중이어야 함), stop = threading.Event (Ctrl+C),
    hold = 팔이 움직이는 동안 주기적으로 부를 함수 (바퀴 0 명령), log = print 함수"""

    def __init__(self, node, stop, hold=None, log=print, pose=ARM_POSE):
        pose_gap_range(pose)   # 모르는 자세면 바로 ValueError
        self.node, self.stop, self.hold, self.log, self.pose = node, stop, hold or (lambda: None), log, pose
        self.err = ""
        self.status = None     # (받은 벽시계, 파싱한 dict)
        self.up = False        # 자세 (BOX_VIEW · ALIGN) 로 폈고 DRIVE 확인 전
        try:
            from rclpy.action import ActionClient
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from geometry_msgs.msg import PoseStamped
            from robot_interfaces.action import ArmPose
            from robot_interfaces.srv import SetPerceptionMode
            from std_msgs.msg import String
            from std_srvs.srv import Trigger
        except ImportError as e:
            self.err = f"ROS 인터페이스 못 불러옴 ({e})"
            return
        self._ArmPose, self._SetMode, self._Pose = ArmPose, SetPerceptionMode, PoseStamped
        self.pose_cli = ActionClient(node, ArmPose, "/manipulator/pose")
        self.mode_cli = node.create_client(SetPerceptionMode, "/perception/set_mode")
        self.meas_cli = node.create_client(Trigger, "/perception/place_box/measure")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 11-23 실물 1641: perception 구독이 TRANSIENT_LOCAL 이라 VOLATILE 발행은 안 맞아 사전값이 한 번도 안 갔음 → 같은 latched 로
        self.prior_pub = node.create_publisher(PoseStamped, "/perception/place_box/prior", latched)
        node.create_subscription(String, "/perception/place_box/status", self._on_status, latched)

    def _on_status(self, m):
        self.status = (time.time(), parse_status(m.data))

    # --- 기다림 도우미: Ctrl+C · 시간 상한 · hold ---
    def _wait(self, fut, timeout):
        t0, last = time.time(), 0.0
        while not fut.done():
            if self.stop.is_set():
                return False
            if time.time() - t0 > timeout:
                return False
            if time.time() - last > 0.2:
                self.hold()
                last = time.time()
            time.sleep(0.02)
        return True

    def available(self, wait=3.0):
        """(쓸 수 있나, 이유). 팔 액션 · set_mode · measure 서비스가 보이는지"""
        if self.err:
            return False, self.err
        missing = []
        if not self.pose_cli.wait_for_server(timeout_sec=wait):
            missing.append("/manipulator/pose")
        if not self.mode_cli.wait_for_service(timeout_sec=1.0):
            missing.append("/perception/set_mode")
        if not self.meas_cli.wait_for_service(timeout_sec=1.0):
            missing.append("/perception/place_box/measure")
        return (not missing), ("없음: " + " · ".join(missing)) if missing else ""

    def go(self, name, timeout=POSE_T):
        """팔 이름 자세로 이동 → (성공, 메시지)"""
        goal = self._ArmPose.Goal()
        goal.name = name
        fut = self.pose_cli.send_goal_async(goal)
        if not self._wait(fut, 5.0):
            return False, "팔 goal 응답 없음 (Ctrl+C 이거나 5 s)"
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False, f"팔 {name} goal 거절 (켜질 때 정리 중 · 안전 검사?)"
        rf = gh.get_result_async()
        if not self._wait(rf, timeout):
            try:
                gh.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            return False, f"팔 {name} {timeout:.0f} s 안에 안 끝남"
        r = rf.result().result
        return bool(r.success), (r.message or "")

    def set_mode(self, mode, timeout=MODE_T):
        req = self._SetMode.Request()
        req.mode = int(mode)
        fut = self.mode_cli.call_async(req)
        if not self._wait(fut, timeout):
            return False, "set_mode 응답 없음"
        r = fut.result()
        return bool(r.success), r.message

    def publish_prior(self, prior, frame="base_link"):
        """prior = (x, y, yaw) 윗면 중심 · 요, frame 기준 (perception 이 base_footprint 로 옮김)"""
        m = self._Pose()
        m.header.frame_id = frame
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = float(prior[0]), float(prior[1])
        m.pose.orientation.z, m.pose.orientation.w = math.sin(prior[2] / 2), math.cos(prior[2] / 2)
        self.prior_pub.publish(m)

    def measure(self, prior=None, timeout=MEASURE_T):
        """사전값 → measure 호출 → 새 status 를 기다림. (status dict 또는 None, 이유)"""
        if prior is not None:
            self.publish_prior(prior)
            t = time.time()
            while time.time() - t < 0.3 and not self.stop.is_set():
                self.hold()
                time.sleep(0.05)
        from std_srvs.srv import Trigger
        t_call = time.time()
        fut = self.meas_cli.call_async(Trigger.Request())
        if not self._wait(fut, 6.0):
            # 11-23 실물 1641: perception 은 요청을 곧 처리했는데 (윗면 못 찾음 1.8 s) 여기선 응답이 안 옴 — 다음 주행에서 어디서 막히는지 가리려고 기록만 더함
            ready = self.meas_cli.service_is_ready()
            fut.add_done_callback(lambda f, t=t_call: self.log(
                f"    · (진단) 늦은 measure 응답 {time.time() - t:.1f} s 뒤: " + (f.result().message if f.result() is not None else "결과 없음")))
            return None, f"measure 응답 없음 (6 s · 서비스 {'보임' if ready else '안 보임'})"
        r = fut.result()
        if not r.success:
            return None, f"measure 거절: {r.message}"
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.stop.is_set():
                return None, "Ctrl+C"
            st = self.status
            if st is not None and st[0] >= t_call:   # 호출 뒤에 온 것만 (latched 로 남은 예전 결과는 안 씀)
                return st[1], ""
            self.hold()
            time.sleep(0.05)
        return None, f"측정 결과 {timeout:.0f} s 안에 안 옴"

    def back_to_drive(self):
        """DRIVE 로 되돌림 (최대 BACK_TRIES 번). 성공 True. Ctrl+C 면 False (팔 위치 모름)"""
        why = ""
        for k in range(BACK_TRIES):
            if self.stop.is_set():
                self.log("    ⚠ Ctrl+C — 팔이 펴진 채일 수 있음 (직접 확인)")
                return False
            ok, why = self.go("DRIVE")
            if ok:
                self.up = False
                return True
            self.log(f"    ⚠ 팔 DRIVE 복귀 {k + 1}/{BACK_TRIES} 실패: {why}")
            t = time.time()
            while time.time() - t < 2.0 and not self.stop.is_set():
                self.hold()
                time.sleep(0.05)
        return False

    def raise_hold(self):
        """(hold 자세) 자율 시작 전 바퀴 0 명령 상태에서 DRIVE → self.pose 한 번. → (성공, 메시지). 실패하면 팔 상태가 불확실 → up 으로 표시 (부르는 쪽이 fold 시도)"""
        t = time.time()
        # 11-23 실물 (1122_2252·2311): STOW 에서 바로 BOX_VIEW 는 manipulator 가 거절 ("arm is at STOW -> send pose 'DRIVE' first (staged unfold)")
        #   → DRIVE 먼저 (STOW 면 단계 펴기, 이미 DRIVE 면 곧 끝남) 뒤 self.pose
        ok, msg = self.go("DRIVE")
        self.up = True   # 실패해도 움직였을 수 있음
        if not ok:
            return False, f"DRIVE 로 못 폄: {msg}"
        ok, msg = self.go(self.pose)
        if ok:
            return True, f"{self.pose} 로 폄 ({time.time() - t:.0f} s)"
        return False, msg

    def fold(self):
        """끝에서 (바퀴 정지 상태) self.pose → DRIVE.  Ctrl+C (self.stop 이 set) 여도 접도록 새 Event 로 기다림. 성공 True (이미 접혀 있어도 True)"""
        if not self.up:
            return True
        import threading
        orig, self.stop = self.stop, threading.Event()
        try:
            return self.back_to_drive()
        finally:
            self.stop = orig

    def look(self, prior=None, odom_fn=None, expect=None, bf=None, move=True):
        """팔을 self.pose (BOX_VIEW · ALIGN) 로 → 켜기 → 재기 → 끄기 → DRIVE 복귀. 기록 dict (rec) 와 FaceResult 꼴 (또는 None) 을 돌려줌.
        move=False: 팔을 움직이지 않고 (raise_hold 로 이미 자세를 유지 중) 켜기 → 재기 → 끄기만 — 자세 호출 · DRIVE 복귀 없음.
        DRIVE 로 못 되돌리면 ArmStuck. odom_fn() = 지금 odom (x, y, yaw, …) · expect = 예상 앞면 중앙 (base_link)"""
        rec = {"t0": time.time(), "ok": False, "why": ""}
        face = None
        t = time.time()
        try:
            if move:
                ok, msg = self.go(self.pose)
                self.up = True   # 어떤 실패든 팔이 움직였을 수 있음 → 끝에 꼭 DRIVE
                rec["pose_s"] = round(time.time() - t, 1)
                if not ok:
                    raise _Skip(f"{self.pose} 실패: {msg}")
            t = time.time()
            while time.time() - t < SETTLE_T and not self.stop.is_set():
                self.hold()
                time.sleep(0.05)
            ok, msg = self.set_mode(READ_TAG)
            if not ok:
                raise _Skip(f"팔 카메라 켜기 실패: {msg}")
            t = time.time()
            st, why = self.measure(prior)
            rec["measure_s"] = round(time.time() - t, 1)
            if st is None:
                raise _Skip(why)
            rec["status"] = {k: st[k] for k in ("ok", "frames", "x", "y", "yaw", "spread_m", "spread_deg", "frame", "camera", "why") if k in st}
            o = odom_fn() if odom_fn else None
            face, why = top_to_face(st, o, expect=expect, bf=bf)
            if face is None:
                raise _Skip(why)
            rec["ok"] = True
        except _Skip as e:
            rec["why"] = str(e)
        finally:
            try:
                self.set_mode(0, timeout=3.0)   # 팔 카메라 끔 (Jetson 부하) — 실패해도 무시
            except Exception:  # noqa: BLE001
                pass
            if self.up and move:
                if not self.back_to_drive() and not self.stop.is_set():   # Ctrl+C 는 어차피 전체 중단
                    raise ArmStuck("팔을 DRIVE 로 못 되돌림")
        rec["total_s"] = round(time.time() - rec.pop("t0"), 1)
        return rec, face


class _Skip(Exception):
    """측정 실패 (팔은 finally 가 DRIVE 로 되돌림)"""
