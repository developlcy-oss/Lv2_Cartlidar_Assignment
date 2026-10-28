"""waffle_4wd Isaac Sim 6.2 ↔ ros2_control 브리지 (Isaac Lab 없이 isaacsim 단독). 실행은 run_bridge.sh 로 (ROS 환경 변수).

    ./run_bridge.sh [--headless] [--usd <waffle_4wd.usda>]
    호스트: ros2 launch opencr_hardware base_bringup.launch.py sim:=true use_teleop:=false

호스트 ros2_control 의 OpenCRHardware 자리를 topic_based_ros2_control/TopicBasedSystem 이 맡고, 이 스크립트가 실물 OpenCR 처럼:
  /isaac_joint_states    (발행)  저장소 관절 이름 (robot.urdf.xacro) 으로 위치 · 속도 · 힘
  /isaac_joint_commands  (구독)  바퀴 velocity · 팔 position. 0.5 s 끊기면 바퀴 정지 (펌웨어 comm_timeout 과 같게)
  /clock · /imu/data (imu_link, 뒤집어 단 보드 축) · /e_stop false 10 Hz (실물은 OpenCRHardware 가 발행, twist_mux 잠금 해제)

관절 대응 (USD waffle_4wd_usd_261027_1711 ↔ 저장소): 바퀴 축은 둘 다 +y. 팔은 USD 축이 모두 반대라 부호를 뒤집는다 (영점 자세 · 링크 길이는 같음).
gripper_roll (그리퍼 개폐 모터, -0.7946 열림 ~ 0.9541 닫힘) 은 손가락 위치로 직선 변환 (USD 미믹은 Newton 전용이라 두 손가락을 각각 명령).
USD 위치값은 저장소 URDF 와 base_link 원점이 다르다 — 로봇 TF 는 호스트 robot_state_publisher 가 저장소 URDF 로 만든다 (docs/frames_tf.md).
"""
import argparse
import math
import time

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="waffle_4wd Isaac Sim 6.2 ros2_control 브리지")
parser.add_argument("--usd", default="/home/pa8/Downloads/urdf_test/waffle_4wd_usd_261027_1711/waffle_4wd/waffle_4wd.usda")
parser.add_argument("--headless", action="store_true", help="창 없이")
parser.add_argument("--cmd_timeout", type=float, default=0.5, help="명령이 이만큼 [s] 끊기면 바퀴 정지 (0 = 끔)")
parser.add_argument("--test_seconds", type=float, default=0.0, help="(점검용) 이 시뮬레이션 시간 [s] 뒤 종료")
args = parser.parse_args()
app = SimulationApp({"headless": args.headless})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
from isaacsim.core.experimental.objects import GroundPlane  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

app_utils.enable_extension("isaacsim.ros2.core")   # jazzy rclpy 를 sys.path 에 (run_bridge.sh 가 라이브러리 경로를 줌)
app.update()
import rclpy  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import Imu, JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

PHYS_HZ = 120
WARMUP_FRAMES = 240
FRICTION = 1.0                                   # 바닥 · 로봇 정지 · 미끄럼 마찰 (urdfv2 robot_params.json 과 같게)
WHEEL_DAMPING = 0.48                             # 바퀴 속도 드라이브 [N·m·s/rad] (urdfv2 검증값)
ARM_STIFFNESS, ARM_DAMPING = 19.0, 0.85          # 팔 위치 드라이브 (urdfv2 검증값)
FINGER_STIFFNESS, FINGER_DAMPING = 200.0, 5.0
# 저장소 관절 이름 -> (USD 관절, 부호)
WHEELS = {"wheel_left_front_joint": "Wheel_Revolute_Front_Left", "wheel_left_rear_joint": "Wheel_Revolute_Rear_Left",
          "wheel_right_front_joint": "Wheel_Revolute_Front_Right", "wheel_right_rear_joint": "Wheel_Revolute_Rear_Right"}
ARM = {"joint1": ("RMX52_Base_Yaw", -1.0), "joint2": ("RMX52_Shoulder", -1.0), "joint3": ("RMX52_Elbow", -1.0),
       "joint4": ("RMX52_Wrist_Pitch", -1.0), "joint5": ("RMX52_Wrist_Roll_Reversed", -1.0)}
GRIPPER = "gripper_roll"
GRIP_OPEN_RAD, GRIP_CLOSED_RAD = -0.7946, 0.9541           # 저장소 gripper_roll 한계 (열림 1550 · 닫힘 2650 tick)
FINGER_OPEN, FINGER_CLOSED = 0.026186, -0.001414           # USD gripper_left_joint 한계 [m] (오른쪽은 부호 반대)
IMU_RPY = (math.pi, 0.0, math.pi / 2)                      # 저장소 imu_link (base_link 기준, 뒤집어 장착)


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_matrix(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def matrix_quat(m):
    """회전 행렬 -> (w, x, y, z)"""
    t = np.trace(m)
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        return 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
    v = [0.0, 0.0, 0.0]
    v[i], v[j], v[k] = 0.25 * s, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s
    return (m[k, j] - m[j, k]) / s, *v


def finger_from_grip(rad):
    s = (min(GRIP_CLOSED_RAD, max(GRIP_OPEN_RAD, rad)) - GRIP_OPEN_RAD) / (GRIP_CLOSED_RAD - GRIP_OPEN_RAD)
    return FINGER_OPEN + s * (FINGER_CLOSED - FINGER_OPEN)


def grip_from_finger(m):
    s = (m - FINGER_OPEN) / (FINGER_CLOSED - FINGER_OPEN)
    return GRIP_OPEN_RAD + s * (GRIP_CLOSED_RAD - GRIP_OPEN_RAD)


# ---------------------------------------------------------------- 장면
def build_scene():
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    # 로봇 1대는 CPU 물리가 빠르다 (GPU 는 매 프레임 관절 목표를 쓰면 동기화로 프레임 34 ms, CPU 8 ms — 2026-10-28 측정)
    SimulationManager.setup_simulation(dt=1.0 / PHYS_HZ, device="cpu")
    GroundPlane("/World/ground")
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(800.0)
    mat = UsdShade.Material.Define(stage, "/World/PhysicsMaterial")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(FRICTION)
    pm.CreateDynamicFrictionAttr(FRICTION)
    pm.CreateRestitutionAttr(0.0)
    robot = stage.DefinePrim("/World/waffle", "Xform")
    robot.GetReferences().AddReference(args.usd)
    robot.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
    for path in ("/World/ground", "/World/waffle"):   # 물리 재질 바인딩은 아래 충돌 형상에 상속됨
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(
            mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.Scene) or prim.IsA(UsdPhysics.Scene):
            PhysxSchema.PhysxSceneAPI.Apply(prim).CreateTimeStepsPerSecondAttr(PHYS_HZ)
    return stage


def set_view(eye, target):
    try:
        from omni.kit.viewport.utility import get_active_viewport
        from omni.kit.viewport.utility.camera_state import ViewportCameraState

        vp = get_active_viewport()
        cs = ViewportCameraState(vp.camera_path, vp)
        cs.set_position_world(Gf.Vec3d(*eye), False)
        cs.set_target_world(Gf.Vec3d(*target), True)
    except Exception as e:  # noqa: BLE001
        print(f"[waffle_bridge] 시점 설정 생략 ({e})")


# ---------------------------------------------------------------- 브리지
class Bridge:
    def __init__(self, robot):
        self.robot = robot
        idx = {n: i for i, n in enumerate(robot.dof_names)}
        self.wheel_names = list(WHEELS)
        self.wheel_ids = [idx[WHEELS[n]] for n in self.wheel_names]
        self.arm_names = list(ARM)
        self.arm_ids = [idx[ARM[n][0]] for n in self.arm_names]
        self.arm_sign = np.array([ARM[n][1] for n in self.arm_names])
        self.finger_ids = [idx["gripper_left_joint"], idx["gripper_right_joint"]]
        self.wheel_cmd = np.zeros(4)
        q = robot.get_dof_positions().numpy()[0]
        self.arm_cmd = q[self.arm_ids] * self.arm_sign
        self.grip_cmd = grip_from_finger(q[self.finger_ids[0]])
        self.last_cmd = None
        self.prev_vel = None
        self.R_bi = rpy_matrix(*IMU_RPY)

        rclpy.init()
        self.node = rclpy.create_node("isaac_waffle_bridge")
        self.pub_js = self.node.create_publisher(JointState, "/isaac_joint_states", 10)
        self.pub_clock = self.node.create_publisher(Clock, "/clock", 10)
        self.pub_imu = self.node.create_publisher(Imu, "/imu/data", 10)
        estop_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_estop = self.node.create_publisher(Bool, "/e_stop", estop_qos)   # robot_interfaces qos::e_stop 과 같게
        self.node.create_subscription(JointState, "/isaac_joint_commands", self.on_cmd, 10)
        self.next_estop = 0.0

    def on_cmd(self, m):
        """TopicBasedSystem 명령: name 은 관절 전체, position · velocity 는 그 인터페이스를 가진 관절만 순서대로 담길 수 있음"""
        names = list(m.name)

        def pick(values, has):   # 관절마다 값 (없으면 None)
            sel = [n for n in names if has(n)]
            if len(values) == len(names):
                return {n: values[i] for i, n in enumerate(names)}
            if len(values) == len(sel):
                return dict(zip(sel, values))
            return {}

        vel = pick(list(m.velocity), lambda n: n in WHEELS)
        pos = pick(list(m.position), lambda n: n in ARM or n == GRIPPER)
        for i, n in enumerate(self.wheel_names):
            if n in vel and math.isfinite(vel[n]):
                self.wheel_cmd[i] = vel[n]
        for i, n in enumerate(self.arm_names):
            if n in pos and math.isfinite(pos[n]):
                self.arm_cmd[i] = pos[n]
        if GRIPPER in pos and math.isfinite(pos[GRIPPER]):
            self.grip_cmd = pos[GRIPPER]
        self.last_cmd = time.monotonic()

    def apply(self):
        r = self.robot
        wheels = self.wheel_cmd
        if args.cmd_timeout > 0 and (self.last_cmd is None or time.monotonic() - self.last_cmd > args.cmd_timeout):
            wheels = np.zeros(4)
        r.set_dof_velocity_targets(wheels.reshape(1, -1), dof_indices=self.wheel_ids)
        f = finger_from_grip(self.grip_cmd)
        r.set_dof_position_targets(np.concatenate([self.arm_cmd * self.arm_sign, [f, -f]]).reshape(1, -1),
                                   dof_indices=self.arm_ids + self.finger_ids)

    def publish(self, t, dt):
        r = self.robot
        stamp = rclpy.time.Time(seconds=t).to_msg()
        self.pub_clock.publish(Clock(clock=stamp))
        q = r.get_dof_positions().numpy()[0]
        v = r.get_dof_velocities().numpy()[0]
        try:
            e = r.get_dof_efforts().numpy()[0]
        except Exception:  # noqa: BLE001
            e = np.zeros_like(q)
        js = JointState()
        js.header.stamp = stamp
        js.name = self.wheel_names + self.arm_names + [GRIPPER]
        js.position = [float(x) for x in np.concatenate([q[self.wheel_ids], q[self.arm_ids] * self.arm_sign,
                                                          [grip_from_finger(q[self.finger_ids[0]])]])]
        js.velocity = [float(x) for x in np.concatenate([v[self.wheel_ids], v[self.arm_ids] * self.arm_sign,
                                                          [v[self.finger_ids[0]] * (GRIP_CLOSED_RAD - GRIP_OPEN_RAD)
                                                           / (FINGER_CLOSED - FINGER_OPEN)]])]
        js.effort = [float(x) for x in np.concatenate([e[self.wheel_ids], e[self.arm_ids] * self.arm_sign, [0.0]])]
        self.pub_js.publish(js)
        # IMU: base_link 자세 · 속도 (월드) -> imu_link (보드 축)
        _, quat = r.get_world_poses()
        lin, ang = r.get_velocities()
        R_wb = quat_matrix(*quat.numpy()[0])
        lin, ang = lin.numpy()[0], ang.numpy()[0]
        acc = np.zeros(3) if self.prev_vel is None or dt <= 0 else (lin - self.prev_vel) / dt
        self.prev_vel = lin
        R_wi = R_wb @ self.R_bi
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        w, x, y, z = matrix_quat(R_wi)
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = w, x, y, z
        gi = R_wi.T @ ang
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (float(c) for c in gi)
        ai = R_wi.T @ (acc + np.array([0.0, 0.0, 9.81]))   # 가속도계 = 가속도 - 중력
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (float(c) for c in ai)
        imu.orientation_covariance = [1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3]
        imu.angular_velocity_covariance = [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4]
        imu.linear_acceleration_covariance = [1e-2, 0, 0, 0, 1e-2, 0, 0, 0, 1e-2]
        self.pub_imu.publish(imu)
        if t >= self.next_estop:
            self.pub_estop.publish(Bool(data=False))
            self.next_estop = t + 0.1

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    SimulationManager.switch_physics_engine("physx")
    build_scene()
    for _ in range(5):
        app.update()
    robot = Articulation("/World/waffle")
    app_utils.play(commit=True)
    for _ in range(3):
        app.update()
    n = len(robot.dof_names)
    wheel_mask = np.array(["Wheel" in d for d in robot.dof_names])
    finger_mask = np.array(["gripper" in d for d in robot.dof_names])
    stiff = np.where(wheel_mask, 0.0, np.where(finger_mask, FINGER_STIFFNESS, ARM_STIFFNESS)).reshape(1, n)
    damp = np.where(wheel_mask, WHEEL_DAMPING, np.where(finger_mask, FINGER_DAMPING, ARM_DAMPING)).reshape(1, n)
    robot.set_dof_gains(stiffnesses=stiff, dampings=damp)
    for _ in range(WARMUP_FRAMES):   # 첫 렌더 준비로 몇 초 멈추는 구간 — /clock 을 내기 전에 넘긴다 (호스트 컨트롤러 전환 5 s 제한)
        app.update()
    bridge = Bridge(robot)
    if not args.headless:
        set_view((-0.9, -0.9, 0.7), (0.1, 0.0, 0.15))
    print(f"[waffle_bridge] 시작: 관절 {robot.dof_names}, 물리 {PHYS_HZ} Hz. 호스트에서 base_bringup sim:=true")
    t_prev = SimulationManager.get_simulation_time()
    wall0, sim0 = time.perf_counter(), t_prev
    while app.is_running():
        rclpy.spin_once(bridge.node, timeout_sec=0.0)
        bridge.apply()
        app.update()
        t = SimulationManager.get_simulation_time()
        bridge.publish(t, t - t_prev)
        t_prev = t
        lag = (t - sim0) - (time.perf_counter() - wall0)   # 실시간보다 빠르면 기다림
        if lag > 0:
            time.sleep(lag)
        if args.test_seconds and t - sim0 >= args.test_seconds:
            break
    bridge.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        app.close()
