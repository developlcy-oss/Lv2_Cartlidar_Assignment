#!/usr/bin/env python3
"""RC-100 리모컨 → /joy (주행 시험 조작용, 2026-11-13). Nav2 실행과는 따로 쓴다 (bringup rc100:=true 일 때만 뜸).

경로: RC-100 ~ 블루투스 ~ BT-410 (주행 OpenCR, Serial2) → 펌웨어가 STATE 빈 칸에 실음 (opencr_protocol RC100_SLOT_*)
      → OpenCRHardware 가 /rc100/raw [버튼, 마지막 패킷 뒤 ms + 1] → 이 노드 → /joy → 기존 teleop_twist_joy · OpenCRHardware E-stop
버튼 → /joy (opencr_hardware/config/teleop.yaml · opencr_hardware.cpp 기본 번호, 사용자 결정 "데드맨 없이"):
  방향 U/D → axes[1] = ±lin (teleop scale_linear 0.2 → 기본 0.5 × 0.2 = 0.1 m/s)
  방향 L/R → axes[0] = ±ang (teleop scale_angular 1.0 → 기본 0.5 rad/s = 모터 1.6, 벨트 출렁임 3.0 아래). 왼쪽 = +
  방향 버튼을 누르는 동안 buttons[4] (teleop enable_button) 를 자동으로 누름 → 떼면 teleop 이 0 을 한 번 내고 멈춤
  1 → buttons[6] (estop_button) = E-stop  ·  2 + 3 동시 → buttons[7] (reset_button) = E-stop 풀기 (켤 때 한 번 필요)
  4 · 5 · 6 은 안 씀
/joy 는 버튼이 바뀌면 바로, 아니면 20 Hz (OpenCRHardware 조이스틱 heartbeat, joy_timeout 0.5 s).
  /rc100/raw 가 link_timeout 동안 없으면 (주행 보드 · 호스트 끊김) /joy 를 멈춤 → OpenCRHardware 가 "조이스틱 무수신" 으로 잠금.
  hold_timeout: 마지막 리모컨 패킷 뒤 이 시간이 지나면 뗀 것으로 봄 (0 = 끔). 리모컨이 누르는 동안 패킷을 되풀이해 보내면 블루투스가
  끊겼을 때도 멈춤. 되풀이하지 않으면 길게 누를 때 끊김 → 로그의 "누른 동안 가장 긴 패킷 간격" 을 보고 정함 (실물 확인 ★).
  리모컨 전원이 꺼져도 /rc100/raw 는 계속 오므로 heartbeat 는 이어짐 (E-stop 잠김 아님) — 그때는 방향 버튼 값만 0.
⚠ 소프트웨어 정지일 뿐 물리 비상정지가 아님.
"""
import sys

BTN = {'U': 0x001, 'D': 0x002, 'L': 0x004, 'R': 0x008, '1': 0x010, '2': 0x020, '3': 0x040, '4': 0x080, '5': 0x100, '6': 0x200}
ARROWS = BTN['U'] | BTN['D'] | BTN['L'] | BTN['R']
J_ENABLE, J_ESTOP, J_RESET = 4, 6, 7      # teleop.yaml enable_button · opencr_hardware.cpp estop_button · reset_button
AX_YAW, AX_LIN = 0, 1                      # teleop.yaml axis_angular.yaw · axis_linear.x
N_AXES, N_BUTTONS = 8, 12                  # joy_teleop 의 시작 · 정지 (9 · 8) 보다 넉넉하게, 모두 0


def names(bits):
    return '+'.join(k for k, v in BTN.items() if bits & v) or '없음'


def effective_bits(bits, age_plus1, hold_timeout):
    """리모컨 값 → 지금 누르고 있다고 볼 버튼. age_plus1 = 0 이면 받은 적 없음 → 0"""
    if age_plus1 <= 0:
        return 0
    if hold_timeout > 0 and (age_plus1 - 1) > hold_timeout * 1000.0:
        return 0
    return bits


def joy_from_bits(bits, lin, ang):
    """버튼 → (axes, buttons). 순수 함수 (시험 test_rc100_joy.py)"""
    axes = [0.0] * N_AXES
    buttons = [0] * N_BUTTONS
    fwd = (1 if bits & BTN['U'] else 0) - (1 if bits & BTN['D'] else 0)
    turn = (1 if bits & BTN['L'] else 0) - (1 if bits & BTN['R'] else 0)
    axes[AX_LIN] = float(lin * fwd)
    axes[AX_YAW] = float(ang * turn)
    if bits & ARROWS:
        buttons[J_ENABLE] = 1
    if bits & BTN['1']:
        buttons[J_ESTOP] = 1
    if (bits & BTN['2']) and (bits & BTN['3']):
        buttons[J_RESET] = 1
    return axes, buttons


def main():
    import time

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Joy
    from std_msgs.msg import UInt32MultiArray

    class Rc100Joy(Node):
        def __init__(self):
            super().__init__('rc100_joy')
            self.lin = float(self.declare_parameter('lin', 0.5).value)
            self.ang = float(self.declare_parameter('ang', 0.5).value)
            self.hold_timeout = float(self.declare_parameter('hold_timeout', 0.5).value)
            self.link_timeout = float(self.declare_parameter('link_timeout', 0.3).value)
            if not (0.0 <= self.lin <= 1.0 and 0.0 <= self.ang <= 1.0):
                raise ValueError('lin · ang 는 0 ~ 1 (teleop scale 을 곱함)')
            self.pub = self.create_publisher(Joy, '/joy', 10)
            self.create_subscription(UInt32MultiArray, '/rc100/raw', self.on_raw, 10)
            self.raw, self.age, self.raw_t = 0, 0, None
            self.sent, self.sent_t = None, 0.0
            self.press_t, self.max_gap_ms, self.prev_age = None, 0, 0
            self.create_timer(0.05, self.tick)
            self.get_logger().info(
                f'/rc100/raw → /joy · 앞뒤 {self.lin} × 0.2 m/s · 회전 {self.ang} × 1.0 rad/s · hold_timeout {self.hold_timeout} s · '
                f'link_timeout {self.link_timeout} s · 1 = E-stop · 2+3 = 풀기 (켤 때 한 번)')

        def on_raw(self, m):
            if len(m.data) < 2:
                return
            bits, age = int(m.data[0]), int(m.data[1])
            now = time.monotonic()
            if bits != self.raw:   # 실물 확인용 기록: 누른 동안 리모컨 패킷 간격 (age 가 다시 작아지면 되풀이해 보낸 것)
                if self.raw and self.press_t is not None:
                    self.get_logger().info(f'뗌 {names(self.raw)} — 누른 {now - self.press_t:.2f} s 동안 가장 긴 패킷 간격 '
                                           f'{self.max_gap_ms} ms (되풀이하면 작음, 안 하면 누른 시간과 비슷)')
                self.press_t, self.max_gap_ms = (now if bits else None), 0
            elif bits:
                self.max_gap_ms = max(self.max_gap_ms, age - 1)
            self.raw, self.age, self.raw_t = bits, age, now
            self.publish(now)   # 바뀐 버튼은 기다리지 않고 바로

        def tick(self):
            self.publish(time.monotonic(), heartbeat=True)

        def publish(self, now, heartbeat=False):
            if self.raw_t is None or now - self.raw_t > self.link_timeout:
                return   # 주행 보드 · 호스트 끊김 → /joy 멈춤 → OpenCRHardware 잠금
            bits = effective_bits(self.raw, self.age + int((now - self.raw_t) * 1000), self.hold_timeout)
            if not heartbeat and bits == self.sent:
                return
            axes, buttons = joy_from_bits(bits, self.lin, self.ang)
            j = Joy()
            j.header.stamp = self.get_clock().now().to_msg()
            j.header.frame_id = 'rc100'
            j.axes, j.buttons = axes, buttons
            self.pub.publish(j)
            if bits != self.sent:
                self.get_logger().info(f'버튼 {names(bits)} → 앞뒤 {axes[AX_LIN]:+.2f} · 회전 {axes[AX_YAW]:+.2f} · '
                                       f'E-stop {buttons[J_ESTOP]} · 풀기 {buttons[J_RESET]}')
            self.sent, self.sent_t = bits, now

    rclpy.init(args=sys.argv)
    node = Rc100Joy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
