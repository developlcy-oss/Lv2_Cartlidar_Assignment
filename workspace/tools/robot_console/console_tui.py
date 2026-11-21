#!/usr/bin/env python3
"""로봇 콘솔 TUI (Jetson 다른 터미널, tui.sh). 수동 조작 · 비상정지 · 모드 바꾸기. 콘솔 데몬 (start.sh) 이 떠 있어야 모드가 바뀜.

키
  Space        비상정지 (데몬이 붙잡음 — TUI 를 꺼도 유지)      r   비상정지 풀기 · 리셋 (팔 HOLD 도 풀려 팔 정리가 움직임)
  w / s        앞 / 뒤 (누르고 있는 동안)                       a / d  왼쪽 / 오른쪽 제자리 회전
  x            바로 멈춤                                          + / -  속도 단계
  0 ~ 9        모드 고르기 → y 로 확정 (다른 키 = 취소)          .   모드 모두 끔
  Enter        지금 할 일에 Enter (시연 '구간 출발' 등)          Y · N · Q · S   할 일에 y · n · q · s + Enter
  Esc          TUI 끝 (로봇 · 모드는 그대로)
수동 조작은 /cmd_vel_teleop (twist_mux 우선순위 100 — Nav2 · 정렬보다 우선) 으로 냄. 비상정지는 모든 움직임을 막음.
"""
import curses
import json
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

SPEEDS = [(0.05, 0.25), (0.10, 0.40), (0.15, 0.60)]   # (m/s, rad/s) 단계
HOLD_S = 0.6        # 키를 한 번 누르면 이만큼 움직임 (터미널 키 반복 첫 지연 ≈ 0.5 s 를 메움)
PHASE_KO = {'idle': '대기', 'starting': '띄우는 중', 'waiting': '준비 기다림', 'running': '실행 중', 'done': '끝',
            'failed': '실패', 'stopping': '끄는 중'}


class Tui(Node):
    def __init__(self):
        super().__init__('robot_console_tui')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_cmd = self.create_publisher(String, '/console/cmd', 10)
        self.pub_vel = self.create_publisher(TwistStamped, '/cmd_vel_teleop', 10)
        self.pub_joy = self.create_publisher(Joy, '/joy', 10)   # 데몬이 없어도 비상정지 · 리셋이 되게 직접도 냄
        self.status = {}
        self.status_rx = 0.0
        self.e_stop = None
        self.loc = '-'
        self.create_subscription(String, '/console/status', self.on_status, latched)
        self.create_subscription(Bool, '/e_stop', lambda m: setattr(self, 'e_stop', bool(m.data)), latched)
        self.create_subscription(String, '/localization/status', lambda m: setattr(self, 'loc', m.data), latched)
        self.move = (0.0, 0.0)
        self.move_until = 0.0
        self.speed = 0
        self.note = ''
        self.create_timer(0.1, self.send_vel)

    def on_status(self, m):
        try:
            self.status = json.loads(m.data)
            self.status_rx = time.time()
        except ValueError:
            pass

    def cmd(self, text):
        self.pub_cmd.publish(String(data=text))

    def joy(self, estop, reset):
        b = [0] * 12
        b[6], b[7] = int(estop), int(reset)   # opencr_hardware estop_button 6 · reset_button 7 (마지막 값을 기억)
        m = Joy(axes=[0.0] * 8, buttons=b)
        m.header.stamp = self.get_clock().now().to_msg()
        self.pub_joy.publish(m)

    def estop(self, on):
        self.cmd('estop' if on else 'reset')
        for _ in range(3):
            self.joy(on, False)
        if not on:
            for _ in range(3):
                self.joy(False, True)
            self.joy(False, False)

    def drive(self, lin, ang):
        v, w = SPEEDS[self.speed]
        self.move = (lin * v, ang * w)
        self.move_until = time.time() + HOLD_S

    def stop(self):
        self.move, self.move_until = (0.0, 0.0), 0.0
        self.publish_vel(0.0, 0.0)

    def publish_vel(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x, m.twist.angular.z = v, w
        self.pub_vel.publish(m)

    def send_vel(self):
        if time.time() < self.move_until:
            self.publish_vel(*self.move)
        elif self.move != (0.0, 0.0):
            self.move = (0.0, 0.0)
            self.publish_vel(0.0, 0.0)   # 한 번 0 을 내고 멈춤 (twist_mux 는 0.5 s 뒤 스스로 놓음)


def draw(scr, t):
    scr.erase()
    h, w = scr.getmaxyx()
    st = t.status
    age = time.time() - t.status_rx if t.status_rx else None

    def put(y, x, s, attr=0):
        if 0 <= y < h:
            scr.addnstr(y, x, s, max(0, w - x - 1), attr)
    red, green, yellow = curses.color_pair(1), curses.color_pair(2), curses.color_pair(3)
    put(0, 0, '로봇 콘솔  (Esc 끝 · 도움말은 아래)', curses.A_BOLD)
    if age is None or age > 3.0:
        put(1, 0, '콘솔 데몬 안 보임 — Jetson 첫 터미널에서 start.sh 를 띄웠는지 (모드 바꾸기 · 비상정지 붙잡기 안 됨)', red | curses.A_BOLD)
    else:
        ph = st.get('phase', '')
        col = red if ph == 'failed' else green if ph in ('running', 'done') else yellow
        put(1, 0, f'모드  {st.get("mode", "-")} {st.get("name", "")}   [{PHASE_KO.get(ph, ph)}]', col | curses.A_BOLD)
        put(2, 2, st.get('msg', ''))
        put(3, 2, f'로그  {st.get("run_dir", "") or "-"}')
    es = t.e_stop
    es_txt = '모름 (하드웨어 안 켜짐)' if es is None else ('★ 비상정지 중 ★' if es else '풀림')
    put(4, 0, f'비상정지 {es_txt}' + ('  (콘솔이 붙잡는 중 — r 로 풀기)' if st.get('estop') else ''),
        (red | curses.A_BOLD) if es or st.get('estop') else green)
    put(5, 0, f'위치 추정 {t.loc}   속도 단계 {t.speed + 1}/{len(SPEEDS)} ({SPEEDS[t.speed][0]:.2f} m/s · {SPEEDS[t.speed][1]:.2f} rad/s)'
        f'   지금 명령 {t.move[0]:+.2f} m/s {t.move[1]:+.2f} rad/s')
    put(7, 0, '모드', curses.A_BOLD)
    y = 8
    for k, name in sorted((st.get('modes') or {}).items(), key=lambda x: int(x[0])):
        put(y, 2, f'{k}  {name}', curses.A_REVERSE if str(st.get('mode')) == str(k) else 0)
        y += 1
    put(y, 2, '.  모두 끔')
    y += 2
    put(y, 0, '최근 사건', curses.A_BOLD)
    y += 1
    for line in (st.get('events') or [])[-max(1, h - y - 6):]:
        put(y, 2, line)
        y += 1
    put(h - 4, 0, t.note, yellow | curses.A_BOLD)
    put(h - 3, 0, 'Space 비상정지   r 풀기·리셋   w/s 앞뒤   a/d 회전   x 멈춤   +/- 속도')
    put(h - 2, 0, '0~9 모드 (y 확정)   . 모두 끔   Enter·Y·N·Q·S 할 일에 입력   Esc 끝')
    scr.refresh()


def run(scr, t):
    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_RED, -1)
    curses.init_pair(2, curses.COLOR_GREEN, -1)
    curses.init_pair(3, curses.COLOR_YELLOW, -1)
    scr.timeout(50)
    pending_mode = None
    while rclpy.ok():
        draw(scr, t)
        k = scr.getch()
        if k == -1:
            continue
        if pending_mode is not None:
            if k in (ord('y'), ord('Y')):
                t.cmd(f'mode {pending_mode}')
                t.note = f'모드 {pending_mode} 요청 — 앞 모드를 끄고 새로 띄움 (수십 초)'
            else:
                t.note = '모드 바꾸기 취소'
            pending_mode = None
            continue
        if k == 27:   # Esc
            t.stop()
            return
        c = chr(k) if 0 <= k < 256 else ''
        if c == ' ':
            t.stop()
            t.estop(True)
            t.note = '비상정지 걸음'
        elif c == 'r':
            t.estop(False)
            t.note = '비상정지 풀기 · 리셋 (팔이 STOW 로 접히거나 펴질 수 있음)'
        elif c in 'wsad':
            t.drive({'w': 1, 's': -1}.get(c, 0), {'a': 1, 'd': -1}.get(c, 0))
        elif c == 'x':
            t.stop()
        elif c in '+=':
            t.speed = min(len(SPEEDS) - 1, t.speed + 1)
        elif c in '-_':
            t.speed = max(0, t.speed - 1)
        elif c.isdigit():
            modes = t.status.get('modes') or {}
            if c in modes or int(c) in modes:
                pending_mode = int(c)
                t.note = f'모드 {c} ({modes.get(c, modes.get(int(c)))}) 로 바꿀까요? y = 예, 다른 키 = 취소'
            else:
                t.note = f'모드 {c} 없음'
        elif c == '.':
            t.cmd('stop')
            t.note = '모드 모두 끔 요청'
        elif k in (10, 13, curses.KEY_ENTER):
            t.cmd('input')
            t.note = '할 일에 Enter'
        elif c in 'YNQS':
            t.cmd(f'input {c.lower()}')
            t.note = f'할 일에 {c.lower()} + Enter'


def main():
    rclpy.init()
    t = Tui()
    th = threading.Thread(target=rclpy.spin, args=(t,), daemon=True)
    th.start()
    try:
        curses.wrapper(run, t)
    except KeyboardInterrupt:
        pass
    finally:
        t.stop()
        time.sleep(0.2)
        t.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
