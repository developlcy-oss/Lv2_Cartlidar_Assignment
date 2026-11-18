#!/usr/bin/env python3
"""로봇 콘솔 데몬 (Jetson). start.sh 가 띄움. 모드 (modes.yaml) 마다 bringup 과 할 일을 띄우고 끄고, 비상정지를 붙잡는다.

받는 것  /console/cmd    (std_msgs/String): "mode <번호>" · "stop" · "estop" · "reset" · "input <글자>" (빈 글자 = Enter)
내는 것  /console/status (std_msgs/String, JSON, 1 Hz · 바뀔 때, transient_local): 모드 · 단계 · 메시지 · 로그 경로 · 비상정지 · 최근 사건
         /joy            (sensor_msgs/Joy): 비상정지 버튼 6 · 리셋 버튼 7 만 (opencr_hardware 가 마지막 값을 기억 → 비상정지는 reset 전까지 유지)
로그     ~/robot_logs/<날짜_시각>_<모드>/launch.log · action.log (터미널에는 사건만)
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time
from glob import glob
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
LOG_ROOT = Path(os.environ.get('ROBOT_LOG_DIR', str(Path.home() / 'robot_logs')))
ESTOP_BTN, RESET_BTN, N_BTN = 6, 7, 12   # opencr.ros2_control.xacro estop_button · reset_button


def now_s():
    return time.strftime('%H:%M:%S')


class Proc:
    """프로세스 그룹 하나 (launch 나 할 일). 표준 출력은 파일로, 표준 입력은 파이프로"""

    def __init__(self, cmd, log_path, stdin=False):
        self.cmd = cmd
        self.log = open(log_path, 'ab', buffering=0)
        self.p = subprocess.Popen(['bash', '-c', cmd], stdout=self.log, stderr=subprocess.STDOUT,
                                  stdin=subprocess.PIPE if stdin else subprocess.DEVNULL, start_new_session=True)

    def alive(self):
        return self.p.poll() is None

    def write(self, text):
        if self.p.stdin and self.alive():
            try:
                self.p.stdin.write(text.encode())
                self.p.stdin.flush()
                return True
            except OSError:
                return False
        return False

    def stop(self, wait_s=12.0):
        """SIGINT → (wait_s) → SIGTERM → (5 s) → SIGKILL, 프로세스 그룹 전체"""
        for sig, t in ((signal.SIGINT, wait_s), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
            if not self.alive():
                break
            try:
                os.killpg(self.p.pid, sig)
            except ProcessLookupError:
                break
            end = time.time() + t
            while time.time() < end and self.alive():
                time.sleep(0.1)
        self.log.close()
        return not self.alive()


class Console(Node):
    def __init__(self, modes_file, first_mode):
        super().__init__('robot_console')
        cfg = yaml.safe_load(open(modes_file, encoding='utf-8'))
        self.common = cfg['common_launch'].strip()
        self.record_topics = ' '.join(str(cfg.get('record_topics', '')).split())
        self.modes = {int(k): v for k, v in cfg['modes'].items()}
        self.lock = threading.Lock()
        self.mode = None            # 지금 모드 번호 (None = 아무것도 안 띄움)
        self.phase = 'idle'         # idle · starting · waiting · running · done · failed · stopping
        self.msg = ''
        self.run_dir = ''
        self.launch = None
        self.action = None
        self.bag = None
        self.events = []
        self.estop = False          # 콘솔이 건 비상정지 (하드웨어 /e_stop 과 별개로 표시)
        self.hw_estop = None
        self.busy = False           # 모드 바꾸는 중 (명령 겹침 막기)
        self.pending = 'none'       # 바꾸는 중에 온 다음 모드 요청 ('none' = 없음, None = 모두 끔)
        self.cancel = False         # 준비 기다림을 그만두라는 표시
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_status = self.create_publisher(String, '/console/status', latched)
        self.pub_joy = self.create_publisher(Joy, '/joy', 10)
        self.create_subscription(String, '/console/cmd', self.on_cmd, 10)
        self.create_subscription(Bool, '/e_stop', lambda m: setattr(self, 'hw_estop', bool(m.data)),
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1.0, self.tick)
        self.event(f'콘솔 시작 (모드 {", ".join(f"{k} {v["name"]}" for k, v in sorted(self.modes.items()))})')
        if first_mode is not None:
            threading.Thread(target=self.switch, args=(first_mode,), daemon=True).start()

    # ---------------- 상태 ----------------
    def event(self, text):
        line = f'{now_s()} {text}'
        print(line, flush=True)
        with self.lock:
            self.events = (self.events + [line])[-30:]
        self.publish()

    def set(self, phase, msg=''):
        with self.lock:
            self.phase, self.msg = phase, msg
        self.event(f'[{self.mode_name()}] {phase}{" — " + msg if msg else ""}')

    def mode_name(self):
        return '-' if self.mode is None else f'{self.mode} {self.modes[self.mode]["name"]}'

    def publish(self):
        with self.lock:
            st = {'mode': self.mode, 'name': self.modes[self.mode]['name'] if self.mode is not None else '',
                  'phase': self.phase, 'msg': self.msg, 'run_dir': self.run_dir, 'estop': self.estop,
                  'hw_estop': self.hw_estop, 'launch_alive': bool(self.launch and self.launch.alive()),
                  'action_alive': bool(self.action and self.action.alive()),
                  'bag_alive': bool(self.bag and self.bag.alive()), 'events': self.events[-12:],
                  'modes': {k: v['name'] for k, v in self.modes.items()}, 't': time.time()}
        try:
            self.pub_status.publish(String(data=json.dumps(st, ensure_ascii=False)))
        except Exception:   # 끝낼 때 (rclpy 가 먼저 내려감) 는 화면 출력만
            pass

    def tick(self):
        if self.estop:   # 비상정지 중에는 1 초마다 다시 냄 (모드를 바꿔 하드웨어가 새로 켜져도 잠긴 채로)
            self.joy(True, False)
        # 할 일이 끝났거나 launch 가 죽으면 알림
        if self.phase == 'running' and self.action and not self.action.alive():
            rc = self.action.p.returncode
            self.set('done' if rc == 0 else 'failed', f'할 일 끝 (종료 코드 {rc}) — 로그 {self.run_dir}/action.log')
        if self.phase in ('waiting', 'running', 'done') and self.launch and not self.launch.alive():
            self.set('failed', f'bringup 이 꺼짐 (종료 코드 {self.launch.p.returncode}) — {self.run_dir}/launch.log')
        self.publish()

    # ---------------- 명령 ----------------
    def on_cmd(self, m):
        t = m.data.strip()
        if t == 'estop':
            self.set_estop(True)
        elif t == 'reset':
            self.set_estop(False)
        elif t.startswith('input'):
            text = t[5:].strip()
            ok = self.action.write(text + '\n') if self.action else False
            self.event(f'입력 "{text or "Enter"}" → {"할 일에 보냄" if ok else "받을 할 일이 없음"}')
        elif t == 'stop':
            threading.Thread(target=self.switch, args=(None,), daemon=True).start()
        elif t.startswith('mode'):
            try:
                n = int(t.split()[1])
            except (IndexError, ValueError):
                self.event(f'모르는 명령: {t}')
                return
            if n not in self.modes:
                self.event(f'없는 모드 {n}')
                return
            threading.Thread(target=self.switch, args=(n,), daemon=True).start()
        else:
            self.event(f'모르는 명령: {t}')

    def joy(self, estop, reset):
        b = [0] * N_BTN
        b[ESTOP_BTN], b[RESET_BTN] = int(estop), int(reset)
        msg = Joy(axes=[0.0] * 8, buttons=b)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_joy.publish(msg)

    def set_estop(self, on):
        """비상정지: 버튼 6 을 누른 채로 (하드웨어가 마지막 값을 기억) · 풀기: 6 을 떼고 7 을 눌렀다 뗌 (팔 REARM 도 이것)"""
        with self.lock:
            self.estop = on
        for _ in range(5):
            self.joy(on, False)
            time.sleep(0.02)
        if not on:
            for _ in range(3):
                self.joy(False, True)
                time.sleep(0.05)
            for _ in range(3):
                self.joy(False, False)
                time.sleep(0.02)
        self.event('비상정지 걸음' if on else '비상정지 풀기 · 리셋 (팔 HOLD 도 풀림 → 켜질 때 팔 정리가 움직임)')

    # ---------------- 모드 ----------------
    def stop_all(self):
        for name in ('action', 'bag', 'launch'):
            p = getattr(self, name)
            if p and p.alive():
                self.event(f'{name} 끄는 중 …')
                p.stop(15.0 if name == 'launch' else 8.0)
            setattr(self, name, None)

    def ports(self):
        if os.environ.get('CONSOLE_PORTS'):   # 시험용 (PC, 로봇 없음)
            return os.environ['CONSOLE_PORTS'].split(',')
        return sorted(glob('/dev/ttyACM*'))

    def others_running(self):
        """이 콘솔 밖에서 띄운 bringup · 하드웨어 (공유 Jetson — 끄지 않고 알리기만)"""
        mine = {p.p.pid for p in (self.launch, self.action) if p and p.alive()}
        out = subprocess.run(['pgrep', '-af', 'ros2 launch|ros2_control_node'], capture_output=True, text=True).stdout
        rows = []
        for line in out.splitlines():
            pid = int(line.split()[0])
            try:
                pgid = os.getpgid(pid)
            except ProcessLookupError:
                continue
            if pgid not in mine:
                rows.append(line[:120])
        return rows

    def wait_nodes(self, names, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if self.cancel:
                return False, '다음 모드 요청으로 그만둠'
            if not (self.launch and self.launch.alive()):
                return False, 'bringup 이 꺼짐'
            have = {('/' + n if ns == '/' else f'{ns}/{n}') for n, ns in self.get_node_names_and_namespaces()}
            miss = [n for n in names if n not in have]
            if not miss:
                return True, ''
            with self.lock:
                self.msg = f'기다리는 노드: {" ".join(miss)} ({int(end - time.time())} s 남음)'
            time.sleep(1.0)
        return False, f'{timeout} s 안에 안 뜬 노드: {" ".join(miss)}'

    def switch(self, n):
        if self.busy:   # 띄우는 중이면 기다림을 그만두고 이 요청으로 (마지막 요청만 남김)
            self.pending, self.cancel = n, True
            self.event(f'모드 {"끔" if n is None else n} 요청 — 지금 띄우는 것을 그만두고 바꿈')
            return
        self.busy = True
        self.cancel = False
        try:
            if self.launch or self.action:
                self.set('stopping', '앞 모드 끔')
                self.stop_all()
            self.mode = n
            if n is None:
                with self.lock:
                    self.phase, self.msg, self.run_dir = 'idle', '', ''
                self.event('모두 끔 (대기)')
                return
            m = self.modes[n]
            others = self.others_running()
            if others:
                self.set('failed', '다른 bringup · 하드웨어가 이미 떠 있음 (끄지 않음 — 띄운 사람에게): ' + ' | '.join(others))
                return
            ports = self.ports()
            if len(ports) != 2:
                self.set('failed', f'OpenCR 포트가 2 개가 아님: {ports or "없음"} — 12 V · USB · RESET 확인 (check_boards.py)')
                return
            d = LOG_ROOT / f'{time.strftime("%y%m%d_%H%M%S")}_mode{n}'
            d.mkdir(parents=True, exist_ok=True)
            with self.lock:
                self.run_dir = str(d)
            fmt = dict(ports=','.join(ports), repo=str(REPO), here=str(HERE))
            launch_cmd = f'{self.common} {m.get("launch_args", "")}'.format(**fmt)
            (d / 'commands.txt').write_text(launch_cmd + '\n' + (m.get('action') or '').format(**fmt) + '\n', encoding='utf-8')
            self.set('starting', f'bringup ({",".join(ports)})')
            self.launch = Proc(launch_cmd, d / 'launch.log')
            self.set('waiting', '노드 기다림')
            ok, why = self.wait_nodes(m.get('wait_nodes', []), m.get('wait_timeout_s', 120))
            if not ok:
                self.set('failed', why + f' — {d}/launch.log')
                return
            time.sleep(m.get('wait_extra_s', 0))
            if m.get('record', True) and self.record_topics:
                self.bag = Proc(f'exec ros2 bag record -o {d}/bag --topics {self.record_topics}', d / 'bag.log')
                self.event(f'rosbag 녹화 시작 → {d}/bag')
            act = (m.get('action') or '').format(**fmt).strip()
            if act:
                self.action = Proc(act, d / 'action.log', stdin=True)
                self.set('running', f'할 일 시작: {act[:80]}')
            else:
                self.set('running', '준비됨 (할 일 없음 — 수동 조작)')
        except Exception as e:   # 데몬은 죽지 않게
            self.set('failed', f'{type(e).__name__}: {e}')
        finally:
            self.busy = False
            nxt, self.pending = self.pending, 'none'
            if nxt != 'none':
                threading.Thread(target=self.switch, args=(nxt,), daemon=True).start()


def main():
    first = None
    if len(sys.argv) > 1 and sys.argv[1] != '-':
        first = int(sys.argv[1])
    rclpy.init()
    node = Console(os.environ.get('CONSOLE_MODES', str(HERE / 'modes.yaml')), first)
    ex = rclpy.executors.MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.event('끝내는 중 — 모드 모두 끔')
        node.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
