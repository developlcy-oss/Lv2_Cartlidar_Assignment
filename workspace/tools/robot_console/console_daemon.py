#!/usr/bin/env python3
"""로봇 콘솔 데몬 (Jetson). start.sh 가 띄움. 모드 (modes.yaml) 마다 bringup 과 할 일을 띄우고 끄고, 비상정지를 붙잡는다.

받는 것  /console/cmd    (std_msgs/String): "mode <번호>" · "stop" · "estop" · "reset" · "input <글자>" (빈 글자 = Enter)
내는 것  /console/status (std_msgs/String, JSON, 1 Hz · 바뀔 때, transient_local): 모드 · 단계 · 메시지 · 로그 경로 · 비상정지 · 최근 사건
         /joy            (sensor_msgs/Joy): 비상정지 버튼 6 · 리셋 버튼 7 만 (opencr_hardware 가 마지막 값을 기억 → 비상정지는 reset 전까지 유지)
로그     ~/robot_logs/<날짜_시각>_<모드>/launch.log · action.log (터미널에는 사건만)
"""
import json
import os
import re
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
# 모드 전환 (2026-11-20 검토 D 승인): 앞 모드 프로세스가 모두 끝난 것을 확인 (최대 SWITCH_END_WAIT_S) 한 뒤 SWITCH_GAP_S 더 기다림.
#   11-20 실물: 모드 3 종료 4 s 뒤 모드 4 시작에서 map_server change_state 응답 전송 시간 초과 → AMCL 지도 없음 → Nav2 시작 중단
SWITCH_END_WAIT_S, SWITCH_GAP_S = 30.0, 10.0
# lifecycle 확인 · 다시 시작 (검토 D): manager 마다 is_active 를 LIFECYCLE_WAIT_S 까지 기다리고, 안 되면 최대 LIFECYCLE_RETRY 번
LIFECYCLE_WAIT_S, LIFECYCLE_RETRY, LIFECYCLE_RETRY_GAP_S = 90.0, 2, 15.0


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
        """끄고, 끈 프로세스 그룹 번호 목록을 돌려줌 (남은 것이 있는지는 wait_group_end 로)"""
        groups = []
        for name in ('action', 'bag', 'launch'):
            p = getattr(self, name)
            if p and p.alive():
                self.event(f'{name} 끄는 중 …')
                groups.append(p.p.pid)
                p.stop(15.0 if name == 'launch' else 8.0)
            setattr(self, name, None)
        return groups

    @staticmethod
    def group_left(pgids):
        left = []
        for g in pgids:
            out = subprocess.run(['pgrep', '-g', str(g), '-a'], capture_output=True, text=True).stdout
            left += [line[:100] for line in out.splitlines()]
        return left

    def wait_group_end(self, pgids, timeout_s=SWITCH_END_WAIT_S):
        """앞 모드 프로세스 그룹이 모두 끝날 때까지 기다림. 돌려줌: 남은 프로세스 (빈 목록 = 모두 끝남)"""
        end = time.time() + timeout_s
        left = self.group_left(pgids)
        while left and time.time() < end:
            time.sleep(1.0)
            left = self.group_left(pgids)
        return left

    @staticmethod
    def svc(name, srv_type, req='{}', timeout=8.0):
        """ros2 service call 한 번 (시간 초과 · 실패 = None). 데몬 실행기를 막지 않게 따로 프로세스로"""
        try:
            r = subprocess.run(['ros2', 'service', 'call', name, srv_type, req], capture_output=True, text=True, timeout=timeout)
            return r.stdout if r.returncode == 0 else None
        except subprocess.TimeoutExpired:
            return None

    def lifecycle_ready(self, run_dir, managers):
        """lifecycle manager 들이 모두 active 인지 보고, 아니면 다시 시작 (검토 D 승인안).
        노드마다 get_state: 시간 초과 · active 아님 → 그 manager RESET → STARTUP / 모두 active 인데 manager 만 아님 → STARTUP.
        최대 LIFECYCLE_RETRY 번, LIFECYCLE_RETRY_GAP_S 간격. 돌려줌: (ok, 이유)"""
        log = Path(run_dir) / 'launch.log'
        bad = list(managers)
        for attempt in range(LIFECYCLE_RETRY + 1):
            end = time.time() + LIFECYCLE_WAIT_S
            while time.time() < end and not self.cancel:
                bad = [m for m in managers if 'success=True' not in (self.svc(f'/{m}/is_active', 'std_srvs/srv/Trigger') or '')]
                failed = log.exists() and log.read_text(errors='ignore').count('Failed to bring up all requested nodes') > attempt
                if not bad or failed:
                    break
                with self.lock:
                    self.msg = f'lifecycle 기다림: {" ".join(bad)} ({int(end - time.time())} s 남음)'
                time.sleep(3.0)
            if not bad:
                return True, ''
            if self.cancel:
                return False, '다음 모드 요청으로 그만둠'
            with open(Path(run_dir) / 'loadavg.txt', 'a') as f:
                f.write(f'{now_s()} lifecycle 실패 {bad} loadavg {open("/proc/loadavg").read().split()[:3]}\n')
            if attempt == LIFECYCLE_RETRY:
                break
            for m in bad:
                try:
                    out = subprocess.run(['ros2', 'param', 'get', f'/{m}', 'node_names'], capture_output=True, text=True, timeout=15).stdout
                except subprocess.TimeoutExpired:
                    out = ''
                names = re.findall(r"'([^']+)'", out)
                not_active = [n for n in names if "label='active'" not in (self.svc(f'/{n}/get_state', 'lifecycle_msgs/srv/GetState') or '')]
                if not_active or not names:
                    self.event(f'lifecycle {m}: active 아님 {not_active or "(노드 목록 못 읽음)"} → RESET → STARTUP ({attempt + 1}/{LIFECYCLE_RETRY})')
                    self.svc(f'/{m}/manage_nodes', 'nav2_msgs/srv/ManageLifecycleNodes', '{command: 3}', 60.0)
                else:
                    self.event(f'lifecycle {m}: 노드는 모두 active, manager 만 아님 → STARTUP ({attempt + 1}/{LIFECYCLE_RETRY})')
                self.svc(f'/{m}/manage_nodes', 'nav2_msgs/srv/ManageLifecycleNodes', '{command: 0}', 90.0)
            time.sleep(LIFECYCLE_RETRY_GAP_S)
        return False, f'lifecycle 시작 실패 ({LIFECYCLE_RETRY} 번 다시 해도): {" ".join(bad)} — 모드를 다시 켜세요'

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
            if self.launch or self.action or self.bag:
                self.set('stopping', '앞 모드 끔')
                left = self.wait_group_end(self.stop_all())
                if left:
                    self.mode = n
                    self.set('failed', f'앞 모드 프로세스가 {SWITCH_END_WAIT_S:.0f} s 안에 안 끝남 (끄지 않음): ' + ' | '.join(left[:4]))
                    return
                if n is not None:
                    self.event(f'앞 모드 프로세스 모두 끝남 — {SWITCH_GAP_S:.0f} s 더 기다림 (시작 부하 겹침 막기)')
                    end = time.time() + SWITCH_GAP_S
                    while time.time() < end and not self.cancel:
                        time.sleep(0.2)
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
            fmt = dict(ports=','.join(ports), repo=str(REPO), here=str(HERE), run_dir=str(d))
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
            managers = m.get('lifecycle_managers')
            if managers is None:   # 기본: AMCL · 지도는 늘, Nav2 는 use_nav2:=false 가 아닐 때
                managers = ['lifecycle_manager_maps', 'lifecycle_manager_amcl'] + \
                    ([] if 'use_nav2:=false' in launch_cmd else ['lifecycle_manager_navigation'])
            if managers:
                ok, why = self.lifecycle_ready(d, managers)
                if not ok:
                    self.set('failed', why + f' — {d}/launch.log')
                    return
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
