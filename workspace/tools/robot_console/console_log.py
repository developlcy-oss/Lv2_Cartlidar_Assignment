#!/usr/bin/env python3
"""로봇 콘솔 로그 보기 (Jetson 다른 터미널, log.sh). 읽기만 한다 — 아무것도 발행 · 호출하지 않음.
  (11-16 demo_watch.py 를 바탕으로 콘솔 사건 · 비상정지 · 위치 추정 상태를 더함)
  보여 주는 것 (값이 바뀔 때만 한 줄):
    [미션]   /mission/status  단계 번호 · 이름 · 상태 · 이유
    [신호등] /perception/signal  상태 · 묶음 수 · 순서 · 본 카메라
    [바코드] /perception/barcode  팔 카메라 읽기 결과 (READ) · 앞 카메라 라벨 찾기 (LOCATE)
    [퍽]     /perception/pucks  보이는 퍽 수 · 집을 후보
    [요약]   10 s 마다: 지금 단계, 신호등 읽는 중 · 못 찾음 · 확정, 바코드 읽음 · 못 읽음
    [콘솔]   /console/status  모드 · 단계 · 사건 (콘솔 데몬)
    [안전]   /e_stop · /localization/status  바뀔 때
    [로그]   /rosout  미션 · 계획기 · 측량 노드의 모든 줄, 그 밖 노드는 WARN 이상 중 핵심 낱말 · ERROR
"""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import Log
from std_msgs.msg import Bool, String
from robot_interfaces.msg import Barcode, MissionStatus, PuckArray, SignalSequence

SIG = {0: 'SEARCHING(신호등 찾는 중)', 1: 'COUNTING(세는 중)', 2: 'CANDIDATE(묶음 1개)', 3: '★CONFIRMED(확정)', 4: 'MISMATCH(묶음끼리 다름)'}
MIS = {0: 'IDLE', 1: 'RUNNING', 2: 'PAUSED', 3: '★DONE', 4: '✗FAILED'}
LVL = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}
OWN = ('mission', 'planner', 'survey')                       # 이 낱말이 든 노드는 모든 줄
KEY = ('바코드', '신호', 'barcode', 'signal', 'plan', 'puck', '퍽', 'observe', 'localization', 'LOST', 'FAILED', 'lock')
C = dict(con='\033[1;34m', safe='\033[1;31m', mis='\033[1;36m', sig='\033[1;33m', bar='\033[1;35m', puck='\033[1;32m', log='\033[0m', err='\033[1;31m', off='\033[0m')


class Watch(Node):
    def __init__(self):
        super().__init__('demo_watch')
        self.last = {}
        rel = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        out = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(MissionStatus, '/mission/status', self.on_mission, 10)
        self.create_subscription(SignalSequence, '/perception/signal', self.on_signal, rel)
        self.create_subscription(Barcode, '/perception/barcode', self.on_barcode, rel)
        for q in (rel,):
            self.create_subscription(PuckArray, '/perception/pucks', self.on_pucks, q)
        self.create_subscription(Log, '/rosout', self.on_log, out)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/console/status', self.on_console, latched)
        self.create_subscription(Bool, '/e_stop', lambda m: self.changed('es', m.data) and self.say('safe', f"[안전]   비상정지 {'★ 걸림' if m.data else '풀림'}"), latched)
        self.create_subscription(String, '/localization/status', lambda m: self.changed('loc', m.data) and self.say('safe' if m.data not in ('OK', 'SUSPECT') else 'log', f"[안전]   위치 추정 {m.data}"), latched)
        self.seen_ev = set()
        self.t = {}   # 마지막으로 받은 시각 · 상태가 바뀐 시각
        self.create_timer(10.0, self.beat)
        self.say('log', '보기 시작 — 값이 바뀔 때 한 줄, 10 s 마다 요약 한 줄. 끄기 Ctrl-C')

    def beat(self):
        # 10 s 마다: 지금 미션 단계와, 신호등 · 바코드가 읽히는 중인지 · 못 읽는지 · 토픽이 안 오는지
        now = time.time()
        mis = self.last.get('mis')
        step = f"단계 {mis[0]} '{mis[1]}' {MIS.get(mis[2], mis[2])}" if mis else '미션 상태 아직 안 옴 (bringup 뜨는 중이거나 꺼짐)'
        def age(k):
            return now - self.t[k] if k in self.t else None
        sig, bar = self.last.get('sig'), self.last.get('bar')
        if age('sig_rx') is None or age('sig_rx') > 3.0:
            s_txt = '신호등 읽기 꺼짐 (토픽 안 옴)'
        elif sig[0] == 0:
            s_txt = f"✗ 신호등 못 찾음 {age('sig_ch'):.0f} s 째 (가림 · 색 문턱 의심)"
        elif sig[0] == 3:
            s_txt = f"★ 신호 확정 '{sig[2]}'"
        else:
            s_txt = f"신호등 읽는 중 ({SIG.get(sig[0])}, 묶음 {sig[1]}, {age('sig_ch'):.0f} s 째)"
        if age('bar_rx') is None or age('bar_rx') > 3.0:
            b_txt = '바코드 읽기 꺼짐 (토픽 안 옴)'
        elif bar and bar[0]:
            b_txt = f"바코드 '{bar[1]}'"
        else:
            b_txt = '✗ 바코드 아직 못 읽음'
        self.say('log', f"[요약]   {step} | {s_txt} | {b_txt}")

    def say(self, kind, text):
        print(f"{time.strftime('%H:%M:%S')} {C[kind]}{text}{C['off']}", flush=True)

    def changed(self, key, val):
        if self.last.get(key) == val:
            return False
        self.last[key] = val
        return True

    def on_console(self, m):
        try:
            st = json.loads(m.data)
        except ValueError:
            return
        for e in st.get('events', []):
            if e not in self.seen_ev:
                self.seen_ev.add(e)
                self.say('err' if '실패' in e or 'failed' in e else 'con', f"[콘솔]   {e}")

    def on_mission(self, m):
        v = (m.step_index, m.step_name, m.state, m.reason)
        if self.changed('mis', v):
            self.say('err' if m.state == 4 else 'mis', f"[미션]   단계 {m.step_index} '{m.step_name}'  {MIS.get(m.state, m.state)}  {m.reason}")

    def on_signal(self, m):
        v = (m.status, m.cycles, m.sequence)
        self.t['sig_rx'] = time.time()
        if self.changed('sig', v):
            self.t['sig_ch'] = time.time()
            cam = {0: '앞', 1: '뒤', 2: '팔'}.get(m.camera, str(m.camera))
            self.say('sig', f"[신호등] {SIG.get(m.status, m.status)}  묶음 {m.cycles}  순서 '{m.sequence}' (R{m.red} Y{m.yellow} G{m.green})  카메라 {cam}  보임 {m.light_visible}")

    def on_barcode(self, m):
        if m.source == Barcode.READ:
            self.t['bar_rx'] = time.time()
            if self.changed('bar', (m.read_ok, m.text)):
                ok = m.text in ('P2-01', 'P2-02', 'P2-03', 'P2-04')
                note = '★ 등록된 바코드' if ok else ('✗ 등록 안 된 문자열 (계획기가 거절함)' if m.read_ok else '아직 못 읽음')
                self.say('bar' if ok or not m.read_ok else 'err', f"[바코드] 읽기 ok={m.read_ok} '{m.text}' 기호 {m.symbols}  {note}")
        elif self.changed('loc', len(m.positions)):
            p = f" 첫째 ({m.positions[0].x:.2f}, {m.positions[0].y:.2f}, {m.positions[0].z:.2f}) m" if m.positions else ''
            self.say('bar', f"[바코드] 앞 카메라 라벨 찾기 {len(m.positions)} 개{p}")

    def on_pucks(self, m):
        if self.changed('puck', (len(m.pucks), m.target_index)):
            self.say('puck', f"[퍽]     보이는 퍽 {len(m.pucks)} 개, 집을 후보 번호 {m.target_index}")

    def on_log(self, m):
        if 'Failed to meet update rate' in m.msg:   # EKF 주기 밀림 (부하 때 흔함, 주행 영향 작음) — 화면을 덮어서 뺌
            return
        own = any(k in m.name for k in OWN)
        if not (own or m.level >= 40 or (m.level >= 30 and any(k in m.msg for k in KEY)) or
                ('perception' in m.name and any(k in m.msg for k in ('바코드', '신호')))):
            return
        if m.name == 'demo_watch' or not self.changed('log:' + m.name, m.msg):   # 같은 노드의 같은 줄 연속 반복은 한 번만
            return
        self.say('err' if m.level >= 40 else 'log', f"[로그]   {LVL.get(m.level, m.level):5s} {m.name}: {m.msg}")


def main():
    rclpy.init()
    try:
        rclpy.spin(Watch())
    except Exception:   # Ctrl-C · 터미널 닫힘 (rclpy 가 먼저 내려감) — 조용히 끝
        pass
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
