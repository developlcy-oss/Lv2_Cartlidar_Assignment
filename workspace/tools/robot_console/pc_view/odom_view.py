#!/usr/bin/env python3
"""PC 감시 창: 신호등 정답 순서 (PC 에 꽂은 신호등 OpenCR 시리얼) + 로봇 odom 오차 (Jetson 에서 ssh 로 받음).
odom_view.sh 가 띄움. 로봇 쪽은 읽기만.

그림 (왼쪽 위 → 오른쪽)
  ① 지도 위 경로: AMCL (파랑, 지도에 맞춘 위치) · EKF 만 (주황) · 바퀴 odom 만 (회색) — 시작 (또는 r) 때 AMCL 자리에 겹쳐 놓고 그 뒤 각자 쌓음
  ② yaw 누적 변화 [°]: 바퀴 · IMU 적분 · EKF · AMCL
  ③ 오차 [cm · °]: EKF 만 · 바퀴 만 이 AMCL 에서 벗어난 거리 · 방향, map→odom 보정이 시작보다 움직인 양
글자: 바퀴/IMU 회전 비 (IMU 가 20° 넘게 돌았을 때부터), 지금 오차, 신호등 순서 · 지금 켜진 등
키: r = 다시 겹치기 (기준을 지금으로) · q = 끝
"""
import json
import math
import os
import subprocess
import sys
import threading
import time
from collections import deque

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt   # noqa: E402
import numpy as np                # noqa: E402

HIST = 3000   # 10 Hz × 300 s


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):   # SE2 a∘b
    c, s = math.cos(a[2]), math.sin(a[2])
    return [a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])]


def inv(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return [-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]]


class Serial(threading.Thread):
    """신호등 OpenCR (signal_light_12.ino): SEQ … · ON n 색 · TERM 줄. pyserial 없이 stty 로"""

    def __init__(self, dev):
        super().__init__(daemon=True)
        self.dev, self.seq, self.lamp, self.lines = dev, '', '', deque(maxlen=5)

    def run(self):
        if not self.dev or not os.path.exists(self.dev):
            self.lamp = f'신호등 시리얼 없음 ({self.dev})'
            return
        subprocess.run(['stty', '-F', self.dev, '115200', 'raw', '-echo'], check=False)
        with open(self.dev, 'rb', buffering=0) as f:
            buf = b''
            while True:
                ch = f.read(1)
                if not ch:
                    time.sleep(0.01)
                    continue
                if ch != b'\n':
                    buf += ch
                    continue
                line = buf.decode(errors='ignore').strip()
                buf = b''
                if not line:
                    continue
                print('[신호등]', line, flush=True)
                self.lines.append(line)
                p = line.split()
                if p[0] == 'SEQ':
                    self.seq = ' '.join(p[1:])
                elif p[0] == 'ON' and len(p) >= 3:
                    self.lamp = f'{p[1]}/12 {p[2]}'
                elif p[0] == 'TERM':
                    self.lamp = '쉼 (3 s)'


class Robot(threading.Thread):
    def __init__(self, cmd):
        super().__init__(daemon=True)
        self.cmd, self.last, self.n, self.err = cmd, None, 0, ''

    def run(self):
        p = subprocess.Popen(self.cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        threading.Thread(target=lambda: [setattr(self, 'err', l.strip()) for l in p.stderr], daemon=True).start()
        for line in p.stdout:
            try:
                self.last = json.loads(line)
                self.n += 1
            except ValueError:
                pass


def load_map(yaml_path):
    import yaml
    from PIL import Image
    m = yaml.safe_load(open(yaml_path))
    img = np.array(Image.open(os.path.join(os.path.dirname(yaml_path), m['image'])).convert('L'))
    r, (ox, oy) = m['resolution'], m['origin'][:2]
    h, w = img.shape
    return img, [ox, ox + w * r, oy, oy + h * r]


def main():
    robot_cmd, serial_dev, map_yaml = sys.argv[1], sys.argv[2], sys.argv[3]
    sig = Serial(serial_dev)
    sig.start()
    rob = Robot(robot_cmd)
    rob.start()
    st = {'anchor': None, 'path': {k: deque(maxlen=HIST) for k in ('amcl', 'ekf', 'wheel')},
          't': deque(maxlen=HIST), 'yaw': {k: deque(maxlen=HIST) for k in ('wheel', 'imu', 'ekf', 'amcl')},
          'err': {k: deque(maxlen=HIST) for k in ('ekf_cm', 'wheel_cm', 'ekf_deg', 'wheel_deg', 'mo_cm', 'mo_deg')}}

    def reset():
        st['anchor'] = None
        for d in (st['path'], st['yaw'], st['err']):
            for q in d.values():
                q.clear()
        st['t'].clear()

    from matplotlib import font_manager
    ko = [f.name for f in font_manager.fontManager.ttflist if any(k in f.name for k in ('Nanum', 'Noto Sans CJK', 'Noto Sans KR', 'UnDotum'))]
    plt.rcParams['font.family'] = (ko[:1] or []) + ['DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig = plt.figure('로봇 odom 오차 · 신호등', figsize=(14, 8))
    ax_map = fig.add_subplot(1, 2, 1)
    ax_yaw = fig.add_subplot(2, 2, 2)
    ax_err = fig.add_subplot(2, 2, 4)
    try:
        img, ext = load_map(map_yaml)
        ax_map.imshow(img, cmap='gray', origin='upper', extent=ext, alpha=0.6)
    except Exception as e:
        print('지도 그림 없음:', e)
    lines = {k: ax_map.plot([], [], c, lw=1.5, label=l)[0] for k, c, l in
             (('amcl', 'b-', 'AMCL (지도 맞춤)'), ('ekf', '-', 'EKF 만'), ('wheel', '-', '바퀴 odom 만'))}
    lines['ekf'].set_color('tab:orange')
    lines['wheel'].set_color('gray')
    ax_map.set_aspect('equal')
    ax_map.legend(loc='upper right', fontsize=8)
    ylines = {k: ax_yaw.plot([], [], label=l)[0] for k, l in
              (('wheel', '바퀴'), ('imu', 'IMU 적분'), ('ekf', 'EKF'), ('amcl', 'AMCL'))}
    ax_yaw.set_ylabel('yaw 누적 [°]')
    ax_yaw.legend(loc='upper left', fontsize=8)
    elines = {k: ax_err.plot([], [], s, label=l)[0] for k, s, l in
              (('ekf_cm', '-', 'EKF 만 위치 [cm]'), ('wheel_cm', '-', '바퀴 만 위치 [cm]'),
               ('ekf_deg', '--', 'EKF 만 방향 [°]'), ('wheel_deg', '--', '바퀴 만 방향 [°]'), ('mo_cm', ':', 'map→odom 보정 [cm]'))}
    ax_err.set_xlabel('시간 [s]')
    ax_err.legend(loc='upper left', fontsize=8)
    info = fig.text(0.01, 0.01, '', fontsize=10, family='monospace')
    fig.canvas.mpl_connect('key_press_event', lambda e: reset() if e.key == 'r' else (plt.close('all') if e.key == 'q' else None))
    t0, prev, unwrap = time.time(), {}, {}
    while plt.fignum_exists(fig.number):
        s = rob.last
        if s and s.get('wheel') and s.get('ekf'):
            a = s.get('amcl')
            if st['anchor'] is None:
                base = a or s['ekf']
                st['anchor'] = {'base': base, 'wheel': s['wheel'], 'ekf': s['ekf'], 'imu': s['imu_yaw'],
                                'amcl': a, 'mo': s.get('map_odom'), 't': s['t']}
                prev, unwrap = {}, {}
            A = st['anchor']
            t = s['t'] - A['t']
            if not st['t'] or t > st['t'][-1] + 0.05:
                st['t'].append(t)
                for k in ('wheel', 'ekf'):
                    st['path'][k].append(compose(A['base'], compose(inv(A[k]), s[k])))
                if a:
                    st['path']['amcl'].append(a)
                raw = {'wheel': s['wheel'][2], 'ekf': s['ekf'][2], 'amcl': a[2] if a else None}
                for k, v in raw.items():
                    if v is None:
                        st['yaw'][k].append(np.nan)
                        continue
                    if k in prev:
                        unwrap[k] = unwrap.get(k, 0.0) + wrap(v - prev[k])
                    prev[k] = v
                    st['yaw'][k].append(math.degrees(unwrap.get(k, 0.0)))
                st['yaw']['imu'].append(math.degrees(s['imu_yaw'] - A['imu']))
                for k in ('ekf', 'wheel'):
                    p = st['path'][k][-1]
                    if a:
                        st['err'][k + '_cm'].append(100 * math.hypot(p[0] - a[0], p[1] - a[1]))
                        st['err'][k + '_deg'].append(math.degrees(abs(wrap(p[2] - a[2]))))
                    else:
                        st['err'][k + '_cm'].append(np.nan)
                        st['err'][k + '_deg'].append(np.nan)
                mo, mo0 = s.get('map_odom'), A.get('mo')
                st['err']['mo_cm'].append(100 * math.hypot(mo[0] - mo0[0], mo[1] - mo0[1]) if mo and mo0 else np.nan)
                st['err']['mo_deg'].append(math.degrees(abs(wrap(mo[2] - mo0[2]))) if mo and mo0 else np.nan)
            for k, ln in lines.items():
                if st['path'][k]:
                    xy = np.array(st['path'][k])
                    ln.set_data(xy[:, 0], xy[:, 1])
            T = np.array(st['t'])
            for k, ln in ylines.items():
                ln.set_data(T[:len(st['yaw'][k])], np.array(st['yaw'][k]))
            for k, ln in elines.items():
                ln.set_data(T[:len(st['err'][k])], np.array(st['err'][k]))
            for ax in (ax_yaw, ax_err):
                ax.relim()
                ax.autoscale_view()
            imu_d = st['yaw']['imu'][-1] if st['yaw']['imu'] else 0.0
            wh_d = st['yaw']['wheel'][-1] if st['yaw']['wheel'] else 0.0
            ratio = f'{wh_d / imu_d:.2f}' if abs(imu_d) > 20 else '- (IMU 20° 넘게 돌면)'
            e = st['err']
            last = lambda q: (f'{q[-1]:.1f}' if q and not np.isnan(q[-1]) else '-')
            info.set_text(
                f'신호등 정답 {sig.seq or "-"}   지금 {sig.lamp or "-"}\n'
                f'바퀴/IMU 회전 비 {ratio}   (바퀴 {wh_d:+.0f}° · IMU {imu_d:+.0f}°)   '
                f'EKF 만 오차 {last(e["ekf_cm"])} cm {last(e["ekf_deg"])}° · 바퀴 만 {last(e["wheel_cm"])} cm {last(e["wheel_deg"])}° · '
                f'map→odom 보정 {last(e["mo_cm"])} cm {last(e["mo_deg"])}°\n'
                f'받은 줄 {rob.n}   AMCL {"있음" if a else "없음 (위치 추정 안 켬)"}   r = 기준 다시 · q = 끝')
        else:
            info.set_text(f'로봇 자료 기다리는 중 … (받은 줄 {rob.n}) {rob.err[-120:]}\n신호등 정답 {sig.seq or "-"}   지금 {sig.lamp or "-"}')
        plt.pause(0.2)


if __name__ == '__main__':
    main()
