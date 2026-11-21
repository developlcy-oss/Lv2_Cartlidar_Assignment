#!/usr/bin/env python3
"""D435 깊이 짝 원인 측정 (콘솔 모드 6 · 7, 검토 승인 A (0)): 앞 · 뒤 D435 의 컬러 · 정렬 깊이 · 정렬 전 깊이 Hz,
앞 컬러마다 가장 가까운 정렬 깊이까지 시각 차 (≤ 20 ms 비율 · p50 · p90 · 최대), realsense 노드 CPU %, /proc/loadavg.
영상 대신 camera_info 를 구독해 측정 자체의 부하를 줄임. 미리보기 창 없음.

쓰기: camera_probe.py --out <폴더> --label <cameras_only | perception_on> [--sec 30] [--perception_mode 57]
"""
import argparse
import bisect
import json
import os
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

TOPICS = {f'{cam}_{kind}': f'/top_camera_{cam}/{path}/camera_info'
          for cam in ('front', 'rear')
          for kind, path in (('color', 'color'), ('aligned', 'aligned_depth_to_color'), ('depth', 'depth'))}


def cpu_ticks(pid):
    try:
        f = open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()
        return int(f[11]) + int(f[12])
    except OSError:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--label', required=True)
    ap.add_argument('--sec', type=float, default=30.0)
    ap.add_argument('--perception_mode', type=int, default=-1, help='>= 0 이면 먼저 /perception/set_mode 로 켬 (모드 7)')
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node('console_camera_probe')
    if a.perception_mode >= 0:
        r = subprocess.run(['ros2', 'service', 'call', '/perception/set_mode', 'robot_interfaces/srv/SetPerceptionMode',
                            f'{{mode: {a.perception_mode}}}'], capture_output=True, text=True, timeout=60)
        print('인지 모드:', (r.stdout.strip().splitlines() or ['응답 없음'])[-1][:120], flush=True)
        time.sleep(5)
    stamps = {k: [] for k in TOPICS}
    for k, t in TOPICS.items():
        n.create_subscription(CameraInfo, t, lambda m, k=k: stamps[k].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
                              qos_profile_sensor_data)
    pids = subprocess.run(['pgrep', '-f', 'realsense2_camera_node'], capture_output=True, text=True).stdout.split()
    hz = os.sysconf('SC_CLK_TCK')
    c0 = {p: cpu_ticks(p) for p in pids}
    load0 = open('/proc/loadavg').read().split()[:3]
    t0 = time.time()
    print(f'{a.label}: {a.sec:.0f} s 재는 중 (영상 창 없음)', flush=True)
    while time.time() - t0 < a.sec:
        rclpy.spin_once(n, timeout_sec=0.05)
    dt = time.time() - t0
    cpu = {p: round(100.0 * ((cpu_ticks(p) or 0) - (c0[p] or 0)) / hz / dt, 1) for p in pids if c0[p] is not None}
    res = {'label': a.label, 'sec': round(dt, 1), 'loadavg_start': load0, 'loadavg_end': open('/proc/loadavg').read().split()[:3],
           'hz': {k: round(len(v) / dt, 2) for k, v in stamps.items()}, 'realsense_cpu_pct': cpu}
    for cam in ('front', 'rear'):
        c, d = stamps[f'{cam}_color'], sorted(stamps[f'{cam}_aligned'])
        gaps = []
        for x in c:
            i = bisect.bisect_left(d, x)
            near = [abs(x - d[j]) for j in (i - 1, i) if 0 <= j < len(d)]
            if near:
                gaps.append(min(near))
        gaps.sort()
        if gaps:
            res[f'{cam}_pair'] = {'n': len(gaps), 'le_20ms': round(sum(g <= 0.02 for g in gaps) / len(gaps), 3),
                                  'p50_ms': round(1000 * gaps[len(gaps) // 2], 1), 'p90_ms': round(1000 * gaps[int(len(gaps) * 0.9)], 1),
                                  'max_ms': round(1000 * gaps[-1], 1)}
    path = os.path.join(a.out, f'camera_probe_{a.label}.json')
    open(path, 'w', encoding='utf-8').write(json.dumps(res, ensure_ascii=False, indent=1))
    for k, v in res['hz'].items():
        print(f'  {k:14s} {v:6.2f} Hz', flush=True)
    for cam in ('front', 'rear'):
        if f'{cam}_pair' in res:
            print(f'  {cam} 짝: ≤20 ms {res[cam + "_pair"]["le_20ms"]:.0%}, p50 {res[cam + "_pair"]["p50_ms"]} ms, '
                  f'p90 {res[cam + "_pair"]["p90_ms"]} ms', flush=True)
    print(f'  realsense CPU {cpu} · loadavg {res["loadavg_end"]} → {path}', flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
