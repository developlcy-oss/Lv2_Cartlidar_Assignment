#!/usr/bin/env python3
"""자율 모드 계획 만들기 — 정지점 순서 정하기 + 출발 자세 (HOME) 정확 복귀 + 정지점마다 HOME 복귀 길 (home_legs).

  python3 auto_order.py --base demo_plan_wide.json --legs legs.json --order plan --home-pose 0.6936,0.95,180 --home-legs --out demo_plan_auto.json
    (11-21 demo_plan_auto.json 을 만든 명령. 길 9 개를 widen_plan 으로 새로 — 병렬 --jobs, 구간마다 몇 초 ~ 2 분)
  python3 auto_order.py --base demo_plan_wide.json --legs legs.json --order opt --dry
    (legs.json time_s 로 9! 순서 전수 탐색 → 순서 · 예상 시간만 출력)

--order
  plan : wide 순서 그대로 (S0 ~ S8 은 wide 를 그대로 복사, S9 L1→HOME 만 goal = home_pose 로 바꿔 widen 으로 새로)
  opt  : legs.json time_s 합이 가장 작은 순서 (HOME 출발 · 복귀 포함). 정지점 좌표는 wide 것 (legs.json L2 는 이전 자리)
  "box1,L1,..." : 그 순서 (9 정지점 모두 한 번씩)
  plan 이 아닌 순서는 구간마다 widen 으로 새로 (원래 길 = legs.json 조각, 길을 못 찾으면 그것을 씀 — L2 끝 자리가 다를 수 있음)
--home-legs : 정지점 9 곳 각각 → home_pose 길 (정지점 앞 곧은 들어가기와 같이 home_pose 앞 곧은 들어가기) 를 "home_legs" 에
--home-face 135,120,110,100,90 : 출발 자세 (서향 180°) 로는 곧게 전진해 들어올 수 없음 (11-21 진단: 출발 구역 폭 ~1 m, 서향 들어가기
    시작점에서 차 뒤가 동쪽 벽 x≈1.1 에 닿음 · 주머니 안 U 턴 불가 — 9 길 모두 없음). 그래서 같은 자리 (x, y) 에 이 방향으로 들어온 뒤
    끝에 제자리 {kind S, dth = 180° − face} 를 붙여 정확히 home_pose 로. 각도는 큰 것 (제자리 작음) 부터: 9 길 모두 되는 가장 큰 각도 하나,
    없으면 정지점마다 되는 가장 큰 각도. 90 이면 제자리 90° = 규칙 v2 한도 경계값 (경고)
widen 옵션은 11-19 demo_plan_wide.json 과 같음: --radii 0.5,0.4 --r-floor 0.4 --r-rev 0.25 (widen_plan.py 머리 주석)
"""
import argparse, itertools, json, math, os, sys, time
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

STOPS = ['box1', 'box2', 'box3', 'box4', 'box5', 'box6', 'L1', 'L2', 'L3']


def stop_poses(base):
    """wide course_text 의 route 에서 정지점 이름 → [x, y, yaw]. 출발 (signal) = HOME"""
    names = ['HOME'] + [L['name'].split('→')[-1].strip() for L in base['legs']]
    poses = [list(base['start'])] + [list(L['goal']) for L in base['legs']]
    out = {}
    for n, p in zip(names, poses):
        if n in STOPS or n == 'HOME':
            out.setdefault(n, p)
    return out


def plan_order(base):
    return [L['name'].split('→')[-1].strip() for L in base['legs']][:-1]


def lib_time(legs_json):
    return {(L['from'], L['to']): L['time_s'] for L in legs_json['legs'] if L.get('ok')}


def order_time(order, T):
    seq = ['HOME'] + list(order) + ['HOME']
    return sum(T[(a, b)] for a, b in zip(seq, seq[1:]))


def best_order(T, stops=STOPS):
    best = (1e18, None)
    for p in itertools.permutations(stops):
        try:
            t = order_time(p, T)
        except KeyError:
            continue
        if t < best[0]:
            best = (t, list(p))
    return best


def _widen_job(job):
    """widen_plan.main 의 구간 하나 고르는 방식 그대로 (곧게 0.25 → 0.15 → 0.08, 반경 큰 것부터, r ≥ r_floor 중 비용 - r_bonus·r 최소)"""
    import widen_plan as W
    key, q0, qg, a = job
    if a.get('hard') is not None:
        W.HARD = a['hard']   # 벽 여유 요구 [m] (기본 0.04)
    W.ARM_HARD = a.get('arm')   # 11-22 팔 ↔ 벽 여유 요구 [m] (None = 팔 검사 안 함)
    C = W.Clear()
    cands, log = [], []
    for app in (0.25, 0.15, 0.08):
        for r in a['radii']:
            res, info = W.plan_leg(C, q0, qg, r, a['deadline'], a['r_rev'], app, a.get('spin_pen'), a.get('cusp'))
            log.append(f"  {key}: r {r} · 곧게 {app} → " + (f"비용 {info['cost']} s · 조각 {''.join(p['kind'] for p in res)}" if res else f"길 없음 ({info})"))
            if res:
                info['app'] = app
                cands.append((info['cost'] - a['r_bonus'] * r, r, res, info))
        if cands:
            break
    if not cands:
        return key, None, None, None, log
    good = [c for c in cands if c[1] >= a['r_floor']] or cands
    _, r, res, info = min(good, key=lambda c: c[0])
    st, t_new = W.stats(C, res)
    return key, res, {'r': r, **info}, {'t_theory': round(t_new, 1), 'lines': st}, log


def summarize(st):
    """stats 줄들 → 후진 합 · 최소 여유 [cm] · 제자리 수"""
    import re
    rev = sum(float(m.group(1)) for s in st for m in [re.match(r'후진 ([\d.]+) m', s)] if m)
    clr = [float(m.group(1)) for s in st for m in [re.search(r'여유 (-?[\d.]+) cm', s)] if m]
    spins = sum(1 for s in st if s.startswith('제자리'))
    return {'rev_m': round(rev, 2), 'min_clear_cm': min(clr) if clr else None, 'spin_n': spins}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', default='demo_plan_wide.json')
    ap.add_argument('--legs', default='legs.json')
    ap.add_argument('--order', default='plan', help='plan | opt | "box1,box2,..."')
    ap.add_argument('--home-pose', default='0.6936,0.95,180', help='x,y,yaw[deg]')
    ap.add_argument('--home-legs', action='store_true')
    ap.add_argument('--home-face', default='135,120,110,100,90', help='출발 자리로 들어올 방향 후보 [deg] (큰 것 = 끝 제자리 작음)')
    ap.add_argument('--out', default='demo_plan_auto.json')
    ap.add_argument('--dry', action='store_true', help='순서 · 예상 시간만 출력')
    ap.add_argument('--radii', default='0.5,0.4')
    ap.add_argument('--r-floor', type=float, default=0.4)
    ap.add_argument('--r-bonus', type=float, default=2.0)
    ap.add_argument('--r-rev', type=float, default=0.25)
    ap.add_argument('--deadline', type=float, default=30.0)
    ap.add_argument('--jobs', type=int, default=8)
    ap.add_argument('--spin-pen', type=float, default=None, help='widen 제자리 회전 벌점 [s / 15°] (기본 = widen_plan SPIN_PEN 6)')
    ap.add_argument('--hard', type=float, default=None, help='widen 벽 여유 요구 [m] (기본 widen_plan HARD 0.04 — 격자 오차로 실제 3.4 cm 가 나오는 곳이 있어 0.045 ~ 0.05 로 올려 시험)')
    ap.add_argument('--arm', type=float, nargs='?', const=0.06, default=None, help='11-22 widen 팔 (BOX_VIEW) 외곽 ↔ 벽 여유 요구 [m] (값 없이 --arm = 0.06)')
    ap.add_argument('--fresh', action='store_true', help='--order 가 wide 순서와 같아도 S0 ~ S8 을 복사하지 않고 새로 계산')
    ap.add_argument('--cusp', type=float, default=None, help='widen 방향 바뀜 벌점 [s] (기본 = widen_plan CUSP 4)')
    ap.add_argument('--via-src', default='L1', help='두 단계 길: 입구 → HOME 꼬리를 이 정지점의 직접 복귀 길에서 잘라 씀')
    ap.add_argument('--via-x', type=float, default=1.5, help='두 단계 길 경유 자세 = 꼬리 길 마지막 전진 조각에서 서향으로 x ≤ 이 값이 되는 첫 자세 (출발 구역 남쪽 입구 복도 y≈0.3)')
    ap.add_argument('--cache-only', action='store_true', help='캐시에 없는 길은 계산하지 않고 못 찾음으로 (직접 길 → 두 단계 대체)')
    ap.add_argument('--cache', default='', help='찾은 길을 길마다 파일로 둘 폴더 (다시 돌릴 때 찾은 길은 건너뜀 · 못 찾은 길만 새로)')
    a = ap.parse_args()
    base = json.load(open(os.path.join(HERE, a.base) if not os.path.isabs(a.base) else a.base, encoding='utf-8'))
    lj = json.load(open(os.path.join(HERE, a.legs) if not os.path.isabs(a.legs) else a.legs, encoding='utf-8'))
    hx, hy, hdeg = (float(v) for v in a.home_pose.split(','))
    home = [hx, hy, math.radians(hdeg)]
    P = stop_poses(base)
    P['HOME'] = home
    T = lib_time(lj)

    if a.order == 'plan':
        order = plan_order(base)
    elif a.order == 'opt':
        _, order = best_order(T)
    else:
        order = [s.strip() for s in a.order.split(',') if s.strip()]
    assert sorted(order) == sorted(STOPS), f'순서는 9 정지점 모두 한 번씩: {order}'
    try:
        t_lib = order_time(order, T)
    except KeyError as e:
        t_lib = float('nan')
        print(f'legs.json 에 없는 구간 {e}')
    t_best, o_best = best_order(T)
    print(f"순서 ({a.order}): HOME → {' → '.join(order)} → HOME · legs.json time_s 합 {t_lib:.1f} s (최적 {t_best:.1f} s: {','.join(o_best)})")
    if a.dry:
        return

    wopt = {'radii': [float(v) for v in a.radii.split(',')], 'r_floor': a.r_floor, 'r_bonus': a.r_bonus, 'r_rev': a.r_rev, 'deadline': a.deadline,
            'spin_pen': a.spin_pen, 'cusp': a.cusp, 'hard': a.hard, 'arm': a.arm}
    same_plan = order == plan_order(base) and not a.fresh
    seq = ['HOME'] + order + ['HOME']
    faces = [float(v) for v in a.home_face.split(',') if v.strip()]
    jobs = {}   # key → (q0, qg) · HOME 으로 가는 길은 들어올 방향마다 'A>HOME@face'
    need_home = set()
    for i, (f, t) in enumerate(zip(seq, seq[1:])):
        if same_plan and i < len(seq) - 2:
            continue   # S0 ~ S8 wide 그대로
        if t == 'HOME':
            need_home.add(f)
        else:
            jobs[f'{f}>{t}'] = (P[f], P[t])
    if a.home_legs:
        need_home |= set(STOPS)
    for s in sorted(need_home):
        for fc in faces:
            jobs[f'{s}>HOME@{fc:g}'] = (P[s], [hx, hy, math.radians(fc)])
    print(f'widen 할 길 {len(jobs)} 개 (병렬 {a.jobs}): {", ".join(jobs)}', flush=True)
    t0 = time.time()
    R = {}
    def cpath(k):
        return os.path.join(a.cache, k.replace('>', '_to_').replace('@', '_at_') + '.json')
    def run(jobs):
        n0 = len(R)
        if a.cache:
            os.makedirs(a.cache, exist_ok=True)
            hit = []
            for k, (q0, qg) in jobs.items():
                if os.path.exists(cpath(k)):
                    c = json.load(open(cpath(k), encoding='utf-8'))
                    if c['q0'] == list(q0) and c['qg'] == list(qg) and c.get('spin_pen') == a.spin_pen and c.get('cusp') == a.cusp and c.get('hard') == a.hard and c.get('arm') == a.arm:   # 벌점 · 여유 요구가 다르면 다시
                        R[k] = (c['res'], c['winfo'], c['st'])
                        hit.append(k)
            if hit:
                print(f'캐시에서 {len(hit)} 개: {", ".join(sorted(hit))}', flush=True)
        todo = [k for k in jobs if k not in R]
        if a.cache_only:
            for k in todo:
                R[k] = (None, None, None)
            todo = []
        with ProcessPoolExecutor(max_workers=a.jobs) as ex:
            futs = [ex.submit(_widen_job, (k, jobs[k][0], jobs[k][1], wopt)) for k in todo]
            from concurrent.futures import as_completed
            for n, fu in enumerate(as_completed(futs), 1):
                k, res, winfo, st, log = fu.result()
                R[k] = (res, winfo, st)
                if a.cache and res is not None:
                    json.dump({'q0': list(jobs[k][0]), 'qg': list(jobs[k][1]), 'res': res, 'winfo': winfo, 'st': st, 'deadline': a.deadline,
                               'spin_pen': a.spin_pen, 'cusp': a.cusp, 'hard': a.hard, 'arm': a.arm},
                              open(cpath(k), 'w', encoding='utf-8'), ensure_ascii=False)
                for s in log:
                    print(s, flush=True)
                if res is None:
                    print(f'✗ {k}: 길 못 찾음 ({n}/{len(todo)}, {time.time() - t0:.0f} s)', flush=True)
                    continue
                sm = summarize(st['lines'])
                print(f"✓ {k} r {winfo['r']} · 곧게 {winfo['app']} · 이론 {st['t_theory']} s · 조각 {''.join(p['kind'] for p in res)} · 후진 {sm['rev_m']} m · 최소 여유 {sm['min_clear_cm']} cm  ({n}/{len(todo)}, {time.time() - t0:.0f} s)", flush=True)
                for s in st['lines']:
                    print('    ' + s, flush=True)
    run(jobs)
    # 들어올 방향 고르기: 모두 되는 가장 큰 각도 하나 → 없으면 정지점마다 가장 큰 각도
    ok = {s: [fc for fc in faces if R[f'{s}>HOME@{fc:g}'][0] is not None] for s in need_home}
    common = [fc for fc in faces if all(fc in ok[s] for s in need_home)]
    face_of = {s: (max(common) if common else (max(ok[s]) if ok[s] else None)) for s in need_home}
    print(f"들어올 방향: {'모두 ' + format(max(common), 'g') + '°' if common else '정지점마다'} · " + ', '.join(f'{s} {face_of[s]}' for s in sorted(need_home)), flush=True)
    # 두 단계 길: 직접 길이 없는 정지점 → 경유 자세 W (via_src 직접 길의 입구 복도 자세) → 그 뒤는 via_src 길 꼬리 그대로 (멈추지 않고 이어 붙임)
    method = {s: ('직접' if face_of[s] is not None else None) for s in need_home}
    miss = [s for s in need_home if face_of[s] is None]
    if miss and face_of.get(a.via_src) is not None:
        vf = face_of[a.via_src]
        vres = R[f'{a.via_src}>HOME@{vf:g}'][0]
        iF = max(i for i, pc in enumerate(vres) if pc['kind'] == 'F')
        assert all(pc['kind'] == 'F' for pc in vres[iF:]), '꼬리는 전진 한 조각'
        PF = vres[iF]['poses']
        j = next(i for i, q in enumerate(PF) if q[0] <= a.via_x and math.cos(q[2]) < -0.9)
        Wp = list(PF[j])
        tail = PF[j:]
        print(f'두 단계 길 경유 자세 W = {Wp} ({a.via_src} 길 꼬리 {len(tail)} 점) · 대상 {sorted(miss)}', flush=True)
        run({f'{s}>VIA': (P[s], Wp) for s in miss})
        import widen_plan as WP
        WP.ARM_HARD = a.arm
        if a.hard is not None:
            WP.HARD = a.hard
        Cm = WP.Clear()
        for s in miss:
            r1 = R[f'{s}>VIA'][0]
            if r1 is None:
                continue
            last = r1[-1]
            assert last['kind'] == 'F' and max(abs(u - v) for u, v in zip(last['poses'][-1], Wp)) < 1e-4
            pcs = json.loads(json.dumps(r1))
            pcs[-1]['poses'] = pcs[-1]['poses'] + [list(q) for q in tail[1:]]   # W 에서 방향 그대로 이어서 (같은 전진 조각)
            st_lines, tt = WP.stats(Cm, pcs)
            R[f'{s}>HOME@{vf:g}'] = (pcs, {**R[f'{s}>VIA'][1], 'two_stage': True, 'via': Wp, 'via_src': a.via_src},
                                     {'t_theory': round(tt, 1), 'lines': st_lines})
            face_of[s] = vf
            method[s] = f'두 단계 (W {Wp[0]:.2f}, {Wp[1]:.2f}, {math.degrees(Wp[2]):.0f}°)'
    for s in need_home:
        fc = face_of[s]
        if fc is None:
            R[f'{s}>HOME'] = (None, None, None)
            continue
        res, winfo, st = R[f'{s}>HOME@{fc:g}']
        print(f'  {s}>HOME: {method[s]} · 들어올 방향 {fc:g}°', flush=True)
        dth = math.remainder(math.radians(hdeg) - math.radians(fc), 2 * math.pi)
        res = res + ([{'kind': 'S', 'dth': dth}] if abs(dth) > 1e-9 else [])
        R[f'{s}>HOME'] = (res, {**winfo, 'method': method[s], 'face_deg': fc, 'end_spin_deg': round(math.degrees(dth), 1)},
                          {**st, 't_theory': round(st['t_theory'] + abs(dth) / 0.5 + 1.5, 1), 'lines': st['lines'] + [f'제자리 {math.degrees(dth):+.0f}° (끝, 출발 자세 맞춤)']})
        if abs(math.degrees(dth)) >= 90 - 1e-6:
            print(f'⚠ {s}>HOME: 끝 제자리 {math.degrees(dth):.0f}° = 규칙 v2 한도 90° 경계값', flush=True)
    bad = [k for k, v in R.items() if v[0] is None and '@' not in k and not k.endswith('>VIA')]

    out = {k: v for k, v in base.items() if k != 'legs'}
    seed = {(L['from'], L['to']): L for L in lj['legs']}
    legs = []
    for i, (f, t) in enumerate(zip(seq, seq[1:])):
        k = f'{f}>{t}'
        if same_plan and i < len(seq) - 2:
            legs.append(json.loads(json.dumps(base['legs'][i])))
            continue
        g = {'leg': i, 'name': f"S{i} {'signal' if f == 'HOME' else f}→{'A1' if t == 'HOME' else t}", 'points': [i, i + 1],
             'goal': list(P[t])}
        import box_quiet as _bq
        if t in _bq.BOX_STOPS:
            g['box'] = {'name': t, 'center': list(_bq.BOXES[t]), 'gap': _bq.STOP_GAP, 'front': _bq.FRONT}
        if same_plan:
            g = {**json.loads(json.dumps(base['legs'][i])), 'goal': list(P[t])}
        res, winfo, st = R[k]
        if res is None:
            g['pieces'] = seed[(f, t)]['pieces']
            g['widen_failed'] = True
        else:
            g['pieces'] = res
            g['widen'] = {**winfo, 't_theory': st['t_theory'], **summarize(st['lines'])}
        legs.append(g)
    out['legs'] = legs
    tag = {'plan': '계획순서', 'opt': '최적순서'}.get(a.order, '지정순서')
    out['plan'] = base['plan'] + f'_자동_{tag}_출발자세복귀'
    out['order'] = order
    out['home_pose'] = [hx, hy, round(math.radians(hdeg), 8)]
    if a.home_legs:
        hl = {}
        for s in STOPS:
            res, winfo, st = R[f'{s}>HOME']
            if res is None:
                continue
            hl[s] = {'name': f'H {s}→HOME', 'start': list(P[s]), 'goal': list(home), 'pieces': res,
                     'widen': {**winfo, 't_theory': st['t_theory'], **summarize(st['lines'])}}
        out['home_legs'] = hl
    out['note_auto'] = (f"11-21 auto_order.py --order {a.order}: HOME → {' → '.join(order)} → HOME (출발 자세 {hx}, {hy}, {hdeg:.0f}° 에 정확히 복귀) · "
                        + ('S0 ~ S8 은 wide 그대로, S9 만 widen 으로 새로' if same_plan else '구간마다 widen 으로 새로')
                        + (' · home_legs = 정지점 9 곳 → 출발 자세 길 (자율 모드 중간 복귀용)' if a.home_legs else '')
                        + f" · 출발 자리 들어올 방향 {dict(sorted(face_of.items()))} 뒤 끝 제자리로 {hdeg:.0f}° 맞춤 (서향 곧게 들어오기는 벽에 막힘)"
                        + f" · widen 옵션 radii {a.radii} · r_floor {a.r_floor} · r_rev {a.r_rev}" + (f" · 팔 ↔ 벽 ≥ {a.arm * 100:.1f} cm" if a.arm is not None else '') + f" · legs.json time_s 합 {t_lib:.1f} s"
                        + (f" · 길 못 찾음 {bad}" if bad else ''))
    dst = a.out if os.path.isabs(a.out) else os.path.join(HERE, a.out)
    json.dump(out, open(dst, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print(f'→ {dst} ({time.time() - t0:.0f} s)' + (f' · ✗ 못 찾음 {bad}' if bad else ''))


if __name__ == '__main__':
    main()
