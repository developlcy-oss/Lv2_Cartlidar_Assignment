#!/usr/bin/env python3
"""실물 벽 보정을 dice_mission.sdf 에 넣음 (arena 의 panel_* · post_* · mark_* 요소 pose 만 바꿈, 나머지 그대로).

apply_real_walls.py <원본 sdf> <출력 sdf> <단계: south | v4 | full>
  south : 동쪽 외벽 중 중심 y ≤ 3.2 m 요소만 직선 x = 4.299 + 0.0249·y 로 (2026-11-20 검토 승인 '남쪽 임시판' — #6 · 스캔)
  v4    : south + 위 외벽 +0.116 · 양옆 채움 패널 (2026-11-20 검토 승인 "E 미통과 판(v4)", points_v3.md)
  full  : (쓰지 않음) south + 북쪽 (sketch_vs_sdf_v3.md 표, 가장 가까운 외벽에서 가장 짧은 사슬)
생성기 generate_world.py 는 저장소에 없음 → 이 스크립트가 같은 결과를 다시 만드는 기록이다.
"""
import math
import re
import sys

EAST_A, EAST_B = 4.299, 0.0249          # 동쪽 외벽 x(y) = A + B·y (sketch_vs_sdf_v2.md 1 절)
TOP_DY = 6.42 - 6.304                   # 위 외벽: 왼쪽 외벽 실측 642 cm (points_v3.md, 2026-11-20) — 단계 v4


def east_x(y):
    return EAST_A + EAST_B * y


# 북쪽 벽 묶음 (단계 full): (이름, [(x0, x1, y0, y1) SDF 좌표 상자 — 요소 중심이 들어가면 그 묶음], dx, dy, 근거).
#   규칙 (검토 승인 2026-11-20): 가장 가까운 외벽에서 가장 짧은 사슬. 위 외벽 y 6.304 · 왼쪽 외벽 x 0 은 미실측(SDF 신뢰).
#   패널 길이는 고정 → 묶음 전체를 같이 옮김. 위 외벽에서 잰 위 끝 = 6.304 − 치수.
NORTH = [
    ('왼쪽 위 세로벽 조각 D (x 1.101, y 5.002~5.706)', [(1.08, 1.12, 4.98, 5.73)], 1.125 - 1.101, (6.304 - 0.705) - 5.706,
     '#15 1125 (왼쪽 외벽) · #16 705 (위 외벽)'),
    ('가로벽 y 4.987 + 세로벽 x 2.569', [(1.83, 2.58, 4.97, 5.0), (2.55, 2.59, 4.97, 5.73)], (1.125 + 0.737) - 1.850, (6.304 - 0.600) - 5.706,
     '#11 737 (조각 D 에서) · #17 600 (위 외벽)'),
    ('위쪽 세로벽 x 3.303 (y 5.002~5.706)', [(3.28, 3.32, 4.97, 5.73)], (2.569 + 0.012 + 0.718) - 3.303, (6.304 - 0.660) - 5.706,
     '#10 718 (x 2.569 에서) · #18 660 (위 외벽)'),
    ('오른쪽 가로벽 y 4.253 (x 3.318~4.022)', [(3.29, 4.05, 4.24, 4.27)],
     (EAST_A + EAST_B * 4.205 - 0.360) - 4.037, ((6.304 - 0.660) - 0.704 - 0.015 - 0.720) - 4.253,
     '#7 360 (동쪽 외벽 직선) · #9 720 (위쪽 세로벽 x 3.303 아래 끝에서)'),
    ('오른쪽 가로벽 y 3.519 (x 3.318~4.022)', [(3.29, 4.05, 3.50, 3.53)], 0.0, ((6.304 - 0.660) - 0.704 - 0.015 - 0.720 - 0.680) - 3.519,
     '#8 680 (가로벽 y 4.253 에서) — 아래 사슬 (#5 + 패널) 은 3.483 으로 4 cm 다름 (불일치 표시)'),
    ('왼쪽 가운데 세로벽 조각 C + 짧은 가로벽 y 4.253', [(1.08, 1.12, 3.51, 4.27), (0.36, 1.10, 4.24, 4.27)], 0.360 - 0.382,
     ((6.304 - 0.600) - 0.704 - 0.015 - 0.735) - 4.253, '#14 360 (왼쪽 외벽) · #12 735 (가로벽 y 4.987 에서) — #13 690 은 약 7 cm 다름 (불일치 표시)'),
]


def north_delta(x, y):
    for name, boxes, dx, dy, _ in NORTH:
        if any(x0 <= x <= x1 and y0 <= y <= y1 for x0, x1, y0, y1 in boxes):
            return name, dx, dy
    return None


def main():
    src, dst, stage = sys.argv[1], sys.argv[2], sys.argv[3]
    s = open(src, encoding='utf-8').read()
    a0 = s.index('<model name="arena">')
    a1 = s.index('</model>', a0)
    arena = s[a0:a1]
    pat = re.compile(r'(<(?:visual|collision) name="((?:panel|post|mark)_\d+)[a-z_]*">\s*<pose>)([-\d. eE]+)(</pose>)')
    moved = {}

    def repl(m):
        name = m.group(2)
        x, y, z, r, p, yaw = [float(v) for v in m.group(3).split()]
        nx, nyaw = x, yaw
        # 모서리 기둥 (y 0 · 6.304) 은 아래 · 위 외벽과 이어져 있고 잰 적이 없어 그대로
        ny = y
        if stage == 'v4' and abs(y - 6.304) < 0.02:
            ny = y + TOP_DY        # 위 외벽 (패널 6~11 · 기둥 7~13, 양 모서리 기둥 포함)
        elif abs(x - 4.404) < 0.02 and 0.3 < y < 6.0 and (y <= 3.2 or stage == 'full'):
            nx = east_x(y)
            if name.startswith('panel'):
                nyaw = -math.atan(EAST_B)     # 패널 긴 축 (로컬 y) 이 직선 방향 (B, 1) 을 따르게
        elif stage == 'full' and (d := north_delta(x, y)):
            nx, ny = x + d[1], y + d[2]
        if (nx, ny, nyaw) == (x, y, yaw):
            return m.group(0)
        moved[name] = (round(x, 4), round(nx, 4), round(y, 4), round(ny, 4))
        return f'{m.group(1)}{nx:.4f} {ny:.4f} {z:g} {r:g} {p:g} {nyaw:.5f}{m.group(4)}'

    arena2 = pat.sub(repl, arena)
    if stage == 'v4':
        # 왼쪽 · 오른쪽 외벽 맨 위 패널 (27 · 43, y 5.925~6.289) 과 옮긴 모서리 기둥 사이를 채움 패널로 메움
        y0, y1 = 6.289, 6.304 + TOP_DY - 0.015
        fill = ''
        for nm, fx in (('panel_fill_left', 0.0), ('panel_fill_right', 4.404)):
            pose = f'{fx:.4f} {(y0 + y1) / 2:.4f} 0.195 0 0 0'
            size = f'0.003 {y1 - y0:.4f} 0.355'
            fill += (f'\n      <visual name="{nm}"><pose>{pose}</pose><geometry><box><size>{size}</size></box></geometry>'
                     f'<material><ambient>0.92 0.92 0.9 1</ambient><diffuse>0.92 0.92 0.9 1</diffuse></material></visual>'
                     f'\n      <collision name="{nm}_collision"><pose>{pose}</pose><geometry><box><size>{size}</size></box></geometry></collision>')
            moved[nm] = (fx, fx, (y0 + y1) / 2, (y0 + y1) / 2)
        cut = arena2.rindex('</link>')
        arena2 = arena2[:cut] + fill + '\n    ' + arena2[cut:]
    if stage == 'full':
        for name, _, dx, dy, why in NORTH:
            print(f'[북쪽] {name}: dx {dx * 1000:+.0f} mm · dy {dy * 1000:+.0f} mm ({why})')
    out = s[:a0] + arena2 + s[a1:]
    out = out.replace('<!-- generate_world.py 로 만든 파일', f'<!-- 2026-11-20 실물 벽 보정 ({stage}): data/261120_newmap/apply_real_walls.py.\n     generate_world.py 로 만든 파일', 1)
    open(dst, 'w', encoding='utf-8').write(out)
    for k, (x0, x1, y0, y1) in sorted(moved.items(), key=lambda kv: kv[1][2]):
        print(f'{k:10s} ({x0:.3f}, {y0:.3f}) → ({x1:.4f}, {y1:.4f})')
    print(f'{len(moved)} 요소 옮김 → {dst}')


if __name__ == '__main__':
    main()
