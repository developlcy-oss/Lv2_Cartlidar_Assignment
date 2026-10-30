#!/usr/bin/env python3
"""퍽 플레이트 (meshes/puck_pallet.stl, mm) -> meshes/parts/puck_plate.{stl,json} + 충돌 볼록 조각 meshes/parts/puck_plate_col/*.stl.
make_revision.py 가 이 결과를 읽어 puck_plate_link 를 만든다. 플레이트 STL 이 바뀌었을 때만 다시 돌린다 (약 30 초).

    python3 -m venv ~/.venvs/puckplate && ~/.venvs/puckplate/bin/pip install trimesh shapely manifold3d mapbox_earcut rtree scipy
    ~/.venvs/puckplate/bin/python tools/make_puck_plate.py

- 받은 STL 좌표 (mm): 원점 = 팔 1 번 관절 (joint1) 축, z 0 = 플레이트 밑면 (상판 윗면에 닿음), +x = 로봇 앞.
  근거: 앞 끝 반원 홈이 원점 중심 반지름 58 mm, 컵 12 개가 원점 중심 반지름 90 · 142 · 191 mm 원 위에서 원점 쪽을 봄 (2026-10-30 분석).
  링크 좌표 = 이 좌표 그대로 (m 로만 바꿈). make_revision.py 가 원점을 (팔 축 x, y, 상판 윗면 z) 에 둔다.
- 질량: 부피 x PLA 1240 kg/m3 x 채움 0.55 (make_revision.py 출력 부품 가정과 같음).
- 충돌 (퍽이 컵에 들어가야 하므로 상자 하나로 못 함, 조각 수는 Gazebo · Isaac 이 감당할 만큼):
  1. 바닥판 z 0 ~ 3 mm (옆면 수직): 격자 구멍은 메움 (퍽 · 손가락에 상관없음). z 0~2 는 바깥 테두리 그대로,
     z 2~3 은 안쪽 줄 컵 4 개 자리를 비움 (그 컵의 퍽은 z 2 턱에 앉음). 단면을 볼록 다각형으로 나눠 (삼각분할 + Hertel-Mehlhorn) 기둥으로.
  2. z 3 mm 위 (컵, 벽이 깔때기처럼 기울어짐): 컵마다 보로노이 칸으로 잘라, 컵 좌표 (사각 구멍 축) 에서 꼭짓점을 네 면으로 나누고
     퍽 턱 높이 아래 (받침) · 위 (벽) 로 나눈 뒤, 이어진 덩어리마다 볼록 껍질 (떨어진 리브가 벽과 한 껍질로 묶이지 않게).
  3. 조각마다 꼭짓점 HULL_MAX_VERTS 개 이하로: Isaac (PhysX) 은 볼록 조각 꼭짓점을 64 개로 줄이면서 바깥으로 부풀려 컵 안을 막는다
     (2026-10-30 시험: 퍽이 튕겨 나감). 원래 꼭짓점 중 여러 방향의 끝점만 골라 다시 감싼다 (부분 집합이라 원래보다 커지지 않음).
  확인: 컵 자리 퍽 (원기둥 지름 30, 높이 60, 0.3 mm 줄임) 이 충돌 조각과 겹치지 않을 것.
- light_boxes (xacro collision:=light): 바닥판 상자 3 개 (반원 홈 비움) + 컵마다 꽉 찬 상자 (컵 좌표로 돌림, 벽 윗면까지).
"""
import hashlib
import json
import math
import shutil
from pathlib import Path

import manifold3d as mf
import numpy as np
import shapely
import trimesh
from shapely.geometry import MultiPoint, Point, Polygon, box
from shapely.ops import unary_union, voronoi_diagram

HERE = Path(__file__).resolve().parents[1]
SRC = HERE / 'meshes/puck_pallet.stl'
OUT = HERE / 'meshes/parts'
COL_DIR = OUT / 'puck_plate_col'
PLA_KG_M3, PLA_FILL = 1240.0, 0.55
ARC_R_MM = 58.0                       # 앞 끝 반원 홈 (팔 축 둘레)
PLATE_TOP_MM = 3.0                    # 바닥판 윗면 (옆면 수직인 구간)
PLATE_LAYERS = ((0.0, 2.0), (2.0, 3.0))   # 안쪽 줄 컵 턱이 z 2 라서 두 층
SIMPLIFY_MM = 0.3
PUCK_R_MM, PUCK_H_MM = 15.0, 60.0
HULL_MAX_VERTS = 60                   # PhysX 기본 한계 64 보다 조금 적게


def reduce_hull(h, n=HULL_MAX_VERTS):
    """볼록 껍질을 꼭짓점 n 개 이하로: 고른 방향 (피보나치 구) 마다 가장 멀리 나간 꼭짓점을 모아 다시 감싼다."""
    V = h.vertices
    if len(V) <= n:
        return h
    for k in (400, 300, 200, 150, 100, 80, 60, 40, 30):
        i = np.arange(k) + 0.5
        phi, th = np.arccos(1 - 2 * i / k), math.pi * (1 + 5 ** 0.5) * i
        dirs = np.c_[np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)]
        keep = np.unique(((V - V.mean(0)) @ dirs.T).argmax(0))
        r = trimesh.convex.convex_hull(V[keep])
        if len(r.vertices) <= n:
            return r
    raise RuntimeError('꼭짓점을 줄이지 못함')


def section(m, z):
    s = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    p, _ = s.to_2D(to_2D=np.eye(4))
    return unary_union(p.polygons_full)


def convex_pieces(poly):
    """다각형 (구멍 포함) -> 볼록 다각형 목록 (삼각분할 뒤 볼록이 유지되는 이웃끼리 합침, Hertel-Mehlhorn)."""
    out = []
    for g in getattr(poly, 'geoms', [poly]):
        g = shapely.geometry.polygon.orient(g, 1.0)
        V, F = trimesh.creation.triangulate_polygon(g, engine='earcut')
        polys = {i: list(f) for i, f in enumerate(F)}
        for k, f in polys.items():                       # 반시계로
            a, b, c = V[f]
            if (b - a)[0] * (c - a)[1] - (b - a)[1] * (c - a)[0] < 0:
                polys[k] = [f[0], f[2], f[1]]
        owner = {}
        for k, f in polys.items():
            for i in range(len(f)):
                owner.setdefault(frozenset((f[i], f[(i + 1) % len(f)])), []).append(k)
        alias = {k: k for k in polys}

        def find(k):
            while alias[k] != k:
                alias[k] = alias[alias[k]]
                k = alias[k]
            return k

        def convex(ring):
            P = V[ring]
            d1 = np.roll(P, -1, 0) - P
            d2 = np.roll(P, -2, 0) - np.roll(P, -1, 0)
            return np.all(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0] >= -1e-9)

        diagonals = sorted((e for e, ks in owner.items() if len(ks) == 2),
                           key=lambda e: -np.linalg.norm(V[list(e)[0]] - V[list(e)[1]]))   # 긴 대각선부터 지움
        for e in diagonals:
            ka, kb = (find(k) for k in owner[e])
            if ka == kb:
                continue
            A, B = polys[ka], polys[kb]
            a, b = tuple(e)
            ia = A.index(a); ib = A.index(b)
            if A[(ia + 1) % len(A)] != b:                 # A 에서 a -> b 순서가 되게
                a, b, ia, ib = b, a, ib, ia
            if A[(ia + 1) % len(A)] != b or B[(B.index(b) + 1) % len(B)] != a:
                continue
            ra = A[ib:] + A[:ib]                          # b ... a
            jb = B.index(a)
            rb = B[jb:] + B[:jb]                          # a ... b
            merged = ra[:-1] + rb[:-1]
            if convex(merged):
                polys[ka] = merged
                del polys[kb]
                alias[kb] = ka
        out += [Polygon(V[r]) for r in polys.values()]
    return out


def to_manifold(t):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))


def from_manifold(M):
    mm = M.to_mesh()
    return trimesh.Trimesh(mm.vert_properties[:, :3], mm.tri_verts, process=True)


def cups(m):
    """컵 12 개: z 5 mm 단면의 사각 구멍 (한 변 약 32 mm) 가운데, 반지름 방향 각, 퍽 앉는 턱 높이, 벽 윗면."""
    out = []
    for g in getattr(section(m, 5.0), 'geoms', []):
        for h in g.interiors:
            hp = Polygon(h)
            if 600 < hp.area < 1400:
                c = np.array(hp.centroid.coords[0])
                rr = np.array(hp.minimum_rotated_rectangle.exterior.coords)
                e = rr[1] - rr[0]
                sq = math.degrees(math.atan2(e[1], e[0])) % 90.0                  # 사각 구멍 변 방향 (0 ~ 90)
                seat, rim = 0.0, 0.0
                for ang in np.deg2rad([0, 90, 180, 270]):        # 턱: 가운데에서 11 mm 떨어진 곳에서 내려다본 가장 높은 면
                    o = [c[0] + 11 * math.cos(ang), c[1] + 11 * math.sin(ang), 100.0]
                    loc, _, _ = m.ray.intersects_location([o], [[0, 0, -1.0]])
                    seat = max(seat, loc[:, 2].max() if len(loc) else 0.0)
                a = math.radians(sq)                                              # 벽 윗면: 컵 좌표 사각 (구멍 + 벽 6 mm) 안 가장 높은 점
                uv = (m.vertices[:, :2] - c) @ np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
                near = m.vertices[np.all(np.abs(uv) < math.sqrt(hp.area) / 2 + 6, axis=1)]
                rim = near[:, 2].max()
                out.append(dict(center_mm=[round(c[0], 2), round(c[1], 2)], radius_mm=round(float(np.hypot(*c)), 1),
                                facing_deg=round(math.degrees(math.atan2(c[1], c[0])), 1), square_deg=round(sq, 2),
                                pocket_mm=round(math.sqrt(hp.area), 1), seat_mm=round(float(seat), 2), rim_mm=round(float(rim), 2)))
    out.sort(key=lambda d: (-d['radius_mm'], d['center_mm'][1]))
    return out


def main():
    m = trimesh.load(SRC)
    assert m.is_watertight and len(m.split()) == 1, '플레이트 STL 이 닫힌 한 덩어리가 아님'
    b = m.bounds
    assert abs(b[0, 2]) < 1e-6, f'밑면이 z 0 이 아님: {b[0, 2]}'
    # 원점 = 팔 축 확인: 앞 끝 반원 홈이 원점 중심 반지름 ARC_R_MM
    ext = max(getattr(section(m, 1.5), 'geoms', [section(m, 1.5)]), key=lambda g: g.area).exterior
    pts = np.array(ext.coords)
    arc = pts[(pts[:, 0] > -ARC_R_MM - 4) & (np.abs(pts[:, 1]) < ARC_R_MM + 4)]
    r = np.hypot(arc[:, 0], arc[:, 1])
    assert len(arc) > 20 and np.abs(r - ARC_R_MM).max() < 0.5, f'반원 홈이 원점 중심 {ARC_R_MM} mm 가 아님 (|r - R| 최대 {np.abs(r - ARC_R_MM).max():.2f})'

    OUT.mkdir(parents=True, exist_ok=True)
    shutil.rmtree(COL_DIR, ignore_errors=True)
    COL_DIR.mkdir()

    # 겉모양: m 단위로 그대로
    vis = m.copy(); vis.apply_scale(0.001)
    vis.export(OUT / 'puck_plate.stl')

    # 질량 특성 (고르게, PLA x 채움)
    vol = m.volume * 1e-9
    kg = vol * PLA_KG_M3 * PLA_FILL
    vis.density = kg / vis.volume
    I = vis.moment_inertia                      # 무게중심 기준
    com = vis.center_mass

    # 충돌 1: 바닥판 두 층 (볼록 기둥). 격자 구멍은 메우고, z 2~3 층만 안쪽 줄 컵 자리를 비움
    cl = cups(m)
    assert len(cl) == 12, f'컵 {len(cl)} 개 (기대 12)'
    inner = [c for c in cl if c['seat_mm'] < PLATE_TOP_MM]
    hulls = []
    for z0, z1 in PLATE_LAYERS:
        sec = section(m, (z0 + z1) / 2)
        outline = unary_union([Polygon(g.exterior) for g in getattr(sec, 'geoms', [sec])])
        if z0 >= 2.0 - 1e-9:
            holes = [Polygon(h) for g in getattr(sec, 'geoms', [sec]) for h in g.interiors]
            pockets = [h for h in holes if any(h.contains(Point(c['center_mm'])) for c in inner)]
            assert len(pockets) == len(inner), f'안쪽 컵 자리 {len(pockets)} 개 (기대 {len(inner)})'
            outline = outline.difference(unary_union(pockets))
        for pc in convex_pieces(outline.simplify(SIMPLIFY_MM)):
            if pc.area < 0.5:
                continue
            hulls.append(('plate', trimesh.creation.extrude_polygon(pc, z1 - z0).apply_translation([0, 0, z0])))
    n_plate = len(hulls)

    # 충돌 2: 컵 (보로노이 칸 -> 컵 좌표 네 면 x 받침 · 벽 -> 볼록 껍질)
    M = to_manifold(m)
    frame = box(b[0, 0] - 5, b[0, 1] - 5, b[1, 0] + 5, b[1, 1] + 5)
    cells = voronoi_diagram(MultiPoint([c['center_mm'] for c in cl]), envelope=frame)
    for c in cl:
        cell = next(g for g in cells.geoms if g.contains(Point(c['center_mm']))).intersection(frame)
        prism = mf.Manifold.extrude(mf.CrossSection([np.asarray(shapely.geometry.polygon.orient(cell, 1.0).exterior.coords)[:-1]]),
                                    b[1, 2] + 5 - PLATE_TOP_MM).translate((0, 0, PLATE_TOP_MM))
        piece = from_manifold(M ^ prism)
        if len(piece.faces) == 0:
            continue
        a = math.radians(c['square_deg'])
        R = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])
        T = piece.triangles                                            # 삼각형 단위로 나눔 (꼭짓점 공유 무관)
        uv = (T[:, :, :2].mean(1) - c['center_mm']) @ R.T
        side = np.where(np.abs(uv[:, 0]) >= np.abs(uv[:, 1]), np.where(uv[:, 0] >= 0, 0, 1), np.where(uv[:, 1] >= 0, 2, 3))
        upper = T[:, :, 2].mean(1) > c['seat_mm'] + 0.05
        for sd in range(4):
            for up in (False, True):
                sel = (side == sd) & (upper == up)
                if sel.sum() < 4:
                    continue
                lo, hi = (-1e9, c['seat_mm'] + 0.05) if not up else (c['seat_mm'] - 0.05, 1e9)
                grp = trimesh.Trimesh(*trimesh.util.append_faces([t for t in T[sel]], [np.array([[0, 1, 2]])] * int(sel.sum())))
                grp.merge_vertices()
                for comp in grp.split(only_watertight=False):         # 떨어진 덩어리 (리브 등) 는 따로 감쌈
                    pts = comp.vertices[(comp.vertices[:, 2] >= lo) & (comp.vertices[:, 2] <= hi)]
                    if len(pts) < 4:
                        continue
                    try:
                        h = trimesh.convex.convex_hull(pts)
                    except Exception:
                        continue
                    if h.volume > 1.0:
                        hulls.append(('cup', h))

    # 꼭짓점 줄이기 (PhysX 한계)
    before = sum(h.volume for _, h in hulls)
    hulls = [(k, reduce_hull(h)) for k, h in hulls]
    print(f'꼭짓점 {HULL_MAX_VERTS} 개 이하로: 부피 {before / 1000:.1f} -> {sum(h.volume for _, h in hulls) / 1000:.1f} cm3, '
          f'꼭짓점 최대 {max(len(h.vertices) for _, h in hulls)}')

    # 확인: 컵 자리 퍽 (조금 줄인 원기둥) 과 겹치는 조각 부피
    hull_vol = sum(h.volume for _, h in hulls)
    worst = 0.0
    for c in cl:
        puck = trimesh.creation.cylinder(radius=PUCK_R_MM - 0.3, height=PUCK_H_MM - 0.6, sections=64)
        puck.apply_translation([c['center_mm'][0], c['center_mm'][1], c['seat_mm'] + 0.3 + PUCK_H_MM / 2])
        P = to_manifold(puck)
        ov = sum((P ^ to_manifold(h)).volume() for _, h in hulls if np.all(h.bounds[0] < puck.bounds[1]) and np.all(puck.bounds[0] < h.bounds[1]))
        worst = max(worst, ov)
    print(f'충돌 조각: 바닥판 {n_plate} + 컵 {len(hulls) - n_plate} = {len(hulls)} 개, 부피 합 {hull_vol / 1000:.1f} cm3 (메시 {m.volume / 1000:.1f}), '
          f'컵 자리 퍽과 겹침 최대 {worst:.1f} mm3')
    assert worst < 50.0, '충돌 조각이 컵 안을 막음'

    # light 상자 (m): 바닥판 = 반원 홈 앞 (x > -R) 은 양옆만, 컵 = 컵 좌표 외곽 (구멍 + 벽) x 벽 윗면
    light = []
    xs, ys = b[:, 0], b[:, 1]
    for name, (x0, x1, y0, y1) in (('base', (xs[0], -ARC_R_MM, ys[0], ys[1])),
                                   ('left', (-ARC_R_MM, xs[1], ARC_R_MM, ys[1])), ('right', (-ARC_R_MM, xs[1], ys[0], -ARC_R_MM))):
        light.append(dict(name=name, center_m=[(x0 + x1) / 2000, (y0 + y1) / 2000, PLATE_TOP_MM / 2000],
                          size_m=[(x1 - x0) / 1000, (y1 - y0) / 1000, PLATE_TOP_MM / 1000], yaw_rad=0.0))
    for i, c in enumerate(cl):
        a = math.radians(c['square_deg'])
        Rm = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])
        uv = (m.vertices[:, :2] - c['center_mm']) @ Rm.T
        sel = np.all(np.abs(uv) < c['pocket_mm'] / 2 + 8, axis=1) & (m.vertices[:, 2] >= PLATE_TOP_MM)
        u0, v0 = uv[sel].min(0); u1, v1 = uv[sel].max(0)
        cxy = np.array(c['center_mm']) + np.array([(u0 + u1) / 2, (v0 + v1) / 2]) @ Rm
        light.append(dict(name=f'cup{i:02d}', center_m=[cxy[0] / 1000, cxy[1] / 1000, (PLATE_TOP_MM + c['rim_mm']) / 2000],
                          size_m=[(u1 - u0) / 1000, (v1 - v0) / 1000, (c['rim_mm'] - PLATE_TOP_MM) / 1000], yaw_rad=a))

    files = []
    for i, (kind, h) in enumerate(hulls):
        h = h.copy(); h.apply_scale(0.001)
        name = f'puck_plate_col/{kind}_{i:03d}.stl'
        h.export(OUT / name)
        files.append(name)

    info = dict(
        source=str(SRC.relative_to(HERE)), source_sha1=hashlib.sha1(SRC.read_bytes()).hexdigest(),
        frame='원점 = 팔 joint1 축 · 플레이트 밑면, +x 앞, m',
        mass_kg=kg, com_m=com.tolist(),
        inertia_kg_m2=dict(ixx=I[0, 0], ixy=I[0, 1], ixz=I[0, 2], iyy=I[1, 1], iyz=I[1, 2], izz=I[2, 2]),
        volume_cm3=m.volume / 1000, bbox_mm=[round(float(v), 2) for v in (b[0, 0], b[1, 0], b[0, 1], b[1, 1], b[0, 2], b[1, 2])],
        arc_radius_mm=ARC_R_MM, cups=cl, collision_meshes=files, light_boxes=light,
        collision_note=f'바닥판 z 0~3 볼록 기둥 {n_plate} 개 (격자 구멍 메움, 안쪽 컵 자리 z 2~3 비움) + 컵 면별 볼록 껍질 {len(hulls) - n_plate} 개')
    (OUT / 'puck_plate.json').write_text(json.dumps(info, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
    print(f'puck_plate: {kg * 1000:.1f} g, 무게중심 {np.round(com * 1000, 1)} mm, 컵 {len(cl)} 개 '
          f'(턱 높이 {sorted({c["seat_mm"] for c in cl})} mm, 벽 윗면 {sorted({c["rim_mm"] for c in cl})} mm)')


if __name__ == '__main__':
    main()
