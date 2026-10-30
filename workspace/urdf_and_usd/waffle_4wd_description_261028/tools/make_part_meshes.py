"""센서 · 부품 겉모양 STL 과 질량 특성 JSON 을 meshes/parts/ 에 만든다. FreeCAD 로 실행 (패키지 폴더에서, Jetson STEP 읽기 포함 약 1 분):

    freecadcmd -c "exec(open('tools/make_part_meshes.py').read())"

- lds03: LDS-03 (COIN-D4). 치수 [mm] = 사용자 Fusion 모델 COIN_D4_OPTICAL_ORIGIN_REFERENCE.f3d (2026-10-27) 안의 형상 (ASM) 값.
  원점 = 광학 중심 (스캔면), +z = 머리 쪽, +x = f3d 의 +x (LiDAR 0° 로 가정, 실물 확인 필요).
  f3d 외형은 ROBOTIS 공식 (54.7 W x 39.7 D x 34 H, 42 g) 보다 1.5~2 mm 작다.
- tfmini_plus: Benewake TFmini Plus. 35 (W) x 18.5 (H) x 21 (D), 앞면 모서리 R2, 뒷면 M2 구멍 2 개 대각 28 x 11.5
  (매뉴얼 SJ-PM-TFmini Plus A02 Figure 4: 뒤에서 본 그림에서 왼쪽 위 · 오른쪽 아래, 케이블은 위).
  원점 = 앞면 (렌즈) 가운데, +x = 측정 방향, +y = 왼쪽, +z = 케이블 쪽.
- tfmini_bracket_v2: 사용자 브라켓 ~/Desktop/CartRider/tfmini_plus_bracket_v2_print.stl (mm) 을 링크 좌표로 옮겨 m 로 쓴다.
  링크 좌표 = STL 을 평행 이동만: 원점 = 벽 바깥 면 (STL x 43) 가운데 밑면, +x = TFmini 측정 방향 (= STL +x), 바닥판 · 받침 턱은 벽 뒤 (-x).
  TFmini 는 벽 바깥 면에 뒷면을 대고 벽 구멍 (STL y 8 z 17 · y 36 z 28.5) 에 M2 로 고정 -> 렌즈 가운데 = (21, 0, 22.75).
  이 방향에서만 TFmini 뒷면 구멍 대각 (왼쪽 위 · 오른쪽 아래) 이 벽 구멍과 맞는다 (벽 안쪽에 대면 대각이 엇갈림, 2026-10-28 확인).
- jetson: NVIDIA Jetson Orin Nano 개발자 키트 STEP (~/Downloads/Jetson_Orin_Nano_DevKit_3D_STEP_Model/P3766-P3768SKU4-P3767ENVELOPE.stp).
  링크 좌표: 원점 = 외곽 밑면 가운데 (받침 다리 바닥), 입출력 단자 변 (STEP -y) -> -x (사용자 2026-10-28: 단자는 로봇 뒤), +z = 팬 쪽.
  겉모양은 부피 20 mm³ 이상 부품만 0.5 mm 편차로 (부피 99 %), 질량 특성은 부품 전부 (1393 개) 를 고르게 176 g 으로.
"""
import json
import math
import os

import FreeCAD as App
import Mesh
import MeshPart
import Part

HERE = os.path.abspath('.')
OUT = os.path.join(HERE, 'meshes/parts')
BRACKET_SRC = os.path.expanduser('~/Desktop/CartRider/tfmini_plus_bracket_v2_print.stl')
PLA_KG_M3, PLA_FILL = 1240.0, 0.55        # 출력 부품 가정 (make_revision.py 와 같음)


def V(x, y, z):
    return App.Vector(x, y, z)


def mass_props(shape, mass_kg):
    """고르게 퍼진 질량으로 무게중심 [m] · 관성 [kg m2] (무게중심 기준)."""
    vol = shape.Volume
    com = shape.CenterOfMass
    I = shape.MatrixOfInertia
    k = mass_kg / vol * 1e-6
    return dict(mass_kg=mass_kg, com_m=[com.x / 1000, com.y / 1000, com.z / 1000],
                inertia_kg_m2=dict(ixx=I.A11 * k, ixy=I.A12 * k, ixz=I.A13 * k, iyy=I.A22 * k, iyz=I.A23 * k, izz=I.A33 * k),
                volume_cm3=vol / 1000)


def write(name, shape_or_mesh, info):
    m = shape_or_mesh if isinstance(shape_or_mesh, Mesh.Mesh) else \
        MeshPart.meshFromShape(Shape=shape_or_mesh, LinearDeflection=0.05, AngularDeflection=0.2)
    m = m.copy()
    m.transform(App.Matrix(0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 1))   # mm -> m
    m.write(os.path.join(OUT, name + '.stl'))
    json.dump(info, open(os.path.join(OUT, name + '.json'), 'w'), indent=1)
    bb = m.BoundBox
    print(f'{name}: 외곽 {bb.XLength * 1000:.1f} x {bb.YLength * 1000:.1f} x {bb.ZLength * 1000:.1f} mm, '
          f'{info["mass_kg"] * 1000:.1f} g, 삼각형 {m.CountFacets}')


def one_solid(s):
    s = s.removeSplitter()
    assert s.isValid() and len(s.Solids) == 1, '형상 오류'
    return s.Solids[0]


# ---------- LDS-03 ----------
BODY, BODY_Z, BODY_R = 38.2, (-26.5, -5.4), 2.0     # 몸체 가로 = 세로, 밑면 (장착면) ~ 윗면, 세로 모서리 둥글기
HEAD_R, HEAD_Z = 17.65, (-5.4, 7.9)                 # 회전 머리
TAB_R, HOLE_R, TAB_Z = 4.3, 1.6, (-26.5, -21.0)     # 귀 끝 반지름, 구멍 (Ø3.2), 귀 두께 5.5 (밑면과 같은 면)
HOLES = [(-21.8, 0.0, 'x'), (9.0, 22.0, 'y'), (9.0, -22.0, 'y')]


def lds03():
    h, t = BODY / 2, TAB_Z[1] - TAB_Z[0]
    body = Part.makeBox(BODY, BODY, BODY_Z[1] - BODY_Z[0], V(-h, -h, BODY_Z[0]))
    body = body.makeFillet(BODY_R, [e for e in body.Edges if abs(e.Vertexes[0].Point.z - e.Vertexes[-1].Point.z) > 1])
    s = body.fuse(Part.makeCylinder(HEAD_R, HEAD_Z[1] - HEAD_Z[0], V(0, 0, HEAD_Z[0])))
    for x, y, d in HOLES:
        end = Part.makeCylinder(TAB_R, t, V(x, y, TAB_Z[0]))
        if d == 'x':                      # 몸체 -x 면에서 구멍 중심까지 (몸체 안으로 0.5 겹침)
            arm = Part.makeBox(abs(x) - h + 0.5, 2 * TAB_R, t, V(x, -TAB_R, TAB_Z[0]))
        else:                             # 몸체 ±y 면에서 구멍 중심까지
            arm = Part.makeBox(2 * TAB_R, abs(y) - h + 0.5, t, V(x - TAB_R, h - 0.5 if y > 0 else y, TAB_Z[0]))
        s = s.fuse(end.fuse(arm))
    for x, y, _ in HOLES:
        s = s.cut(Part.makeCylinder(HOLE_R, t + 2, V(x, y, TAB_Z[0] - 1)))
    s = one_solid(s)
    bb = s.BoundBox
    info = mass_props(s, 0.042)           # ROBOTIS e-Manual appendix_lds_03
    info.update(bbox_mm=[bb.XMin, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax], body_box_mm=[BODY, BODY, BODY_Z[0], BODY_Z[1]],
                head_mm=[HEAD_R, HEAD_Z[0], HEAD_Z[1]], holes_mm=[[x, y, 2 * HOLE_R] for x, y, _ in HOLES], mount_face_z_mm=BODY_Z[0])
    write('lds03', s, info)


# ---------- TFmini Plus ----------
TF_W, TF_H, TF_D, TF_R = 35.0, 18.5, 21.0, 2.0
TF_HOLES = [(14.0, 5.75), (-14.0, -5.75)]           # 뒷면 M2 (y, z): 뒤에서 보면 왼쪽 위 · 오른쪽 아래
TF_WINDOW = (15.5, 14.0, 0.8)                        # 렌즈 창 2 개 (가로, 세로, 두 창 사이), 0.3 mm 들어감 (그림 비율, 표시용)
TF_GLAND = (2.0, 3.0, 2.7)                          # 케이블 출구 반지름 · 높이 · 뒷면에서 거리 (윗면; 크기 · 거리는 그림 비율, 미확정)


def tfmini():
    s = Part.makeBox(TF_D, TF_W, TF_H, V(-TF_D, -TF_W / 2, -TF_H / 2))
    s = s.makeFillet(TF_R, [e for e in s.Edges if abs(e.Vertexes[0].Point.x - e.Vertexes[-1].Point.x) > 1])
    w, hh, gap = TF_WINDOW
    for sy in (-1, 1):
        cy = sy * (gap / 2 + w / 2)
        win = Part.makeBox(0.3, w, hh, V(-0.3, cy - w / 2, -hh / 2))
        win = win.makeFillet(1.5, [e for e in win.Edges if abs(e.Vertexes[0].Point.x - e.Vertexes[-1].Point.x) > 0.2])
        s = s.cut(win)
    for y, z in TF_HOLES:
        s = s.cut(Part.makeCylinder(1.0, 4.0, V(-TF_D, y, z), V(1, 0, 0)))
    r, gh, gx = TF_GLAND
    s = s.fuse(Part.makeCylinder(r, gh, V(-TF_D + gx, 0, TF_H / 2), V(0, 0, 1)))
    s = one_solid(s)
    info = mass_props(s, 0.012)           # 데이터시트 12 ± 1 g (1안 build_rover.py 기록)
    info.update(box_mm=[TF_D, TF_W, TF_H], fov_deg=3.6, range_m=[0.1, 12.0], rate_hz=100)
    write('tfmini_plus', s, info)


# ---------- TFmini 브라켓 (사용자 v2) ----------
BRACKET_CENTER_Y = 22.0        # STL 에서 TFmini 가운데 y (벽 구멍 8 · 36 의 가운데 = 바닥판 44 의 가운데)
WALL_OUT_X = 43.0              # STL 벽 바깥 면 x (= 링크 원점)
LENS_STL = (64.0, 22.0, 22.75) # STL 에서 렌즈 가운데: 벽 바깥 면 43 + 깊이 21, 벽 구멍 z 17 · 28.5 의 가운데


def bracket():
    src = Mesh.Mesh(BRACKET_SRC)
    # STL (mm) -> 링크 좌표 (mm): 평행 이동만 (벽 바깥 면 가운데 밑면이 원점)
    src.transform(App.Matrix(1, 0, 0, -WALL_OUT_X, 0, 1, 0, -BRACKET_CENTER_Y, 0, 0, 1, 0, 0, 0, 0, 1))
    sh = Part.Shape()
    sh.makeShapeFromMesh(src.Topology, 0.01)
    s = Part.makeSolid(Part.Shell(sh.Faces) if not sh.Shells else sh.Shells[0])
    assert s.isValid() and abs(s.Volume - abs(src.Volume)) < 1, '브라켓 메시 -> 솔리드 변환 오류'
    vol_m3 = s.Volume * 1e-9
    info = mass_props(s, vol_m3 * PLA_KG_M3 * PLA_FILL)
    bb = src.BoundBox
    lens = [LENS_STL[0] - WALL_OUT_X, LENS_STL[1] - BRACKET_CENTER_Y, LENS_STL[2]]
    # 충돌 상자 (링크 좌표 mm): 바닥판 · 벽 (STL x 38~43) · 받침 턱 (STL x 18.5~38, y 19~25, z ~12) (STL 분석 값)
    pts = [p for p in src.Points if p.x > 38.5 - WALL_OUT_X and p.z > 4.0]
    wall_y = (min(p.y for p in pts), max(p.y for p in pts))
    boxes = [dict(name='base', min=[bb.XMin, bb.YMin, 0.0], max=[bb.XMax, bb.YMax, 3.5]),
             dict(name='wall', min=[38.0 - WALL_OUT_X, wall_y[0], 3.5], max=[0.0, wall_y[1], 32.0]),
             dict(name='ledge', min=[18.5 - WALL_OUT_X, 19.0 - BRACKET_CENTER_Y, 3.5], max=[38.0 - WALL_OUT_X, 25.0 - BRACKET_CENTER_Y, 12.0])]
    info.update(source=BRACKET_SRC, bbox_mm=[bb.XMin, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax], lens_mm=lens,
                collision_boxes_mm=boxes, pla_fill=PLA_FILL)
    write('tfmini_plus_bracket_v2', src, info)


# ---------- Jetson Orin Nano 개발자 키트 ----------
JETSON_STEP = os.path.expanduser('~/Downloads/Jetson_Orin_Nano_DevKit_3D_STEP_Model/P3766-P3768SKU4-P3767ENVELOPE.stp')
JETSON_KG = 0.176              # JetsonHacks 실측 (전원 어댑터 제외), NVIDIA 사양 0.175 kg (1안 build_rover.py 기록)
JETSON_VIS_MIN_MM3 = 20.0      # 이보다 작은 부품 (칩 부품 등) 은 겉모양에서 뺌
JETSON_DEFL = (0.5, 0.8)       # 겉모양 메시 편차 [mm, rad]
JETSON_UPPER_Z = 20.0          # STEP z 이 값을 넘는 부품 = 방열판 · 팬 (충돌 상자 위쪽), 나머지 = 보드 · 단자 (아래쪽)


def jetson():
    sh = Part.read(JETSON_STEP)
    bb0 = sh.BoundBox
    upper0 = [s.BoundBox for s in sh.Solids if s.BoundBox.ZMax > JETSON_UPPER_Z]
    lower0 = [s.BoundBox for s in sh.Solids if s.BoundBox.ZMax <= JETSON_UPPER_Z]
    T = App.Matrix()
    T.move(V(-(bb0.XMin + bb0.XMax) / 2, -(bb0.YMin + bb0.YMax) / 2, -bb0.ZMin))
    R = App.Matrix()
    R.rotateZ(-math.pi / 2)                                  # STEP -y (단자) -> -x
    T = R.multiply(T)
    sh.transformShape(T)
    solids = sh.Solids
    # 질량 특성: 부품마다 (부피, 무게중심, 관성) 을 합침 -> 고른 밀도로 JETSON_KG
    vol = sum(s.Volume for s in solids)
    com = sum((s.CenterOfMass * s.Volume for s in solids), V(0, 0, 0)) * (1 / vol)
    I = [[0.0] * 3 for _ in range(3)]
    for s in solids:
        Mi, d = s.MatrixOfInertia, s.CenterOfMass - com
        Ii = [[Mi.A11, Mi.A12, Mi.A13], [Mi.A21, Mi.A22, Mi.A23], [Mi.A31, Mi.A32, Mi.A33]]
        dv, dd = [d.x, d.y, d.z], d.x ** 2 + d.y ** 2 + d.z ** 2
        for a in range(3):
            for b in range(3):
                I[a][b] += Ii[a][b] + s.Volume * ((dd if a == b else 0.0) - dv[a] * dv[b])
    k = JETSON_KG / vol * 1e-6
    info = dict(mass_kg=JETSON_KG, com_m=[com.x / 1000, com.y / 1000, com.z / 1000],
                inertia_kg_m2=dict(ixx=I[0][0] * k, ixy=I[0][1] * k, ixz=I[0][2] * k, iyy=I[1][1] * k, iyz=I[1][2] * k, izz=I[2][2] * k),
                volume_cm3=vol / 1000, source=JETSON_STEP, parts=len(solids))

    def box_of(bbs):                                         # STEP 외곽 상자들 -> 링크 좌표 외곽 상자 [mm]
        pts = [T.multVec(V(x, y, z)) for b in bbs for x in (b.XMin, b.XMax) for y in (b.YMin, b.YMax) for z in (b.ZMin, b.ZMax)]
        return [min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)], [max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)]
    lo, hi = box_of(lower0)
    ulo, uhi = box_of(upper0)
    bb = sh.BoundBox
    info.update(bbox_mm=[bb.XMin, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax],
                collision_boxes_mm=[dict(name='board', min=lo, max=hi), dict(name='heatsink', min=ulo, max=uhi)])
    m = Mesh.Mesh()
    for s in solids:
        if s.Volume >= JETSON_VIS_MIN_MM3:
            m.addMesh(MeshPart.meshFromShape(Shape=s, LinearDeflection=JETSON_DEFL[0], AngularDeflection=JETSON_DEFL[1]))
    write('jetson_orin_nano', m, info)


os.makedirs(OUT, exist_ok=True)
lds03()
tfmini()
bracket()
jetson()
