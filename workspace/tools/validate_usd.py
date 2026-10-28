"""시뮬레이션 USD 정적 검사 (Isaac 없이, usd-core 만). 물리 · 렌더링 · ROS 연결은 보지 않는다.

  python3 -m venv ~/.venv-usdcheck && ~/.venv-usdcheck/bin/pip install usd-core
  ~/.venv-usdcheck/bin/python validate_usd.py <로봇.usda> [--out 결과.json]
  예: ~/Downloads/urdf_test/waffle_4wd_usd_261027_1711/waffle_4wd/waffle_4wd.usda (규약: workspace/docs/frames_tf.md)

본다: 못 찾는 참조 파일 · 미터 · Z 위, 다른 노드가 쓰는 프레임 이름 (frames_tf.md), 광학 프레임 축 (+z 앞 · +x 오른쪽),
      라이다 스캔면 방향 (뒤집어 장착이면 z 아래), 바퀴 간격 (drive_<belt|direct>.yaml 과 비교값만 출력).
"""
import argparse
import json
import sys

from pxr import Gf, Usd, UsdGeom, UsdUtils

# frames_tf.md 표: realsense2_camera (camera_name top_camera_front · top_camera_rear) 컬러 frame_id, Arducam, 라이다 스캔
OPTICAL = {'top_camera_front_color_optical_frame': 'top_camera_front_link',
           'top_camera_rear_color_optical_frame': 'top_camera_rear_link',
           'camera_optical_frame': 'camera_link'}
LIDAR = ['lidar_front_scan', 'lidar_rear_scan']
WHEEL_Y = 'wheel_front_left'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('usd')
    ap.add_argument('--out')
    args = ap.parse_args()

    _, _, unresolved = UsdUtils.ComputeAllDependencies(args.usd)
    stage = Usd.Stage.Open(args.usd)
    prims = {}
    for p in stage.Traverse():   # 같은 이름의 겉모양 자식 (wheel_front_left/wheel_front_left) 보다 링크 (먼저 나옴) 를 쓴다
        if p.IsA(UsdGeom.Xformable):
            prims.setdefault(p.GetName(), p)
    cache = UsdGeom.XformCache()
    to_base = cache.GetLocalToWorldTransform(prims['base_link']).GetInverse()

    def in_base(name):
        return cache.GetLocalToWorldTransform(prims[name]) * to_base

    def axis(m, v):
        return m.TransformDir(Gf.Vec3d(*v)).GetNormalized()

    fail, info = [], {}
    if unresolved:
        fail.append(f'못 찾는 참조: {unresolved}')
    if UsdGeom.GetStageMetersPerUnit(stage) != 1.0 or UsdGeom.GetStageUpAxis(stage) != 'Z':
        fail.append('미터 · Z 위가 아님')

    # 광학 프레임: z = 몸체 x (앞), x = 몸체 -y (오른쪽)
    for opt, body in OPTICAL.items():
        if opt not in prims or body not in prims:
            fail.append(f'프레임 없음: {opt} / {body}')
            continue
        o, b = in_base(opt), in_base(body)
        dz = Gf.Dot(axis(o, (0, 0, 1)), axis(b, (1, 0, 0)))
        dx = Gf.Dot(axis(o, (1, 0, 0)), axis(b, (0, -1, 0)))
        info[opt] = dict(xyz=list(o.ExtractTranslation()), z_dot_body_x=dz, x_dot_body_minus_y=dx)
        if dz < 0.999 or dx < 0.99:   # 컬러는 깊이 원점과 약간 어긋남 (실측 외부보정 반영 USD)
            fail.append(f'{opt} 광학 축이 규약과 다름')

    # 라이다: 스캔 프레임 z 가 아래면 뒤집어 장착 (URDF laser_* 에 roll π 필요)
    for name in LIDAR:
        if name not in prims:
            fail.append(f'프레임 없음: {name}')
            continue
        m = in_base(name)
        info[name] = dict(xyz=list(m.ExtractTranslation()), upside_down=axis(m, (0, 0, 1))[2] < 0)

    if WHEEL_Y in prims:
        info['wheel_separation'] = 2 * abs(in_base(WHEEL_Y).ExtractTranslation()[1])

    report = dict(status='FAIL' if fail else 'PASS_STATIC_ONLY', usd=args.usd, fail=fail, frames=info,
                  not_checked=['물리 (질량 · 관절 구동)', 'Isaac Sim 렌더링', 'ROS 토픽 · TF 발행', 'URDF 와 위치 일치'])
    text = json.dumps(report, indent=2, ensure_ascii=False, default=float)
    print(text)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            f.write(text + '\n')
    return 1 if fail else 0


if __name__ == '__main__':
    sys.exit(main())
