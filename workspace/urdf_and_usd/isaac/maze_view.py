"""주사위 경기장 + 로봇 장면 (scenes/dice_mission_waffle_*.usda) 을 Isaac Sim 창으로 열어 로봇 쪽을 비추고 재생한다 (./run.sh maze).

    ./run.sh maze [--light] [--paused]      창을 닫으면 끝
- 기본은 정밀 판 장면. 카메라는 로봇 뒤 왼쪽 위에서 로봇이 보는 쪽을 봄 (뷰포트에서 마우스로 자유롭게 바꿔도 됨).
- --shot <png>: 창 없이 열어 뷰포트 그림만 저장하고 끝 (점검용).
- Isaac Lab AppLauncher 대신 Isaac Sim SimulationApp 을 씀: Isaac Lab 실행기는 재생 (Play) 해도 물리를 돌리지 않아 (코드가 직접 진행)
  팔이 처음 자리 (영 자세) 에 멈춰 보인다 (2026-10-30 확인). SimulationApp 은 재생하면 물리가 돌아 장면의 시작 자세 (주행) 를 잡는다.
"""
import argparse
import math
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser(description="경기장 장면 보기")
ap.add_argument("--light", action="store_true", help="가벼운 판 장면")
ap.add_argument("--paused", action="store_true", help="재생하지 않고 열기만")
ap.add_argument("--shot", default=None, help="창 없이 뷰포트 그림만 저장 (점검용)")
args, _ = ap.parse_known_args()          # run.sh 가 넘기는 --viz kit 등은 무시
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": bool(args.shot), "width": 1600, "height": 900})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

SCENE = os.path.join(HERE, "scenes", f"dice_mission_waffle_{'light' if args.light else 'full'}.usda")


def robot_pose(path):
    """장면 파일에 적힌 로봇 자리 x, y 와 방향 yaw (make_maze_scene.py 가 씀)."""
    txt = open(path, encoding="utf-8").read()
    m = re.search(r"xformOp:translate = \(([-\d.]+), ([-\d.]+), ([-\d.]+)\)", txt)
    q = re.search(r"xformOp:orient = \(([-\d.]+), 0, 0, ([-\d.]+)\)", txt)
    return float(m.group(1)), float(m.group(2)), 2 * math.atan2(float(q.group(2)), float(q.group(1)))


def main():
    ctx = omni.usd.get_context()
    ctx.open_stage(SCENE)
    for _ in range(30):
        app.update()
    stage = ctx.get_stage()
    assert stage.GetPrimAtPath("/World/waffle_4wd/Geometry/base_link").IsValid(), f"로봇이 장면에 없음: {SCENE}"

    # 보기 카메라 (세션 층에만 만들어 장면 파일은 그대로): 로봇 뒤 왼쪽 위
    x, y, yaw = robot_pose(SCENE)
    c, sn = math.cos(yaw), math.sin(yaw)
    loc = lambda f, l, z: Gf.Vec3d(x + f * c - l * sn, y + f * sn + l * c, z)     # 로봇 좌표 (앞, 왼쪽) -> 지도
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        cam = UsdGeom.Camera.Define(stage, "/World/ViewCamera")
        cam.GetFocalLengthAttr().Set(16.0)
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))
        eye, target = loc(-1.0, 0.6, 1.0), loc(0.35, 0.0, 0.12)                     # 뒤 왼쪽 위 -> 로봇 앞
        UsdGeom.Xformable(cam).AddTransformOp().Set(Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse())
    from omni.kit.viewport.utility import get_active_viewport

    vp = get_active_viewport()
    vp.camera_path = "/World/ViewCamera"

    if not args.paused:
        omni.timeline.get_timeline_interface().play()
    if args.shot:
        from omni.kit.viewport.utility import capture_viewport_to_file

        for _ in range(240):
            app.update()
        capture_viewport_to_file(vp, os.path.abspath(args.shot))
        for _ in range(30):
            app.update()
        print(f"[maze_view] 그림 저장: {args.shot}")
        return
    print(f"[maze_view] {os.path.relpath(SCENE, HERE)} 열림. 창을 닫으면 끝")
    while app.is_running():
        app.update()


if __name__ == "__main__":
    main()
    app.close()
