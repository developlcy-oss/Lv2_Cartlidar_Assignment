#!/usr/bin/env python3
"""시뮬용 '알려진 물체' 지도: map_nav (벽 · 상자 · 받침대 · 신호등, SDF 자리) 에서 상자 · L 9 개를 지우고 실측 자리 (길 모음 BOX) 에 다시 그림.
   python3 make_known_map.py <mission_planner/maps> <나갈 이름 (확장자 없이)>
Gazebo 상자도 같은 실측 자리로 옮김 (scenario.py --move-boxes) → 길 모음 · 감시 · Gazebo 가 같은 세계.
"""
import math
import os
import sys

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import replan  # noqa: E402,F401  (PLAN_DIR 경로 설정)
import make_legs as ml  # noqa: E402

SDF_XY = {"start_box_1": (1.468, 0.950), "start_box_2": (3.670, 3.152), "start_box_3": (2.936, 5.354), "start_box_4": (0.734, 5.354),
          "start_box_5": (2.202, 5.354), "start_box_6": (3.670, 3.886), "place_L1": (3.670, 0.950), "place_L2": (2.202, 3.700), "place_L3": (3.670, 5.354)}
HALF = 0.205 / 2


def main():
    mp, out = sys.argv[1], sys.argv[2]
    m = yaml.safe_load(open(os.path.join(mp, "map_nav.yaml")))
    nav = np.array(Image.open(os.path.join(mp, m["image"])))[::-1].copy()
    amcl = np.array(Image.open(os.path.join(mp, "map_amcl.pgm")))[::-1]
    res, ox, oy = m["resolution"], m["origin"][0], m["origin"][1]
    walls = ndimage.binary_dilation(amcl < 90, iterations=1)
    obj = (nav < 90) & ~walls
    lab, n = ndimage.label(obj)
    erased = 0
    for k in range(1, n + 1):
        jj, ii = np.nonzero(lab == k)
        cx, cy = ox + (ii.mean() + 0.5) * res, oy + (jj.mean() + 0.5) * res
        if min(math.hypot(cx - x, cy - y) for x, y in SDF_XY.values()) < 0.2:
            nav[lab == k] = 254
            erased += 1
    H, W = nav.shape
    X, Y = np.meshgrid(ox + (np.arange(W) + 0.5) * res, oy + (np.arange(H) + 0.5) * res)
    for name, (x, y) in ml.BOX.items():
        nav[(np.abs(X - x) <= HALF + res / 2) & (np.abs(Y - y) <= HALF + res / 2)] = 0
    Image.fromarray(nav[::-1]).save(out + ".pgm")
    m2 = dict(m)
    m2["image"] = os.path.basename(out) + ".pgm"
    yaml.safe_dump(m2, open(out + ".yaml", "w"), sort_keys=False)
    print(f"{out}.yaml: SDF 상자 덩어리 {erased} 개 지움 · 실측 자리 {len(ml.BOX)} 개 그림")


if __name__ == "__main__":
    main()
