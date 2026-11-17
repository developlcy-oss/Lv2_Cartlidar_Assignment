"""maze_real_aligned 에서 상자 · 받침대 (이름 붙은 물체 덩어리) 만 빈칸으로 → 벽만 지도 (상자가 옮겨져도 AMCL 이 덜 헷갈리게)."""
import math, os, numpy as np, yaml
from PIL import Image
from scipy import ndimage
mp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "ros2_ws", "src", "mission_planner", "maps")  # 저장소 발제 지도
m = yaml.safe_load(open("maze_real_aligned.yaml")); s = np.array(Image.open("maze_real_aligned.pgm"))[::-1].copy()
w = np.array(Image.open(f"{mp}/map_amcl.pgm"))[::-1]; res = m["resolution"]; ox, oy = m["origin"][:2]
Dw = ndimage.distance_transform_edt(~(w < 90)) * res
names = {"box1": (1.468, 0.950), "box2": (3.670, 3.152), "box3": (2.936, 5.354), "box4": (0.734, 5.354), "box5": (2.202, 5.354), "box6": (3.670, 3.886), "L1": (3.670, 0.950), "L2": (2.202, 3.700), "L3": (3.670, 5.354)}
obj = (s < 90) & (Dw > 0.08); lab, n = ndimage.label(ndimage.binary_dilation(obj, iterations=3))
cleared = 0
for k in range(1, n + 1):
    sel = (lab == k); yy, xx = np.nonzero(sel & obj)
    if len(yy) < 8: continue
    cx, cy = ox + (xx.mean() + 0.5) * res, oy + (yy.mean() + 0.5) * res
    if min(math.hypot(a - cx, b - cy) for a, b in names.values()) < 0.35:
        s[sel & (s < 250)] = 254; cleared += 1   # 덩어리와 둘레 (회색 '모름' 포함) 를 빈칸으로
Image.fromarray(s[::-1]).save("maze_real_walls.pgm")
m2 = dict(m); m2["image"] = "maze_real_walls.pgm"; yaml.safe_dump(m2, open("maze_real_walls.yaml", "w"), sort_keys=False)
print(f"물체 덩어리 {cleared} 개 지움 → maze_real_walls.pgm/.yaml")
