"""발제 좌표로 맞춘 SLAM 지도에서: 벽만의 어긋남 (구역별) · 벽이 아닌 물체 (상자 · 받침대) 덩어리 위치 ↔ 지도 물체 (map_nav) 비교."""
import sys, math, numpy as np, yaml
from PIL import Image
from scipy import ndimage
mp = sys.argv[1]  # mission_planner/maps
def load(y):
    m = yaml.safe_load(open(y)); img = np.array(Image.open(y.rsplit("/", 1)[0] + "/" + m["image"]))[::-1]; return m, img
_, s = load("./maze_real_aligned.yaml"); m, w = load(f"{mp}/map_amcl.yaml"); _, nv = load(f"{mp}/map_nav.yaml")
res = m["resolution"]; ox, oy = m["origin"][:2]
s_occ, w_occ, n_occ = s < 90, w < 90, nv < 90
obj_ref = n_occ & ~ndimage.binary_dilation(w_occ, iterations=1)
Dw = ndimage.distance_transform_edt(~w_occ) * res
# 1) 벽 어긋남: SLAM 점유 중 발제 벽 0.12 m 안 (= 벽에 속한 것)
jj, ii = np.nonzero(s_occ); d = Dw[jj, ii]; wall = d < 0.12
print(f"SLAM 벽 칸 (발제 벽 12 cm 안) {wall.sum()} 개: 중앙 {np.median(d[wall])*100:.1f} cm · 90 % {np.percentile(d[wall],90)*100:.1f} cm · 5 cm 넘음 {np.mean(d[wall]>0.05)*100:.0f} %")
X, Y = ox + (ii + 0.5) * res, oy + (jj + 0.5) * res
print("  구역별 (벽 칸, 5 cm 넘는 비율 · 90 %):")
for name, (x0, x1, y0, y1) in {"서 바깥벽 x≈0": (-0.2, 0.15, 0, 6.4), "동 바깥벽 x≈4.4": (4.3, 4.7, 0, 6.4), "남 바깥벽 y≈0": (0, 4.5, -0.2, 0.15), "북 바깥벽 y≈6.3": (0, 4.5, 6.2, 6.6),
                               "안쪽 칸막이": (0.2, 4.25, 0.2, 6.15)}.items():
    k = wall & (X > x0) & (X < x1) & (Y > y0) & (Y < y1)
    if k.sum(): print(f"    {name:<14} {k.sum():5d} 칸 · 5 cm 넘음 {np.mean(d[k]>0.05)*100:3.0f} % · 90 % {np.percentile(d[k],90)*100:4.1f} cm")
# 2) 물체: 발제 벽에서 8 cm 넘게 떨어진 SLAM 점유 → 덩어리
objmask = s_occ & (Dw > 0.08)
lab, n = ndimage.label(ndimage.binary_dilation(objmask, iterations=3))
rlab, rn = ndimage.label(obj_ref)
refs = []
for k in range(1, rn + 1):
    yy, xx = np.nonzero(rlab == k)
    if len(yy) < 6: continue
    refs.append((ox + (xx.mean() + 0.5) * res, oy + (yy.mean() + 0.5) * res, (xx.max() - xx.min() + 1) * res, (yy.max() - yy.min() + 1) * res))
names = {"box1": (1.468, 0.950), "box2": (3.670, 3.152), "box3": (2.936, 5.354), "box4": (0.734, 5.354), "box5": (2.202, 5.354), "box6": (3.670, 3.886),
         "L1": (3.670, 0.950), "L2": (2.202, 3.700), "L3": (3.670, 5.354)}
print(f"\n벽이 아닌 SLAM 물체 덩어리 (칸 ≥ 8):")
rows = []
for k in range(1, n + 1):
    sel = (lab == k) & objmask; yy, xx = np.nonzero(sel)
    if len(yy) < 8: continue
    x0, x1 = ox + xx.min() * res, ox + (xx.max() + 1) * res; y0, y1 = oy + yy.min() * res, oy + (yy.max() + 1) * res
    # 20.5 cm 정사각형 가정: 한 변만 보이면 보이는 끝에서 반 변
    cx = (x0 + x1) / 2; cy = (y0 + y1) / 2
    nm = min(names, key=lambda q: math.hypot(names[q][0] - cx, names[q][1] - cy)); dn = math.hypot(names[nm][0] - cx, names[nm][1] - cy)
    rows.append((nm if dn < 0.35 else "?", cx, cy, x1 - x0, y1 - y0, len(yy), dn))
for nm, cx, cy, wx, wy, c, dn in sorted(rows, key=lambda r: r[0]):
    ref = names.get(nm)
    print(f"  {nm:<5} SLAM 가운데 ({cx:.3f}, {cy:.3f}) · 크기 {wx*100:4.1f} × {wy*100:4.1f} cm · 칸 {c:3d}" + (f" · 지도 ({ref[0]:.3f}, {ref[1]:.3f}) 와 Δx {(cx-ref[0])*100:+5.1f} Δy {(cy-ref[1])*100:+5.1f} cm" if ref and nm != '?' else ""))
print("\n지도 (map_nav) 물체 덩어리:", ", ".join(f"({a:.2f},{b:.2f} {c*100:.0f}×{d*100:.0f})" for a, b, c, d in refs))
