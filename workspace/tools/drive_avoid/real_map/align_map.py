"""SLAM 지도 (map = 시작 때 odom) → 발제 지도 좌표로 맞춤 (벽 거리장 정합) · 비교 그림 · 발제 좌표 지도 (pgm/yaml) 저장.
   python3 align_map.py <slam 이름> <발제 map_amcl.yaml> <amcl map→odom csv> <출력 이름>"""
import sys, math, csv, yaml, numpy as np
from PIL import Image
from scipy import ndimage, optimize
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.rcParams["font.family"] = ["NanumGothic", "DejaVu Sans"]
slam, ref_yaml, amcl_csv, out = sys.argv[1:5]

def load_map(y):
    m = yaml.safe_load(open(y)); img = np.array(Image.open(y.rsplit("/", 1)[0] + "/" + m["image"] if "/" in y else m["image"]))
    if img.ndim == 3: img = img[..., 0]
    img = img[::-1]  # 행 0 = y 최소
    p = img / 255.0 if not m.get("negate", 0) else 1 - img / 255.0
    occ = (1 - p) > m.get("occupied_thresh", 0.65); free = (1 - p) < m.get("free_thresh", 0.25)
    return m, occ, free
sm, s_occ, s_free = load_map(slam + ".yaml")
rm, r_occ, r_free = load_map(ref_yaml)
sres, (sox, soy, syaw) = sm["resolution"], sm["origin"]; rres, (rox, roy, _) = rm["resolution"], rm["origin"]
assert abs(syaw) < 1e-6
jj, ii = np.nonzero(s_occ)
P = np.c_[sox + (ii + 0.5) * sres, soy + (jj + 0.5) * sres]  # SLAM 점유 칸 가운데 (slam map 좌표)
D = ndimage.distance_transform_edt(~r_occ) * rres
def dist_ref(Q):
    c = (Q[:, 0] - rox) / rres - 0.5; r = (Q[:, 1] - roy) / rres - 0.5
    return ndimage.map_coordinates(D, [r, c], order=1, mode="constant", cval=1.0)
def tf(x, Q):
    c, s = math.cos(x[2]), math.sin(x[2]); return np.c_[x[0] + c * Q[:, 0] - s * Q[:, 1], x[1] + s * Q[:, 0] + c * Q[:, 1]]

# 첫 추정: 경기장 전체 거친 찾기 (녹화 때 AMCL 은 위치를 잃어서 못 씀 — 11-17 진단)
rng = np.random.default_rng(0); S = P[rng.choice(len(P), min(500, len(P)), replace=False)]
c0 = P.mean(0)
cands = []
for th in np.radians(np.arange(0, 360, 2)):
    c, s_ = math.cos(th), math.sin(th); R = np.array([[c, -s_], [s_, c]])
    rq = (S - c0) @ R.T
    xs, ys = np.arange(0.0, 4.5, 0.05), np.arange(0.0, 6.4, 0.05)
    X, Y = np.meshgrid(xs, ys); X = X.ravel(); Y = Y.ravel()
    WX = X[:, None] + rq[None, :, 0]; WY = Y[:, None] + rq[None, :, 1]
    dd = ndimage.map_coordinates(D, [((WY - roy) / rres - 0.5).ravel(), ((WX - rox) / rres - 0.5).ravel()], order=1, mode="constant", cval=1.0).reshape(WX.shape)
    cost = np.mean(np.minimum(dd, 0.3), axis=1); k = int(np.argmin(cost))
    t_ = np.array([X[k], Y[k]]) - R @ c0          # 점 p → R p + t
    cands.append((cost[k], np.array([t_[0], t_[1], th])))
cands.sort(key=lambda c: c[0])
x0 = cands[0][1]
print(f"첫 추정 (전체 찾기): x {x0[0]:.3f} y {x0[1]:.3f} yaw {math.degrees(x0[2]):.1f}° · 평균 거리 {cands[0][0]*100:.1f} cm (다음 후보 {cands[1][0]*100:.1f} cm, yaw {math.degrees(cands[1][1][2]):.0f}°)")
print(f"  첫 추정 그대로: 점 {len(P)} 개 · 발제 벽까지 중앙 {np.median(dist_ref(tf(x0, P)))*100:.1f} cm")
# 거친 → 고운 정합 (멀리 떨어진 점은 실제 차이로 보고 덜 셈)
best = x0
for sc in (0.30, 0.15, 0.06, 0.03):
    res = optimize.least_squares(lambda x: np.minimum(dist_ref(tf(x, P)), sc), best, loss="soft_l1", f_scale=sc / 3, x_scale=[0.05, 0.05, 0.02])
    best = res.x
d = dist_ref(tf(best, P))
print(f"정합 결과: x {best[0]:.3f} y {best[1]:.3f} yaw {math.degrees(best[2]):.2f}°  (첫 추정과 {math.hypot(*(best[:2]-x0[:2]))*100:.1f} cm · {math.degrees(best[2]-x0[2]):.2f}°)")
print(f"SLAM 벽 점 → 발제 벽 거리: 중앙 {np.median(d)*100:.1f} cm · 90 % {np.percentile(d,90)*100:.1f} cm · 5 cm 넘는 점 {np.mean(d>0.05)*100:.1f} % · 10 cm 넘는 점 {np.mean(d>0.10)*100:.1f} %")
# 반대: 발제 벽 칸 중 SLAM 이 벽으로 못 본 것 (발제 지도에만 있는 벽)
Q = tf(best, P); Ds = np.full(r_occ.shape, 9.0)
qi = ((Q[:, 0] - rox) / rres).astype(int); qj = ((Q[:, 1] - roy) / rres).astype(int)
ok = (qi >= 0) & (qi < r_occ.shape[1]) & (qj >= 0) & (qj < r_occ.shape[0])
sm_on_ref = np.zeros(r_occ.shape, bool); sm_on_ref[qj[ok], qi[ok]] = True
Ds = ndimage.distance_transform_edt(~sm_on_ref) * rres
rj, ri = np.nonzero(r_occ); dr = Ds[rj, ri]
print(f"발제 벽 칸 → SLAM 벽 거리: 중앙 {np.median(dr)*100:.1f} cm · 10 cm 넘는 칸 {np.mean(dr>0.10)*100:.1f} % (실물에 없거나 라이다가 못 본 벽)")

# 발제 좌표 격자로 다시 그림 (같은 원점 · 크기 · 해상도 → map_amcl 대신 바로 씀)
H, W = r_occ.shape
gx, gy = np.meshgrid(rox + (np.arange(W) + 0.5) * rres, roy + (np.arange(H) + 0.5) * rres)
inv = np.array([-best[0], -best[1]]); c, s = math.cos(-best[2]), math.sin(-best[2])
G = np.c_[gx.ravel(), gy.ravel()] + inv; G = np.c_[c * G[:, 0] - s * G[:, 1], s * G[:, 0] + c * G[:, 1]]
si = np.floor((G[:, 0] - sox) / sres).astype(int); sj = np.floor((G[:, 1] - soy) / sres).astype(int)
inb = (si >= 0) & (si < s_occ.shape[1]) & (sj >= 0) & (sj < s_occ.shape[0])
outimg = np.full(H * W, 205, np.uint8)
idx = np.where(inb)[0]
outimg[idx[s_free[sj[inb], si[inb]]]] = 254
outimg[idx[s_occ[sj[inb], si[inb]]]] = 0
outimg = outimg.reshape(H, W)
Image.fromarray(outimg[::-1]).save(out + ".pgm")
yaml.safe_dump({"image": out.rsplit("/", 1)[-1] + ".pgm", "mode": "trinary", "resolution": rres, "origin": [rox, roy, 0.0],
                "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.25}, open(out + ".yaml", "w"), sort_keys=False)
print("저장:", out + ".pgm/.yaml (발제 좌표, 원점 · 크기 = map_amcl)")

# 그림
fig, axs = plt.subplots(1, 2, figsize=(16, 11), dpi=100)
ext = [rox, rox + W * rres, roy, roy + H * rres]
ax = axs[0]
bg = np.ones((H, W, 3)); bg[r_occ] = [0.55, 0.55, 0.55]
ax.imshow(bg, origin="lower", extent=ext)
sc = ax.scatter(Q[:, 0], Q[:, 1], c=np.clip(d * 100, 0, 15), s=2, cmap="turbo", vmin=0, vmax=15)
plt.colorbar(sc, ax=ax, shrink=0.6, label="SLAM 벽 점 ↔ 발제 벽 [cm]")
ax.set_title("회색 = 발제 지도 벽 (map_amcl) · 점 = 실물 SLAM 벽 (색 = 어긋남)"); ax.set_aspect("equal"); ax.grid(alpha=0.2)
ax = axs[1]
ax.imshow(outimg, origin="lower", extent=ext, cmap="gray", vmin=0, vmax=255)
ax.contour(np.arange(W) * rres + rox + rres / 2, np.arange(H) * rres + roy + rres / 2, r_occ.astype(float), levels=[0.5], colors="red", linewidths=0.6)
ax.set_title("실물 SLAM 지도 (발제 좌표로 맞춤) · 빨강 = 발제 벽 윤곽"); ax.set_aspect("equal")
for a_ in axs: a_.set_xlim(ext[0], ext[1]); a_.set_ylim(ext[2], ext[3])
fig.tight_layout(); fig.savefig(out + "_비교.png"); print("그림:", out + "_비교.png")
