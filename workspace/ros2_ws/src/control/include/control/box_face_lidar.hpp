// 상자 앞면 라이다 측정 (2026-11-21, drive_node 상자 중앙 맞춤 교차확인) — ROS 의존 없음 → 단위 시험 (test_box_face_lidar.cpp)
//   tools/drive_demo/box_face.py find_box_face (PCA + RANSAC) 의 최소판. /scan_front 원본을 씀 (/scan_merged 는 차체 마스크
//   x ≤ 0.285 가 상자를 지움). LDS-03 range_min ≈ 0.12 · 라이다 x 0.1318 ≈ 앞 끝 0.1328 → 틈 < 약 0.14 면 못 믿음.
//   라이다 laser_front = base_link (0.1318, 0, 0.0525), rpy (π, 0, π) → 2D 행렬 [[−1, 0], [0, 1]]
//     (roll π 때문에 yaw 만 쓰면 y 부호가 틀림 → 반드시 2 × 2 행렬. drive_node 는 TF 에서 행렬을 뽑고 없으면 이 기본값)
//   기본값은 box_face.py FACE_CFG + FACE_CFG_LOOSE (실물 거리 잡음 σ ≈ 1 cm) / AVG_LOOSE (중앙값 ±4 cm · ±8°, 4 장 이상)
//   생략: 끝점 가림 판정 (실패 사유 문구만 다름), 결과는 같은 기하 정의
//   정의 (base_link, 측정 순간): (cx, cy) = 앞면 선분 중점, nyaw = 로봇 → 상자 법선 yaw,
//     gap = n·C − front_offset, lateral = 로봇 옆 어긋남 (box_center.hpp CenterError 와 같은 부호: + = 로봇이 상자 중심선 왼쪽
//     — box_face.py 의 lateral 과 부호 반대), yaw_err = nyaw (= 법선 − 로봇 yaw)
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace control
{

struct LaserTf2
{
  double tx = 0.1318, ty = 0.0;
  double r00 = -1.0, r01 = 0.0, r10 = 0.0, r11 = 1.0;   // base = t + R·laser (roll π 반영)
};

struct Pt2 { double x = 0.0, y = 0.0; };

inline std::vector<Pt2> lidar_scan_to_base(const std::vector<float> & ranges, double angle_min, double angle_inc,
  double range_min, double range_max, const LaserTf2 & tf = {})
{
  std::vector<Pt2> out;
  out.reserve(ranges.size());
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    if (!std::isfinite(r) || r < range_min || r > range_max) {continue;}
    const double th = angle_min + angle_inc * static_cast<double>(i);
    const double lx = r * std::cos(th), ly = r * std::sin(th);
    out.push_back({tf.tx + tf.r00 * lx + tf.r01 * ly, tf.ty + tf.r10 * lx + tf.r11 * ly});
  }
  return out;
}

struct FaceLidarConfig
{
  double x_min = 0.14, x_max = 0.60, y_abs = 0.30;   // 기본 창 (base_link)
  double expect_win = 0.10;      // 기대 앞면이 있으면 법선 ±win, 접선 ±(len_nom/2 + win)
  double angle_inc = 0.01745;    // 스캔 각 간격 (drive_node 는 msg 값으로 덮어씀)
  double laser_x = 0.1318, laser_y = 0.0;
  double clu_min = 0.03, clu_k = 3.0;
  double ransac_thr = 0.008;
  int ransac_max_src = 60;
  double refine_thr = 0.04;      // LOOSE
  double run_min = 0.025;        // LOOSE
  double run_k = 2.5;
  double rms_max = 0.015;        // LOOSE
  int min_ransac = 6;
  int min_inliers = 10;
  double len_nom = 0.205, len_tol = 0.025;
  double yaw_max = 0.436;        // 25°
  double front_offset = 0.1328;
  bool end_quant = true;
};

struct FaceLidar
{
  bool ok = false;
  std::string why;
  double cx = 0.0, cy = 0.0, nyaw = 0.0, length = 0.0, rms = 0.0;
  int n = 0;
  double gap = std::numeric_limits<double>::quiet_NaN();
  double lateral = std::numeric_limits<double>::quiet_NaN();
  double yaw_err = std::numeric_limits<double>::quiet_NaN();
};

namespace bfl
{
inline double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}

struct Line { double mx, my, phi; };

inline Line pca(const std::vector<Pt2> & p)
{
  double mx = 0, my = 0;
  for (const auto & q : p) {mx += q.x; my += q.y;}
  mx /= p.size(); my /= p.size();
  double sxx = 0, syy = 0, sxy = 0;
  for (const auto & q : p) {
    sxx += (q.x - mx) * (q.x - mx); syy += (q.y - my) * (q.y - my); sxy += (q.x - mx) * (q.y - my);
  }
  return {mx, my, 0.5 * std::atan2(2.0 * sxy, sxx - syy)};
}

inline double rms(const std::vector<Pt2> & p)
{
  const auto l = pca(p);
  const double tx = std::cos(l.phi), ty = std::sin(l.phi);
  double s = 0;
  for (const auto & q : p) {const double e = -(q.x - l.mx) * ty + (q.y - l.my) * tx; s += e * e;}
  return std::sqrt(s / p.size());
}

// 결정적 RANSAC (솎은 전수 쌍, MSAC 비용) → inlier 번호
inline std::vector<std::size_t> ransac(const std::vector<Pt2> & p, double thr, int max_src)
{
  const std::size_t n = p.size();
  std::vector<std::size_t> all(n);
  for (std::size_t i = 0; i < n; ++i) {all[i] = i;}
  if (n < 2) {return all;}
  const std::size_t step = std::max<std::size_t>(1, (n + max_src - 1) / max_src);
  const double thr2 = thr * thr;
  double best = std::numeric_limits<double>::infinity();
  Pt2 ba{}; double bnx = 0, bny = 0; bool found = false;
  for (std::size_t a = 0; a < n; a += step) {
    for (std::size_t b = a + step; b < n; b += step) {
      const double dx = p[b].x - p[a].x, dy = p[b].y - p[a].y, L = std::hypot(dx, dy);
      if (L < 0.02) {continue;}
      const double nx = -dy / L, ny = dx / L;
      double cost = 0;
      for (const auto & q : p) {const double e = (q.x - p[a].x) * nx + (q.y - p[a].y) * ny; cost += std::min(e * e, thr2);}
      if (cost < best) {best = cost; ba = p[a]; bnx = nx; bny = ny; found = true;}
    }
  }
  if (!found) {return all;}
  std::vector<std::size_t> out;
  for (std::size_t i = 0; i < n; ++i) {
    if (std::fabs((p[i].x - ba.x) * bnx + (p[i].y - ba.y) * bny) <= thr) {out.push_back(i);}
  }
  return out;
}

// inlier 직선 → 창 전체에서 thr 안 점 → 선 방향으로 이어진 구간 (중심 포함) 만, 2 번
inline std::vector<Pt2> refine(std::vector<Pt2> inl, const std::vector<Pt2> & all, const FaceLidarConfig & c)
{
  for (int it = 0; it < 2; ++it) {
    const auto l = pca(inl);
    const double tx = std::cos(l.phi), ty = std::sin(l.phi), nx = -ty, ny = tx;
    std::vector<std::pair<double, Pt2>> near;
    for (const auto & q : all) {
      if (std::fabs(-(q.x - l.mx) * ty + (q.y - l.my) * tx) <= c.refine_thr) {
        near.push_back({(q.x - l.mx) * tx + (q.y - l.my) * ty, q});
      }
    }
    if (near.size() < 2) {return inl;}
    std::sort(near.begin(), near.end(), [](const auto & a, const auto & b) {return a.first < b.first;});
    auto gap_ok = [&](const Pt2 & a, const Pt2 & b, double ga) {
        const double mx = 0.5 * (a.x + b.x) - c.laser_x, my = 0.5 * (a.y + b.y) - c.laser_y;
        const double r = std::hypot(mx, my);
        const double ci = std::fabs(mx * nx + my * ny) / std::max(r, 1e-9);
        return ga <= std::max(c.run_min, c.run_k * r * c.angle_inc / std::max(ci, 0.4));
      };
    std::size_t k = 0;
    for (std::size_t i = 1; i < near.size(); ++i) {if (std::fabs(near[i].first) < std::fabs(near[k].first)) {k = i;}}
    std::size_t lo = k, hi = k;
    while (lo > 0 && gap_ok(near[lo - 1].second, near[lo].second, near[lo].first - near[lo - 1].first)) {--lo;}
    while (hi + 1 < near.size() && gap_ok(near[hi].second, near[hi + 1].second, near[hi + 1].first - near[hi].first)) {++hi;}
    inl.clear();
    for (std::size_t i = lo; i <= hi; ++i) {inl.push_back(near[i].second);}
  }
  return inl;
}
}  // namespace bfl

// base_link 점 → 상자 앞면. expect (ok 인 이전 결과 · 사전값) 가 있으면 그 주변만
inline FaceLidar find_box_face_lidar(const std::vector<Pt2> & pts, const FaceLidarConfig & c = {},
  const FaceLidar * expect = nullptr)
{
  FaceLidar f;
  const double lx = c.laser_x, ly = c.laser_y;
  std::vector<Pt2> win;
  for (const auto & p : pts) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {continue;}
    if (!expect) {
      if (p.x >= c.x_min && p.x <= c.x_max && std::fabs(p.y) <= c.y_abs) {win.push_back(p);}
    } else {
      const double dx = p.x - expect->cx, dy = p.y - expect->cy;
      const double ce = std::cos(expect->nyaw), se = std::sin(expect->nyaw);
      if (std::fabs(dx * ce + dy * se) <= c.expect_win && std::fabs(-dx * se + dy * ce) <= c.len_nom / 2 + c.expect_win) {
        win.push_back(p);
      }
    }
  }
  if (static_cast<int>(win.size()) < c.min_inliers) {f.why = "창 안 점 " + std::to_string(win.size()) + " 개"; f.n = win.size(); return f;}
  std::sort(win.begin(), win.end(), [&](const Pt2 & a, const Pt2 & b) {
      return std::atan2(a.y - ly, a.x - lx) < std::atan2(b.y - ly, b.x - lx);
    });
  std::vector<std::vector<Pt2>> clusters(1);
  clusters[0].push_back(win[0]);
  for (std::size_t i = 1; i < win.size(); ++i) {
    const auto & a = win[i - 1], & b = win[i];
    const double r = std::max(std::hypot(a.x - lx, a.y - ly), std::hypot(b.x - lx, b.y - ly));
    if (std::hypot(b.x - a.x, b.y - a.y) > std::max(c.clu_min, c.clu_k * r * c.angle_inc)) {clusters.emplace_back();}
    clusters.back().push_back(b);
  }
  double best_score = std::numeric_limits<double>::infinity();
  FaceLidar best;
  std::string last_fail = "선분 없음";
  for (const auto & cl : clusters) {
    std::vector<Pt2> rest = cl;
    for (int pass = 0; pass < 2; ++pass) {   // L 자 모서리면 첫 선분을 빼고 한 번 더
      if (static_cast<int>(rest.size()) < c.min_inliers) {break;}
      const auto idx = bfl::ransac(rest, c.ransac_thr, c.ransac_max_src);
      std::vector<Pt2> inl, keep;
      std::vector<bool> used(rest.size(), false);
      for (auto i : idx) {inl.push_back(rest[i]); used[i] = true;}
      for (std::size_t i = 0; i < rest.size(); ++i) {if (!used[i]) {keep.push_back(rest[i]);}}
      rest.swap(keep);
      if (static_cast<int>(inl.size()) < c.min_ransac) {last_fail = "RANSAC 점 " + std::to_string(inl.size()) + " 개"; continue;}
      inl = bfl::refine(inl, win, c);
      if (static_cast<int>(inl.size()) < c.min_inliers) {last_fail = "선분 점 " + std::to_string(inl.size()) + " 개"; continue;}
      // 선분: 법선은 로봇 → 상자 쪽
      const auto l = bfl::pca(inl);
      double nx = std::sin(l.phi), ny = -std::cos(l.phi);
      if (nx * l.mx + ny * l.my < 0) {nx = -nx; ny = -ny;}
      const double tx = -ny, ty = nx, d = nx * l.mx + ny * l.my;
      double s0 = std::numeric_limits<double>::infinity(), s1 = -s0;
      Pt2 e0{}, e1{};
      for (const auto & q : inl) {
        const double s = tx * q.x + ty * q.y;
        if (s < s0) {s0 = s; e0 = q;}
        if (s > s1) {s1 = s; e1 = q;}
      }
      if (c.end_quant) {
        s0 -= std::hypot(e0.x - lx, e0.y - ly) * c.angle_inc / 2.0;
        s1 += std::hypot(e1.x - lx, e1.y - ly) * c.angle_inc / 2.0;
      }
      const double sm = 0.5 * (s0 + s1), len = s1 - s0;
      const double cx = nx * d + tx * sm, cy = ny * d + ty * sm, nyaw = std::atan2(ny, nx);
      const double r = bfl::rms(inl);
      if (std::fabs(bfl::wrap(nyaw)) > c.yaw_max) {last_fail = "기울기 큼"; continue;}
      if (std::fabs(len - c.len_nom) > c.len_tol) {last_fail = "길이 " + std::to_string(len).substr(0, 5) + " m"; continue;}
      if (r > c.rms_max) {last_fail = "RMS " + std::to_string(r * 1000).substr(0, 4) + " mm"; continue;}
      const double score = std::fabs(len - c.len_nom) / c.len_tol + std::hypot(cx, cy) / c.x_max;
      if (score < best_score) {
        best_score = score;
        best = FaceLidar{};
        best.ok = true; best.cx = cx; best.cy = cy; best.nyaw = nyaw; best.length = len; best.rms = r;
        best.n = static_cast<int>(inl.size());
      }
    }
  }
  if (!best.ok) {f.why = last_fail; f.n = win.size(); return f;}
  const double cn = std::cos(best.nyaw), sn = std::sin(best.nyaw);
  best.gap = cn * best.cx + sn * best.cy - c.front_offset;
  best.lateral = -(-sn * best.cx + cn * best.cy);   // 로봇 옆 (+ = 로봇이 중심선 왼쪽)
  best.yaw_err = bfl::wrap(best.nyaw);
  best.why = "길이 " + std::to_string(best.length).substr(0, 5) + " m · 점 " + std::to_string(best.n) + " 개";
  return best;
}

// 여러 장 → 중앙값 (중앙값에서 ±spread_xy · ±spread_yaw 밖 측정은 빼고 min_ok 장 이상 남아야 ok)
inline FaceLidar median_faces(const std::vector<FaceLidar> & fs, int min_ok = 4, double spread_xy = 0.04,
  double spread_yaw = 0.14, double front_offset = 0.1328)
{
  auto med = [](std::vector<double> v) {
      std::sort(v.begin(), v.end());
      const std::size_t n = v.size();
      return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
    };
  std::vector<FaceLidar> ok;
  for (const auto & f : fs) {if (f.ok) {ok.push_back(f);}}
  FaceLidar out;
  if (static_cast<int>(ok.size()) < min_ok) {
    out.why = "ok " + std::to_string(ok.size()) + " / " + std::to_string(fs.size()) + " 장";
    for (const auto & f : fs) {if (!f.ok) {out.why += " (" + f.why + ")"; break;}}
    return out;
  }
  std::vector<double> xs, ys, as;
  for (const auto & f : ok) {xs.push_back(f.cx); ys.push_back(f.cy); as.push_back(f.nyaw);}
  const double mx = med(xs), my = med(ys), ma = med(as);
  std::vector<double> kx, ky, ka;
  for (const auto & f : ok) {
    if (std::hypot(f.cx - mx, f.cy - my) <= spread_xy && std::fabs(bfl::wrap(f.nyaw - ma)) <= spread_yaw) {
      kx.push_back(f.cx); ky.push_back(f.cy); ka.push_back(f.nyaw);
    }
  }
  if (static_cast<int>(kx.size()) < min_ok) {out.why = "흩어짐 (" + std::to_string(kx.size()) + " 장만 모임)"; return out;}
  out.ok = true;
  out.cx = med(kx); out.cy = med(ky); out.nyaw = med(ka); out.n = kx.size();
  const double cn = std::cos(out.nyaw), sn = std::sin(out.nyaw);
  out.gap = cn * out.cx + sn * out.cy - front_offset;
  out.lateral = -(-sn * out.cx + cn * out.cy);
  out.yaw_err = bfl::wrap(out.nyaw);
  out.why = std::to_string(kx.size()) + " 장 중앙값";
  return out;
}

}  // namespace control
