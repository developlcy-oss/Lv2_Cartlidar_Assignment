// (11-21) /scan_front 상자 앞면 맞춤 시험 (box_face_lidar.hpp) — 합성 스캔 (roll π 라이다), 잡음 0 · σ 1 cm
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "control/box_face_lidar.hpp"

using namespace control;

namespace
{
constexpr double kDeg = M_PI / 180.0;

struct Scan { std::vector<float> ranges; double amin, ainc; };

// base_link 에서 상자 (중심 bx, by, 요 byaw, 변 0.205) 를 laser_front (0.1318, 0, rpy π,0,π) 로 본 스캔.
//   laser 각 θ → base 방향 (−cosθ, sinθ). 상자 네 변과의 가장 가까운 교점 + 잡음
Scan synth(double bx, double by, double byaw, double sigma, std::mt19937 & rng)
{
  std::normal_distribution<double> N(0.0, 1.0);
  Scan s;
  s.amin = -M_PI; s.ainc = 1.0 * kDeg;
  const double ox = 0.1318, oy = 0.0, h = 0.1025;
  const double c = std::cos(byaw), sn = std::sin(byaw);
  double cx[4], cy[4];
  const double lx[4] = {-h, h, h, -h}, ly[4] = {-h, -h, h, h};
  for (int i = 0; i < 4; ++i) {cx[i] = bx + c * lx[i] - sn * ly[i]; cy[i] = by + sn * lx[i] + c * ly[i];}
  for (int k = 0; k < 360; ++k) {
    const double th = s.amin + s.ainc * k;
    const double dx = -std::cos(th), dy = std::sin(th);
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 4; ++i) {
      const double ax = cx[i], ay = cy[i], ex = cx[(i + 1) % 4] - ax, ey = cy[(i + 1) % 4] - ay;
      const double den = dx * ey - dy * ex;
      if (std::fabs(den) < 1e-12) {continue;}
      const double t = ((ax - ox) * ey - (ay - oy) * ex) / den;
      const double u = ((ax - ox) * dy - (ay - oy) * dx) / den;
      if (t > 0 && u >= 0 && u <= 1) {best = std::min(best, t);}
    }
    s.ranges.push_back(std::isfinite(best) ? static_cast<float>(best + sigma * N(rng)) : std::numeric_limits<float>::infinity());
  }
  return s;
}

struct Truth { double gap, lateral, yaw; };
Truth truth(double bx, double by, double byaw)
{
  const double n = std::remainder(byaw, M_PI / 2);   // 로봇 쪽 면 법선 (yaw 0 근처)
  const double c = std::cos(n), s = std::sin(n);
  return {c * bx + s * by - 0.1025 - 0.1328, -(-s * bx + c * by), n};
}
}  // namespace

TEST(BoxFaceLidar, RollPiTransform)
{
  // laser 각 0 · 거리 0.3 → base x = 0.1318 − 0.3 (뒤), laser 각 +90° → base y +0.3 (y 부호 그대로)
  const auto p = lidar_scan_to_base({0.3f, 0.3f}, 0.0, M_PI / 2, 0.12, 8.0);
  ASSERT_EQ(p.size(), 2u);
  EXPECT_NEAR(p[0].x, 0.1318 - 0.3, 1e-6);
  EXPECT_NEAR(p[0].y, 0.0, 1e-6);
  EXPECT_NEAR(p[1].x, 0.1318, 1e-6);
  EXPECT_NEAR(p[1].y, 0.3, 1e-6);
  // range_min 0.12 아래 · inf 는 버림
  EXPECT_TRUE(lidar_scan_to_base({0.1f, std::numeric_limits<float>::infinity()}, 0.0, 0.1, 0.12, 8.0).empty());
}

TEST(BoxFaceLidar, NoiseFreeExact)
{
  std::mt19937 rng(1);
  for (double gap : {0.18, 0.25, 0.32}) {
    for (double by : {-0.04, 0.0, 0.03}) {
      for (double yaw : {-8 * kDeg, 0.0, 5 * kDeg}) {
        const double bx = 0.1328 + gap + 0.1025;
        const auto s = synth(bx, by, yaw, 0.0, rng);
        FaceLidarConfig c;
        c.angle_inc = s.ainc;
        const auto f = find_box_face_lidar(lidar_scan_to_base(s.ranges, s.amin, s.ainc, 0.12, 8.0), c);
        ASSERT_TRUE(f.ok) << f.why << " gap " << gap << " by " << by << " yaw " << yaw;
        const auto t = truth(bx, by, yaw);
        EXPECT_NEAR(f.gap, t.gap, 0.004);
        EXPECT_NEAR(f.yaw_err, t.yaw, 0.5 * kDeg);
        EXPECT_NEAR(f.lateral, t.lateral, 0.006);
        if (by > 0.02 && yaw == 0.0) {EXPECT_LT(f.lateral, 0.0);}   // 상자가 왼쪽 = 로봇이 중심선 오른쪽 (CenterError 부호)
      }
    }
  }
}

TEST(BoxFaceLidar, NoisyMedian)
{
  std::mt19937 rng(20261121);
  int ok = 0, n = 0;
  std::vector<double> el, ey, eg;
  for (int trial = 0; trial < 60; ++trial) {
    const double gap = 0.20 + 0.002 * trial, by = 0.04 * std::sin(trial), yaw = 8 * kDeg * std::cos(1.7 * trial);
    const double bx = 0.1328 + gap + 0.1025;
    std::vector<FaceLidar> fs;
    for (int k = 0; k < 5; ++k) {
      const auto s = synth(bx, by, yaw, 0.01, rng);
      FaceLidarConfig c;
      c.angle_inc = s.ainc;
      fs.push_back(find_box_face_lidar(lidar_scan_to_base(s.ranges, s.amin, s.ainc, 0.12, 8.0), c));
    }
    const auto m = median_faces(fs);
    ++n;
    if (!m.ok) {continue;}
    ++ok;
    const auto t = truth(bx, by, yaw);
    el.push_back(std::fabs(m.lateral - t.lateral));
    ey.push_back(std::fabs(m.yaw_err - t.yaw));
    eg.push_back(std::fabs(m.gap - t.gap));
  }
  auto p95 = [](std::vector<double> v) {std::sort(v.begin(), v.end()); return v[static_cast<size_t>(0.95 * (v.size() - 1))];};
  std::printf("라이다 σ 1 cm · 5 장: 성공 %d/%d, 옆 p95 %.1f mm · 요 p95 %.2f° · 틈 p95 %.1f mm\n",
    ok, n, p95(el) * 1000, p95(ey) / kDeg, p95(eg) * 1000);
  EXPECT_GE(ok, n * 9 / 10);
  EXPECT_LT(p95(el), 0.025);   // box_face.py 조정 결과 p95 8 ~ 18 mm 와 같은 대
  EXPECT_LT(p95(ey), 3.0 * kDeg);
  EXPECT_LT(p95(eg), 0.012);
}

TEST(BoxFaceLidar, TooFewPoints)
{
  const auto f = find_box_face_lidar({{0.3, 0.0}, {0.3, 0.01}});
  EXPECT_FALSE(f.ok);
  EXPECT_TRUE(std::isnan(f.gap));
  EXPECT_FALSE(median_faces({f, f, f, f, f}).ok);
}
