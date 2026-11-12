// 비스듬히 서기 계산 (oblique.hpp): 요 접기 · 회전각 · 보기 거리 · TFmini 목표
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "mission/oblique.hpp"

using mission::ObliqueGeom;

namespace
{
constexpr double kDeg = M_PI / 180.0;

// 축 (x 축, 원점 = 렌즈) 을 따라 쏜 빔이 중심 (r, 0) · 요 beta 인 정사각 (반 변 h) 에 처음 닿는 거리 (슬랩 방법)
double ray_square(double r, double beta, double h)
{
  // 상자 좌표로: 빔 원점 (-r, 0) 을 -beta 돌림, 방향 (1, 0) 도 -beta 돌림
  const double c = std::cos(-beta), s = std::sin(-beta);
  const double ox = c * -r, oy = s * -r, dx = c, dy = s;
  double t0 = -std::numeric_limits<double>::infinity(), t1 = std::numeric_limits<double>::infinity();
  for (const auto & [o, d] : {std::pair{ox, dx}, std::pair{oy, dy}}) {
    if (std::abs(d) < 1e-12) {continue;}
    double a = (-h - o) / d, b = (h - o) / d;
    if (a > b) {std::swap(a, b);}
    t0 = std::max(t0, a);
    t1 = std::min(t1, b);
  }
  return t0 <= t1 ? t0 : -1.0;
}
}  // namespace

TEST(Oblique, Fold45)
{
  EXPECT_NEAR(mission::oblique_fold45(20 * kDeg), 20 * kDeg, 1e-12);
  EXPECT_NEAR(mission::oblique_fold45(50 * kDeg), -40 * kDeg, 1e-12);
  EXPECT_NEAR(mission::oblique_fold45(-130 * kDeg), -40 * kDeg, 1e-12);
  EXPECT_NEAR(mission::oblique_fold45(90 * kDeg), 0.0, 1e-12);
  EXPECT_NEAR(std::abs(mission::oblique_fold45(45 * kDeg)), 45 * kDeg, 1e-12);
}

TEST(Oblique, TurnPutsCenterOnAxis)
{
  const ObliqueGeom g;
  EXPECT_NEAR(mission::oblique_turn(g, 0.6853, 0.0), 0.0, 1e-12);
  // 회전 중심 둘레로 phi 돌면 중심이 새 x 축 위 range_after_turn 에 옴
  for (const auto & [cx, cy] : {std::pair{0.6853, 0.03}, std::pair{0.70, -0.05}, std::pair{0.62, 0.08}}) {
    const double phi = mission::oblique_turn(g, cx, cy);
    const double rx = cx - g.icr_x, ry = cy;
    const double nx = std::cos(-phi) * rx - std::sin(-phi) * ry + g.icr_x;
    const double ny = std::sin(-phi) * rx + std::cos(-phi) * ry;
    EXPECT_NEAR(ny, 0.0, 1e-12);
    EXPECT_NEAR(nx, mission::oblique_range_after_turn(g, cx, cy), 1e-12);
  }
}

TEST(Oblique, LookRange)
{
  const ObliqueGeom g;
  EXPECT_NEAR(mission::oblique_look_range(g, 0.35, 0.0), 0.1328 + 0.35 + 0.1025, 1e-12);
  EXPECT_NEAR(mission::oblique_look_range(g, 0.35, 45 * kDeg), 0.1328 + 0.35 + 0.1025 * std::sqrt(2.0), 1e-12);
  // 요 부호와 상관없음
  EXPECT_NEAR(mission::oblique_look_range(g, 0.35, -30 * kDeg), mission::oblique_look_range(g, 0.35, 30 * kDeg), 1e-12);
}

TEST(Oblique, TfTargetMatchesRayCast)
{
  const ObliqueGeom g;
  EXPECT_NEAR(mission::oblique_tf_target(g, mission::oblique_look_range(g, 0.35, 0.0), 0.0), 0.35 + 0.1328 - 0.13, 1e-12);
  for (double deg : {0.0, 10.0, 20.0, -30.0, 40.0, 44.0}) {
    const double b = deg * kDeg;
    const double r = mission::oblique_look_range(g, 0.35, b);
    // 렌즈는 base x = tf_x: 렌즈에서 중심까지 r - tf_x
    EXPECT_NEAR(mission::oblique_tf_target(g, r, b), ray_square(r - g.tf_x, b, g.half), 1e-9) << deg;
  }
}
