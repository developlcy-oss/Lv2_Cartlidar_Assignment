// minJerkSamples (min_jerk.hpp): 끝점 · 단조 · 속도 상한 · 끄기
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "control/min_jerk.hpp"

using control::minJerkSamples;

TEST(MinJerk, EndsAtTargetWithZeroVelocity)
{
  const std::vector<double> from{0.0, 1.0}, to{1.0, -1.0};
  const auto p = minJerkSamples(from, to, 2.0, 0.05);
  ASSERT_EQ(p.size(), 40u);
  EXPECT_DOUBLE_EQ(p.back().t, 2.0);
  for (size_t i = 0; i < to.size(); ++i) {
    EXPECT_DOUBLE_EQ(p.back().q[i], to[i]);
    EXPECT_DOUBLE_EQ(p.back().v[i], 0.0);
    EXPECT_DOUBLE_EQ(p.back().a[i], 0.0);
  }
}

TEST(MinJerk, MonotonicAndPeakVelocity)
{
  const double d = 3.0, T = 10.0;
  const auto p = minJerkSamples({0.0}, {d}, T, 0.01);
  double prev = 0.0, vmax = 0.0;
  for (const auto & s : p) {
    EXPECT_GE(s.q[0], prev - 1e-12);
    prev = s.q[0];
    vmax = std::max(vmax, s.v[0]);
  }
  EXPECT_NEAR(vmax, 1.875 * d / T, 1e-3);   // 가운데 (τ = 0.5) 가 가장 빠름
  EXPECT_NEAR(p[0].a[0], 60.0 * 0.001 * d / (T * T), 1e-3);   // 출발 직후 가속도 ≈ 0 (3 차는 6 d / T² 로 바로 뜀)
}

TEST(MinJerk, MidpointIsHalfway)
{
  const auto p = minJerkSamples({1.0}, {2.0}, 1.0, 0.1);
  EXPECT_NEAR(p[4].t, 0.5, 1e-12);
  EXPECT_NEAR(p[4].q[0], 1.5, 1e-12);
}

TEST(MinJerk, DisabledOrBadInputGivesSinglePoint)
{
  EXPECT_EQ(minJerkSamples({0.0}, {1.0}, 2.0, 0.0).size(), 1u);
  EXPECT_EQ(minJerkSamples({0.0}, {1.0}, 0.0, 0.05).size(), 1u);
  EXPECT_EQ(minJerkSamples({0.0, 0.0}, {1.0}, 2.0, 0.05).size(), 1u);
  const auto p = minJerkSamples({0.0}, {1.0}, 1.5, 0.0);
  EXPECT_DOUBLE_EQ(p[0].t, 1.5);
  EXPECT_DOUBLE_EQ(p[0].q[0], 1.0);
}
