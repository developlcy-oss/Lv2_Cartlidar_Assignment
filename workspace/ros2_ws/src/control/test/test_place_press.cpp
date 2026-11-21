// PlacePress 시험: 누르는 중 닿음 판정 (관절 2 ~ 4 · 시작 때 값 빼기 · 연속 주기) · 내려간 거리 ·
//   빠지는 경유점 자르기 (닿은 높이 아래로 안 내려감) · 닿은 자리에서 다시 짜는 시간표
#include <gtest/gtest.h>

#include <vector>

#include "control/place_press.hpp"

using control::Arm5;
using control::PressContact;
using control::TimedWaypoint;

TEST(PlacePress, ContactNeedsConsecutiveDeviationFromStart)
{
  const Arm5 plan{0.1, -0.70, 1.20, 0.05, 0.0};
  Arm5 meas = plan;
  meas[2] -= 0.025;   // 시작 때부터 있던 0.025 rad 차이 (처짐 잔차 · 지연) = 닿음 아님
  PressContact c;
  EXPECT_FALSE(c.update(plan, meas, 0.03));   // 첫 주기 = 시작 값 기록
  EXPECT_FALSE(c.update(plan, meas, 0.03));
  EXPECT_NEAR(c.deviation(plan, meas), 0.0, 1e-12);
  // 관절 3 이 시작 때보다 0.031 더 뒤처짐: 3 주기 연속이어야 닿음
  Arm5 blocked = meas;
  blocked[2] -= 0.031;
  EXPECT_FALSE(c.update(plan, blocked, 0.03));
  EXPECT_FALSE(c.update(plan, blocked, 0.03));
  EXPECT_TRUE(c.update(plan, blocked, 0.03));
}

TEST(PlacePress, SingleSpikeResetsCount)
{
  const Arm5 plan{0.0, -0.70, 1.20, 0.05, 0.0};
  PressContact c;
  c.update(plan, plan, 0.03);
  Arm5 spike = plan;
  spike[1] += 0.05;
  EXPECT_FALSE(c.update(plan, spike, 0.03));
  EXPECT_FALSE(c.update(plan, spike, 0.03));
  EXPECT_FALSE(c.update(plan, plan, 0.03));   // 한 번 돌아오면 다시 0 부터
  EXPECT_FALSE(c.update(plan, spike, 0.03));
  EXPECT_FALSE(c.update(plan, spike, 0.03));
  EXPECT_TRUE(c.update(plan, spike, 0.03));
}

TEST(PlacePress, IgnoresJoint1And5)
{
  const Arm5 plan{0.0, -0.70, 1.20, 0.05, 0.0};
  PressContact c;
  c.update(plan, plan, 0.03);
  Arm5 m = plan;
  m[0] += 0.1;   // 관절 1 · 5 흔들림은 수직으로 내려가는 데 상관없음
  m[4] -= 0.1;
  for (int i = 0; i < 10; ++i) {EXPECT_FALSE(c.update(plan, m, 0.03));}
}

TEST(PlacePress, DepthIsProgressTimesHeight)
{
  EXPECT_NEAR(control::pressDepth(10.0, 10.0, 12.5, 0.025), 0.0, 1e-12);
  EXPECT_NEAR(control::pressDepth(11.0, 10.0, 12.5, 0.025), 0.010, 1e-12);
  EXPECT_NEAR(control::pressDepth(20.0, 10.0, 12.5, 0.025), 0.025, 1e-12);   // 끝 넘으면 전체 높이
  EXPECT_NEAR(control::pressDepth(9.0, 10.0, 12.5, 0.025), 0.0, 1e-12);
}

TEST(PlacePress, TrimRetreatDropsPointsBelowContact)
{
  // tip → above (25 mm, 3 등분) 수직 점 + 뒤로 빠짐 + 이동 자세
  const std::vector<TimedWaypoint> after{
    {Arm5{0, 1, 0, 0, 0}, 0.17, 0.00833}, {Arm5{0, 2, 0, 0, 0}, 0.17, 0.01667}, {Arm5{0, 3, 0, 0, 0}, 0.17, 0.025},
    {Arm5{0, 4, 0, 0, 0}, 0.2, -1.0}, {Arm5{0, 5, 0, 0, 0}, 1.0, -1.0}};
  // 15 mm 위에서 닿음 -> 8.3 mm 점 버림, 16.7 mm 부터 (아래로 안 내려감)
  auto t = control::trimRetreat(after, 0.015);
  ASSERT_EQ(t.size(), 4u);
  EXPECT_NEAR(t.front().rise, 0.01667, 1e-9);
  // above 에서 바로 닿음 (내려가기 전) -> 수직 점 모두 버림
  t = control::trimRetreat(after, 0.025);
  ASSERT_EQ(t.size(), 2u);
  EXPECT_LT(t.front().rise, 0.0);
  // tip 까지 내려가 닿음 -> 모두 씀
  EXPECT_EQ(control::trimRetreat(after, 0.0).size(), 5u);
  // 빈 경로
  EXPECT_TRUE(control::trimRetreat({}, 0.01).empty());
}

TEST(PlacePress, ReleaseFromContactOpensThenFollowsRetreat)
{
  const Arm5 contact{0.0, -0.70, 1.20, 0.05, 0.0};
  const std::vector<TimedWaypoint> after{
    {Arm5{0.0, -0.72, 1.18, 0.05, 0.0}, 0.2, 0.01},    // 수직으로 위 (작은 변화 -> min_dt 0.2)
    {Arm5{0.0, -0.90, -0.10, 1.79, 0.0}, 1.0, -1.0}};  // 이동 자세 (큰 변화 -> Δq 1.74 / 0.8 = 2.175 s)
  const auto tl = control::releaseFrom(contact, 0.923, 0.0798, 1.0, 0.3, after, 0.8);
  // 시작 = 닿은 자리 · 쥠, 벌림 1.0 s 동안 자리 그대로
  EXPECT_NEAR(tl.at(0.0).grip, 0.923, 1e-12);
  EXPECT_NEAR(tl.at(0.0).q[2], 1.20, 1e-12);
  EXPECT_NEAR(tl.at(1.0).grip, 0.0798, 1e-12);
  EXPECT_NEAR(tl.at(1.2).q[2], 1.20, 1e-12);   // 머묾 중에도 그 자리
  // 시간: 1.0 + 0.3 + max(0.2, 0.02 / 0.8) + max(1.0, 1.74 / 0.8)
  EXPECT_NEAR(tl.duration(), 1.0 + 0.3 + 0.2 + 1.74 / 0.8, 1e-9);
  EXPECT_NEAR(tl.at(tl.duration()).q[3], 1.79, 1e-12);
  EXPECT_NEAR(tl.at(tl.duration()).grip, 0.0798, 1e-12);
}

TEST(PlacePress, ReleaseFromEmptyRetreatAndZeroSpeed)
{
  const Arm5 contact{0.0, -0.70, 1.20, 0.05, 0.0};
  auto tl = control::releaseFrom(contact, 0.923, 0.0798, 1.0, 0.3, {}, 0.8);
  EXPECT_NEAR(tl.duration(), 1.3, 1e-12);   // 벌림 · 머묾만
  EXPECT_NEAR(tl.at(5.0).q[1], -0.70, 1e-12);
  // speed 0 = min_dt 만
  tl = control::releaseFrom(contact, 0.923, 0.0798, 1.0, 0.3, {{Arm5{0.5, -0.70, 1.20, 0.05, 0.0}, 0.4, -1.0}}, 0.0);
  EXPECT_NEAR(tl.duration(), 1.7, 1e-12);
}
