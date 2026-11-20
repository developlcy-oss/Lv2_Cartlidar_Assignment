// STOW 어긋남 안내 (stow_hint.hpp) 시험
#include <gtest/gtest.h>

#include <cmath>

#include "control/arm_ik.hpp"
#include "control/stow_hint.hpp"

using control::stowHintText;
using control::stowJointHints;

static const std::vector<double> kStow = {2.200, -2.088, 1.549, -1.868, 0.0, 0.923};
static double deg(double d) {return d * M_PI / 180.0;}

TEST(StowHint, TipMatchesArmIKForward)
{
  control::ArmIK ik{control::ArmIKParams{}};
  const auto p = ik.forward({kStow[0], kStow[1], kStow[2], kStow[3], kStow[4]});
  const auto t = control::stowTip({kStow[0], kStow[1], kStow[2], kStow[3], kStow[4]}, {});
  EXPECT_NEAR(t[0], p.x(), 1e-9);
  EXPECT_NEAR(t[1], p.y(), 1e-9);
  EXPECT_NEAR(t[2], p.z(), 1e-9);
  // 2026-11-20 STOW 손끝 (83, -114, 210) mm, joint1 축에서 141 mm
  EXPECT_NEAR(t[0], 0.0828, 1e-3);
  EXPECT_NEAR(t[1], -0.1137, 1e-3);
  EXPECT_NEAR(std::hypot(t[0], t[1]), 0.1406, 1e-3);
}

TEST(StowHint, J1CounterClockwiseMovesTipLeftAndForward)
{
  auto q = kStow;
  q[0] += deg(10.0);
  const auto h = stowJointHints(q, kStow);
  ASSERT_EQ(h.size(), 1u);
  EXPECT_EQ(h[0].joint, 1);
  EXPECT_NEAR(h[0].dq, deg(10.0), 1e-9);
  // 현 길이 = 2 R sin(5°)
  EXPECT_NEAR(h[0].dist, 2.0 * 0.1406 * std::sin(deg(5.0)), 1e-3);
  EXPECT_GT(h[0].move[1], 0.010);   // 왼쪽
  EXPECT_GT(h[0].move[0], 0.010);   // 앞
  EXPECT_NEAR(h[0].move[2], 0.0, 1e-9);
  const auto text = control::stowJointHintText(h[0]);
  EXPECT_NE(text.find("J1 몸통 회전 +10.0°"), std::string::npos) << text;
  EXPECT_NE(text.find("반시계"), std::string::npos) << text;
  EXPECT_NE(text.find("손끝 왼쪽 "), std::string::npos) << text;   // 좌우를 맨 앞에
}

TEST(StowHint, J1ClockwiseIsRight)
{
  auto q = kStow;
  q[0] -= deg(5.0);
  const auto h = stowJointHints(q, kStow);
  ASSERT_EQ(h.size(), 1u);
  EXPECT_LT(h[0].move[1], 0.0);
  EXPECT_NE(control::stowJointHintText(h[0]).find("손끝 오른쪽 "), std::string::npos);
}

TEST(StowHint, SortedByTipDistanceAndSmallJointsDropped)
{
  auto q = kStow;
  q[0] += deg(3.0);     // 손끝 ≈ 7 mm
  q[2] -= deg(20.0);    // 팔꿈치: 손끝 훨씬 많이
  q[4] += deg(30.0);    // 손목 회전: 손끝 0 mm → 맨 뒤
  q[1] += deg(0.2);     // 0.5° 아래 → 뺌
  q[5] = 0.0;           // 그리퍼는 안 봄
  const auto h = stowJointHints(q, kStow);
  ASSERT_EQ(h.size(), 3u);
  EXPECT_EQ(h[0].joint, 3);
  EXPECT_EQ(h[1].joint, 1);
  EXPECT_EQ(h[2].joint, 5);
  EXPECT_NEAR(h[2].dist, 0.0, 1e-9);
}

TEST(StowHint, WrapsAroundTwoPi)
{
  auto q = kStow;
  q[0] = kStow[0] + deg(5.0) - 2.0 * M_PI;
  const auto h = stowJointHints(q, kStow);
  ASSERT_EQ(h.size(), 1u);
  EXPECT_NEAR(h[0].dq, deg(5.0), 1e-9);
}

TEST(StowHint, TextMarksBeyondTolerance)
{
  auto q = kStow;
  q[0] += 0.80;
  q[3] -= 0.10;
  const auto s = stowHintText(q, kStow, 0.65);
  EXPECT_NE(s.find("STOW 어긋남"), std::string::npos);
  EXPECT_NE(s.find("J1 몸통 회전 +45.8°"), std::string::npos) << s;
  EXPECT_NE(s.find("허용 ±37.2°"), std::string::npos) << s;
  EXPECT_EQ(s.find("★ 넘음"), s.rfind("★ 넘음")) << s;   // J1 만
  EXPECT_LT(s.find("J1"), s.find("J4")) << s;
  EXPECT_NE(stowHintText(kStow, kStow, 0.65).find("어긋남 없음"), std::string::npos);
}
