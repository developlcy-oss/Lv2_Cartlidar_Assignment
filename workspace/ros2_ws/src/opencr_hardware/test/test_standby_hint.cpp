// 팔 대기 자세 어긋남 안내 (standby_hint.hpp) 시험 — 범위 = opencr.ros2_control.xacro arm_standby_min/max_tick
#include <gtest/gtest.h>

#include <cmath>

#include "opencr_hardware/standby_hint.hpp"

using opencr_hardware::arm_standby_hint;

static const std::vector<long> kLo = {3425, 653, 3025, 797, 1849};
static const std::vector<long> kHi = {3539, 721, 3091, 864, 2246};
static std::vector<long> centre() {return {3482, 687, 3058, 830, 2047};}

TEST(StandbyHint, InsideRangeIsEmpty)
{
  EXPECT_EQ(arm_standby_hint(centre(), kLo, kHi), "");
  EXPECT_EQ(arm_standby_hint({1, 2}, kLo, kHi), "");
}

TEST(StandbyHint, J1PlusIsCounterClockwiseLeft)
{
  auto t = centre();
  t[0] += 400;   // +35.2° (2026-11-18 실물 J1 -289 tick 처럼 범위 밖)
  const auto s = arm_standby_hint(t, kLo, kHi);
  EXPECT_NE(s.find("J1 몸통 회전 +35.2° (위에서 볼 때 반시계) → 손끝 왼쪽 "), std::string::npos) << s;
  t[0] = 3482 - 289;
  const auto r = arm_standby_hint(t, kLo, kHi);
  EXPECT_NE(r.find("J1 몸통 회전 -25.4° (위에서 볼 때 시계) → 손끝 오른쪽 "), std::string::npos) << r;
}

TEST(StandbyHint, MatchesTipArcAndSortsByDistance)
{
  // 손끝 수평 반지름 141 mm (STOW): J1 10° → 현 2 × 141 × sin 5° = 24.6 mm
  const std::array<double, 5> stow{2.200, -2.088, 1.549, -1.868, 0.0};
  const auto tip = opencr_hardware::standby_tip(stow);
  EXPECT_NEAR(std::hypot(tip[0], tip[1]), 0.1406, 1e-3);
  auto t = centre();
  t[4] = 2600;   // 손목 회전 (손끝 0 mm) → 맨 뒤
  t[2] += 300;   // 팔꿈치
  t[0] += 120;   // 몸통
  const auto s = arm_standby_hint(t, kLo, kHi);
  const auto j1 = s.find("J1"), j3 = s.find("J3"), j5 = s.find("J5");
  ASSERT_NE(j1, std::string::npos);
  ASSERT_NE(j3, std::string::npos);
  ASSERT_NE(j5, std::string::npos);
  EXPECT_LT(j3, j1) << s;
  EXPECT_LT(j1, j5) << s;
  EXPECT_NE(s.find("집게만 비틀림"), std::string::npos) << s;
}
