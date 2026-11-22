// Spin 충돌검사 끄기 판정 (spin_guard.hpp, 11-22)
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "control/spin_guard.hpp"

using control::SpinGuardConfig;
using control::decide_spin_nocheck;

namespace
{
const double kDeg = M_PI / 180.0;
// 360° 스캔, 모든 빔 range r (한 방향만 가까운 점 override)
std::vector<float> ring(float r, int n = 360) {return std::vector<float>(n, r);}
}  // namespace

TEST(SpinGuard, ClearEnoughDisables)
{
  const SpinGuardConfig c;
  auto s = ring(2.0f);
  const auto r = decide_spin_nocheck(3 * kDeg, true, true, s, -M_PI, 2 * M_PI / 360, 0.12, c);
  EXPECT_TRUE(r.disable_checks);
  EXPECT_STREQ(r.reason, "ok");
  EXPECT_NEAR(r.sweep, 0.260 * 3 * kDeg, 1e-3);
}

TEST(SpinGuard, FrontTooCloseKeepsCheck)
{
  const SpinGuardConfig c;
  auto s = ring(2.0f);
  s[180] = 0.1328f + 0.03f;   // 각 0 (앞) 틈 3 cm < 쓸림 1.4 + 3 cm
  const auto r = decide_spin_nocheck(3 * kDeg, true, true, s, -M_PI, 2 * M_PI / 360, 0.12, c);
  EXPECT_FALSE(r.disable_checks);
  EXPECT_STREQ(r.reason, "clear_short");
}

TEST(SpinGuard, FrontGapEnoughDisables)
{
  const SpinGuardConfig c;
  auto s = ring(2.0f);
  s[180] = 0.1328f + 0.13f;   // 틈 13 cm (상자 앞 실제 상황)
  EXPECT_TRUE(decide_spin_nocheck(3 * kDeg, true, true, s, -M_PI, 2 * M_PI / 360, 0.12, c).disable_checks);
}

TEST(SpinGuard, RearCornerConsidered)
{
  const SpinGuardConfig c;
  auto s = ring(2.0f);
  s[0] = 0.2135f + 0.03f;     // 각 −180° (뒤) 틈 3 cm
  const auto r = decide_spin_nocheck(3 * kDeg, true, true, s, -M_PI, 2 * M_PI / 360, 0.12, c);
  EXPECT_FALSE(r.disable_checks);
  EXPECT_STREQ(r.reason, "clear_short");
}

TEST(SpinGuard, SideConsidered)
{
  const SpinGuardConfig c;
  auto s = ring(2.0f);
  s[270] = 0.1676f + 0.02f;   // 각 +90° (왼쪽) 틈 2 cm
  EXPECT_FALSE(decide_spin_nocheck(3 * kDeg, true, true, s, -M_PI, 2 * M_PI / 360, 0.12, c).disable_checks);
}

TEST(SpinGuard, NoScanOrStale)
{
  const SpinGuardConfig c;
  auto s = ring(2.0f);
  EXPECT_STREQ(decide_spin_nocheck(1 * kDeg, false, false, {}, -M_PI, 0.0175, 0.12, c).reason, "no_scan");
  const auto r = decide_spin_nocheck(1 * kDeg, true, false, s, -M_PI, 2 * M_PI / 360, 0.12, c);
  EXPECT_FALSE(r.disable_checks);
  EXPECT_STREQ(r.reason, "stale_scan");
}

TEST(SpinGuard, AngleOverLimit)
{
  const SpinGuardConfig c;
  auto s = ring(2.0f);
  const auto r = decide_spin_nocheck(3.5 * kDeg, true, true, s, -M_PI, 2 * M_PI / 360, 0.12, c);
  EXPECT_FALSE(r.disable_checks);
  EXPECT_STREQ(r.reason, "big_angle");
  EXPECT_TRUE(decide_spin_nocheck(-3.0 * kDeg, true, true, s, -M_PI, 2 * M_PI / 360, 0.12, c).disable_checks);
}

TEST(SpinGuard, DisabledByParam)
{
  SpinGuardConfig c;
  c.max_deg = 0.0;
  auto s = ring(2.0f);
  EXPECT_STREQ(decide_spin_nocheck(1 * kDeg, true, true, s, -M_PI, 2 * M_PI / 360, 0.12, c).reason, "angle_off");
}
