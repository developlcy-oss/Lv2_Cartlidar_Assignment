// navigate 전 곧게 전진 시험 (2026-11-19, control/pullout.hpp)
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "control/align_controller.hpp"
#include "control/pullout.hpp"

using control::PulloutConfig;
using control::PulloutStep;
using control::pullout_front_blocked;
using control::pullout_step;
using control::pullout_wanted;
using control::rear_min_range;

namespace
{
// 360 칸 (1°), angle_min −π. deg 방향에 r 하나, 나머지 inf
std::vector<float> scan_with(double deg, float r)
{
  std::vector<float> v(360, std::numeric_limits<float>::infinity());
  const int i = static_cast<int>(std::lround(deg + 180.0)) % 360;
  v[i] = r;
  return v;
}
constexpr double kMin = -M_PI;
constexpr double kInc = M_PI / 180.0;
}  // namespace

TEST(Pullout, RearMinSeesOnlyRearSector)
{
  EXPECT_NEAR(rear_min_range(scan_with(180.0, 0.29f), kMin, kInc, 0.785), 0.29, 1e-6);    // 바로 뒤
  EXPECT_NEAR(rear_min_range(scan_with(-140.0, 0.30f), kMin, kInc, 0.785), 0.30, 1e-6);   // 뒤 오른쪽 40°
  EXPECT_TRUE(std::isinf(rear_min_range(scan_with(90.0, 0.20f), kMin, kInc, 0.785)));     // 옆 벽은 안 셈
  EXPECT_TRUE(std::isinf(rear_min_range(scan_with(0.0, 0.10f), kMin, kInc, 0.785)));      // 앞
}

TEST(Pullout, IgnoresInvalidRanges)
{
  auto v = scan_with(180.0, 0.005f);   // 0.01 아래 = 무효
  v[1] = std::nanf("");
  EXPECT_TRUE(std::isinf(rear_min_range(v, kMin, kInc, 0.785)));
}

TEST(Pullout, FrontBlocked)
{
  PulloutConfig c;   // tfmini 0.20
  EXPECT_FALSE(pullout_front_blocked(c, false, 0.50));
  EXPECT_TRUE(pullout_front_blocked(c, true, 0.50));          // 앞 /scan
  EXPECT_TRUE(pullout_front_blocked(c, false, 0.15));         // TFmini 가까움 (상자 · 퍽)
  EXPECT_TRUE(pullout_front_blocked(c, false, std::nullopt)); // 값 없음 = 0.10 m 안일 수 있음
}

TEST(Pullout, WantedOnlyWhenRearNearAndFrontFree)
{
  PulloutConfig c;   // rear_clear 0.33
  EXPECT_TRUE(pullout_wanted(c, 0.29, false));    // 집기 자리 (뒤 벽 0.29)
  EXPECT_FALSE(pullout_wanted(c, 0.40, false));   // 뒤 넉넉
  EXPECT_FALSE(pullout_wanted(c, 0.29, true));    // 앞 막힘 → 그대로 Nav2
  c.enabled = false;
  EXPECT_FALSE(pullout_wanted(c, 0.29, false));
}

TEST(Pullout, Steps)
{
  PulloutConfig c;   // max 0.06, timeout 3
  EXPECT_EQ(pullout_step(c, 0.29, 0.01, 0.2, false), PulloutStep::DRIVE);
  EXPECT_EQ(pullout_step(c, 0.33, 0.03, 0.8, false), PulloutStep::CLEAR);
  EXPECT_EQ(pullout_step(c, 0.30, 0.06, 1.5, false), PulloutStep::MAX_DISTANCE);
  EXPECT_EQ(pullout_step(c, 0.30, 0.02, 0.5, true), PulloutStep::FRONT_BLOCKED);
  EXPECT_EQ(pullout_step(c, 0.30, 0.02, 3.5, false), PulloutStep::TIMEOUT);   // 바퀴가 안 움직임
  EXPECT_EQ(pullout_step(c, 0.33, 0.06, 9.0, true), PulloutStep::CLEAR);      // 뒤 비었음이 먼저
}

TEST(Pullout, TfminiNearestForFrontCheck)
{
  control::BoxRangeFilter f;   // window 5, max_age 0.3
  EXPECT_FALSE(f.nearest(0.0).has_value());               // 측정 없음
  f.push(0.00, 0.46); f.push(0.05, 0.33); f.push(0.10, 0.44);
  EXPECT_FALSE(f.value(0.12).has_value());                // 퍼짐 13 cm → 정렬용 값은 없음 (Gazebo 집기 자리)
  ASSERT_TRUE(f.nearest(0.12).has_value());
  EXPECT_NEAR(*f.nearest(0.12), 0.33, 1e-9);              // 앞 막힘 판단은 가장 가까운 값
  f.push(0.15, std::nullopt);                             // 무효 (0.10 m 안일 수 있음) 가 섞이면 없음
  EXPECT_FALSE(f.nearest(0.16).has_value());
  EXPECT_FALSE(f.nearest(1.0).has_value());               // 모두 오래됨
}
