// 복귀 판정 (return_check.hpp): 차체 네 모서리가 출발 구역 안인지 · 가장자리까지 거리
#include <gtest/gtest.h>

#include <cmath>

#include "mission/return_check.hpp"

using mission::judge_return;
using mission::ReturnZone;

TEST(ReturnCheck, StartPoseIsCentered)
{
  // 출발 자세 (localization 기본): 차체 외곽 가운데 = 구역 가운데, 앞이 −X → x 여유 0.3 − 0.17315, y 여유 0.3 − 0.1676
  const auto r = judge_return(ReturnZone{}, 0.69365, 0.95, M_PI);
  EXPECT_TRUE(r.inside);
  EXPECT_NEAR(r.min_margin, 0.3 - 0.17315, 1e-6);
}

TEST(ReturnCheck, ShiftedOut)
{
  const auto r = judge_return(ReturnZone{}, 0.69365 + 0.15, 0.95, M_PI);   // 여유 0.127 보다 더 밀림
  EXPECT_FALSE(r.inside);
  EXPECT_NEAR(r.min_margin, 0.3 - 0.17315 - 0.15, 1e-6);
}

TEST(ReturnCheck, RotationEatsMargin)
{
  const ReturnZone z;
  const double straight = judge_return(z, 0.69365, 0.95, M_PI).min_margin;
  const double turned = judge_return(z, 0.69365, 0.95, M_PI + 0.1).min_margin;   // Nav2 yaw 허용 오차 약 6°
  EXPECT_LT(turned, straight);
  EXPECT_GT(turned, 0.08);   // 6° 로는 아직 안 (모서리가 약 2 ~ 4 cm 더 나감)
}

TEST(ReturnCheck, Turned90UsesOtherSide)
{
  // 90° 돌면 차체 길이 (0.3463) 가 y 로 감: 외곽 가운데를 구역 가운데에 두면 y 여유 0.3 − 0.17315
  const ReturnZone z;
  const double xc = (z.body_x_min + z.body_x_max) / 2;   // base_link 기준 외곽 가운데 x
  const auto r = judge_return(z, z.cx, z.cy - xc, M_PI / 2);
  EXPECT_TRUE(r.inside);
  EXPECT_NEAR(r.min_margin, 0.3 - 0.17315, 1e-6);
}
