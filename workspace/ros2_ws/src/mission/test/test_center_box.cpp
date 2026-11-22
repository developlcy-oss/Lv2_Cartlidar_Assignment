// 상자 중앙 맞춤 단계 (center_box.hpp): 어느 approach 뒤에 붙이는지 · status 판정 · 단계 시간
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "mission/center_box.hpp"

using namespace mission;

TEST(CenterBox, StartBoxPickGroupGetsCenter)
{
  const std::vector<std::string> t = {
    "goto 1.0 2.0 0.0", "approach 0.13", "skill drive turn_p17", "skill manipulator target_look:base_footprint:0.3:0.05:0.1",
    "skill manipulator toppick_tilt", "goto 2.0 2.0 0.0"};
  EXPECT_TRUE(approach_needs_center(t, 1));
  EXPECT_FALSE(approach_needs_center(t, 0));   // approach 가 아님
  EXPECT_FALSE(approach_needs_center(t, 2));
}

TEST(CenterBox, PickAndPickTarget)
{
  EXPECT_TRUE(approach_needs_center({"approach 0.15", "pick"}, 0));
  EXPECT_TRUE(approach_needs_center({"approach 0.15", "pick_target 1 2 0.1"}, 0));
}

TEST(CenterBox, AlreadyInsertedNotTwice)
{
  EXPECT_FALSE(approach_needs_center({"approach 0.13", "center_box 0.13", "pick"}, 0));
}

TEST(CenterBox, SurveyAndVerifyApproachesSkipped)
{
  // 퍽 측량 (조사 자세 + approach + survey_view) · verify 앞 접근은 집지 않음
  EXPECT_FALSE(approach_needs_center({"goto 1 1 0", "approach 0.13", "survey_view 2 SURVEY", "goto 2 2 0", "approach 0.13",
      "skill manipulator target_look:a"}, 1));
  EXPECT_FALSE(approach_needs_center({"approach 0.13", "verify L1 1 2 0.07 000000000 0"}, 0));
  EXPECT_FALSE(approach_needs_center({"approach 0.13", "navigate HOME", "pick"}, 0));
  EXPECT_FALSE(approach_needs_center({"approach 0.13", "approach_box 0.13", "pick"}, 0));
}

TEST(CenterBox, DriveSkillOnlyNotEnough)
{
  // drive turn 만 있고 팔 집기가 없으면 붙이지 않음
  EXPECT_FALSE(approach_needs_center({"approach 0.13", "skill drive turn_0", "goto 1 1 0"}, 0));
  EXPECT_FALSE(approach_needs_center({"approach 0.13"}, 0));
  EXPECT_FALSE(approach_needs_center({"approach 0.13"}, 5));
}

TEST(CenterBox, StatusOk)
{
  EXPECT_TRUE(place_box_status_ok("{\"ok\": true, \"frames\": 3}"));
  EXPECT_FALSE(place_box_status_ok("{\"ok\": false, \"why\": \"상자 못 찾음\"}"));
  EXPECT_FALSE(place_box_status_ok(""));
}

TEST(CenterBox, TimeoutBudget)
{
  // 반복 0 = 예전 식 (팔 2 × 10 + settle 0.5 + 재기 12 + 여유) — drive 는 반복 항으로만
  EXPECT_DOUBLE_EQ(center_box_timeout(10.0, 0.5, 12.0, 45.0, 0, 12.5), 45.0);
  // 11-21 기본: 팔 2 × 10 + (0.8 + 12) + 3 × (45 + 0.8 + 12) + 12.5 = 218.7
  EXPECT_NEAR(center_box_timeout(10.0, 0.8, 12.0, 45.0, 3, 12.5), 218.7, 1e-9);
  // 반복이 하나 늘 때마다 drive + settle + 재기 만큼
  EXPECT_NEAR(center_box_timeout(10.0, 0.8, 12.0, 45.0, 2, 12.5) + 57.8, center_box_timeout(10.0, 0.8, 12.0, 45.0, 3, 12.5), 1e-9);
  EXPECT_DOUBLE_EQ(center_box_timeout(10.0, 0.8, 12.0, 45.0, -1, 12.5), center_box_timeout(10.0, 0.8, 12.0, 45.0, 0, 12.5));
}

TEST(CenterBox, CenterErrorStraight)
{
  // 상자가 정면 0.285 m, 오른쪽 2 cm (by = -0.02) → 로봇이 중심선 왼쪽 + 2 cm
  const auto e = center_error(0.285, -0.02, 0.0);
  EXPECT_NEAR(e.lateral, 0.02, 1e-12);
  EXPECT_NEAR(e.yaw, 0.0, 1e-12);
  EXPECT_NEAR(e.depth, 0.285, 1e-12);
  EXPECT_NEAR(center_error(0.285, 0.013, 0.0).lateral, -0.013, 1e-12);
}

TEST(CenterBox, CenterErrorYawFoldsTo45)
{
  // 상자 요는 90° 마다 같음 → 법선은 로봇 yaw 에 가장 가까운 것
  for (double k : {-2.0, -1.0, 0.0, 1.0, 2.0}) {
    const double yaw = 0.05 + k * M_PI / 2.0;
    EXPECT_NEAR(center_error(0.3, 0.0, yaw).yaw, 0.05, 1e-9) << k;
  }
  EXPECT_NEAR(std::fabs(center_error(0.3, 0.0, M_PI / 4.0 - 1e-6).yaw), M_PI / 4.0, 1e-5);
}

TEST(CenterBox, CenterErrorRotatedLine)
{
  // 법선이 로봇 yaw 에서 psi 돈 상자: 로봇이 중심선 위에 있으면 옆 0
  const double psi = 0.1, d = 0.3;
  const auto on = center_error(d * std::cos(psi), d * std::sin(psi), psi);
  EXPECT_NEAR(on.lateral, 0.0, 1e-12);
  EXPECT_NEAR(on.depth, d, 1e-12);
  // 같은 상자를 중심선 직각으로 왼쪽 (로봇 기준 상자가 오른쪽) 1 cm 옮기면 로봇은 중심선 왼쪽 + 1 cm
  const double nx = std::sin(psi), ny = -std::cos(psi);   // 중심선의 오른쪽 직각
  const auto off = center_error(d * std::cos(psi) + 0.01 * nx, d * std::sin(psi) + 0.01 * ny, psi);
  EXPECT_NEAR(off.lateral, 0.01, 1e-12);
  // 상자가 뒤면 depth < 0 (mission 은 이 측정을 버림)
  EXPECT_LT(center_error(-0.3, 0.0, 0.0).depth, 0.0);
}

TEST(CenterBox, LidarDisagree)
{
  EXPECT_FALSE(lidar_disagrees(std::nan(""), 0.01, 0.02));   // 라이다 못 잼 (틈 < 0.15)
  EXPECT_FALSE(lidar_disagrees(0.015, 0.0, 0.02));
  EXPECT_TRUE(lidar_disagrees(0.025, 0.0, 0.02));
  EXPECT_TRUE(lidar_disagrees(-0.012, 0.012, 0.02));
  EXPECT_FALSE(lidar_disagrees(0.5, 0.0, 0.0));               // 끔
  EXPECT_FALSE(lidar_disagrees(0.0, std::nan(""), 0.02));
}
