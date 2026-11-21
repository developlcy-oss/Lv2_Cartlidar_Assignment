// 상자 중앙 맞춤 단계 (center_box.hpp): 어느 approach 뒤에 붙이는지 · status 판정 · 단계 시간
#include <gtest/gtest.h>

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
  // 기본 timeout.center_box 90 s = 팔 2 × 10 + settle 0.5 + 재기 12 + drive 45 + 여유 12.5
  EXPECT_DOUBLE_EQ(center_box_timeout(10.0, 0.5, 12.0, 45.0, 12.5), 90.0);
}
