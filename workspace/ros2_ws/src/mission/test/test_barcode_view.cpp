// 바코드 관찰 자리 · 빠져나오기 단계 (barcode_view.hpp)
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "mission/barcode_view.hpp"
#include "mission/give_up.hpp"

using namespace mission;

TEST(BarcodeView, EmptyViewPoseKeepsOldFlow)
{
  BarcodeView b;
  b.leave_back_m = 0.3;   // view_pose 가 비면 빠져나옴 설정이 있어도 무시
  EXPECT_TRUE(barcode_pre_lines(b).empty());
  EXPECT_TRUE(barcode_post_lines(b, "DRIVE").empty());
  EXPECT_EQ(barcode_view_error(b), "");
}

TEST(BarcodeView, ViewThenBackUp)
{
  BarcodeView b;
  b.view_pose = {0.20, 0.30, 90.0};
  b.leave_back_m = 0.25;
  const auto pre = barcode_pre_lines(b), post = barcode_post_lines(b, "DRIVE");
  ASSERT_EQ(pre.size(), 1u);
  const auto t = step_tokens(pre[0]);
  ASSERT_EQ(t.size(), 4u);
  EXPECT_EQ(t[0], "goto");
  EXPECT_NEAR(std::stod(t[3]), M_PI / 2.0, 1e-4);
  ASSERT_EQ(post.size(), 2u);   // 팔을 먼저 접고 후진
  EXPECT_EQ(post[0], "pose DRIVE");
  EXPECT_EQ(step_tokens(post[1])[0], "backup");
  EXPECT_NEAR(std::stod(step_tokens(post[1])[1]), 0.25, 1e-9);
}

TEST(BarcodeView, LeavePoseWinsOverBack)
{
  BarcodeView b;
  b.view_pose = {0.2, 0.3, 90.0};
  b.leave_pose = {0.4, 0.1, 0.0};
  b.leave_back_m = 0.25;
  const auto post = barcode_post_lines(b, "DRIVE");
  ASSERT_EQ(post.size(), 2u);
  EXPECT_EQ(post[0], "pose DRIVE");
  EXPECT_EQ(step_tokens(post[1])[0], "goto");
}

TEST(BarcodeView, NoLeaveConfigMeansNoLeaveStep)
{
  BarcodeView b;
  b.view_pose = {0.2, 0.3, 0.0};
  EXPECT_EQ(barcode_pre_lines(b).size(), 1u);
  EXPECT_TRUE(barcode_post_lines(b, "DRIVE").empty());
}

TEST(BarcodeView, Validation)
{
  BarcodeView b;
  b.view_pose = {0.2, 0.3};
  EXPECT_NE(barcode_view_error(b), "");
  b.view_pose = {0.2, 0.3, 0.0};
  b.leave_back_m = 2.0;
  EXPECT_NE(barcode_view_error(b), "");
  b.leave_back_m = 0.3;
  b.backup_speed = 0.0;
  EXPECT_NE(barcode_view_error(b), "");
}

// 새 기본값 (mission_plan.yaml): view [0.575, 0.95, 180], back 0.12 / 0.10
TEST(BarcodeView, DefaultPlanOrder)
{
  BarcodeView b;
  b.view_pose = {0.575, 0.95, 180.0};
  b.leave_back_m = 0.12;
  b.backup_speed = 0.10;
  ASSERT_EQ(barcode_view_error(b), "");
  std::vector<std::string> steps = {"unfold DRIVE"};
  for (const auto & l : barcode_pre_lines(b)) {steps.push_back(l);}
  steps.push_back("observe");
  for (const auto & l : barcode_post_lines(b, "DRIVE")) {steps.push_back(l);}
  steps.push_back("survey_start");
  ASSERT_EQ(steps.size(), 6u);
  EXPECT_EQ(steps[1], "goto 0.575 0.95 3.14159");
  EXPECT_EQ(steps[2], "observe");
  EXPECT_EQ(steps[3], "pose DRIVE");
  EXPECT_EQ(steps[4], "backup 0.12 0.1");
}

// 노드의 goto · backup 검사가 요구하는 모양: goto = 숫자 3 개, backup = 거리 (0, 1] · 속도 (0, 0.3]
static bool is_num(const std::string & s, double & v)
{
  try {size_t n = 0; v = std::stod(s, &n); return n == s.size() && std::isfinite(v);} catch (...) {return false;}
}

TEST(BarcodeView, LinesPassNodeShapeChecks)
{
  BarcodeView b;
  b.view_pose = {0.575, 0.95, 180.0};
  b.leave_pose = {0.6936, 0.95, 180.0};
  double v;
  for (const auto & l : {barcode_pre_lines(b)[0], barcode_post_lines(b, "DRIVE")[1]}) {
    const auto t = step_tokens(l);
    ASSERT_EQ(t.size(), 4u);
    EXPECT_EQ(t[0], "goto");
    for (int i = 1; i < 4; ++i) {EXPECT_TRUE(is_num(t[i], v)) << l;}
    EXPECT_NEAR(std::stod(t[3]), M_PI, 1e-4);
  }
  b.leave_pose.clear();
  b.leave_back_m = 0.12;
  const auto t = step_tokens(barcode_post_lines(b, "DRIVE")[1]);
  ASSERT_EQ(t.size(), 3u);
  double d, sp;
  ASSERT_TRUE(is_num(t[1], d) && is_num(t[2], sp));
  EXPECT_TRUE(d > 0.0 && d <= 1.0 && sp > 0.0 && sp <= 0.3);
}

TEST(BarcodeView, LeaveFailureIsNonFatalOnlyForLeaveSteps)
{
  const std::vector<std::string> back = {"unfold DRIVE", "goto 0.575 0.95 3.14159", "observe", "pose DRIVE",
    "backup 0.12 0.1", "survey_start"};
  EXPECT_FALSE(is_leave_step(back, 0));
  EXPECT_FALSE(is_leave_step(back, 1));   // 관찰 자리 goto: 치명
  EXPECT_FALSE(is_leave_step(back, 2));
  EXPECT_FALSE(is_leave_step(back, 3));   // 팔 접기: 치명
  EXPECT_TRUE(leave_failure_nonfatal(back, 4));
  EXPECT_FALSE(leave_failure_nonfatal(back, 5));
  const std::vector<std::string> go = {"unfold DRIVE", "goto 0.575 0.95 3.14159", "observe", "pose DRIVE",
    "goto 0.6936 0.95 3.14159", "survey_start"};
  EXPECT_FALSE(leave_failure_nonfatal(go, 1));
  EXPECT_TRUE(leave_failure_nonfatal(go, 4));
  const std::vector<std::string> last = {"observe", "pose DRIVE", "backup 0.12 0.1"};
  EXPECT_FALSE(leave_failure_nonfatal(last, 2));   // 뒤 단계 없음
  EXPECT_FALSE(leave_failure_nonfatal(last, 9));
}
