// 켜질 때 정리 자세 계산 (startup_stow.hpp, manipulator_node startup.*)
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "control/startup_stow.hpp"

using control::parseStartupStages;
using control::startupStageDuration;
using control::startupStageNeeded;
using control::startupStageTarget;

namespace
{
const std::vector<double> kLower{-3.14159, -2.14, -1.6, -1.92, -2.8, -0.7946};
const std::vector<double> kUpper{3.14159, 1.6, 1.6, 2.0, 2.8, 0.9541};
const std::vector<double> kStow{-1.5953, -2.0816, 1.5478, -1.8607, -0.0138, 0.923};
}  // namespace

TEST(StartupStow, ParsesDefaultStages)
{
  const auto s = parseStartupStages({"2,3,4,5,6", "1"}, 6);
  ASSERT_EQ(s.size(), 2u);
  EXPECT_EQ(s[0], (std::vector<size_t>{1, 2, 3, 4, 5}));
  EXPECT_EQ(s[1], (std::vector<size_t>{0}));
  EXPECT_EQ(parseStartupStages({" 1 , 2"}, 6)[0], (std::vector<size_t>{0, 1}));
  // manipulator.yaml 기본 = 대기 → 주행 실물 순서 J3 → J2 → J1 → J4 → (J5 · 그리퍼) (#144 · #165)
  const auto d = parseStartupStages({"3", "2", "1", "4", "5,6"}, 6);
  ASSERT_EQ(d.size(), 5u);
  EXPECT_EQ(d[0], (std::vector<size_t>{2}));
  EXPECT_EQ(d[2], (std::vector<size_t>{0}));
  EXPECT_EQ(d[4], (std::vector<size_t>{4, 5}));
}

TEST(StartupStow, RejectsBadStages)
{
  EXPECT_THROW(parseStartupStages({"0"}, 6), std::invalid_argument);       // 1 부터
  EXPECT_THROW(parseStartupStages({"7"}, 6), std::invalid_argument);       // 관절 수 넘음
  EXPECT_THROW(parseStartupStages({"1,2", "2"}, 6), std::invalid_argument);   // 묶음 사이 중복
  EXPECT_THROW(parseStartupStages({""}, 6), std::invalid_argument);        // 빈 묶음
  EXPECT_THROW(parseStartupStages({"1,x"}, 6), std::invalid_argument);
  EXPECT_THROW(parseStartupStages({"2a"}, 6), std::invalid_argument);
}

// 묶음 관절만 정리 자세로, 나머지는 지금 자리 — 한계 밖에서 시작했으면 한계 안으로 자른 값
TEST(StartupStow, StageTargetMovesOnlyStageJointsAndClampsOthers)
{
  // 2026-11-08 실물: joint4 2.23 rad (한계 2.0 밖) 에서 시작
  const std::vector<double> current{0.1, -0.5, -1.618, 2.23, 0.0, 0.5};
  const auto first = startupStageTarget(current, kStow, {1, 2, 3, 4, 5}, kLower, kUpper);
  EXPECT_DOUBLE_EQ(first[0], 0.1);   // 허리는 그대로 (첫 묶음 밖)
  for (size_t k = 1; k < 6; ++k) {EXPECT_DOUBLE_EQ(first[k], kStow[k]) << k;}

  const auto waist_only = startupStageTarget(current, kStow, {0}, kLower, kUpper);
  EXPECT_DOUBLE_EQ(waist_only[0], kStow[0]);
  EXPECT_DOUBLE_EQ(waist_only[2], -1.6);   // 한계 밖 → 한계로
  EXPECT_DOUBLE_EQ(waist_only[3], 2.0);
  EXPECT_DOUBLE_EQ(waist_only[1], -0.5);
}

TEST(StartupStow, SkipsStageAlreadyAtPose)
{
  auto at = kStow;
  at[0] = 0.5;   // 허리만 다름
  EXPECT_FALSE(startupStageNeeded(at, kStow, {1, 2, 3, 4, 5}, 0.05));
  EXPECT_TRUE(startupStageNeeded(at, kStow, {0}, 0.05));
  at[3] += 0.04;   // 허용 안
  EXPECT_FALSE(startupStageNeeded(at, kStow, {1, 2, 3, 4, 5}, 0.05));
  at[3] += 0.02;   // 허용 밖
  EXPECT_TRUE(startupStageNeeded(at, kStow, {1, 2, 3, 4, 5}, 0.05));
}

TEST(StartupStow, DurationFromLargestMoveIncludingGripper)
{
  const std::vector<double> from{0, 0, 0, 0, 0, -0.7};
  std::vector<double> to = from;
  to[0] = 1.0;
  EXPECT_DOUBLE_EQ(startupStageDuration(from, to, 0.2, 1.0), 5.0);   // 1.0 rad / 0.2
  to[0] = 0.0;
  to[5] = 0.9;
  EXPECT_DOUBLE_EQ(startupStageDuration(from, to, 0.2, 1.0), 8.0);   // 그리퍼 1.6 rad / 0.2
  EXPECT_DOUBLE_EQ(startupStageDuration(from, from, 0.2, 1.0), 1.0);   // 최소 시간
}
