// 집게 쥠 판정 (grasp_detector.hpp, manipulator_node gripper.*, #201)
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "control/grasp_detector.hpp"

using control::GraspHysteresis;
using control::gripInGraspRange;

namespace
{
// manipulator.yaml 기본: 열림 −0.764 · 닫힘 0.923 · grasp_margin 0.15 · grasp_min_position 0.30
constexpr double kLo = 0.30;
constexpr double kHi = 0.923 - 0.15;
}  // namespace

TEST(GraspDetector, RangeSeparatesPuckEmptyAndBlocked)
{
  EXPECT_TRUE(gripInGraspRange(0.505, kLo, kHi));     // 퍽 30 mm 에서 막힘 (10-27 벤치 2377 tick)
  EXPECT_TRUE(gripInGraspRange(0.556, kLo, kHi));     // GripGuard 가 막힌 자리 + 33 tick 으로 쥠
  EXPECT_FALSE(gripInGraspRange(0.923, kLo, kHi));    // 끝까지 닫힘 = 빈손
  EXPECT_FALSE(gripInGraspRange(-0.764, kLo, kHi));   // 열린 채 (안 닫힘) — 예전 판정은 이것도 쥠으로 봤음
  EXPECT_FALSE(gripInGraspRange(0.10, kLo, kHi));     // 퍽보다 큰 물체에 막힘
  EXPECT_FALSE(gripInGraspRange(std::numeric_limits<double>::quiet_NaN(), kLo, kHi));
}

// 30 Hz 에서 켜짐 8 스텝 · 꺼짐 3 스텝 (모의 정책과 같은 꼴)
TEST(GraspDetector, HysteresisOnEightOffThreeStepsAt30Hz)
{
  GraspHysteresis h(0.26, 0.10);
  const double dt = 1.0 / 30.0;
  for (int k = 1; k <= 7; ++k) {EXPECT_FALSE(h.update(true, dt)) << k;}
  EXPECT_TRUE(h.update(true, dt));                     // 8 번째
  EXPECT_TRUE(h.update(false, dt));                    // 1 번 밖 → 아직 쥠
  EXPECT_TRUE(h.update(false, dt));                    // 2 번
  EXPECT_FALSE(h.update(false, dt));                   // 3 번째 꺼짐
}

TEST(GraspDetector, ShortGlitchDoesNotFlip)
{
  GraspHysteresis h(0.26, 0.10);
  const double dt = 1.0 / 30.0;
  for (int k = 0; k < 8; ++k) {h.update(true, dt);}
  ASSERT_TRUE(h.grasped());
  EXPECT_TRUE(h.update(false, dt));                    // 한두 번 튀어도
  EXPECT_TRUE(h.update(true, dt));                     // 다시 범위 안이면 시간 초기화
  EXPECT_TRUE(h.update(false, dt));
  EXPECT_TRUE(h.update(false, dt));
  EXPECT_TRUE(h.grasped());
  h.reset();
  EXPECT_FALSE(h.grasped());
}
