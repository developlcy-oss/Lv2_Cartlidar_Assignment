// ARM_ALIGN 목표 퍽 유지 시험 (arm_target_lock.hpp). 정규화 좌표, 빨강 0 · 초록 1
#include <gtest/gtest.h>

#include "perception/arm_target_lock.hpp"

using namespace perception;

namespace
{
constexpr int RED = 0, GREEN = 1;
ArmBox box(double x, int cls, double score, double w = 0.10) {return {x, 0.0, w, cls, score};}
}  // namespace

// 퍽 A(왼쪽) · B(오른쪽, 0.3 옆). 처음에 A 점수가 높아 A 를 잡고, 뒤에 B 점수가 높아져도 A 를 유지
TEST(ArmLock, KeepsTargetWhenScoresSwap)
{
  ArmTargetLock lock;
  EXPECT_EQ(lock.update({box(-0.15, RED, 0.9), box(0.15, RED, 0.6)}, std::nullopt), 0);
  EXPECT_EQ(lock.update({box(-0.12, RED, 0.5), box(0.17, RED, 0.95)}, std::nullopt), 0);
  EXPECT_EQ(lock.update({box(0.18, RED, 0.95), box(-0.10, RED, 0.4)}, std::nullopt), 1);   // 목록 순서가 바뀌어도 A
}

// A 가 안 보이면 -1 (B 로 넘어가지 않음), 직전 자리 근처에 다시 보이면 이어 간다
TEST(ArmLock, LostTargetDoesNotSwitchAndReacquires)
{
  ArmTargetLock lock;
  ASSERT_EQ(lock.update({box(-0.15, RED, 0.9), box(0.15, RED, 0.6)}, std::nullopt), 0);
  EXPECT_EQ(lock.update({box(0.15, RED, 0.9)}, std::nullopt), -1);
  EXPECT_EQ(lock.update({}, std::nullopt), -1);
  EXPECT_EQ(lock.update({box(0.15, RED, 0.9), box(-0.10, RED, 0.5)}, std::nullopt), 1);
  EXPECT_TRUE(lock.locked());
  lock.reset();   // ARM_ALIGN 끄고 다시 켬 → 새로 고름
  EXPECT_EQ(lock.update({box(0.15, RED, 0.9)}, std::nullopt), 0);
}

// 처음 고를 때 D435 목표 투영점(B 자리)에 가까운 박스. 점수가 높은 A 가 아님. 투영점 근처에 없으면 잠그지 않음
TEST(ArmLock, SeedPicksProjectedTarget)
{
  ArmTargetLock lock;
  EXPECT_EQ(lock.update({box(-0.15, RED, 0.9), box(0.15, RED, 0.6)}, ArmPoint{0.13, 0.02}), 1);
  ArmTargetLock lock2;
  EXPECT_EQ(lock2.update({box(-0.15, RED, 0.9)}, ArmPoint{0.40, 0.0}), -1);
  EXPECT_FALSE(lock2.locked());
}

// 바로 옆이라도 다른 색은 잡지 않는다
TEST(ArmLock, DifferentClassNotTaken)
{
  ArmTargetLock lock;
  ASSERT_EQ(lock.update({box(0.0, RED, 0.9)}, std::nullopt), 0);
  EXPECT_EQ(lock.update({box(0.02, GREEN, 0.9)}, std::nullopt), -1);
}

// 폭이 2 배 넘게 달라지면 다른 물체로 본다 (가까워지며 조금씩 커지는 것은 따라감)
TEST(ArmLock, SizeJumpNotTaken)
{
  ArmTargetLock lock;
  ASSERT_EQ(lock.update({box(0.0, RED, 0.9, 0.10)}, std::nullopt), 0);
  EXPECT_EQ(lock.update({box(0.01, RED, 0.9, 0.13)}, std::nullopt), 0);
  EXPECT_EQ(lock.update({box(0.02, RED, 0.9, 0.30)}, std::nullopt), -1);
}

// 투영점에서 seed_gate(0.10) 밖 박스는 처음에 잠그지 않는다 (목표가 안 잡힌 장에 0.2 옆 퍽을 잠그지 않게)
TEST(ArmLock, SeedGateRejectsNeighbor)
{
  ArmTargetLock lock;
  EXPECT_EQ(lock.update({box(0.20, RED, 0.9)}, ArmPoint{0.0, 0.0}), -1);
  EXPECT_FALSE(lock.locked());
  EXPECT_EQ(lock.update({box(0.20, RED, 0.9), box(0.03, RED, 0.5)}, ArmPoint{0.0, 0.0}), 1);
}

// 목표가 화면 밖으로 나가는 동안 같은 색 이웃이 그 자리로 들어와도, 투영점(화면 밖)에서 멀어 잡지 않는다.
// 목표가 돌아와 투영점 근처에 보이면 다시 잡는다
TEST(ArmLock, NeighborInTargetSpotNotTakenWhileOut)
{
  ArmTargetLock lock;
  ASSERT_EQ(lock.update({box(0.80, RED, 0.9, 0.12), box(0.62, RED, 0.9, 0.12)}, ArmPoint{0.80, 0.0}), 0);
  EXPECT_EQ(lock.update({box(0.62, RED, 0.9, 0.12)}, ArmPoint{1.10, 0.0}), -1);                        // 목표 화면 밖
  EXPECT_EQ(lock.update({box(0.80, RED, 0.9, 0.12)}, ArmPoint{1.20, 0.0}), -1);                        // 이웃이 목표 자리로
  EXPECT_EQ(lock.update({box(0.70, RED, 0.9, 0.12), box(0.88, RED, 0.9, 0.10)}, ArmPoint{0.90, 0.0}), 1);   // 목표가 돌아옴
}

// 투영점이 없으면(D435 목표 없음) 놓친 뒤 직전 자리 근처(0.6 × 박스 폭)에서 다시 잡는다
TEST(ArmLock, ReacquireNearLastWithoutSeed)
{
  ArmTargetLock lock;
  ASSERT_EQ(lock.update({box(0.0, RED, 0.9, 0.12)}, std::nullopt), 0);
  EXPECT_EQ(lock.update({}, std::nullopt), -1);
  EXPECT_EQ(lock.update({box(0.20, RED, 0.9, 0.12)}, std::nullopt), -1);   // 0.20 > 0.6 × 0.24
  EXPECT_EQ(lock.update({box(0.10, RED, 0.9, 0.12)}, std::nullopt), 0);
}

// 한 장에 박스 폭(x 단위 0.24)에 가깝게 움직이는 빠른 이동도 직전 이동으로 예측해 따라간다
TEST(ArmLock, PredictionFollowsFastMove)
{
  ArmTargetLock lock;
  ASSERT_EQ(lock.update({box(0.0, GREEN, 0.9, 0.12)}, std::nullopt), 0);
  EXPECT_EQ(lock.update({box(0.10, GREEN, 0.9, 0.12)}, std::nullopt), 0);   // 0.10 ≤ 0.144
  EXPECT_EQ(lock.update({box(0.30, GREEN, 0.9, 0.12)}, std::nullopt), 0);   // 직전에서 0.20 이지만 예측 0.20 에서 0.10
  EXPECT_EQ(lock.update({box(0.50, GREEN, 0.9, 0.12)}, std::nullopt), 0);   // 예측 0.50
  ArmTargetLock still;
  ASSERT_EQ(still.update({box(0.0, GREEN, 0.9, 0.12)}, std::nullopt), 0);
  EXPECT_EQ(still.update({box(0.20, GREEN, 0.9, 0.12)}, std::nullopt), -1);   // 예측 없이 0.20 은 못 따라감
}

// 세로 거리는 화면 비율(높이 / 폭 0.75)을 곱해 잰다: y 0.18 = x 단위 0.135 < 0.144
TEST(ArmLock, VerticalDistanceUsesAspect)
{
  ArmTargetLock lock;
  ASSERT_EQ(lock.update({box(0.0, RED, 0.9, 0.12)}, std::nullopt), 0);
  EXPECT_EQ(lock.update({{0.0, 0.18, 0.12, RED, 0.9}}, std::nullopt), 0);
}

// 목표 라벨이 화면 끝에 잘려 이웃과 한 덩어리(폭 1.5 배)가 되면 따라가지 않고 놓친다
TEST(ArmLock, MergedBoxNotFollowed)
{
  ArmTargetLock lock;
  ASSERT_EQ(lock.update({box(0.0, GREEN, 0.9, 0.134)}, std::nullopt), 0);
  EXPECT_EQ(lock.update({box(-0.03, GREEN, 0.9, 0.203)}, std::nullopt), -1);
  EXPECT_EQ(lock.update({box(0.0, GREEN, 0.9, 0.14)}, std::nullopt), 0);   // 다시 하나로 보이면 이어 감
}

TEST(ArmLock, ReacquireUsesUnclippedWidth)
{
  ArmTargetLock lock;
  auto step = [&](double x,double w) {return lock.update({{x,0,w,0,.9,.15}},ArmPoint{x,0});};
  ASSERT_EQ(step(.78,.12),0);
  ASSERT_EQ(step(.90,.10),0);
  ASSERT_EQ(step(.92,.08),0);
  ASSERT_EQ(step(.935,.065),0);
  ASSERT_EQ(step(.95,.05),0);
  ASSERT_EQ(lock.update({},ArmPoint{1.2,0}),-1);
  EXPECT_EQ(step(.2,.12),0);
}

TEST(ArmLock, ReferenceWidthKeepsClassAndSizeGuards)
{
  ArmTargetLock lock;
  ASSERT_EQ(lock.update({{0,0,.12,0,.9,.15}},ArmPoint{0,0}),0);
  ASSERT_EQ(lock.update({},ArmPoint{1.2,0}),-1);
  EXPECT_EQ(lock.update({{0,0,.12,1,.9,.15}},ArmPoint{0,0}),-1);
  EXPECT_EQ(lock.update({{0,0,.7,0,.9,.15}},ArmPoint{0,0}),-1);
}

// FOV 안에서도 집게 가림은 상자를 축소한다. 놓친 뒤 작은 상자가 다시 잡혀도
// 연속 추적으로 확인한 폭을 버리지 않아 온전한 복귀 목표를 거부하지 않는다.
TEST(ArmLock, OccludedReacquireDoesNotShrinkReference)
{
  ArmTargetLock lock;
  EXPECT_EQ(lock.update({{0,0,.11,0,.9,.15}}, ArmPoint{0,0}),0);
  EXPECT_EQ(lock.update({}, ArmPoint{0,0}),-1);
  EXPECT_EQ(lock.update({{0,0,.06,0,.5,.08}}, ArmPoint{0,0}),0);
  EXPECT_EQ(lock.update({}, ArmPoint{0,0}),-1);
  EXPECT_EQ(lock.update({{0,0,.19,0,.9,.22}}, ArmPoint{0,0}),0);
}

TEST(ArmLock, CameraMotionUsesProjectedSeedDisplacement)
{
  ArmTargetLock lock;
  ASSERT_EQ(lock.update({box(0,RED,.9)},ArmPoint{.02,0}),0);
  // 카메라 회전으로 목표가 0.6 이동. 예전 자리의 점수 높은 이웃을 고르면 안 된다.
  EXPECT_EQ(lock.update({box(0,RED,.99),box(.6,RED,.7)},ArmPoint{.62,0}),1);
  EXPECT_EQ(lock.update({box(0,RED,.99)},ArmPoint{1.2,0}),-1);
}
TEST(ArmLock, CameraMotionStillRejectsWrongClassAndSize)
{
  ArmTargetLock lock;
  ASSERT_EQ(lock.update({box(0,RED,.9)},ArmPoint{0,0}),0);
  EXPECT_EQ(lock.update({box(.6,GREEN,.9),box(.6,RED,.9,.4)},ArmPoint{.6,0}),-1);
}

TEST(ArmLock, DistinctSeedDistancesChooseNearest)
{
  ArmTargetLock lock;
  EXPECT_EQ(lock.update({box(.01,RED,.6),box(.08,RED,.99)},ArmPoint{0,0}),0);
  EXPECT_EQ(lock.update({},ArmPoint{0,0}),-1);
  EXPECT_EQ(lock.update({box(.08,RED,.99),box(.01,RED,.6)},ArmPoint{0,0}),1);
}
TEST(ArmLock, EqualSeedDistancesStayUncertain)
{
  ArmTargetLock lock;
  EXPECT_EQ(lock.update({box(-.01,RED,.6),box(.01,RED,.99)},ArmPoint{0,0}),-1);
  EXPECT_FALSE(lock.locked());
}

TEST(ArmLock, RotatingBoxUsesStableDetectionSize)
{
  ArmTargetLock lock;
  ASSERT_EQ(lock.update({{0,0,.035,0,.9,.10,.06}},ArmPoint{0,0,.4}),0);
  EXPECT_EQ(lock.update({{.14,0,.05,0,.9,.08,.062}},ArmPoint{.14,0,.4}),0);
}
TEST(ArmLock, DepthPredictsSizeAndReacquireReference)
{
  ArmTargetLock lock;
  ASSERT_EQ(lock.update({{0,0,.12,0,.9,.12,.12}},ArmPoint{0,0,.2}),0);
  EXPECT_EQ(lock.update({{0,0,.06,0,.9,.06,.06}},ArmPoint{0,0,.4}),0);
  EXPECT_EQ(lock.update({},ArmPoint{0,0,.4}),-1);
  EXPECT_EQ(lock.update({{0,0,.03,0,.9,.03,.03}},ArmPoint{0,0,.8}),0);
  EXPECT_EQ(lock.update({{0,0,.09,0,.9,.09,.09}},ArmPoint{0,0,.8}),-1);
}
