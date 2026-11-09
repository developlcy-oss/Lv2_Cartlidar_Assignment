// BoxPark 시험 (#157): 박스 앞 정차 → 물러남 확인 뒤 해제 · 다시 보내기 · navigate 전 후진 판단
#include <gtest/gtest.h>

#include "control/box_park.hpp"

using control::BackoffStep;
using control::BoxPark;
using control::BoxParkConfig;
using control::backoff_step;

TEST(BoxPark, ParksWhenAlignEndsNearBox)
{
  BoxPark p;
  EXPECT_TRUE(p.on_align_end(0.0, 0.10, 0.13));   // FINE 자리 0.13 < 0.35 → 정차
  EXPECT_TRUE(p.parked());
  EXPECT_DOUBLE_EQ(p.stop_m(), 0.10);
}

TEST(BoxPark, NotParkedWhenFarFromBox)
{
  BoxPark p;
  EXPECT_FALSE(p.on_align_end(0.0, 0.10, 0.40));  // 실패로 끝났지만 이미 멀리 → 바로 되돌림
  EXPECT_FALSE(p.parked());
}

TEST(BoxPark, InvalidRangeAtEndCountsAsNear)
{
  BoxPark p;
  EXPECT_TRUE(p.on_align_end(0.0, 0.10, std::nullopt));   // 0.10 m 안 (무효) 일 수 있음
}

TEST(BoxPark, ReleasesAfterConsecutiveFarReadings)
{
  BoxPark p;
  p.on_align_end(0.0, 0.10, 0.13);
  EXPECT_FALSE(p.on_range(0.20));
  EXPECT_FALSE(p.on_range(0.36));
  EXPECT_FALSE(p.on_range(0.36));
  EXPECT_TRUE(p.on_range(0.36));                  // 3 번째 연속 → 해제
  EXPECT_FALSE(p.parked());
  EXPECT_FALSE(p.on_range(0.36));                 // 해제 뒤에는 다시 true 를 내지 않음
}

TEST(BoxPark, NearOrInvalidReadingResetsCount)
{
  BoxPark p;
  p.on_align_end(0.0, 0.10, 0.13);
  p.on_range(0.36);
  p.on_range(0.36);
  EXPECT_FALSE(p.on_range(std::nullopt));         // 무효 → 처음부터
  p.on_range(0.36);
  p.on_range(0.36);
  EXPECT_FALSE(p.on_range(0.30));                 // 가까움 → 처음부터
  EXPECT_TRUE(p.parked());
}

TEST(BoxPark, RefreshEveryPeriodWhileParked)
{
  BoxPark p;   // refresh_s 20
  EXPECT_FALSE(p.refresh_due(25.0));              // 정차 전
  p.on_align_end(100.0, 0.10, 0.13);
  EXPECT_FALSE(p.refresh_due(119.0));
  EXPECT_TRUE(p.refresh_due(120.0));
  EXPECT_FALSE(p.refresh_due(139.0));
  EXPECT_TRUE(p.refresh_due(140.0));
  p.clear();
  EXPECT_FALSE(p.refresh_due(200.0));
}

TEST(BoxPark, BackoffSteps)
{
  BoxParkConfig c;   // max 0.30 m, timeout 8 s
  EXPECT_EQ(backoff_step(c, true, 0.05, 1.0, false), BackoffStep::DRIVE);
  EXPECT_EQ(backoff_step(c, true, 0.05, 1.0, true), BackoffStep::HOLD);
  EXPECT_EQ(backoff_step(c, false, 0.22, 3.0, false), BackoffStep::RELEASED);
  EXPECT_EQ(backoff_step(c, true, 0.30, 4.0, false), BackoffStep::MAX_DISTANCE);
  EXPECT_EQ(backoff_step(c, true, 0.10, 8.5, true), BackoffStep::TIMEOUT);
  EXPECT_EQ(backoff_step(c, false, 0.30, 9.0, true), BackoffStep::RELEASED);   // 해제가 먼저
}
