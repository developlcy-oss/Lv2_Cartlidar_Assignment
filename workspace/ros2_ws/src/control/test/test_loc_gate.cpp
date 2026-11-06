// LocGate 시험 (#121): 보내기 전 OK 기다림 · 진행 중 잃었다 돌아오면 다시 보냄 · 잠금 중 Nav2 실패 · 오래된 상태
#include <gtest/gtest.h>

#include "control/loc_gate.hpp"

using control::LocGate;
using control::LocGateParams;
using A = LocGate::Action;

TEST(LocGate, SendsImmediatelyWhenOk)
{
  LocGate g;
  g.status("OK", 0.0);
  EXPECT_TRUE(g.begin(0.1));
  EXPECT_EQ(g.tick(0.2, true), A::NONE);
}

TEST(LocGate, WaitsForOkBeforeSending)
{
  LocGate g;
  g.status("RECOVERING", 0.0);
  EXPECT_FALSE(g.begin(0.0));
  EXPECT_EQ(g.tick(0.5, false), A::NONE);
  g.status("SUSPECT", 1.0);   // 보내기 전에는 OK 만 (start_states)
  EXPECT_EQ(g.tick(1.0, false), A::NONE);
  g.status("OK", 2.0);
  EXPECT_EQ(g.tick(2.0, false), A::SEND);
  EXPECT_FALSE(g.waiting());
}

TEST(LocGate, FailsAfterWaitTimeout)
{
  LocGateParams p;
  p.wait_s = 5.0;
  LocGate g(p);
  EXPECT_FALSE(g.begin(0.0));   // 받은 적 없음 = 나쁨
  EXPECT_EQ(g.tick(4.9, false), A::NONE);
  EXPECT_EQ(g.tick(5.1, false), A::FAIL);
  EXPECT_EQ(g.tick(5.2, false), A::NONE);
}

TEST(LocGate, ResendsAfterRecovery)
{
  LocGate g;
  g.status("OK", 0.0);
  ASSERT_TRUE(g.begin(0.0));
  g.status("SUSPECT", 1.0);   // 진행 중 SUSPECT 는 괜찮음 (twist_mux 잠금 없음)
  EXPECT_EQ(g.tick(1.0, true), A::NONE);
  EXPECT_FALSE(g.badSeen());
  g.status("LOST", 2.0);
  EXPECT_EQ(g.tick(2.0, true), A::NONE);
  g.status("RECOVERING", 3.0);
  EXPECT_EQ(g.tick(3.0, true), A::NONE);
  g.status("SUSPECT", 4.0);   // 돌아올 때는 OK 까지 기다림
  EXPECT_EQ(g.tick(4.0, true), A::NONE);
  g.status("OK", 5.0);
  EXPECT_EQ(g.tick(5.0, true), A::RESEND);
  EXPECT_EQ(g.tick(5.1, true), A::NONE);   // 한 번만
}

TEST(LocGate, NavFailureWhileLostWaitsAndResends)
{
  LocGate g;
  g.status("OK", 0.0);
  ASSERT_TRUE(g.begin(0.0));
  g.status("LOST", 1.0);
  g.tick(1.0, true);
  EXPECT_TRUE(g.onNavFailed(1.5));   // 잠금 중 progress checker 가 끝냄 → 실패로 끝내지 않음
  EXPECT_TRUE(g.waiting());
  g.status("OK", 3.0);
  EXPECT_EQ(g.tick(3.0, false), A::SEND);
}

TEST(LocGate, NavFailureWithGoodLocalizationIsFailure)
{
  LocGate g;
  g.status("OK", 0.0);
  ASSERT_TRUE(g.begin(0.0));
  g.tick(0.5, true);
  EXPECT_FALSE(g.onNavFailed(1.0));
}

TEST(LocGate, StaleStatusIsBad)
{
  LocGate g;   // stale_s 1.0
  g.status("OK", 0.0);
  EXPECT_TRUE(g.startOk(0.9));
  EXPECT_FALSE(g.startOk(1.1));
  EXPECT_FALSE(g.begin(1.5));
  EXPECT_EQ(g.describe(1.5), "OK (오래됨)");
}

TEST(LocGate, DisabledPassesThrough)
{
  LocGateParams p;
  p.enabled = false;
  LocGate g(p);
  EXPECT_TRUE(g.begin(0.0));   // 상태 없이도
  EXPECT_EQ(g.tick(1.0, true), A::NONE);
  EXPECT_FALSE(g.onNavFailed(1.0));
}
