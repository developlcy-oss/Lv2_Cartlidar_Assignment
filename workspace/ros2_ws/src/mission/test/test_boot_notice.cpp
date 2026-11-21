// 기동 안내 판정 (boot_notice.hpp): E-stop 만 남음 · 기동 완료 · 반복 간격
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "mission/boot_notice.hpp"

using namespace mission;

namespace
{
const std::vector<std::string> kGated = {"E-stop · 진단", "팔 정리 (/manipulator/startup_done)"};

std::vector<ReadyItem> items(ReadyState safety, ReadyState arm, ReadyState nav)
{
  return {
    {"1 안전", "E-stop · 진단", safety, ""},
    {"2 컨트롤러", "diff_drive_controller", ReadyState::OK, "active"},
    {"3 노드 서버", "/drive/navigate", ReadyState::OK, "있음"},
    {"3 노드 서버", "팔 정리 (/manipulator/startup_done)", arm, ""},
    {"4 위치 추정", "/localization/status", ReadyState::OK, "OK"},
    {"5 Nav2", "lifecycle_manager_navigation", nav, ""},
  };
}
}  // namespace

TEST(BootNotice, OnlyEstopLeft)
{
  const auto F = ReadyState::FAIL, W = ReadyState::WAIT, O = ReadyState::OK;
  EXPECT_EQ(boot_phase(items(F, W, O), true, kGated), BootPhase::WAIT_ESTOP);
  EXPECT_EQ(boot_phase(items(F, O, O), true, kGated), BootPhase::WAIT_ESTOP);
  // Nav2 가 아직이면 안내 없음
  EXPECT_EQ(boot_phase(items(F, W, W), true, kGated), BootPhase::NOT_READY);
  // E-stop 은 풀렸는데 진단 오류 · 팔 정리 중이면 안내 없음
  EXPECT_EQ(boot_phase(items(F, O, O), false, kGated), BootPhase::NOT_READY);
  EXPECT_EQ(boot_phase(items(O, W, O), false, kGated), BootPhase::NOT_READY);
  EXPECT_EQ(boot_phase(items(O, O, O), false, kGated), BootPhase::ALL_READY);
}

TEST(BootNotice, Due)
{
  using P = BootPhase;
  EXPECT_FALSE(boot_notice_due(P::NOT_READY, P::NOT_READY, 100.0, 10.0));
  EXPECT_FALSE(boot_notice_due(P::WAIT_ESTOP, P::NOT_READY, 0.0, 10.0));
  EXPECT_TRUE(boot_notice_due(P::NOT_READY, P::WAIT_ESTOP, 0.0, 10.0));
  EXPECT_FALSE(boot_notice_due(P::WAIT_ESTOP, P::WAIT_ESTOP, 9.0, 10.0));   // 도배 금지
  EXPECT_TRUE(boot_notice_due(P::WAIT_ESTOP, P::WAIT_ESTOP, 10.0, 10.0));   // 10 s 재알림
  EXPECT_TRUE(boot_notice_due(P::WAIT_ESTOP, P::ALL_READY, 0.0, 10.0));
  EXPECT_FALSE(boot_notice_due(P::ALL_READY, P::ALL_READY, 100.0, 10.0));   // 기동 완료는 한 번
  EXPECT_NE(std::string(boot_notice_text(P::WAIT_ESTOP)).find("리모컨 2+3"), std::string::npos);
}
