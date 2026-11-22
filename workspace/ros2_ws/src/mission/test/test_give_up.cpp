// 재시도 소진 처리 (give_up.hpp, R1) · 시작 전 준비 확인 표 (readiness.hpp, R3)
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "mission/give_up.hpp"
#include "mission/readiness.hpp"

using namespace mission;

namespace
{
// mission_lines.py 모양: 박스 하나 (퍽 둘, 하나는 회전) · 장소 하나 (놓기 둘) · verify · 복귀
std::vector<std::string> plan()
{
  return {
    "goto 0.5 1.2 0",                                                          // 0
    "approach 0.13",                                                           // 1
    "skill drive turn_p17 40 mode=OFF",                                        // 2
    "skill manipulator target_look:base_footprint:0.2:0.03:0.100 40 mode=SEARCH_FRONT+ARM_ALIGN+READ_TAG",  // 3
    "skill manipulator toppick_tilt 40 mode=SEARCH_FRONT+ARM_ALIGN+READ_TAG",  // 4
    "skill manipulator pallet_load_1_2_tilt 40 mode=OFF",                      // 5
    "skill drive turn_0 40 mode=OFF",                                          // 6
    "skill manipulator target_look:base_footprint:0.2:-0.03:0.100 40 mode=SEARCH_FRONT+ARM_ALIGN+READ_TAG",  // 7
    "skill manipulator toppick_tilt 40 mode=SEARCH_FRONT+ARM_ALIGN+READ_TAG",  // 8
    "skill manipulator pallet_load_3_0_tilt 40 mode=OFF",                      // 9
    "goto 1.5 0.5 1.57",                                                       // 10
    "approach_box 0.13",                                                       // 11
    "skill manipulator pallet_unload_1_2_tilt 40 mode=OFF",                    // 12
    "skill manipulator place_L1_m1_p1_tilt 40 mode=SEARCH_FRONT+ARM_ALIGN",    // 13
    "skill manipulator pallet_unload_3_0_tilt 40 mode=OFF",                    // 14
    "skill manipulator place_L1_0_0_tilt 40 mode=SEARCH_FRONT+ARM_ALIGN",      // 15
    "verify L1 1.5 0.7 0.07 100010001 0",                                      // 16
    "goto 0.69365 0.95 3.1416",                                                // 17
    "wait 3",                                                                  // 18
  };
}
}  // namespace

TEST(GiveUp, PalletSlot)
{
  EXPECT_EQ(pallet_slot("skill manipulator pallet_load_1_2_tilt 40 mode=OFF", "load"), "1_2");
  EXPECT_EQ(pallet_slot("skill manipulator pallet_unload_3_0_tilt 40", "unload"), "3_0");
  EXPECT_EQ(pallet_slot("skill manipulator pallet_unload_3_0_tilt 40", "load"), "");
  EXPECT_EQ(pallet_slot("skill manipulator toppick_tilt 40", "load"), "");
}

TEST(GiveUp, PickGroupSkipsToAfterLoad)
{
  const auto t = plan();
  for (size_t f : {2u, 3u, 4u}) {   // 회전 · 보기 · 집기 실패 -> pallet_load_1_2 다음 (turn_0)
    const auto sk = pick_skip(t, f);
    ASSERT_TRUE(sk.has_value()) << f;
    EXPECT_EQ(sk->slot, "1_2");
    EXPECT_EQ(sk->resume, 6u);
  }
  const auto sk2 = pick_skip(t, 8);   // 회전 없는 퍽 -> 다음 goto
  ASSERT_TRUE(sk2.has_value());
  EXPECT_EQ(sk2->slot, "3_0");
  EXPECT_EQ(sk2->resume, 10u);
}

TEST(GiveUp, NotPickGroup)
{
  const auto t = plan();
  for (size_t f : {0u, 1u, 5u, 6u, 10u, 12u, 13u, 16u, 17u}) {   // 주행 · 정렬 · pallet_load · turn_0 · 놓기 · verify · 복귀
    EXPECT_FALSE(pick_skip(t, f).has_value()) << f;
  }
}

TEST(GiveUp, PickWithoutLoadIsNotSkipped)
{
  // pallet_load 앞에 다른 단계가 끼면 (형식이 바뀜) 건너뛰지 않음 -> HOME 복귀로
  const std::vector<std::string> t = {
    "skill manipulator toppick_tilt 40", "goto 1 1 0", "skill manipulator pallet_load_1_2_tilt 40", "goto 0 0 0", "wait 3"};
  EXPECT_FALSE(pick_skip(t, 0).has_value());
}

TEST(GiveUp, EmptyHandLoad)
{
  // 빈손 pallet_load (운반 중 떨어뜨림, 2026-11-16): 다시 잴 곳 = 같은 묶음 target_look, 다 쓰면 그 칸만 건너뜀
  const auto t = plan();
  const std::string empty = "manipulator: pallet_load: empty hand: not holding a puck (dropped after pick, gripper closed fully)";
  const std::string jam = "manipulator: pallet_load: trajectory aborted";
  EXPECT_TRUE(is_empty_hand_load(t[5], empty));
  EXPECT_FALSE(is_empty_hand_load(t[5], jam));                     // 쥔 채 걸림 -> 예전처럼 HOME 복귀
  EXPECT_FALSE(is_empty_hand_load(t[4], empty));                   // pallet_load 가 아님
  EXPECT_FALSE(is_empty_hand_load(t[5], "pallet_load: no recent /joint_states for gripper"));

  EXPECT_EQ(retry_look(t, 5), std::optional<size_t>(3));           // 회전 있는 퍽
  EXPECT_EQ(retry_look(t, 9), std::optional<size_t>(7));           // 회전 없는 퍽
  EXPECT_EQ(retry_look(t, 4), std::optional<size_t>(3));           // toppick_tilt 도 같은 규칙
  EXPECT_FALSE(retry_look(t, 12).has_value());                     // 앞이 approach_box (주행 · 정렬) -> 없음
  EXPECT_FALSE(retry_look(t, 2).has_value());

  const auto sk = load_skip(t, 5, "pallet_load_1_2 failed after 2 retries: " + empty);
  ASSERT_TRUE(sk.has_value());
  EXPECT_EQ(sk->slot, "1_2");
  EXPECT_EQ(sk->resume, 6u);                                       // turn_0 (퍽 없이 정렬 방향으로)
  const auto sk2 = load_skip(t, 9, empty);
  ASSERT_TRUE(sk2.has_value());
  EXPECT_EQ(sk2->resume, 10u);
  EXPECT_FALSE(load_skip(t, 5, jam).has_value());
  EXPECT_FALSE(load_skip(t, 13, empty).has_value());               // place 는 대상 아님
}

TEST(GiveUp, AfterUnloadSkipsPlace)
{
  const auto t = plan();
  EXPECT_EQ(after_unload(t, 12), 14u);
  EXPECT_EQ(after_unload(t, 14), 16u);
  EXPECT_EQ(after_unload(t, 15), 16u);   // 뒤가 place 가 아니면 하나만
}

TEST(GiveUp, HomeIndex)
{
  EXPECT_EQ(home_index(plan()), std::optional<size_t>(17));
  EXPECT_FALSE(home_index({"observe", "plan"}).has_value());   // 계획 전
  EXPECT_FALSE(home_index({"goto 0 0 0", "verify L1 1 1 0.07 100010001 0"}).has_value());
  EXPECT_FALSE(home_index({"wait 3"}).has_value());
}

TEST(Readiness, FormatAndMissing)
{
  const std::vector<ReadyItem> items = {
    {"1 안전", "E-stop · 진단", ReadyState::OK, "정상"},
    {"2 컨트롤러", "diff_drive_controller", ReadyState::OK, "active"},
    {"2 컨트롤러", "arm_controller", ReadyState::FAIL, "inactive"},
    {"5 Nav2", "lifecycle_manager_navigation", ReadyState::WAIT, "응답 기다림"},
  };
  EXPECT_FALSE(all_ready(items));
  EXPECT_EQ(ready_missing(items), "arm_controller (inactive), lifecycle_manager_navigation (응답 기다림)");
  const std::string s = format_ready(items);
  EXPECT_NE(s.find("NOT READY 2/4"), std::string::npos);
  EXPECT_NE(s.find("[실패] arm_controller: inactive"), std::string::npos);
  EXPECT_NE(s.find("[대기] lifecycle_manager_navigation"), std::string::npos);
  EXPECT_EQ(s.find("\n  2 컨트롤러"), s.rfind("\n  2 컨트롤러"));   // 같은 단계 이름은 한 번

  const std::vector<ReadyItem> ok = {{"1 안전", "E-stop · 진단", ReadyState::OK, "정상"}};
  EXPECT_TRUE(all_ready(ok));
  EXPECT_NE(format_ready(ok).find("READY 1/1"), std::string::npos);
}

// ---- 이동 실패 시 그곳만 건너뛰기 (nav_skip, 11-21) ----
TEST(GiveUp, NavSkipBoxGoto)
{
  const auto t = plan();
  const auto nk = nav_skip(t, 0);   // 시작 박스 goto 실패 -> 두 퍽 집기 묶음 건너뜀, 장소 goto 부터
  ASSERT_TRUE(nk.has_value());
  EXPECT_EQ(nk->resume, 10u);
  EXPECT_EQ(nk->pick_slots, (std::vector<std::string>{"1_2", "3_0"}));
  EXPECT_TRUE(nk->place_slots.empty());
  EXPECT_TRUE(nk->verifies.empty());
}

TEST(GiveUp, NavSkipPlaceGotoAndApproach)
{
  const auto t = plan();
  for (size_t f : {10u, 11u}) {   // 장소 goto · approach_box 실패 -> 놓기 둘 · verify 건너뛰고 복귀 줄
    const auto nk = nav_skip(t, f);
    ASSERT_TRUE(nk.has_value()) << f;
    EXPECT_EQ(nk->resume, 17u);
    EXPECT_TRUE(nk->pick_slots.empty());
    EXPECT_EQ(nk->place_slots, (std::vector<std::string>{"1_2", "3_0"}));
    EXPECT_EQ(nk->verifies, (std::vector<std::string>{"L1"}));
  }
}

TEST(GiveUp, NavSkipApproachAndInsertedSteps)
{
  // 접근 · 자동으로 붙는 measure_start · center_box 도 그곳에 서는 단계 (손이 빔)
  const std::vector<std::string> t = {
    "goto 0.5 1.2 0", "measure_start B1:S", "approach 0.13", "center_box 0.13",
    "skill manipulator target_look:base_footprint:0.2:0.03:0.100 40", "skill manipulator toppick_tilt 40",
    "skill manipulator pallet_load_1_2_tilt 40", "goto 1.5 0.5 1.57", "approach_box 0.13", "measure_box L1",
    "skill manipulator pallet_unload_1_2_tilt 40", "skill manipulator place_L1_0_0_tilt 40", "goto 0.69365 0.95 3.1416", "wait 3"};
  for (size_t f : {1u, 2u, 3u}) {
    const auto nk = nav_skip(t, f);
    ASSERT_TRUE(nk.has_value()) << f;
    EXPECT_EQ(nk->resume, 7u);
    EXPECT_EQ(nk->pick_slots, (std::vector<std::string>{"1_2"}));
  }
  for (size_t f : {8u, 9u}) {
    const auto nk = nav_skip(t, f);
    ASSERT_TRUE(nk.has_value()) << f;
    EXPECT_EQ(nk->resume, 12u);
    EXPECT_EQ(nk->place_slots, (std::vector<std::string>{"1_2"}));
  }
}

TEST(GiveUp, NavSkipNotWhenHandMayHoldPuck)
{
  const auto t = plan();
  // 팔 · 집기 단계 실패는 nav_skip 대상 아님 (pick_skip · return_home 이 맡음)
  for (size_t f : {2u, 3u, 4u, 5u, 6u, 12u, 13u, 16u}) {EXPECT_FALSE(nav_skip(t, f).has_value()) << f;}
  // 복귀 줄 goto (뒤에 goto 없음) · 범위 밖
  EXPECT_FALSE(nav_skip(t, 17).has_value());
  EXPECT_FALSE(nav_skip(t, 18).has_value());
  EXPECT_FALSE(nav_skip(t, 99).has_value());
  // 같은 곳에서 퍽을 다룬 뒤의 접근 실패 (손에 퍽이 있을 수 있음) -> 건너뛰지 않음
  const std::vector<std::string> t2 = {
    "goto 1 1 0", "approach_box 0.13", "skill manipulator pallet_unload_1_2_tilt 40", "approach_box 0.13",
    "goto 2 2 0", "wait 3"};
  EXPECT_FALSE(nav_skip(t2, 3).has_value());
  EXPECT_TRUE(nav_skip(t2, 1).has_value());
  // goto 없이 시작하는 접근 (tag 모드 등)
  EXPECT_FALSE(nav_skip({"approach 0.13", "pick", "goto 1 1 0"}, 0).has_value());
}

TEST(GiveUp, NavSkipNavigateStep)
{
  const std::vector<std::string> t = {"navigate box_a", "approach 0.15", "pick", "navigate home", "wait 3"};
  const auto nk = nav_skip(t, 0);
  ASSERT_TRUE(nk.has_value());
  EXPECT_EQ(nk->resume, 3u);
  EXPECT_TRUE(is_nav_step("navigate home"));
  EXPECT_FALSE(is_nav_step("approach 0.13"));
  EXPECT_TRUE(is_positioning_step("center_box 0.13"));
  EXPECT_FALSE(is_positioning_step("check_marker L1"));
}

TEST(GiveUp, NavSkipAllowedStreak)
{
  // max 2: 첫 실패는 건너뜀, 연속 둘째는 복귀
  EXPECT_TRUE(nav_skip_allowed(1, 2));
  EXPECT_FALSE(nav_skip_allowed(2, 2));
  EXPECT_TRUE(nav_skip_allowed(2, 3));
  EXPECT_FALSE(nav_skip_allowed(1, 1));   // 1 이하 = 건너뛰지 않음
  EXPECT_FALSE(nav_skip_allowed(1, 0));
  EXPECT_FALSE(nav_skip_allowed(0, 2));
}
