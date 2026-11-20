// 상자 중앙 맞춤 단계 (approach.center.*, 2026-11-20) — ROS 의존 없음 → 단위 시험 (test_center_box.cpp)
//   approach (COARSE → 팔 ALIGN → FINE) 뒤에 자동으로 center_box <거리> 를 붙임 (그 접근에서 퍽을 집을 때만):
//   ① 팔 place_box.arm_pose (BOX_VIEW, 상자 윗면 위) → ② approach.center.mode → settle → /perception/place_box/measure (팔 Arducam)
//   ③ 팔 align.arm_pose (ALIGN) 로 되돌림 → ④ kFineMode 켜고 /drive/align goal.center (잰 상자 중심 · 요) → drive 가 옆 이동 · 법선 · 퍽 마무리
//   측정 실패면 ③ 뒤 바로 끝 (FINE 결과 그대로, 실패 아님). drive 실패 (옮긴 뒤 FINE 도 못 맞춤) 는 단계 실패 → 앞 approach 부터 다시
#pragma once

#include <string>
#include <vector>

#include "mission/give_up.hpp"

namespace mission
{

// 접근 묶음을 끝내는 단계 (다음 자리로 감 · 다른 접근)
inline bool ends_approach_group(const std::string & type)
{
  return type == "goto" || type == "navigate" || type == "approach" || type == "approach_box" || type == "approach_view" ||
         type == "search" || type == "survey_view" || type == "verify" || type == "wait";
}

// texts[idx] 가 approach 이고, 그 접근 (다음 goto · navigate · 다른 접근 전) 에서 퍽을 집으면 true
//   (pick · pick_target · manipulator skill). 바로 뒤가 이미 center_box 면 false (재시도 때 두 번 붙지 않음)
inline bool approach_needs_center(const std::vector<std::string> & texts, size_t idx)
{
  if (idx >= texts.size()) {return false;}
  const auto t0 = step_tokens(texts[idx]);
  if (t0.empty() || t0[0] != "approach") {return false;}
  for (size_t i = idx + 1; i < texts.size(); ++i) {
    const auto t = step_tokens(texts[i]);
    if (t.empty()) {continue;}
    if (t[0] == "center_box") {return false;}
    if (ends_approach_group(t[0])) {return false;}
    if (t[0] == "pick" || t[0] == "pick_target") {return true;}
    if (t[0] == "skill" && t.size() >= 2 && t[1] == "manipulator") {return true;}
  }
  return false;
}

// place_box status (JSON 문자열) 가 성공인지. 비면 아직 없음
inline bool place_box_status_ok(const std::string & status)
{
  return status.find("\"ok\": true") != std::string::npos;
}

// center_box 단계 시간 상한 [s] = 팔 자세 2 번 + 모드 · settle + 측정 + drive 중앙 맞춤 (align.center.timeout_s) + 여유
inline double center_box_timeout(double pose_s, double settle_s, double measure_s, double drive_center_s, double margin_s)
{
  return 2.0 * pose_s + settle_s + measure_s + drive_center_s + margin_s;
}

}  // namespace mission
