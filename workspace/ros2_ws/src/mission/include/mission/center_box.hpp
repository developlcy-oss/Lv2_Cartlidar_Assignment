// 상자 중앙 맞춤 단계 (approach.center.*, 2026-11-20 · 11-21 반복 폐루프) — ROS 의존 없음 → 단위 시험 (test_center_box.cpp)
//   approach (COARSE → 팔 ALIGN → FINE) 뒤에 자동으로 center_box <거리> 를 붙임 (그 접근에서 퍽을 집을 때만):
//   ① 팔 place_box.arm_pose (BOX_VIEW, 상자 윗면 위) — 루프 동안 그대로 둠 (ALIGN ↔ BOX_VIEW 왕복 3 ~ 5 s 를 아낌)
//   ② approach.center.mode → settle → /perception/place_box/measure (팔 Arducam) → 잰 상자로 옆 e · 요 psi (center_error)
//   ③ 판단 (control/box_center.hpp decide_loop, k = 지금까지 보낸 drive goal 수):
//        DONE (|e| ≤ 5 mm · |psi| ≤ 1°) · FINAL_WARN (k = max_iter, |e| ≤ 10 mm, 경고) · FAIL (못 잼 · max_shift · 라이다 불일치 · 반복 다 씀) → ⑤
//        SHIFT → ④
//   ④ kFineMode 켜고 /drive/align goal.center (잰 상자, center_iter k) → 결과 center_status:
//        MOVED → ② 다시 재기 · DONE (drive 허용 안이라 안 움직임) · SKIPPED · FALLBACK → 사유 기록 뒤 ⑤
//   ⑤ 팔 align.arm_pose (ALIGN) 로 한 번 되돌리고 끝 (실패 아님). drive goal 실패 (옮긴 뒤 FINE 도 못 맞춤) · 시간 초과는 단계 실패 → 앞 approach 부터 다시
#pragma once

#include <cmath>
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

// center_box 단계 시간 상한 [s] = 팔 자세 2 번 (BOX_VIEW · 끝 ALIGN) + 첫 settle · 재기 + 반복마다 (drive 중앙 맞춤 + settle · 재기) + 여유
//   반복 = approach.center.max_iter (drive goal 최대 수). 모드 전환 (서비스) 은 여유에 넣음
inline double center_box_timeout(double pose_s, double settle_s, double measure_s, double drive_center_s, int iters,
  double margin_s)
{
  const int k = iters < 0 ? 0 : iters;
  return 2.0 * pose_s + settle_s + measure_s + k * (drive_center_s + settle_s + measure_s) + margin_s;
}

// 잰 상자 (base_link 로 옮긴 윗면 중심 bx, by · 요 box_yaw) → 로봇의 옆 오차 · 요 오차 (control/box_center.hpp 와 같은 부호)
//   법선 = 상자 요 + n·90° 중 로봇 yaw (0) 에 가장 가까운 것 → psi = 법선 − 로봇 yaw ∈ [−45°, 45°]
//   e = 상자 중심선 (중심을 지나 psi 방향) 에서 로봇 (원점) 의 옆 거리, + = 로봇이 중심선 왼쪽
struct CenterErr
{
  double lateral = 0.0;   // e [m]
  double yaw = 0.0;       // psi [rad]
  double depth = 0.0;     // 중심선을 따라 로봇 → 상자 중심 [m] (> 0 = 상자가 앞)
};
inline CenterErr center_error(double bx, double by, double box_yaw)
{
  const double q = M_PI / 2.0;
  double psi = std::remainder(box_yaw, q);   // [−45°, 45°]
  if (!std::isfinite(psi)) {psi = 0.0;}
  CenterErr e;
  e.yaw = psi;
  e.lateral = -by * std::cos(psi) + bx * std::sin(psi);
  e.depth = bx * std::cos(psi) + by * std::sin(psi);
  return e;
}

// drive 가 돌려준 라이다 옆 (끝 자세, NaN = 못 잼) 과 Arducam 옆 e 가 limit 넘게 다르면 true (교차확인)
inline bool lidar_disagrees(double lidar_lateral, double cam_lateral, double limit)
{
  return std::isfinite(lidar_lateral) && std::isfinite(cam_lateral) && limit > 0.0 &&
         std::fabs(lidar_lateral - cam_lateral) > limit;
}

}  // namespace mission
