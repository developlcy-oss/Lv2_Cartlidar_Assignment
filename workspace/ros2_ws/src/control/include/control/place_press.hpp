// PlacePress — 상자 놓기 "위에서 멈춘 뒤 천천히 눌러 놓기" (manipulator_node runPlaceStream, place.press_*)
//
// 왜 (2026-11-21 실물 dice --bench): 45° 직선으로 놓는 자리까지 0.05 m/s 로 내려가 바로 벌리면, 모델 · 실물 차이로
//     집게가 상자를 넘어가 퍽이 공중에서 떨어지거나 (1 번 퍽) 퍽이 상자를 눌러 밀린다 (2 번 퍽).
// 어떻게: 놓는 자리보다 press_height 위 (above) 에서 멈춤 → 수직으로 press_speed 로 천천히 내려감 →
//     퍽이 상자에 닿으면 그 자리에서 벌림 · 머묾 → 수직으로 올라 빠짐. 닿지 않으면 계획대로 놓는 자리 (tip) 까지 내려가 벌린다.
//   닿음 = 수직으로 내려갈 때 움직이는 관절 2 ~ 4 의 (계획 − 측정) 이 누르기 시작 때 값보다 threshold 넘게 벌어짐 (연속 PressContact::kHits 주기).
//     시작 때 값을 빼서 처짐 보정 잔차 · 추종 지연을 닿음으로 보지 않고, 여러 주기 연속이어야 해서 한 번 튐을 거름.
// releaseFrom: 닿은 자리 (측정 관절) 에서 시작하는 새 시간표 (벌림 → 머묾 → 빠지는 경유점).
//   빠지는 경유점 중 tip → above 수직 구간은 rise (tip 위 높이) 를 갖고, 닿은 높이 이하인 점은 trimRetreat 로 버린다
//   (안 버리면 벌린 직후 팔이 그 점까지 아래로 빠르게 내려가 상자를 찍거나 놓은 퍽을 끈다, 2026-11-21 검토).
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "control/sag_comp.hpp"

namespace control
{

struct TimedWaypoint
{
  Arm5 q{};
  double min_dt{0.02};   // 앞 점에서 이 점까지 최소 시간 (s). 실제 = max(min_dt, Δq 최대 / speed)
  double rise{-1.0};     // tip → above 수직 구간 점이면 tip 위 높이 (m), 아니면 음수
};

// 누르는 중 닿음 판정: 관절 2 ~ 4 의 (계획 − 측정) 이 시작 때 값 (base) 에서 threshold 넘게 벌어진 주기가 kHits 번 연속
class PressContact
{
public:
  static constexpr int kHits = 3;

  void reset() {have_base_ = false; hits_ = 0;}

  // 매 주기 (누르는 구간 안에서만). 반환: 닿음
  bool update(const Arm5 & plan, const Arm5 & measured, double threshold)
  {
    if (!have_base_) {
      for (size_t j = 0; j < 5; ++j) {base_[j] = plan[j] - measured[j];}
      have_base_ = true;
      return false;
    }
    hits_ = deviation(plan, measured) > threshold ? hits_ + 1 : 0;
    return hits_ >= kHits;
  }

  // 관절 2 ~ 4 (index 1 ~ 3) 의 시작 때 대비 벌어짐 최대값 (rad)
  double deviation(const Arm5 & plan, const Arm5 & measured) const
  {
    double d = 0.0;
    for (size_t j = 1; j <= 3; ++j) {d = std::max(d, std::abs((plan[j] - measured[j]) - (have_base_ ? base_[j] : 0.0)));}
    return d;
  }

private:
  Arm5 base_{};
  bool have_base_{false};
  int hits_{0};
};

// 누르는 구간 [t0, t1] 에서 지금까지 내려간 거리 (m): 진행 비율 × press_height
inline double pressDepth(double t, double t0, double t1, double press_height)
{
  if (t1 <= t0) {return press_height;}
  return std::clamp((t - t0) / (t1 - t0), 0.0, 1.0) * press_height;
}

// 닿은 높이 (tip 위 contact_rise m) 이하인 앞쪽 수직 구간 점을 버림 → 빠질 때 아래로 내려가지 않음
inline std::vector<TimedWaypoint> trimRetreat(const std::vector<TimedWaypoint> & after, double contact_rise)
{
  size_t k = 0;
  while (k < after.size() && after[k].rise >= 0.0 && after[k].rise <= contact_rise + 1e-9) {++k;}
  return std::vector<TimedWaypoint>(after.begin() + static_cast<std::ptrdiff_t>(k), after.end());
}

// 닿은 자리 q_contact 에서: 그리퍼 hold_grip → release (open_time) → 머묾 (dwell) → after 경유점 (speed rad/s)
inline JointTimeline releaseFrom(
  const Arm5 & q_contact, double hold_grip, double release, double open_time, double dwell,
  const std::vector<TimedWaypoint> & after, double speed)
{
  JointTimeline tl(q_contact, hold_grip);
  tl.hold(open_time, release);
  tl.hold(dwell, release);
  Arm5 prev = q_contact;
  for (const auto & w : after) {
    double d = 0.0;
    for (size_t j = 0; j < 5; ++j) {d = std::max(d, std::abs(w.q[j] - prev[j]));}
    tl.moveTo(w.q, std::max(w.min_dt, speed > 0.0 ? d / speed : 0.0), release);
    prev = w.q;
  }
  return tl;
}

}  // namespace control
