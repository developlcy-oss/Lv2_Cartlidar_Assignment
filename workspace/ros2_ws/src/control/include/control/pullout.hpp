// navigate 전 곧게 전진 (2026-11-19): 뒤가 벽에 붙은 채 Nav2 로 보내면 RotationShim 제자리 회전에 뒤 모서리가 벽 칸에 걸림
//   (Gazebo: 팔레트 집기 자리 (1.47, 0.31), 뒤 벽 0.29 m → 30° 돌면 치명 칸 → 현재 자세가 충돌 판정이라 MPPI · spin · backup 모두 거부)
//   규칙: 뒤 /scan (π ± rear_half_angle) 가장 가까운 점이 rear_clear_m 안이면 Nav2 전에 곧게 전진
//         뒤가 rear_clear_m 이상 비거나, max_m 을 가거나, 앞이 막히거나 (앞 /scan · TFmini), timeout_s 가 지나면 멈추고 Nav2 (실패로 끝내지 않음)
//   ROS 의존 없음 → 단위 시험 (test/test_pullout.cpp)
#pragma once

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace control
{

struct PulloutConfig
{
  bool enabled = true;
  double rear_clear_m = 0.33;      // base_link → 뒤 점 거리 [m]. 뒤 모서리 회전 반지름 0.271 + footprint_padding 0.01 + costmap 칸 0.02 + 여유
  double rear_half_angle = 0.785;  // 뒤 ± [rad] (45°, 모서리 방향 ±38° 포함, 옆 통로 벽은 덜 잡게)
  double front_clear_m = 0.35;     // 앞 /scan (± scan.half_angle) 이 이 안이면 전진 안 함 [m] (주행 자세 팔 앞 0.25 + 여유)
  double front_tfmini_m = 0.20;    // TFmini 가 이 안이거나 값 없음 (0.10 m 안일 수 있음) 이면 전진 안 함 [m] (/scan 아래 상자 · 퍽)
  double speed = 0.05;             // [m/s]
  double max_m = 0.06;             // 최대 전진 [m] (odom)
  double timeout_s = 3.0;          // [s]
};

// 뒤 ± half_angle 안 가장 가까운 점 [m]. 점 없음 = +inf
inline double rear_min_range(const std::vector<float> & ranges, double angle_min, double angle_inc, double half_angle)
{
  double m = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < ranges.size(); ++i) {
    const float r = ranges[i];
    if (!std::isfinite(r) || r <= 0.01f) {continue;}
    double a = angle_min + angle_inc * static_cast<double>(i) - M_PI;
    a = std::atan2(std::sin(a), std::cos(a));
    if (std::fabs(a) <= half_angle && r < m) {m = r;}
  }
  return m;
}

// 앞 막힘: 앞 /scan 이 front_clear_m 안 또는 TFmini 가 front_tfmini_m 안 · 값 없음
inline bool pullout_front_blocked(const PulloutConfig & c, bool scan_front_near, std::optional<double> tfmini)
{
  return scan_front_near || !tfmini || *tfmini < c.front_tfmini_m;
}

// navigate 를 받았을 때 전진부터 할지
inline bool pullout_wanted(const PulloutConfig & c, double rear_min, bool front_blocked)
{
  return c.enabled && rear_min < c.rear_clear_m && !front_blocked;
}

enum class PulloutStep { DRIVE, CLEAR, MAX_DISTANCE, FRONT_BLOCKED, TIMEOUT };

// 전진 한 tick 의 판단. 순서: 뒤 비었음 → 최대 거리 → 앞 막힘 → 시간 초과 → 계속
inline PulloutStep pullout_step(const PulloutConfig & c, double rear_min, double traveled, double elapsed, bool front_blocked)
{
  if (rear_min >= c.rear_clear_m) {return PulloutStep::CLEAR;}
  if (traveled >= c.max_m) {return PulloutStep::MAX_DISTANCE;}
  if (front_blocked) {return PulloutStep::FRONT_BLOCKED;}
  if (elapsed > c.timeout_s) {return PulloutStep::TIMEOUT;}
  return PulloutStep::DRIVE;
}

}  // namespace control
