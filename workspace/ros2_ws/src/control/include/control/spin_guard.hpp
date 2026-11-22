// (11-22) 제자리 보정 Spin 의 Nav2 충돌검사를 꺼도 되는지 판정 — ROS 의존 없음
//   배경: 상자 앞에서 Spin 이 시작 직후 "Collision Ahead" 로 끝남 (팔 끝 + padding 이 면에 닿음). 작은 각만 검사를 끔.
//   조건 (모두 만족해야 끔, 하나라도 아니면 검사를 켜 둠 = 안전한 쪽):
//     1) 각도 |dyaw| ≤ max_deg (기본 3°)
//     2) /scan 이 있고 오래되지 않음
//     3) 스캔 점 → 차체 외곽 직사각형 (앞 front_x · 뒤 rear_x · 옆 ±half_y, base_link 기준) 의 거리 ≥ 쓸림 + clear
//        쓸림 = ρmax·|dyaw|  (ρmax = ICR 에서 가장 먼 모서리 거리, 앞 · 뒤 4 모서리 모두 — 호 길이 ≥ 현 이므로 보수적)
//        점이 외곽 안 (거리 0) 이어도 거절. /scan 은 base_link 중심 · 방향으로 가정 (라이다 ±0.13 오프셋은 clear 가 흡수)
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace control
{

struct SpinGuardConfig
{
  double max_deg = 3.0;       // [deg] 이 각 이하만 검사 끔 (0 이하 = 항상 켬)
  double clear = 0.03;        // [m] 쓸림 거리에 더하는 여유
  double front_x = 0.1328;    // [m] base_link → 앞 끝
  double rear_x = 0.2135;     // [m] base_link → 뒤 끝 (양수)
  double half_y = 0.1676;     // [m] 옆 끝 |y|
  double icr_x = -0.0658;     // [m] 회전 중심 x
};

struct SpinGuardResult
{
  bool disable_checks = false;
  const char * reason = "";   // angle_off · big_angle · no_scan · stale_scan · clear_short · ok
  double sweep = 0.0;         // 모서리 쓸림 [m]
  double min_dist = 0.0;      // 스캔 점 ↔ 외곽 최소 거리 [m] (검사했을 때만)
};

// 가장 먼 모서리의 ICR 거리
inline double spin_rho_max(const SpinGuardConfig & c)
{
  const double xf = c.front_x - c.icr_x, xr = -c.rear_x - c.icr_x;
  return std::max(std::hypot(xf, c.half_y), std::hypot(xr, c.half_y));
}

// 점 (px, py) ↔ 외곽 직사각형 거리 (안이면 0)
inline double dist_to_footprint(double px, double py, const SpinGuardConfig & c)
{
  const double dx = std::max({-c.rear_x - px, 0.0, px - c.front_x});
  const double dy = std::max({-c.half_y - py, 0.0, py - c.half_y});
  return std::hypot(dx, dy);
}

// scan_present: 스캔을 받은 적 있음, scan_fresh: 타임스탬프가 최근
inline SpinGuardResult decide_spin_nocheck(double dyaw_rad, bool scan_present, bool scan_fresh,
  const std::vector<float> & ranges, double angle_min, double angle_inc, double range_min,
  const SpinGuardConfig & c)
{
  SpinGuardResult r;
  r.sweep = spin_rho_max(c) * std::fabs(dyaw_rad);
  if (!(c.max_deg > 0.0)) {r.reason = "angle_off"; return r;}
  if (std::fabs(dyaw_rad) > c.max_deg * M_PI / 180.0 + 1e-9) {r.reason = "big_angle"; return r;}
  if (!scan_present) {r.reason = "no_scan"; return r;}
  if (!scan_fresh) {r.reason = "stale_scan"; return r;}
  const double need = r.sweep + c.clear;
  double best = 1e9;
  for (size_t i = 0; i < ranges.size(); ++i) {
    const float rg = ranges[i];
    if (!std::isfinite(rg) || rg <= std::max(0.01, range_min)) {continue;}
    const double a = angle_min + angle_inc * static_cast<double>(i);
    best = std::min(best, dist_to_footprint(rg * std::cos(a), rg * std::sin(a), c));
  }
  r.min_dist = best;
  if (best < need) {r.reason = "clear_short"; return r;}
  r.disable_checks = true;
  r.reason = "ok";
  return r;
}

}  // namespace control
