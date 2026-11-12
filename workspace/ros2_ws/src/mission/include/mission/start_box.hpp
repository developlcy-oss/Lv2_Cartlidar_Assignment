// 시작 박스 집기: 잰 상자 요로 회전각 · target_look 다시 계산 (start_box.*, #226) — ROS 없음, 헤더만 (test_start_box.cpp)
//   좌표: 회전 기준 틀 (FINE 정렬 때 차체 = drive turn 기준 yaw) 의 base x · y [m], 각 [rad] (이름 · 범위만 [deg])
//   - 이름 각 n: drive turn_<p|m><n>. drive 는 J1 옆 이동이 (j1_x - center_x) sin n 이 되게 실제 회전 중심 icr_x 둘레로
//     phi = asin((j1_x - center_x) sin n / (j1_x - icr_x)) 만큼 돈다 (control turn_skill.hpp turn_angle_rad 와 같은 식).
//   - 정책의 닫힘 방향 = J1 -> 퍽 방향. 직육면체 퍽 면은 상자 축과 같고 90° 마다 같다 -> 남는 면 어긋남 = |J1 -> 퍽 방향 - 상자 축| (±45° 로 접음).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

namespace mission
{

struct StartBoxGeom
{
  double j1_x{0.096};        // base -> J1 축 x [m] (mission_planner poses.J1_X)
  double center_x{-0.04035}; // 계획기 회전 중심 x [m] (poses.CENTER_X, drive turn.center_x)
  double icr_x{-0.0658};     // 실제 제자리 회전 중심 x [m] (control robot_geometry.yaml base.icr_x)
  double box_x{0.3625};      // FINE 자리 상자 중심 x [m] (앞 끝 0.1328 + gap 0.1272 + 반 변 0.1025, 모든 접근 같음)
};

// 상자 축 각을 ±45° 로 접음 (직육면체 면 90° 주기)
inline double sb_fold45(double a)
{
  return std::remainder(a, M_PI / 2);
}

// 잰 상자 요를 지도 축 기준 ±45° 로 접되, 딱 ±45° (1e-4 x 90° 안) 는 잰 부호를 그대로 둠.
//   정사각 윗면만으로는 +45° 와 -45° 를 못 가름 (퍽 자리는 57 mm 다름). perception 은 사전값 요 (0) ±45° 안 값을 냄
inline double sb_fold45_keep(double a)
{
  const double q = a / (M_PI / 2);
  double k = std::round(q);
  const double frac = q - std::trunc(q);
  if (std::abs(std::abs(frac) - 0.5) < 1e-4) {k = std::trunc(q);}   // 반 바퀴 경계: 0 쪽 정수
  return a - k * (M_PI / 2);
}

// 이름 각 n -> 실제 회전 phi
inline double sb_phi(const StartBoxGeom & g, double n)
{
  const double r = g.j1_x - g.icr_x;
  const double lat = (g.j1_x - g.center_x) * std::sin(n);
  return std::asin(std::clamp(lat / r, -1.0, 1.0));
}

// 실제 회전 phi -> 이름 각 n
inline double sb_name_from_phi(const StartBoxGeom & g, double phi)
{
  const double r = g.j1_x - g.icr_x;
  return std::asin(std::clamp(r * std::sin(phi) / (g.j1_x - g.center_x), -1.0, 1.0));
}

// 이름 각 n 으로 돈 뒤 J1 자리 (회전 기준 틀): icr + R (cos phi, sin phi), R = j1_x - icr_x
inline std::pair<double, double> sb_j1(const StartBoxGeom & g, double n)
{
  const double r = g.j1_x - g.icr_x, phi = sb_phi(g, n);
  return {g.icr_x + r * std::cos(phi), r * std::sin(phi)};
}

// 이름 각 n 으로 돈 뒤 남는 면 어긋남 [rad, >= 0]: |fold45(J1 -> 퍽 방향 - 상자 축 beta)|
inline double sb_misalign(const StartBoxGeom & g, double px, double py, double beta, double n)
{
  const auto [jx, jy] = sb_j1(g, n);
  return std::abs(sb_fold45(std::atan2(py - jy, px - jx) - beta));
}

struct SbTurn
{
  double n{0.0};          // 고른 이름 각 [deg] (풀이 그대로)
  double residual{0.0};   // 그때 남는 면 어긋남 [deg]
  double lo{0.0}, hi{0.0};// 쓴 범위 [deg] (turn_range ∩ ±max_turn)
  double n_sent{0.0};     // 보낼 이름 각 [deg] (0.1° 반올림, |n| < 0.5 면 0)
  double residual_sent{0.0};
  std::string name;       // turn_p12.3 · turn_m4.7 · turn_0
};

// 이름 각 [deg] -> drive skill 이름 (0.1° 자리, |n| < 0.5° 면 turn_0)
inline std::string sb_turn_name(double n_deg)
{
  if (std::abs(n_deg) < 0.5) {return "turn_0";}
  char buf[32];
  std::snprintf(buf, sizeof(buf), "turn_%c%.1f", n_deg > 0 ? 'p' : 'm', std::abs(n_deg));
  return buf;
}

// 회전각 고르기: 퍽 (px, py) · 상자 축 beta (회전 기준 틀), 이름 각 범위 [max(lo, -max), min(hi, max)] [deg].
//   남는 어긋남이 가장 작은 n. 후보 = 범위 양 끝 + 면마다 (beta' = beta, beta ± 90°) J1 -> 퍽 = beta' 가 되는 풀이
//   (R sin(phi - beta') = cross(e(beta'), P - icr)) + J1 -> 퍽 방향이 극값인 자리 (어긋남이 0 이 안 되는 극소). 같으면 |n| 작은 쪽.
inline SbTurn sb_choose_turn(
  const StartBoxGeom & g, double px, double py, double beta, double lo_deg, double hi_deg, double max_deg)
{
  constexpr double kDeg = M_PI / 180.0;
  SbTurn out;
  out.lo = std::max(lo_deg, -max_deg);
  out.hi = std::min(hi_deg, max_deg);
  if (out.lo > out.hi) {out.lo = out.hi = 0.0;}   // 범위가 비면 돌지 않음
  const double nlo = out.lo * kDeg, nhi = out.hi * kDeg;
  const double plo = sb_phi(g, nlo), phi_hi = sb_phi(g, nhi);
  const double r = g.j1_x - g.icr_x, vx = px - g.icr_x, vy = py;
  std::vector<double> cand{nlo, nhi};
  auto add_phi = [&](double phi) {
      phi = std::remainder(phi, 2 * M_PI);
      if (phi >= plo - 1e-12 && phi <= phi_hi + 1e-12) {cand.push_back(std::clamp(sb_name_from_phi(g, phi), nlo, nhi));}
    };
  for (int k = -1; k <= 1; ++k) {
    const double b = beta + k * M_PI / 2;
    const double c = (std::cos(b) * vy - std::sin(b) * vx) / r;
    if (std::abs(c) > 1.0) {continue;}
    add_phi(b + std::asin(c));
    add_phi(b + M_PI - std::asin(c));
  }
  const double rho = std::hypot(vx, vy);
  if (rho > r) {
    const double a = std::atan2(vy, vx), w = std::acos(r / rho);
    add_phi(a + w);
    add_phi(a - w);
  }
  double best_n = 0.0, best_r = 1e9;
  for (double n : cand) {
    const double m = sb_misalign(g, px, py, beta, n);
    if (m < best_r - 1e-9 || (std::abs(m - best_r) <= 1e-9 && std::abs(n) < std::abs(best_n))) {
      best_n = n;
      best_r = m;
    }
  }
  out.n = best_n / kDeg;
  out.residual = best_r / kDeg;
  // 보낼 값: 0.1° 반올림 (범위 밖으로 나가면 안쪽 0.1° 로), |n| < 0.5° 면 turn_0 (기준 방향 그대로)
  double ns = std::round(out.n * 10.0) / 10.0;
  if (ns > out.hi) {ns = std::floor(out.hi * 10.0 + 1e-9) / 10.0;}
  if (ns < out.lo) {ns = std::ceil(out.lo * 10.0 - 1e-9) / 10.0;}
  if (std::abs(ns) < 0.5) {ns = 0.0;}
  out.n_sent = ns;
  out.residual_sent = sb_misalign(g, px, py, beta, ns * kDeg) / kDeg;
  out.name = sb_turn_name(ns);
  return out;
}

// 계획 줄 target_look:base_footprint:lx:ly (회전 뒤 예상 자리) + 앞 turn 이름 각 t [rad] -> 회전 전 자리 P0 (계획기 기하, center_x 둘레).
//   mission_planner poses.puck_after_turn 의 역: P0 = C + R(t) (lx - C, ly)
inline std::pair<double, double> sb_puck_before_turn(const StartBoxGeom & g, double lx, double ly, double t)
{
  const double c = std::cos(t), s = std::sin(t), dx = lx - g.center_x;
  return {g.center_x + c * dx - s * ly, s * dx + c * ly};
}

// 상자 틀 (FINE 자리 차체 축) 의 퍽 자리 = P0 - (box_x, 0). S · N 접근 (0, ±0.04), W · E (±0.04, 0)
inline std::pair<double, double> sb_puck_local(const StartBoxGeom & g, double lx, double ly, double t)
{
  const auto [x0, y0] = sb_puck_before_turn(g, lx, ly, t);
  return {x0 - g.box_x, y0};
}

// 위와 같고, 되살린 P0 에서 tol 안에 자세표 퍽 (J1 앞 fwd · 옆 lat -> (j1_x + fwd, lat)) 이 있으면 그 값을 씀.
//   계획 줄은 이름 각이 정수 (turn_p17, 실제 계획 각 17.06) · 자리가 4 자리라 되살린 자리가 약 0.4 mm 어긋나고, 그만큼 회전각이 약 0.2° 틀어짐.
//   snapped = 자세표 값을 썼으면 true
inline std::pair<double, double> sb_puck_local_snap(
  const StartBoxGeom & g, double lx, double ly, double t, const std::vector<std::pair<double, double>> & table, double tol,
  bool * snapped = nullptr)
{
  const auto [px, py] = sb_puck_before_turn(g, lx, ly, t);
  double x0 = px, y0 = py, best = tol;
  bool hit = false;
  for (const auto & [fwd, lat] : table) {
    const double d = std::hypot(g.j1_x + fwd - px, lat - py);
    if (d <= best) {
      best = d;
      hit = true;
      x0 = g.j1_x + fwd;
      y0 = lat;
    }
  }
  if (snapped) {*snapped = hit;}
  return {x0 - g.box_x, y0};
}

// 잰 상자 (중심 cx, cy · 상자 틀 x 축 방향 axis, 같은 frame) + local -> 그 frame 의 퍽 자리
inline std::pair<double, double> sb_puck_in_frame(double cx, double cy, double axis, double lx, double ly)
{
  const double c = std::cos(axis), s = std::sin(axis);
  return {cx + c * lx - s * ly, cy + s * lx + c * ly};
}

}  // namespace mission
