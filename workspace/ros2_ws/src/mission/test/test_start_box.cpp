// 시작 박스 집기 계산 (start_box.hpp): 회전각 고르기 · 면 고르기 · 범위 자름 · 퍽 자리 되살리기 · odom 퍽 자리 (#226, S1)
#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

#include "mission/start_box.hpp"

using mission::StartBoxGeom;

namespace
{
constexpr double kDeg = M_PI / 180.0;
constexpr double kFwd = 0.2665;   // J1 -> 퍽 앞 (approach_poses B<n>:S pucks fwd)

// 계획기 poses.puck_after_turn (계획기 중심 C 둘레, deg)
std::pair<double, double> puck_after_turn(const StartBoxGeom & g, double fwd, double lat, double deg)
{
  const double t = deg * kDeg, dx = g.j1_x + fwd - g.center_x, dy = lat;
  return {g.center_x + dx * std::cos(t) + dy * std::sin(t), -dx * std::sin(t) + dy * std::cos(t)};
}

// 상자 (FINE 자리 중심 (box_x + sx, sy), 차체 기준 요 beta) 위 퍽 (상자 틀 (0, lat)) -> 차체 틀
std::pair<double, double> puck(const StartBoxGeom & g, double beta, double lat, double sx, double sy)
{
  return mission::sb_puck_in_frame(g.box_x + sx, sy, beta, 0.0, lat);
}

// 수치 풀이: 0.01° 격자에서 어긋남이 가장 작은 n (같으면 |n| 작은 쪽) [deg]
std::pair<double, double> grid(const StartBoxGeom & g, double px, double py, double beta, double lo, double hi)
{
  double bn = 0.0, br = 1e9;
  for (long i = std::lround(lo * 100); i <= std::lround(hi * 100); ++i) {
    const double n = i / 100.0;
    const double m = mission::sb_misalign(g, px, py, beta, n * kDeg) / kDeg;
    if (m < br - 1e-9 || (std::abs(m - br) <= 1e-9 && std::abs(n) < std::abs(bn))) {bn = n; br = m;}
  }
  return {bn, br};
}
}  // namespace

// 이름 각 <-> 실제 회전: J1 옆 이동 = (j1_x - center_x) sin n (drive turn_angle_rad 와 같음)
TEST(StartBox, NameToPhi)
{
  StartBoxGeom g;
  for (double n : {-30.0, -17.06, -5.0, 0.0, 12.3, 30.0}) {
    const auto [jx, jy] = mission::sb_j1(g, n * kDeg);
    EXPECT_NEAR(jy, (g.j1_x - g.center_x) * std::sin(n * kDeg), 1e-12);
    EXPECT_NEAR(std::hypot(jx - g.icr_x, jy), g.j1_x - g.icr_x, 1e-12);
    EXPECT_NEAR(mission::sb_name_from_phi(g, mission::sb_phi(g, n * kDeg)), n * kDeg, 1e-12);
  }
}

// S1: 고른 n · 남는 어긋남이 수치 풀이 (0.01° 격자) 와 0.05° 안. β -45 ~ 45 (5° 간격) x 퍽 좌 · 우 x 밀림 x 범위
TEST(StartBox, ChooseMatchesGrid)
{
  StartBoxGeom g;
  int cases = 0;
  double max_dn = 0.0, max_dr = 0.0, max_res = 0.0;
  const std::pair<double, double> ranges[] = {{-45, 45}, {-10, 5}, {-3, 30}, {-30, -20}};
  for (const auto & [lo, hi] : ranges) {
    for (int b = -45; b <= 45; b += 5) {
      for (double lat : {0.04, -0.04}) {
        for (const auto & [sx, sy] : {std::pair{0.0, 0.0}, std::pair{0.05, -0.05}, std::pair{-0.05, 0.05}}) {
          const auto [px, py] = puck(g, b * kDeg, lat, sx, sy);
          const auto t = mission::sb_choose_turn(g, px, py, b * kDeg, lo, hi, 30.0);
          const auto [gn, gr] = grid(g, px, py, b * kDeg, std::max(lo, -30.0), std::min(hi, 30.0));
          EXPECT_NEAR(t.residual, gr, 0.05) << "b " << b << " lat " << lat << " range " << lo << ".." << hi;
          EXPECT_NEAR(t.n, gn, 0.05) << "b " << b << " lat " << lat << " range " << lo << ".." << hi;
          EXPECT_LE(t.residual, gr + 1e-6);   // 닫힌 풀이가 격자보다 나쁘지 않음
          max_dn = std::max(max_dn, std::abs(t.n - gn));
          max_dr = std::max(max_dr, std::abs(t.residual - gr));
          if (lo == -45) {max_res = std::max(max_res, t.residual);}
          ++cases;
        }
      }
    }
  }
  std::cout << "[S1] 격자 비교 " << cases << " 경우: |n - 격자| 최대 " << max_dn << "°, |어긋남 - 격자| 최대 " << max_dr
            << "°, 범위 ±30 에서 남는 어긋남 최대 " << max_res << "°\n";
}

// S1: 같은 중심 (icr_x = center_x) · β = 0 이면 계획기 turn_deg = asin(lat / d) 와 0.01° 안. 두 중심 차는 기록
TEST(StartBox, PlannerTurnDegSameCenter)
{
  StartBoxGeom same;
  same.icr_x = same.center_x;
  StartBoxGeom real;   // icr_x -0.0658
  const double d = same.j1_x - same.center_x;
  for (double lat : {0.04, -0.04}) {
    const auto [px, py] = puck(same, 0.0, lat, 0.0, 0.0);
    const auto t = mission::sb_choose_turn(same, px, py, 0.0, -45, 45, 30.0);
    EXPECT_NEAR(t.n, std::asin(lat / d) / kDeg, 0.01);
    EXPECT_NEAR(t.residual, 0.0, 1e-6);
  }
  double max_diff = 0.0;
  for (int b = -45; b <= 45; b += 5) {
    for (double lat : {0.04, -0.04}) {
      const auto [px, py] = puck(real, b * kDeg, lat, 0.0, 0.0);
      const auto a = mission::sb_choose_turn(same, px, py, b * kDeg, -45, 45, 30.0);
      const auto c = mission::sb_choose_turn(real, px, py, b * kDeg, -45, 45, 30.0);
      // 같은 이름 각을 실제 중심으로 돌렸을 때 남는 어긋남 차 (계획기 중심으로 고르면 얼마나 손해인가)
      const double r_planner_on_real = mission::sb_misalign(real, px, py, b * kDeg, a.n * kDeg) / kDeg;
      max_diff = std::max(max_diff, std::abs(r_planner_on_real - c.residual));
    }
  }
  std::cout << "[S1] 두 중심 (계획기 center_x vs 실제 icr_x) 으로 고른 각의 남는 어긋남 차 최대 " << max_diff << "°\n";
  EXPECT_LT(max_diff, 1.0);
}

// 면 고르기: β 가 ±90° 돌아도 (같은 직육면체) 같은 답, 축 기준 ±45° 접기
TEST(StartBox, FacePeriodic)
{
  StartBoxGeom g;
  for (int b = -40; b <= 40; b += 10) {
    const auto [px, py] = puck(g, b * kDeg, 0.04, 0.02, -0.01);
    const auto t0 = mission::sb_choose_turn(g, px, py, b * kDeg, -45, 45, 30.0);
    for (int k : {-1, 1, 2}) {
      const auto tk = mission::sb_choose_turn(g, px, py, b * kDeg + k * M_PI / 2, -45, 45, 30.0);
      EXPECT_NEAR(tk.n, t0.n, 1e-9);
      EXPECT_NEAR(tk.residual, t0.residual, 1e-9);
    }
  }
  EXPECT_NEAR(mission::sb_fold45(50 * kDeg), -40 * kDeg, 1e-12);
  EXPECT_NEAR(mission::sb_fold45_keep(45 * kDeg), 45 * kDeg, 1e-12);
  EXPECT_NEAR(mission::sb_fold45_keep(-45 * kDeg), -45 * kDeg, 1e-12);
  EXPECT_NEAR(mission::sb_fold45_keep(135 * kDeg), 45 * kDeg, 1e-12);
  EXPECT_NEAR(mission::sb_fold45_keep(-135 * kDeg), -45 * kDeg, 1e-12);
  EXPECT_NEAR(mission::sb_fold45_keep(100 * kDeg), 10 * kDeg, 1e-12);
  EXPECT_NEAR(mission::sb_fold45_keep(-50 * kDeg), 40 * kDeg, 1e-12);
}

// 자름: 범위 끝 (lo · hi · max 30) 에 걸리면 끝값, 그 끝이 어긋남이 작은 쪽
TEST(StartBox, Clip)
{
  StartBoxGeom g;
  // 왼쪽 퍽 (lat +0.04), β -20°: 최적은 +30 밖 -> max 30 · hi 12 · max 20 끝값
  {
    const auto [px, py] = puck(g, -20 * kDeg, 0.04, 0.0, 0.0);
    const auto t = mission::sb_choose_turn(g, px, py, -20 * kDeg, -45, 45, 30.0);
    EXPECT_NEAR(t.n, 30.0, 1e-9);
    EXPECT_EQ(t.name, "turn_p30.0");
    const auto t2 = mission::sb_choose_turn(g, px, py, -20 * kDeg, -45, 12, 30.0);   // hi 12
    EXPECT_NEAR(t2.n, 12.0, 1e-9);
    EXPECT_EQ(t2.name, "turn_p12.0");
    const auto t3 = mission::sb_choose_turn(g, px, py, -20 * kDeg, -45, 45, 20.0);   // max_turn 20
    EXPECT_NEAR(t3.n, 20.0, 1e-9);
  }
  // 왼쪽 퍽, β +20°: 풀이 -25.4 (어긋남 0) 가 lo -12 밖 -> -12
  {
    const auto [px, py] = puck(g, 20 * kDeg, 0.04, 0.0, 0.0);
    const auto t = mission::sb_choose_turn(g, px, py, 20 * kDeg, -12, 45, 30.0);
    EXPECT_NEAR(t.n, -12.0, 1e-9);
    EXPECT_EQ(t.name, "turn_m12.0");
    EXPECT_NEAR(t.residual, mission::sb_misalign(g, px, py, 20 * kDeg, -12 * kDeg) / kDeg, 1e-9);
  }
  // 왼쪽 퍽, β 0: 풀이 +17.06 이 hi 10 밖 -> 10
  {
    const auto [px, py] = puck(g, 0.0, 0.04, 0.0, 0.0);
    const auto t = mission::sb_choose_turn(g, px, py, 0.0, -45, 10, 30.0);
    EXPECT_NEAR(t.n, 10.0, 1e-9);
    EXPECT_GT(t.residual, 1.0);
    EXPECT_NEAR(t.residual, mission::sb_misalign(g, px, py, 0.0, 10 * kDeg) / kDeg, 1e-9);
    const auto u = mission::sb_choose_turn(g, px, py, 0.0, -45, 45, 30.0);
    EXPECT_NEAR(u.n, 17.06, 0.01);
    EXPECT_EQ(u.name, "turn_p17.1");
  }
  // 범위가 0 하나면 turn_0
  {
    const auto [px, py] = puck(g, 20 * kDeg, -0.04, 0.0, 0.0);
    const auto t = mission::sb_choose_turn(g, px, py, 20 * kDeg, 0, 0, 30.0);
    EXPECT_EQ(t.name, "turn_0");
    EXPECT_EQ(t.n_sent, 0.0);
  }
  // 이름: 0.1° 자리, |n| < 0.5 -> turn_0
  EXPECT_EQ(mission::sb_turn_name(12.34), "turn_p12.3");
  EXPECT_EQ(mission::sb_turn_name(-4.66), "turn_m4.7");
  EXPECT_EQ(mission::sb_turn_name(0.49), "turn_0");
  EXPECT_EQ(mission::sb_turn_name(-0.5), "turn_m0.5");
}

// 퍽 자리 되살리기: 계획 줄 (회전 뒤 예상 자리 · 이름 각) -> 회전 전 자리 -> 상자 틀 local · odom 자리, 1 mm 안
TEST(StartBox, PuckRecover)
{
  StartBoxGeom g;
  for (double lat : {0.04, -0.04}) {
    const double deg = std::asin(lat / (g.j1_x - g.center_x)) / kDeg;   // 계획기 turn_deg (17.06)
    const auto [lx, ly] = puck_after_turn(g, kFwd, lat, deg);
    // 계획 줄은 4 자리 · 이름은 정수 각 (turn_p17)
    const double lx4 = std::round(lx * 1e4) / 1e4, ly4 = std::round(ly * 1e4) / 1e4;
    const auto [ox, oy] = mission::sb_puck_local(g, lx4, ly4, std::round(deg) * kDeg);
    EXPECT_NEAR(ox, 0.0, 0.001);
    EXPECT_NEAR(oy, lat, 0.001);
    const auto [ex, ey] = mission::sb_puck_local(g, lx, ly, deg * kDeg);   // 정확한 각이면 정확히
    EXPECT_NEAR(ex, 0.0, 1e-9);
    EXPECT_NEAR(ey, lat, 1e-9);
    std::cout << "[S1] lat " << lat << ": 정수 각 · 4 자리로 되살린 local 오차 " << std::hypot(ox, oy - lat) * 1000 << " mm\n";
    // 자세표 퍽 (fwd, lat) 으로 맞추면 정확히 (5 mm 안), 멀면 그대로
    const std::vector<std::pair<double, double>> table{{kFwd, 0.04}, {kFwd, -0.04}};
    bool snapped = false;
    const auto [sx, sy] = mission::sb_puck_local_snap(g, lx4, ly4, std::round(deg) * kDeg, table, 0.005, &snapped);
    EXPECT_TRUE(snapped);
    EXPECT_NEAR(sx, kFwd + g.j1_x - g.box_x, 1e-12);
    EXPECT_NEAR(sy, lat, 1e-12);
    const std::vector<std::pair<double, double>> far{{kFwd + 0.02, lat}};
    const auto [fx, fy] = mission::sb_puck_local_snap(g, lx4, ly4, std::round(deg) * kDeg, far, 0.005, &snapped);
    EXPECT_FALSE(snapped);
    EXPECT_NEAR(fx, ox, 1e-12);
    EXPECT_NEAR(fy, oy, 1e-12);
  }
  // inline (W · E): 회전 없음, 퍽이 J1 앞 일직선 -> local (±0.04, 0)
  {
    const auto [ox, oy] = mission::sb_puck_local(g, g.box_x - 0.04, 0.0, 0.0);
    EXPECT_NEAR(ox, -0.04, 1e-12);
    EXPECT_NEAR(oy, 0.0, 1e-12);
  }
  // odom 자리: 상자 (1.2, -0.3) 축 100°, local (0, 0.04) -> 중심 + R(100°) local
  {
    const auto [x, y] = mission::sb_puck_in_frame(1.2, -0.3, 100 * kDeg, 0.0, 0.04);
    EXPECT_NEAR(x, 1.2 - 0.04 * std::sin(100 * kDeg), 1e-12);
    EXPECT_NEAR(y, -0.3 + 0.04 * std::cos(100 * kDeg), 1e-12);
  }
}

// README '실제 중심' 표: 왼쪽 퍽 밀림 0, 범위 ±30 의 남는 어긋남 (β +10/20/30/45 = 0/0/8.5/24.2, -β = 4.3/14.1/23.7/27.7)
TEST(StartBox, ReadmeTable)
{
  StartBoxGeom g;
  const double beta[] = {10, 20, 30, 45};
  const double plus[] = {0.0, 0.0, 8.5, 24.2}, minus[] = {4.3, 14.1, 23.7, 27.7};
  for (int i = 0; i < 4; ++i) {
    for (int sgn : {1, -1}) {
      const double b = sgn * beta[i] * kDeg;
      const auto [px, py] = puck(g, b, 0.04, 0.0, 0.0);
      const auto t = mission::sb_choose_turn(g, px, py, b, -45, 45, 30.0);
      EXPECT_NEAR(t.residual, sgn > 0 ? plus[i] : minus[i], 0.1) << "beta " << sgn * beta[i];
      std::cout << "[S1] 왼쪽 퍽 β " << sgn * beta[i] << "°: n " << t.n << "°, 남는 어긋남 " << t.residual << "°\n";
    }
  }
}
