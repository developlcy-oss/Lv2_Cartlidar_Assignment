// (11-21) 상자 중앙 맞춤 개정 시험: ICR 상쇄 · 앞 모서리 침범 표 · S 자 / 후진 계획 · decide_loop 경계 · fuse_measure ·
//   몬테카를로 (반복 폐루프 vs 열린 고리) — box_center.hpp
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "control/box_center.hpp"

using namespace control;

namespace
{
constexpr double kDeg = M_PI / 180.0;

struct Pose { double x, y, yaw; };

// base_link 자세를 ICR (몸체 x = icr_x) 둘레로 a 만큼 돌림
Pose rotate_about_icr(Pose p, double a, double icr_x)
{
  const double ix = p.x + std::cos(p.yaw) * icr_x, iy = p.y + std::sin(p.yaw) * icr_x;
  const double vx = p.x - ix, vy = p.y - iy, c = std::cos(a), s = std::sin(a);
  return {ix + c * vx - s * vy, iy + s * vx + c * vy, p.yaw + a};
}
Pose straight(Pose p, double d) {return {p.x + d * std::cos(p.yaw), p.y + d * std::sin(p.yaw), p.yaw};}

// 계획을 기하로 실행 (TURN0 은 이미 법선 = 0 이라 생략). 오차: 회전 끝 yaw (d1 · d2), 회전마다 옆 미끄럼
Pose execute(const ShiftPlan & p, double icr_x, double d1 = 0, double d2 = 0, double slip1 = 0, double slip2 = 0)
{
  Pose q{0.0, 0.0, 0.0};
  q = straight(q, -p.back);
  q = rotate_about_icr(q, p.alpha + d1, icr_x);
  q.y += slip1;
  q = straight(q, p.fwd);
  q = rotate_about_icr(q, -p.alpha - d1 + d2, icr_x);
  q.y += slip2;
  return q;
}
}  // namespace

// ICR 오프셋은 TURN1 (+α) · TURN2 (−α) 에서 상쇄 → 끝 자세 옆 이동 = s, 깊이 그대로 (BACK) / |s|/tanα 줄어듦 (S 자)
TEST(BoxCenterLoop, IcrCancels)
{
  BoxCenterConfig c;
  for (double icr : {-0.07, -0.0658, 0.0, 0.07}) {
    for (double lat : {0.03, -0.012, 0.06}) {
      CenterError e;
      e.lateral = lat;
      for (double g0 : {0.13, 0.06}) {   // 0.13 = S 자 되는 틈 (lat 작을 때), 0.06 = 후진 필요
        const auto m = plan_center_move(e, c, g0, 0.15);
        ASSERT_TRUE(m.ok) << icr << " " << lat << " " << g0;
        const auto q = execute(m.p, icr);
        EXPECT_NEAR(q.y, -lat, 1e-9) << "icr " << icr;
        EXPECT_NEAR(q.yaw, 0.0, 1e-12);
        EXPECT_NEAR(-q.x, m.extra_gap, 1e-9);   // 끝 틈 − g0 = 뒤로 간 만큼
      }
    }
  }
}

// 앞 모서리 침범 표 (review_center.md 2 절): ρ 0.260 · φ 40.2°
TEST(BoxCenterLoop, CornerIntrusionTable)
{
  EXPECT_NEAR(corner_intrusion(0.0), 0.0, 1e-12);
  EXPECT_NEAR(corner_intrusion(10 * kDeg), 0.0270, 0.001);
  EXPECT_NEAR(corner_intrusion(20 * kDeg), 0.0454, 0.001);
  EXPECT_NEAR(corner_intrusion(25 * kDeg), 0.0523, 0.001);
  EXPECT_NEAR(corner_intrusion(28 * kDeg), 0.0554, 0.001);
  EXPECT_NEAR(corner_intrusion(-25 * kDeg), corner_intrusion(25 * kDeg), 1e-12);   // 대칭
  EXPECT_NEAR(corner_intrusion(60 * kDeg), std::hypot(0.1986, 0.1676) - 0.1986, 1e-9);   // φ 넘으면 ρ − x0
  // 단조 증가 (0 ~ φ)
  double prev = -1;
  for (double a = 0; a < 40 * kDeg; a += kDeg) {
    const double v = corner_intrusion(a);
    EXPECT_GT(v, prev);
    prev = v;
  }
  // 수치 쓸기와 같음 (모서리 두 개, 각 0 ~ α)
  for (double a : {5 * kDeg, 17 * kDeg, 25 * kDeg, 33 * kDeg}) {
    double worst = 0;
    for (double sy : {0.1676, -0.1676}) {
      for (int k = 0; k <= 400; ++k) {
        const double t = a * k / 400.0;
        const double vx = 0.1328 + 0.0658, vy = sy;
        worst = std::max(worst, -0.0658 + std::cos(t) * vx - std::sin(t) * vy - 0.1328);
      }
    }
    EXPECT_NEAR(corner_intrusion(a), worst, 1e-5) << a / kDeg;
  }
}

TEST(BoxCenterLoop, PlanSCurveVsBack)
{
  BoxCenterConfig c;   // max_alpha 0.35, corner_margin 0.02
  CenterError e;
  e.lateral = 0.01;
  // FINE 자리 틈 0.13: S 자 (후진 없음), 깊이 소모 + 모서리 ≤ 틈
  auto m = plan_center_move(e, c, 0.13, 0.0);
  ASSERT_TRUE(m.ok);
  EXPECT_TRUE(m.s_curve);
  EXPECT_DOUBLE_EQ(m.p.back, 0.0);
  EXPECT_LT(m.p.alpha, 0.0);
  EXPECT_GE(m.min_gap, c.corner_margin - 1e-9);
  // 틈 0.06: S 자 불가 → 후진, 모서리 몫 δ 만큼 끝에 더 떨어짐
  m = plan_center_move(e, c, 0.06, 0.15);
  ASSERT_TRUE(m.ok);
  EXPECT_FALSE(m.s_curve);
  EXPECT_GT(m.extra_gap, 0.0);
  EXPECT_GE(m.extra_gap, corner_intrusion(m.p.alpha, c) + c.corner_margin - 0.06 - 1e-9);   // 모서리 몫 (+ back_min 으로 늘어난 몫)
  EXPECT_GE(m.p.back, c.back_min - 1e-9);
  EXPECT_NEAR(m.p.back - m.extra_gap, 0.01 / std::tan(std::fabs(m.p.alpha)), 1e-9);   // 옆 이동에 쓰인 깊이
  EXPECT_GE(m.min_gap, c.corner_margin - 1e-9);
  // 같은 틈, 뒤 여유 0 → 불가 (사유 rear_room)
  m = plan_center_move(e, c, 0.06, 0.0);
  EXPECT_FALSE(m.ok);
  EXPECT_STREQ(m.reason, "rear_room");
  EXPECT_GT(m.need_back, 0.0);
  // 큰 옆 · 뒤 여유 부족: 각을 alpha_hard 까지 키워 맞춤
  e.lateral = -0.05;
  m = plan_center_move(e, c, 0.13, 0.10);
  ASSERT_TRUE(m.ok);
  EXPECT_FALSE(m.s_curve);
  EXPECT_LE(m.p.back, 0.10 + 1e-9);
  EXPECT_GT(std::fabs(m.p.alpha), c.max_alpha);
  EXPECT_LE(std::fabs(m.p.alpha), c.alpha_hard + 1e-9);
  EXPECT_NEAR(execute(m.p, c.icr_x).y, 0.05, 1e-9);
  // 이미 너무 붙어 TURN0 도 안 됨 (모서리) → corner
  e.yaw_err = 10 * kDeg;
  m = plan_center_move(e, c, 0.02, 0.15);
  EXPECT_FALSE(m.ok);
  EXPECT_STREQ(m.reason, "corner");
  // 옆 0 → none
  CenterError z;
  EXPECT_STREQ(plan_center_move(z, c, 0.13, 0.15).reason, "none");
}

TEST(BoxCenterLoop, DecideLoopBoundaries)
{
  LoopConfig c;   // 5 mm · 1° · 10 mm · 8 cm · K 3
  EXPECT_EQ(decide_loop(0.005, 0.0, 0, c).d, LoopDecision::DONE);
  EXPECT_EQ(decide_loop(-0.005, 0.0174, 2, c).d, LoopDecision::DONE);
  EXPECT_EQ(decide_loop(0.0051, 0.0, 0, c).d, LoopDecision::SHIFT);
  auto v = decide_loop(0.004, 0.02, 0, c);   // 옆은 됐고 요만 1° 넘음 → 한 번 더 (drive 는 기동 없이 법선만)
  EXPECT_EQ(v.d, LoopDecision::SHIFT);
  EXPECT_STREQ(v.why, "yaw");
  EXPECT_EQ(decide_loop(0.08, 0.0, 0, c).d, LoopDecision::SHIFT);
  v = decide_loop(0.0801, 0.0, 0, c);
  EXPECT_EQ(v.d, LoopDecision::FAIL);
  EXPECT_STREQ(v.why, "max_shift");
  EXPECT_EQ(decide_loop(0.010, 0.0, 3, c).d, LoopDecision::FINAL_WARN);
  EXPECT_EQ(decide_loop(0.0101, 0.0, 3, c).d, LoopDecision::FAIL);
  EXPECT_EQ(decide_loop(0.004, 0.0, 3, c).d, LoopDecision::DONE);   // 반복을 다 써도 허용 안이면 DONE
  EXPECT_EQ(decide_loop(0.009, 0.0, 2, c).d, LoopDecision::SHIFT);
  EXPECT_EQ(decide_loop(0.001, 0.0, 0, c, true).d, LoopDecision::FAIL);   // 불일치
  EXPECT_STREQ(decide_loop(NAN, 0.0, 0, c).why, "no_measure");
}

TEST(BoxCenterLoop, FuseMeasure)
{
  FuseConfig c;
  SideMeas cam{true, 0.010, 0.02, 0.13}, lid{true, 0.025, 0.04, 0.16};
  auto f = fuse_measure(cam, lid, c);
  EXPECT_TRUE(f.ok);
  EXPECT_DOUBLE_EQ(f.lateral, 0.010);                       // 옆은 Arducam
  EXPECT_NEAR(f.yaw, 0.93 * 0.02 + 0.07 * 0.04, 1e-12);      // 요는 가중
  EXPECT_DOUBLE_EQ(f.gap, 0.16);                            // 틈은 라이다
  EXPECT_FALSE(f.disagree);
  lid.lateral = 0.0301;
  EXPECT_TRUE(fuse_measure(cam, lid, c).disagree);
  lid.gap = 0.13;                                           // 라이다 너무 가까움 → 안 씀
  f = fuse_measure(cam, lid, c);
  EXPECT_FALSE(f.lidar_used);
  EXPECT_FALSE(f.disagree);
  EXPECT_DOUBLE_EQ(f.yaw, 0.02);
  lid = {true, 0.02, 0.3, 0.2};                             // 요가 4° 넘게 다름 → 요는 섞지 않음
  EXPECT_DOUBLE_EQ(fuse_measure(cam, lid, c).yaw, 0.02);
  cam.ok = false;                                           // 주행 모드: 라이다만
  lid = {true, 0.02, 0.01, 0.2};
  f = fuse_measure(cam, lid, c);
  EXPECT_TRUE(f.ok);
  EXPECT_STREQ(f.src, "lidar");
  EXPECT_DOUBLE_EQ(f.lateral, 0.02);
  lid.ok = false;
  EXPECT_FALSE(fuse_measure(cam, lid, c).ok);
}

// 반복 폐루프 vs 열린 고리: 회전마다 미끄럼 ±3 mm · 회전 끝 yaw ±1° · 측정 σ 3 mm (옆) · 0.4° (요)
//   초기 옆 ±4 cm, 앞 틈 0.13 (FINE 자리), 뒤 여유 0.15. 기대: 폐루프 p95 |e| < 8 mm, 기동 ≤ 3
TEST(BoxCenterLoop, MonteCarloConverges)
{
  BoxCenterConfig bc;
  bc.max_alpha = 0.436; bc.back_min = 0.04;   // drive.yaml 11-21 값
  LoopConfig lc;
  std::mt19937 rng(20261121);
  std::uniform_real_distribution<double> U(-1.0, 1.0);
  std::normal_distribution<double> N(0.0, 1.0);
  const int runs = 2000;
  std::vector<double> closed, open;
  int fails = 0, max_k = 0;
  for (int r = 0; r < runs; ++r) {
    const double e0 = 0.04 * U(rng);
    // 열린 고리: 한 번 재고 한 번 기동
    auto once = [&](double e_true, double meas) {
        CenterError ce;
        ce.lateral = meas;
        const auto m = plan_center_move(ce, bc, 0.13, 0.15);
        if (!m.ok) {return e_true;}
        const auto q = execute(m.p, bc.icr_x, 1.0 * kDeg * U(rng), 1.0 * kDeg * U(rng), 0.003 * U(rng), 0.003 * U(rng));
        return e_true + q.y;
      };
    open.push_back(std::fabs(once(e0, e0 + 0.003 * N(rng))));
    // 폐루프 (mission center_box 반복)
    double e = e0;
    int k = 0;
    for (;;) {
      const double meas = e + 0.003 * N(rng), psi = 0.4 * kDeg * N(rng);
      const auto v = decide_loop(meas, psi, k, lc);
      if (v.d == LoopDecision::FAIL) {++fails; break;}
      if (v.d != LoopDecision::SHIFT) {break;}
      if (std::fabs(meas) > 0.005) {e = once(e, meas);}   // drive lateral_tolerance 0.005 (안이면 법선만)
      ++k;
    }
    max_k = std::max(max_k, k);
    closed.push_back(std::fabs(e));
  }
  auto p95 = [](std::vector<double> v) {std::sort(v.begin(), v.end()); return v[static_cast<size_t>(0.95 * v.size())];};
  const double pc = p95(closed), po = p95(open);
  std::printf("몬테카를로 %d 회: 폐루프 p95 %.1f mm (FAIL %d, 최대 기동 %d) · 열린 고리 p95 %.1f mm\n",
    runs, pc * 1000, fails, max_k, po * 1000);
  EXPECT_LT(pc, 0.008);
  EXPECT_LE(max_k, lc.max_iter);
  EXPECT_LT(fails, runs / 50);   // 2 % 미만
  EXPECT_LT(pc, po);
}
