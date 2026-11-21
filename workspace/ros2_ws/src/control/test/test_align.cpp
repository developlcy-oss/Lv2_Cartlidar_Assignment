// /drive/align 제어 법칙 시험: 부호, 포화, 정착 판정, 간단한 폐루프 시뮬레이션
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "control/align_controller.hpp"

using namespace control;

TEST(Align, SignsAndSaturation)
{
  AlignController c;
  auto o = c.step(0.5, 0.40, 0.15, 0.01);   // 물체가 오른쪽 · 멀리
  EXPECT_LT(o.w, 0.0);                       // 오른쪽(시계 방향)으로 회전
  EXPECT_GE(o.w, -0.6);
  EXPECT_GT(o.v, 0.0);                       // 앞으로
  EXPECT_LE(o.v, 0.08);
  o = c.step(-0.5, 0.12, 0.15, 0.01);       // 왼쪽 · 너무 가까움
  EXPECT_GT(o.w, 0.0);
  EXPECT_LT(o.v, 0.0);
  EXPECT_FALSE(o.done);
}

TEST(Align, SettlesOnlyAfterConsecutiveCycles)
{
  AlignGains g;
  g.settle_cycles = 5;
  AlignController c(g);
  for (int i = 0; i < 4; ++i) {EXPECT_FALSE(c.step(0.0, 0.15, 0.15, 0.01).done);}
  EXPECT_FALSE(c.step(0.2, 0.15, 0.15, 0.01).done);   // 한 번 벗어나면 다시 셈
  for (int i = 0; i < 4; ++i) {EXPECT_FALSE(c.step(0.0, 0.151, 0.15, 0.01).done);}
  const auto o = c.step(0.0, 0.15, 0.15, 0.01);
  EXPECT_TRUE(o.done);
  EXPECT_DOUBLE_EQ(o.v, 0.0);
  EXPECT_DOUBLE_EQ(o.w, 0.0);
}

TEST(Align, ClosedLoopConverges)
{
  // 단순 모델: 가로 오차는 회전에 비례해 줄고(화면 0.7 rad 폭), 거리는 전진 속도로 줄어듦
  AlignController c;
  double lateral = 0.6, dist = 0.45;
  const double dt = 0.05;
  bool done = false;
  for (int k = 0; k < 400 && !done; ++k) {
    const auto o = c.step(lateral, dist, 0.15, 0.01);
    done = o.done;
    lateral += o.w * dt / 0.7 * 2.0;   // w<0(오른쪽) → 오른쪽 오차 감소
    dist -= o.v * dt;
  }
  EXPECT_TRUE(done);
  EXPECT_NEAR(dist, 0.15, 0.01);
  EXPECT_NEAR(lateral, 0.0, 0.04);
}

// 박스 접근 (#160): 퍽 없이 시작 yaw 유지 + TFmini 거리
TEST(BoxApproach, HoldsYawAndDrivesToBoxDistance)
{
  BoxApproach c;
  auto o = c.step(0.1, 0.45, 0.13, 0.01);       // 오른쪽으로 0.1 rad 틀어짐 (목표가 왼쪽) · 0.45 m
  EXPECT_GT(o.w, 0.0);                          // 왼쪽으로 돌고
  EXPECT_GT(o.v, 0.0);
  EXPECT_LT(o.v, 0.08 * 0.5);                   // 많이 틀어졌으면 천천히
  o = c.step(0.0, 0.45, 0.13, 0.01);
  EXPECT_DOUBLE_EQ(o.w, 0.0);
  EXPECT_DOUBLE_EQ(o.v, 0.08);                  // 최고 속도로 자름
  o = c.step(0.0, 0.10, 0.13, 0.01);            // 지나침 → 후진
  EXPECT_LT(o.v, 0.0);
}

TEST(BoxApproach, ClosedLoopConverges)
{
  BoxApproach c;
  double yaw_err = -0.08, dist = 0.45;
  const double dt = 0.05;
  bool done = false;
  for (int k = 0; k < 400 && !done; ++k) {
    const auto o = c.step(yaw_err, dist, 0.13, 0.01);
    done = o.done;
    yaw_err -= o.w * dt;   // 돌면 남은 오차가 줄어듦
    dist -= o.v * dt;
  }
  EXPECT_TRUE(done);
  EXPECT_NEAR(dist, 0.13, 0.01);
  EXPECT_NEAR(yaw_err, 0.0, 0.02);
}

TEST(CoarseAlign, TurnsFirstThenDrivesToBumperDistance)
{
  CoarseAlign c;
  auto o = c.step(1.0, 0.8, 0.2, 0.4, 0.02);   // 물체가 왼쪽 38°
  EXPECT_GT(o.w, 0.0);
  EXPECT_DOUBLE_EQ(o.v, 0.0);                   // 각도가 크면 제자리 회전 먼저
  o = c.step(1.0, 0.0, 0.2, 0.4, 0.02);         // 정면 · 범퍼 기준 0.8 m → 0.4 m 까지 전진
  EXPECT_GT(o.v, 0.0);
  o = c.step(0.5, 0.0, 0.2, 0.4, 0.02);         // 범퍼 기준 0.3 m (더 가까움) → 후진
  EXPECT_LT(o.v, 0.0);
  CoarseAlign d;
  bool done = false;
  for (int i = 0; i < 10 && !done; ++i) {done = d.step(0.6, 0.0, 0.2, 0.4, 0.02).done;}
  EXPECT_TRUE(done);
}

TEST(ScanSafety, FrontAndRearSectors)
{
  // 360 개, angle_min -π, 1° 간격
  std::vector<float> r(360, 5.0f);
  const double amin = -M_PI, inc = M_PI / 180;
  EXPECT_FALSE(scan_blocked(r, amin, inc, 0.0, 0.349, 0.9));
  r[180 + 10] = 0.5f;                             // 전방 +10° 에 0.5 m
  EXPECT_TRUE(scan_blocked(r, amin, inc, 0.0, 0.349, 0.9));
  EXPECT_FALSE(scan_blocked(r, amin, inc, M_PI, 0.349, 0.3));   // 후방은 비어 있음
  r[5] = 0.2f;                                    // 후방 (-175°) 0.2 m
  EXPECT_TRUE(scan_blocked(r, amin, inc, M_PI, 0.349, 0.3));
}

// 박스 앞면 거리 다듬기: 중앙값, 퍼짐 · 무효 · 오래된 값
TEST(BoxRange, MedianOfFreshValid)
{
  BoxRangeFilter f;   // window 5, min 3, spread 0.03, age 0.3
  EXPECT_FALSE(f.value(0.0));
  f.push(0.00, 0.200);
  f.push(0.02, 0.210);
  EXPECT_FALSE(f.value(0.02));                   // 2개 < 3
  f.push(0.04, 0.205);
  ASSERT_TRUE(f.value(0.04));
  EXPECT_NEAR(*f.value(0.04), 0.205, 1e-9);
  f.push(0.06, 0.202);
  EXPECT_NEAR(*f.value(0.06), 0.2035, 1e-9);     // 짝수 개 → 가운데 둘 평균
  EXPECT_FALSE(f.value(0.40));                   // 모두 300 ms 넘음
}

TEST(BoxRange, EdgeFlickerAndInvalidRejected)
{
  BoxRangeFilter f;
  for (double t : {0.00, 0.02, 0.04}) {f.push(t, 0.20);}
  f.push(0.06, 0.85);                            // 빔이 박스 옆으로 → 뒤쪽 먼 곳
  EXPECT_FALSE(f.value(0.06));
  BoxRangeFilter g;
  g.push(0.00, 0.20);
  g.push(0.02, std::nullopt);
  g.push(0.04, std::nullopt);
  g.push(0.06, std::nullopt);
  EXPECT_FALSE(g.value(0.06));                   // 무효가 섞여 유효 1개
}

TEST(BoxRange, ApproachStopDistance)
{
  EXPECT_NEAR(box_approach_stop(0.15, 0.03, 0.10), 0.12, 1e-9);
  EXPECT_NEAR(box_approach_stop(0.11, 0.03, 0.10), 0.10, 1e-9);   // TFmini 최소 거리 아래로는 안 내림
}

TEST(RearCamera, BlocksOnlyReverseWhenBlockedOrMissing)
{
  EXPECT_TRUE(rear_camera_blocks(-0.05, true, true));            // 후진 + 막힘
  EXPECT_TRUE(rear_camera_blocks(-0.05, true, std::nullopt));    // 후진 + 값 없음 · 오래됨 → 안전 쪽
  EXPECT_FALSE(rear_camera_blocks(-0.05, true, false));          // 후진 + 비어 있음
  EXPECT_FALSE(rear_camera_blocks(0.05, true, true));            // 전진은 안 막음
  EXPECT_FALSE(rear_camera_blocks(0.0, true, true));             // 제자리 (회전) 는 안 막음
  EXPECT_FALSE(rear_camera_blocks(-0.05, false, true));          // 꺼져 있으면 안 봄
}

// ---------- 상자 중앙 맞춤 (box_center.hpp) ----------
#include "control/box_center.hpp"

TEST(BoxCenter, FacingYawPicksNearest90)
{
  EXPECT_NEAR(box_facing_yaw(0.1, 0.0), 0.0, 1e-9);
  EXPECT_NEAR(box_facing_yaw(0.1, M_PI / 2), 0.0, 1e-9);       // 정사각형: 90° 돌린 요도 같은 법선
  EXPECT_NEAR(box_facing_yaw(1.5, 0.05), M_PI / 2 + 0.05, 1e-9);
  EXPECT_NEAR(std::fabs(box_facing_yaw(3.1, 0.0)), M_PI, 1e-9);
}

TEST(BoxCenter, ErrorSigns)
{
  // 상자 (0.4, 0) 요 0, 로봇 (0, 0.03) yaw 0.1 → 로봇이 중심선 왼쪽 3 cm, 법선 0 으로 오른쪽 회전 필요
  const auto e = center_error(0.0, 0.03, 0.1, 0.4, 0.0, 0.0);
  EXPECT_NEAR(e.lateral, 0.03, 1e-9);
  EXPECT_NEAR(e.yaw_err, -0.1, 1e-9);
  EXPECT_NEAR(e.depth, -0.4, 1e-9);
  EXPECT_NEAR(e.heading, 0.0, 1e-9);
  // 같은 배치를 90° 돌린 odom: 결과 같음
  const auto r = center_error(-0.03, 0.0, M_PI / 2 + 0.1, 0.0, 0.4, M_PI / 2);
  EXPECT_NEAR(r.lateral, 0.03, 1e-9);
  EXPECT_NEAR(r.depth, -0.4, 1e-9);
}

TEST(BoxCenter, Decide)
{
  BoxCenterConfig c;
  CenterError e;
  e.depth = -0.4;
  e.lateral = 0.005;
  EXPECT_EQ(decide_center(e, c), CenterDecision::DONE);
  e.lateral = -0.03;
  EXPECT_EQ(decide_center(e, c), CenterDecision::SHIFT);
  e.lateral = 0.12;
  EXPECT_EQ(decide_center(e, c), CenterDecision::TOO_FAR);
  e.lateral = 0.03;
  e.depth = 0.05;
  EXPECT_EQ(decide_center(e, c), CenterDecision::BEHIND);
}

TEST(BoxCenter, PlanGeometry)
{
  BoxCenterConfig c;
  CenterError e;
  e.lateral = 0.03;   // 왼쪽 → 오른쪽으로 0.03
  const auto p = plan_shift(e, c);
  EXPECT_LT(p.alpha, 0.0);
  EXPECT_GE(p.back, c.back_min);
  EXPECT_LE(p.back, c.back_max);
  EXPECT_NEAR(p.fwd * std::cos(p.alpha), p.back, 1e-9);        // 깊이 되돌림
  EXPECT_NEAR(p.fwd * std::sin(p.alpha), -0.03, 1e-9);         // 옆 이동
  e.lateral = -0.08;
  const auto q = plan_shift(e, c);
  EXPECT_NEAR(q.back, c.back_max, 1e-9);                       // 상한에 걸림 → 각이 커짐
  EXPECT_GT(q.alpha, c.max_alpha);
}

// 단순 차동 구동 적분으로 기동 폐루프: 끝나면 옆 어긋남 · 요 · 깊이가 돌아옴
TEST(BoxCenter, ManeuverClosedLoop)
{
  BoxCenterConfig c;
  const double bx = 0.40, by = 0.0, byaw = 0.0;
  for (double y0 : {0.04, -0.025, 0.06}) {
    double x = 0.0, y = y0, yaw = -0.08;   // FINE 이 퍽 쪽으로 돌아 비스듬히 끝난 상태
    const auto e0 = center_error(x, y, yaw, bx, by, byaw);
    ASSERT_EQ(decide_center(e0, c), CenterDecision::SHIFT);
    ShiftManeuver m(c);
    m.start(plan_shift(e0, c));
    const double dt = 0.05;
    double min_x = x;
    int n = 0;
    for (; n < 2000; ++n) {
      const auto o = m.step(x, y, yaw);
      if (o.done) {break;}
      EXPECT_LE(std::fabs(o.v), c.speed + 1e-9);
      EXPECT_LE(std::fabs(o.w), c.max_angular + 1e-9);
      x += o.v * std::cos(yaw) * dt;
      y += o.v * std::sin(yaw) * dt;
      yaw += o.w * dt;
      min_x = std::min(min_x, x);
    }
    ASSERT_LT(n, 2000);
    const auto e1 = center_error(x, y, yaw, bx, by, byaw);
    EXPECT_LT(std::fabs(e1.lateral), 0.006) << "y0 " << y0;
    EXPECT_LT(std::fabs(e1.yaw_err), c.yaw_tol + 0.01);
    EXPECT_NEAR(e1.depth, e0.depth, 0.01);                       // 깊이는 대체로 그대로
    EXPECT_GE(min_x, -c.back_max - 0.01);                        // 후진은 back_max 안
  }
}

TEST(BoxCenter, CutForwardSkipsToFinalTurn)
{
  BoxCenterConfig c;
  CenterError e;
  e.heading = 0.0;
  e.lateral = 0.04;
  ShiftManeuver m(c);
  m.start(plan_shift(e, c));
  double x = 0, y = 0, yaw = 0;
  for (int i = 0; i < 2000 && !m.forward(); ++i) {
    const auto o = m.step(x, y, yaw);
    x += o.v * std::cos(yaw) * 0.05;
    y += o.v * std::sin(yaw) * 0.05;
    yaw += o.w * 0.05;
  }
  ASSERT_TRUE(m.forward());
  m.cut_forward();
  EXPECT_EQ(m.phase(), ShiftManeuver::Phase::TURN2);
}

TEST(BoxCenter, RearRoomLimitsBack)
{
  BoxCenterConfig c;
  EXPECT_DOUBLE_EQ(rear_back_room(std::numeric_limits<double>::infinity(), 0.30, c), c.back_max);
  EXPECT_NEAR(rear_back_room(0.40, 0.30, c), 0.10, 1e-9);
  EXPECT_DOUBLE_EQ(rear_back_room(0.33, 0.30, c), 0.0);        // back_min 0.06 안 됨 → 기동 안 함
  CenterError e;
  e.lateral = -0.05;
  const auto p = plan_shift(e, c, 0.08);
  EXPECT_NEAR(p.back, 0.08, 1e-9);
  EXPECT_NEAR(p.fwd * std::sin(p.alpha), 0.05, 1e-9);          // 후진을 줄여도 옆 이동량은 그대로 (각이 커짐)
}

TEST(BoxCenter, PuckCheck)
{
  BoxCenterConfig c;   // 6°
  // 로봇 (0, 0) 법선 0, 상자 중심 (0.36, 0). 퍽이 중심선 왼쪽 2 cm (칸 하나 안) → 2.0° → 돌아도 됨
  auto k = puck_check(0.0, 0.0, 0.33, 0.02, 0.36, 0.0, 0.0, c);
  EXPECT_TRUE(k.rotate);
  EXPECT_NEAR(k.lateral, 0.02, 1e-9);
  EXPECT_GT(k.yaw_dev, 0.0);
  // 오른쪽 6 cm (칸 하나 어긋남) → 10° 넘음 → 돌지 않음
  k = puck_check(0.0, 0.0, 0.33, -0.06, 0.36, 0.0, 0.0, c);
  EXPECT_FALSE(k.rotate);
  EXPECT_NEAR(k.lateral, -0.06, 1e-9);
  // 90° 돌린 odom 에서도 같은 판단
  k = puck_check(0.0, 0.0, -0.02, 0.33, 0.0, 0.36, M_PI / 2, c);
  EXPECT_TRUE(k.rotate);
  EXPECT_NEAR(k.lateral, 0.02, 1e-9);
}
