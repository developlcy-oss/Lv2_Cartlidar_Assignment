// 누운 직육면체 세워 놓기 정책 (lieupright.hpp) — 학습 환경 lieupright_env.py 식으로 낸 기준값과 비교
//   기준값: lv2_next_module 261111_1300_lie_skill/gen_ref.py (target_R_base · _stand_spot 을 torch 로 그대로 옮김)
#include <gtest/gtest.h>

#include <cmath>
#include <numeric>

#include <Eigen/Geometry>

#include "control/lieupright.hpp"

using control::ArmDiffIK;
using control::Joints5;
using control::LieUprightParams;

namespace
{
Eigen::Matrix3d rowMajor(const double (&v)[9])
{
  Eigen::Matrix3d m;
  for (int i = 0; i < 9; ++i) {m(i / 3, i % 3) = v[i];}
  return m;
}

double rotErr(const Eigen::Matrix3d & a, const Eigen::Matrix3d & b)
{
  return Eigen::AngleAxisd(a * b.transpose()).angle();
}
}  // namespace

TEST(LieUpright, TargetRotationMatchesTrainingEnv)
{
  const ArmDiffIK ik;
  struct Case {Eigen::Vector3d g; double beta, rho; double r[9];};
  const Case cases[] = {
    {{0.376, 0.0, 0.12}, M_PI / 4, 0.0,
      {0.707106781, 0.000000000, 0.707106781, 0.000000000, 1.000000000, -0.000000000, -0.707106781, 0.000000000, 0.707106781}},
    {{0.33, -0.09, 0.15}, 1.2, 0.7,
      {0.869914586, 0.492439999, 0.027412793, -0.334582533, 0.630062997, -0.700767542, -0.362357754, 0.600436064, 0.712862813}},
    {{0.30, 0.11, 0.10}, -0.1, -2.0,
      {-0.087872760, -0.598850301, -0.796025562, -0.047382371, -0.795699455, 0.603835481, -0.995004165, 0.090778269, 0.041545361}},
  };
  for (const auto & c : cases) {
    const Eigen::Matrix3d r = control::lieTargetRotation(ik, c.g, c.beta, c.rho);
    EXPECT_LT((r - rowMajor(c.r)).cwiseAbs().maxCoeff(), 1e-8);
    EXPECT_NEAR(r.determinant(), 1.0, 1e-9);
  }
}

TEST(LieUpright, StandSpotMatchesTrainingEnv)
{
  const LieUprightParams p;
  struct Case {Eigen::Vector2d bc; double yaw; Eigen::Vector2d nb, d;};
  const Case cases[] = {
    {{0.344775094, -0.118181151}, -0.297742381, {0.333040587, -0.156421200}, {0.379905905, -0.053124821}},
    {{0.344775094, 0.118181151}, 0.297742381, {0.333040587, 0.156421200}, {0.379905905, 0.053124821}},
    {{0.362500000, 0.000000000}, 0.000000000, {0.402500000, 0.000000000}, {0.377000000, 0.072500000}},
    {{0.350000000, 0.020000000}, 0.050000000, {0.330000000, 0.050000000}, {0.381862549, -0.036478119}},
  };
  for (const auto & c : cases) {
    const Eigen::Vector2d d = control::lieStandSpot(p, c.bc, c.yaw, c.nb, 0.096);
    EXPECT_NEAR(d.x(), c.d.x(), 1e-7);
    EXPECT_NEAR(d.y(), c.d.y(), 1e-7);
  }
}

TEST(LieUpright, ZeroActionHoldsStartPose)
{
  // 시작 자세 (READY_POSE_45) 는 β = π/4 · ρ = 0 목표와 맞아서, 행동 0 이면 거의 움직이지 않음
  const ArmDiffIK ik;
  const LieUprightParams p;
  Joints5 q = control::kLieReadyPose45;
  double beta = p.beta0, rho = 0.0;
  const auto f0 = ik.forward(q);
  EXPECT_LT(rotErr(control::lieTargetRotation(ik, f0.tcp, beta, rho), f0.rot), 0.02);
  for (int i = 0; i < 30; ++i) {q = control::lieApplyAction(ik, p, q, q, {0.f, 0.f, 0.f, 0.f, 0.f, 1.f}, beta, rho);}
  EXPECT_LT((ik.forward(q).tcp - f0.tcp).norm(), 0.002);
  EXPECT_DOUBLE_EQ(beta, p.beta0);
}

TEST(LieUpright, ActionMovesGraspPointAndTiltsGripper)
{
  const ArmDiffIK ik;
  const LieUprightParams p;
  // 잡는 점 x 한 스텝 = cart_scale (5 mm) 쯤
  {
    Joints5 q = control::kLieReadyPose45;
    double beta = p.beta0, rho = 0.0;
    for (int i = 0; i < 5; ++i) {q = control::lieApplyAction(ik, p, q, q, {0.f, 0.f, 0.f, 0.f, 0.f, 1.f}, beta, rho);}
    const auto g0 = ik.forward(q).tcp;
    const Joints5 q1 = control::lieApplyAction(ik, p, q, q, {1.f, 0.f, 0.f, 0.f, 0.f, 1.f}, beta, rho);
    const Eigen::Vector3d d = ik.forward(q1).tcp - g0;
    EXPECT_NEAR(d.x(), 0.005, 0.0007);
    EXPECT_LT(std::hypot(d.y(), d.z()), 0.0007);
  }
  // β 를 0.5 rad 올리고 (10 스텝) 몇 스텝 더 두면 집게 방향이 목표에 맞음. 행동 크기는 [-1, 1] 로 자름
  {
    Joints5 q = control::kLieReadyPose45;
    double beta = p.beta0, rho = 0.0;
    for (int i = 0; i < 10; ++i) {q = control::lieApplyAction(ik, p, q, q, {0.f, 0.f, 0.f, 3.f, 0.f, 1.f}, beta, rho);}
    EXPECT_NEAR(beta, p.beta0 + 0.5, 1e-9);
    for (int i = 0; i < 30; ++i) {q = control::lieApplyAction(ik, p, q, q, {0.f, 0.f, 0.f, 0.f, 0.f, 1.f}, beta, rho);}
    const auto f = ik.forward(q);
    EXPECT_LT(rotErr(control::lieTargetRotation(ik, f.tcp, beta, rho), f.rot), 0.01);
  }
  // β · ρ 는 범위로 자름
  {
    Joints5 q = control::kLieReadyPose45;
    double beta = p.beta0, rho = 0.0;
    for (int i = 0; i < 100; ++i) {control::lieApplyAction(ik, p, q, q, {0.f, 0.f, 0.f, 1.f, -1.f, 1.f}, beta, rho);}
    EXPECT_DOUBLE_EQ(beta, p.beta_max);
    EXPECT_DOUBLE_EQ(rho, -p.rho_max);
  }
}

TEST(LieUpright, TargetLeadAndJointLimits)
{
  const ArmDiffIK ik;
  const LieUprightParams p;
  const Joints5 q = control::kLieReadyPose45;
  Joints5 far = q;
  far[1] += 1.0;   // 직전 목표가 측정보다 1 rad 앞섬 → 측정 ± 0.25 로 자름
  double beta = p.beta0, rho = 0.0;
  const Joints5 t = control::lieApplyAction(ik, p, far, q, {0.f, 0.f, 0.f, 0.f, 0.f, 1.f}, beta, rho);
  EXPECT_LE(t[1], q[1] + p.lead_max + 1e-12);
  for (size_t k = 0; k < 5; ++k) {
    EXPECT_GE(t[k], ik.params().lower[k]);
    EXPECT_LE(t[k], ik.params().upper[k]);
  }
}

TEST(LieUpright, HoldHysteresis)
{
  const LieUprightParams p;
  control::LieHoldFilter h;
  for (int i = 0; i < p.hold_on - 1; ++i) {EXPECT_FALSE(h.update(-0.006, p));}
  EXPECT_TRUE(h.update(-0.006, p));            // 연속 8 스텝
  EXPECT_TRUE(h.update(-0.011, p));            // 끝까지 닫힘 (빈손) 1 · 2 스텝은 아직 쥠
  EXPECT_TRUE(h.update(-0.011, p));
  EXPECT_FALSE(h.update(-0.011, p));           // 연속 3 스텝 → 놓음
  EXPECT_FALSE(h.update(0.010, p));            // 벌림
  for (int i = 0; i < 7; ++i) {h.update(-0.005, p);}
  EXPECT_FALSE(h.update(0.010, p));            // 7 스텝 뒤 끊기면 처음부터
}

TEST(LieUpright, ObservationLayout)
{
  const ArmDiffIK ik;
  control::LieUprightState s;
  s.q = control::kLieReadyPose45;
  s.puck = {0.33, -0.08, 0.07825};
  s.long_yaw = 0.3;
  s.spot = {0.38, -0.05, 0.09525};
  s.box_c = {0.3448, -0.1182};
  s.box_yaw = -0.2977;
  const auto o = control::lieUprightObservation(ik, s);
  const auto & sizes = control::lieUprightObservationSizes();
  int total = 0;
  for (const auto & [name, n] : sizes) {
    ASSERT_TRUE(o.count(name)) << name;
    EXPECT_EQ(static_cast<int>(o.at(name).size()), n) << name;
    total += n;
  }
  EXPECT_EQ(total, 45);
  EXPECT_FLOAT_EQ(o.at("joint_pos_rel")[1], 0.f);
  EXPECT_FLOAT_EQ(o.at("long_axis_cos_sin2")[0], static_cast<float>(std::cos(0.6)));
  EXPECT_FLOAT_EQ(o.at("neighbor_present")[0], 0.f);
  EXPECT_FLOAT_EQ(o.at("rho_cos_sin_beta")[2], static_cast<float>(M_PI / 4));
  EXPECT_FLOAT_EQ(o.at("prev_action")[5], 1.f);
  // 옆 퍽: 인식 퍽 기준 상대 xy
  s.neighbor = Eigen::Vector2d(0.32, -0.15);
  const auto o2 = control::lieUprightObservation(ik, s);
  EXPECT_NEAR(o2.at("neighbor_rel")[0], -0.01, 1e-6);
  EXPECT_NEAR(o2.at("neighbor_rel")[1], -0.07, 1e-6);
  EXPECT_FLOAT_EQ(o2.at("neighbor_present")[0], 1.f);
}
