// ArmDiffIK 시험: 기준값 = urdf_and_usd/policy/manipulator/pallet_traj.py Arm (URDF FK) + 같은 식의 numpy 감쇠 최소제곱
//   (야코비안은 수치 미분, 2026-11-05 계산). Isaac toppick_env._ik_step 과 같은 식이어야 정책이 학습 때처럼 움직인다.
#include <gtest/gtest.h>

#include <cmath>

#include "control/arm_diff_ik.hpp"

using control::ArmDiffIK;
using control::Joints5;

namespace
{
struct Case
{
  Joints5 q;
  Eigen::Vector3d dp;
  bool tilted;
  Eigen::Vector3d tcp;
  double rot[9];   // 행 우선
  Joints5 dq;
};

const Case kCases[] = {
  {{0.0, -0.462, 0.771, 0.476, 0.0}, {0.003, -0.002, -0.004}, true, {0.297397634, 0.0, 0.15932279},
    {0.707388269, 0.0, 0.706825181, 0.0, 1.0, 0.0, -0.706825181, 0.0, 0.707388269},
    {-0.004697499, 0.037364808, 0.006807887, -0.043705576, -0.003316626}},
  {{0.3, 0.48, -0.76, 1.84, 0.2}, {0.0, 0.005, 0.0}, false, {0.288385886, 0.059511929, 0.159230541},
    {0.010313925, -0.099844478, 0.994949598, 0.003190471, 0.995000744, 0.099816537, -0.99994172, 0.002144857, 0.010580913},
    {0.02410965, 0.037319576, -0.061479908, 0.034423111, -0.174361562}},
  {{-0.5, -1.32032, 0.67766, 1.42805, -0.1}, {0.002, 0.002, 0.002}, true, {0.183758723, -0.047942809, 0.201001411},
    {0.620549646, 0.415079828, 0.665302091, -0.339007817, 0.907042061, -0.249696617, -0.707101009, -0.070593462, 0.703579936},
    {0.004792782, 0.015767813, -0.01687945, 0.001076529, 0.103107831}},
};
}  // namespace

TEST(ArmDiffIK, ForwardMatchesUrdf)
{
  ArmDiffIK ik;
  for (const auto & c : kCases) {
    const auto f = ik.forward(c.q);
    EXPECT_LT((f.tcp - c.tcp).norm(), 1e-8);
    for (int i = 0; i < 9; ++i) {EXPECT_NEAR(f.rot(i / 3, i % 3), c.rot[i], 1e-8);}
  }
}

TEST(ArmDiffIK, JacobianMatchesFiniteDifference)
{
  ArmDiffIK ik;
  const Joints5 q{0.2, -0.3, 0.5, 1.0, 0.4};
  const auto j = ik.jacobian(q);
  const auto f0 = ik.forward(q);
  for (int k = 0; k < 5; ++k) {
    Joints5 qq = q;
    qq[k] += 1e-7;
    const auto f1 = ik.forward(qq);
    EXPECT_LT(((f1.tcp - f0.tcp) / 1e-7 - j.block<3, 1>(0, k)).norm(), 1e-5);
    const Eigen::Matrix3d w = (f1.rot - f0.rot) / 1e-7 * f0.rot.transpose();
    EXPECT_LT((Eigen::Vector3d(w(2, 1), w(0, 2), w(1, 0)) - j.block<3, 1>(3, k)).norm(), 1e-5);
  }
}

TEST(ArmDiffIK, StepMatchesIsaacFormula)
{
  ArmDiffIK ik;
  for (const auto & c : kCases) {
    const auto dq = ik.step(c.q, c.dp, c.tilted);
    for (int k = 0; k < 5; ++k) {EXPECT_NEAR(dq[k], c.dq[k], 1e-6) << "joint " << k + 1;}
  }
}

TEST(ArmDiffIK, ReadyTiltHoldsAndConvergesTo45Deg)
{
  // READY_TILT (학습 기울임 시작 자세) 에서 변위 0 → 자세 오차 ≈ 0 이라 거의 안 움직임. 목표까지 되풀이 → 위치 · 45° 맞춤
  ArmDiffIK ik;
  Joints5 q{0.0, -0.462, 0.771, 0.476, 0.0};
  const auto d0 = ik.step(q, Eigen::Vector3d::Zero(), true);
  for (double v : d0) {EXPECT_LT(std::abs(v), 2e-3);}
  const Eigen::Vector3d goal = ik.forward(q).tcp + Eigen::Vector3d(-0.02, 0.04, -0.03);
  for (int i = 0; i < 300; ++i) {
    const Eigen::Vector3d e = goal - ik.forward(q).tcp;
    const Eigen::Vector3d a = (e / ik.params().cart_scale).cwiseMax(-1.0).cwiseMin(1.0);
    q = ik.applyAction(q, q, a, true);
  }
  const auto f = ik.forward(q);
  EXPECT_LT((f.tcp - goal).norm(), 1e-3);
  const Eigen::Vector3d y = ik.tangentCloseDir(f.tcp);
  EXPECT_GT(f.approach().dot(ik.approachTarget(y, f.tcp, true)), std::cos(2.0 * M_PI / 180.0));
  EXPECT_LT(std::abs(ik.rollError(q)), 2.0 * M_PI / 180.0);
}

TEST(ArmDiffIK, ApplyActionClampsLeadAndLimits)
{
  ArmDiffIK ik;
  const Joints5 q{0.0, 1.59, 0.0, 1.0, 0.0};
  Joints5 far = q;
  far[2] = q[2] + 1.0;   // 직전 목표가 측정보다 1 rad 앞섬 → ±0.25 로 잘림
  const auto t = ik.applyAction(far, q, Eigen::Vector3d(5.0, 0.0, 0.0), false);   // 행동은 ±1 로 잘림
  EXPECT_LE(t[2], q[2] + ik.params().lead_max + 1e-12);
  for (int k = 0; k < 5; ++k) {
    EXPECT_GE(t[k], ik.params().lower[k]);
    EXPECT_LE(t[k], ik.params().upper[k]);
  }
}
