// toppick_obs 시험: 기준값 = Isaac toppick_env._get_observations 식을 pallet_traj.Arm (URDF FK) 로 계산 (2026-11-05)
#include <gtest/gtest.h>

#include <cmath>

#include "control/toppick_obs.hpp"

using namespace control;

TEST(TopPickObs, MatchesIsaacFormula)
{
  ArmDiffIK ik;
  TopPickState s;
  s.q = {0.2, -0.5, 0.8, 0.5, 0.1};
  s.qd = {0.1, -0.2, 0.3, -0.4, 0.5};
  s.finger_m = -0.006;
  s.puck = {0.30, 0.05, 0.10};
  s.prev_action = {0.1f, -0.2f, 0.3f, -1.f};
  const auto o = topPickObservation(ik, s);

  int total = 0;
  for (const auto & kv : topPickObservationSizes()) {
    ASSERT_EQ(o.count(kv.first), 1u) << kv.first;
    EXPECT_EQ(static_cast<int>(o.at(kv.first).size()), kv.second) << kv.first;
    total += kv.second;
  }
  EXPECT_EQ(total, 36);

  const double jp[5] = {0.2, -0.98, 1.56, -1.34, 0.1};
  for (int k = 0; k < 5; ++k) {
    EXPECT_NEAR(o.at("joint_pos_rel")[k], jp[k], 1e-6);
    EXPECT_NEAR(o.at("joint_vel")[k], s.qd[k], 1e-6);
  }
  const double grasp[3] = {0.287312881, 0.038781041, 0.157176452};
  const double pmg[3] = {0.012687119, 0.011218959, -0.057176452};
  const double appr[3] = {0.68281896, 0.138414256, -0.717356091};
  const double close[3] = {-0.127488256, 0.989398252, 0.069554611};
  for (int k = 0; k < 3; ++k) {
    EXPECT_NEAR(o.at("grasp_pos")[k], grasp[k], 1e-6);
    EXPECT_NEAR(o.at("puck_minus_grasp")[k], pmg[k], 1e-6);
    EXPECT_NEAR(o.at("approach_axis")[k], appr[k], 1e-6);
    EXPECT_NEAR(o.at("close_axis")[k], close[k], 1e-6);
  }
  EXPECT_NEAR(o.at("roll_cos_sin")[0], 0.9950041652780258, 1e-6);
  EXPECT_NEAR(o.at("roll_cos_sin")[1], 0.09983341664682809, 1e-6);
  // 옆 퍽 없음 = NB_ABSENT, 원기둥, 기울임 모드
  EXPECT_FLOAT_EQ(o.at("neighbor_rel")[0], 0.0f);
  EXPECT_FLOAT_EQ(o.at("neighbor_rel")[1], 0.5f);
  EXPECT_FLOAT_EQ(o.at("is_box")[0], 0.0f);
  EXPECT_FLOAT_EQ(o.at("tilted")[0], 1.0f);
  EXPECT_FLOAT_EQ(o.at("prev_action")[3], -1.0f);

  s.neighbor = Eigen::Vector3d(0.30, 0.13, 0.10);
  s.is_box = true;
  const auto o2 = topPickObservation(ik, s);
  EXPECT_NEAR(o2.at("neighbor_rel")[0], 0.0, 1e-6);
  EXPECT_NEAR(o2.at("neighbor_rel")[1], 0.08, 1e-6);
  EXPECT_FLOAT_EQ(o2.at("is_box")[0], 1.0f);
}

TEST(TopPickObs, GripperMapEndpointsAndInverse)
{
  GripperMap g;
  EXPECT_NEAR(g.toMeters(g.rad_open), 0.020, 1e-12);
  EXPECT_NEAR(g.toMeters(g.rad_close), -0.011, 1e-12);
  for (double m : {0.010, -0.0075, -0.003}) {EXPECT_NEAR(g.toMeters(g.toRad(m)), m, 1e-12);}
  EXPECT_GT(g.toRad(-0.0075), g.toRad(0.010));   // 닫기 = 큰 rad (실물 닫힘 +)
}
