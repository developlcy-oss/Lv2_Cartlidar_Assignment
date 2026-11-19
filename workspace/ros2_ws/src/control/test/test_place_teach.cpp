#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <functional>

#include "control/arm_ik.hpp"
#include "control/place_teach.hpp"

using control::PlaceTeachPoint;
using control::placeTeachOffset;

namespace
{
std::vector<PlaceTeachPoint> grid(const std::function<Eigen::Vector3d(double, double)> & f, double cx = 0.189)
{
  std::vector<PlaceTeachPoint> pts;
  for (int i = -1; i <= 1; ++i) {
    for (int k = -1; k <= 1; ++k) {
      PlaceTeachPoint p;
      p.tip = Eigen::Vector3d(cx + 0.06 * k, 0.06 * i, -0.005);
      p.err = f(p.tip.x(), p.tip.y());
      pts.push_back(p);
    }
  }
  return pts;
}
}  // namespace

TEST(PlaceTeach, EmptyGivesZero)
{
  const auto d = placeTeachOffset({}, Eigen::Vector3d(0.2, 0.0, 0.1));
  EXPECT_DOUBLE_EQ(d.err.norm(), 0.0);
  EXPECT_TRUE(std::isinf(d.nearest));
}

TEST(PlaceTeach, LinearFieldReproducedInsideAndOutside)
{
  // 오차가 자리의 1 차식이면 칸 위 · 칸 사이 · 칸 밖 어디서나 그대로 (9 칸 3 x 3)
  auto f = [](double x, double y) {return Eigen::Vector3d(0.002 + 0.05 * (x - 0.189), -0.001 + 0.03 * y, -0.004 + 0.02 * x);};
  const auto pts = grid(f);
  for (double x : {0.12, 0.19, 0.26}) {
    for (double y : {-0.09, 0.0, 0.03}) {
      const auto d = placeTeachOffset(pts, Eigen::Vector3d(x, y, 0.15));   // 높이는 안 봄
      EXPECT_NEAR((d.err - f(x, y)).norm(), 0.0, 1e-9);
    }
  }
  EXPECT_NEAR(placeTeachOffset(pts, Eigen::Vector3d(0.249, 0.06, 0.0)).nearest, 0.0, 1e-12);
}

TEST(PlaceTeach, CollinearFallsBackToWeightedMean)
{
  std::vector<PlaceTeachPoint> pts(2);
  pts[0].tip = Eigen::Vector3d(0.20, 0.0, 0.0);
  pts[0].err = Eigen::Vector3d(0.001, 0.0, 0.0);
  pts[1].tip = Eigen::Vector3d(0.26, 0.0, 0.0);
  pts[1].err = Eigen::Vector3d(0.003, 0.0, 0.0);
  EXPECT_NEAR(placeTeachOffset(pts, Eigen::Vector3d(0.23, 0.0, 0.0)).err.x(), 0.002, 1e-12);
  const double near = placeTeachOffset(pts, Eigen::Vector3d(0.21, 0.0, 0.0)).err.x();
  EXPECT_GT(near, 0.001);
  EXPECT_LT(near, 0.002);
  const double out = placeTeachOffset(pts, Eigen::Vector3d(0.40, 0.05, 0.0)).err.x();   // 밖으로 끌고 나가지 않음
  EXPECT_GE(out, 0.001);
  EXPECT_LE(out, 0.003);
}

// 모델이 틀린 팔 (위팔 −5 mm · 아래팔 +3 mm · 관절 1 ~ 4 영점 어긋남) 에서 9 칸 손 맞춤 → 칸 사이 · 칸 밖 놓는 자리 오차
//   (상자 가운데 J1 0.189 m = 5 cm 정지, 놓는 점을 ±9 cm 격자 1.5 cm 간격으로)
TEST(PlaceTeach, CorrectsModelErrorNearTaughtCells)
{
  control::ArmIKParams mp;                 // 노드 모델
  control::ArmIKParams tp = mp;            // 실물 (모름)
  tp.upper_arm_z -= 0.005;
  tp.forearm += 0.003;
  tp.joint_offsets = {0.01, 0.03, -0.02, 0.01, 0.0};
  const control::ArmIK model(mp), real(tp);
  const double pitch = -M_PI_4, z = -0.005;   // 상자 윗면 퍽 놓는 집게 끝 (arm_base_link), 상자 가운데 J1 0.189 m (5 cm 정지)
  std::vector<PlaceTeachPoint> pts;
  for (int i = -1; i <= 1; ++i) {
    for (int k = -1; k <= 1; ++k) {
      PlaceTeachPoint t;
      t.tip = Eigen::Vector3d(0.189 + 0.06 * k, 0.06 * i, z);
      const auto qm = real.solve(t.tip, pitch, 0.0, true);   // 손으로 맞춘 관절 = 실물에서 그 자리
      ASSERT_TRUE(qm.has_value());
      t.err = t.tip - model.forward(*qm);
      pts.push_back(t);
    }
  }
  double worst_raw = 0.0, worst_in = 0.0, worst_out = 0.0, worst_near = 0.0;
  for (double dx = -0.09; dx <= 0.0901; dx += 0.015) {
    for (double dy = -0.09; dy <= 0.0901; dy += 0.015) {
      const Eigen::Vector3d p(0.189 + dx, dy, z);
      const auto qi = model.solve(p, pitch, 0.0, true);
      const auto qc = model.solve(p - placeTeachOffset(pts, p).err, pitch, 0.0, true);
      if (!qi || !qc) {continue;}
      const double raw = (real.forward(*qi) - p).norm(), cor = (real.forward(*qc) - p).norm();
      worst_raw = std::max(worst_raw, raw);
      const bool in = std::abs(dx) <= 0.0601 && std::abs(dy) <= 0.0601;
      double & w = in ? worst_in : (dx < -0.0601 ? worst_near : worst_out);   // 가까운 쪽 (J1 0.13 m 안) 은 팔이 접혀 따로
      w = std::max(w, cor);
    }
  }
  std::printf("[ PlaceTeach ] 모델만 최대 %.1f mm | 맞춤 9 칸 안 최대 %.2f mm | 밖 3 cm (먼 쪽 · 옆) 최대 %.2f mm | J1 쪽 3 cm 최대 %.2f mm\n",
    worst_raw * 1000.0, worst_in * 1000.0, worst_out * 1000.0, worst_near * 1000.0);
  EXPECT_GT(worst_raw, 0.005);
  EXPECT_LT(worst_in, 0.001);      // 2026-11-17: 0.91 mm (모델만 10.2 mm)
  EXPECT_LT(worst_out, 0.0015);    // 1.37 mm
  EXPECT_LT(worst_near, worst_raw);   // 4.97 mm: 팔이 접히는 J1 쪽은 밖으로 늘리기 약함 -> 맞출 때 상자를 가장 가까운 정지 자리 (5 cm) 에
}
