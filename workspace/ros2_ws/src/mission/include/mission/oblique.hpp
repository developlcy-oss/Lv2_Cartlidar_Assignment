// 비스듬히 서기 (oblique.*) 계산 — ROS 없음, 헤더만 (test_oblique.cpp)
//   상자 (정사각 윗면, 반 변 half) 중심이 차체 x 축 위에 오게 돌고, 축을 따라 보기 거리 R_look 까지 다가간다.
//   좌표: base (base_link = base_footprint x · y) 기준, 각 [rad]
#pragma once

#include <cmath>

namespace mission
{

struct ObliqueGeom
{
  double front{0.1328};    // base -> 차체 앞 끝 x [m]
  double half{0.1025};     // 장소 상자 반 변 [m]
  double tf_x{0.13};       // 1층 TFmini 렌즈 x [m]
  double icr_x{-0.0658};   // 제자리 회전 중심 x [m] (control robot_geometry.yaml base.icr_x)
};

// 상자 요는 90° 마다 같음: ±45° 안으로 접음
inline double oblique_fold45(double a)
{
  return a - (M_PI / 2) * std::round(a / (M_PI / 2));
}

// 차체 x 축이 상자 중심 (cx, cy) 을 지나게 하는 제자리 회전각 (회전 중심 icr_x 둘레)
inline double oblique_turn(const ObliqueGeom & g, double cx, double cy)
{
  return std::atan2(cy, cx - g.icr_x);
}

// 회전 뒤 상자 중심까지 축 거리 (회전 중심 둘레로 돌아 중심이 축 위 icr_x + ρ 에 옴)
inline double oblique_range_after_turn(const ObliqueGeom & g, double cx, double cy)
{
  return g.icr_x + std::hypot(cx - g.icr_x, cy);
}

// 보기 자리: 앞 끝 -> 상자 가장 가까운 점 = view_gap 일 때 base -> 상자 중심 거리 (beta = 상자 요 - 차체 요, ±45°)
inline double oblique_look_range(const ObliqueGeom & g, double view_gap, double beta)
{
  return g.front + view_gap + g.half * (std::abs(std::cos(beta)) + std::abs(std::sin(beta)));
}

// 축 위 상자 중심이 r 에 있을 때 TFmini 렌즈 -> 축이 닿는 면 거리 (축은 중심에서 면 법선과 beta 각, |beta| <= 45°)
inline double oblique_tf_target(const ObliqueGeom & g, double r, double beta)
{
  return r - g.half / std::cos(beta) - g.tf_x;
}

}  // namespace mission
