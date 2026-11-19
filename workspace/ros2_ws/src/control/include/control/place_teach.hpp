// 장소 상자 놓기 손 맞춤 보정 (2026-11-17). 상자 위 칸마다 쥔 퍽을 손으로 맞춘 관절 q (slot_teach.py --box) 를 노드 모델
//   FK 에 넣으면 모델이 생각하는 집게 끝 FK(q) 가 나옴. 실제 집게 끝은 맞춘 자리 p 이므로 칸마다 모델 오차 e = p − FK(q) 를 저장.
//   놓을 때 목표 p 대신 p − e(p) 로 노드 IK 를 풀면 실물 집게 끝 ≈ p. e(p) = 주변 칸 e 로 맞춘 국소 2 차식 값 (아래 placeTeachOffset).
//   모델 오차 (링크 길이 #295 · 영점 #292 · 처짐) 는 자리에 따라 매끄럽게 변해 9 칸 사이 · 칸 밖 몇 cm 까지 저차 다항식으로 이어짐.
//   (관절 차이 δq 를 섞는 방식은 링크 길이가 틀리면 δq 가 J1 거리에 따라 휘어 칸 사이 오차가 더 남음, test_place_teach)
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace control
{

struct PlaceTeachPoint
{
  Eigen::Vector3d tip;    // 맞춘 자리의 집게 끝 (arm_base_link) = 퍽 바닥 + 퍽 반 높이 + grasp_z_offset
  Eigen::Vector3d err;    // 모델 오차 = tip − 노드 FK(손 맞춤 관절)
};

struct PlaceTeachOffset
{
  Eigen::Vector3d err{Eigen::Vector3d::Zero()};
  double nearest{std::numeric_limits<double>::infinity()};   // 가장 가까운 맞춘 칸까지 수평 거리 (m)
};

namespace detail
{
// 가중 최소제곱: 기저 phi(Δx, Δy) 의 차수 deg (1 = 1 차 3 항, 2 = 2 차 6 항). 풀리면 상수항 (= 질의점 값), 아니면 nullopt
template<int N>
inline bool placeTeachFit(
  const std::vector<PlaceTeachPoint> & pts, const Eigen::Vector3d & p, double h, Eigen::Vector3d & out)
{
  if (pts.size() < static_cast<size_t>(N)) {return false;}
  Eigen::Matrix<double, N, N> A = Eigen::Matrix<double, N, N>::Zero();
  Eigen::Matrix<double, N, 3> B = Eigen::Matrix<double, N, 3>::Zero();
  for (const auto & t : pts) {
    const double dx = (t.tip.x() - p.x()) / h, dy = (t.tip.y() - p.y()) / h;   // h 로 나눠 조건수를 맞춤
    const double w = std::max(std::exp(-0.5 * (dx * dx + dy * dy)), 1e-12);
    Eigen::Matrix<double, N, 1> phi;
    if constexpr (N == 3) {
      phi << 1.0, dx, dy;
    } else {
      phi << 1.0, dx, dy, dx * dx, dx * dy, dy * dy;
    }
    A += w * phi * phi.transpose();
    B += w * phi * t.err.transpose();
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, N, N>> es(A);
  if (es.eigenvalues()(0) < 1e-9 * es.eigenvalues()(N - 1)) {return false;}   // 점이 한 줄 · 한 곳에 몰림
  out = A.ldlt().solve(B).row(0).transpose();
  return true;
}
}  // namespace detail

// 수평 (x, y) 국소 다항식 (moving least squares): 칸마다 가우스 가중 w = exp(−d² / 2h²) 로 e 를 2 차식 (점 6 개 이상, 아니면 1 차,
//   그래도 안 되면 가중 평균) 으로 맞춰 질의점 값을 씀. 모델 오차는 팔이 접힌 가까운 줄에서 빨리 커져 (Isaac · 모의 팔 9 칸:
//   J1 0.13 m 8.9 mm, 0.19 m 3.4 mm, 0.25 m 1.3 mm) 1 차로는 칸 사이 2 mm 가 남음. h = 칸 간격 정도 (기본 0.06 m)
inline PlaceTeachOffset placeTeachOffset(
  const std::vector<PlaceTeachPoint> & pts, const Eigen::Vector3d & p, double h = 0.06)
{
  PlaceTeachOffset out;
  if (pts.empty()) {return out;}
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  double wsum = 0.0;
  for (const auto & t : pts) {
    const double d = std::hypot(t.tip.x() - p.x(), t.tip.y() - p.y());
    out.nearest = std::min(out.nearest, d);
    const double w = std::max(std::exp(-0.5 * d * d / (h * h)), 1e-12);
    mean += w * t.err;
    wsum += w;
  }
  if (!detail::placeTeachFit<6>(pts, p, h, out.err) && !detail::placeTeachFit<3>(pts, p, h, out.err)) {
    out.err = mean / wsum;
  }
  return out;
}

}  // namespace control
