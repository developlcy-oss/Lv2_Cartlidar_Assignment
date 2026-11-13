// MinJerk — 관절 목표 하나로 가는 이동을 최소 저크 (5 차) 곡선의 촘촘한 점으로 나눔 (manipulator_node arm.min_jerk_dt)
//
// 왜: 끝점 하나 (속도 0) 만 보내면 arm_controller 가 3 차 곡선으로 잇는다. 3 차는 출발 · 도착 순간 가속도가 0 에서 한 번에 뛰어
//     (저크 무한대) 관절 2 · 3 (I 게인, 처짐) 이 늦게 출발했다가 멈출 때 넘어가고, 손목 · 집게 끝이 출렁인다
//     (2026-11-13 실물 STOW → DRIVE: J2 출발 18 tick 늦음 · 멈출 때 6 tick 넘침, 2 층 팔레트 쪽으로 숙일 때 끝 떨림).
// 어떻게: s(τ) = 10τ³ − 15τ⁴ + 6τ⁵ (τ = t / T) — 출발 · 도착에서 속도 · 가속도 모두 0.
//     dt 마다 위치 · 속도 · 가속도를 채운 점을 보내 컨트롤러 보간 방식과 상관없이 같은 곡선을 따르게 한다.
//     같은 시간 T 에서 가장 빠른 순간 속도 = 1.875 × (이동량 / T) (3 차는 1.5 배).
#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace control
{

struct JointSample
{
  double t;                 // s, 출발부터
  std::vector<double> q;    // rad
  std::vector<double> v;    // rad/s
  std::vector<double> a;    // rad/s²
};

// from → to 를 duration 동안, 약 dt 간격 (마지막 점 = to, 속도 · 가속도 0). 첫 점은 t = duration / n (출발점 자체는 넣지 않음).
//   duration · dt 가 0 이하이거나 크기가 다르면 끝점 하나
inline std::vector<JointSample> minJerkSamples(
  const std::vector<double> & from, const std::vector<double> & to, double duration, double dt)
{
  const size_t m = to.size();
  if (from.size() != m || !(duration > 0.0) || !(dt > 0.0)) {
    return {JointSample{std::max(duration, 0.0), to, std::vector<double>(m, 0.0), std::vector<double>(m, 0.0)}};
  }
  const int n = std::max(1, static_cast<int>(std::ceil(duration / dt - 1e-9)));
  std::vector<JointSample> out;
  out.reserve(static_cast<size_t>(n));
  for (int k = 1; k <= n; ++k) {
    const double tau = static_cast<double>(k) / n;
    const double t2 = tau * tau, t3 = t2 * tau, t4 = t3 * tau, t5 = t4 * tau;
    const double s = 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
    const double ds = (30.0 * t2 - 60.0 * t3 + 30.0 * t4) / duration;
    const double dds = (60.0 * tau - 180.0 * t2 + 120.0 * t3) / (duration * duration);
    JointSample p{duration * tau, std::vector<double>(m), std::vector<double>(m), std::vector<double>(m)};
    for (size_t i = 0; i < m; ++i) {
      const double d = to[i] - from[i];
      p.q[i] = k == n ? to[i] : from[i] + d * s;
      p.v[i] = k == n ? 0.0 : d * ds;
      p.a[i] = k == n ? 0.0 : d * dds;
    }
    out.push_back(std::move(p));
  }
  return out;
}

}  // namespace control
