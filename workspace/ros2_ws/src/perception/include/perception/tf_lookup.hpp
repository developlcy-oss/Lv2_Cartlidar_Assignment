// 촬영 시각 TF 조회 (ROS 노드 의존 없음 → tf2::BufferCore 로 단위 시험)
#pragma once

#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <thread>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/buffer_core.hpp"

namespace perception
{

// 시각 tp 의 변환 target ← source 가 버퍼에 들어올 때까지 최대 wait_s 기다린다. 들어오면 true.
// buf 는 다른 스레드(TF 리스너)가 채운다. 벽시계로 재므로 시뮬레이션 시계가 멈춰도 처리 주기를 붙잡지 않는다
inline bool wait_transform(
  const tf2::BufferCore & buf, const std::string & target, const std::string & source, const tf2::TimePoint & tp,
  double wait_s)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(wait_s);
  while (!buf.canTransform(target, source, tp)) {
    if (std::chrono::steady_clock::now() >= end) {return false;}
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

// 움직이는 변환 target ← source 를 촬영 시각 tp 로 찾는다. 그 시각 값이 없으면(아직 안 옴 · 버퍼 밖) 가장 최근 값을 쓰되,
// 그 값의 시각이 tp 와 max_dt_s 넘게 다르면 쓰지 않는다(nullopt). max_dt_s < 0 이면 촬영 시각 값만.
// 예전에는 시각 차를 보지 않고 최근 값을 썼다 → 팔 · 차체가 움직이는 중 TF 가 늦거나 멈추면 다른 자세로 위치를 냄.
// 변환이 아예 없으면 tf2::TransformException 을 그대로 던진다
inline std::optional<geometry_msgs::msg::TransformStamped> lookup_near(
  const tf2::BufferCore & buf, const std::string & target, const std::string & source, const tf2::TimePoint & tp,
  double max_dt_s)
{
  if (buf.canTransform(target, source, tp)) {return buf.lookupTransform(target, source, tp);}
  if (max_dt_s < 0) {return std::nullopt;}
  auto m = buf.lookupTransform(target, source, tf2::TimePointZero);
  const auto got = tf2::TimePoint(std::chrono::seconds(m.header.stamp.sec) + std::chrono::nanoseconds(m.header.stamp.nanosec));
  const double dt = std::abs(std::chrono::duration<double>(got - tp).count());
  if (dt > max_dt_s) {return std::nullopt;}
  return m;
}

}  // namespace perception
