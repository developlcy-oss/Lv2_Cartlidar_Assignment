// 복귀 판정 (ROS 의존 없음 → 단위 시험). 발제 "차체 전체가 출발 구역 안에서 3 초 정지" 의 근거 기록용
//   차체 외곽 (base_link 기준 직사각) 네 모서리가 출발 구역 (지도 축에 맞춘 정사각) 안인지와 가장자리까지 가장 가까운 거리.
//   mission_node 는 wait 단계 처음 · 끝 자세로 이것을 셈해 mission_log 에 "return" 으로 남긴다 (판정만, 미션은 멈추지 않음)
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace mission
{

struct ReturnZone
{
  double cx = 0.734, cy = 0.95;       // 출발 구역 가운데 (지도 = 발제 좌표) [m]
  double half = 0.30;                 // 출발 구역 반변 (0.6 m 사각)
  double body_x_min = -0.2135;        // 차체 외곽 (base_link, B안 판, CHANGES_B.md)
  double body_x_max = 0.1328;
  double body_y_half = 0.1676;
};

struct ReturnJudge
{
  double min_margin = 0.0;            // 네 모서리 중 구역 가장자리까지 가장 가까운 거리 [m] (음수 = 밖으로 나감)
  bool inside = false;
};

inline ReturnJudge judge_return(const ReturnZone & z, double x, double y, double yaw)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  double m = std::numeric_limits<double>::infinity();
  for (const double bx : {z.body_x_min, z.body_x_max}) {
    for (const double by : {-z.body_y_half, z.body_y_half}) {
      const double wx = x + c * bx - s * by;
      const double wy = y + s * bx + c * by;
      m = std::min(m, z.half - std::max(std::abs(wx - z.cx), std::abs(wy - z.cy)));
    }
  }
  return {m, m >= 0.0};
}

}  // namespace mission
