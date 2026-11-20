// 팔 대기 자세 (STOW) 어긋남 안내 — check_arm_standby 거부 · 경고 메시지에 붙임 (2026-11-20)
//
// 관절마다 대기 범위 가운데 (= manipulator.yaml STOW) 대비 각도와, 그 관절만 지금 값일 때 손끝 (집게 끝) 이 옮겨 간 거리 · 방향
//   (로봇 앞 +x · 왼쪽 +y · 위 +z). control/include/control/stow_hint.hpp 와 같은 계산 (패키지 의존을 만들지 않으려고 따로 둠)
// 기구 = urdf/robot.urdf.xacro 116-121 (joint1 z 0.077 · joint3 (0.024, 0.128) · joint4 0.124) + 손목 → 집게 끝 0.167 (control arm_ik.hpp)
// tick → rad = (tick − 2048) × 2π / 4096 (모델 각도, 관절 1 은 팔 fw v6 이 π 돌려 보냄)
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace opencr_hardware
{

inline double standby_tick_to_rad(double tick) {return (tick - 2048.0) * 2.0 * M_PI / 4096.0;}

// 관절 1~5 (rad) → 손끝 (m, arm_base_link)
inline std::array<double, 3> standby_tip(const std::array<double, 5> & q)
{
  constexpr double kShoulder = 0.077, kUpX = 0.024, kUpZ = 0.128, kFore = 0.124, kTool = 0.167;
  const double a = std::hypot(kUpX, kUpZ);
  const double th_a = std::atan2(kUpZ, kUpX) - q[1];
  const double th_b = -(q[1] + q[2]);
  const double th_t = -(q[1] + q[2] + q[3]);
  const double d = a * std::cos(th_a) + kFore * std::cos(th_b) + kTool * std::cos(th_t);
  const double z = kShoulder + a * std::sin(th_a) + kFore * std::sin(th_b) + kTool * std::sin(th_t);
  return {d * std::cos(q[0]), d * std::sin(q[0]), z};
}

// 범위 밖 관절 (칸 0~4) 마다 "\n    J1 +12.3° (위에서 볼 때 반시계) → 손끝 왼쪽 30 · 앞 22 mm" (STOW = 범위 가운데 대비, 손끝 거리 큰 것부터).
// ticks · lo · hi 는 칸 0~4 (그리퍼 칸 5 는 안 봄). 범위 안 관절은 뺌. 모두 안이면 빈 문자열
inline std::string arm_standby_hint(
  const std::vector<long> & ticks, const std::vector<long> & lo, const std::vector<long> & hi)
{
  if (ticks.size() < 5 || lo.size() < 5 || hi.size() < 5) {return "";}
  std::array<double, 5> stow{};
  for (size_t k = 0; k < 5; ++k) {stow[k] = standby_tick_to_rad(0.5 * (lo[k] + hi[k]));}
  const auto tip0 = standby_tip(stow);
  struct Line {double dist; double deg; std::string text;};
  std::vector<Line> lines;
  static const char * kName[] = {"몸통 회전", "어깨", "팔꿈치", "손목 굽힘", "손목 회전"};
  for (size_t k = 0; k < 5; ++k) {
    if (ticks[k] >= lo[k] && ticks[k] <= hi[k]) {continue;}
    const double dq = std::remainder(standby_tick_to_rad(static_cast<double>(ticks[k])) - stow[k], 2.0 * M_PI);
    auto q = stow;
    q[k] += dq;
    const auto tip = standby_tip(q);
    const double m[3] = {(tip[0] - tip0[0]) * 1000.0, (tip[1] - tip0[1]) * 1000.0, (tip[2] - tip0[2]) * 1000.0};
    const double dist = std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
    char head[160];
    const char * sense = k == 0 ? (dq > 0 ? " (위에서 볼 때 반시계)" : " (위에서 볼 때 시계)") :
      k == 4 ? (dq > 0 ? " (손끝에서 손목 쪽으로 볼 때 반시계)" : " (손끝에서 손목 쪽으로 볼 때 시계)") : "";   // J2~4: 손끝 방향만
    std::snprintf(head, sizeof(head), "\n    J%zu %s %+.1f°%s", k + 1, kName[k], dq * 180.0 / M_PI, sense);
    std::string text = head;
    if (k == 4) {
      text += " → 손끝 자리는 그대로, 집게만 비틀림";
    } else {
      // 성분: 1 mm 넘는 것만 큰 것부터, J1 은 좌우를 맨 앞에
      struct Part {double mm; const char * word; int axis;};
      std::vector<Part> parts = {{m[0], m[0] >= 0 ? "앞" : "뒤", 0}, {m[1], m[1] >= 0 ? "왼쪽" : "오른쪽", 1},
        {m[2], m[2] >= 0 ? "위" : "아래", 2}};
      std::stable_sort(parts.begin(), parts.end(), [k](const Part & a, const Part & b) {
          if (k == 0 && (a.axis == 1) != (b.axis == 1)) {return a.axis == 1;}
          return std::abs(a.mm) > std::abs(b.mm);
        });
      std::string words;
      for (const auto & p : parts) {
        if (std::abs(p.mm) < 1.0 && !(k == 0 && p.axis == 1)) {continue;}
        char buf[48];
        std::snprintf(buf, sizeof(buf), "%s%s %.0f", words.empty() ? "" : " · ", p.word, std::abs(p.mm));
        words += buf;
      }
      text += " → 손끝 " + (words.empty() ? std::string("0") : words + " mm");
    }
    lines.push_back({dist, std::abs(dq), text});
  }
  std::stable_sort(lines.begin(), lines.end(), [](const Line & a, const Line & b) {
      if (std::abs(a.dist - b.dist) > 0.5) {return a.dist > b.dist;}
      return a.deg > b.deg;
    });
  if (lines.empty()) {return "";}
  std::string s = "\n  STOW (범위 가운데) 대비, 손끝 = 집게 끝, 로봇 앞 기준 · 큰 것부터 — 손으로 반대쪽으로 옮기세요:";
  for (const auto & l : lines) {s += l.text;}
  return s;
}

}  // namespace opencr_hardware
