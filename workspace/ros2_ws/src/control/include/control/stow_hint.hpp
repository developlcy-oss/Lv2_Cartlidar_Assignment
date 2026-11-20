// StowHint — 팔이 STOW (대기 자세) 와 얼마나 어긋났는지 사람이 손으로 바로 고칠 수 있게 관절마다 각도 · 손끝 거리 · 방향으로 (2026-11-20)
//
// 왜: 켜질 때 STOW 에서 auto_stow_max 넘게 벗어나면 manipulator_node 가 움직이지 않고 기다리는데, 예전 로그는 "최대 차이 0.80 rad" 뿐이라
//     어느 관절을 어느 쪽으로 얼마나 옮겨야 하는지 몰랐다.
// 어떻게: 관절 하나만 STOW 값에서 지금 값으로 바꿨을 때 손끝 (집게 끝 tool_link) 이 STOW 자리에서 옮겨 간 벡터 (arm_base_link: x 앞 · y 왼 · z 위)
//     를 URDF 기구로 정확히 (작은 각 근사 아님) 계산. 거리 = 그 벡터 길이 (현 길이). 큰 것부터.
//     기구 = opencr_hardware/urdf/robot.urdf.xacro 116-121 (joint1 z 0.077 · joint3 (0.024, 0.128) · joint4 0.124 · joint5 0.04)
//     + 손목 → 집게 끝 0.167 (= 0.04 + 0.041 + 0.086, arm_ik.hpp). 관절 각은 명령값 그대로 (ik.joint_offsets 안 씀: 접힌 STOW 실물과 맞춘 값이 아님)
//   방향: joint1 축 +z (위에서 볼 때 + = 반시계), joint2~4 축 +y (+ = 앞을 아래로 숙임), joint5 축 = 손목 → 손끝 (손끝에서 손목 쪽으로 볼 때 + = 반시계)
//   2026-11-20 STOW [2.200, -2.088, 1.549, -1.868] 의 손끝 = (83, −114, 210) mm, joint1 축에서 141 mm → J1 + 1° = 손끝 왼쪽 1.4 · 앞 2.0 mm
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace control
{

struct StowHintGeom
{
  double shoulder_height{0.077};
  double upper_arm_x{0.024};
  double upper_arm_z{0.128};
  double forearm{0.124};
  double wrist_to_tool{0.167};
};

using StowVec3 = std::array<double, 3>;

// 관절 1~5 (rad, 명령값) → 손끝 (m, arm_base_link). ArmIK::forward 와 같은 식 (offset 0 · sign 1)
inline StowVec3 stowTip(const std::array<double, 5> & q, const StowHintGeom & g)
{
  const double a = std::hypot(g.upper_arm_x, g.upper_arm_z);
  const double th_a = std::atan2(g.upper_arm_z, g.upper_arm_x) - q[1];
  const double th_b = -(q[1] + q[2]);
  const double th_t = -(q[1] + q[2] + q[3]);
  const double d = a * std::cos(th_a) + g.forearm * std::cos(th_b) + g.wrist_to_tool * std::cos(th_t);
  const double z = g.shoulder_height + a * std::sin(th_a) + g.forearm * std::sin(th_b) + g.wrist_to_tool * std::sin(th_t);
  return {d * std::cos(q[0]), d * std::sin(q[0]), z};
}

struct StowJointHint
{
  int joint{0};          // 1..5
  double dq{0.0};        // 지금 − STOW (rad, (-pi, pi])
  StowVec3 move{};       // 이 관절만 지금 값일 때 손끝 − STOW 손끝 (m)
  double dist{0.0};      // |move| (m)
  double lever{0.0};     // 그 관절 축 ↔ 손끝 (m, STOW 에서). 허용 각 → mm 환산용
};

// 관절 축에서 손끝까지 (STOW 자세, m): J1 = 수평 거리, J2~4 = 팔 평면 안 거리, J5 = 0 (손끝이 축 위)
inline double stowLever(int joint, const std::array<double, 5> & stow, const StowHintGeom & g)
{
  const StowVec3 tip = stowTip(stow, g);
  if (joint == 1) {return std::hypot(tip[0], tip[1]);}
  if (joint == 5) {return 0.0;}
  const double a = std::hypot(g.upper_arm_x, g.upper_arm_z);
  const double th_a = std::atan2(g.upper_arm_z, g.upper_arm_x) - stow[1];
  const double th_b = -(stow[1] + stow[2]);
  const double th_t = -(stow[1] + stow[2] + stow[3]);
  // 팔 평면 (수평 거리 d, 높이 z) 에서 관절 축 자리
  double ad = 0.0, az = g.shoulder_height;
  if (joint >= 3) {ad += a * std::cos(th_a); az += a * std::sin(th_a);}
  if (joint >= 4) {ad += g.forearm * std::cos(th_b); az += g.forearm * std::sin(th_b);}
  const double td = a * std::cos(th_a) + g.forearm * std::cos(th_b) + g.wrist_to_tool * std::cos(th_t);
  const double tz = g.shoulder_height + a * std::sin(th_a) + g.forearm * std::sin(th_b) + g.wrist_to_tool * std::sin(th_t);
  return std::hypot(td - ad, tz - az);
}

// 관절마다 어긋남. min_rad 보다 작은 관절은 뺌. 손끝 거리 큰 것부터 (같으면 각도 큰 것부터 — J5 는 거리 0)
inline std::vector<StowJointHint> stowJointHints(
  const std::vector<double> & current, const std::vector<double> & stow, const StowHintGeom & g = {},
  double min_rad = 0.0087)
{
  std::vector<StowJointHint> out;
  if (current.size() < 5 || stow.size() < 5) {return out;}
  std::array<double, 5> s{};
  for (size_t k = 0; k < 5; ++k) {s[k] = stow[k];}
  const StowVec3 tip0 = stowTip(s, g);
  for (int j = 1; j <= 5; ++j) {
    const double dq = std::remainder(current[j - 1] - s[j - 1], 2.0 * M_PI);
    if (std::abs(dq) < min_rad) {continue;}
    std::array<double, 5> q = s;
    q[j - 1] = s[j - 1] + dq;
    const StowVec3 tip = stowTip(q, g);
    StowJointHint h;
    h.joint = j;
    h.dq = dq;
    h.move = {tip[0] - tip0[0], tip[1] - tip0[1], tip[2] - tip0[2]};
    h.dist = std::sqrt(h.move[0] * h.move[0] + h.move[1] * h.move[1] + h.move[2] * h.move[2]);
    h.lever = stowLever(j, s, g);
    out.push_back(h);
  }
  std::stable_sort(out.begin(), out.end(), [](const StowJointHint & a, const StowJointHint & b) {
      if (std::abs(a.dist - b.dist) > 5e-4) {return a.dist > b.dist;}
      return std::abs(a.dq) > std::abs(b.dq);
    });
  return out;
}

// 손끝 옮김 벡터 → "왼쪽 52 · 앞 35 mm". 1 mm 넘는 성분만, 큰 것부터. left_right_first = 좌우를 맨 앞에 (J1)
inline std::string stowMoveWords(const StowVec3 & m, bool left_right_first = false)
{
  struct Part {double mm; const char * word; int axis;};
  std::vector<Part> parts = {
    {m[0] * 1000.0, m[0] >= 0 ? "앞" : "뒤", 0},
    {m[1] * 1000.0, m[1] >= 0 ? "왼쪽" : "오른쪽", 1},
    {m[2] * 1000.0, m[2] >= 0 ? "위" : "아래", 2}};
  std::stable_sort(parts.begin(), parts.end(), [left_right_first](const Part & a, const Part & b) {
      if (left_right_first && (a.axis == 1) != (b.axis == 1)) {return a.axis == 1;}
      return std::abs(a.mm) > std::abs(b.mm);
    });
  std::string s;
  for (const auto & p : parts) {
    if (std::abs(p.mm) < 1.0 && !(left_right_first && p.axis == 1)) {continue;}
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%s%s %.0f", s.empty() ? "" : " · ", p.word, std::abs(p.mm));
    s += buf;
  }
  return s.empty() ? "0" : s + " mm";
}

// 한 관절 안내 한 줄: "J1 +12.3° (위에서 볼 때 반시계) → 손끝 왼쪽 30 · 앞 22 mm"
inline std::string stowJointHintText(const StowJointHint & h)
{
  static const char * kName[] = {"", "몸통 회전", "어깨", "팔꿈치", "손목 굽힘", "손목 회전"};
  const double deg = h.dq * 180.0 / M_PI;
  char head[96];
  std::snprintf(head, sizeof(head), "J%d %s %+.1f°", h.joint, kName[h.joint], deg);
  std::string s = head;
  if (h.joint == 1) {
    s += h.dq > 0 ? " (위에서 볼 때 반시계)" : " (위에서 볼 때 시계)";
  } else if (h.joint == 5) {
    s += h.dq > 0 ? " (손끝에서 손목 쪽으로 볼 때 반시계) → 손끝 자리는 그대로, 집게만 비틀림" :
      " (손끝에서 손목 쪽으로 볼 때 시계) → 손끝 자리는 그대로, 집게만 비틀림";
    return s;
  }   // J2~4: 접힌 STOW 에서는 숙임 · 듦이 손끝 방향과 맞지 않아 손끝 방향만
  return s + " → 손끝 " + stowMoveWords(h.move, h.joint == 1);
}

// 전체 안내: "STOW 어긋남 (지금 − STOW, 손끝 = 집게 끝, 로봇 앞 +x · 왼쪽 +y): J1 ... ; J3 ... | 허용 각 ±37.2° (J1 손끝 ≈ 92 mm)
//   → 손으로 반대쪽으로 옮기세요". 줄마다 "\n  " 로 시작 (로그에 이어 붙임). tol_rad > 0 이면 넘는 관절에 ★
inline std::string stowHintText(
  const std::vector<double> & current, const std::vector<double> & stow, double tol_rad, const StowHintGeom & g = {})
{
  const auto hints = stowJointHints(current, stow, g);
  if (hints.empty()) {return "\n  STOW 어긋남 없음 (관절마다 0.5° 안)";}
  std::string s = "\n  STOW 어긋남 (지금 − STOW, 손끝 = 집게 끝, 로봇 앞 기준 · 큰 것부터. 손으로 반대쪽으로 옮기세요):";
  for (const auto & h : hints) {
    s += "\n    " + stowJointHintText(h);
    if (tol_rad > 0.0) {
      char buf[96];
      if (h.joint == 5) {
        std::snprintf(buf, sizeof(buf), " (허용 ±%.1f°)", tol_rad * 180.0 / M_PI);
      } else {
        std::snprintf(buf, sizeof(buf), " (허용 ±%.1f° ≈ 손끝 %.0f mm)", tol_rad * 180.0 / M_PI, h.lever * tol_rad * 1000.0);
      }
      s += buf;
      if (std::abs(h.dq) > tol_rad) {s += " ★ 넘음";}
    }
  }
  return s;
}

}  // namespace control
