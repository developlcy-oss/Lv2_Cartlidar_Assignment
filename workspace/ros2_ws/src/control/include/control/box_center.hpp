// 상자 중앙 맞춤 (2026-11-20, drive_node FINE 뒤) — ROS 의존 없음 → 단위 시험 (test_align.cpp)
//   FINE (Arducam 퍽 arm_offset → 회전, TFmini → 전후) 은 퍽 쪽으로만 돌아 상자 앞면과 비스듬하고 옆으로 어긋난 채 끝날 수 있다
//   (차동 구동이라 옆으로 못 감). Arducam 상자 윗면 측정 (/perception/place_box, odom 중심 · 요) 으로
//   ① 상자 법선 방향 (요, 90° 마다 같음 → 지금 yaw 에 가장 가까운 것) 과 ② 상자 중심선에서 옆 어긋남을 구하고
//   차동 구동 옆 이동 기동: 법선으로 회전 → 곧게 후진 L → shift 쪽으로 alpha 회전 → 비스듬히 hypot(L, shift) 전진 → 법선으로 회전.
//   깊이는 그대로 (후진 L = 전진의 법선 성분), 옆만 shift 만큼 옮겨짐. 남는 거리 · 요는 drive_node 가 BoxApproach (법선 yaw + TFmini) 로 맞춤
#pragma once

#include <algorithm>
#include <cmath>

namespace control
{

struct BoxCenterConfig
{
  bool enabled = true;
  double lateral_tol = 0.015;    // [m] 이 안이면 옆 이동 안 함 (법선 yaw 만 맞춤)
  double max_shift = 0.08;       // [m] 이보다 크면 측정을 믿지 않고 (다른 상자 · 잘못 찾음) 기존 동작
  double back_min = 0.06;        // [m] 후진 거리 하한
  double back_max = 0.15;        // [m] 후진 거리 상한 (뒤 /scan rear_stop 0.30 과 따로 확인)
  double max_alpha = 0.35;       // [rad] 비스듬 전진 각 목표 (약 20°). L = |shift| / tan(max_alpha) 를 [back_min, back_max] 로
  double speed = 0.05;           // [m/s] 후진 · 비스듬 전진
  double min_speed = 0.015;      // [m/s] 끝 가까이 줄일 때 하한
  double kp_lin = 1.5;           // (m/s) / m 남은 거리 → 속도
  double kp_yaw = 2.0;           // (rad/s) / rad 회전 · 직진 중 방향 유지
  double max_angular = 0.4;      // [rad/s]
  double min_angular = 0.12;     // [rad/s] 제자리 회전 하한 (turn.min_angular 와 비슷하게)
  double yaw_tol = 0.017;        // [rad] 회전 끝 허용 (약 1°)
  double hold_kp_yaw = 1.0;      // 직진 중 방향 유지 이득
  double hold_max_ang = 0.15;    // 직진 중 방향 보정 상한
  int settle_cycles = 3;         // 회전 끝 판정 연속 주기
  int max_iter = 1;              // 측정 → 옆 이동 반복 횟수 (2 이상이면 이동 뒤 다시 잼)
  bool final_puck = false;       // 옆 이동을 했으면 마지막에 퍽 arm_offset 으로 회전 (전후 TFmini) 마무리
                                 //   (11-21 사용자 결정: 중앙 = 상자 면 중심 → 기본 끔. 켜면 6° 돌아 그리퍼가 약 2.6 cm 옆으로 감)
  double max_puck_yaw_dev = 0.105;   // [rad] 퍽 쪽 방향이 상자 법선에서 이보다 벌어지면 돌지 않고 경고만 (약 6°)
  // (11-21) 회전 중 앞 모서리 침범 · S 자 (후진 없음) · 각 상한
  double alpha_hard = 0.52;      // [rad] 뒤 여유가 모자라면 각을 이것까지 키워 다시 계획 (약 30°)
  double alpha_min = 0.10;       // [rad] S 자 (후진 없음) 각 하한 — 작을수록 깊이를 많이 씀
  double corner_margin = 0.02;   // [m] 회전 중 앞 모서리 ↔ 상자 앞면 평면 여유
  bool s_curve = true;           // 앞 틈이 넉넉하면 후진 없이 S 자 (TURN1 → 비스듬 전진 → TURN2)
  double front_x = 0.1328;       // [m] base_link → 상판 앞 끝 x
  double corner_y = 0.1676;      // [m] 상판 앞 모서리 |y|
  double icr_x = -0.0658;        // [m] 제자리 회전 중심 x (robot_geometry.yaml base.icr_x)
};

inline double bc_wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}

// 상자 요 (90° 마다 같음) 중 robot_yaw 에 가장 가까운 방향 = 로봇이 바라볼 상자 법선
inline double box_facing_yaw(double robot_yaw, double box_yaw)
{
  return bc_wrap(robot_yaw + std::remainder(box_yaw - robot_yaw, M_PI / 2));
}

struct CenterError
{
  double heading = 0.0;   // 바라볼 yaw (odom)
  double lateral = 0.0;   // [m] 상자 중심선에서 로봇 (base_link) 옆 어긋남, + = 로봇이 중심선 왼쪽 (오른쪽으로 옮겨야 함)
  double yaw_err = 0.0;   // heading − 로봇 yaw (+ = 왼쪽으로 돌아야)
  double depth = 0.0;     // [m] 상자 중심 → 로봇, 법선 방향 성분 (앞에 상자가 있으면 음수)
};

// 로봇 (rx, ry, ryaw) · 상자 중심 (bx, by) · 상자 요 byaw — 모두 같은 프레임 (odom)
inline CenterError center_error(double rx, double ry, double ryaw, double bx, double by, double byaw)
{
  CenterError e;
  e.heading = box_facing_yaw(ryaw, byaw);
  const double c = std::cos(e.heading), s = std::sin(e.heading);
  const double dx = rx - bx, dy = ry - by;
  e.lateral = -s * dx + c * dy;
  e.depth = c * dx + s * dy;
  e.yaw_err = bc_wrap(e.heading - ryaw);
  return e;
}

enum class CenterDecision { DONE, SHIFT, TOO_FAR, BEHIND };

// 측정 결과로 할 일: 옆 오차가 허용 안 → DONE (요는 drive_node 유지 단계가 맞춤), max_shift 넘음 → TOO_FAR (믿지 않음),
//   상자가 앞에 없음 (depth >= 0) → BEHIND (잘못 찾음)
inline CenterDecision decide_center(const CenterError & e, const BoxCenterConfig & c)
{
  if (e.depth >= 0.0) {return CenterDecision::BEHIND;}
  if (std::fabs(e.lateral) <= c.lateral_tol) {return CenterDecision::DONE;}
  if (std::fabs(e.lateral) > c.max_shift) {return CenterDecision::TOO_FAR;}
  return CenterDecision::SHIFT;
}

struct ShiftPlan
{
  double heading = 0.0;   // 법선 yaw (odom)
  double back = 0.0;      // [m] 후진 거리
  double alpha = 0.0;     // [rad] 비스듬 전진 방향 = heading + alpha (+ = 왼쪽)
  double fwd = 0.0;       // [m] 비스듬 전진 거리
};

// lateral (+ = 로봇이 왼쪽) → 오른쪽으로 옮김 (shift = −lateral)
//   max_back = 뒤 여유 (rear_back_room) 로 줄인 후진 상한
inline ShiftPlan plan_shift(const CenterError & e, const BoxCenterConfig & c, double max_back = 1e9)
{
  ShiftPlan p;
  p.heading = e.heading;
  const double shift = -e.lateral;
  const double ta = std::tan(std::clamp(c.max_alpha, 0.05, 1.2));
  p.back = std::clamp(std::fabs(shift) / ta, c.back_min, std::max(c.back_min, std::min(c.back_max, max_back)));
  p.alpha = std::atan2(shift, p.back);
  p.fwd = std::hypot(p.back, shift);
  return p;
}

// ---------- (11-21) 회전 중 앞 모서리 침범 ----------
// 모서리 (front_x, ±corner_y) 가 ICR (icr_x, 0) 둘레로 |alpha| 돌 때 처음 앞 끝 평면보다 앞으로 나오는 최대 거리 [m] (≥ 0).
//   ρ = hypot(front_x − icr_x, corner_y), φ = atan2(corner_y, front_x − icr_x), Δx = ρ·cos(φ − |α|) − (front_x − icr_x)
//   (|α| ≥ φ 면 ρ − (front_x − icr_x)). 기본 기하: ρ 0.260, φ 40.2° → 20° 4.6 cm · 25° 5.2 cm · 28° 5.5 cm.
//   비스듬 자세 (법선에서 α) 로 곧게 가는 동안 · TURN2 시작에서도 같은 Δx 가 상자 면 쪽으로 나와 있음 (평면 기준, 보수적)
inline double corner_intrusion(double alpha, double icr_x = -0.0658, double front_x = 0.1328, double corner_y = 0.1676)
{
  const double x0 = front_x - icr_x;
  const double rho = std::hypot(x0, corner_y), phi = std::atan2(corner_y, x0);
  const double a = std::fabs(alpha);
  return (a >= phi ? rho : rho * std::cos(phi - a)) - x0;
}
inline double corner_intrusion(double alpha, const BoxCenterConfig & c)
{
  return corner_intrusion(alpha, c.icr_x, c.front_x, c.corner_y);
}

// 옆 이동 계획 (앞 틈 g0 · 뒤 여유 rear_room 반영). 끝 자세 옆 = −lateral, 깊이는 같거나 (BACK) 줄거나 (S 자, end_gap < g0)
//   ICR 오프셋은 TURN1 (+α) · TURN2 (−α) 에서 상쇄되므로 끝 자세에 영향 없음 (test_align ICR 상쇄 시험) — 회전 중 모서리에만 씀.
//   S 자 (s_curve): 후진 0, 전진 f = |s|/sinα, 깊이 소모 |s|/tanα. 조건 g0 − |s|/tanα ≥ Δx(α) + margin (TURN2 · 비스듬 전진 끝)
//     α 는 max_alpha 부터 alpha_min 까지 줄여 가며 처음 되는 각 (큰 각 = 직진 짧음 = yaw 오차에 덜 민감)
//   BACK: α = max_alpha 부터 alpha_hard 까지 키워 가며 d = |s|/tanα + δ, δ = max(0, Δx(α) + margin − g0)
//     (δ = 끝에 상자에서 더 떨어져 있을 거리 — 모서리 때문에 다 못 들어감, 남는 거리는 HOLD (법선 + TFmini) 가 메움)
//     d ≥ back_min (늘면 δ 도 늚), d ≤ min(back_max, rear_room). TURN1 조건 g0 + d ≥ Δx + margin 은 d ≥ δ 로 자동
struct MovePlan
{
  bool ok = false;
  const char * reason = "";   // 실패 사유 코드: none · corner · rear_room · back_max
  ShiftPlan p;
  bool s_curve = false;
  double extra_gap = 0.0;     // δ: 끝 자세 틈 − g0 (S 자면 음수 = 깊이 소모)
  double min_gap = 0.0;       // 기동 중 차체 앞 끝 (모서리 침범 뺀) ↔ 상자 면 최소 예상 틈 [m]
  double need_back = 0.0;     // BACK 실패 때 필요했던 최소 후진 [m] (로그용)
};

inline MovePlan plan_center_move(const CenterError & e, const BoxCenterConfig & c, double g0, double rear_room)
{
  MovePlan m;
  m.p.heading = e.heading;
  const double shift = -e.lateral, as = std::fabs(shift), sg = shift >= 0.0 ? 1.0 : -1.0;
  if (as < 1e-4) {m.reason = "none"; return m;}
  if (!(g0 >= corner_intrusion(e.yaw_err, c) + c.corner_margin - 1e-9)) {m.reason = "corner"; return m;}   // TURN0 (법선으로)
  const double step = 0.0175;   // 1°
  if (c.s_curve) {
    for (double a = std::max(c.max_alpha, c.alpha_min); a >= c.alpha_min - 1e-9; a -= step) {
      const double used = as / std::tan(a);
      const double need = corner_intrusion(a, c) + c.corner_margin;
      if (g0 - used >= need) {
        m.ok = true; m.s_curve = true;
        m.p.back = 0.0; m.p.alpha = sg * a; m.p.fwd = as / std::sin(a);
        m.extra_gap = -used; m.min_gap = g0 - used - corner_intrusion(a, c);
        return m;
      }
    }
  }
  const double back_cap = std::min(c.back_max, rear_room);
  double best_need = 1e9;
  for (double a = c.max_alpha; a <= std::max(c.max_alpha, c.alpha_hard) + 1e-9; a += step) {
    const double dx = corner_intrusion(a, c);
    double delta = std::max(0.0, dx + c.corner_margin - g0);
    double d = as / std::tan(a) + delta;
    if (d < c.back_min) {delta += c.back_min - d; d = c.back_min;}
    best_need = std::min(best_need, d);
    if (d <= back_cap + 1e-9) {
      m.ok = true;
      m.p.back = d; m.p.alpha = sg * a; m.p.fwd = (d - delta) / std::cos(a);
      m.extra_gap = delta; m.min_gap = std::min(g0 + d, g0 + delta) - dx;
      return m;
    }
  }
  m.need_back = best_need;
  m.reason = rear_room < c.back_max ? "rear_room" : "back_max";
  return m;
}

// ---------- (11-21) 반복 판단 · 센서 결합 (mission center_box 루프가 씀, drive 는 결과만 돌려줌) ----------
struct LoopConfig
{
  double done_lateral = 0.005;       // [m]
  double done_yaw = 0.01745;         // [rad] 1°
  double final_warn_lateral = 0.010; // [m] 반복을 다 써도 이 안이면 경고와 함께 끝
  double max_shift = 0.08;           // [m] 넘으면 측정을 믿지 않음
  int max_iter = 3;                  // K_max (기동 횟수)
};

enum class LoopDecision { DONE, SHIFT, FINAL_WARN, FAIL };

struct LoopVerdict
{
  LoopDecision d = LoopDecision::FAIL;
  const char * why = "";   // done · shift · yaw · iter_max · max_shift · disagree · no_measure
};

// e = 옆 오차 [m] (+ = 로봇이 중심선 왼쪽), psi = 법선 − 로봇 yaw, k = 이미 한 기동 수, disagree = 센서 불일치
inline LoopVerdict decide_loop(double e, double psi, int k, const LoopConfig & c, bool disagree = false)
{
  if (!std::isfinite(e) || !std::isfinite(psi)) {return {LoopDecision::FAIL, "no_measure"};}
  if (disagree) {return {LoopDecision::FAIL, "disagree"};}
  const double ae = std::fabs(e);
  if (ae > c.max_shift) {return {LoopDecision::FAIL, "max_shift"};}
  if (ae <= c.done_lateral && std::fabs(psi) <= c.done_yaw) {return {LoopDecision::DONE, "done"};}
  if (k >= c.max_iter) {
    return ae <= c.final_warn_lateral ? LoopVerdict{LoopDecision::FINAL_WARN, "iter_max"} : LoopVerdict{LoopDecision::FAIL, "iter_max"};
  }
  return {LoopDecision::SHIFT, ae <= c.done_lateral ? "yaw" : "shift"};
}

// 한 센서의 측정 (lateral = 로봇 옆 + 왼쪽, yaw = 법선 − 로봇 yaw, gap = 차체 앞 끝 → 상자 면)
struct SideMeas
{
  bool ok = false;
  double lateral = 0.0, yaw = 0.0, gap = 0.0;
};

struct FuseConfig
{
  double w_cam_yaw = 0.93;          // Arducam σ≈0.4° · 라이다 5 장 σ≈1.5° 분산 가중
  double disagree = 0.02;           // [m] 옆 차이가 이보다 크면 불일치
  double yaw_disagree = 0.07;       // [rad] 4° 넘게 다르면 라이다 요를 섞지 않음
  double lidar_min_gap = 0.15;      // [m] 라이다 틈이 이보다 작으면 라이다를 쓰지 않음 (range_min 0.12)
};

struct Fused
{
  bool ok = false;
  double lateral = 0.0, yaw = 0.0, gap = 0.0;
  bool disagree = false;
  bool lidar_used = false;
  const char * src = "none";   // cam · cam+lidar · lidar · none
};

// 옆은 Arducam 만 (라이다는 교차확인), 요는 분산 가중, 틈은 라이다 우선. Arducam 이 없으면 라이다만 (주행 모드)
inline Fused fuse_measure(const SideMeas & cam, const SideMeas & lid, const FuseConfig & c = {})
{
  Fused f;
  const bool lid_ok = lid.ok && lid.gap >= c.lidar_min_gap;
  if (cam.ok) {
    f.ok = true; f.lateral = cam.lateral; f.yaw = cam.yaw; f.gap = cam.gap; f.src = "cam";
    if (lid_ok) {
      f.lidar_used = true; f.src = "cam+lidar"; f.gap = lid.gap;
      f.disagree = std::fabs(lid.lateral - cam.lateral) > c.disagree;
      const double dy = bc_wrap(lid.yaw - cam.yaw);
      if (std::fabs(dy) <= c.yaw_disagree) {f.yaw = bc_wrap(cam.yaw + (1.0 - c.w_cam_yaw) * dy);}
    }
    return f;
  }
  if (lid_ok) {
    f.ok = true; f.lateral = lid.lateral; f.yaw = lid.yaw; f.gap = lid.gap; f.lidar_used = true; f.src = "lidar";
  }
  return f;
}

// 뒤 /scan 가장 가까운 점 rear_min (pullout.hpp rear_min_range) 에서 rear_stop 을 남기고 후진할 수 있는 거리.
//   back_min 보다 작으면 0 (기동 안 함). 점 없음 (+inf) = back_max
inline double rear_back_room(double rear_min, double rear_stop, const BoxCenterConfig & c)
{
  const double room = rear_min - rear_stop;
  if (!(room >= c.back_min)) {return 0.0;}
  return std::min(room, c.back_max);
}

// 퍽 마무리 판단: 로봇 (rx, ry) · 퍽 (px, py) · 상자 중심 (bx, by) · 법선 heading (모두 odom)
struct PuckCheck
{
  double yaw_dev = 0.0;    // [rad] 로봇 → 퍽 방향 − 법선 (+ = 왼쪽)
  double lateral = 0.0;    // [m] 퍽이 상자 중심선에서 벗어난 거리 (+ = 왼쪽)
  bool rotate = false;     // |yaw_dev| <= max_puck_yaw_dev 면 퍽 쪽으로 돌아도 됨
};
inline PuckCheck puck_check(double rx, double ry, double px, double py, double bx, double by, double heading,
  const BoxCenterConfig & c)
{
  PuckCheck k;
  const double co = std::cos(heading), si = std::sin(heading);
  const double fx = co * (px - rx) + si * (py - ry), fy = -si * (px - rx) + co * (py - ry);
  k.yaw_dev = std::atan2(fy, fx);
  k.lateral = -si * (px - bx) + co * (py - by);
  k.rotate = fx > 0.0 && std::fabs(k.yaw_dev) <= c.max_puck_yaw_dev;
  return k;
}

struct ShiftCmd
{
  double v = 0.0;
  double w = 0.0;
  bool done = false;
};

// 기동 실행 (odom x · y · yaw 되먹임, 주기마다 step). 단계: TURN0 법선 → BACK → TURN1 heading+alpha → FWD → TURN2 법선 → DONE
//   직진 거리는 단계 시작 위치에서 진행 방향 성분 (미끄러져 옆으로 밀려도 앞뒤만 셈)
class ShiftManeuver
{
public:
  enum class Phase { TURN0, BACK, TURN1, FWD, TURN2, DONE };

  explicit ShiftManeuver(BoxCenterConfig c = {})
  : c_(c) {}

  void start(const ShiftPlan & p)
  {
    p_ = p;
    phase_ = Phase::TURN0;
    settled_ = 0;
    started_ = false;
  }

  Phase phase() const {return phase_;}
  const ShiftPlan & plan() const {return p_;}
  bool backing() const {return phase_ == Phase::BACK;}
  bool forward() const {return phase_ == Phase::FWD;}

  // 지금 직진 단계의 남은 거리 (직진이 아니면 0)
  double remaining() const {return remaining_;}

  // 비스듬 전진을 일찍 끝냄 (TFmini 가 너무 가까움) → TURN2
  void cut_forward()
  {
    if (phase_ == Phase::FWD) {next(Phase::TURN2);}
  }

  ShiftCmd step(double x, double y, double yaw)
  {
    ShiftCmd o;
    remaining_ = 0.0;
    switch (phase_) {
      case Phase::TURN0: o = turn(yaw, p_.heading, Phase::BACK); break;
      case Phase::BACK: o = straight(x, y, yaw, p_.heading, -1.0, p_.back, Phase::TURN1); break;
      case Phase::TURN1: o = turn(yaw, bc_wrap(p_.heading + p_.alpha), Phase::FWD); break;
      case Phase::FWD: o = straight(x, y, yaw, bc_wrap(p_.heading + p_.alpha), 1.0, p_.fwd, Phase::TURN2); break;
      case Phase::TURN2: o = turn(yaw, p_.heading, Phase::DONE); break;
      case Phase::DONE: break;
    }
    o.done = phase_ == Phase::DONE;
    if (o.done) {o.v = 0.0; o.w = 0.0;}
    return o;
  }

private:
  void next(Phase ph)
  {
    phase_ = ph;
    settled_ = 0;
    started_ = false;
  }

  ShiftCmd turn(double yaw, double target, Phase then)
  {
    ShiftCmd o;
    const double e = bc_wrap(target - yaw);
    if (std::fabs(e) <= c_.yaw_tol) {
      if (++settled_ >= c_.settle_cycles) {next(then);}
      return o;
    }
    settled_ = 0;
    double w = std::clamp(c_.kp_yaw * e, -c_.max_angular, c_.max_angular);
    if (std::fabs(w) < c_.min_angular) {w = std::copysign(c_.min_angular, e);}
    o.w = w;
    return o;
  }

  // dir = +1 전진, −1 후진. heading = 차체가 유지할 yaw
  ShiftCmd straight(double x, double y, double yaw, double heading, double dir, double dist, Phase then)
  {
    ShiftCmd o;
    if (!started_) {x0_ = x; y0_ = y; started_ = true;}
    const double along = dir * (std::cos(heading) * (x - x0_) + std::sin(heading) * (y - y0_));
    remaining_ = dist - along;
    if (remaining_ <= 0.002) {next(then); remaining_ = 0.0; return o;}
    const double sp = std::clamp(c_.kp_lin * remaining_, c_.min_speed, c_.speed);
    o.v = dir * sp;
    o.w = std::clamp(c_.hold_kp_yaw * bc_wrap(heading - yaw), -c_.hold_max_ang, c_.hold_max_ang);
    return o;
  }

  BoxCenterConfig c_;
  ShiftPlan p_;
  Phase phase_ = Phase::DONE;
  int settled_ = 0;
  bool started_ = false;
  double x0_ = 0.0, y0_ = 0.0, remaining_ = 0.0;
};

}  // namespace control
