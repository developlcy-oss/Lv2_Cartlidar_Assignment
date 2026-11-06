// LocGate — /drive/navigate 를 위치 추정 상태 (/localization/status, loc_monitor) 로 막기 (#121)
//
// 왜: 위치를 다시 찾으면 추정 자세가 뛴다. 그 전 자세로 만든 Nav2 경로를 그대로 쫓으면 안 된다.
//     twist_mux 위치 잃음 잠금 (#93) 은 잃은 동안 속도만 막고, 풀린 뒤 낡은 경로를 쫓는 것은 못 막는다.
// 규칙 (인지 Isaac 모의 K2e 와 같음):
//   1. 목표를 보내기 전에 상태가 start_states (기본 OK) 가 될 때까지 기다림 (최대 wait_s, 넘으면 실패)
//   2. 진행 중 run_states (기본 OK · SUSPECT = twist_mux 잠금이 안 걸리는 상태) 밖으로 나갔다가 start_states 로
//      돌아오면 지금 Nav2 목표를 취소하고 같은 목표를 다시 보냄
//   3. 위치 추정이 나빴던 동안 Nav2 가 목표를 실패로 끝내면 (잠금 중 progress checker 등) 실패로 끝내지 않고 1 로 돌아감
//   상태를 stale_s 동안 못 받으면 나쁜 상태로 본다 (loc_monitor 가 죽은 경우). spin (제자리 회전) 은 2 를 보지 않음.
// 시각은 노드 시계 (s). ROS 의존 없음 → test_loc_gate.
#pragma once

#include <algorithm>
#include <string>
#include <vector>

namespace control
{

struct LocGateParams
{
  bool enabled{true};
  double wait_s{60.0};    // 1 · 3 에서 기다리는 최대 시간
  double stale_s{1.0};    // 상태 메시지가 이보다 오래되면 나쁜 상태 (loc_monitor 10 Hz)
  std::vector<std::string> start_states{"OK"};
  std::vector<std::string> run_states{"OK", "SUSPECT"};
};

class LocGate
{
public:
  enum class Action {NONE, SEND, RESEND, FAIL};

  LocGate() = default;
  explicit LocGate(LocGateParams p): p_(std::move(p)) {}

  void status(const std::string & s, double t) {state_ = s; rx_ = t; have_ = true;}

  bool startOk(double t) const {return fresh(t) && in(p_.start_states);}
  bool runOk(double t) const {return fresh(t) && in(p_.run_states);}

  // 새 목표. true = 바로 보냄, false = 기다림 (tick 이 SEND · FAIL 을 냄)
  bool begin(double t)
  {
    reset();
    if (!p_.enabled || startOk(t)) {return true;}
    waiting_ = true;
    wait_start_ = t;
    return false;
  }

  // Nav2 목표가 실패로 끝남. true = 위치 추정 때문으로 보고 다시 기다림 (규칙 3), false = 실패 그대로
  bool onNavFailed(double t)
  {
    if (!p_.enabled || !(bad_seen_ || !runOk(t))) {return false;}
    bad_seen_ = false;
    waiting_ = true;
    wait_start_ = t;
    return true;
  }

  // 주기 호출. nav_active = Nav2 navigate_to_pose 목표가 진행 중
  Action tick(double t, bool nav_active)
  {
    if (!p_.enabled) {return Action::NONE;}
    if (waiting_) {
      if (startOk(t)) {waiting_ = false; return Action::SEND;}
      if (t - wait_start_ > p_.wait_s) {waiting_ = false; return Action::FAIL;}
      return Action::NONE;
    }
    if (!nav_active) {return Action::NONE;}
    if (!runOk(t)) {bad_seen_ = true; return Action::NONE;}
    if (bad_seen_ && startOk(t)) {bad_seen_ = false; return Action::RESEND;}
    return Action::NONE;
  }

  void reset() {waiting_ = false; bad_seen_ = false;}
  bool waiting() const {return waiting_;}
  bool badSeen() const {return bad_seen_;}
  // 로그용: 지금 상태 (못 받음 · 오래됨 포함)
  std::string describe(double t) const
  {
    if (!have_) {return "받은 적 없음";}
    return fresh(t) ? state_ : state_ + " (오래됨)";
  }
  const LocGateParams & params() const {return p_;}

private:
  bool fresh(double t) const {return have_ && t - rx_ <= p_.stale_s;}
  bool in(const std::vector<std::string> & v) const {return std::find(v.begin(), v.end(), state_) != v.end();}

  LocGateParams p_;
  std::string state_;
  double rx_{0.0};
  bool have_{false};
  bool waiting_{false};
  bool bad_seen_{false};
  double wait_start_{0.0};
};

}  // namespace control
