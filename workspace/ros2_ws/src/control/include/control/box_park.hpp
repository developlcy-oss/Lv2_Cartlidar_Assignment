// 박스 앞 정차 (#157): FINE 정렬로 박스 앞 (TFmini 0.13 m) 에 선 뒤 obstacle_stop 정지 거리를 언제 평소 값으로 되돌릴지 (ROS 의존 없음 → 단위 시험)
//
//   문제: 정렬이 끝나자마자 평소 정지 거리 (0.25 m) 로 되돌리면 0.13 < 0.25 → 비상 정지 → twist_mux 잠금이 drive · Nav2 를 모두 막아
//         회전 skill · goto 가 못 움직이고, 해제 (0.35 m 이상 3 번) 에 필요한 후진도 못 함 → 첫 박스에서 미션이 멈춤
//   규칙: FINE 이 끝날 때 (성공 · 실패 · 취소) 박스에서 release_m 안이면 "정차" — 낮춘 정지 거리를 그대로 둔다
//         정차 중 TFmini 가 release_m 이상을 release_count 번 연속 재면 (물러남 확인) 해제 → 평소 정지 거리로
//         정차 중 refresh_s 마다 낮춘 값을 다시 보낸다 (obstacle_stop 의 approach_max_s 자동 복귀는 drive_node 가 죽었을 때만 걸리게)
//   navigate 는 정차 중이면 먼저 곧게 후진한다 (backoff_step) — 물러남이 확인되거나 backoff_max_m 을 가면 Nav2 로
#pragma once

#include <cmath>
#include <optional>

namespace control
{

struct BoxParkConfig
{
  double release_m = 0.35;        // 물러남 확인 거리 [m] = obstacle_stop release_m 과 같게
  int release_count = 3;          // 연속 측정 수 = obstacle_stop release_count 와 같게
  double refresh_s = 20.0;        // 정차 중 낮춘 정지 거리 다시 보내는 주기 [s] (< obstacle_stop approach_max_s 60)
  double backoff_speed = 0.08;    // navigate 전 후진 속도 [m/s]
  double backoff_max_m = 0.30;    // 후진 최대 거리 [m] (odom)
  double backoff_timeout_s = 8.0; // 후진 최대 시간 [s] (막혀 기다린 시간 포함)
};

class BoxPark
{
public:
  explicit BoxPark(BoxParkConfig c = {}) : c_(c) {}

  // FINE 정렬이 끝날 때. stop_m = 정렬 동안 넣은 낮춘 정지 거리, range = 그때 TFmini (nullopt = 모름 · 무효)
  //   true = 정차 (낮춘 값 유지), false = 박스에서 멀어 바로 되돌려도 됨
  //   무효 값은 "너무 가까움 (0.10 m 안)" 일 수 있어 정차로 본다
  bool on_align_end(double t, double stop_m, std::optional<double> range)
  {
    parked_ = !(range && std::isfinite(*range) && *range >= c_.release_m);
    stop_m_ = stop_m;
    sent_t_ = t;
    ok_count_ = 0;
    return parked_;
  }

  // TFmini 측정 하나. true = 이번 측정으로 해제 (지금 평소 정지 거리로 되돌릴 것)
  bool on_range(std::optional<double> range)
  {
    if (!parked_) {return false;}
    if (range && std::isfinite(*range) && *range >= c_.release_m) {
      if (++ok_count_ >= c_.release_count) {parked_ = false; ok_count_ = 0; return true;}
    } else {
      ok_count_ = 0;   // 가깝거나 무효 (너무 가까움일 수 있음) → 처음부터
    }
    return false;
  }

  // 정차 중 다시 보낼 때인가. true 면 보낸 시각을 t 로
  bool refresh_due(double t)
  {
    if (!parked_ || t - sent_t_ < c_.refresh_s) {return false;}
    sent_t_ = t;
    return true;
  }

  void clear() {parked_ = false; ok_count_ = 0;}
  bool parked() const {return parked_;}
  double stop_m() const {return stop_m_;}
  const BoxParkConfig & config() const {return c_;}

private:
  BoxParkConfig c_;
  bool parked_ = false;
  double stop_m_ = 0.0, sent_t_ = 0.0;
  int ok_count_ = 0;
};

// navigate 전 후진 한 tick 의 판단
enum class BackoffStep { DRIVE, HOLD, RELEASED, MAX_DISTANCE, TIMEOUT };

//   parked = 아직 정차 (물러남 미확인), traveled = 후진 시작 뒤 이동 거리 [m], elapsed = 후진 시작 뒤 시간 [s],
//   rear_blocked = 뒤 /scan · 뒤 카메라가 막힘 (그동안은 멈춰 기다림, 시간은 셈)
inline BackoffStep backoff_step(const BoxParkConfig & c, bool parked, double traveled, double elapsed, bool rear_blocked)
{
  if (!parked) {return BackoffStep::RELEASED;}
  if (traveled >= c.backoff_max_m) {return BackoffStep::MAX_DISTANCE;}
  if (elapsed > c.backoff_timeout_s) {return BackoffStep::TIMEOUT;}
  return rear_blocked ? BackoffStep::HOLD : BackoffStep::DRIVE;
}

}  // namespace control
