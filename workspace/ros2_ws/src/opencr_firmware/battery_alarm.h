// battery_alarm.h — 배터리 잔량 추정 · 저전압 비프 판단 (Firmware 팀, 2026-11-11). Arduino 의존 없음 → 호스트 gtest 로 시험.
// opencr_firmware.ino 가 10 ms 마다 update(지금 ms, 전원 전압) 를 부르고, 돌려받은 비프를 부저로 울린다 (tone, 제어 루프를 막지 않음).
// 호스트 (opencr_hardware) 도 같은 잔량 표 (soc_percent) 를 써서 /diagnostics · /battery/* 에 낸다.
//
// 보드마다 자기 전원 입력 (보드별 배터리, LiPo 3S) 을 본다. 판단은 1 초 평균 전압으로 — 가속 때 잠깐 처지는 전압에는 반응하지 않는다.
//   배터리 없음  평균 < none_v (USB 만, 책상 시험)     → 울리지 않음
//   정상         그 밖                                  → 배터리를 처음 알아챈 뒤 settle_ms 에 잔량 비프 한 번 (75 % 이상 4 번 … 25 % 미만 1 번)
//   저전압 WARN  평균 < warn_v 가 warn_hold_ms 이어짐   → 바로 1 번, 그 뒤 warn_period_ms 마다 1 번
//   방전 직전 CRIT 평균 < crit_v 가 crit_hold_ms 이어짐 → 바로 3 번, 그 뒤 crit_period_ms 마다 3 번
//   되돌림: 기준 + hyst_v 위로 recover_hold_ms 이어져야 한 단계 위로 (부하를 빼면 오르는 전압에 깜빡이지 않게)
// 비프만 한다 — 모터는 건드리지 않는다 (정지는 호스트 진단 ERROR → mission 정지, 2026-11-11 사용자 결정).
#ifndef OPENCR_BATTERY_ALARM_H_
#define OPENCR_BATTERY_ALARM_H_

#include <stdint.h>

namespace opencr_battery
{

enum Level : uint8_t { NONE = 0, OK = 1, WARN = 2, CRIT = 3 };

// LiPo 셀 휴지 전압 → 잔량 % (흔히 쓰는 방전 표, 5 % 간격 선형 보간). 부하 중에는 전압이 처져 낮게 나온다 ★
inline float cell_soc_percent(float cell_v)
{
  static const float V[21] = {3.27f, 3.61f, 3.69f, 3.71f, 3.73f, 3.75f, 3.77f, 3.79f, 3.80f, 3.82f, 3.84f,
    3.85f, 3.87f, 3.91f, 3.95f, 3.98f, 4.02f, 4.08f, 4.11f, 4.15f, 4.20f};   // 0 %, 5 %, … 100 %
  if (!(cell_v > V[0])) {return 0.f;}   // NaN 도 0
  if (cell_v >= V[20]) {return 100.f;}
  for (int i = 1; i < 21; ++i) {
    if (cell_v < V[i]) {return 5.f * ((float)(i - 1) + (cell_v - V[i - 1]) / (V[i] - V[i - 1]));}
  }
  return 100.f;
}

inline float soc_percent(float pack_v, int cells) {return cells > 0 ? cell_soc_percent(pack_v / (float)cells) : 0.f;}

// 잔량 비프 횟수: 75 % 이상 4 · 50 % 이상 3 · 25 % 이상 2 · 그 아래 1
inline uint8_t soc_beeps(float percent) {return percent >= 75.f ? 4 : (percent >= 50.f ? 3 : (percent >= 25.f ? 2 : 1));}

struct Config
{
  int cells;                 // 직렬 셀 수 (3S)
  float none_v;              // 이 아래면 배터리 없음
  float warn_v;              // 저전압 (호스트 battery_warn_v 와 같게)
  float crit_v;              // 방전 직전 (호스트 battery_error_v 와 같게)
  float hyst_v;              // 되돌림 여유
  uint32_t warn_hold_ms;
  uint32_t crit_hold_ms;
  uint32_t recover_hold_ms;
  uint32_t settle_ms;        // 배터리를 알아챈 뒤 잔량 비프까지 (평균이 자리 잡게)
  uint32_t warn_period_ms;
  uint32_t crit_period_ms;
};

struct Beep
{
  uint16_t freq;   // Hz, 0 = 이번 주기에는 시작할 비프 없음
  uint16_t ms;
};

class BatteryAlarm
{
public:
  // 비프 소리 (enum — C++11 에서 정의 없이 시험 코드가 값으로 비교할 수 있게)
  enum : uint16_t { SOC_HZ = 1500, SOC_MS = 100, SOC_GAP_MS = 200 };
  enum : uint16_t { WARN_HZ = 2000, WARN_MS = 150 };
  enum : uint16_t { CRIT_HZ = 2500, CRIT_MS = 100, CRIT_GAP_MS = 150 };

  explicit BatteryAlarm(const Config & c) : c_(c) {}

  // 10 ms 마다. now_ms 는 millis() (넘침 안전 — 차이로만 씀)
  Beep update(uint32_t now_ms, float v)
  {
    Beep none = {0, 0};
    if (!(v == v)) {v = 0.f;}   // NaN → 0
    if (!started_) {
      started_ = true;
      avg_ = v;
    } else {
      const uint32_t dt = now_ms - last_ms_;
      const float a = dt >= 1000u ? 1.f : (float)dt / 1000.f;   // 시간 기준 1 차 저역 통과 (τ ≈ 1 s)
      avg_ += a * (v - avg_);
    }
    last_ms_ = now_ms;

    if (avg_ < c_.none_v) {   // 배터리 없음 · 빠짐: 조용히, 다시 꽂히면 처음부터
      level_ = NONE;
      seq_left_ = 0;
      soc_done_ = false;
      below_warn_ = below_crit_ = above_warn_ = above_crit_ = false;
      return none;
    }
    if (level_ == NONE) {   // 배터리를 알아챔 (부팅 · 다시 꽂음): 평균을 지금 값부터 (0 V 에서 천천히 올라오지 않게)
      level_ = OK;
      present_ms_ = now_ms;
      avg_ = v;
    }
    step_level(now_ms);

    // 진행 중인 비프 묶음
    if (seq_left_ > 0) {
      if ((int32_t)(now_ms - seq_next_ms_) >= 0) {
        --seq_left_;
        seq_next_ms_ = now_ms + seq_ms_ + seq_gap_ms_;
        Beep b = {seq_hz_, seq_ms_};
        return b;
      }
      return none;
    }
    // 잔량 비프 (배터리를 알아챈 뒤 한 번) — settle 이 지나고 평균이 지금 전압을 따라잡았을 때 (꽂는 중 오르는 전압으로 세지 않게)
    const float lag = v > avg_ ? v - avg_ : avg_ - v;
    if (!soc_done_ && now_ms - present_ms_ >= c_.settle_ms && lag < 0.1f) {
      soc_done_ = true;
      start_seq(now_ms, soc_beeps(soc_percent(avg_, c_.cells)), SOC_HZ, SOC_MS, SOC_GAP_MS);
      return update_seq(now_ms);
    }
    // 경고 · 위험 반복
    if ((level_ == WARN || level_ == CRIT) && (int32_t)(now_ms - alarm_next_ms_) >= 0) {
      if (level_ == CRIT) {
        alarm_next_ms_ = now_ms + c_.crit_period_ms;
        start_seq(now_ms, 3, CRIT_HZ, CRIT_MS, CRIT_GAP_MS);
      } else {
        alarm_next_ms_ = now_ms + c_.warn_period_ms;
        start_seq(now_ms, 1, WARN_HZ, WARN_MS, 0);
      }
      return update_seq(now_ms);
    }
    return none;
  }

  Level level() const {return level_;}
  float average_v() const {return avg_;}
  float percent() const {return level_ == NONE ? 0.f : soc_percent(avg_, c_.cells);}

private:
  void step_level(uint32_t now_ms)
  {
    // 아래로: 기준 밑이 hold 동안 이어지면
    hold(avg_ < c_.crit_v, below_crit_, below_crit_ms_, now_ms);
    hold(avg_ < c_.warn_v, below_warn_, below_warn_ms_, now_ms);
    if (level_ != CRIT && below_crit_ && now_ms - below_crit_ms_ >= c_.crit_hold_ms) {
      level_ = CRIT;
      alarm_next_ms_ = now_ms;   // 들어가자마자 울림
    } else if (level_ == OK && below_warn_ && now_ms - below_warn_ms_ >= c_.warn_hold_ms) {
      level_ = WARN;
      alarm_next_ms_ = now_ms;
    }
    // 위로: 기준 + 여유 위가 recover_hold 동안 이어지면 한 단계
    hold(avg_ >= c_.crit_v + c_.hyst_v, above_crit_, above_crit_ms_, now_ms);
    hold(avg_ >= c_.warn_v + c_.hyst_v, above_warn_, above_warn_ms_, now_ms);
    if (level_ == CRIT && above_crit_ && now_ms - above_crit_ms_ >= c_.recover_hold_ms) {
      level_ = WARN;
      alarm_next_ms_ = now_ms + c_.warn_period_ms;
    } else if (level_ == WARN && above_warn_ && now_ms - above_warn_ms_ >= c_.recover_hold_ms) {
      level_ = OK;
    }
  }

  static void hold(bool cond, bool & active, uint32_t & since, uint32_t now_ms)
  {
    if (!cond) {
      active = false;
    } else if (!active) {
      active = true;
      since = now_ms;
    }
  }

  void start_seq(uint32_t now_ms, uint8_t n, uint16_t hz, uint16_t ms, uint16_t gap)
  {
    seq_left_ = n;
    seq_hz_ = hz;
    seq_ms_ = ms;
    seq_gap_ms_ = gap;
    seq_next_ms_ = now_ms;
  }

  Beep update_seq(uint32_t now_ms)
  {
    --seq_left_;
    seq_next_ms_ = now_ms + seq_ms_ + seq_gap_ms_;
    Beep b = {seq_hz_, seq_ms_};
    return b;
  }

  Config c_;
  bool started_ = false;
  uint32_t last_ms_ = 0;
  float avg_ = 0.f;
  Level level_ = NONE;
  uint32_t present_ms_ = 0;
  bool soc_done_ = false;
  bool below_warn_ = false, below_crit_ = false, above_warn_ = false, above_crit_ = false;
  uint32_t below_warn_ms_ = 0, below_crit_ms_ = 0, above_warn_ms_ = 0, above_crit_ms_ = 0;
  uint32_t alarm_next_ms_ = 0;
  uint8_t seq_left_ = 0;
  uint16_t seq_hz_ = 0, seq_ms_ = 0, seq_gap_ms_ = 0;
  uint32_t seq_next_ms_ = 0;
};

}  // namespace opencr_battery

#endif  // OPENCR_BATTERY_ALARM_H_
