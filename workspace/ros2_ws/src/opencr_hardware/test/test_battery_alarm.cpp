// test_battery_alarm.cpp — 배터리 잔량 표 · 저전압 비프 판단 (opencr_firmware/battery_alarm.h, 실제 펌웨어 코드) +
//   호스트 진단 판정 (src/battery_status.hpp). 설정은 실제 config.h 값 (LiPo 3S, 경고 11.1 · 위험 10.5 V). ROS 필요 없음.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "config.h"
#include "battery_alarm.h"
#include "battery_status.hpp"

using opencr_battery::BatteryAlarm;
using opencr_battery::Beep;

namespace
{

opencr_battery::Config real_config()
{
  return {BATTERY_CELLS, BATTERY_NONE_V, BATTERY_WARN_V, BATTERY_CRIT_V, BATTERY_HYST_V, BATTERY_WARN_HOLD_MS,
    BATTERY_CRIT_HOLD_MS, BATTERY_RECOVER_HOLD_MS, BATTERY_SETTLE_MS, BATTERY_WARN_PERIOD_MS, BATTERY_CRIT_PERIOD_MS};
}

// 펌웨어 loop 처럼 10 ms 마다 update
struct Sim
{
  explicit Sim(uint32_t t0 = 0) : t(t0) {}
  BatteryAlarm a{real_config()};
  uint32_t t;
  std::vector<std::pair<uint32_t, Beep>> beeps;   // (울린 시각, 비프)

  void run(uint32_t ms, float v)
  {
    for (uint32_t k = 0; k < ms; k += 10) {
      const Beep b = a.update(t, v);
      if (b.freq) {beeps.push_back({t, b});}
      t += 10;
    }
  }
  int count(uint16_t hz) const
  {
    int n = 0;
    for (const auto & b : beeps) {n += b.second.freq == hz ? 1 : 0;}
    return n;
  }
  void clear() {beeps.clear();}
};

}  // namespace

// LiPo 셀 전압 → 잔량 표: 끝점 · 가운데 · 단조 증가 · NaN
TEST(BatteryAlarm, SocTable)
{
  using opencr_battery::cell_soc_percent;
  using opencr_battery::soc_percent;
  EXPECT_FLOAT_EQ(cell_soc_percent(4.20f), 100.f);
  EXPECT_FLOAT_EQ(cell_soc_percent(4.35f), 100.f);
  EXPECT_FLOAT_EQ(cell_soc_percent(3.27f), 0.f);
  EXPECT_FLOAT_EQ(cell_soc_percent(3.00f), 0.f);
  EXPECT_NEAR(cell_soc_percent(3.84f), 50.f, 1e-3);
  EXPECT_FLOAT_EQ(cell_soc_percent(std::numeric_limits<float>::quiet_NaN()), 0.f);
  float prev = -1.f;
  for (float v = 3.2f; v <= 4.3f; v += 0.005f) {
    const float p = cell_soc_percent(v);
    EXPECT_GE(p, prev) << v;
    prev = p;
  }
  // 3S 팩: 만충 · 경고 · 위험 기준이 어느 잔량인지 (설명 · 문서와 같은지)
  EXPECT_FLOAT_EQ(soc_percent(12.6f, 3), 100.f);
  EXPECT_NEAR(soc_percent(BATTERY_WARN_V, BATTERY_CELLS), 12.5f, 0.5f) << "경고 11.1 V = 셀 3.70 V ≈ 12 %";
  EXPECT_NEAR(soc_percent(BATTERY_CRIT_V, BATTERY_CELLS), 3.4f, 0.5f) << "위험 10.5 V = 셀 3.50 V ≈ 3 %";
  EXPECT_FLOAT_EQ(soc_percent(12.f, 0), 0.f);
  EXPECT_EQ(opencr_battery::soc_beeps(80.f), 4);
  EXPECT_EQ(opencr_battery::soc_beeps(60.f), 3);
  EXPECT_EQ(opencr_battery::soc_beeps(30.f), 2);
  EXPECT_EQ(opencr_battery::soc_beeps(10.f), 1);
}

// USB 만 (배터리 없음): 오래 켜 둬도 울리지 않음
TEST(BatteryAlarm, SilentWithoutBattery)
{
  Sim s;
  s.run(120000, 0.4f);
  EXPECT_TRUE(s.beeps.empty());
  EXPECT_EQ(s.a.level(), opencr_battery::NONE);
}

// 배터리를 알아채면 settle 뒤 잔량 비프 한 번 (횟수 = 잔량 구간), 그 뒤 정상이면 조용
TEST(BatteryAlarm, SocBeepsOnceAfterSettle)
{
  struct Case { float v; int n; };
  for (const Case c : {Case{12.4f, 4}, Case{11.6f, 3}, Case{11.3f, 2}}) {
    Sim s;
    s.run(BATTERY_SETTLE_MS - 20, c.v);
    EXPECT_TRUE(s.beeps.empty()) << c.v << " V: settle 전에는 안 울림";
    s.run(60000, c.v);
    EXPECT_EQ(s.count(BatteryAlarm::SOC_HZ), c.n) << c.v << " V";
    EXPECT_EQ(static_cast<int>(s.beeps.size()), c.n) << c.v << " V: 정상이면 잔량 비프만";
    for (size_t i = 1; i < s.beeps.size(); ++i) {
      EXPECT_GE(s.beeps[i].first - s.beeps[i - 1].first, static_cast<uint32_t>(BatteryAlarm::SOC_MS + BatteryAlarm::SOC_GAP_MS))
        << "비프가 겹치지 않음";
    }
    EXPECT_EQ(s.a.level(), opencr_battery::OK);
  }
}

// 저전압: 기준 밑이 warn_hold 이어진 뒤 바로 1 번, 그 뒤 30 s 마다
TEST(BatteryAlarm, WarnAfterHoldThenPeriodic)
{
  Sim s;
  s.run(5000, 12.4f);
  s.clear();
  const uint32_t t_drop = s.t;
  s.run(BATTERY_WARN_HOLD_MS - 500, 11.0f);   // 평균이 내려오는 시간 + hold 전에는 안 울림
  EXPECT_EQ(s.count(BatteryAlarm::WARN_HZ), 0);
  s.run(70000, 11.0f);
  ASSERT_GE(s.count(BatteryAlarm::WARN_HZ), 1);
  EXPECT_EQ(s.a.level(), opencr_battery::WARN);
  const uint32_t first = s.beeps.front().first;
  EXPECT_GE(first - t_drop, BATTERY_WARN_HOLD_MS);
  // 평균 (τ 1 s) 이 12.4 → 11.0 으로 내려오며 11.1 을 지나는 데 ln(1.4 / 0.1) ≈ 2.64 s, 그 뒤 hold 10 s
  EXPECT_NEAR(static_cast<double>(first - t_drop), BATTERY_WARN_HOLD_MS + 2640.0, 100.0);
  EXPECT_EQ(s.count(BatteryAlarm::WARN_HZ), 3) << "약 70 s 동안: 처음 + 30 s + 60 s";
  EXPECT_EQ(s.count(BatteryAlarm::CRIT_HZ), 0);
  for (size_t i = 1; i < s.beeps.size(); ++i) {EXPECT_EQ(s.beeps[i].first - s.beeps[i - 1].first, BATTERY_WARN_PERIOD_MS);}
}

// 가속 때처럼 잠깐 (2 s) 처지는 전압에는 반응하지 않음
TEST(BatteryAlarm, ShortSagIgnored)
{
  Sim s;
  s.run(5000, 12.0f);
  s.clear();
  for (int k = 0; k < 5; ++k) {
    s.run(2000, 10.0f);
    s.run(8000, 12.0f);
  }
  EXPECT_TRUE(s.beeps.empty());
  EXPECT_EQ(s.a.level(), opencr_battery::OK);
}

// 방전 직전: crit_hold 뒤 3 번씩, 2 s 마다
TEST(BatteryAlarm, CritAfterHoldRepeats)
{
  Sim s;
  s.run(5000, 12.0f);
  s.clear();
  const uint32_t t_drop = s.t;
  s.run(10000, 10.2f);
  ASSERT_GE(s.count(BatteryAlarm::CRIT_HZ), 3);
  EXPECT_EQ(s.a.level(), opencr_battery::CRIT);
  EXPECT_GE(s.beeps.front().first - t_drop, BATTERY_CRIT_HOLD_MS);
  EXPECT_EQ(s.count(BatteryAlarm::WARN_HZ), 0) << "warn_hold (10 s) 보다 crit_hold (3 s) 가 먼저";
  EXPECT_EQ(s.count(BatteryAlarm::CRIT_HZ) % 3, 0) << "3 번 묶음";
  EXPECT_GE(s.count(BatteryAlarm::CRIT_HZ), 3 * 3) << "약 6 s 동안 2 s 마다";
}

// 되돌림: 기준 + 0.2 V 위가 3 s 이어져야 한 단계 위로 (부하를 빼서 잠깐 오르는 전압에는 그대로)
TEST(BatteryAlarm, RecoversWithHysteresis)
{
  Sim s;
  s.run(5000, 12.0f);
  s.run(8000, 10.2f);
  ASSERT_EQ(s.a.level(), opencr_battery::CRIT);
  s.run(10000, BATTERY_CRIT_V + 0.1f);
  EXPECT_EQ(s.a.level(), opencr_battery::CRIT) << "여유 (0.2 V) 안이면 그대로";
  s.run(6000, 10.85f);
  EXPECT_EQ(s.a.level(), opencr_battery::WARN);
  s.run(6000, 11.2f);
  EXPECT_EQ(s.a.level(), opencr_battery::WARN) << "11.1 + 0.2 = 11.3 V 밑";
  s.run(6000, 11.6f);
  EXPECT_EQ(s.a.level(), opencr_battery::OK);
  s.clear();
  s.run(60000, 11.6f);
  EXPECT_TRUE(s.beeps.empty()) << "정상으로 돌아오면 조용";
}

// 배터리를 빼면 조용 · 다시 꽂으면 잔량 비프부터
TEST(BatteryAlarm, ReplugRestarts)
{
  Sim s;
  s.run(5000, 12.4f);
  EXPECT_EQ(s.count(BatteryAlarm::SOC_HZ), 4);
  s.clear();
  s.run(10000, 0.3f);
  EXPECT_TRUE(s.beeps.empty());
  EXPECT_EQ(s.a.level(), opencr_battery::NONE);
  s.run(8000, 11.6f);
  EXPECT_EQ(s.count(BatteryAlarm::SOC_HZ), 3) << "다시 꽂은 배터리의 잔량 (평균이 따라잡은 뒤)";
  EXPECT_EQ(s.count(BatteryAlarm::CRIT_HZ) + s.count(BatteryAlarm::WARN_HZ), 0) << "꽂는 중 오르는 전압으로 경고하지 않음";
}

// 펌웨어 입력처럼 전압이 서서히 오르며 꽂혀도 (τ 0.1 s 필터) 잔량은 다 오른 값으로
TEST(BatteryAlarm, RampedPlugCountsSettledVoltage)
{
  Sim s;
  s.run(3000, 0.3f);
  float v = 0.3f;
  for (int k = 0; k < 800; ++k) {   // 8 s: 펌웨어 g_battery_v 와 같은 1 차 필터 (α 0.1 / 10 ms)
    v = 0.9f * v + 0.1f * 12.4f;
    s.run(10, v);
  }
  EXPECT_EQ(s.count(BatteryAlarm::SOC_HZ), 4);
  EXPECT_EQ(static_cast<int>(s.beeps.size()), 4);
}

// millis() 넘침 (약 49.7 일) 을 지나도 같은 동작
TEST(BatteryAlarm, MillisWrapAround)
{
  Sim s(0xFFFFFFFFu - 3000u);
  s.run(6000, 12.4f);
  EXPECT_EQ(s.count(BatteryAlarm::SOC_HZ), 4);
  s.clear();
  s.run(8000, 10.2f);
  EXPECT_GE(s.count(BatteryAlarm::CRIT_HZ), 3);
}

// 호스트 진단 판정: 주행 (전압 필수) · 팔 (예전 펌웨어 0 → 보고 없음)
TEST(BatteryStatus, EvaluateBothBoards)
{
  using opencr_hardware::evaluate_battery;
  const double warn = BATTERY_WARN_V, err = BATTERY_CRIT_V;
  auto w = evaluate_battery(12.4f, warn, err, 3, true);
  EXPECT_TRUE(w.reported);
  EXPECT_EQ(w.level, 0);
  EXPECT_GT(w.percent, 85.f);
  EXPECT_EQ(w.state, "정상");
  EXPECT_EQ(evaluate_battery(11.0f, warn, err, 3, true).level, 1);
  EXPECT_EQ(evaluate_battery(10.3f, warn, err, 3, true).level, 2);
  EXPECT_EQ(evaluate_battery(0.f, warn, err, 3, true).level, 2) << "주행 보드는 예전처럼 0 V 면 전압 위험";
  auto a0 = evaluate_battery(0.f, warn, err, 3, false);
  EXPECT_FALSE(a0.reported);
  EXPECT_EQ(a0.level, 0) << "예전 팔 펌웨어 (0) 는 오류로 보지 않음";
  EXPECT_EQ(a0.state, "보고 없음");
  EXPECT_EQ(evaluate_battery(10.3f, warn, err, 3, false).level, 2) << "팔 배터리도 위험이면 오류 → mission 정지";
  EXPECT_EQ(opencr_hardware::battery_summary("팔", a0), "팔 보고 없음");
  const std::string sw = opencr_hardware::battery_summary("주행", w);
  EXPECT_NE(sw.find("12.40 V"), std::string::npos) << sw;
  EXPECT_NE(sw.find('%'), std::string::npos) << sw;
  const std::string sl = opencr_hardware::battery_summary("팔", evaluate_battery(11.0f, warn, err, 3, false));
  EXPECT_NE(sl.find("전압 낮음"), std::string::npos) << sl;
}
