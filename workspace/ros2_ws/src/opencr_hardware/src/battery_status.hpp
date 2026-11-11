// battery_status.hpp — 보드별 배터리 전압 → 진단 수준 · 잔량 % (OpenCRHardware 안에서만 씀, 설치하지 않음. 2026-11-11)
//   잔량 표는 펌웨어와 같은 opencr_firmware/battery_alarm.h 를 그대로 쓴다 (CMake 가 include 경로에 넣음).
//   수준 기준은 URDF <param> battery_warn_v · battery_error_v (= 펌웨어 config.h BATTERY_WARN_V · BATTERY_CRIT_V, test_robot_description 이 비교).
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

#include "battery_alarm.h"

namespace opencr_hardware
{

// 이보다 낮은 값은 "전압을 보내지 않음" — 예전 팔 펌웨어 (STATE battery_v 0) · USB 만 연결
inline constexpr float BATTERY_REPORTED_MIN_V = 1.0f;

struct BatteryReport
{
  bool reported = false;   // 이 보드가 전압을 보냄
  float voltage = 0.f;
  float percent = 0.f;     // 잔량 추정 % (LiPo 휴지 전압 표 — 부하 중에는 낮게 나옴)
  uint8_t level = 0;       // diagnostic_msgs/DiagnosticStatus: 0 OK · 1 WARN · 2 ERROR
  std::string state;       // 정상 · 전압 낮음 · 전압 위험 · 보고 없음
};

// required: 전압이 없을 때도 기준으로 판정할지. 주행 보드 true (예전 동작 — 0 V 면 전압 위험).
//   팔 보드 false: 예전 펌웨어는 0 을 보내므로 "보고 없음" (수준 OK) 으로 넘긴다 — 팔 전원이 정말 없으면 모터 응답 없음으로 따로 잡힘
inline BatteryReport evaluate_battery(float v, double warn_v, double error_v, int cells, bool required)
{
  BatteryReport r;
  r.voltage = v;
  r.reported = v >= BATTERY_REPORTED_MIN_V;
  if (!r.reported && !required) {
    r.state = "보고 없음";
    return r;
  }
  r.percent = opencr_battery::soc_percent(v, cells);
  if (v < error_v) {
    r.level = 2;
    r.state = "전압 위험";
  } else if (v < warn_v) {
    r.level = 1;
    r.state = "전압 낮음";
  } else {
    r.state = "정상";
  }
  return r;
}

// 진단 메시지 한 조각: "주행 12.45 V 93 %" · "팔 11.02 V 9 % 전압 낮음" · "팔 보고 없음"
inline std::string battery_summary(const char * name, const BatteryReport & r)
{
  char b[128];
  if (!r.reported && r.level == 0) {
    std::snprintf(b, sizeof(b), "%s 보고 없음", name);
  } else {
    std::snprintf(b, sizeof(b), "%s %.2f V %.0f %%%s%s", name, r.voltage, r.percent,
      r.level ? " " : "", r.level ? r.state.c_str() : "");
  }
  return b;
}

}  // namespace opencr_hardware
