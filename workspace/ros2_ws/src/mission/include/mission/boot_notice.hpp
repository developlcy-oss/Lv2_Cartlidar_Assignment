// 기동 안내 (2026-11-20): 준비 확인 표에서 "E-stop 만 남음" · "기동 완료" 판정 (헤더만)
//
// E-stop 은 기동 때 일부러 잠긴 채 시작하고 RC-100 리모컨 2+3 으로만 풀린다.
// 1 안전 · 팔 정리 (startup_done) 처럼 E-stop 이 풀려야 OK 가 되는 항목을 빼고 나머지가 모두 OK 이면
// WAIT_ESTOP → mission_node 가 "리모컨으로 해제" 안내를 띄운다.
#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include "mission/readiness.hpp"

namespace mission
{

enum class BootPhase { NOT_READY, WAIT_ESTOP, ALL_READY };

// estop_gated: E-stop 이 걸려 있는 동안 OK 가 될 수 없는 항목 이름 (안전 항목 · 팔 정리)
inline BootPhase boot_phase(
  const std::vector<ReadyItem> & items, bool estop_active,
  const std::vector<std::string> & estop_gated)
{
  if (all_ready(items)) {return BootPhase::ALL_READY;}
  if (!estop_active) {return BootPhase::NOT_READY;}
  for (const auto & i : items) {
    if (i.state == ReadyState::OK) {continue;}
    if (std::find(estop_gated.begin(), estop_gated.end(), i.name) == estop_gated.end()) {
      return BootPhase::NOT_READY;
    }
  }
  return BootPhase::WAIT_ESTOP;
}

// 안내를 낼지: 상태가 바뀌면 (처음 NOT_READY 제외) 한 번, WAIT_ESTOP 이 이어지면 repeat_s 마다 다시
inline bool boot_notice_due(BootPhase prev, BootPhase cur, double since_last_s, double repeat_s)
{
  if (cur != prev) {return cur != BootPhase::NOT_READY;}
  return cur == BootPhase::WAIT_ESTOP && repeat_s > 0.0 && since_last_s >= repeat_s;
}

inline const char * boot_notice_text(BootPhase p)
{
  switch (p) {
    case BootPhase::WAIT_ESTOP:
      return "★ 준비 완료 (컨트롤러 · 액션 서버 · 위치 · Nav2 OK) — 주변 확인 후 리모컨 2+3 으로 E-stop 해제 "
             "(OpenCR 보드 리셋 버튼 누르지 말 것: 재부팅돼 포트가 바뀜)";
    case BootPhase::ALL_READY:
      return "★ 기동 완료 — 모든 항목 OK, /mission/start 가능";
    default:
      return "";
  }
}

}  // namespace mission
