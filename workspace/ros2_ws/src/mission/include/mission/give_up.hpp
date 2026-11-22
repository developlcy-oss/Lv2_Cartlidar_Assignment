// 재시도를 다 쓴 단계 처리 (R1, 2026-11-14): 퍽 하나만 건너뛰기 · HOME 복귀 자리 찾기 (헤더만, 단계 원문으로 판단)
//
// 계획기 단계 (mission_planner/mission_lines.py) 의 퍽 하나 집기 묶음:
//   [skill drive turn_<p|m><deg>] · skill manipulator target_look:… · skill manipulator toppick_tilt ·
//   skill manipulator pallet_load_<열>_<칸>_tilt · [skill drive turn_0]
// 놓기 묶음: skill manipulator pallet_unload_<열>_<칸>_tilt · skill manipulator place_<L>_<dx>_<dy>_tilt
// 복귀: goto <HOME x y yaw> · wait <s> (맨 끝)
// 빈손 pallet_load (운반 중 떨어뜨림, 실패 문구에 kEmptyHand) 는 집기 묶음 실패처럼: target_look 부터 다시, 다 쓰면 그 퍽만 건너뜀
#pragma once

#include <algorithm>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace mission
{

// "skill manipulator toppick_tilt 40 mode=…" -> {"skill", "manipulator", "toppick_tilt", …}
inline std::vector<std::string> step_tokens(const std::string & text)
{
  std::istringstream in(text);
  std::vector<std::string> t;
  std::string w;
  while (in >> w) {t.push_back(w);}
  return t;
}

// skill <who> <name> 이고 name 이 prefix 로 시작하면 name, 아니면 빈 문자열
inline std::string skill_name(const std::string & text, const std::string & who, const std::string & prefix)
{
  const auto t = step_tokens(text);
  if (t.size() < 3 || t[0] != "skill" || t[1] != who || t[2].rfind(prefix, 0) != 0) {return "";}
  return t[2];
}

// pallet_load_1_2_tilt · pallet_unload_1_2_tilt -> "1_2" (아니면 빈 문자열)
inline std::string pallet_slot(const std::string & text, const std::string & kind)
{
  const std::string n = skill_name(text, "manipulator", "pallet_" + kind + "_");
  const std::string head = "pallet_" + kind + "_", tail = "_tilt";
  if (n.empty() || n.size() <= head.size() + tail.size() || n.compare(n.size() - tail.size(), tail.size(), tail) != 0) {
    return "";
  }
  return n.substr(head.size(), n.size() - head.size() - tail.size());
}

// 집기 묶음 안 단계 (pallet_load 앞): 제자리 회전 (turn_0 제외) · target_look · toppick_tilt
inline bool is_pick_group_step(const std::string & text)
{
  const std::string turn = skill_name(text, "drive", "turn_");
  if (!turn.empty()) {return turn != "turn_0";}
  return !skill_name(text, "manipulator", "target_look:").empty() ||
         !skill_name(text, "manipulator", "toppick_tilt").empty();
}

struct PickSkip
{
  std::string slot;     // 빈 칸이 되는 팔레트 칸 "열_칸"
  size_t resume{0};     // 다시 시작할 단계 = pallet_load 다음 (turn_0 이 있으면 그것, 퍽 없이 정렬 방향으로 되돌림)
};

// failed 단계가 집기 묶음 안이면 같은 묶음의 pallet_load 를 찾아 건너뛸 범위를 줌.
// 묶음 밖 단계 (주행 · 정렬 · 놓기 · pallet_load 자체) 이거나 pallet_load 가 없으면 nullopt
inline std::optional<PickSkip> pick_skip(const std::vector<std::string> & texts, size_t failed)
{
  if (failed >= texts.size() || !is_pick_group_step(texts[failed])) {return std::nullopt;}
  for (size_t i = failed + 1; i < texts.size(); ++i) {
    const std::string slot = pallet_slot(texts[i], "load");
    if (!slot.empty()) {return PickSkip{slot, i + 1};}
    if (!is_pick_group_step(texts[i])) {return std::nullopt;}
  }
  return std::nullopt;
}

// manipulator 실패 문구 속 빈손 표시 (manipulator_node.cpp toppick 들기 뒤 · pallet_load 시작 검사, 2026-11-16)
inline constexpr const char * kEmptyHand = "empty hand";

// pallet_load 가 빈손으로 실패 = 집은 퍽을 운반 중 떨어뜨림 -> 집기 묶음 실패처럼 다룸 (다시 집기 · 건너뛰기)
inline bool is_empty_hand_load(const std::string & text, const std::string & reason)
{
  return !pallet_slot(text, "load").empty() && reason.find(kEmptyHand) != std::string::npos;
}

// 다시 잴 단계: failed 바로 앞으로 이어진 manipulator · drive skill 안의 가장 가까운 target_look (사이에 주행 · 정렬이 끼면 없음)
inline std::optional<size_t> retry_look(const std::vector<std::string> & texts, size_t failed)
{
  for (size_t i = std::min(failed, texts.size()); i-- > 0;) {
    const auto t = step_tokens(texts[i]);
    if (t.empty() || t[0] != "skill") {return std::nullopt;}
    if (!skill_name(texts[i], "manipulator", "target_look:").empty()) {return i;}
  }
  return std::nullopt;
}

// 빈손 pallet_load 의 재시도를 다 쓰면: 그 칸만 빈 칸, 바로 다음 단계 (turn_0 이 있으면 그것) 부터
inline std::optional<PickSkip> load_skip(const std::vector<std::string> & texts, size_t failed, const std::string & reason)
{
  if (failed >= texts.size() || !is_empty_hand_load(texts[failed], reason)) {return std::nullopt;}
  return PickSkip{pallet_slot(texts[failed], "load"), failed + 1};
}

// 놓기 묶음 건너뛰기: unload 단계 idx 다음이 place_ 면 그 뒤, 아니면 idx + 1
inline size_t after_unload(const std::vector<std::string> & texts, size_t idx)
{
  if (idx + 1 < texts.size() && !skill_name(texts[idx + 1], "manipulator", "place_").empty()) {return idx + 2;}
  return idx + 1;
}

// ---- 이동 실패 시 그곳만 건너뛰기 (on_fail.skip_nav, 11-21, review_arch ②) ----
// 한 "곳" = goto · navigate 하나부터 다음 goto · navigate 앞까지 (그 자리에서 하는 접근 · 집기 · 놓기 · verify)
inline bool is_nav_step(const std::string & text)
{
  const auto t = step_tokens(text);
  return !t.empty() && (t[0] == "goto" || t[0] == "navigate");
}

// 그곳에 서기 위한 단계 (손은 비어 있음): 주행 · 접근 · 상자 재기 · 중앙 맞춤.
//   check_marker 는 첫 pallet_unload 뒤 (퍽을 쥠) 라 넣지 않음
inline bool is_positioning_step(const std::string & text)
{
  const auto t = step_tokens(text);
  if (t.empty()) {return false;}
  const std::string & k = t[0];
  return k == "goto" || k == "navigate" || k == "approach" || k == "approach_box" || k == "approach_view" ||
         k == "center_box" || k == "measure_start" || k == "measure_box";
}

struct NavSkip
{
  size_t resume{0};                       // 다음 goto · navigate (복귀 줄일 수 있음)
  std::vector<std::string> pick_slots;    // 그곳에서 실을 칸 -> 빈 칸 (뒤의 그 칸 꺼내기 · 놓기도 건너뜀)
  std::vector<std::string> place_slots;   // 그곳에서 꺼내 놓을 칸 -> 퍽은 팔레트에 남음
  std::vector<std::string> verifies;      // 건너뛰는 verify 의 장소 (L1 …)
};

// failed 가 그곳에 서는 단계 (is_positioning_step) 이고, 그곳의 goto 와 failed 사이에 팔 · 집기 단계가 없으면
//   (손이 비어 있음) 다음 goto · navigate 까지 건너뛸 범위. 그 밖 · 뒤에 goto 가 없으면 (복귀 줄 자체) nullopt
inline std::optional<NavSkip> nav_skip(const std::vector<std::string> & texts, size_t failed)
{
  if (failed >= texts.size() || !is_positioning_step(texts[failed])) {return std::nullopt;}
  if (!is_nav_step(texts[failed])) {
    // 같은 곳의 goto 까지 거슬러 가며 팔 · 집기 단계가 끼었는지 본다
    bool found = false;
    for (size_t i = failed; i-- > 0;) {
      if (is_nav_step(texts[i])) {found = true; break;}
      if (!is_positioning_step(texts[i])) {return std::nullopt;}
    }
    if (!found) {return std::nullopt;}
  }
  NavSkip s;
  for (size_t i = failed + 1; i < texts.size(); ++i) {
    if (is_nav_step(texts[i])) {
      s.resume = i;
      return s;
    }
    const std::string load = pallet_slot(texts[i], "load");
    if (!load.empty()) {s.pick_slots.push_back(load);}
    const std::string unload = pallet_slot(texts[i], "unload");
    if (!unload.empty()) {s.place_slots.push_back(unload);}
    const auto t = step_tokens(texts[i]);
    if (t.size() >= 2 && t[0] == "verify") {s.verifies.push_back(t[1]);}
  }
  return std::nullopt;
}

// 연속 이동 실패 (이번 것 포함) consecutive 곳째: max 에 이르면 건너뛰지 않고 복귀.
//   max 2 = 한 곳은 건너뛰고 연속 둘째 실패에서 HOME 복귀. max <= 1 = 건너뛰지 않음
inline bool nav_skip_allowed(int consecutive, int max_consecutive)
{
  return consecutive >= 1 && consecutive < max_consecutive;
}

// 복귀 줄: 맨 끝 wait 바로 앞 goto (계획기 return). 없으면 nullopt (계획 전 · tag 모드)
inline std::optional<size_t> home_index(const std::vector<std::string> & texts)
{
  if (texts.size() < 2) {return std::nullopt;}
  const auto last = step_tokens(texts.back());
  if (last.empty() || last[0] != "wait") {return std::nullopt;}
  const auto prev = step_tokens(texts[texts.size() - 2]);
  if (prev.empty() || prev[0] != "goto") {return std::nullopt;}
  return texts.size() - 2;
}

}  // namespace mission
