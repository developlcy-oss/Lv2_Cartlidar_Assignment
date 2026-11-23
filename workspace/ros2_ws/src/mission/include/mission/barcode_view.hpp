// 바코드 관찰 자리 · 빠져나오기 단계 만들기 (헤더만, 2026-11-23)
//
// 미션 모드 순서: unfold → [관찰 자리로 이동] → observe → [빠져나옴] → survey · plan …
//   view_pose [x, y, yaw_deg] (map) 이 있으면 observe 앞에 `goto x y yaw_rad` (goto 가 주행 자세 DRIVE 로 이동)
//   observe 뒤: 먼저 `pose <travel>` (barcode_look 에서 팔을 접음) 그 다음
//     leave_pose [x, y, yaw_deg] 가 있으면 `goto` (drive_node 가 정차 중 먼저 곧게 후진, #157), 아니면
//     leave_back_m > 0 이면 `backup <m> <speed>` (Nav2 BackUp), 둘 다 없으면 빠져나오지 않음
//   view_pose 가 비면 이동도 빠져나옴도 없이 예전 순서 (observe 만).
#pragma once

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "mission/give_up.hpp"

namespace mission
{

struct BarcodeView
{
  std::vector<double> view_pose;   // [x, y, yaw_deg] 빈 배열 = 이동 안 함
  std::vector<double> leave_pose;  // [x, y, yaw_deg] 빈 배열 = 안 씀
  double leave_back_m{0.0};        // > 0 이면 후진 거리 (leave_pose 가 없을 때)
  double backup_speed{0.10};       // m/s
};

// 설정 검사. 문제 있으면 문장, 없으면 빈 문자열
inline std::string barcode_view_error(const BarcodeView & b)
{
  if (!b.view_pose.empty() && b.view_pose.size() != 3) {return "observe.view_pose must be [] or [x, y, yaw_deg]";}
  if (!b.leave_pose.empty() && b.leave_pose.size() != 3) {return "observe.leave_pose must be [] or [x, y, yaw_deg]";}
  if (!(b.leave_back_m >= 0.0 && b.leave_back_m <= 1.0)) {return "observe.leave_back_m must be in [0, 1] m";}
  if (!(b.backup_speed > 0.0 && b.backup_speed <= 0.3)) {return "observe.backup_speed must be in (0, 0.3] m/s";}
  return "";
}

inline std::string goto_line(const std::vector<double> & p)
{
  std::ostringstream o;
  o << "goto " << p[0] << " " << p[1] << " " << p[2] * M_PI / 180.0;
  return o.str();
}

// observe 앞 (이동) · 뒤 (빠져나옴) 단계 줄
inline std::vector<std::string> barcode_pre_lines(const BarcodeView & b)
{
  if (b.view_pose.size() != 3) {return {};}
  return {goto_line(b.view_pose)};
}

inline std::vector<std::string> barcode_post_lines(const BarcodeView & b, const std::string & travel_pose)
{
  if (b.view_pose.size() != 3) {return {};}   // 이동 안 했으면 예전과 같게
  std::vector<std::string> v;
  if (b.leave_pose.size() == 3) {
    v.push_back("pose " + travel_pose);
    v.push_back(goto_line(b.leave_pose));
  } else if (b.leave_back_m > 0.0) {
    std::ostringstream o;
    o << "backup " << b.leave_back_m << " " << b.backup_speed;
    v.push_back("pose " + travel_pose);
    v.push_back(o.str());
  }
  return v;
}

// observe 뒤 빠져나오기 단계인가: [observe, pose <팔 접기>, backup | goto] 꼴의 마지막.
//   바코드 · 신호를 이미 읽은 뒤라 이 단계가 실패해도 미션은 계속 (pose 는 팔이 펴진 채 이동하면 안 되므로 해당 없음).
//   observe 앞 관찰 자리 goto 는 pose 가 앞에 없어 해당 없음.
inline bool is_leave_step(const std::vector<std::string> & texts, size_t idx)
{
  if (idx < 2 || idx >= texts.size()) {return false;}
  const auto t = step_tokens(texts[idx]);
  if (t.empty() || (t[0] != "backup" && t[0] != "goto")) {return false;}
  const auto p = step_tokens(texts[idx - 1]), o = step_tokens(texts[idx - 2]);
  return !p.empty() && p[0] == "pose" && !o.empty() && o[0] == "observe";
}

// 실패한 단계를 건너뛰고 계속할지 (다음 단계가 있을 때만)
inline bool leave_failure_nonfatal(const std::vector<std::string> & texts, size_t failed)
{
  return is_leave_step(texts, failed) && failed + 1 < texts.size();
}

}  // namespace mission
