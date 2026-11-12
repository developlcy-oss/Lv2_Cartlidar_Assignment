// GraspDetector — 집게 (6 번째 칸 gripper_roll, 실물 ID 15) 측정 위치로 퍽을 쥐었는지 판정 (manipulator_node gripper.*, #201)
//
// 왜: 위치 모드 + 펌웨어 GripGuard 라 퍽에 막히면 손가락이 퍽 폭에서 멈춘다. 예전 판정은 "닫힘보다 grasp_margin 넘게 덜 닫힘" 한쪽만
//     봐서, 그리퍼가 안 닫히고 열린 채 (모터 고장 · 명령 무시) 여도 쥠으로 봤다.
// 어떻게: 측정 위치가 [grasp_min, 닫힘 − grasp_margin] 안이면 쥠 범위.
//     - 더 닫힘 (닫힘 쪽 밖) = 빈손, 덜 닫힘 (열림 쪽 밖) = 더 큰 물체 (상자 등) 에 막힘 또는 안 닫힘
//     - 연속 신호 (정책 · 진단, /manipulator/grasped): 범위 안이 on_time 동안 이어지면 1, 범위 밖이 off_time 동안 이어지면 0
//       (모의 정책 판정과 같은 꼴 — 30 Hz 에서 켜짐 8 스텝 ≈ 0.27 s · 꺼짐 3 스텝 ≈ 0.1 s, #201)
//     문턱은 실측 전 값 (★ #201: 퍽 쥠 · 빈손 · 큰 물체 막힘 각 10 회로 정함)
#pragma once

#include <algorithm>
#include <cmath>

namespace control
{

// 측정 위치가 쥠 범위 안인지. lo · hi 는 rad (lo < hi), 측정값이 유한하지 않으면 false
inline bool gripInGraspRange(double q, double lo, double hi)
{
  return std::isfinite(q) && q >= lo && q <= hi;
}

// 쥠 범위 상한: 닫힘 끝 (close − margin) 과 마지막 닫기 명령 (command − cmd_margin) 중 작은 쪽.
//   정책 집기는 끝까지 닫지 않고 덜 닫는 명령 (예: 손가락 −0.0075 m ≈ 0.64 rad) 을 보내서, 빈손이어도 그 명령 자리에서 멈춘다.
//   닫힘 끝만 보면 그 자리를 쥠으로 잘못 봄 → 명령보다 cmd_margin 넘게 덜 닫힌 채 멈춰야 (= 막혀야) 쥠
inline double graspUpperBound(double close, double margin, double command, double cmd_margin)
{
  return std::min(close - margin, command - cmd_margin);
}

// 범위 안 · 밖이 일정 시간 이어져야 상태를 바꾸는 쥠 신호
class GraspHysteresis
{
public:
  GraspHysteresis(double on_time, double off_time)
  : on_time_(on_time), off_time_(off_time) {}

  // in_range: 이번 측정이 범위 안인지, dt: 지난 갱신 뒤 시간 (s). 반환 = 지금 쥠 상태
  bool update(bool in_range, double dt)
  {
    dt = std::max(0.0, dt);
    if (in_range == grasped_) {
      t_ = 0.0;   // 지금 상태와 같은 측정 → 바꾸려던 시간을 지움
      return grasped_;
    }
    t_ += dt;
    if (t_ + 1e-9 >= (grasped_ ? off_time_ : on_time_)) {
      grasped_ = in_range;
      t_ = 0.0;
    }
    return grasped_;
  }

  void reset() {grasped_ = false; t_ = 0.0;}
  bool grasped() const {return grasped_;}

private:
  double on_time_, off_time_;
  bool grasped_{false};
  double t_{0.0};   // 지금 상태와 반대인 측정이 이어진 시간
};

}  // namespace control
