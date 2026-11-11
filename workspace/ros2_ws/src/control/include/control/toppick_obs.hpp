// TopPick 강화학습 정책 (policies.yaml toppick_tilt) 관측 36 조립 · 그리퍼 단위 대응 — ROS 없는 순수 C++ (manipulator_node 가 씀)
//   Isaac toppick_env.py _get_observations 와 같은 값 · 순서 (이름은 policies.yaml observation 이름):
//     joint_pos_rel 5 (joint1~5 − READY_POSE), joint_vel 5, finger_pos 1 (gripper_left_joint m), puck_pos 3 (인식 퍽 중심 base_link),
//     grasp_pos 3 (잡는 점), puck_minus_grasp 3, approach_axis 3, close_axis 3, neighbor_rel 2 (옆 퍽 − 퍽 xy, 없으면 NB_ABSENT),
//     is_box 1, roll_cos_sin 2 (닫는 축 − 목표 닫는 방향), prev_action 4, tilted 1
//   관절은 URDF (모델) 값, 좌표는 base_link (arm_diff_ik.hpp 와 같음)
// 그리퍼: 학습 = 손가락 미끄럼 관절 (m, 벌림 mm = 41.6 + 2000 x 값), 실물 = gripper_roll 모터 각 (rad) → 두 끝점 선형 대응 (GripperMap).
//   기본 끝점 = 실물 열림 −0.764 rad ↔ URDF 손가락 한계 0.020 m, 닫힘 +0.923 ↔ −0.011 (2026-11-05 가정, 실측 뒤 manipulator.yaml 에서 고침)

#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "control/arm_diff_ik.hpp"

namespace control
{

constexpr Joints5 kTopPickReadyPose{0.0, 0.48, -0.76, 1.84, 0.0};   // 관측 기준 (toppick_env_cfg READY_POSE, 기울임 모드도 이 기준)
constexpr double kNbAbsentX = 0.0;   // 옆 퍽 없음 관측 (toppick_env_cfg NB_ABSENT)
constexpr double kNbAbsentY = 0.5;
// 2026-11-11 세운 퍽 45° 집기 k7 (policies.yaml sidepick45_k7, Isaac Waffle-SidePick45Chain-Direct sidepick_env._get_observations):
//   관측 33 = 같은 식에서 joint_pos_rel 기준 = READY_POSE_45, 옆 퍽 없음 = (0, 0), roll_cos_sin · tilted 없음 (yaml 에 없는 이름은 안 씀)
constexpr Joints5 kSidePick45ReadyPose{0.0, -0.338, 0.559, 0.564, 0.0};

// 정책마다 다른 관측 기준 (manipulator.yaml <정책 이름>.obs_ref_pose · neighbor_absent)
struct TopPickLayout
{
  Joints5 ref_pose{kTopPickReadyPose};
  double nb_absent_x{kNbAbsentX};
  double nb_absent_y{kNbAbsentY};
};

struct GripperMap
{
  double rad_open{-0.764};   // 실물 gripper_roll 열림 · 닫힘 명령 (manipulator.yaml gripper.open_position · close_position)
  double rad_close{0.923};
  double m_open{0.020};      // 그 두 자리에 대응하는 학습 손가락 값 (m)
  double m_close{-0.011};
  double toMeters(double rad) const {return m_open + (rad - rad_open) * (m_close - m_open) / (rad_close - rad_open);}
  double toRad(double m) const {return rad_open + (m - m_open) * (rad_close - rad_open) / (m_close - m_open);}
};

struct TopPickState
{
  Joints5 q{};                               // 측정 관절 (모델 값)
  Joints5 qd{};                              // 관절 속도 (rad/s)
  double finger_m{0.0};                      // 손가락 (학습 단위 m, GripperMap 으로 바꾼 값)
  Eigen::Vector3d puck{Eigen::Vector3d::Zero()};   // 인식 퍽 중심 (base_link)
  std::optional<Eigen::Vector3d> neighbor;   // 옆 퍽 중심 (base_link), 없으면 nullopt
  bool is_box{false};
  std::array<float, 4> prev_action{0.f, 0.f, 0.f, 1.f};   // 첫 스텝 = 학습 리셋 값 (벌림)
  bool tilted{true};
};

// 이름 → 값 (policies.yaml observation 이름과 같게). ObservationBuilder 가 yaml 순서로 이어 붙임
std::map<std::string, std::vector<float>> topPickObservation(
  const ArmDiffIK & ik, const TopPickState & s, const TopPickLayout & layout = TopPickLayout{});

// 이 함수가 만드는 관측 이름 · 크기 (노드가 policies.yaml 항목을 확인할 때)
const std::map<std::string, int> & topPickObservationSizes();

}  // namespace control
