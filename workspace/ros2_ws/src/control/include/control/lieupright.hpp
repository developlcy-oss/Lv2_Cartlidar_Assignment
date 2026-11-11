// 누운 직육면체 세워 놓기 정책 (policies.yaml lie_upright, Isaac Waffle-LieUprightB7-Direct = r2g2, #192 · #193) — ROS 없는 순수 C++
//   manipulator_node 가 30 Hz 로: 관측 45 조립 → 정책 → 행동 6 → 집게 자세 (β 숙임 · ρ 롤) + 잡는 점 변위 → 감쇠 최소제곱 IK → 관절 목표.
//   학습 환경 lieupright_env.py (_pre_physics_step · _ik_pose · target_R_base · _get_observations · _stand_spot · _update_hold) 와 같은 식 · 순서.
//
// 좌표 · 기구 = arm_diff_ik.hpp 와 같음 (base_link, 관절 = URDF 모델 값, 잡는 점 = gripper_roll_link + (0.14, 0, 0)).
// 행동 6 = (잡는 점 변위 x · y · z [-1, 1] x cart_scale m, β 변화 x beta_scale rad, ρ 변화 x rho_scale rad, 집게 > 0 벌림 · ≤ 0 닫음).
//   β · ρ 는 적분 상태 (시작 β = π/4 · ρ = 0). 집게 방향 a = −cos β z + sin β h (h = J1 축 → 잡는 점 수평 단위),
//   t = z x h, u0 = a x t, 닫는 축 c = cos ρ t + sin ρ u0, u = −sin ρ t + cos ρ u0. 목표 자세 = 열 (a, c, u).
//   자세 오차 = 목표 R · 지금 Rᵀ 의 축-각 (Isaac quat_box_minus). 오차 · 야코비안 각속도 줄에 rot_weight.
// 쥠 (관측 35): 손가락 값이 grasp 범위 안이 연속 hold_on 스텝이면 1, 아니면 연속 hold_off 스텝 뒤 0 (학습 obs_hold_mode finger 와 같은 규칙).
// 세울 자리 D: 상자 가장자리 spot_edge 안 spot_step 격자, J1 축 거리 spot_r, 옆 퍽 spot_nb 밖, 점수 |r − r_best| − nb_w · min(옆 퍽 거리, nb_cap) 최소.
//   옆 퍽이 없으면 '있었다면' 자리 (같은 상자 다른 칸) 로 계산 (학습과 같음).
// 시험: test/test_lieupright.cpp (학습 환경 식으로 낸 값과 비교)

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

constexpr Joints5 kLieReadyPose45{0.0, -0.338, 0.559, 0.564, 0.0};   // 관측 기준 = 시작 자세 (sidepick_env_cfg READY_POSE_45)

struct LieUprightParams
{
  double cart_scale{0.005};     // [m / 행동 1]
  double beta_scale{0.05};      // [rad / 행동 1]
  double rho_scale{0.06};
  double beta_min{-0.15};
  double beta_max{1.75};
  double rho_max{2.6};
  double beta0{M_PI / 4};       // 시작 β (45° 아래)
  double rot_weight{1.0};       // rot_hold_weight
  double damping{1e-4};         // ik_damping
  double lead_max{0.25};        // target_lead_max [rad]
  double finger_open{0.010};    // 학습 손가락 값 [m] (gripper_open)
  double finger_close{-0.011};  // FINGER_CLOSE
  double grasp_lo{-0.0108};     // 쥠 손가락 범위 (grasp_finger_range)
  double grasp_hi{-0.003};
  int hold_on{8};               // [스텝]
  int hold_off{3};
  // 세울 자리 (_stand_spot)
  double box_size{0.205};
  double spot_edge{0.030};
  double spot_step{0.0145};
  double spot_r_min{0.27};
  double spot_r_max{0.32};
  double spot_r_best{0.285};
  double spot_nb{0.07};
  double spot_nb_w{0.2};
  double spot_nb_cap{0.12};
};

// 목표 집게 자세 (열 = 집게 방향 a, 닫는 축 c, 손가락 높이 방향 u), base_link
Eigen::Matrix3d lieTargetRotation(const ArmDiffIK & ik, const Eigen::Vector3d & grasp, double beta, double rho);

// 한 스텝: 행동 (자른 값) 으로 β · ρ 를 갱신하고 새 관절 목표. target_prev = 직전 목표 (첫 스텝은 측정값), q_meas = 측정 관절
Joints5 lieApplyAction(
  const ArmDiffIK & ik, const LieUprightParams & p, const Joints5 & target_prev, const Joints5 & q_meas,
  const std::array<float, 6> & action, double & beta, double & rho);

// 관절 속도 제한이 있는 한 스텝: 관절 목표 변화가 max_step 을 넘으면 행동 (잡는 점 변위 · β · ρ 변화) 을 같은 비율 s 로 줄여 다시 풂.
//   관절마다 따로 자르면 잡는 점이 정책이 뜻한 방향과 다르게 움직이고, 퍽이 손 안에서 덜 선 채 벌리는 실패가 늘었음
//   (Isaac r2e 접촉 없는 성공: 관절마다 자름 3.0 rad/s 79.4 % · 2.0 4.5 %, 비율로 줄임 3.0 95.4 % · 2.0 90.6 %, 제한 없음 96.4 %).
//   관측 β 와 실제 집게 자세 차이는 어느 방식이든 작았음 (최대 0.06 rad). 비율로 줄이면 β · ρ 는 줄인 만큼만 감.
//   s 는 이분법 (iters 번), 그래도 넘는 몫은 관절별로 자름 (안전). 쓴 비율을 scale_used 에 (1 = 줄이지 않음)
Joints5 lieApplyActionLimited(
  const ArmDiffIK & ik, const LieUprightParams & p, const Joints5 & target_prev, const Joints5 & q_meas,
  const std::array<float, 6> & action, double & beta, double & rho, double max_step, double * scale_used = nullptr, int iters = 8);

// 지금 집게 자세 (FK) → β · ρ (학습 set_beta_rho_from, 범위로 자름). 끝난 뒤 위로 뺄 때 자세를 그대로 두려고 씀
void lieBetaRhoFrom(const ArmDiffIK & ik, const LieUprightParams & p, const Joints5 & q, double & beta, double & rho);

// 쥠 판정 (손가락 범위 + 히스테리시스)
struct LieHoldFilter
{
  int on{0};
  int off{0};
  bool state{false};
  bool update(double finger_m, const LieUprightParams & p);
};

// 세울 자리 xy (base_link). box_c · box_yaw = 상자 중심 · 요, nb = 옆 퍽 (없으면 '있었다면' 자리) xy
Eigen::Vector2d lieStandSpot(
  const LieUprightParams & p, const Eigen::Vector2d & box_c, double box_yaw, const Eigen::Vector2d & nb, double joint1_x);

struct LieUprightState
{
  Joints5 q{};                       // 측정 관절 (모델 값)
  Joints5 qd{};                      // rad/s
  double finger_m{0.0};              // 학습 손가락 단위 (m)
  Eigen::Vector3d puck{Eigen::Vector3d::Zero()};   // 인식 누운 퍽 중심 (base_link), z = 상자 윗면 + 15 mm
  double long_yaw{0.0};              // 긴 축 요 (base_link, 180° 대칭)
  Eigen::Vector3d spot{Eigen::Vector3d::Zero()};   // 세울 자리 D (세운 퍽 중심, base_link)
  std::optional<Eigen::Vector2d> neighbor;          // 옆 퍽 xy (보일 때만)
  Eigen::Vector2d box_c{Eigen::Vector2d::Zero()};
  double box_yaw{0.0};
  bool held{false};
  double beta{M_PI / 4};
  double rho{0.0};
  std::array<float, 6> action{0.f, 0.f, 0.f, 0.f, 0.f, 1.f};   // 직전 행동 (자른 값, 놓은 뒤 집게 = 1). 시작 = (0, ..., 0, 1)
};

// 이름 → 값 (policies.yaml lie_upright observation 이름과 같게)
std::map<std::string, std::vector<float>> lieUprightObservation(const ArmDiffIK & ik, const LieUprightState & s);
const std::map<std::string, int> & lieUprightObservationSizes();

}  // namespace control
