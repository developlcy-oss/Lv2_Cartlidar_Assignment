// ArmDiffIK — 강화학습 TopPick 정책 (Waffle-TopPick-Direct) 의 행동 = 잡는 점 (TCP) 변위를 관절 목표로 바꾸는 감쇠 최소제곱 IK.
//   Isaac 학습 환경 toppick_env.py `_ik_step` · `_pre_physics_step` 과 같은 식 (값이 다르면 정책이 학습 때와 다르게 움직임):
//     잡는 점 변위 dp (base_link, m) + 집게 자세 목표 (집게 방향 = 아래, 기울임 모드면 닫는 축 둘레로 45°, 손가락 닫는 축 = J1 접선)
//     → 6 x 5 야코비안 (잡는 점 선속도 3 + 각속도 3 x rot_weight) · 감쇠 최소제곱 → dq
//     → 관절 목표 = clamp(직전 목표 + dq, 측정 ± lead_max) → 관절 한계로 clamp
//   자세 오차 = 0.5 Σ (지금 축 × 목표 축) (쿼터니언은 180° 근처에서 NaN, 정책_학습기록 4.15)
//
// 기구 = Isaac 학습 URDF (urdf_and_usd/waffle_4wd_description/urdf/waffle_4wd.urdf, base_link 좌표):
//   base_link → joint1 (0.096, 0, 0.131) 축 z → joint2 (0, 0, 0.0595) 축 y → joint3 (0.024, 0, 0.128) 축 y
//   → joint4 (0.124, 0, 0) 축 y → joint5 (0.028, 0, 0) 축 x → gripper_roll_link. 잡는 점 = gripper_roll_link (0.14, 0, 0)
//   (= 퍽 중심 + 15 mm. ROS tool_link 는 joint4 + 0.167 이라 잡는 점보다 1 mm 안쪽 — 이 IK 는 tool_link 를 쓰지 않음)
//   집게 축: gripper_roll_link +x = 집게 방향, +y = 손가락 닫는 축, +z = 카메라 쪽
// 관절값은 URDF (모델) 값. 실제 명령 = ik.joint_signs · ik.joint_offsets 변환은 노드가 (arm_ik.hpp 와 같은 규약)
// ROS 없는 순수 C++ (Eigen). 시험: test/test_diff_ik.cpp (pallet_traj.Arm FK · 수치 미분 IK 와 비교)

#pragma once

#include <array>
#include <optional>

#include <Eigen/Core>

namespace control
{

struct ArmDiffIKParams
{
  Eigen::Vector3d joint1_origin{0.096, 0.0, 0.131};   // base_link → joint1 (arm_base_joint)
  Eigen::Vector3d joint2_origin{0.0, 0.0, 0.0595};    // joint1 → joint2
  Eigen::Vector3d joint3_origin{0.024, 0.0, 0.128};
  Eigen::Vector3d joint4_origin{0.124, 0.0, 0.0};
  Eigen::Vector3d joint5_origin{0.028, 0.0, 0.0};
  Eigen::Vector3d tcp_offset{0.14, 0.0, 0.0};          // gripper_roll_link 좌표의 잡는 점 (GRASP_OFFSET_EE)
  double cart_scale{0.005};      // [m / 행동 1] 정책 행동 → 잡는 점 변위 (toppick cart_scale, 30 Hz)
  double rot_weight{0.3};        // 자세 오차 · 각속도 줄의 가중치 (rot_hold_weight)
  double damping{1e-4};          // 감쇠 λ (ik_damping)
  double lead_max{0.25};         // [rad] 관절 목표가 측정값보다 앞설 수 있는 한계 (target_lead_max)
  double tilt_deg{45.0};         // 기울임 모드 집게 기울기
  std::array<double, 5> lower{-3.14159, -2.14, -1.6, -1.92, -2.8};   // URDF 한계 (Isaac soft limit = URDF)
  std::array<double, 5> upper{3.14159, 1.6, 1.6, 2.0, 2.8};
};

using Joints5 = std::array<double, 5>;

struct ArmFrame
{
  Eigen::Vector3d tcp;        // 잡는 점 (base_link)
  Eigen::Matrix3d rot;        // gripper_roll_link 자세 (열 = 집게 x · y · z 축, base_link)
  Eigen::Vector3d approach() const {return rot.col(0);}
  Eigen::Vector3d close_axis() const {return rot.col(1);}
};

class ArmDiffIK
{
public:
  explicit ArmDiffIK(const ArmDiffIKParams & p = ArmDiffIKParams{});

  ArmFrame forward(const Joints5 & q) const;

  // 잡는 점에서의 기하 야코비안 (위 3 줄 = 선속도, 아래 3 줄 = 각속도, base_link)
  Eigen::Matrix<double, 6, 5> jacobian(const Joints5 & q) const;

  // 목표 손가락 닫는 방향 (수평 단위) = J1 축에서 본 잡는 점의 접선 (학습 close_dir_w, 기울임 모드는 늘 이것)
  Eigen::Vector3d tangentCloseDir(const Eigen::Vector3d & tcp) const;

  // 목표 집게 방향: 수직 = 아래, 기울임 = 아래에서 닫는 축 y 둘레로 tilt_deg (손끝이 J1 에서 먼 쪽) (학습 appr_dir_w)
  Eigen::Vector3d approachTarget(const Eigen::Vector3d & y, const Eigen::Vector3d & tcp, bool tilted) const;

  // 감쇠 최소제곱 한 스텝: 잡는 점 변위 dp [m] + 자세 목표 → dq. close_dir 없으면 tangentCloseDir (수직 모드 상자 퍽은 인식 면 방향을 넘김)
  Joints5 step(const Joints5 & q, const Eigen::Vector3d & dp, bool tilted,
    const std::optional<Eigen::Vector3d> & close_dir = std::nullopt) const;

  // 정책 행동 (앞 3 개 = 변위 [-1, 1], cart_scale 배) → 새 관절 목표 (학습 _pre_physics_step).
  //   target_prev = 직전 관절 목표 (첫 스텝은 측정값), q_meas = 측정 관절 (야코비안 · 앞섬 한계 기준)
  Joints5 applyAction(const Joints5 & target_prev, const Joints5 & q_meas, const Eigen::Vector3d & action_xyz, bool tilted,
    const std::optional<Eigen::Vector3d> & close_dir = std::nullopt) const;

  // 손가락 닫는 축 − 목표 닫는 방향 각 (집게 방향 둘레, rad, 앞뒤 대칭) — 정책 관측 roll (cos, sin) 에 씀
  double rollError(const Joints5 & q, const std::optional<Eigen::Vector3d> & close_dir = std::nullopt) const;

  const ArmDiffIKParams & params() const {return p_;}

private:
  // 관절 k (0..4) 의 축 (base_link) 과 원점, 그리고 gripper_roll_link 프레임
  void chain(const Joints5 & q, std::array<Eigen::Vector3d, 5> & axis, std::array<Eigen::Vector3d, 5> & origin,
    Eigen::Matrix3d & r_ee, Eigen::Vector3d & p_ee) const;

  ArmDiffIKParams p_;
};

}  // namespace control
