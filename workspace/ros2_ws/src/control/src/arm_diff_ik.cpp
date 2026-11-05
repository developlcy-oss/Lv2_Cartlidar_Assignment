#include "control/arm_diff_ik.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>
#include <Eigen/LU>

namespace control
{

ArmDiffIK::ArmDiffIK(const ArmDiffIKParams & p)
: p_(p) {}

void ArmDiffIK::chain(
  const Joints5 & q, std::array<Eigen::Vector3d, 5> & axis, std::array<Eigen::Vector3d, 5> & origin,
  Eigen::Matrix3d & r_ee, Eigen::Vector3d & p_ee) const
{
  // URDF 관절 원점은 모두 평행 이동 (rpy 0), 축 = z · y · y · y · x (부모 링크 좌표)
  const std::array<Eigen::Vector3d, 5> off{p_.joint1_origin, p_.joint2_origin, p_.joint3_origin, p_.joint4_origin,
    p_.joint5_origin};
  const std::array<Eigen::Vector3d, 5> local{Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY(),
    Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitX()};
  Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  for (size_t k = 0; k < 5; ++k) {
    t += r * off[k];
    origin[k] = t;
    axis[k] = r * local[k];
    r = r * Eigen::AngleAxisd(q[k], local[k]).toRotationMatrix();
  }
  r_ee = r;
  p_ee = t;
}

ArmFrame ArmDiffIK::forward(const Joints5 & q) const
{
  std::array<Eigen::Vector3d, 5> ax, org;
  Eigen::Matrix3d r;
  Eigen::Vector3d t;
  chain(q, ax, org, r, t);
  return ArmFrame{t + r * p_.tcp_offset, r};
}

Eigen::Matrix<double, 6, 5> ArmDiffIK::jacobian(const Joints5 & q) const
{
  std::array<Eigen::Vector3d, 5> ax, org;
  Eigen::Matrix3d r;
  Eigen::Vector3d t;
  chain(q, ax, org, r, t);
  const Eigen::Vector3d tcp = t + r * p_.tcp_offset;
  Eigen::Matrix<double, 6, 5> j;
  for (int k = 0; k < 5; ++k) {
    j.block<3, 1>(0, k) = ax[k].cross(tcp - org[k]);
    j.block<3, 1>(3, k) = ax[k];
  }
  return j;
}

Eigen::Vector3d ArmDiffIK::tangentCloseDir(const Eigen::Vector3d & tcp) const
{
  const Eigen::Vector3d a = tcp - p_.joint1_origin;
  const double th = std::atan2(a.y(), a.x()) + M_PI / 2.0;
  return {std::cos(th), std::sin(th), 0.0};
}

Eigen::Vector3d ArmDiffIK::approachTarget(const Eigen::Vector3d & y, const Eigen::Vector3d & tcp, bool tilted) const
{
  const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d h = y.cross(up);
  h /= std::max(h.norm(), 1e-6);
  if ((tcp - p_.joint1_origin).dot(h) < 0.0) {h = -h;}
  const double t = tilted ? p_.tilt_deg * M_PI / 180.0 : 0.0;
  return -up * std::cos(t) + h * std::sin(t);
}

Joints5 ArmDiffIK::step(
  const Joints5 & q, const Eigen::Vector3d & dp, bool tilted, const std::optional<Eigen::Vector3d> & close_dir) const
{
  const ArmFrame f = forward(q);
  const Eigen::Vector3d ax = f.rot.col(0), ay = f.rot.col(1), az = f.rot.col(2);
  Eigen::Vector3d y = close_dir ? *close_dir : tangentCloseDir(f.tcp);
  if (y.dot(ay) < 0.0) {y = -y;}   // 닫는 축은 앞뒤 대칭: 지금 축에 가까운 부호 (손목을 180° 돌리지 않게)
  const Eigen::Vector3d down = approachTarget(y, f.tcp, tilted);
  const Eigen::Vector3d z = down.cross(y);
  const Eigen::Vector3d rot_err = 0.5 * (ax.cross(down) + ay.cross(y) + az.cross(z));
  Eigen::Matrix<double, 6, 5> jf = jacobian(q);
  jf.bottomRows<3>() *= p_.rot_weight;
  Eigen::Matrix<double, 6, 1> err;
  err << dp, rot_err * p_.rot_weight;
  const Eigen::Matrix<double, 6, 6> jjt = jf * jf.transpose() + p_.damping * Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::Matrix<double, 5, 1> dq = jf.transpose() * jjt.partialPivLu().solve(err);
  return {dq(0), dq(1), dq(2), dq(3), dq(4)};
}

Joints5 ArmDiffIK::applyAction(
  const Joints5 & target_prev, const Joints5 & q_meas, const Eigen::Vector3d & action_xyz, bool tilted,
  const std::optional<Eigen::Vector3d> & close_dir) const
{
  const Eigen::Vector3d a = action_xyz.cwiseMax(-1.0).cwiseMin(1.0);
  const Joints5 dq = step(q_meas, a * p_.cart_scale, tilted, close_dir);
  Joints5 out{};
  for (size_t k = 0; k < 5; ++k) {
    const double t = std::clamp(target_prev[k] + dq[k], q_meas[k] - p_.lead_max, q_meas[k] + p_.lead_max);
    out[k] = std::clamp(t, p_.lower[k], p_.upper[k]);
  }
  return out;
}

double ArmDiffIK::rollError(const Joints5 & q, const std::optional<Eigen::Vector3d> & close_dir) const
{
  const ArmFrame f = forward(q);
  const Eigen::Vector3d appr = f.rot.col(0), cl = f.rot.col(1);
  Eigen::Vector3d want = close_dir ? *close_dir : tangentCloseDir(f.tcp);
  if (want.dot(cl) < 0.0) {want = -want;}
  return std::atan2(want.cross(cl).dot(appr), want.dot(cl));
}

}  // namespace control
