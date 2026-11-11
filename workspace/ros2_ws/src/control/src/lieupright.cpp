#include "control/lieupright.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>
#include <Eigen/LU>

namespace control
{

namespace
{
std::vector<float> vec(const Eigen::Vector3d & v)
{
  return {static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())};
}
}  // namespace

Eigen::Matrix3d lieTargetRotation(const ArmDiffIK & ik, const Eigen::Vector3d & grasp, double beta, double rho)
{
  const Eigen::Vector3d & j1 = ik.params().joint1_origin;
  Eigen::Vector3d h(grasp.x() - j1.x(), grasp.y() - j1.y(), 0.0);
  h /= std::max(h.norm(), 1e-6);
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d t = z.cross(h);
  const Eigen::Vector3d a = -std::cos(beta) * z + std::sin(beta) * h;
  const Eigen::Vector3d u0 = a.cross(t);
  const Eigen::Vector3d c = std::cos(rho) * t + std::sin(rho) * u0;
  const Eigen::Vector3d u = -std::sin(rho) * t + std::cos(rho) * u0;
  Eigen::Matrix3d r;
  r.col(0) = a;
  r.col(1) = c;
  r.col(2) = u;
  return r;
}

Joints5 lieApplyAction(
  const ArmDiffIK & ik, const LieUprightParams & p, const Joints5 & target_prev, const Joints5 & q_meas,
  const std::array<float, 6> & action, double & beta, double & rho)
{
  auto clip = [](double v) {return std::clamp(v, -1.0, 1.0);};
  beta = std::clamp(beta + clip(action[3]) * p.beta_scale, p.beta_min, p.beta_max);
  rho = std::clamp(rho + clip(action[4]) * p.rho_scale, -p.rho_max, p.rho_max);
  const Eigen::Vector3d dp = Eigen::Vector3d(clip(action[0]), clip(action[1]), clip(action[2])) * p.cart_scale;
  const ArmFrame f = ik.forward(q_meas);
  const Eigen::Matrix3d rt = lieTargetRotation(ik, f.tcp, beta, rho);
  // Isaac quat_box_minus(목표, 지금) = 목표 · 지금⁻¹ 의 축-각 (0 ~ π)
  const Eigen::AngleAxisd aa(rt * f.rot.transpose());
  const Eigen::Vector3d rot_err = aa.axis() * aa.angle();
  Eigen::Matrix<double, 6, 5> jf = ik.jacobian(q_meas);
  jf.bottomRows<3>() *= p.rot_weight;
  Eigen::Matrix<double, 6, 1> err;
  err << dp, rot_err * p.rot_weight;
  const Eigen::Matrix<double, 6, 6> jjt = jf * jf.transpose() + p.damping * Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::Matrix<double, 5, 1> dq = jf.transpose() * jjt.partialPivLu().solve(err);
  const auto & lo = ik.params().lower;
  const auto & hi = ik.params().upper;
  Joints5 out{};
  for (size_t k = 0; k < 5; ++k) {
    const double t = std::clamp(target_prev[k] + dq(static_cast<int>(k)), q_meas[k] - p.lead_max, q_meas[k] + p.lead_max);
    out[k] = std::clamp(t, lo[k], hi[k]);
  }
  return out;
}

bool LieHoldFilter::update(double finger_m, const LieUprightParams & p)
{
  const bool raw = finger_m > p.grasp_lo && finger_m < p.grasp_hi;
  on = raw ? on + 1 : 0;
  off = raw ? 0 : off + 1;
  if (on >= p.hold_on) {
    state = true;
  } else if (off >= p.hold_off) {
    state = false;
  }
  return state;
}

Eigen::Vector2d lieStandSpot(
  const LieUprightParams & p, const Eigen::Vector2d & box_c, double box_yaw, const Eigen::Vector2d & nb, double joint1_x)
{
  const double lim = p.box_size / 2 - p.spot_edge;
  std::vector<double> g;
  for (double v = -lim; v <= lim + 1e-6; v += p.spot_step) {g.push_back(v);}
  const double cy = std::cos(box_yaw), sy = std::sin(box_yaw);
  double best = std::numeric_limits<double>::infinity();
  Eigen::Vector2d out = box_c;
  // 학습: meshgrid(indexing xy) 를 펼친 순서 = V 바깥 · U 안쪽, 같은 점수면 앞의 것
  for (double vv : g) {
    for (double uu : g) {
      const Eigen::Vector2d c(box_c.x() + cy * uu - sy * vv, box_c.y() + sy * uu + cy * vv);
      const double r = std::hypot(c.x() - joint1_x, c.y());
      const double dn = (c - nb).norm();
      const bool ok = r >= p.spot_r_min && r <= p.spot_r_max && dn >= p.spot_nb;
      const double sc = std::abs(r - p.spot_r_best) - p.spot_nb_w * std::min(dn, p.spot_nb_cap) + (ok ? 0.0 : 10.0);
      if (sc < best) {
        best = sc;
        out = c;
      }
    }
  }
  return out;
}

const std::map<std::string, int> & lieUprightObservationSizes()
{
  static const std::map<std::string, int> sizes{
    {"joint_pos_rel", 5}, {"joint_vel", 5}, {"finger_pos", 1}, {"puck_pos", 3}, {"long_axis_cos_sin2", 2}, {"grasp_pos", 3},
    {"approach_axis", 3}, {"close_axis", 3}, {"stand_spot", 3}, {"neighbor_rel", 2}, {"neighbor_present", 1},
    {"box_center", 2}, {"box_yaw_cos_sin", 2}, {"held", 1}, {"rho_cos_sin_beta", 3}, {"prev_action", 6}};
  return sizes;
}

std::map<std::string, std::vector<float>> lieUprightObservation(const ArmDiffIK & ik, const LieUprightState & s)
{
  std::map<std::string, std::vector<float>> o;
  std::vector<float> jp(5), jv(5);
  for (size_t k = 0; k < 5; ++k) {
    jp[k] = static_cast<float>(s.q[k] - kLieReadyPose45[k]);
    jv[k] = static_cast<float>(s.qd[k]);
  }
  o["joint_pos_rel"] = jp;
  o["joint_vel"] = jv;
  o["finger_pos"] = {static_cast<float>(s.finger_m)};
  o["puck_pos"] = vec(s.puck);
  o["long_axis_cos_sin2"] = {static_cast<float>(std::cos(2 * s.long_yaw)), static_cast<float>(std::sin(2 * s.long_yaw))};
  const ArmFrame f = ik.forward(s.q);
  o["grasp_pos"] = vec(f.tcp);
  o["approach_axis"] = vec(f.approach());
  o["close_axis"] = vec(f.close_axis());
  o["stand_spot"] = vec(s.spot);
  if (s.neighbor) {
    o["neighbor_rel"] = {static_cast<float>(s.neighbor->x() - s.puck.x()), static_cast<float>(s.neighbor->y() - s.puck.y())};
    o["neighbor_present"] = {1.f};
  } else {
    o["neighbor_rel"] = {0.f, 0.f};
    o["neighbor_present"] = {0.f};
  }
  o["box_center"] = {static_cast<float>(s.box_c.x()), static_cast<float>(s.box_c.y())};
  o["box_yaw_cos_sin"] = {static_cast<float>(std::cos(s.box_yaw)), static_cast<float>(std::sin(s.box_yaw))};
  o["held"] = {s.held ? 1.f : 0.f};
  o["rho_cos_sin_beta"] = {static_cast<float>(std::cos(s.rho)), static_cast<float>(std::sin(s.rho)), static_cast<float>(s.beta)};
  o["prev_action"] = std::vector<float>(s.action.begin(), s.action.end());
  return o;
}

}  // namespace control
