#include "control/toppick_obs.hpp"

#include <cmath>

namespace control
{

namespace
{
std::vector<float> vec(const Eigen::Vector3d & v)
{
  return {static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())};
}
}  // namespace

const std::map<std::string, int> & topPickObservationSizes()
{
  static const std::map<std::string, int> sizes{
    {"joint_pos_rel", 5}, {"joint_vel", 5}, {"finger_pos", 1}, {"puck_pos", 3}, {"grasp_pos", 3}, {"puck_minus_grasp", 3},
    {"approach_axis", 3}, {"close_axis", 3}, {"neighbor_rel", 2}, {"is_box", 1}, {"roll_cos_sin", 2}, {"prev_action", 4},
    {"tilted", 1}};
  return sizes;
}

std::map<std::string, std::vector<float>> topPickObservation(const ArmDiffIK & ik, const TopPickState & s, const TopPickLayout & layout)
{
  std::map<std::string, std::vector<float>> o;
  std::vector<float> jp(5), jv(5);
  for (size_t k = 0; k < 5; ++k) {
    jp[k] = static_cast<float>(s.q[k] - layout.ref_pose[k]);
    jv[k] = static_cast<float>(s.qd[k]);
  }
  o["joint_pos_rel"] = jp;
  o["joint_vel"] = jv;
  o["finger_pos"] = {static_cast<float>(s.finger_m)};
  const ArmFrame f = ik.forward(s.q);
  o["puck_pos"] = vec(s.puck);
  o["grasp_pos"] = vec(f.tcp);
  o["puck_minus_grasp"] = vec(s.puck - f.tcp);
  o["approach_axis"] = vec(f.approach());
  o["close_axis"] = vec(f.close_axis());
  if (s.neighbor) {
    o["neighbor_rel"] = {static_cast<float>(s.neighbor->x() - s.puck.x()), static_cast<float>(s.neighbor->y() - s.puck.y())};
  } else {
    o["neighbor_rel"] = {static_cast<float>(layout.nb_absent_x), static_cast<float>(layout.nb_absent_y)};
  }
  o["is_box"] = {s.is_box ? 1.f : 0.f};
  // 기울임 모드 목표 닫는 방향 = J1 접선 (수직 모드 상자 퍽은 학습에서 인식 면 방향이지만 지금 배포는 기울임만)
  const double roll = ik.rollError(s.q);
  o["roll_cos_sin"] = {static_cast<float>(std::cos(roll)), static_cast<float>(std::sin(roll))};
  o["prev_action"] = {s.prev_action[0], s.prev_action[1], s.prev_action[2], s.prev_action[3]};
  o["tilted"] = {s.tilted ? 1.f : 0.f};
  return o;
}

}  // namespace control
