// manipulator.yaml 관절 한계 · 자세 · 팔레트 궤적이 URDF (= 펌웨어 config.h) 한계와 맞는지 (#83)
//   펌웨어는 범위 밖 CMD 를 통째로 거부하므로 세 곳이 어긋나면 팔이 움직이지 않는다
#include <gtest/gtest.h>

#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace
{
std::string readFile(const std::string & path)
{
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// robot.urdf.xacro 의 <xacro:arm_joint name="joint1" ... lower="-3.14159" upper="3.14159"/>
std::map<std::string, std::pair<double, double>> urdfLimits()
{
  const std::string text = readFile(URDF_XACRO);
  std::map<std::string, std::pair<double, double>> out;
  // 패턴 안에 )" 가 있어 원시 문자열 구분자 re 를 붙임
  const std::regex re(R"re(<xacro:arm_joint\s+name="(\w+)"[^>]*lower="([-0-9.]+)"\s+upper="([-0-9.]+)")re");
  for (auto it = std::sregex_iterator(text.begin(), text.end(), re); it != std::sregex_iterator(); ++it) {
    out[(*it)[1]] = {std::stod((*it)[2]), std::stod((*it)[3])};
  }
  return out;
}

YAML::Node params()
{
  return YAML::LoadFile(MANIPULATOR_YAML)["manipulator_node"]["ros__parameters"];
}
}  // namespace

TEST(ConfigLimits, YamlLimitsEqualUrdf)
{
  const auto urdf = urdfLimits();
  const auto p = params();
  const auto names = p["arm"]["joint_names"].as<std::vector<std::string>>();
  const auto lo = p["arm"]["lower_limits"].as<std::vector<double>>();
  const auto hi = p["arm"]["upper_limits"].as<std::vector<double>>();
  ASSERT_EQ(names.size(), 6u);
  for (size_t i = 0; i < names.size(); ++i) {
    ASSERT_TRUE(urdf.count(names[i])) << names[i] << " not in " << URDF_XACRO;
    EXPECT_NEAR(lo[i], urdf.at(names[i]).first, 1e-4) << names[i];
    EXPECT_NEAR(hi[i], urdf.at(names[i]).second, 1e-4) << names[i];
  }
}

TEST(ConfigLimits, PosesWithinLimits)
{
  const auto p = params();
  const auto lo = p["arm"]["lower_limits"].as<std::vector<double>>();
  const auto hi = p["arm"]["upper_limits"].as<std::vector<double>>();
  for (const auto & name : p["poses"]["names"].as<std::vector<std::string>>()) {
    const auto q = p["poses"][name].as<std::vector<double>>();
    ASSERT_EQ(q.size(), 6u) << name;
    for (size_t i = 0; i < 6; ++i) {
      EXPECT_GE(q[i], lo[i]) << name << "[" << i << "]";
      EXPECT_LE(q[i], hi[i]) << name << "[" << i << "]";
    }
  }
}

TEST(ConfigLimits, GripperPositionsWithinSixthJoint)
{
  const auto p = params();
  const auto lo = p["arm"]["lower_limits"].as<std::vector<double>>();
  const auto hi = p["arm"]["upper_limits"].as<std::vector<double>>();
  if (p["gripper"]["mode"].as<std::string>() != "arm_joint") {GTEST_SKIP();}
  for (const char * k : {"open_position", "close_position"}) {
    const double v = p["gripper"][k].as<double>();
    EXPECT_GE(v, lo[5]) << k;
    EXPECT_LE(v, hi[5]) << k;
  }
}

TEST(ConfigLimits, PalletWaypointsWithinLimits)
{
  const auto p = params();
  const auto lo = p["arm"]["lower_limits"].as<std::vector<double>>();
  const auto hi = p["arm"]["upper_limits"].as<std::vector<double>>();
  const YAML::Node doc = YAML::LoadFile(PALLET_JSON);
  ASSERT_EQ(doc["slots"].size(), 10u);   // 팔레트 v5 (2026-11-13): 열마다 4 · 4 · 2 (이전 v3 12 칸)
  for (const auto & slot : doc["slots"]) {
    for (const auto & seg : slot["segments"]) {
      for (const auto & q : seg["q"].as<std::vector<std::vector<double>>>()) {
        ASSERT_EQ(q.size(), 5u);
        for (size_t i = 0; i < 5; ++i) {
          EXPECT_GE(q[i], lo[i]) << "row " << slot["row"].as<int>() << " slot " << slot["index"].as<int>();
          EXPECT_LE(q[i], hi[i]) << "row " << slot["row"].as<int>() << " slot " << slot["index"].as<int>();
        }
      }
    }
  }
}

namespace
{
// robot.urdf.xacro 의 <xacro:arm_joint name=".." ... xyz=".."/> · <xacro:fixed_link name=".." ... xyz=".." rpy=".."/> 원점
std::map<std::string, std::vector<double>> urdfOrigins(const std::string & macro, const std::string & attr)
{
  const std::string text = readFile(URDF_XACRO);
  std::map<std::string, std::vector<double>> out;
  const std::regex re("<xacro:" + macro + R"re(\s+name="(\w+)"[^>]*\s)re" + attr + R"re(="([^"]+)")re");
  for (auto it = std::sregex_iterator(text.begin(), text.end(), re); it != std::sregex_iterator(); ++it) {
    std::istringstream in((*it)[2].str());
    std::vector<double> v;
    double x;
    while (in >> x) {v.push_back(x);}
    out[(*it)[1]] = v;
  }
  return out;
}
}  // namespace

// manipulator.yaml 기구 치수 (ArmIK · target_look 카메라) = robot.urdf.xacro 원점 (#112). URDF 만 바꾸면 IK 가 조용히 틀어지는 것을 막음
TEST(ConfigLimits, IkGeometryEqualsUrdf)
{
  const auto joints = urdfOrigins("arm_joint", "xyz");
  const auto links = urdfOrigins("fixed_link", "xyz");
  const auto rpy = urdfOrigins("fixed_link", "rpy");
  for (const char * j : {"joint1", "joint3", "joint4", "joint5", "gripper_roll"}) {
    ASSERT_EQ(joints.count(j), 1u) << j << " xyz not found in " << URDF_XACRO;
    ASSERT_EQ(joints.at(j).size(), 3u) << j;
  }
  for (const char * l : {"tool_link", "arm_camera_link"}) {
    ASSERT_EQ(links.count(l), 1u) << l << " xyz not found in " << URDF_XACRO;
    ASSERT_EQ(links.at(l).size(), 3u) << l;
  }
  ASSERT_EQ(rpy.count("arm_camera_link"), 1u);
  const auto ik = params()["ik"];
  const double tol = 1e-6;
  EXPECT_NEAR(ik["shoulder_height"].as<double>(), joints.at("joint1")[2], tol);
  EXPECT_NEAR(ik["upper_arm_x"].as<double>(), joints.at("joint3")[0], tol);
  EXPECT_NEAR(ik["upper_arm_z"].as<double>(), joints.at("joint3")[2], tol);
  EXPECT_NEAR(ik["forearm"].as<double>(), joints.at("joint4")[0], tol);
  // joint4 → tool_link = joint5 + gripper_roll + tool_link (모두 집게 축 x 위)
  EXPECT_NEAR(ik["wrist_to_tool"].as<double>(),
    joints.at("joint5")[0] + joints.at("gripper_roll")[0] + links.at("tool_link")[0], tol);
  const auto look = params()["target_look"];
  EXPECT_NEAR(look["camera_x"].as<double>(), joints.at("joint5")[0] + links.at("arm_camera_link")[0], tol);
  EXPECT_NEAR(look["camera_z"].as<double>(), links.at("arm_camera_link")[2], tol);
  EXPECT_NEAR(look["camera_tilt"].as<double>(), rpy.at("arm_camera_link")[1], tol);
}
