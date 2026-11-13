// odom_icr (#92, C++ 로 옮김 #265): 바퀴 odom 에 스키드 회전 중심 (ICR) 옆 이동을 채워 EKF 에 넘긴다.
//
// 4 륜 스키드는 base_link 가 아니라 ICR (base_link x = icr_x, 모의 -0.0658 m) 둘레로 돈다.
// 그래서 돌 때 base_link 는 옆으로 vy = -icr_x * w 만큼 움직이는데, diff_drive /odom 은 vy 를 늘 0 으로 낸다.
//   /odom (nav_msgs/Odometry) + /imu/data (w) -> /odom_icr (vy 채움, 나머지는 /odom 그대로)
//   - w: IMU 회전 속도 (바퀴 회전 속도는 미끄러져 큼). IMU 가 imu_timeout 넘게 끊기면 /odom 의 w (경고)
//   - vy 분산 = (icr_x * yaw_rate_sd)^2 + (w * icr_x_sd)^2 + vy_var_min
//   icr_x = robot_geometry.yaml base.icr_x (drive_node turn 보정과 같은 값 한 곳). ekf.yaml odom0 = /odom_icr (vx · vy)
// Python 판 (scripts/odom_icr.py) 은 Jetson 정지 상태에서 CPU 28 % (코어 하나 기준, #265) — 메시지마다 rclpy 변환 비용.
#include <memory>
#include <optional>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace bringup
{

class OdomIcr : public rclcpp::Node
{
public:
  explicit OdomIcr(const rclcpp::NodeOptions & options)
  : Node("odom_icr", options)
  {
    icr_x_ = declare_parameter<double>("base.icr_x", 0.0);
    icr_x_sd_ = declare_parameter<double>("icr_x_sd", 0.02);        // [m] icr_x 불확실 (실측 전)
    yaw_rate_sd_ = declare_parameter<double>("yaw_rate_sd", 0.05);  // [rad/s] IMU w 오차
    vy_var_min_ = declare_parameter<double>("vy_var_min", 1e-6);    // [(m/s)^2] 0 분산 방지
    imu_timeout_ = declare_parameter<double>("imu_timeout", 0.2);   // [s]
    const auto odom_in = declare_parameter<std::string>("odom_in", "/odom");
    const auto odom_out = declare_parameter<std::string>("odom_out", "/odom_icr");
    const auto imu_in = declare_parameter<std::string>("imu_in", "/imu/data");
    pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_out, 10);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_in, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
        w_imu_ = m->angular_velocity.z;
        t_imu_ = now();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_in, 10, [this](nav_msgs::msg::Odometry::UniquePtr m) {onOdom(std::move(m));});
    RCLCPP_INFO(get_logger(), "%s + %s -> %s, icr_x %+.4f m", odom_in.c_str(), imu_in.c_str(), odom_out.c_str(), icr_x_);
  }

private:
  void onOdom(nav_msgs::msg::Odometry::UniquePtr m)
  {
    double w;
    if (t_imu_ && (now() - *t_imu_).seconds() <= imu_timeout_) {
      w = w_imu_;
    } else {
      w = m->twist.twist.angular.z;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "IMU 없음 -> 바퀴 w 로 vy");
    }
    m->twist.twist.linear.y = -icr_x_ * w;
    m->twist.covariance[7] = (icr_x_ * yaw_rate_sd_) * (icr_x_ * yaw_rate_sd_) + (w * icr_x_sd_) * (w * icr_x_sd_) + vy_var_min_;
    pub_->publish(std::move(m));
  }

  double icr_x_, icr_x_sd_, yaw_rate_sd_, vy_var_min_, imu_timeout_;
  double w_imu_{0.0};
  std::optional<rclcpp::Time> t_imu_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace bringup

RCLCPP_COMPONENTS_REGISTER_NODE(bringup::OdomIcr)
