// OpenCRHardware — ros2_control SystemInterface 플러그인 (Firmware 팀)
// controller_manager(① ros2_control_node) 안에서 로드되어 100 Hz 로 read()/write() 가 불린다.
//   command: 바퀴 velocity × N(≤6) · 팔 position × 6 · 그리퍼 position × 0/1
//   state  : 관절 position · velocity · effort · IMU 10
//   USB CDC 후보 포트 2개 — CMD 0x01 송신 / STATE 0x81 수신, STATE.role로 보드 판별
//   E-stop 은 이 플러그인 한 곳에서 판단 (잠금식, estop_latch.hpp):
//     /joy 구독 (heartbeat · E-stop · 리셋 버튼) → /e_stop 발행 (TransientLocal, 10 Hz)
//     /diagnostics 발행 (opencr/link · opencr/motors · opencr/battery · opencr/estop, 1 Hz + 오류 즉시)
//     /battery/wheel · /battery/arm 발행 (sensor_msgs/BatteryState, 1 Hz — 보드마다 전용 배터리, 2026-11-11)
//   토픽은 하드웨어 컴포넌트 노드(get_node(), controller_manager executor 가 돌림)에서 처리하고,
//   read()/write() 실시간 루프와는 atomic 값으로만 주고받는다 (실시간 루프에서 할당 · 발행 없음).
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "opencr_hardware/estop_latch.hpp"
#include "opencr_hardware/opencr_protocol.hpp"
#include "opencr_hardware/serial_port.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"

namespace opencr_hardware
{

class OpenCRHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum class Role { WHEEL, ARM, GRIPPER };
  struct JointMap
  {
    std::string name;
    Role role;
    int slot;            // WHEEL 0~5 · ARM 0~5 · GRIPPER 0
    double lo = -std::numeric_limits<double>::infinity();   // 팔 명령 한계 (URDF command_interface min/max)
    double hi = std::numeric_limits<double>::infinity();
  };

  bool poll_serial();
  bool wait_first_state(double timeout_s);
  bool roles_ready() const;
  void refresh_aggregate_state();
  void apply_state_to_interfaces();
  bool check_arm_standby() const;  // on_activate: 처음 켤 때 팔이 대기 자세 범위 안인지
  void set_state_if(const std::string & name, double v);
  bool send_cmd(bool torque_on);
  bool send_cmd_for_role(uint8_t role, bool torque_on);
  void update_estop();             // read() 안에서 100 Hz
  void setup_ros_io();             // 하드웨어 컴포넌트 노드에 /joy · /e_stop · /diagnostics
  void publish_estop();
  void publish_diagnostics(bool force);
  void publish_battery_state(const rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr & pub,
    const char * location, float v, bool reported, uint8_t level, float percent);
  static int64_t steady_ns();

  std::vector<JointMap> joints_;
  std::string imu_name_;
  // 파라미터 (URDF <hardware><param>)
  // 후보 포트는 역할을 미리 가정하지 않는다. 각 포트의 STATE.role로 WHEEL/ARM을 판별한다.
  std::string port_ = "/dev/ttyACM0,/dev/ttyACM1";
  double comm_timeout_s_ = 0.5;
  double first_state_timeout_s_ = 2.0;
  bool joy_required_ = true;
  double joy_timeout_s_ = 0.5;
  int estop_button_ = 6;
  int reset_button_ = 7;
  double battery_warn_v_ = 11.1;
  double battery_error_v_ = 10.5;
  int battery_cells_ = 3;              // 보드마다 LiPo 3S (2026-11-11) — 잔량 % 추정에만 씀
  double battery_capacity_ah_ = 1.8;   // BatteryState.design_capacity
  double wheel_gear_ = 1.0;    // 바퀴 각속도 / 모터 각속도 (URDF drive 인자). 펌웨어 STATE · CMD 는 모터 축 기준
  bool listen_only_ = false;   // true: CMD 를 한 번도 보내지 않음 — 토크 · 모션 없이 STATE 수신만 확인
  bool auto_rearm_ = false;    // true: 잠금이 풀리면 리셋 버튼 없이 REARM (조이스틱 없는 벤치 전용, 실기 운용 금지)
  // 팔 대기 자세 tick 범위 (URDF arm_standby_min_tick · arm_standby_max_tick, 칸 0~5). 비어 있으면 검사 안 함
  std::vector<int> standby_min_tick_, standby_max_tick_;

  struct BoardLink
  {
    explicit BoardLink(std::string path_in) : path(std::move(path_in)) {}
    std::string path;
    SerialPort serial;
    opencr_proto::FrameParser parser;
    opencr_proto::State state{};
    bool have_state = false;
    rclcpp::Time last_state_time{0, 0, RCL_STEADY_TIME};
  };
  std::vector<std::unique_ptr<BoardLink>> links_;
  opencr_proto::State latest_{};
  float arm_battery_raw_ = 0.f;   // 팔 보드 STATE battery_v (latest_ 는 주행 보드 값을 담음 — 예전 계약 그대로)
  bool have_state_ = false;
  rclcpp::Clock steady_{RCL_STEADY_TIME};
  uint8_t seq_ = 0;
  uint8_t last_error_ = 0;

  // E-stop (실시간 루프 ↔ 노드 콜백, atomic 만)
  EstopLatch latch_;
  RearmPulse rearm_;              // v5: 운용자 리셋 → 팔 보드에 REARM 펄스
  bool rearm_flag_ = false;       // 이번 주기 팔 CMD 에 FLAG_REARM
  ArmRearmSlew rearm_slew_{0.0, 0};   // on_init 에서 rearm_slew_rad_s 로 다시 만듦 (REARM 때 튀지 않게)
  double cmd_dt_s_ = 0.01;        // write() 주기
  bool prev_arm_hold_ = false;    // STATE ERR_HOLD 상승 → 잠금 trip
  bool prev_reset_btn_ = false;
  std::atomic<bool> estop_{true};
  std::atomic<int64_t> joy_rx_ns_{0};
  std::atomic<bool> joy_estop_btn_{false};
  std::atomic<bool> joy_reset_btn_{false};
  std::atomic<bool> link_ok_{false};
  std::atomic<uint8_t> fw_error_{0};
  std::atomic<uint8_t> fw_version_{0};
  std::atomic<float> battery_v_{0.f};       // 주행 보드
  std::atomic<float> arm_battery_v_{0.f};   // 팔 보드 (예전 팔 펌웨어는 0 → "보고 없음")
  std::atomic<uint32_t> bad_frames_{0};
  std::atomic<int> estop_reason_code_{1};   // 0 없음 · 1 시작 · 2 링크 · 3 물리 · 4 버튼 · 5 joy · 6 팔 HOLD
  // RC-100 (주행 보드 STATE 빈 팔 칸, opencr_protocol RC100_SLOT_*, 2026-11-13) → /rc100/raw [버튼, 패킷 뒤 ms + 1]
  std::atomic<uint32_t> rc100_buttons_{0};
  std::atomic<uint32_t> rc100_age_{0};
  std::atomic<int64_t> rc100_rx_ns_{0};   // 주행 보드 STATE 를 마지막으로 받은 시각 (0 = 아직)
  // 모터별 Hardware Error Status · 온도 (°C, 0 = 모름) — STATE 진단 칸 (opencr_protocol diag_slot, 2026-11-14 #259).
  //   칸 = 관절 상태 칸 (바퀴 0 ~ 5, 팔 MAX_WHEELS + 0 ~ 5). read() 가 쓰고 진단 타이머가 읽음
  std::array<std::atomic<uint8_t>, opencr_proto::STATE_JOINTS> motor_hw_{};
  std::array<std::atomic<uint8_t>, opencr_proto::STATE_JOINTS> motor_temp_{};
  std::array<uint8_t, opencr_proto::STATE_JOINTS> logged_motor_hw_{};   // read() 안에서만 (바뀌면 로그)
  uint32_t rc100_sent_buttons_ = 0xFFFFFFFFu;
  int64_t rc100_sent_ns_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr bat_wheel_pub_, bat_arm_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr rc100_pub_;
  rclcpp::TimerBase::SharedPtr estop_timer_, diag_timer_, rc100_timer_;
  uint8_t last_diag_level_ = 0;
};

}  // namespace opencr_hardware
