// DriveNode (Controller 팀) — 바퀴 명령 주체는 이 노드 하나 (/cmd_vel_drive → twist_mux priority 50)
//   /drive/navigate      목표가 현재 위치와 xy_goal_tolerance 이내면 yaw 차이만큼 nav2 spin, 아니면 navigate_to_pose
//                        navigate.loc_gate (#121, control/loc_gate.hpp): /localization/status 가 OK 일 때만 보내고,
//                        진행 중 위치를 잃었다 OK 로 돌아오면 같은 목표를 다시 보냄 (잃은 동안 Nav2 가 실패해도 기다렸다 다시)
//   /drive/align         goal 수신 뒤 첫 새 result 의 ARM_ALIGN 비트로 모드를 정하고 끝까지 유지
//                        정밀: arm_offset → 회전, /tfmini/range (1층 TFmini → 박스 앞면, BoxRangeFilter) → 전후 (20 Hz)
//                        정밀 동안 obstacle_stop 정지 거리를 "목표 − align.box_stop_margin" 으로 낮추고 (파라미터 approach_stop_m)
//                        끝날 때(성공 · 실패 · 취소 · 선점) 박스 앞 (TFmini < align.box.park_release_m) 이면 "정차" 로 낮춘 값을 유지하고,
//                        TFmini 로 물러남을 확인한 뒤 0 으로 되돌린다 (#157, control/box_park.hpp). 정차 중에는 park_refresh_s 마다 다시 보냄
//                        (바로 되돌리면 0.13 < 평소 0.25 → twist_mux 잠금이 회전 · 후진 · Nav2 를 모두 막아 박스 앞에서 멈춤)
//                        먼 거리: object_pose(D435, base_link) → 회전 + 전진/후진, 범퍼 기준 거리, /scan 장애물 감시
//                        박스 (goal.box_only, #160 빈 장소 상자): 인식 결과를 기다리지 않음. 시작 yaw (odom) 유지 + TFmini → 전후.
//                        정지 거리 낮추기 · 박스 앞 정차는 정밀과 같음. 남는 좌우 · 각도는 mission measure_box 가 메움
//   (11-20) 상자 중앙 맞춤 (align.center.*, control/box_center.hpp): FINE 이 끝나면 Arducam 상자 윗면 측정
//          (/perception/place_box/prior → /perception/place_box/measure → /perception/place_box, odom 중심 · 요) 으로
//          상자 법선 · 중심선 옆 어긋남을 구해 회전 → 곧은 후진 → 비스듬 전진 → 회전 (차동 구동 옆 이동) 뒤 법선 yaw + TFmini 유지.
//          측정 실패 · 믿기 어려운 값이면 움직이지 않고 FINE 결과로 성공, 기동 중 막힘 · 끊김 · 시간 초과면 기존 FINE 으로 되돌아감
//          ★ 측정은 팔 ALIGN 자세에서는 상자 변이 화면 밖이라 기본은 mission 이 함: mission center_box 단계가 BOX_VIEW 로 잰 상자를
//          goal.center · center_box 로 넘기면 (팔은 ALIGN 으로 되돌린 뒤) 측정 없이 위 기동부터. FINE 안에서 스스로 재는 것은
//          align.center.auto_after_fine (기본 false)
//   (#157) /drive/navigate 를 정차 중에 받으면 먼저 곧게 후진 (navigate.backoff.*, 뒤 /scan · 뒤 카메라 확인) → 물러남 확인 뒤 Nav2
//   (11-19) 뒤가 벽에 붙은 채 받으면 먼저 곧게 전진 (navigate.pullout.*, control/pullout.hpp, 앞 /scan · TFmini 확인) → Nav2
//          (Nav2 제자리 회전에 뒤 모서리가 벽 칸에 걸려 MPPI · 복구가 모두 거부되는 것 막음). 못 가도 실패 없이 Nav2
//   /drive/execute_skill 강화학습 정책 (robot_policy, CPU) → 속도
//                        turn_p<deg> · turn_m<deg> · turn_0 = 직전 FINE 정렬 방향 기준 제자리 회전 (#77, 정책 · skill.enabled 와 상관없음)
//   result · /tfmini/range · /scan · odom 은 header.stamp 300 ms 지나면 버림. 바퀴 goal 은 한 번에 하나.
//   후진 (v < 0) 은 scan.rear_camera 면 /perception/rear_obstacle (뒤 D435, 받은 지 stale_s 안) 도 확인 (#84):
//   COARSE = 실패로 끝냄, FINE · skill = 전후 속도만 0 (회전 허용)
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/names.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "control/align_controller.hpp"
#include "control/box_center.hpp"
#include "control/box_park.hpp"
#include "control/pullout.hpp"
#include "control/loc_gate.hpp"
#include "control/turn_skill.hpp"
#include "robot_interfaces/srv/set_perception_mode.hpp"
#include "robot_interfaces/action/align_base.hpp"
#include "robot_interfaces/action/execute_skill.hpp"
#include "robot_interfaces/msg/perception_result.hpp"
#include "robot_policy/policy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace control
{
namespace names = robot_interfaces::names;
namespace qos = robot_interfaces::qos;
using namespace std::chrono_literals;
using Nav = nav2_msgs::action::NavigateToPose;
using Spin = nav2_msgs::action::Spin;
using Align = robot_interfaces::action::AlignBase;
using Skill = robot_interfaces::action::ExecuteSkill;
using NavGH = rclcpp_action::ServerGoalHandle<Nav>;
using AlignGH = rclcpp_action::ServerGoalHandle<Align>;
using SkillGH = rclcpp_action::ServerGoalHandle<Skill>;
using PerceptionResult = robot_interfaces::msg::PerceptionResult;
using PerceptionMode = robot_interfaces::srv::SetPerceptionMode::Request;   // 모드 비트

class DriveNode : public rclcpp::Node
{
public:
  explicit DriveNode(const rclcpp::NodeOptions & options)
  : Node("drive_node", options)
  {
    AlignGains g;
    g.kp_yaw = declare_parameter("align.kp_yaw", g.kp_yaw);
    g.kp_lin = declare_parameter("align.kp_lin", g.kp_lin);
    g.max_ang = declare_parameter("align.max_angular", g.max_ang);
    g.max_lin = declare_parameter("align.max_linear", g.max_lin);
    g.lateral_tol = declare_parameter("align.lateral_tolerance", g.lateral_tol);
    g.settle_cycles = declare_parameter("align.settle_cycles", g.settle_cycles);
    fine_ = AlignController(g);
    CoarseGains cg;
    cg.kp_yaw = declare_parameter("align.coarse_kp_yaw", cg.kp_yaw);
    cg.kp_lin = declare_parameter("align.coarse_kp_lin", cg.kp_lin);
    cg.max_lin = declare_parameter("align.coarse_max_speed", cg.max_lin);
    cg.max_ang = declare_parameter("align.coarse_max_angular", cg.max_ang);
    cg.yaw_tol = declare_parameter("align.coarse_yaw_tolerance", cg.yaw_tol);
    coarse_ = CoarseAlign(cg);
    BoxApproachGains bg;
    bg.kp_yaw = declare_parameter("align.box_only.kp_yaw", bg.kp_yaw);
    bg.max_ang = declare_parameter("align.box_only.max_angular", bg.max_ang);
    bg.yaw_tol = declare_parameter("align.box_only.yaw_tolerance", bg.yaw_tol);
    bg.kp_lin = g.kp_lin;
    bg.max_lin = g.max_lin;
    bg.settle_cycles = g.settle_cycles;
    box_only_ = BoxApproach(bg);
    rate_hz_ = declare_parameter("align.rate_hz", 20.0);
    lost_s_ = declare_parameter("align.lost_timeout_s", 0.5);
    mode_wait_s_ = declare_parameter("align.mode_wait_s", 0.5);
    align_timeout_s_ = declare_parameter("align.timeout_s", 30.0);
    min_fine_target_ = declare_parameter("align.min_target_distance", 0.13);   // 박스 거리: 최소 측정 0.10 + 정지 여유
    BoxRangeConfig bc;
    bc.window = static_cast<int>(declare_parameter("align.box.window", bc.window));
    bc.min_samples = static_cast<int>(declare_parameter("align.box.min_samples", bc.min_samples));
    bc.spread_m = declare_parameter("align.box.spread_m", bc.spread_m);
    box_ = BoxRangeFilter(bc);
    box_stop_margin_ = declare_parameter("align.box.stop_margin", 0.03);
    box_stop_min_ = declare_parameter("align.box.stop_min", 0.10);
    obstacle_stop_node_ = declare_parameter("align.box.obstacle_stop_node", std::string("/obstacle_stop"));
    BoxParkConfig pc;
    pc.release_m = declare_parameter("align.box.park_release_m", pc.release_m);
    pc.release_count = static_cast<int>(declare_parameter("align.box.park_release_count", pc.release_count));
    pc.refresh_s = declare_parameter("align.box.park_refresh_s", pc.refresh_s);
    pc.backoff_speed = declare_parameter("navigate.backoff.speed", pc.backoff_speed);
    pc.backoff_max_m = declare_parameter("navigate.backoff.max_m", pc.backoff_max_m);
    pc.backoff_timeout_s = declare_parameter("navigate.backoff.timeout_s", pc.backoff_timeout_s);
    box_park_ = BoxPark(pc);
    pull_.enabled = declare_parameter("navigate.pullout.enabled", pull_.enabled);
    pull_.rear_clear_m = declare_parameter("navigate.pullout.rear_clear_m", pull_.rear_clear_m);
    pull_.rear_half_angle = declare_parameter("navigate.pullout.rear_half_angle", pull_.rear_half_angle);
    pull_.front_clear_m = declare_parameter("navigate.pullout.front_clear_m", pull_.front_clear_m);
    pull_.front_tfmini_m = declare_parameter("navigate.pullout.front_tfmini_m", pull_.front_tfmini_m);
    pull_.speed = declare_parameter("navigate.pullout.speed", pull_.speed);
    pull_.max_m = declare_parameter("navigate.pullout.max_m", pull_.max_m);
    pull_.timeout_s = declare_parameter("navigate.pullout.timeout_s", pull_.timeout_s);
    center_cfg_.enabled = declare_parameter("align.center.enabled", center_cfg_.enabled);
    center_cfg_.lateral_tol = declare_parameter("align.center.lateral_tolerance", center_cfg_.lateral_tol);
    center_cfg_.max_shift = declare_parameter("align.center.max_shift", center_cfg_.max_shift);
    center_cfg_.back_min = declare_parameter("align.center.back_min", center_cfg_.back_min);
    center_cfg_.back_max = declare_parameter("align.center.back_max", center_cfg_.back_max);
    center_cfg_.max_alpha = declare_parameter("align.center.max_alpha", center_cfg_.max_alpha);
    center_cfg_.speed = declare_parameter("align.center.speed", center_cfg_.speed);
    center_cfg_.kp_yaw = declare_parameter("align.center.kp_yaw", center_cfg_.kp_yaw);
    center_cfg_.max_angular = declare_parameter("align.center.max_angular", center_cfg_.max_angular);
    center_cfg_.min_angular = declare_parameter("align.center.min_angular", center_cfg_.min_angular);
    center_cfg_.yaw_tol = declare_parameter("align.center.yaw_tolerance", center_cfg_.yaw_tol);
    center_cfg_.max_iter = static_cast<int>(declare_parameter("align.center.max_iter", center_cfg_.max_iter));
    center_cfg_.final_puck = declare_parameter("align.center.final_puck", center_cfg_.final_puck);
    center_cfg_.max_puck_yaw_dev = declare_parameter("align.center.max_puck_yaw_dev_deg", center_cfg_.max_puck_yaw_dev * 180 / M_PI) * M_PI / 180;
    center_rear_half_ = declare_parameter("align.center.rear_half_angle", 0.785);
    shift_ = ShiftManeuver(center_cfg_);
    center_service_ = declare_parameter("align.center.measure_service", std::string("/perception/place_box/measure"));
    center_box_half_ = declare_parameter("align.center.box_half", 0.1025);
    center_prior_delay_s_ = declare_parameter("align.center.prior_delay_s", 0.3);
    center_measure_timeout_s_ = declare_parameter("align.center.measure_timeout_s", 12.0);
    center_phase_timeout_s_ = declare_parameter("align.center.phase_timeout_s", 8.0);
    center_timeout_s_ = declare_parameter("align.center.timeout_s", 45.0);
    center_fwd_min_m_ = declare_parameter("align.center.forward_min_m", 0.12);
    center_auto_ = declare_parameter("align.center.auto_after_fine", false);
    coarse_min_ = declare_parameter("align.coarse_min", 0.3);
    default_tol_ = declare_parameter("align.default_tolerance", 0.01);
    front_offset_ = declare_parameter("base.front_offset", 0.1328);   // base_link → 차체 앞 끝 (B안) ★
    scan_half_ = declare_parameter("scan.half_angle", 0.349);        // ±20°
    scan_front_margin_ = declare_parameter("scan.front_margin", 0.18);
    scan_rear_stop_ = declare_parameter("scan.rear_stop", 0.30);
    require_scan_ = declare_parameter("scan.required", true);
    rear_camera_ = declare_parameter("scan.rear_camera", false);   // 뒤 D435 후진 확인 (카메라 없는 실행 · 시뮬은 false)
    stale_s_ = declare_parameter("stale_s", 0.3);
    xy_tol_ = declare_parameter("navigate.xy_goal_tolerance", 0.25);
    spin_time_ = declare_parameter("navigate.spin_time_allowance", 15.0);
    map_frame_ = declare_parameter("frames.map", std::string("map"));
    base_frame_ = declare_parameter("frames.base", std::string("base_link"));
    LocGateParams lg;
    lg.enabled = declare_parameter("navigate.loc_gate.enabled", lg.enabled);
    lg.wait_s = declare_parameter("navigate.loc_gate.wait_s", lg.wait_s);
    lg.stale_s = declare_parameter("navigate.loc_gate.stale_s", lg.stale_s);
    lg.start_states = declare_parameter("navigate.loc_gate.start_states", lg.start_states);
    lg.run_states = declare_parameter("navigate.loc_gate.run_states", lg.run_states);
    loc_gate_ = LocGate(lg);
    nav_action_name_ = declare_parameter("nav2_action", std::string("navigate_to_pose"));
    spin_action_name_ = declare_parameter("spin_action", std::string("spin"));
    skill_enabled_ = declare_parameter("skill.enabled", false);
    const auto policies_file = declare_parameter("skill.policies_file", std::string(""));
    max_v_ = declare_parameter("skill.max_linear", 0.3);
    max_w_ = declare_parameter("skill.max_angular", 1.5);
    scan_bins_ = declare_parameter("skill.scan_bins", 360);
    skill_front_margin_ = declare_parameter("skill.front_margin", 0.10);   // 범퍼 앞면에서 이 거리 안에 점 → 전진 0
    TurnGains tg;
    tg.kp = declare_parameter("turn.kp", tg.kp);
    tg.max_ang = declare_parameter("turn.max_angular", tg.max_ang);
    tg.min_ang = declare_parameter("turn.min_angular", tg.min_ang);
    tg.tol = declare_parameter("turn.tolerance_deg", 1.0) * M_PI / 180.0;
    tg.settle_cycles = declare_parameter("turn.settle_cycles", tg.settle_cycles);
    turn_ctl_ = TurnController(tg);
    turn_max_deg_ = declare_parameter("turn.max_deg", 30.0);
    turn_geo_.j1_x = declare_parameter("turn.j1_x", turn_geo_.j1_x);
    turn_geo_.center_x = declare_parameter("turn.center_x", turn_geo_.center_x);
    // 실제 회전 중심 = robot_geometry.yaml base.icr_x (EKF odom_icr 와 같은 값, #92). 기본 = 보정 없음
    turn_geo_.icr_x = declare_parameter("base.icr_x", turn_geo_.center_x);

    if (skill_enabled_) {
      std::vector<std::string> errs;
      policies_.load(policies_file, errs);
      for (const auto & e : errs) {RCLCPP_WARN(get_logger(), "정책 로드 실패: %s", e.c_str());}
      RCLCPP_INFO(get_logger(), "정책 %zu개 로드", policies_.names().size());
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions so;
    so.callback_group = cbg_;
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(names::CMD_VEL_DRIVE, 10);
    perc_sub_ = create_subscription<PerceptionResult>(names::PERCEPTION_RESULT, qos::result(),
        [this](PerceptionResult::ConstSharedPtr m) {result_ = m; result_rx_ = now();}, so);
    box_sub_ = create_subscription<sensor_msgs::msg::Range>(names::TFMINI_RANGE, qos::sensor(),
        [this](sensor_msgs::msg::Range::ConstSharedPtr m) {
          const bool ok = std::isfinite(m->range) && m->range >= m->min_range && m->range <= m->max_range;
          box_.push(rclcpp::Time(m->header.stamp, RCL_ROS_TIME).seconds(), ok ? std::optional<double>(m->range) : std::nullopt);
          if (box_park_.on_range(ok ? std::optional<double>(m->range) : std::nullopt)) {   // 물러남 확인 (#157)
            RCLCPP_INFO(get_logger(), "박스 앞 정차 해제: TFmini %.3f m — 평소 정지 거리로", m->range);
            set_box_approach(0.0);
          }
        }, so);
    obstacle_params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, obstacle_stop_node_);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(names::SCAN, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {scan_ = m;}, so);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(names::ODOM_FILTERED, 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {odom_ = m;}, so);
    if (center_cfg_.enabled && center_auto_) {   // FINE 안 자동 측정: perception place_box (latched) · 사전값 · 측정 서비스
      const auto latched = rclcpp::QoS(1).reliable().transient_local();
      pb_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/perception/place_box", latched,
          [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {pb_pose_ = m; pb_pose_rx_ = now();}, so);
      pb_status_sub_ = create_subscription<std_msgs::msg::String>("/perception/place_box/status", latched,
          [this](std_msgs::msg::String::ConstSharedPtr m) {pb_status_ = m; pb_status_rx_ = now();}, so);
      pb_prior_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/perception/place_box/prior", latched);
      pb_client_ = create_client<std_srvs::srv::Trigger>(center_service_, rclcpp::ServicesQoS(), cbg_);
    }
    if (center_cfg_.enabled) {   // 퍽 마무리 전 퍽 위치 판단 (goal.center · 자동 둘 다)
      arm_target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
          declare_parameter("align.center.arm_target_topic", std::string("/perception/arm_target")), qos::result(),
          [this](geometry_msgs::msg::PointStamped::ConstSharedPtr m) {arm_target_ = m;}, so);
    }
    if (lg.enabled) {
      loc_sub_ = create_subscription<std_msgs::msg::String>(names::LOCALIZATION_STATUS, rclcpp::QoS(1).transient_local(),
          [this](std_msgs::msg::String::ConstSharedPtr m) {loc_gate_.status(m->data, now().seconds());}, so);
    }
    if (rear_camera_) {
      rear_sub_ = create_subscription<std_msgs::msg::Bool>(names::PERCEPTION_REAR_OBSTACLE, qos::latched_state(),
          [this](std_msgs::msg::Bool::ConstSharedPtr m) {rear_ = m->data; rear_rx_ = now();}, so);
      rear_reason_sub_ = create_subscription<std_msgs::msg::String>(names::PERCEPTION_REAR_OBSTACLE_REASON, qos::latched_state(),
          [this](std_msgs::msg::String::ConstSharedPtr m) {rear_reason_ = m->data;}, so);
    }

    nav_client_ = rclcpp_action::create_client<Nav>(this, nav_action_name_, cbg_);
    spin_client_ = rclcpp_action::create_client<Spin>(this, spin_action_name_, cbg_);

    nav_server_ = rclcpp_action::create_server<Nav>(this, names::DRIVE_NAVIGATE,
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Nav::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<NavGH>) {return rclcpp_action::CancelResponse::ACCEPT;},
        [this](std::shared_ptr<NavGH> gh) {start_navigate(gh);},
        rcl_action_server_get_default_options(), cbg_);
    align_server_ = rclcpp_action::create_server<Align>(this, names::DRIVE_ALIGN,
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Align::Goal> g) {
          // target_distance 는 float32 → 0.13 이 0.12999999 로 와서 같은 값도 거부됨. 1e-4 m 여유로 비교
          if (g->target_distance < min_fine_target_ - 1e-4) {
            RCLCPP_WARN(get_logger(), "target_distance %.3f < %.2f m — 거부", g->target_distance, min_fine_target_);
            return rclcpp_action::GoalResponse::REJECT;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<AlignGH>) {return rclcpp_action::CancelResponse::ACCEPT;},
        [this](std::shared_ptr<AlignGH> gh) {start_align(gh);},
        rcl_action_server_get_default_options(), cbg_);
    skill_server_ = rclcpp_action::create_server<Skill>(this, names::DRIVE_SKILL,
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Skill::Goal> g) {
          if (const auto deg = parse_turn_deg(g->policy)) {   // 정해진 회전 — 정책 · skill.enabled 와 상관없음 (#77)
            if (std::fabs(*deg) > turn_max_deg_) {
              RCLCPP_WARN(get_logger(), "execute_skill 거부: %s 는 turn.max_deg %.0f° 초과", g->policy.c_str(), turn_max_deg_);
              return rclcpp_action::GoalResponse::REJECT;
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          }
          std::string why;
          if (!skill_enabled_) {why = "skill.enabled=false";}
          else if (!policies_.find(g->policy)) {why = "정책 '" + g->policy + "' 없음";}
          else if (policies_.find(g->policy)->spec.action_kind != "base_twist") {why = "주행 정책이 아님";}
          if (!why.empty()) {
            RCLCPP_WARN(get_logger(), "execute_skill 거부: %s", why.c_str());
            return rclcpp_action::GoalResponse::REJECT;   // mission 이 fallback 실행
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<SkillGH>) {return rclcpp_action::CancelResponse::ACCEPT;},
        [this](std::shared_ptr<SkillGH> gh) {start_skill(gh);},
        rcl_action_server_get_default_options(), cbg_);

    // 노드 시계 기준 (use_sim_time 이면 /clock) — wall timer 는 시뮬레이션 실시간 비율이 1 이 아니면 주기 · timeout 이 어긋남
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_hz_),
        [this]() {tick();}, cbg_);
    RCLCPP_INFO(get_logger(), "drive_node 준비 (align %.0f Hz, 정책 %s)", rate_hz_,
      skill_enabled_ ? "사용" : "사용 안 함");
  }

private:
  bool fresh(const builtin_interfaces::msg::Time & stamp) const
  {
    const double age = (now() - rclcpp::Time(stamp, RCL_ROS_TIME)).seconds();
    return age <= stale_s_ && age >= -0.5;
  }

  void publish_cmd(double v, double w)
  {
    geometry_msgs::msg::TwistStamped t;
    t.header.stamp = now();
    t.header.frame_id = base_frame_;
    t.twist.linear.x = v;
    t.twist.angular.z = w;
    cmd_pub_->publish(t);
  }

  // ---------- 바퀴 goal 하나만 ----------
  template<typename GH, typename R>
  void end(std::shared_ptr<GH> & gh, bool ok, std::shared_ptr<R> r, bool canceled = false)
  {
    if (gh && gh->is_active()) {
      if (canceled) {gh->canceled(r);} else if (ok) {gh->succeed(r);} else {gh->abort(r);}
    }
    gh.reset();
  }

  void preempt_all(const std::string & why)
  {
    if (align_gh_) {
      end_box_approach();
      publish_cmd(0, 0);
      auto r = std::make_shared<Align::Result>();
      r->message = why;
      end(align_gh_, false, r);
    }
    if (skill_gh_) {
      publish_cmd(0, 0);
      auto r = std::make_shared<Skill::Result>();
      r->message = why;
      end(skill_gh_, false, r);
      stop_skill_timer();
    }
    if (nav_gh_) {
      cancel_nav2();
      end(nav_gh_, false, std::make_shared<Nav::Result>());
    }
  }

  // ---------- /drive/navigate ----------
  std::optional<geometry_msgs::msg::PoseStamped> current_pose(const std::string & frame)
  {
    try {
      const auto t = tf_buffer_->lookupTransform(frame, base_frame_, tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped p;
      p.header = t.header;
      p.pose.position.x = t.transform.translation.x;
      p.pose.position.y = t.transform.translation.y;
      p.pose.orientation = t.transform.rotation;
      return p;
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  void start_navigate(std::shared_ptr<NavGH> gh)
  {
    preempt_all("새 /drive/navigate goal 로 대체");
    ref_yaw_.reset();   // turn_* 기준 방향은 다음 FINE 정렬 성공으로 다시 정함
    nav_gh_ = gh;
    if (box_park_.parked()) {start_backoff(); return;}   // 박스 앞이면 먼저 후진 (#157) → tick_backoff 가 begin_nav
    if (start_pullout()) {return;}                       // 뒤가 벽에 붙었으면 먼저 전진 (11-19) → tick_pullout 이 begin_nav
    begin_nav();
  }

  void begin_nav()
  {
    if (!loc_gate_.begin(now().seconds())) {   // 위치 추정 OK 를 기다렸다 tick 이 보냄 (#121)
      RCLCPP_WARN(get_logger(), "navigate: 위치 추정 %s → OK 기다림 (최대 %.0f s)",
        loc_gate_.describe(now().seconds()).c_str(), loc_gate_.params().wait_s);
      return;
    }
    dispatch_nav();
  }

  // nav_gh_ 의 목표를 Nav2 로 보냄 (가까우면 spin)
  void dispatch_nav()
  {
    const uint64_t gen = ++nav_gen_;
    const auto & goal = *nav_gh_->get_goal();
    const std::string frame = goal.pose.header.frame_id.empty() ? map_frame_ : goal.pose.header.frame_id;
    auto cur = current_pose(frame);
    if (cur) {
      const double dx = goal.pose.pose.position.x - cur->pose.position.x;
      const double dy = goal.pose.pose.position.y - cur->pose.position.y;
      if (std::hypot(dx, dy) <= xy_tol_) {
        // 시작 = 목표 위치인 navigate_to_pose 는 경로가 안 나오거나 회전 없이 성공 → spin 으로 처리
        double dyaw = tf2::getYaw(goal.pose.pose.orientation) - tf2::getYaw(cur->pose.orientation);
        dyaw = std::atan2(std::sin(dyaw), std::cos(dyaw));
        Spin::Goal s;
        s.target_yaw = static_cast<float>(dyaw);
        s.time_allowance = rclcpp::Duration::from_seconds(spin_time_);
        RCLCPP_INFO(get_logger(), "navigate → spin %.1f°", dyaw * 180 / M_PI);
        if (!spin_client_->action_server_is_ready()) {finish_nav(false, "nav2 spin 서버 없음"); return;}
        auto opts = rclcpp_action::Client<Spin>::SendGoalOptions();
        opts.goal_response_callback = [this, gen](rclcpp_action::ClientGoalHandle<Spin>::SharedPtr h) {
            if (gen != nav_gen_) {if (h) {spin_client_->async_cancel_goal(h);} return;}
            if (!h) {finish_nav(false, "spin 거부"); return;}
            spin_handle_ = h;
          };
        opts.result_callback = [this, gen](const rclcpp_action::ClientGoalHandle<Spin>::WrappedResult & res) {
            if (gen != nav_gen_) {return;}
            spin_handle_.reset();
            finish_nav(res.code == rclcpp_action::ResultCode::SUCCEEDED, "spin");
          };
        spin_client_->async_send_goal(s, opts);
        return;
      }
    }
    if (!nav_client_->action_server_is_ready()) {finish_nav(false, "nav2 navigate_to_pose 서버 없음"); return;}
    auto opts = rclcpp_action::Client<Nav>::SendGoalOptions();
    opts.goal_response_callback = [this, gen](rclcpp_action::ClientGoalHandle<Nav>::SharedPtr h) {
        if (gen != nav_gen_) {if (h) {nav_client_->async_cancel_goal(h);} return;}
        if (!h) {finish_nav(false, "nav2 가 goal 거부"); return;}
        nav2_handle_ = h;
      };
    opts.feedback_callback = [this, gen](rclcpp_action::ClientGoalHandle<Nav>::SharedPtr,
      const std::shared_ptr<const Nav::Feedback> fb) {
        if (gen == nav_gen_ && nav_gh_ && nav_gh_->is_active()) {nav_gh_->publish_feedback(std::make_shared<Nav::Feedback>(*fb));}
      };
    opts.result_callback = [this, gen](const rclcpp_action::ClientGoalHandle<Nav>::WrappedResult & res) {
        if (gen != nav_gen_) {return;}
        nav2_handle_.reset();
        const bool ok = res.code == rclcpp_action::ResultCode::SUCCEEDED;
        if (!ok && loc_gate_.onNavFailed(now().seconds())) {   // 위치를 잃은 동안 끝남 → OK 되면 다시 (#121)
          RCLCPP_WARN(get_logger(), "navigate_to_pose 실패 (위치 추정 %s) → OK 되면 같은 목표 다시",
            loc_gate_.describe(now().seconds()).c_str());
          return;
        }
        finish_nav(ok, "navigate_to_pose");
      };
    nav_client_->async_send_goal(goal, opts);
  }

  // ---------- 박스 앞 정차 중 navigate: 먼저 곧게 후진 (#157) ----------
  void start_backoff()
  {
    backoff_active_ = true;
    backoff_start_ = now();
    backoff_from_.reset();
    const auto & c = box_park_.config();
    RCLCPP_INFO(get_logger(), "navigate: 박스 앞 정차 중 → 먼저 후진 (%.2f m/s, TFmini %.2f m 이상 확인 또는 최대 %.2f m)",
      c.backoff_speed, c.release_m, c.backoff_max_m);
  }

  void tick_backoff()
  {
    const auto & c = box_park_.config();
    const double el = (now() - backoff_start_).seconds();
    double traveled = 0.0;
    if (odom_ && fresh(odom_->header.stamp)) {
      const auto & p = odom_->pose.pose.position;
      if (!backoff_from_) {backoff_from_ = std::make_pair(p.x, p.y);}
      traveled = std::hypot(p.x - backoff_from_->first, p.y - backoff_from_->second);
    }
    // 뒤 확인 = 정책 주행과 같은 규칙: 뒤 /scan ±scan.half_angle 에 scan.rear_stop, 뒤 카메라 (#84). /scan 이 없으면 기다림
    const bool scan_ok = scan_ && fresh(scan_->header.stamp);
    const bool rear_blocked = (!scan_ok && require_scan_) ||
      (scan_ok && scan_blocked(scan_->ranges, scan_->angle_min, scan_->angle_increment, M_PI, scan_half_, scan_rear_stop_)) ||
      rear_camera_blocks(-c.backoff_speed, rear_camera_, rear_state());
    switch (backoff_step(c, box_park_.parked(), traveled, el, rear_blocked)) {
      case BackoffStep::DRIVE:
        publish_cmd(-c.backoff_speed, 0.0);
        return;
      case BackoffStep::HOLD:
        publish_cmd(0, 0);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "후진 대기: 뒤 막힘 (/scan %s, 뒤 카메라 %s)",
          scan_ok ? "확인" : "없음", rear_why().c_str());
        return;
      case BackoffStep::TIMEOUT:
        publish_cmd(0, 0);
        backoff_active_ = false;
        finish_nav(false, "박스 앞 후진 시간 초과 (뒤 막힘 또는 물러남 확인 못 함)");   // 정차는 유지 → mission 재시도 때 다시 후진
        return;
      case BackoffStep::MAX_DISTANCE:
        RCLCPP_WARN(get_logger(), "후진 %.2f m 갔지만 TFmini 로 물러남을 확인 못 함 — 평소 정지 거리로 되돌리고 Nav2", traveled);
        box_park_.clear();
        set_box_approach(0.0);
        break;
      case BackoffStep::RELEASED:   // TFmini 콜백이 이미 평소 정지 거리로 되돌림
        break;
    }
    publish_cmd(0, 0);
    backoff_active_ = false;
    RCLCPP_INFO(get_logger(), "박스 앞 후진 끝 (%.2f m, %.1f s) → Nav2", traveled, el);
    begin_nav();
  }

  // ---------- 뒤가 벽에 붙은 채 navigate: 먼저 곧게 전진 (11-19, control/pullout.hpp) ----------
  // 뒤 /scan 가장 가까운 점 (/scan 없음 · 오래됨 = nullopt)
  std::optional<double> pullout_rear_min()
  {
    if (!scan_ || !fresh(scan_->header.stamp)) {return std::nullopt;}
    return rear_min_range(scan_->ranges, scan_->angle_min, scan_->angle_increment, pull_.rear_half_angle);
  }

  bool pullout_front_blocked_now()
  {
    const bool scan_near = !scan_ || !fresh(scan_->header.stamp) ||
      scan_blocked(scan_->ranges, scan_->angle_min, scan_->angle_increment, 0.0, scan_half_, pull_.front_clear_m);
    return pullout_front_blocked(pull_, scan_near, box_.nearest(now().seconds()));
  }

  // 전진을 시작하면 true. 목표가 도착 기준 안 (spin 으로 처리) 이면 하지 않음 — 위치가 바뀌면 안 됨
  bool start_pullout()
  {
    const auto rear = pullout_rear_min();
    if (!rear) {return false;}
    const bool front = pullout_front_blocked_now();
    if (!pullout_wanted(pull_, *rear, front)) {
      if (pull_.enabled && *rear < pull_.rear_clear_m) {
        const auto tf = box_.nearest(now().seconds());
        double front_min = std::numeric_limits<double>::infinity();
        if (scan_ && fresh(scan_->header.stamp)) {
          for (size_t i = 0; i < scan_->ranges.size(); ++i) {
            const float r = scan_->ranges[i];
            if (!std::isfinite(r) || r <= 0.01f) {continue;}
            double a = scan_->angle_min + scan_->angle_increment * static_cast<double>(i);
            a = std::atan2(std::sin(a), std::cos(a));
            if (std::fabs(a) <= scan_half_ && r < front_min) {front_min = r;}
          }
        }
        RCLCPP_WARN(get_logger(), "navigate: 뒤 %.2f m 로 가깝지만 앞이 막혀 전진 없이 Nav2 (앞 /scan %.2f m < %.2f ?, TFmini %s < %.2f ?)",
          *rear, front_min, pull_.front_clear_m, tf ? std::to_string(*tf).substr(0, 5).c_str() : "값 없음", pull_.front_tfmini_m);
      }
      return false;
    }
    const auto & goal = *nav_gh_->get_goal();
    const std::string frame = goal.pose.header.frame_id.empty() ? map_frame_ : goal.pose.header.frame_id;
    if (auto cur = current_pose(frame)) {
      if (std::hypot(goal.pose.pose.position.x - cur->pose.position.x,
        goal.pose.pose.position.y - cur->pose.position.y) <= xy_tol_) {return false;}
    }
    pullout_active_ = true;
    pullout_start_ = now();
    pullout_from_.reset();
    RCLCPP_INFO(get_logger(), "navigate: 뒤 %.2f m (< %.2f) → 먼저 곧게 전진 (%.2f m/s, 최대 %.2f m)",
      *rear, pull_.rear_clear_m, pull_.speed, pull_.max_m);
    return true;
  }

  void tick_pullout()
  {
    const double el = (now() - pullout_start_).seconds();
    double traveled = 0.0;
    if (odom_ && fresh(odom_->header.stamp)) {
      const auto & p = odom_->pose.pose.position;
      if (!pullout_from_) {pullout_from_ = std::make_pair(p.x, p.y);}
      traveled = std::hypot(p.x - pullout_from_->first, p.y - pullout_from_->second);
    }
    const auto rear = pullout_rear_min();
    const char * why = nullptr;
    switch (pullout_step(pull_, rear.value_or(0.0), traveled, el, pullout_front_blocked_now())) {
      case PulloutStep::DRIVE:
        if (!rear) {why = "/scan 없음"; break;}   // 뒤를 못 보면 더 가지 않음
        publish_cmd(pull_.speed, 0.0);
        return;
      case PulloutStep::CLEAR: why = "뒤 비었음"; break;
      case PulloutStep::MAX_DISTANCE: why = "최대 거리"; break;
      case PulloutStep::FRONT_BLOCKED: why = "앞 막힘"; break;
      case PulloutStep::TIMEOUT: why = "시간 초과"; break;
    }
    publish_cmd(0, 0);
    pullout_active_ = false;
    RCLCPP_INFO(get_logger(), "전진 끝 (%s, %.3f m, %.1f s, 뒤 %.2f m) → Nav2", why, traveled, el, rear.value_or(NAN));
    begin_nav();
  }

  void cancel_nav2()
  {
    if (backoff_active_) {backoff_active_ = false; publish_cmd(0, 0);}
    if (pullout_active_) {pullout_active_ = false; publish_cmd(0, 0);}
    ++nav_gen_;
    loc_gate_.reset();
    if (nav2_handle_) {nav_client_->async_cancel_goal(nav2_handle_); nav2_handle_.reset();}
    if (spin_handle_) {spin_client_->async_cancel_goal(spin_handle_); spin_handle_.reset();}
  }

  void finish_nav(bool ok, const std::string & what)
  {
    if (!ok) {RCLCPP_WARN(get_logger(), "navigate 실패 (%s)", what.c_str());}
    end(nav_gh_, ok, std::make_shared<Nav::Result>());
  }

  // ---------- /drive/align ----------
  enum class AlignMode { WAIT, COARSE, FINE, BOX };   // BOX = goal.box_only (#160)

  void start_align(std::shared_ptr<AlignGH> gh)
  {
    preempt_all("새 /drive/align goal 로 대체");
    ref_yaw_.reset();
    align_gh_ = gh;
    fine_.reset();
    coarse_.reset();
    box_only_.reset();
    align_mode_ = AlignMode::WAIT;
    align_start_ = now();
    last_valid_ = align_start_;
    align_target_ = gh->get_goal()->target_distance;
    box_.reset();
    align_tol_ = gh->get_goal()->tolerance > 0.0 ? gh->get_goal()->tolerance : default_tol_;
    box_yaw_.reset();
    center_phase_ = CenterPhase::OFF;
    center_tried_ = false;
    center_iter_ = 0;
    center_last_measured_iter_ = 0;
    center_lateral_ = 0.0;
    center_note_.clear();
    center_given_ = false;
    if (gh->get_goal()->center) {   // 상자 중앙 맞춤만 (mission 이 잰 상자): 측정 없이 기동부터, 정밀 모드
      if (!center_cfg_.enabled) {finish_align(true, "중앙 맞춤 꺼짐 (align.center.enabled false) — 움직이지 않음"); return;}
      if (!odom_ || !fresh(odom_->header.stamp)) {finish_align(true, "중앙 맞춤 안 함: /odometry/filtered 없음"); return;}
      enter_box_approach(AlignMode::FINE);
      center_given_ = true;
      center_given_box_ = gh->get_goal()->center_box;
      center_tried_ = true;
      center_start_ = now();
      center_phase_ = CenterPhase::GIVEN;
      center_phase_start_ = center_start_;
      return;
    }
    if (gh->get_goal()->box_only) {   // 인식 결과 없이 바로 박스 모드: 지금 yaw 를 지킴 (#160)
      if (!odom_ || !fresh(odom_->header.stamp)) {finish_align(false, "박스 접근: /odometry/filtered 없음 (yaw 를 지킬 수 없음)"); return;}
      box_yaw_ = tf2::getYaw(odom_->pose.pose.orientation);
      enter_box_approach(AlignMode::BOX);
    }
  }

  // 정밀 · 박스: obstacle_stop 정지 거리를 낮추고 이 정렬이 맡음 (끝날 때 end_box_approach 가 정차 판단)
  void enter_box_approach(AlignMode mode)
  {
    align_mode_ = mode;
    last_valid_ = now();
    box_park_.clear();
    fine_stop_m_ = box_approach_stop(align_target_, box_stop_margin_, box_stop_min_);
    set_box_approach(fine_stop_m_);
    RCLCPP_INFO(get_logger(), "align 모드: %s, 목표 %.3f m",
      mode == AlignMode::BOX ? "박스 (퍽 없이, yaw 유지 + TFmini)" : "정밀(TFmini 박스 거리)", align_target_);
  }

  // 정렬이 끝날 때 (성공 · 실패 · 취소 · 선점) 낮춘 정지 거리 처리 (#157)
  //   FINE: 박스 앞 (TFmini < park_release_m · 무효) 이면 정차 = 낮춘 값 유지, 멀면 바로 되돌림
  //   COARSE · 모드 결정 전: 앞서 정차한 값이 있으면 유지 (TFmini 로 물러남을 확인하면 콜백이 되돌림)
  void end_box_approach()
  {
    if (!box_approach_set_) {return;}
    if (align_mode_ == AlignMode::FINE || align_mode_ == AlignMode::BOX) {
      const auto r = box_.value(now().seconds());
      if (box_park_.on_align_end(now().seconds(), fine_stop_m_, r)) {
        RCLCPP_INFO(get_logger(), "박스 앞 정차: TFmini %.3f m — 정지 거리 %.3f m 유지 (물러난 뒤 되돌림)", r ? *r : NAN, fine_stop_m_);
        return;
      }
    } else if (box_park_.parked()) {
      return;
    }
    set_box_approach(0.0);
  }

  // obstacle_stop 정지 거리 (0 = 평소로). 응답을 기다리지 않는다 — 실패하면 경고만 (정지 거리가 평소 값이면 박스 앞에서 서서 정렬이 시간 초과)
  void set_box_approach(double stop_m)
  {
    if (stop_m == 0.0 && !box_approach_set_) {return;}
    box_approach_set_ = stop_m != 0.0;
    if (!obstacle_params_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "%s 파라미터 서비스 없음 — 박스 접근 정지 거리 %.3f 를 넣지 못함", obstacle_stop_node_.c_str(), stop_m);
      return;
    }
    obstacle_params_->set_parameters({rclcpp::Parameter("approach_stop_m", stop_m)},
      [this, stop_m](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> f) {
        const auto r = f.get();
        if (r.empty() || !r[0].successful) {
          RCLCPP_WARN(get_logger(), "obstacle_stop approach_stop_m=%.3f 거부: %s", stop_m, r.empty() ? "" : r[0].reason.c_str());
        }
      });
  }

  void finish_align(bool ok, const std::string & msg)
  {
    if (ok && (align_mode_ == AlignMode::FINE || align_mode_ == AlignMode::BOX)) {   // turn_* 기준 yaw₀ (#77)
      if (odom_ && fresh(odom_->header.stamp)) {
        ref_yaw_ = tf2::getYaw(odom_->pose.pose.orientation);
      } else {
        RCLCPP_WARN(get_logger(), "정밀 정렬 끝 yaw 를 /odometry/filtered 에서 못 읽음 — turn_* 를 쓸 수 없음");
      }
    }
    end_box_approach();
    publish_cmd(0, 0);
    auto r = std::make_shared<Align::Result>();
    r->success = ok;
    r->message = msg;
    RCLCPP_INFO(get_logger(), "align(%s) %s: %s",
      align_mode_ == AlignMode::FINE ? "정밀" : align_mode_ == AlignMode::BOX ? "박스" : "먼 거리",
      ok ? "성공" : "실패", msg.c_str());
    end(align_gh_, ok, r);
  }

  void tick_align()
  {
    if (align_gh_->is_canceling()) {
      end_box_approach();
      publish_cmd(0, 0);
      auto r = std::make_shared<Align::Result>();
      r->message = "취소됨";
      end(align_gh_, false, r, true);
      return;
    }
    const auto t = now();
    if (center_phase_ != CenterPhase::OFF) {   // 상자 중앙 맞춤은 따로 시간을 잼 (측정이 Jetson 에서 장당 약 3 s)
      if ((t - center_start_).seconds() > center_timeout_s_) {center_fallback("중앙 맞춤 시간 초과"); return;}
    } else if ((t - align_start_).seconds() > align_timeout_s_) {
      finish_align(false, "시간 초과");
      return;
    }

    if (align_mode_ == AlignMode::WAIT) {
      // goal 수신 뒤 stamp 인 첫 result 의 모드로 결정 — 이후 바꾸지 않음
      if (result_ && rclcpp::Time(result_->header.stamp, RCL_ROS_TIME) >= align_start_ && result_->mode != 0) {
        align_mode_ = (result_->mode & PerceptionMode::ARM_ALIGN) ? AlignMode::FINE : AlignMode::COARSE;
        if (align_mode_ == AlignMode::COARSE && align_target_ < coarse_min_) {
          finish_align(false, "먼 거리 모드인데 target_distance < align.coarse_min (팔을 편 채 접근 방지)");
          return;
        }
        last_valid_ = t;
        if (align_mode_ == AlignMode::FINE) {
          enter_box_approach(AlignMode::FINE);
        } else {
          RCLCPP_INFO(get_logger(), "align 모드: 먼 거리(범퍼), 목표 %.3f m", align_target_);
        }
      } else if ((t - align_start_).seconds() > mode_wait_s_) {
        finish_align(false, "0.5 s 안에 새 perception result 없음");
      }
      return;
    }

    auto fb = std::make_shared<Align::Feedback>();
    AlignCmd c;
    if (align_mode_ == AlignMode::BOX) {
      const bool odom_ok = odom_ && fresh(odom_->header.stamp);
      const auto box = box_.value(t.seconds());
      if (!odom_ok || !box) {
        publish_cmd(0, 0);
        if ((t - last_valid_).seconds() > lost_s_) {
          finish_align(false, !odom_ok ? "박스 접근: /odometry/filtered 끊김" :
            "박스 거리 무효 (/tfmini/range 끊김 · 무효 · 박스 모서리)");
        }
        return;
      }
      last_valid_ = t;
      const double yaw_err = wrap_pi(*box_yaw_ - tf2::getYaw(odom_->pose.pose.orientation));
      c = box_only_.step(yaw_err, *box, align_target_, align_tol_);
      fb->lateral_error = static_cast<float>(yaw_err);
      fb->distance = static_cast<float>(*box);
      if (rear_camera_blocks(c.v, rear_camera_, rear_state())) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "박스 접근: 후방 장애물 (뒤 카메라: %s) — 후진 0", rear_why().c_str());
        c.v = 0.0;
      }
    } else if (align_mode_ == AlignMode::FINE && center_phase_ != CenterPhase::OFF) {
      tick_center(t);
      return;
    } else if (align_mode_ == AlignMode::FINE) {
      const bool off_ok = result_ && fresh(result_->header.stamp) && (result_->mode & PerceptionMode::ARM_ALIGN) && result_->offset_valid;
      const auto box = box_.value(t.seconds());
      if (!off_ok || !box) {
        publish_cmd(0, 0);   // 정밀 중 값이 끊겨도 먼 거리로 바꾸지 않음 (팔을 편 채 범퍼 기준 접근 = 충돌)
        if ((t - last_valid_).seconds() > lost_s_) {
          finish_align(false, !off_ok ? "물체 놓침 (arm_offset)" : "박스 거리 무효 (/tfmini/range 끊김 · 무효 · 박스 모서리)");
        }
        return;
      }
      last_valid_ = t;
      c = fine_.step(result_->arm_offset.x, *box, align_target_, align_tol_);
      fb->lateral_error = static_cast<float>(result_->arm_offset.x);
      fb->distance = static_cast<float>(*box);
      if (rear_camera_blocks(c.v, rear_camera_, rear_state())) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "정밀 정렬: 후방 장애물 (뒤 카메라: %s) — 후진 0", rear_why().c_str());
        c.v = 0.0;
      }
    } else {
      const bool obj_ok = result_ && fresh(result_->header.stamp) && result_->object_found;
      if (!obj_ok) {
        publish_cmd(0, 0);
        if ((t - last_valid_).seconds() > lost_s_) {finish_align(false, "물체 놓침 (object_pose)");}
        return;
      }
      last_valid_ = t;
      const auto & p = result_->object_pose.pose.position;
      c = coarse_.step(p.x, p.y, front_offset_, align_target_, std::max(align_tol_, 0.02));
      fb->lateral_error = static_cast<float>(std::atan2(p.y, p.x));
      fb->distance = static_cast<float>(p.x - front_offset_);
      // 직접 P 제어라 nav2 회피가 없음 → /scan 확인
      if (c.v != 0.0) {
        const bool scan_ok = scan_ && fresh(scan_->header.stamp);
        if (!scan_ok && require_scan_) {finish_align(false, "/scan 없음 — 먼 거리 접근 중지"); return;}
        if (scan_ok) {
          const bool fwd = c.v > 0;
          const double limit = fwd ? std::hypot(p.x, p.y) - scan_front_margin_ : scan_rear_stop_;
          if (scan_blocked(scan_->ranges, scan_->angle_min, scan_->angle_increment, fwd ? 0.0 : M_PI, scan_half_, limit)) {
            finish_align(false, fwd ? "전방 장애물 (물체보다 가까운 점)" : "후방 장애물 0.3 m");
            return;
          }
        }
        if (rear_camera_blocks(c.v, rear_camera_, rear_state())) {
          finish_align(false, "후방 장애물 (뒤 카메라: " + rear_why() + ")");
          return;
        }
      }
    }
    align_gh_->publish_feedback(fb);
    if (c.done && align_mode_ == AlignMode::FINE && center_cfg_.enabled && center_auto_ && !center_tried_) {start_center(); return;}
    if (c.done) {finish_align(true, center_note_.empty() ? "허용오차 안에서 정지" : "허용오차 안에서 정지 (" + center_note_ + ")"); return;}
    publish_cmd(c.v, c.w);
  }

  // ---------- 상자 중앙 맞춤 (FINE 뒤, control/box_center.hpp) ----------
  enum class CenterPhase { OFF, GIVEN, PRIOR, MEASURE, MOVE, HOLD, PUCK };   // GIVEN = goal.center (잰 상자를 받음)

  void start_center()
  {
    publish_cmd(0, 0);
    center_tried_ = true;
    center_start_ = now();
    if (!pb_client_) {return;}
    enter_center_prior();
  }

  // 측정 사전값 = TFmini 로 본 상자 중심 (base_link 앞 front_offset + 거리 + 반 변, 요 0) → 잠시 뒤 측정 서비스
  void enter_center_prior()
  {
    const auto r = box_.value(now().seconds());
    geometry_msgs::msg::PoseStamped pr;
    pr.header.stamp = now();
    pr.header.frame_id = base_frame_;
    pr.pose.position.x = front_offset_ + (r ? *r : align_target_) + center_box_half_;
    pr.pose.orientation.w = 1.0;
    pb_prior_pub_->publish(pr);
    center_svc_state_ = 0;
    center_phase_ = CenterPhase::PRIOR;
    center_phase_start_ = now();
    RCLCPP_INFO(get_logger(), "중앙 맞춤 %d: 상자 사전값 base_link x %.3f → %s", center_iter_ + 1, pr.pose.position.x, center_service_.c_str());
  }

  // 움직이기 전 (측정 실패 · 믿기 어려운 값): 지금 자리가 FINE 결과 그대로이므로 성공으로 끝냄
  void center_skip(const std::string & why)
  {
    RCLCPP_WARN(get_logger(), "중앙 맞춤 안 함 (%s)", why.c_str());
    center_phase_ = CenterPhase::OFF;
    finish_align(true, "허용오차 안에서 정지 (중앙 맞춤 안 함: " + why + ")");
  }

  // 움직인 뒤 문제: 기존 FINE (퍽 arm_offset + TFmini) 로 되돌아가 처음부터 다시 맞춤 (시간도 새로)
  void center_fallback(const std::string & why)
  {
    RCLCPP_WARN(get_logger(), "중앙 맞춤 포기 (%s) → 기존 FINE 으로", why.c_str());
    publish_cmd(0, 0);
    center_phase_ = CenterPhase::OFF;
    center_note_ = "중앙 맞춤 포기: " + why;
    fine_.reset();
    align_start_ = now();
    last_valid_ = align_start_;
  }

  void tick_center(const rclcpp::Time & t)
  {
    auto fb = std::make_shared<Align::Feedback>();
    const bool odom_ok = odom_ && fresh(odom_->header.stamp);
    const auto range = box_.value(t.seconds());
    fb->lateral_error = static_cast<float>(center_lateral_);
    fb->distance = static_cast<float>(range.value_or(NAN));
    align_gh_->publish_feedback(fb);
    if (!odom_ok) {
      publish_cmd(0, 0);
      if ((t - last_valid_).seconds() > lost_s_) {
        if (center_phase_ == CenterPhase::GIVEN || center_phase_ == CenterPhase::PRIOR || center_phase_ == CenterPhase::MEASURE) {
          center_skip("/odometry/filtered 끊김");
        } else {
          center_fallback("/odometry/filtered 끊김");
        }
      }
      return;
    }
    last_valid_ = t;
    const double rx = odom_->pose.pose.position.x, ry = odom_->pose.pose.position.y;
    const double ryaw = tf2::getYaw(odom_->pose.pose.orientation);
    const double in_phase = (t - center_phase_start_).seconds();

    if (center_phase_ == CenterPhase::PRIOR) {
      publish_cmd(0, 0);
      if (in_phase < center_prior_delay_s_) {return;}
      if (!pb_client_->service_is_ready()) {center_skip(center_service_ + " 서비스 없음"); return;}
      center_req_t_ = now();
      const int gen = ++center_req_gen_;
      pb_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this, gen](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
          if (gen != center_req_gen_) {return;}
          const auto r = f.get();
          center_svc_state_ = r->success ? 1 : -1;
          center_svc_msg_ = r->message;
        });
      center_phase_ = CenterPhase::MEASURE;
      center_phase_start_ = now();
      return;
    }

    if (center_phase_ == CenterPhase::MEASURE) {
      publish_cmd(0, 0);
      if (center_svc_state_ < 0) {center_skip("측정 거부: " + center_svc_msg_); return;}
      const bool st_new = pb_status_ && pb_status_rx_ > center_req_t_;
      const bool pose_new = pb_pose_ && pb_pose_rx_ > center_req_t_;
      if (!pose_new) {
        if (st_new && pb_status_->data.find("\"ok\": false") != std::string::npos) {center_skip("상자 못 찾음 " + pb_status_->data); return;}
        if (in_phase > center_measure_timeout_s_) {center_skip("측정 시간 초과"); return;}
        return;
      }
      center_use_box(*pb_pose_, rx, ry, ryaw);
      return;
    }

    if (center_phase_ == CenterPhase::GIVEN) {   // goal.center: 받은 상자로 바로 판단 (측정 없음)
      publish_cmd(0, 0);
      center_use_box(center_given_box_, rx, ry, ryaw);
      return;
    }

    if (center_phase_ == CenterPhase::MOVE) {
      if (shift_.phase() != center_move_phase_) {center_move_phase_ = shift_.phase(); center_phase_start_ = t;}
      else if (in_phase > center_phase_timeout_s_) {center_fallback("기동 단계 시간 초과"); return;}
      if (shift_.forward() && range && *range < center_fwd_min_m_) {shift_.cut_forward();}   // TFmini 가 너무 가까우면 비스듬 전진 끝
      const auto c = shift_.step(rx, ry, ryaw);
      if (c.done) {enter_center_hold(); return;}
      if (c.v < 0.0) {
        const bool scan_ok = scan_ && fresh(scan_->header.stamp);
        if (!scan_ok && require_scan_) {center_fallback("후진 중 /scan 없음"); return;}
        if (scan_ok && scan_blocked(scan_->ranges, scan_->angle_min, scan_->angle_increment, M_PI, scan_half_, scan_rear_stop_)) {
          center_fallback("후방 장애물 (/scan)");
          return;
        }
        if (rear_camera_blocks(c.v, rear_camera_, rear_state())) {center_fallback("후방 장애물 (뒤 카메라: " + rear_why() + ")"); return;}
      }
      publish_cmd(c.v, c.w);
      return;
    }

    if (center_phase_ == CenterPhase::PUCK) {tick_center_puck(t, rx, ry, ryaw, range); return;}

    // HOLD: 법선 yaw 유지 + TFmini 거리 (BoxApproach). 끝나면 반복 남았으면 다시 잼
    if (!range) {
      publish_cmd(0, 0);
      if (in_phase > lost_s_ && (t - center_range_ok_).seconds() > lost_s_) {center_fallback("유지 중 박스 거리 무효");}
      return;
    }
    center_range_ok_ = t;
    auto c = box_only_.step(wrap_pi(center_heading_ - ryaw), *range, align_target_, align_tol_);
    if (rear_camera_blocks(c.v, rear_camera_, rear_state())) {c.v = 0.0;}
    if (c.done) {
      if (!center_given_ && pb_client_ && center_iter_ < center_cfg_.max_iter && center_last_measured_iter_ < center_iter_) {
        center_last_measured_iter_ = center_iter_;
        enter_center_prior();   // 옮긴 뒤 다시 잼
        return;
      }
      if (center_cfg_.final_puck && center_iter_ > 0) {   // 옮겼으면 퍽 arm_offset 으로 마무리
        fine_.reset();
        center_puck_checked_ = false;
        center_phase_ = CenterPhase::PUCK;
        center_phase_start_ = t;
        return;
      }
      center_finish("");
      return;
    }
    publish_cmd(c.v, c.w);
  }

  // 잰 상자 (자동 측정 · goal.center) → 법선 · 옆 어긋남 → 건너뜀 · 유지 · 옆 이동 기동
  void center_use_box(const geometry_msgs::msg::PoseStamped & box, double rx, double ry, double ryaw)
  {
    double bx = box.pose.position.x, by = box.pose.position.y;
    double byaw = tf2::getYaw(box.pose.orientation);
    const std::string & of = odom_->header.frame_id;
    if (!of.empty() && !box.header.frame_id.empty() && box.header.frame_id != of) {   // place_box = odom_frame, /odometry/filtered 와 다르면 옮김
      try {
        const auto tr = tf_buffer_->lookupTransform(of, box.header.frame_id, tf2::TimePointZero);
        geometry_msgs::msg::PoseStamped o;
        tf2::doTransform(box, o, tr);
        bx = o.pose.position.x;
        by = o.pose.position.y;
        byaw = tf2::getYaw(o.pose.orientation);
      } catch (const tf2::TransformException & e) {
        center_skip(std::string("상자 프레임 변환 실패: ") + e.what());
        return;
      }
    }
    const auto e = center_error(rx, ry, ryaw, bx, by, byaw);
    center_box_ = {bx, by};
    center_lateral_ = e.lateral;
    center_heading_ = e.heading;
    const auto d = decide_center(e, center_cfg_);
    RCLCPP_INFO(get_logger(), "중앙 맞춤 %d: 상자 (%.3f, %.3f) 요 %.1f° → 옆 %+.1f mm · 요 오차 %+.2f° · 깊이 %.3f m",
      center_iter_ + 1, bx, by, byaw * 180 / M_PI, e.lateral * 1000, e.yaw_err * 180 / M_PI, -e.depth);
    if (d == CenterDecision::BEHIND || d == CenterDecision::TOO_FAR) {
      const std::string why = d == CenterDecision::BEHIND ? "상자가 앞에 없음" : "옆 어긋남이 max_shift 초과";
      if (center_iter_ == 0) {center_skip(why); return;}
      enter_center_hold();   // 이미 한 번 옮김: 측정이 이상하면 법선 유지만 하고 끝냄
      return;
    }
    if (d == CenterDecision::DONE || center_iter_ >= center_cfg_.max_iter) {enter_center_hold(); return;}
    // 뒤 여유 (pullout 처럼 뒤 /scan ± rear_half_angle 가장 가까운 점 − scan.rear_stop) 로 후진 거리 제한
    const bool scan_ok = scan_ && fresh(scan_->header.stamp);
    if (!scan_ok && require_scan_) {
      if (center_iter_ == 0) {center_skip("/scan 없음 — 뒤 여유 모름"); return;}
      enter_center_hold();
      return;
    }
    const double rear_min = scan_ok ?
      rear_min_range(scan_->ranges, scan_->angle_min, scan_->angle_increment, center_rear_half_) :
      std::numeric_limits<double>::infinity();
    const double room = rear_back_room(rear_min, scan_rear_stop_, center_cfg_);
    if (room <= 0.0) {
      const std::string why = "뒤 여유 부족 (뒤 /scan " + std::to_string(rear_min) + " m)";
      if (center_iter_ == 0) {center_skip(why); return;}
      enter_center_hold();
      return;
    }
    const auto p = plan_shift(e, center_cfg_, room);
    shift_.start(p);
    ++center_iter_;
    center_phase_ = CenterPhase::MOVE;
    center_phase_start_ = now();
    center_move_phase_ = shift_.phase();
    RCLCPP_INFO(get_logger(), "중앙 맞춤 기동: 법선 %.1f° · 후진 %.3f m · %+.1f° · 전진 %.3f m",
      p.heading * 180 / M_PI, p.back, p.alpha * 180 / M_PI, p.fwd);
  }

  void center_finish(const std::string & extra)
  {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "상자 중앙 맞춤 %d 회, 마지막 측정 옆 %+.1f mm", center_iter_, center_lateral_ * 1000);
    center_phase_ = CenterPhase::OFF;
    finish_align(true, extra.empty() ? std::string(buf) : std::string(buf) + ", " + extra);
  }

  // PUCK: 퍽 arm_offset → 회전, TFmini → 전후 (기존 FINE 법칙). 법선에서 max_puck_yaw_dev 넘게 돌아야 하면 돌지 않고 경고
  void tick_center_puck(const rclcpp::Time & t, double rx, double ry, double ryaw, const std::optional<double> & range)
  {
    const double dev_lim = center_cfg_.max_puck_yaw_dev;
    if (!center_puck_checked_ && arm_target_ && fresh(arm_target_->header.stamp)) {   // 돌기 전에 퍽 위치로 미리 판단
      center_puck_checked_ = true;
      try {
        const auto tr = tf_buffer_->lookupTransform(odom_->header.frame_id, arm_target_->header.frame_id, tf2::TimePointZero);
        geometry_msgs::msg::PointStamped o;
        tf2::doTransform(*arm_target_, o, tr);
        const auto k = puck_check(rx, ry, o.point.x, o.point.y, center_box_.first, center_box_.second, center_heading_, center_cfg_);
        if (!k.rotate) {
          RCLCPP_WARN(get_logger(), "퍽이 상자 중심선에서 %+.1f mm 벗어남 (방향 %+.1f° > %.1f°) — 칸 어긋남? 퍽 쪽으로 돌지 않음",
            k.lateral * 1000, k.yaw_dev * 180 / M_PI, dev_lim * 180 / M_PI);
          center_finish("퍽 마무리 안 함: 퍽 방향 벌어짐");
          return;
        }
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "퍽 마무리: arm_target 변환 실패 (%s) — 회전 중 확인만", e.what());
      }
    }
    const bool off_ok = result_ && fresh(result_->header.stamp) && (result_->mode & PerceptionMode::ARM_ALIGN) && result_->offset_valid;
    if (!off_ok || !range) {
      publish_cmd(0, 0);
      if ((t - center_phase_start_).seconds() > lost_s_ && (t - center_range_ok_).seconds() > lost_s_) {
        RCLCPP_WARN(get_logger(), "퍽 마무리 건너뜀: %s", !off_ok ? "퍽이 화면에 없음 (arm_offset)" : "TFmini 무효");
        center_finish("퍽 마무리 건너뜀");
      }
      return;
    }
    center_range_ok_ = t;
    auto c = fine_.step(result_->arm_offset.x, *range, align_target_, align_tol_);
    const double dev = wrap_pi(ryaw - center_heading_);
    if (std::fabs(dev) > dev_lim && c.w * dev > 0.0) {   // 더 벌어지려 함 (미리 판단 못 했을 때 막음)
      RCLCPP_WARN(get_logger(), "퍽이 상자 중심선에서 벗어남 (법선에서 %+.1f° 넘게 돌아야 함) — 칸 어긋남? 퍽 마무리 멈춤", dev * 180 / M_PI);
      center_finish("퍽 마무리 멈춤: 퍽 방향 벌어짐");
      return;
    }
    if (rear_camera_blocks(c.v, rear_camera_, rear_state())) {c.v = 0.0;}
    if (c.done) {center_finish("퍽 마무리 회전 " + std::to_string(dev * 180 / M_PI).substr(0, 5) + "°"); return;}
    publish_cmd(c.v, c.w);
  }

  void enter_center_hold()
  {
    box_only_.reset();
    center_phase_ = CenterPhase::HOLD;
    center_phase_start_ = now();
    center_range_ok_ = now();
  }

  // ---------- /drive/execute_skill ----------
  void start_skill(std::shared_ptr<SkillGH> gh)
  {
    preempt_all("새 execute_skill goal 로 대체");
    skill_gh_ = gh;
    skill_start_ = now();
    skill_timeout_ = gh->get_goal()->timeout > 0 ? gh->get_goal()->timeout : 30.0;
    turn_active_ = false;
    if (const auto deg = parse_turn_deg(gh->get_goal()->policy)) {
      skill_ = nullptr;
      if (!ref_yaw_) {
        finish_skill(false, "turn: 정렬 방향(yaw₀) 없음 — 직전 FINE 정렬 성공이 필요");
        return;
      }
      const double ang = turn_angle_rad(*deg, turn_geo_);
      turn_target_ = wrap_pi(*ref_yaw_ + ang);
      turn_total_ = 0.0;
      turn_last_odom_ = skill_start_;
      turn_ctl_.reset();
      turn_active_ = true;
      skill_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_hz_),
          [this]() {tick_skill();}, cbg_);
      RCLCPP_INFO(get_logger(), "skill %s 시작: yaw₀ %.2f° %+.2f° (이름 %+.2f°, icr_x %.4f), %.1f s",
        gh->get_goal()->policy.c_str(), *ref_yaw_ * 180 / M_PI, ang * 180 / M_PI, *deg, turn_geo_.icr_x, skill_timeout_);
      return;
    }
    skill_ = policies_.find(gh->get_goal()->policy);
    skill_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / skill_->spec.rate_hz),
        [this]() {tick_skill();}, cbg_);
    RCLCPP_INFO(get_logger(), "skill %s 시작 (%.0f Hz, %.1f s)", skill_->spec.name.c_str(), skill_->spec.rate_hz, skill_timeout_);
  }

  // 타이머 콜백(tick_skill) 안에서 끝날 수 있으므로 파괴하지 않고 멈추기만 한다 (다음 start_skill 에서 교체)
  void stop_skill_timer()
  {
    if (skill_timer_) {skill_timer_->cancel();}
  }

  void finish_skill(bool ok, const std::string & msg, bool canceled = false)
  {
    publish_cmd(0, 0);
    stop_skill_timer();
    auto r = std::make_shared<Skill::Result>();
    r->success = ok;
    r->message = msg;
    RCLCPP_INFO(get_logger(), "skill %s: %s", ok ? "성공" : "실패", msg.c_str());
    end(skill_gh_, ok, r, canceled);
  }

  std::optional<std::vector<float>> scan_bins()
  {
    if (!scan_ || !fresh(scan_->header.stamp)) {return std::nullopt;}
    std::vector<float> b(scan_bins_, scan_->range_max);
    for (size_t i = 0; i < scan_->ranges.size(); ++i) {
      const float r = scan_->ranges[i];
      if (!std::isfinite(r) || r < scan_->range_min) {continue;}
      double a = scan_->angle_min + scan_->angle_increment * i;
      a = std::atan2(std::sin(a), std::cos(a));   // -π ~ π
      const int k = std::clamp(static_cast<int>((a + M_PI) / (2 * M_PI) * scan_bins_), 0, scan_bins_ - 1);
      b[k] = std::min(b[k], std::min(r, scan_->range_max));
    }
    return b;
  }

  void tick_skill()
  {
    if (!skill_gh_) {stop_skill_timer(); return;}
    if (skill_gh_->is_canceling()) {finish_skill(false, "취소됨", true); return;}
    const double el = (now() - skill_start_).seconds();
    if (turn_active_) {tick_turn(el); return;}
    if (el >= skill_timeout_) {
      if (skill_->spec.finish_on_timeout) {finish_skill(true, "정해진 시간 완료");} else {finish_skill(false, "시간 초과");}
      return;
    }
    robot_policy::ObsValues v;
    v["scan"] = scan_bins();
    if (odom_ && fresh(odom_->header.stamp)) {
      v["odom_twist"] = std::vector<float>{static_cast<float>(odom_->twist.twist.linear.x),
        static_cast<float>(odom_->twist.twist.linear.y), static_cast<float>(odom_->twist.twist.angular.z)};
    } else {
      v["odom_twist"] = std::nullopt;
    }
    if (result_ && fresh(result_->header.stamp) && result_->object_found) {
      const auto & p = result_->object_pose.pose.position;
      v["object_pose_base"] = std::vector<float>{static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)};
    } else {
      v["object_pose_base"] = std::nullopt;
    }
    std::vector<float> obs, raw;
    std::string err;
    if (!robot_policy::ObservationBuilder(skill_->spec).build(v, obs, err)) {finish_skill(false, "필수 관측 무효: " + err); return;}
    if (!skill_->runner->run(obs, raw)) {finish_skill(false, "추론 실패"); return;}
    const auto a = robot_policy::ActionMapper(skill_->spec).map(raw);
    double cmd_v = std::clamp<double>(a[0], -max_v_, max_v_);
    const double cmd_w = std::clamp<double>(a[1], -max_w_, max_w_);
    // 정책 출력은 nav2 · collision_monitor 를 거치지 않음 → 진행 방향 /scan 확인 (align 과 같은 규칙)
    //   막힌 방향의 전후 속도만 0, 회전은 그대로 (정책이 돌아서 빠져나갈 수 있게)
    if (std::fabs(cmd_v) > 1e-3) {
      const bool scan_ok = scan_ && fresh(scan_->header.stamp);
      if (!scan_ok && require_scan_) {finish_skill(false, "/scan 없음 — 정책 주행 중지"); return;}
      if (scan_ok) {
        const bool fwd = cmd_v > 0;
        const double limit = fwd ? front_offset_ + skill_front_margin_ : scan_rear_stop_;
        if (scan_blocked(scan_->ranges, scan_->angle_min, scan_->angle_increment, fwd ? 0.0 : M_PI, scan_half_, limit)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "skill: %s 장애물 — 전후 속도 0", fwd ? "전방" : "후방");
          cmd_v = 0.0;
        }
      }
      if (rear_camera_blocks(cmd_v, rear_camera_, rear_state())) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "skill: 후방 장애물 (뒤 카메라: %s) — 전후 속도 0", rear_why().c_str());
        cmd_v = 0.0;
      }
    }
    publish_cmd(cmd_v, cmd_w);
    auto fb = std::make_shared<Skill::Feedback>();
    fb->progress = static_cast<float>(el / skill_timeout_);
    skill_gh_->publish_feedback(fb);
  }

  // 제자리 회전 (v 0). 회전 여유는 계획기 BOX_GAP 이 보장 — /scan 확인 없음 (11-20 앞 라이다를 낮춰 스캔면 59.75 mm 가 박스 70 mm 를 보게 됐지만, 제자리 회전 여유는 BOX_GAP 로 보장)
  void tick_turn(double el)
  {
    if (el >= skill_timeout_) {finish_skill(false, "turn: 시간 초과"); return;}
    if (!odom_ || !fresh(odom_->header.stamp)) {
      publish_cmd(0, 0);
      if ((now() - turn_last_odom_).seconds() > lost_s_) {finish_skill(false, "turn: /odometry/filtered 끊김");}
      return;
    }
    turn_last_odom_ = now();
    const double err = wrap_pi(turn_target_ - tf2::getYaw(odom_->pose.pose.orientation));
    if (turn_total_ <= 0.0) {turn_total_ = std::max(std::fabs(err), 1e-3);}
    const auto c = turn_ctl_.step(err);
    if (c.done) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "turn: 끝 yaw 오차 %.2f°", err * 180 / M_PI);
      finish_skill(true, buf);
      return;
    }
    publish_cmd(0.0, c.w);
    auto fb = std::make_shared<Skill::Feedback>();
    fb->progress = static_cast<float>(std::clamp(1.0 - std::fabs(err) / turn_total_, 0.0, 1.0));
    skill_gh_->publish_feedback(fb);
  }

  // 뒤 카메라 최신 값 (받은 지 stale_s 넘으면 없음 = nullopt)
  std::optional<bool> rear_state()
  {
    if (!rear_ || (now() - rear_rx_).seconds() > stale_s_) {return std::nullopt;}
    return rear_;
  }

  std::string rear_why()
  {
    if (!rear_state()) {return "값 없음 · 오래됨";}
    return rear_reason_.empty() ? "막힘" : rear_reason_;
  }

  void tick()
  {
    if (nav_gh_ && nav_gh_->is_canceling()) {
      cancel_nav2();
      end(nav_gh_, false, std::make_shared<Nav::Result>(), true);
    }
    if (nav_gh_) {
      if (backoff_active_) {tick_backoff();} else if (pullout_active_) {tick_pullout();} else {tick_loc_gate();}
    }
    if (align_gh_) {tick_align();}
    if (box_park_.refresh_due(now().seconds())) {set_box_approach(box_park_.stop_m());}   // 정차 중 다시 보냄 (#157)
  }

  void tick_loc_gate()
  {
    const double t = now().seconds();
    switch (loc_gate_.tick(t, static_cast<bool>(nav2_handle_))) {
      case LocGate::Action::SEND:
        RCLCPP_INFO(get_logger(), "navigate: 위치 추정 OK → 목표 보냄");
        dispatch_nav();
        break;
      case LocGate::Action::RESEND:
        RCLCPP_WARN(get_logger(), "navigate: 위치를 다시 찾음 → 같은 목표 다시 보냄 (낡은 경로 버림)");
        cancel_nav2();
        dispatch_nav();
        break;
      case LocGate::Action::FAIL:
        finish_nav(false, "위치 추정 OK 기다림 시간 초과 (" + loc_gate_.describe(t) + ")");
        break;
      case LocGate::Action::NONE:
        break;
    }
  }

  // 파라미터
  double rate_hz_, lost_s_, mode_wait_s_, align_timeout_s_, min_fine_target_, coarse_min_, default_tol_, front_offset_;
  double scan_half_, scan_front_margin_, scan_rear_stop_, stale_s_, xy_tol_, spin_time_, max_v_, max_w_, skill_front_margin_;
  bool require_scan_, skill_enabled_, rear_camera_;
  int scan_bins_;
  std::string map_frame_, base_frame_, nav_action_name_, spin_action_name_;
  AlignController fine_;
  LocGate loc_gate_;
  CoarseAlign coarse_;
  BoxApproach box_only_;
  std::optional<double> box_yaw_;   // 박스 접근 목표 yaw (odom, goal 받은 때)
  // 상자 중앙 맞춤 (align.center.*)
  BoxCenterConfig center_cfg_;
  ShiftManeuver shift_;
  CenterPhase center_phase_ = CenterPhase::OFF;
  ShiftManeuver::Phase center_move_phase_ = ShiftManeuver::Phase::DONE;
  bool center_tried_ = false;
  bool center_auto_ = false;    // align.center.auto_after_fine: FINE 안에서 스스로 잼 (팔 ALIGN 자세라 대개 못 찾음)
  bool center_given_ = false;   // 이 goal 이 goal.center (mission 이 잰 상자)
  geometry_msgs::msg::PoseStamped center_given_box_;
  int center_iter_ = 0, center_last_measured_iter_ = 0, center_req_gen_ = 0, center_svc_state_ = 0;
  double center_lateral_ = 0.0, center_heading_ = 0.0;
  std::string center_service_, center_svc_msg_, center_note_;
  double center_box_half_ = 0.1025, center_prior_delay_s_ = 0.3, center_measure_timeout_s_ = 12.0;
  double center_phase_timeout_s_ = 8.0, center_timeout_s_ = 45.0, center_fwd_min_m_ = 0.12;
  rclcpp::Time center_start_{0, 0, RCL_ROS_TIME}, center_phase_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time center_req_t_{0, 0, RCL_ROS_TIME}, center_range_ok_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped::ConstSharedPtr pb_pose_;
  std_msgs::msg::String::ConstSharedPtr pb_status_;
  rclcpp::Time pb_pose_rx_{0, 0, RCL_ROS_TIME}, pb_status_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pb_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pb_status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pb_prior_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pb_client_;
  std::pair<double, double> center_box_{0.0, 0.0};   // 측정한 상자 중심 (odom)
  double center_rear_half_ = 0.785;
  bool center_puck_checked_ = false;
  geometry_msgs::msg::PointStamped::ConstSharedPtr arm_target_;   // /perception/arm_target (퍽 중심, base_footprint)
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr arm_target_sub_;
  robot_policy::PolicyLibrary policies_;

  // 입력 (최신값, 300 ms 규칙은 쓸 때 확인)
  PerceptionResult::ConstSharedPtr result_;
  rclcpp::Time result_rx_{0, 0, RCL_ROS_TIME};
  BoxRangeFilter box_;   // /tfmini/range (박스 앞면)
  double box_stop_margin_ = 0.03, box_stop_min_ = 0.10;
  std::string obstacle_stop_node_;
  bool box_approach_set_ = false;
  // 박스 앞 정차 (#157)
  BoxPark box_park_;
  double fine_stop_m_ = 0.0;   // 이번 FINE 정렬이 넣은 정지 거리
  bool backoff_active_ = false;
  rclcpp::Time backoff_start_{0, 0, RCL_ROS_TIME};
  std::optional<std::pair<double, double>> backoff_from_;   // 후진 시작 odom (x, y)
  PulloutConfig pull_;
  bool pullout_active_ = false;
  rclcpp::Time pullout_start_{0, 0, RCL_ROS_TIME};
  std::optional<std::pair<double, double>> pullout_from_;   // 전진 시작 odom (x, y)
  std::shared_ptr<rclcpp::AsyncParametersClient> obstacle_params_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  std::optional<bool> rear_;   // /perception/rear_obstacle (#84)
  rclcpp::Time rear_rx_{0, 0, RCL_ROS_TIME};
  std::string rear_reason_;

  // goal
  std::shared_ptr<NavGH> nav_gh_;
  std::shared_ptr<AlignGH> align_gh_;
  std::shared_ptr<SkillGH> skill_gh_;
  rclcpp_action::ClientGoalHandle<Nav>::SharedPtr nav2_handle_;
  rclcpp_action::ClientGoalHandle<Spin>::SharedPtr spin_handle_;
  uint64_t nav_gen_ = 0;
  AlignMode align_mode_ = AlignMode::WAIT;
  rclcpp::Time align_start_{0, 0, RCL_ROS_TIME}, last_valid_{0, 0, RCL_ROS_TIME}, skill_start_{0, 0, RCL_ROS_TIME};
  double align_target_ = 0.15, align_tol_ = 0.01, skill_timeout_ = 30.0;
  robot_policy::LoadedPolicy * skill_ = nullptr;
  // turn_* (#77)
  TurnController turn_ctl_;
  TurnGeometry turn_geo_;
  std::optional<double> ref_yaw_;   // 직전 FINE 정렬 성공 때 yaw (odom frame)
  bool turn_active_ = false;
  double turn_target_ = 0.0, turn_total_ = 0.0, turn_max_deg_ = 30.0;
  rclcpp::Time turn_last_odom_{0, 0, RCL_ROS_TIME};

  rclcpp::CallbackGroup::SharedPtr cbg_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Subscription<PerceptionResult>::SharedPtr perc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr box_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr loc_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rear_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rear_reason_sub_;
  rclcpp_action::Client<Nav>::SharedPtr nav_client_;
  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  rclcpp_action::Server<Nav>::SharedPtr nav_server_;
  rclcpp_action::Server<Align>::SharedPtr align_server_;
  rclcpp_action::Server<Skill>::SharedPtr skill_server_;
  rclcpp::TimerBase::SharedPtr timer_, skill_timer_;
};

}  // namespace control

RCLCPP_COMPONENTS_REGISTER_NODE(control::DriveNode)
