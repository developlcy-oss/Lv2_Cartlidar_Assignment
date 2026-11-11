// ManipulatorNode — 팔 · 그리퍼 제어 (Controller 팀)
// 명세: docs/nodes/manipulator_node.md
//
// 통신 규약: docs/interfaces.md
// - Action 서버 /manipulator/pose, /manipulator/pick, /manipulator/execute_skill (← mission_node)
// - Action 클라이언트 /arm_controller/follow_joint_trajectory, /gripper_controller/gripper_cmd
// - Topic 발행 /arm_controller/joint_trajectory (강화학습 정책 스트리밍)
// - 팔 · 그리퍼 명령은 이 노드만 보낸다. 작업은 한 번에 하나: 새 goal이 오면 이전 goal은 abort
// - 인식 값은 header.stamp(ROS 시계 촬영 시각) 기준 stale_timeout(300ms)보다 오래되면 버린다 (예외 없음)
// - 물체 위치는 perception 결과(object_found · object_pose · /perception/pucks)만 쓴다. 기억하는 것은 target_look 이 잰
//   퍽 위치 하나뿐 (odom, 바로 다음 toppick_tilt 가 쓰고 지움, #82)
// - robot_interfaces는 팀 공용 정의를 그대로 쓴다 (이 노드 때문에 바꾸지 않는다)
// - pick 목표 우선순위: ① 정밀 정렬 결과 (arm_offset · /arm_range → 팔 끝 축 위 지점) ② 추적 위치
// - /arm_range: perception_node 가 D435 · Arducam 측위로 낸 "팔 끝(range_frame = tool_link) → 정렬 중인 퍽 표면" 거리.
//   미세 보정의 전후 성분 · 최종 전진 거리에 쓴다. TFmini(1층, 박스 앞면 거리)는 drive 정렬 전용이라 여기서 쓰지 않는다
// - align 진행 중에는 팔을 움직이지 않는다 — 순서는 mission이 보장
// - 그리퍼 (gripper.mode=arm_joint, 실물): 6 번째 칸 gripper_roll = 그리퍼 개폐 모터 (열림 -0.764 · 닫힘 +0.923 rad).
//   arm_controller 궤적의 6 번째 값으로 여닫고, 모든 팔 궤적에 지금 그리퍼 명령을 같이 실어 쥔 퍽을 놓치지 않는다.
//   펌웨어 GripGuard 가 막히면 약하게 쥔다 → 닫은 뒤 위치가 닫힘 명령보다 grasp_margin 넘게 덜 닫혔으면 잡음
// - 주사위 미션 skill (/manipulator/execute_skill 의 이름, mission_planner mission_lines.py · 이슈 #63 #74). 같은 이름의
//   정책이 로드돼 있으면 정책을, 없으면 아래 고정 동작을 쓴다 (강화학습은 IK 대체 경로를 항상 유지)
//     target_look:<frame>:<x>:<y>:<z>  퍽 예상 중심을 팔 카메라로 내려다보는 타겟체크자세 (target_look_pose.hpp 로 계산) → 머문 뒤
//                                    /perception/pucks 에서 예상 자리 (target_look.match_distance 안) 퍽을 재서 odom 에 붙잡음 (#82).
//                                    팔 카메라 퍽 우선, target_look.arm_wait 뒤에는 D435 퍽도. 45° 집기 중에는 팔 카메라가 퍽을 못 봄
//     box_look:<frame>:<x>:<y>:<z>     장소 3×3 칸 (중심 x, y · 상자 윗면 z) 을 팔 카메라로 위에서 내려다보는 자세로 이동 (#65,
//                                    mission verify). target_look 과 같은 자세 계산 (target_look_pose.hpp), 시선 · 거리만 box_look.*.
//                                    퍽은 재지 않는다 (칸 판정은 perception check_dice). 인지 모의: 윗면 약 0.20 m 위 · 거의 수직
//     barcode_look:<frame>:<x>:<y>:<z> 벽의 바코드 라벨 중심을 팔 카메라로 거의 수평으로 보는 자세로 이동 (#65, mission observe).
//                                    출발 자세에서 라벨까지 J1 0.60 m 라 카메라는 약 0.3 m 가 가장 가까움 (인지: 0.3 m 에서 읽음)
//     toppick_tilt                   박스 위 퍽을 집게 45° 로 집기 (pick 과 같은 과정, 기울기만 toppick.pitch).
//                                    목표 = 직전 target_look 이 잰 위치, 없으면 ② 추적 위치. ① 정밀 정렬 결과 · 미세 보정은
//                                    toppick.use_alignment 일 때만 (45° 는 집게 축이 퍽을 보지 않아 기본 끔, #82)
//                                    접근 toppick.approach_mode top (기본): 잡는 점 위 · J1 쪽에서 벌린 채 직선으로 내려와 쥠,
//                                    잡는 점은 J1 에서 먼 쪽으로 radial_offset (Isaac 검증 2026-11-09). axis = 집게 축 반대 (이전)
//     pallet_load_<열>_<칸>_tilt      쥔 퍽을 팔레트 칸에 놓기 = pallet_place_traj.json 의 칸 궤적 (3 열 쥔 구간 0.4 rad/s, #75)
//     pallet_unload_<열>_<칸>_tilt    팔레트 칸의 퍽을 집어 들기 = 놓기 궤적 거꾸로 (빈 집게로 rotate_back · return · retract 거꾸로
//                                     → 0.3 s → 쥠 → place · carry · rotate 거꾸로 → q_front, 0.8 rad/s, Isaac pallet_unload 24/24)
//                                    pallet_* 은 sag.enabled 면 처짐 보정 스트리밍 (sag_comp.hpp, 시작 자세에서 sag.settle_time 먼저 맞춤)
//     place_<L>_<dx>_<dy>_tilt        쥔 퍽을 장소 L 의 3×3 칸 (dx, dy = m1 · 0 · p1) 에 45° 로 놓기.
//                                    칸 좌표 = mission_planner config/approach_poses.yaml cells (지도) → TF 로 팔 기준
//     place_at:<frame>:<x>:<y>:<z>   쥔 퍽을 칸 좌표 (frame 기준 x, y, 윗면 높이 z) 에 놓기 — perception place_box 칸 (#74)
// - 강화학습 집기 (policies.yaml action tcp_delta(4), 예 toppick_tilt = Isaac Waffle-TopPick-Direct): 관측 36 = toppick_obs.hpp,
//   행동 = 잡는 점 변위 → arm_diff_ik.hpp (학습과 같은 감쇠 최소제곱 IK) → /arm_controller/joint_trajectory 스트리밍 (rate_hz).
//   퍽 = 직전 target_look 값 (없으면 /perception/pucks target_index) 을 시작 때 fixed_frame 에 고정, 옆 퍽 · 모양은 같은 /perception/pucks.
//   정책 전에 학습 시작 자세 (toppick_rl.start_pose = READY_TILT) · 그리퍼 벌림으로 옮기고, toppick_rl.max_duration 안에 못 쥐면 실패.
//   그리퍼는 gripper.mode arm_joint 만 (6 번째 칸, 학습 손가락 m ↔ gripper_roll rad 선형 대응 toppick_rl.finger_m_at_*).
//   성공 = 손가락이 쥔 범위 + 잡는 점이 처음 퍽 잡는 높이보다 toppick_rl.success_lift 위로 success_hold 동안 (학습 성공과 같은 뜻)
// - 결과를 콜백 안에서 기다리지 않는다. 각 단계는 이전 단계의 result 콜백에서 이어진다
// - 켜질 때 정리 (startup.*, startup_stow.hpp): arm_controller 가 준비되고 팔 HOLD 가 풀리면 (/diagnostics opencr/motors
//   arm_hold 0 = 리셋 버튼 REARM 뒤) 한 번, 대기 자세 (STOW) 에서 startup.stages 순서 (J3 → J2 → J1 → J4) 로 startup.pose (DRIVE) 까지 천천히. 그 전에 goal 을 받으면 하지 않음
//
// callback group
// - job_cb_group_: Action 서버 · 클라이언트 · tick 타이머. 작업 상태는 이 그룹에서만 바뀐다 (락 불필요)
// - sensor_cb_group_: /perception/result · /arm_range · /joint_states 구독. sensor_mutex_로 캐시 보호

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <utility>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "action_msgs/srv/cancel_goal.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "ament_index_cpp/get_package_share_path.hpp"
#include "control/arm_ik.hpp"
#include "control/sag_comp.hpp"
#include "control/startup_stow.hpp"
#include "control/toppick_obs.hpp"
#include "control/lieupright.hpp"
#include "control/target_look_pose.hpp"
#include "robot_interfaces/action/arm_pose.hpp"
#include "robot_interfaces/action/execute_skill.hpp"
#include "robot_interfaces/action/pick.hpp"
#include "robot_interfaces/msg/perception_result.hpp"
#include "robot_interfaces/msg/puck_array.hpp"
#include "robot_interfaces/srv/set_perception_mode.hpp"
#include "robot_policy/policy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "yaml-cpp/yaml.h"

namespace control
{

class ManipulatorNode : public rclcpp::Node
{
public:
  using ArmPose = robot_interfaces::action::ArmPose;
  using Pick = robot_interfaces::action::Pick;
  using ExecuteSkill = robot_interfaces::action::ExecuteSkill;
  using SkillGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteSkill>;
  using PoseGoalHandle = rclcpp_action::ServerGoalHandle<ArmPose>;
  using PickGoalHandle = rclcpp_action::ServerGoalHandle<Pick>;
  using FollowJT = control_msgs::action::FollowJointTrajectory;
  using GripperCmd = control_msgs::action::ParallelGripperCommand;
  using PerceptionResult = robot_interfaces::msg::PerceptionResult;
  using PuckArray = robot_interfaces::msg::PuckArray;
  using Puck = robot_interfaces::msg::Puck;
  using SetPerceptionMode = robot_interfaces::srv::SetPerceptionMode;
  using Mode = SetPerceptionMode::Request;
  using Done = std::function<void (bool ok, const std::string & message)>;

  // Pick feedback.stage
  static constexpr uint8_t STAGE_PLAN = 0;
  static constexpr uint8_t STAGE_APPROACH = 1;
  static constexpr uint8_t STAGE_GRASP = 2;
  static constexpr uint8_t STAGE_LIFT = 3;

  explicit ManipulatorNode(const rclcpp::NodeOptions & options): rclcpp::Node("manipulator_node", options)
  {
    loadParameters();

    job_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sensor_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock(), tf2::durationFromSec(get_parameter("tf_cache_time").as_double()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- 구독 ---
    rclcpp::SubscriptionOptions sub_opt;
    sub_opt.callback_group = sensor_cb_group_;
    result_sub_ = create_subscription<PerceptionResult>(
      "/perception/result",
      rclcpp::QoS(1).reliable(),  // 계약: Reliable, depth 1
      [this](PerceptionResult::ConstSharedPtr msg) {onPerceptionResult(msg);}, sub_opt);

    range_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/arm_range", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Range::ConstSharedPtr msg) {onRange(msg);}, sub_opt);

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {onJointStates(msg);}, sub_opt);

    pucks_sub_ = create_subscription<PuckArray>(   // target_look (#82)
      "/perception/pucks", rclcpp::QoS(1).reliable(),   // 계약: /perception/result 와 같음
      [this](PuckArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        last_pucks_ = msg;
      }, sub_opt);

    diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(   // 켜질 때 정리: 팔 HOLD · E-stop
      "/diagnostics", rclcpp::QoS(10),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {onDiagnostics(msg);}, sub_opt);

    // --- Action 클라이언트 ---
    if (lie_enabled_) {   // 누운 직육면체: 집는 상자 윗면 측정 (perception place_box, 사전값 토픽)
      const auto latched = rclcpp::QoS(1).reliable().transient_local();
      lie_prior_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/perception/place_box/prior", latched);
      lie_box_client_ = create_client<std_srvs::srv::Trigger>("/perception/place_box/measure", rclcpp::ServicesQoS(), sensor_cb_group_);
      lie_box_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/perception/place_box", latched,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(sensor_mutex_);
          lie_box_ = msg;
          lie_box_rx_ = now();
        }, sub_opt);
      lie_status_sub_ = create_subscription<std_msgs::msg::String>("/perception/place_box/status", latched,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(sensor_mutex_);
          lie_status_ = msg;
          lie_status_rx_ = now();
        }, sub_opt);
    }
    arm_client_ = rclcpp_action::create_client<FollowJT>(this, "/arm_controller/follow_joint_trajectory", job_cb_group_);
    gripper_client_ = rclcpp_action::create_client<GripperCmd>(this, "/gripper_controller/gripper_cmd", job_cb_group_);

    // --- Action 서버 ---
    pose_server_ = rclcpp_action::create_server<ArmPose>(
      this, "/manipulator/pose",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ArmPose::Goal> goal) {
        if (poses_.count(goal->name) == 0) {
          RCLCPP_WARN(get_logger(), "pose: unknown name '%s' -> reject", goal->name.c_str());
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (startupMoving("pose")) {return rclcpp_action::GoalResponse::REJECT;}
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<PoseGoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<PoseGoalHandle> gh) {startPose(gh);},
      rcl_action_server_get_default_options(), job_cb_group_);

    pick_server_ = rclcpp_action::create_server<Pick>(
      this, "/manipulator/pick",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Pick::Goal>) {
        if (startupMoving("pick")) {return rclcpp_action::GoalResponse::REJECT;}
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<PickGoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<PickGoalHandle> gh) {startPick(gh);},
      rcl_action_server_get_default_options(), job_cb_group_);

    // 강화학습 정책 실행. 정책이 연결되지 않았거나(skill.enabled false) 모르는 정책이면 거절 → mission이 fallback
    skill_server_ = rclcpp_action::create_server<ExecuteSkill>(
      this, "/manipulator/execute_skill",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteSkill::Goal> goal) {
        if (skill_ready_.count(goal->policy) == 0 && !parseTaskSkill(goal->policy)) {
          RCLCPP_WARN(get_logger(), "execute_skill '%s': not a mission skill and policy not loaded (skill.enabled · "
            "policies.yaml · ONNX Runtime) -> reject", goal->policy.c_str());
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (startupMoving("execute_skill")) {return rclcpp_action::GoalResponse::REJECT;}
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<SkillGoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<SkillGoalHandle> gh) {startSkill(gh);},
      rcl_action_server_get_default_options(), job_cb_group_);
    jt_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", rclcpp::QoS(10));

    // cancel 처리 · 미세 보정 대기용 (20Hz)
    tick_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.05), [this]() {onTick();},
      job_cb_group_);

    RCLCPP_INFO(get_logger(), "manipulator_node ready (poses: %zu, gripper mode: %s, pallet slots: %zu, place cells: %zu)",
      poses_.size(), gripper_mode_name_.c_str(), pallet_.size(), place_cells_.size());
  }

private:
  enum class Job { NONE, POSE, PICK, SKILL, STARTUP };   // STARTUP = 켜질 때 정리 자세 (action goal 없음)
  enum class PickStep { PLAN, PREGRASP, CORRECT_WAIT, CORRECT_MOVE, OPEN, FINAL, CLOSE, CLOSE_WAIT, LIFT, TASK, TARGET_LOOK_WAIT };
  enum class GripperMode { ARM_JOINT, ACTION, NONE };
  // 미션 skill 종류 (execute_skill 이름)
  enum class Task { NONE, TARGET_LOOK, LOOK, TOPPICK, PALLET_LOAD, PALLET_UNLOAD, PLACE, LIE_UPRIGHT };
  struct TaskSkill
  {
    Task kind{Task::NONE};
    int row{0}, index{0};                 // pallet_*: 열 1~3 · 칸 0~3
    std::string place, cell;              // place_<L>_<dx>_<dy>: 장소 · "dx,dy" (-1 · 0 · 1)
    std::string frame;                    // place_at · target_look · box_look · barcode_look: 좌표 frame
    std::string look;                     // LOOK: "box_look" · "barcode_look"
    double x{0.0}, y{0.0}, z{0.0};        // place_at: 칸 중심 · 윗면 높이 / target_look: 퍽 예상 중심 / lie_upright: 상자 x · y · 요 deg
  };
  // pallet_place_traj.json 의 칸 하나: 구간 (이름 · 그리퍼 hold/open · 경유 관절각 5개)
  struct PalletSegment
  {
    std::string name;
    bool hold{true};
    std::vector<std::vector<double>> q;
  };

  // ------------------------------------------------------------------ 파라미터
  void loadParameters()
  {
    // 팔 관절: controllers.yaml의 arm_controller joints와 같아야 한다 (joint1~5 + gripper_roll)
    joint_names_ = declare_parameter<std::vector<std::string>>("arm.joint_names",
      {"joint1", "joint2", "joint3", "joint4", "joint5", "gripper_roll"});
    // 관절 한계 = URDF (opencr_hardware/urdf/robot.urdf.xacro) = 펌웨어 config.h ARM_MIN/MAX_RAD (#83). 6 번째 = 그리퍼 개폐
    lower_ = declare_parameter<std::vector<double>>("arm.lower_limits",
      {-3.14159, -2.14, -1.6, -1.92, -2.8, -0.7946});
    upper_ = declare_parameter<std::vector<double>>("arm.upper_limits",
      {3.14159, 1.6, 1.6, 2.0, 2.8, 0.9541});
    max_joint_vel_ = declare_parameter<double>("arm.max_joint_velocity", 0.5);
    min_duration_ = declare_parameter<double>("arm.min_duration", 1.0);
    joint_state_timeout_ = declare_parameter<double>("arm.joint_state_timeout", 1.0);
    requireSize("arm.joint_names", joint_names_.size(), 6);
    requireSize("arm.lower_limits", lower_.size(), 6);
    requireSize("arm.upper_limits", upper_.size(), 6);

    // 기구 치수 = 차체 URDF = 기준 모델 B안 = kinematics.py (arm_ik.hpp 머리말). 값을 바꾸면 URDF 와 같이
    ArmIKParams ik;
    ik.shoulder_height = declare_parameter<double>("ik.shoulder_height", ik.shoulder_height);
    ik.upper_arm_x = declare_parameter<double>("ik.upper_arm_x", ik.upper_arm_x);
    ik.upper_arm_z = declare_parameter<double>("ik.upper_arm_z", ik.upper_arm_z);
    ik.forearm = declare_parameter<double>("ik.forearm", ik.forearm);
    ik.wrist_to_tool = declare_parameter<double>("ik.wrist_to_tool", ik.wrist_to_tool);
    const auto offsets = declare_parameter<std::vector<double>>(
      "ik.joint_offsets", std::vector<double>(5, 0.0));
    const auto signs = declare_parameter<std::vector<double>>(
      "ik.joint_signs", std::vector<double>(5, 1.0));
    requireSize("ik.joint_offsets", offsets.size(), 5);
    requireSize("ik.joint_signs", signs.size(), 5);
    std::copy(offsets.begin(), offsets.end(), ik.joint_offsets.begin());
    std::copy(signs.begin(), signs.end(), ik.joint_signs.begin());
    ik_ = std::make_unique<ArmIK>(ik);
    ik_signs_ = ik.joint_signs;     // 강화학습 집기: 측정 → 모델 관절 (q_model = (q − offset) / sign)
    ik_offsets_ = ik.joint_offsets;
    // 앞에서부터 순서대로 시도하는 집게 pitch 후보 (rad, 위로 +, -pi/4 = 45° 아래, -pi/2 = 바로 아래)
    approach_pitches_ = declare_parameter<std::vector<double>>(
      "ik.approach_pitches", {-M_PI_4, -M_PI_2, -M_PI / 3.0});
    allow_elbow_down_ = declare_parameter<bool>("ik.allow_elbow_down", false);
    use_object_yaw_ = declare_parameter<bool>("ik.use_object_yaw", false);
    wrist_roll_default_ = declare_parameter<double>("ik.wrist_roll_default", 0.0);
    // gripper.mode 가 action · none 일 때 6 번째 칸에 보낼 값 (예전 손목 롤 가정)
    gripper_roll_default_ = declare_parameter<double>("ik.gripper_roll_default", 0.0);

    // 이름 자세 [joint1~5, 6 번째]. gripper.mode=arm_joint 면 6 번째는 쓰지 않고 지금 그리퍼 명령을 유지 (쥔 퍽을 놓치지 않게)
    //   TAG_VIEW = 팔 끝 Arducam이 태그를 보는 자세, BOX_VIEW = 상자 윗면을 내려다보는 자세 (#74)
    const auto names = declare_parameter<std::vector<std::string>>(
      "poses.names", {"STOW", "DRIVE", "ALIGN", "READY", "PLACE", "TAG_VIEW", "BOX_VIEW", "TRANSIT_TILT"});
    for (const auto & name : names) {
      auto q = declare_parameter<std::vector<double>>("poses." + name, std::vector<double>(6, 0.0));
      requireSize("poses." + name, q.size(), 6);
      poses_[name] = q;
    }

    // 그리퍼: arm_joint (실물 — 6 번째 칸 gripper_roll 이 개폐 모터) | action (gripper_controller) | none (팔만 시험)
    gripper_mode_name_ = declare_parameter<std::string>("gripper.mode", "arm_joint");
    if (gripper_mode_name_ == "arm_joint") {
      gripper_mode_ = GripperMode::ARM_JOINT;
    } else if (gripper_mode_name_ == "action") {
      gripper_mode_ = GripperMode::ACTION;
    } else if (gripper_mode_name_ == "none") {
      gripper_mode_ = GripperMode::NONE;
    } else {
      throw std::invalid_argument("gripper.mode must be arm_joint | action | none");
    }
    gripper_joint_ = declare_parameter<std::string>("gripper.joint_name", "gripper_joint");   // action 모드
    // arm_joint: rad (열림 1550 tick · 닫힘 2650 tick, docs/arm_bench_test_2026-10-27.md). action: 컨트롤러 단위
    gripper_open_ = declare_parameter<double>("gripper.open_position", -0.764);
    gripper_close_ = declare_parameter<double>("gripper.close_position", 0.923);
    gripper_effort_ = declare_parameter<double>("gripper.max_effort", 0.0);  // action 모드, 0 = 지정 안 함
    gripper_move_time_ = declare_parameter<double>("gripper.move_time", 1.0);   // arm_joint: 끝에서 끝 0.9 s (벤치)
    gripper_settle_ = declare_parameter<double>("gripper.settle_time", 0.4);    // arm_joint: 닫은 뒤 판정까지 (GripGuard 0.3 s)
    // arm_joint: 닫힘 명령보다 이만큼 넘게 덜 닫혔으면 퍽을 잡음 (퍽 30 mm 에서 약 +0.51 rad 에서 막힘, 벤치)
    grasp_margin_ = declare_parameter<double>("gripper.grasp_margin", 0.15);
    // 닫았는데 끝까지 닫힘 = 빈손 → pick 실패
    gripper_fail_on_empty_ = declare_parameter<bool>("gripper.fail_on_empty", true);

    // 미션 skill — toppick_tilt: pick 과 같은 과정, 집게 기울기 · 접근 거리만 다름
    toppick_pitch_ = declare_parameter<double>("toppick.pitch", -M_PI_4);
    toppick_approach_ = declare_parameter<double>("toppick.approach_distance", 0.06);
    // ① 정밀 정렬 결과 (/arm_range = 집게 축 위 거리) · 미세 보정을 toppick 에서도 쓸지. 45° 는 집게 축이 퍽을 보지 않고
    // 접근 위치에서 퍽이 팔 카메라 화면 밖이라 (#82) 기본 끔 → 목표 = target_look 이 잰 위치 (없으면 ② 추적 위치) 그대로
    toppick_use_alignment_ = declare_parameter<bool>("toppick.use_alignment", false);
    toppick_latch_max_age_ = declare_parameter<double>("toppick.latch_max_age", 30.0);   // target_look 값 유효 시간 (s)
    // 접근 경로 top | axis (2026-11-09 Isaac 검증: top 84/88 vs axis 48/88, manipulator.yaml 주석)
    {
      const auto mode = declare_parameter<std::string>("toppick.approach_mode", "top");
      if (mode != "top" && mode != "axis") {
        throw std::invalid_argument("toppick.approach_mode must be top or axis");
      }
      toppick_top_ = (mode == "top");
    }
    toppick_approach_height_ = declare_parameter<double>("toppick.approach_height", 0.05);
    toppick_approach_inward_ = declare_parameter<double>("toppick.approach_inward", 0.03);
    toppick_radial_offset_ = declare_parameter<double>("toppick.radial_offset", 0.002);
    // target_look 타겟체크자세 (#82, target_look_pose.hpp). 카메라 장착 = robot.urdf.xacro arm_camera_link
    target_look_.camera_x = declare_parameter<double>("target_look.camera_x", target_look_.camera_x);
    target_look_.camera_z = declare_parameter<double>("target_look.camera_z", target_look_.camera_z);
    target_look_.camera_tilt = declare_parameter<double>("target_look.camera_tilt", target_look_.camera_tilt);
    target_look_.wrist_to_tool = get_parameter("ik.wrist_to_tool").as_double();
    target_look_.view_angles.clear();
    for (double d : declare_parameter<std::vector<double>>("target_look.view_angles_deg",
        {70.0, 72.0, 68.0, 75.0, 65.0, 78.0, 80.0, 62.0, 85.0, 60.0}))
    {
      target_look_.view_angles.push_back(d * M_PI / 180.0);
    }
    target_look_.distances = declare_parameter<std::vector<double>>("target_look.distances",
      {0.17, 0.16, 0.18, 0.15, 0.19, 0.20, 0.21, 0.22});
    target_look_.min_clearance = declare_parameter<double>("target_look.min_clearance", target_look_.min_clearance);
    target_look_settle_ = declare_parameter<double>("target_look.settle_time", 0.3);
    // box_look · barcode_look (#65): 카메라 장착은 target_look 과 같고, 시선 (광축이 수평 아래로 이루는 각) · 거리 후보만 다름
    look_params_["box_look"] = lookParams("box_look", {90.0, 85.0, 80.0, 75.0, 70.0}, {0.20, 0.22, 0.18, 0.24, 0.26}, 0.04);
    look_params_["barcode_look"] = lookParams("barcode_look", {0.0, 5.0, -5.0, 10.0, -10.0, 15.0, -15.0},
        {0.25, 0.28, 0.30, 0.32, 0.35, 0.40}, -1.0);
    target_look_arm_wait_ = declare_parameter<double>("target_look.arm_wait", 1.0);
    target_look_timeout_ = declare_parameter<double>("target_look.timeout", 3.0);
    target_look_match_ = declare_parameter<double>("target_look.match_distance", 0.04);
    // 집게 끝(tool_link) = 퍽 중심 + 위로 grasp_z_offset (팔레트 궤적 grasp_dz 0.015 와 같게)
    grasp_z_offset_ = declare_parameter<double>("pick.grasp_z_offset", 0.015);
    // 팔레트 칸 궤적 (빈 문자열 = <control share>/config/pallet_place_traj.json, pallet_traj.py 가 만든 파일 사본)
    pallet_file_ = declare_parameter<std::string>("pallet.traj_file", "");
    pallet_speed_ = declare_parameter<double>("pallet.speed", 0.8);              // rad/s (#75)
    pallet_row3_hold_speed_ = declare_parameter<double>("pallet.row3_hold_speed", 0.4);   // 3 열 쥔 구간 (#75)
    pallet_require_held_ = declare_parameter<bool>("pallet.require_held", true);
    // 칸 안에서 벌리는 폭: 끝까지 열면 손가락이 칸 벽에 닿음 (Isaac FINGER_RELEASED 37.6 mm). 기본 = gripper.open_position
    pallet_release_ = declare_parameter<double>("pallet.release_position", gripper_open_);
    pallet_dwell_ = declare_parameter<double>("pallet.place_dwell", 0.3);
    // 처짐 보정 (sag_comp.hpp): 팔레트 칸 궤적을 계획 + 적분 bias 로 스트리밍 (Isaac pallet_place_check --ki 4, 24/24)
    sag_enabled_ = declare_parameter<bool>("sag.enabled", true);
    sag_ki_ = declare_parameter<double>("sag.ki", 4.0);
    sag_bias_max_ = declare_parameter<double>("sag.bias_max", 0.15);
    sag_settle_ = declare_parameter<double>("sag.settle_time", 1.5);
    sag_rate_ = declare_parameter<double>("sag.rate_hz", 50.0);
    sag_horizon_ = declare_parameter<double>("sag.horizon", 0.06);
    sag_max_error_ = declare_parameter<double>("sag.max_error", 0.3);   // 놓는 자세에서 벌리기 · 쥐기 전 머묾 (처짐 가라앉음)
    // place_<L>_*: 칸 좌표표 (빈 문자열 = <mission_planner share>/config/approach_poses.yaml), 칸 좌표 frame
    place_cells_file_ = declare_parameter<std::string>("place.cells_file", "");
    place_cells_frame_ = declare_parameter<std::string>("place.cells_frame", "map");
    place_pitch_ = declare_parameter<double>("place.pitch", -M_PI_4);
    place_surface_height_ = declare_parameter<double>("place.surface_height", 0.070);   // 장소 상자 윗면 (발제 Z 70 mm)
    puck_half_height_ = declare_parameter<double>("place.puck_half_height", 0.030);
    place_clearance_ = declare_parameter<double>("place.clearance", 0.002);
    place_approach_ = declare_parameter<double>("place.approach_distance", 0.04);
    place_retreat_ = declare_parameter<double>("place.retreat_distance", 0.04);
    // 2026-11-11 Isaac 주사위 연속 놓기 (dice_place_check.py, 2048 대): 끝까지 벌리면 (81.6 mm) 먼저 놓은 옆 퍽을 밀고,
    //   퍽 쥔 팔 처짐으로 J1 쪽으로 3 ~ 10 mm 덜 감 -> 놓을 때 벌림 폭 · J1 바깥 보정 (판 전체 33.6 % -> 90.0 %)
    place_release_ = declare_parameter<double>("place.release_position", gripper_open_);   // rad, 기본 = 끝까지 (이전 동작)
    place_radial_a_ = declare_parameter<double>("place.radial_offset_mm", 0.0);              // J1 거리 0.2665 m 칸에서 바깥으로 [mm]
    place_radial_b_ = declare_parameter<double>("place.radial_offset_slope_mm", 0.0);        // 칸 (0.06 m) 멀어질 때마다 더할 값 [mm]
    place_sag_ = declare_parameter<bool>("place.sag", true);   // 2026-11-11 놓기도 처짐 보정 스트리밍 (sag.enabled 일 때)
    // 바닥 frame (z = 바닥). arm_base_link 높이 = 이 frame → arm_base_link TF, 없으면 arm_base_height
    floor_frame_ = declare_parameter<std::string>("floor_frame", "base_footprint");
    arm_base_height_ = declare_parameter<double>("arm_base_height", 0.120);   // ★ base_link 바닥 위 0.00675 + 0.1135 (B안)

    base_frame_ = declare_parameter<std::string>("arm_base_frame", "arm_base_link");
    // 측정 시각 이후 차체 · 팔이 움직였을 수 있으므로 측정 시각 기준으로 fixed_frame을 거쳐
    // 현재 arm_base_link로 변환한다 (300ms 안이라 차이는 작다)
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    // /arm_range 기준 프레임 (perception arm.range.frame 과 같게). x = 팔 접근 방향
    range_frame_ = declare_parameter<std::string>("range_frame", "tool_link");
    declare_parameter<double>("tf_cache_time", 120.0);

    // 접근 위치 = 파지점(물체 중심) 앞 approach_distance
    approach_distance_ = declare_parameter<double>("pick.approach_distance", 0.15);
    // /arm_range → 파지점까지 거리 환산: 파지점 거리 = range + range_offset
    //   /arm_range 는 끝단(tool_link)에서 퍽 표면까지라 range_offset = 퍽 반지름 (perception puck.half_width 와 같게)
    range_offset_ = declare_parameter<double>("pick.range_offset", 0.015);
    // 측정 기반 파지점 거리와 계획(SEARCH object_pose) 거리 차이 허용값.
    // 넘으면 다른 퍽(이웃)을 잰 것으로 보고 /arm_range 값을 쓰지 않는다
    range_mismatch_tol_ = declare_parameter<double>("pick.range_mismatch_tolerance", 0.04);
    max_final_approach_ = declare_parameter<double>("pick.max_final_approach", 0.25);
    lift_height_ = declare_parameter<double>("pick.lift_height", 0.05);
    // 최종 접근 · 들어올리기 직선 경로: 경유점 간격 (m) · 끝단 속도 (m/s) · 경유점 사이 관절 변화 상한 (rad)
    cartesian_step_ = declare_parameter<double>("pick.cartesian_step", 0.01);
    cartesian_speed_ = declare_parameter<double>("pick.cartesian_speed", 0.05);
    max_waypoint_joint_step_ = declare_parameter<double>("pick.max_waypoint_joint_step", 0.2);
    if (cartesian_step_ <= 0.0 || cartesian_speed_ <= 0.0) {
      throw std::invalid_argument("pick.cartesian_step / pick.cartesian_speed must be > 0");
    }
    // 300ms 규칙 (계약): header.stamp 기준 이보다 오래된 인식 결과 · 거리는 버린다
    stale_timeout_ = declare_parameter<double>("stale_timeout", 0.3);

    // pick 목표 ① 정밀 정렬 결과: |arm_offset| <= offset_tolerance이고 /arm_range가 유효하면
    //   팔 끝 축 위 (range + object_radius) 지점을 파지점으로 쓴다
    // TODO(Perception 협의): arm_offset 단위 확정 후 offset_tolerance 조정
    // TODO(Controller): 대상 물체 크기 확정 후 object_radius 조정
    offset_tolerance_ = declare_parameter<double>("pick.offset_tolerance", 0.01);
    object_radius_ = declare_parameter<double>("pick.object_radius", 0.015);
    // pick 목표 ② 물체 위치: 300ms 안의 result에서 object_found = true인 object_pose
    //   (READY 팔이 D435를 가리면 ②는 없을 수 있다 → ① 정밀 정렬 결과가 기본)

    correction_enabled_ = declare_parameter<bool>("correction.enabled", true);
    correction_max_iter_ = declare_parameter<int>("correction.max_iterations", 5);
    // TODO(Perception/Controller): arm_offset 단위 · 축 (추정) — header.frame_id 축 기준,
    //   "카메라 축 → 물체" 벡터로 가정. gain = arm_offset 1단위당 이동 거리(m)
    //   음수면 보정 방향 반전 (부호 규약이 반대일 때), 0이면 좌우 보정 끔
    offset_gain_ = declare_parameter<double>("correction.offset_gain", 1.0);
    lateral_tol_ = declare_parameter<double>("correction.lateral_tolerance", 0.005);
    range_tol_ = declare_parameter<double>("correction.range_tolerance", 0.01);
    // 보정 중 /arm_range 목표값 = 접근 위치에서 나와야 할 측정값 (별도 파라미터로 두면 어긋나므로 계산)
    target_range_ = approach_distance_ - range_offset_;
    if (target_range_ <= 0.0) {
      throw std::invalid_argument("pick.approach_distance - pick.range_offset must be > 0");
    }
    max_step_ = declare_parameter<double>("correction.max_step", 0.02);
    correction_min_duration_ = declare_parameter<double>("correction.min_duration", 0.3);
    settle_time_ = declare_parameter<double>("correction.settle_time", 0.2);
    wait_timeout_ = declare_parameter<double>("correction.wait_timeout", 1.0);
    // true: 보정 실패(arm_offset 없음 · 반복 횟수 초과)면 abort, 재시도는 mission이 판단
    // false: 경고만 남기고 최종 접근 진행
    correction_required_ = declare_parameter<bool>("correction.required", true);

    // 강화학습 정책 (robot_policy 연결 전까지 false → execute_skill은 거절, mission이 fallback)
    skill_enabled_ = declare_parameter<bool>("skill.enabled", false);
    skill_names_ = declare_parameter<std::vector<std::string>>(
      "skill.policies", std::vector<std::string>{});
    // 2026-11-11 미션 skill "toppick_tilt" 를 이 이름의 정책으로 돌림 (예 sidepick45_k7). 빈 값 · 로드 안 됨 = 이름 그대로
    //   (policies.yaml toppick_tilt 정책, 없으면 고정 동작). mission 의 toppick_tilt 실패 처리 (target_look 부터 다시) 는 그대로
    toppick_tilt_policy_ = declare_parameter<std::string>("skill.toppick_tilt_policy", "");
    // 빈 문자열 = <robot_policy share>/config/policies.yaml (학습 담당이 ONNX와 함께 제공, drive_node와 같은 파일)
    skill_file_ = declare_parameter<std::string>("skill.policies_file", "");
    // 스트리밍 1점 궤적의 time_from_start = horizon / rate_hz (JTC 떨림 방지, 실물 테스트 후 조정)
    skill_horizon_ = declare_parameter<double>("skill.horizon", 1.5);
    // 정책 목표의 한 tick 변화량 ≤ max_joint_velocity / rate_hz (arm.max_joint_velocity 와 별개, 학습 환경 한계와 맞출 것)
    skill_max_joint_vel_ = declare_parameter<double>("skill.max_joint_velocity", 0.5);

    // 강화학습 집기 (action tcp_delta(4), toppick_obs.hpp · arm_diff_ik.hpp). 값은 Isaac toppick_env_cfg 와 같게
    rl_frame_ = declare_parameter<std::string>("toppick_rl.base_frame", "base_link");   // 학습 관측 좌표 (arm_diff_ik 기구 기준)
    rl_grip_.rad_open = gripper_open_;
    rl_grip_.rad_close = gripper_close_;
    rl_grip_.m_open = declare_parameter<double>("toppick_rl.finger_m_at_open", 0.020);    // ★ 실측 전 가정 (URDF 손가락 한계)
    rl_grip_.m_close = declare_parameter<double>("toppick_rl.finger_m_at_close", -0.011);
    rl_open_m_ = declare_parameter<double>("toppick_rl.open_m", 0.010);        // 행동 > 0 (gripper_open, 61.6 mm)
    rl_closed_m_ = declare_parameter<double>("toppick_rl.closed_m", -0.0075);  // 행동 ≤ 0 (gripper_closed)
    rl_grasp_lo_ = declare_parameter<double>("toppick_rl.grasp_finger_min", -0.010);   // 쥔 손가락 범위 (FINGER_GRASP_RANGE)
    rl_grasp_hi_ = declare_parameter<double>("toppick_rl.grasp_finger_max", -0.003);
    rl_grasp_dz_ = declare_parameter<double>("toppick_rl.grasp_dz", 0.015);           // 잡는 점 = 퍽 중심 + 이 높이
    rl_success_lift_ = declare_parameter<double>("toppick_rl.success_lift", 0.03);
    rl_success_hold_ = declare_parameter<double>("toppick_rl.success_hold", 1.0);
    rl_nb_match_ = declare_parameter<double>("toppick_rl.neighbor_distance", 0.12);   // 같은 상자 옆 퍽 (퍽 간격 0.08)
    rl_self_match_ = declare_parameter<double>("toppick_rl.target_match", 0.03);      // 목표와 같은 퍽으로 볼 거리 (모양 찾기)
    rl_default_box_ = declare_parameter<bool>("toppick_rl.default_is_box", false);    // 모양을 모를 때
    rl_max_joint_vel_ = declare_parameter<double>("toppick_rl.max_joint_velocity", 2.0);   // 안전 상한 (학습은 앞섬 ±0.25 rad 만)
    // 학습 에피소드 시작 자세 = READY_TILT (toppick_env_cfg, 집게 45°, 모델 관절값) + 그리퍼 벌림. 정책은 여기서 시작한 것만 배웠으므로
    //   직전 자세 (target_look 등) 에서 바로 돌리지 않고 먼저 이 자세로 옮긴다 (#69). 빈 배열 = 옮기지 않음
    rl_start_pose_ = declare_parameter<std::vector<double>>("toppick_rl.start_pose", {0.0, -0.462, 0.771, 0.476, 0.0});
    // 정책을 돌리는 시간 상한 (학습 에피소드 6 s). 넘으면 실패 → mission 이 target_look 부터 다시 (goal timeout 보다 먼저 끝냄)
    rl_max_duration_ = declare_parameter<double>("toppick_rl.max_duration", 12.0);

    // 누운 직육면체 세워 놓기 (lie_upright, lieupright.hpp). 값은 Isaac lieupright_env_cfg (WaffleLieUprightB7 = r2g2) 와 같게
    lie_enabled_ = declare_parameter<bool>("lie_upright.enabled", true);
    lie_policy_name_ = declare_parameter<std::string>("lie_upright.policy", "lie_upright");
    lie_file_ = declare_parameter<std::string>("lie_upright.policies_file", "");   // 빈 값 = robot_policy share config/policies.yaml
    lie_floor_frame_ = declare_parameter<std::string>("lie_upright.floor_frame", "base_footprint");
    lie_box_top_ = declare_parameter<double>("lie_upright.box_top", 0.07);         // 상자 윗면 (바닥 위, 발제 70 mm)
    lie_puck_dz_ = declare_parameter<double>("lie_upright.puck_dz", 0.015);        // 관측 누운 퍽 중심 = 윗면 + 15 mm
    // 세울 자리 D 높이 = 윗면 + 29 mm. 학습 _stand_spot 은 30 + 2 mm 를 생성 높이 (base_link 9.75 mm) 로 뺐는데 차체는 6.75 mm 로
    //   내려앉아, 실제 base_link 기준 D 는 윗면 + 29 mm 로 학습됨 (RL 261111 1 확인, 11-11). 학습과 같은 입력을 주려고 29 mm
    lie_spot_dz_ = declare_parameter<double>("lie_upright.spot_dz", 0.029);
    // 상자 보기 자세 (모델 값): 집는 상자 윗면 + 여유 15 mm 가 편 영상 (fx 602 · fy 793) 안, 모의 팔-몸체 접촉 0 N · 자기 충돌 근사 65 mm
    //   (11-11 Isaac B안. 옛 [∓0.30, −1.80, 0.50, 1.90, 0] 은 몸체에 닿음 7.9 N). place_box 모의 렌더 120 장 중심 0.71 mm · 요 0.14° 안
    lie_view_left_ = declare_parameter<std::vector<double>>("lie_upright.view_left", {-0.300, -1.25, 0.15, 1.85, 0.0});
    lie_view_right_ = declare_parameter<std::vector<double>>("lie_upright.view_right", {0.300, -1.25, 0.15, 1.85, 0.0});
    lie_view_inline_ = declare_parameter<std::vector<double>>("lie_upright.view_inline", {0.0, -1.25, 0.15, 1.85, 0.0});
    lie_side_y_ = declare_parameter<double>("lie_upright.side_y", 0.05);           // 사전 상자 중심 |y| 가 이보다 크면 회전 접근
    lie_box_settle_ = declare_parameter<double>("lie_upright.box_settle", 0.5);
    lie_box_timeout_ = declare_parameter<double>("lie_upright.box_timeout", 3.0);
    lie_box_required_ = declare_parameter<bool>("lie_upright.box_required", true);
    lie_pucks_age_ = declare_parameter<double>("lie_upright.pucks_max_age", 2.0);   // 긴 축 · 옆 퍽을 읽을 /perception/pucks 나이 (s)
    lie_max_duration_ = declare_parameter<double>("lie_upright.max_duration", 15.0);   // 학습 에피소드 12 s
    lie_lift_ok_ = declare_parameter<double>("lie_upright.lift_ok", 0.03);
    lie_clear_dist_ = declare_parameter<double>("lie_upright.clear_distance", 0.03);
    lie_clear_hold_ = declare_parameter<double>("lie_upright.clear_hold", 1.0);
    // 팔 관절 목표 변화 한계 (rad/s) · 방식. Isaac r2e 평가 (접촉 없는 성공, 512 × 3 seed, 제한 없음 96.4 %):
    //   clip (관절마다 자름) 4.0 95.8 % · 3.0 79.4 % · 2.0 4.5 %, scale (행동 전체 비율) 4.0 95.2 % · 3.0 95.4 % · 2.0 90.6 %.
    //   기본 = scale · 3.0 (11-11 결정, r2g2 는 이 값으로 학습 → 3.0 넘게 올리려면 다시 평가). 실물 관절 속도 (#201) 가 3.0 rad/s 를 못 내면 다시 정함
    lie_max_joint_vel_ = declare_parameter<double>("lie_upright.max_joint_velocity", 3.0);
    {
      const auto m = declare_parameter<std::string>("lie_upright.velocity_limit", "scale");   // clip | scale (lieApplyActionLimited)
      if (m != "clip" && m != "scale") {throw std::invalid_argument("lie_upright.velocity_limit must be clip | scale");}
      lie_vel_scale_ = m == "scale";
    }
    lie_look_after_ = declare_parameter<bool>("lie_upright.look_after", true);
    lie_release_xy_ = declare_parameter<double>("lie_upright.release_xy", 0.03);   // 놓음 = 잡는 점이 D 수평 이 안 (학습 spot_tol)
    lie_release_z_ = declare_parameter<double>("lie_upright.release_z", 0.06);     // · D 위 이 아래. 밖에서 벌리면 떨어뜨림 실패
    lie_release_beta_ = declare_parameter<double>("lie_upright.release_beta", 1.4);
    lie_release_center_offset_ = declare_parameter<double>("lie_upright.release_center_offset", 0.011);   // 잡는 점 → 퍽 중심 (집게 방향, grasp_dz_a)   // · 집게 숙임 β 이 위 (세운 퍽, 수평 1.57)
    lie_retreat_up_ = declare_parameter<double>("lie_upright.retreat_up", 0.05);   // 끝난 뒤 잡는 점을 위로 (m)
    lie_retreat_time_ = declare_parameter<double>("lie_upright.retreat_time", 1.0);   // 그 이동 최소 시간 (s)
    lie_params_.finger_open = declare_parameter<double>("lie_upright.finger_open", lie_params_.finger_open);
    lie_params_.finger_close = declare_parameter<double>("lie_upright.finger_close", lie_params_.finger_close);
    lie_params_.grasp_lo = declare_parameter<double>("lie_upright.grasp_finger_min", lie_params_.grasp_lo);
    lie_params_.grasp_hi = declare_parameter<double>("lie_upright.grasp_finger_max", lie_params_.grasp_hi);
    for (const auto * v : {&lie_view_left_, &lie_view_right_, &lie_view_inline_}) {
      if (v->size() != 5) {throw std::invalid_argument("lie_upright.view_* must be [joint1..joint5] (model values)");}
    }

    validateParameters(signs);
    loadPolicies();
    loadLiePolicy();
    loadPallet();
    loadPlaceCells();
  }

  // pallet_place_traj.json (JSON = YAML 이라 yaml-cpp 로 읽음). 없으면 pallet_* skill 이 실패를 돌려준다
  void loadPallet()
  {
    std::string file = pallet_file_;
    try {
      if (file.empty()) {
        file = (ament_index_cpp::get_package_share_path("control") / "config" / "pallet_place_traj.json").string();
      }
      const YAML::Node doc = YAML::LoadFile(file);
      if (doc["transit"] && doc["transit"]["q_front_tilt"]) {
        transit_tilt_ = doc["transit"]["q_front_tilt"].as<std::vector<double>>();
      }
      for (const auto & slot : doc["slots"]) {
        std::vector<PalletSegment> segs;
        for (const auto & g : slot["segments"]) {
          PalletSegment seg;
          seg.name = g["name"].as<std::string>();
          seg.hold = g["gripper"].as<std::string>() == "hold";
          seg.q = g["q"].as<std::vector<std::vector<double>>>();
          segs.push_back(seg);
        }
        const std::pair<int, int> key{slot["row"].as<int>(), slot["index"].as<int>()};
        pallet_[key] = segs;
        if (slot["q_front"]) {pallet_front_[key] = slot["q_front"].as<std::vector<double>>();}   // 꺼내기 끝 자세
      }
      RCLCPP_INFO(get_logger(), "pallet: %zu slot(s) from %s", pallet_.size(), file.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "pallet trajectories not loaded (%s): %s -> pallet_* skills will fail",
        file.c_str(), e.what());
    }
  }

  // approach_poses.yaml: <L>.cells."dx,dy".x · y (지도 = 발제 좌표). 없으면 place_<L>_* skill 이 실패를 돌려준다
  void loadPlaceCells()
  {
    std::string file = place_cells_file_;
    try {
      if (file.empty()) {
        file = (ament_index_cpp::get_package_share_path("mission_planner") / "config" / "approach_poses.yaml").string();
      }
      const YAML::Node doc = YAML::LoadFile(file);
      for (const char * L : {"L1", "L2", "L3"}) {
        if (!doc[L] || !doc[L]["cells"]) {continue;}
        for (const auto & kv : doc[L]["cells"]) {
          place_cells_[std::string(L) + "/" + kv.first.as<std::string>()] =
            {kv.second["x"].as<double>(), kv.second["y"].as<double>()};
        }
      }
      RCLCPP_INFO(get_logger(), "place: %zu cell(s) from %s", place_cells_.size(), file.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "place cells not loaded (%s): %s -> place_<L>_* skills will fail", file.c_str(), e.what());
    }
  }

  // 노드 시작 시 정책 전부 로드 (실행 중 로드 없음). 실패한 정책은 없는 것으로 → execute_skill 거절 → mission fallback
  void loadPolicies()
  {
    if (!skill_enabled_) {return;}
    std::string file = skill_file_;
    if (file.empty()) {
      file = (ament_index_cpp::get_package_share_path("robot_policy") / "config" / "policies.yaml").string();
    }
    std::vector<std::string> errors;
    policies_.load(file, errors);
    for (const auto & e : errors) {
      RCLCPP_ERROR(get_logger(), "policy load failed: %s", e.c_str());
    }
    for (const auto & name : skill_names_) {
      const auto * p = policies_.find(name);
      if (!p) {
        RCLCPP_ERROR(get_logger(), "skill '%s' not loaded from %s", name.c_str(), file.c_str());
        continue;
      }
      const auto & spec = p->spec;
      if (spec.action_kind == "tcp_delta") {   // 강화학습 집기: 관측 = toppick_obs 이름 · 크기, 모두 필수
        bool ok = spec.action_dim == 4;
        for (const auto & o : spec.observation) {
          const auto it = topPickObservationSizes().find(o.name);
          ok &= it != topPickObservationSizes().end() && it->second == o.size && o.required;
        }
        if (!ok) {
          RCLCPP_ERROR(get_logger(), "skill '%s': tcp_delta(4) needs toppick_obs observations (all required)", name.c_str());
          continue;
        }
        if (gripper_mode_ != GripperMode::ARM_JOINT) {
          RCLCPP_ERROR(get_logger(), "skill '%s': tcp_delta needs gripper.mode arm_joint", name.c_str());
          continue;
        }
        // 2026-11-11 정책마다 관측 기준 · 시작 자세 · 쥘 때 손가락 (<이름>.*, 없으면 toppick_rl.* · TopPick 기준).
        //   세운 퍽 45° k7 (sidepick45_k7) 은 READY_POSE_45 · 옆 퍽 없음 (0, 0) · 시작 READY_POSE_45 · 쥠 −0.011 (manipulator.yaml)
        TcpPolicyCfg pc;
        const auto ref = declare_parameter<std::vector<double>>(name + ".obs_ref_pose",
          std::vector<double>(kTopPickReadyPose.begin(), kTopPickReadyPose.end()));
        const auto nba = declare_parameter<std::vector<double>>(name + ".neighbor_absent", {kNbAbsentX, kNbAbsentY});
        pc.start_pose = declare_parameter<std::vector<double>>(name + ".start_pose", rl_start_pose_);
        pc.closed_m = declare_parameter<double>(name + ".closed_m", rl_closed_m_);
        if (ref.size() != 5 || nba.size() != 2 || !(pc.start_pose.empty() || pc.start_pose.size() == 5)) {
          RCLCPP_ERROR(get_logger(), "skill '%s': %s.obs_ref_pose (5) · neighbor_absent (2) · start_pose (5 or []) size", name.c_str(),
            name.c_str());
          continue;
        }
        for (size_t k = 0; k < 5; ++k) {pc.layout.ref_pose[k] = ref[k];}
        pc.layout.nb_absent_x = nba[0];
        pc.layout.nb_absent_y = nba[1];
        tcp_cfg_[name] = pc;
        skill_ready_.insert(name);
        tcp_skills_.insert(name);
        RCLCPP_INFO(get_logger(), "skill '%s' loaded (%s, %.0f Hz, tcp_delta + diff IK, obs ref [%.3f %.3f %.3f %.3f %.3f], "
          "no neighbor (%.2f, %.2f), closed %.4f m)", name.c_str(), spec.model.c_str(), spec.rate_hz, ref[0], ref[1], ref[2], ref[3],
          ref[4], nba[0], nba[1], pc.closed_m);
        continue;
      }
      if (spec.action_kind != "joint_target" || spec.action_dim != static_cast<int>(joint_names_.size())) {
        RCLCPP_ERROR(get_logger(), "skill '%s': action must be joint_target(6)", name.c_str());
        continue;
      }
      bool known = true;
      for (const auto & o : spec.observation) {
        // 이 노드가 만들 수 있는 관측 (policies.yaml 이름과 같게)
        known &= (o.name == "joint_pos" && o.size == 6) || (o.name == "arm_offset" && o.size == 3) ||
          (o.name == "arm_range" && o.size == 1);
      }
      if (!known) {
        RCLCPP_ERROR(get_logger(), "skill '%s': observation must be joint_pos(6) · arm_offset(3) · arm_range(1)",
          name.c_str());
        continue;
      }
      skill_ready_.insert(name);
      RCLCPP_INFO(get_logger(), "skill '%s' loaded (%s, %.0f Hz)", name.c_str(), spec.model.c_str(), spec.rate_hz);
    }
  }

  // 잘못된 값(0으로 나누기 · 방향 반전 등)은 노드 시작 때 거부한다
  void validateParameters(const std::vector<double> & signs)
  {
    for (size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) || lower_[i] >= upper_[i]) {
        throw std::invalid_argument(
                "arm.lower_limits[" + std::to_string(i) + "] must be < arm.upper_limits[" +
                std::to_string(i) + "]");
      }
    }
    for (size_t i = 0; i < signs.size(); ++i) {
      if (!std::isfinite(signs[i]) || std::abs(signs[i]) < 1e-9) {
        throw std::invalid_argument("ik.joint_signs[" + std::to_string(i) + "] must be non-zero");
      }
    }
    if (approach_pitches_.empty()) {
      throw std::invalid_argument("ik.approach_pitches must not be empty");
    }
    for (const auto & [name, q] : poses_) {
      for (double v : q) {
        if (!std::isfinite(v)) {
          throw std::invalid_argument("poses." + name + " has a non-finite value");
        }
      }
    }

    // 켜질 때 정리 자세 (startup_stow.hpp). 팔 HOLD 가 풀린 뒤 (/diagnostics opencr/motors arm_hold 0) 한 번, 묶음 순서로 천천히
    startup_pose_ = declare_parameter<std::string>("startup.pose", "DRIVE");
    const auto stages = declare_parameter<std::vector<std::string>>("startup.stages", {"3", "2", "1", "4", "5,6"});
    startup_vel_ = declare_parameter<double>("startup.max_joint_velocity", 0.1);
    startup_skip_tol_ = declare_parameter<double>("startup.skip_tolerance", 0.05);
    startup_diag_wait_ = declare_parameter<double>("startup.diag_wait", 5.0);
    startup_warn_after_ = declare_parameter<double>("startup.warn_after", 15.0);
    if (!startup_pose_.empty()) {
      if (poses_.count(startup_pose_) == 0) {
        throw std::invalid_argument("startup.pose '" + startup_pose_ + "' is not in poses.names (빈 문자열 = 끔)");
      }
      if (!withinLimits(poses_.at(startup_pose_))) {
        throw std::invalid_argument("startup.pose '" + startup_pose_ + "' exceeds joint limits");
      }
      startup_stages_ = parseStartupStages(stages, joint_names_.size());
    }
    requirePositive("startup.max_joint_velocity", startup_vel_);
    requirePositive("startup.skip_tolerance", startup_skip_tol_);
    requireNonNegative("startup.diag_wait", startup_diag_wait_);
    requirePositive("startup.warn_after", startup_warn_after_);

    requirePositive("arm.max_joint_velocity", max_joint_vel_);
    requirePositive("arm.min_duration", min_duration_);
    requirePositive("arm.joint_state_timeout", joint_state_timeout_);
    requirePositive("ik.shoulder_height", get_parameter("ik.shoulder_height").as_double());
    requirePositive("ik.forearm", get_parameter("ik.forearm").as_double());
    requirePositive("ik.wrist_to_tool", get_parameter("ik.wrist_to_tool").as_double());
    if (gripper_mode_ == GripperMode::ARM_JOINT) {
      for (double g : {gripper_open_, gripper_close_}) {
        if (!(g >= lower_[5] && g <= upper_[5])) {
          throw std::invalid_argument("gripper.open_position / close_position must be within arm limits[5] (gripper_roll)");
        }
      }
      if (!(gripper_close_ > gripper_open_)) {
        throw std::invalid_argument("gripper.close_position must be > open_position (닫을수록 증가)");
      }
    }
    requirePositive("gripper.move_time", gripper_move_time_);
    requireNonNegative("gripper.settle_time", gripper_settle_);
    requirePositive("gripper.grasp_margin", grasp_margin_);
    requirePositive("toppick.approach_distance", toppick_approach_);
    requirePositive("toppick.latch_max_age", toppick_latch_max_age_);
    requirePositive("toppick.approach_height", toppick_approach_height_);
    requireNonNegative("toppick.approach_inward", toppick_approach_inward_);
    if (!rl_start_pose_.empty() && rl_start_pose_.size() != 5) {
      throw std::invalid_argument("toppick_rl.start_pose must be [joint1..joint5] (model values) or empty");
    }
    requirePositive("toppick_rl.max_duration", rl_max_duration_);
    if (target_look_.view_angles.empty() || target_look_.distances.empty()) {
      throw std::invalid_argument("target_look.view_angles_deg and target_look.distances must not be empty");
    }
    for (double d : target_look_.distances) {requirePositive("target_look.distances", d);}
    requireNonNegative("target_look.settle_time", target_look_settle_);
    requireNonNegative("target_look.arm_wait", target_look_arm_wait_);
    requirePositive("target_look.timeout", target_look_timeout_);
    requirePositive("target_look.match_distance", target_look_match_);
    for (const auto & [name, lp] : look_params_) {
      if (lp.view_angles.empty() || lp.distances.empty()) {
        throw std::invalid_argument(name + ".view_angles_deg and " + name + ".distances must not be empty");
      }
      for (double d : lp.distances) {requirePositive(name + ".distances", d);}
    }
    requireNonNegative("pick.grasp_z_offset", grasp_z_offset_);
    requirePositive("pallet.speed", pallet_speed_);
    requirePositive("pallet.row3_hold_speed", pallet_row3_hold_speed_);
    requireNonNegative("pallet.place_dwell", pallet_dwell_);
    requireNonNegative("sag.ki", sag_ki_);
    requireNonNegative("sag.bias_max", sag_bias_max_);
    requireNonNegative("sag.settle_time", sag_settle_);
    requirePositive("sag.rate_hz", sag_rate_);
    requirePositive("sag.horizon", sag_horizon_);
    requirePositive("sag.max_error", sag_max_error_);
    if (gripper_mode_ == GripperMode::ARM_JOINT && !(pallet_release_ >= gripper_open_ && pallet_release_ < gripper_close_)) {
      throw std::invalid_argument("pallet.release_position must be in [gripper.open_position, gripper.close_position)");
    }
    requireNonNegative("place.surface_height", place_surface_height_);
    requirePositive("place.puck_half_height", puck_half_height_);
    requireNonNegative("place.clearance", place_clearance_);
    requirePositive("place.approach_distance", place_approach_);
    requirePositive("place.retreat_distance", place_retreat_);
    requirePositive("arm_base_height", arm_base_height_);
    requireNonNegative("gripper.max_effort", gripper_effort_);
    requirePositive("tf_cache_time", get_parameter("tf_cache_time").as_double());
    requirePositive("pick.approach_distance", approach_distance_);
    requireNonNegative("pick.range_offset", range_offset_);
    requirePositive("pick.range_mismatch_tolerance", range_mismatch_tol_);
    requirePositive("pick.max_final_approach", max_final_approach_);
    requireNonNegative("pick.lift_height", lift_height_);
    requirePositive("pick.max_waypoint_joint_step", max_waypoint_joint_step_);
    requirePositive("stale_timeout", stale_timeout_);
    requirePositive("pick.offset_tolerance", offset_tolerance_);
    requireNonNegative("pick.object_radius", object_radius_);

    requirePositive("skill.horizon", skill_horizon_);
    requirePositive("skill.max_joint_velocity", skill_max_joint_vel_);
    if (correction_max_iter_ < 0) {
      throw std::invalid_argument("correction.max_iterations must be >= 0");
    }
    if (!std::isfinite(offset_gain_)) {
      throw std::invalid_argument("correction.offset_gain must be finite");
    }
    requirePositive("correction.lateral_tolerance", lateral_tol_);
    requirePositive("correction.range_tolerance", range_tol_);
    requirePositive("correction.max_step", max_step_);
    requirePositive("correction.min_duration", correction_min_duration_);
    requireNonNegative("correction.settle_time", settle_time_);
    requirePositive("correction.wait_timeout", wait_timeout_);
  }

  static void requirePositive(const std::string & name, double v)
  {
    if (!std::isfinite(v) || v <= 0.0) {
      throw std::invalid_argument(name + " must be > 0 (got " + std::to_string(v) + ")");
    }
  }

  static void requireNonNegative(const std::string & name, double v)
  {
    if (!std::isfinite(v) || v < 0.0) {
      throw std::invalid_argument(name + " must be >= 0 (got " + std::to_string(v) + ")");
    }
  }

  void requireSize(const std::string & name, size_t actual, size_t expected)
  {
    if (actual != expected) {
      throw std::invalid_argument(
              name + ": expected " + std::to_string(expected) + " values, got " +
              std::to_string(actual));
    }
  }

  // ------------------------------------------------------------------ 구독 콜백
  void onPerceptionResult(PerceptionResult::ConstSharedPtr msg)
  {
    // 최신 결과 하나만 둔다. 물체 위치는 perception 추적이 맡으므로 따로 기억하지 않는다
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    last_result_ = msg;
  }

  void onRange(sensor_msgs::msg::Range::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    last_range_ = msg;
  }

  void onJointStates(sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    const size_t n = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < n; ++i) {
      joint_pos_[msg->name[i]] = msg->position[i];
    }
    const size_t nv = std::min(msg->name.size(), msg->velocity.size());
    for (size_t i = 0; i < nv; ++i) {
      joint_vel_[msg->name[i]] = msg->velocity[i];
    }
    joint_rx_ = now();
  }

  // opencr_hardware 진단 (이름 opencr/…): motors 의 arm_hold (펌웨어 팔 HOLD) · estop 잠김. 켜질 때 정리가 HOLD 가 풀린 뒤에만
  //   움직이게 쓴다 — HOLD 중 보낸 명령은 펌웨어가 무시하다가 REARM 순간 그 자리로 한 번에 튄다
  void onDiagnostics(diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    for (const auto & s : msg->status) {
      if (s.name == "opencr/motors") {
        for (const auto & kv : s.values) {
          if (kv.key == "arm_hold") {
            diag_arm_hold_ = kv.value != "0";
            diag_rx_ = now();
          }
        }
      } else if (s.name == "opencr/estop") {
        diag_estop_ = s.level != DiagnosticStatus::OK;
      }
    }
  }

  // /joint_states = IK 시작값 · 궤적 시간 계산 기준
  std::optional<std::vector<double>> currentJoints()
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (joint_rx_.nanoseconds() == 0 || (now() - joint_rx_).seconds() > joint_state_timeout_) {
      return std::nullopt;
    }
    std::vector<double> q;
    for (const auto & name : joint_names_) {
      auto it = joint_pos_.find(name);
      // NaN = 하드웨어가 활성화되지 않음 (joint_state_broadcaster 가 값 없이 냄)
      if (it == joint_pos_.end() || !std::isfinite(it->second)) {
        return std::nullopt;
      }
      q.push_back(it->second);
    }
    return q;
  }

  // ------------------------------------------------------------------ 공통
  bool withinLimits(const std::vector<double> & q) const
  {
    for (size_t i = 0; i < q.size(); ++i) {
      if (q[i] < lower_[i] || q[i] > upper_[i]) {
        return false;
      }
    }
    return true;
  }

  double trajDuration(
    const std::vector<double> & from, const std::vector<double> & to, double min_duration) const
  {
    double max_delta = 0.0;
    // 팔 관절 5개만 (6 번째 그리퍼는 쥐면 명령과 측정이 달라 시간이 늘어남. 그리퍼 이동 0.9 s 는 min_duration 안)
    for (size_t i = 0; i < std::min<size_t>(5, from.size()); ++i) {
      max_delta = std::max(max_delta, std::abs(to[i] - from[i]));
    }
    return std::max(min_duration, max_delta / max_joint_vel_);
  }

  // 위치 p, pitch → 6관절 명령값 (joint1~5 + gripper_roll). 한계 안의 해 중 seed에 가장 가까운 것
  std::optional<std::vector<double>> solveArm(
    const Eigen::Vector3d & p, double pitch, const std::vector<double> & seed) const
  {
    double roll = wrist_roll_default_;
    if (use_object_yaw_ && object_yaw_valid_) {
      // 평행 그리퍼는 180도 대칭이라 [-pi/2, pi/2]로 접는다
      // TODO(Controller): joint5 부호 · 기준은 URDF 확정 후 확인
      roll = std::remainder(object_yaw_ - std::atan2(p.y(), p.x()), M_PI);
    }

    std::optional<std::vector<double>> best;
    double best_cost = 0.0;
    for (bool elbow_up : {true, false}) {
      if (!elbow_up && !allow_elbow_down_) {
        continue;
      }
      const auto sol = ik_->solve(p, pitch, roll, elbow_up);
      if (!sol || (ik_->forward(*sol) - p).norm() > 1e-3) {
        continue;
      }
      std::vector<double> q(sol->begin(), sol->end());
      q.push_back(sixthValue(seed));
      if (!withinLimits(q)) {
        continue;
      }
      double cost = 0.0;
      for (size_t i = 0; i < q.size(); ++i) {
        cost += (q[i] - seed[i]) * (q[i] - seed[i]);
      }
      if (!best || cost < best_cost) {
        best = q;
        best_cost = cost;
      }
    }
    return best;
  }

  // 취소 요청을 받은 작업은 새 명령을 내지 않는다 (마무리는 onTick)
  bool jobCanceling() const
  {
    return (job_ == Job::POSE && pose_gh_->is_canceling()) ||
           (job_ == Job::PICK && pick_gh_->is_canceling()) ||
           (job_ == Job::SKILL && skill_gh_->is_canceling());
  }

  // ------------------------------------------------------------------ 컨트롤러 Action
  // 관절 목표 하나로 이동 (끝 속도 0)
  void sendArm(
    const std::vector<double> & q, double duration, Done done,
    std::function<void(double)> on_progress = nullptr)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = q;
    point.velocities.assign(q.size(), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(duration);
    sendTrajectory({point}, duration, done, on_progress);
  }

  // 여러 점 궤적 전송. duration = 마지막 점의 time_from_start (progress 계산용)
  void sendTrajectory(
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> & points, double duration,
    Done done, std::function<void(double)> on_progress = nullptr)
  {
    sag_end_plan_.reset();   // 처짐 bias 를 뺀 궤적이 나감 -> 다음 놓기 스트림은 bias 를 새로 맞춤
    if (jobCanceling()) {
      return;
    }
    if (!arm_client_->action_server_is_ready()) {
      done(false, "arm_controller action server not ready");
      return;
    }
    FollowJT::Goal goal;
    goal.trajectory.joint_names = joint_names_;
    goal.trajectory.points = points;
    last_sent_ = points.back().positions;   // 그리퍼만 움직일 때 팔 기준

    const uint64_t id = job_id_;
    rclcpp_action::Client<FollowJT>::SendGoalOptions opt;
    opt.goal_response_callback =
      [this, id, done](rclcpp_action::ClientGoalHandle<FollowJT>::SharedPtr gh) {
        if (id != job_id_) {
          // 이미 끝난 작업의 goal → 바로 취소
          if (gh) {arm_client_->async_cancel_goal(gh);}
          return;
        }
        if (!gh) {
          done(false, "arm_controller rejected goal");
          return;
        }
        arm_goal_ = gh;
      };
    if (on_progress) {
      opt.feedback_callback =
        [this, id, duration, on_progress](
        rclcpp_action::ClientGoalHandle<FollowJT>::SharedPtr,
        const std::shared_ptr<const FollowJT::Feedback> fb) {
          if (id != job_id_) {return;}
          // TODO(Controller): arm_controller가 desired.time_from_start를 채우는지 빌드 후 확인
          //   (안 채우면 progress가 0에 머문다. 팔 동작 · mission에는 영향 없음)
          const double t = rclcpp::Duration(fb->desired.time_from_start).seconds();
          on_progress(std::clamp(t / duration, 0.0, 1.0));
        };
    }
    opt.result_callback =
      [this, id, done](const rclcpp_action::ClientGoalHandle<FollowJT>::WrappedResult & wr) {
        if (id != job_id_) {return;}
        arm_goal_.reset();
        const bool ok = wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result &&
          wr.result->error_code == FollowJT::Result::SUCCESSFUL;
        done(ok, ok ? "" : "arm trajectory failed: " +
          (wr.result ? wr.result->error_string : std::string("no result")));
      };
    arm_client_->async_send_goal(goal, opt);
  }

  // 6 번째 칸에 실을 값: arm_joint 모드면 지금 그리퍼 명령 (아직 없으면 측정값), 아니면 ik.gripper_roll_default
  double sixthValue(const std::vector<double> & current) const
  {
    if (gripper_mode_ == GripperMode::ARM_JOINT) {
      return gripper_cmd_ ? *gripper_cmd_ : current[5];
    }
    return gripper_roll_default_;
  }

  // 그리퍼 여닫기. arm_joint: 팔은 마지막 명령 자리 그대로 두고 6 번째 값만 바꾼 궤적. action: gripper_cmd Action
  void moveGripper(double position, Done done)
  {
    if (gripper_mode_ == GripperMode::NONE) {
      done(true, "");
      return;
    }
    if (gripper_mode_ == GripperMode::ACTION) {
      sendGripper(position, done);
      return;
    }
    const auto current = currentJoints();
    if (!current) {
      done(false, "no recent /joint_states for arm joints");
      return;
    }
    std::vector<double> q = last_sent_.size() == joint_names_.size() ? last_sent_ : *current;
    gripper_cmd_ = position;
    q[5] = position;
    sendArm(q, gripper_move_time_, done);
  }

  // arm_joint 모드: 그리퍼 측정값이 닫힘 명령보다 grasp_margin 넘게 덜 닫혔으면 손가락이 퍽에 막힘 = 쥐고 있음
  std::optional<bool> gripperHolding()
  {
    const auto current = currentJoints();
    if (!current) {return std::nullopt;}
    return (*current)[5] < gripper_close_ - grasp_margin_;
  }

  void sendGripper(double position, Done done)
  {
    if (jobCanceling()) {
      return;
    }
    if (!gripper_client_->action_server_is_ready()) {
      done(false, "gripper_controller action server not ready");
      return;
    }
    GripperCmd::Goal goal;
    goal.command.name = {gripper_joint_};
    goal.command.position = {position};
    if (gripper_effort_ > 0.0) {
      goal.command.effort = {gripper_effort_};
    }

    const uint64_t id = job_id_;
    rclcpp_action::Client<GripperCmd>::SendGoalOptions opt;
    opt.goal_response_callback =
      [this, id, done](rclcpp_action::ClientGoalHandle<GripperCmd>::SharedPtr gh) {
        if (id != job_id_) {
          if (gh) {gripper_client_->async_cancel_goal(gh);}
          return;
        }
        if (!gh) {
          done(false, "gripper_controller rejected goal");
          return;
        }
        gripper_goal_ = gh;
      };
    opt.result_callback =
      [this, id, done](const rclcpp_action::ClientGoalHandle<GripperCmd>::WrappedResult & wr) {
        if (id != job_id_) {return;}
        gripper_goal_.reset();
        // stalled = 물체를 잡고 멈춤 (grasped 판정에 사용), reached_goal = 목표 위치까지 감
        gripper_stalled_ = wr.result && wr.result->stalled;
        const bool ok = wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result &&
          (wr.result->reached_goal || wr.result->stalled);
        done(ok, ok ? "" : "gripper command failed");
      };
    gripper_client_->async_send_goal(goal, opt);
  }

  void cancelControllers()
  {
    cancelGoal<FollowJT>(arm_client_, arm_goal_, "arm_controller");
    cancelGoal<GripperCmd>(gripper_client_, gripper_goal_, "gripper_controller");
  }

  // 취소 요청 + 응답 확인 (거부되면 ERROR 로그). 이미 끝난 goal(UNKNOWN_GOAL_ID · GOAL_TERMINATED)은 정상
  template<typename ActionT>
  void cancelGoal(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr & gh, const std::string & label)
  {
    if (!gh) {
      return;
    }
    client->async_cancel_goal(
      gh, [this, label](typename rclcpp_action::Client<ActionT>::CancelResponse::SharedPtr res) {
        using CancelGoal = action_msgs::srv::CancelGoal::Response;
        if (!res || res->return_code == CancelGoal::ERROR_REJECTED) {
          RCLCPP_ERROR(get_logger(), "%s rejected cancel request", label.c_str());
        }
      });
    gh.reset();
  }

  // 작업 종료: 진행 중인 컨트롤러 goal 취소, job_id_ 증가로 늦게 오는 콜백 무시
  void endJob()
  {
    cancelControllers();
    if (stream_active_) {holdAtMeasured();}   // 스트리밍이 실패 · 취소로 끊김 (성공은 이미 내림)
    job_ = Job::NONE;
    task_ = Task::NONE;
    gripper_wait_ = nullptr;
    pose_gh_.reset();
    pick_gh_.reset();
    skill_gh_.reset();
    rl_.reset();
    lie_.reset();
    // 타이머 콜백 안에서 끝날 수 있으므로 파괴하지 않고 멈추기만 한다 (다음 startSkill에서 교체)
    if (skill_timer_) {skill_timer_->cancel();}
    ++job_id_;
  }

  // 토픽 스트리밍 (팔레트 처짐 보정 · 정책) 이 끊길 때: arm_controller 의 마지막 점은 계획대로 앞서 있어서, E-stop · HOLD 뒤
  //   REARM 순간 펌웨어가 그 점으로 한 번에 튈 수 있다 (#144 · #107). 지금 측정 자세 한 점으로 다시 잡는다 (그리퍼는 지금 명령 유지)
  void holdAtMeasured()
  {
    stream_active_ = false;
    const auto cur = currentJoints();
    if (!cur) {return;}
    std::vector<double> q = *cur;
    if (gripper_mode_ == GripperMode::ARM_JOINT && gripper_cmd_) {q[5] = *gripper_cmd_;}
    for (size_t i = 0; i < q.size(); ++i) {q[i] = std::clamp(q[i], lower_[i], upper_[i]);}
    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = q;
    point.time_from_start = rclcpp::Duration::from_seconds(0.1);
    jt.points.push_back(point);
    jt_pub_->publish(jt);
    last_sent_ = q;
    RCLCPP_INFO(get_logger(), "stream ended early -> hold at measured joints");
  }

  // 새 goal이 오면 이전 작업은 abort (팔 명령 주체는 하나, 작업도 한 번에 하나)
  void preemptCurrent()
  {
    startup_done_ = true;   // goal 을 받음 = 운용자가 팔을 쓰는 중 → 켜질 때 정리는 하지 않는다 (정리 중 goal 은 거절됨)
    if (job_ == Job::POSE && pose_gh_->is_active()) {
      RCLCPP_WARN(get_logger(), "pose '%s' preempted by new goal", pose_name_.c_str());
      auto result = std::make_shared<ArmPose::Result>();
      result->success = false;
      pose_gh_->abort(result);
    } else if (job_ == Job::PICK && pick_gh_->is_active()) {
      RCLCPP_WARN(get_logger(), "pick preempted by new goal");
      auto result = std::make_shared<Pick::Result>();
      result->success = false;
      result->message = "preempted by new goal";
      pick_gh_->abort(result);
    } else if (job_ == Job::SKILL && skill_gh_->is_active()) {
      RCLCPP_WARN(get_logger(), "skill preempted by new goal");
      auto result = std::make_shared<ExecuteSkill::Result>();
      result->success = false;
      result->message = "preempted by new goal";
      skill_gh_->abort(result);
    }
    if (job_ != Job::NONE) {
      endJob();
    }
  }

  void onTick()
  {
    if (job_ == Job::POSE && pose_gh_->is_canceling()) {
      RCLCPP_INFO(get_logger(), "pose '%s' canceled", pose_name_.c_str());
      auto result = std::make_shared<ArmPose::Result>();
      result->success = false;
      pose_gh_->canceled(result);
      endJob();
    } else if (job_ == Job::PICK && pick_gh_->is_canceling()) {
      RCLCPP_INFO(get_logger(), "pick canceled");
      auto result = std::make_shared<Pick::Result>();
      result->success = false;
      result->message = "canceled";
      pick_gh_->canceled(result);
      endJob();
    } else if (job_ == Job::SKILL && skill_gh_->is_canceling()) {
      RCLCPP_INFO(get_logger(), "skill canceled");
      auto result = std::make_shared<ExecuteSkill::Result>();
      result->success = false;
      result->message = "canceled";
      skill_gh_->canceled(result);
      endJob();
    } else if (pickActive() && step_ == PickStep::CORRECT_WAIT) {
      correctionTick();
    } else if (job_ == Job::SKILL && task_ == Task::TARGET_LOOK && step_ == PickStep::TARGET_LOOK_WAIT) {
      targetLookTick();
    } else if (job_ == Job::SKILL && task_ == Task::LIE_UPRIGHT && lie_ && lie_->step == LieStep::BOX_WAIT) {
      lieBoxTick();
    } else if (gripper_wait_ && (now() - wait_since_).seconds() >= gripper_settle_) {
      auto next = std::move(gripper_wait_);   // 닫은 뒤 GripGuard 가 막힘을 잡을 시간을 두고 위치로 판정
      gripper_wait_ = nullptr;
      next();
    }
    startupTick();
  }

  // ------------------------------------------------------------------ 켜질 때 정리 자세 (startup.*, startup_stow.hpp)
  // 조건: arm_controller 준비 · /joint_states (NaN 아님) · 팔 HOLD 와 E-stop 해제 (opencr 진단). 진단이 startup.diag_wait 동안
  //   한 번도 안 오면 (시뮬레이션 · mock) HOLD 확인 없이 진행. 정리 전에 pose · pick · skill goal 을 받으면 운용자가 쓰는
  //   중이라 보고 정리를 하지 않는다. 정리 중에 오는 goal 은 거절, HOLD · E-stop 이 걸리면 멈추고 풀린 뒤 다시 판단
  bool startupMoving(const char * what)
  {
    if (job_ != Job::STARTUP) {return false;}
    RCLCPP_WARN(get_logger(), "%s: arm is moving to startup pose '%s' -> reject (retry after it finishes)",
      what, startup_pose_.c_str());
    return true;
  }

  // 펌웨어 팔 HOLD 또는 E-stop 잠김. 진단이 오다가 끊겨도 (3 s) 모름 = 잡힘으로 본다
  bool armHeld()
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (diag_rx_.nanoseconds() == 0) {return false;}
    return diag_arm_hold_ || diag_estop_ || (now() - diag_rx_).seconds() > 3.0;
  }

  void startupTick()
  {
    if (startup_done_ || startup_pose_.empty()) {return;}
    const auto t = now();
    if (job_ == Job::STARTUP) {
      if (armHeld()) {
        RCLCPP_WARN(get_logger(), "startup: arm HOLD / E-stop -> stop, retry after release");
        endJob();
      }
      return;
    }
    if (job_ != Job::NONE || t < startup_retry_at_) {return;}
    if (startup_wait_since_.nanoseconds() == 0) {startup_wait_since_ = t;}
    const bool warn = (t - startup_wait_since_).seconds() > startup_warn_after_;

    const auto current = currentJoints();
    if (!current || !arm_client_->action_server_is_ready()) {
      startup_diag_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      if (warn) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
          "startup: waiting for arm_controller · /joint_states — if arm_controller stays unconfigured/inactive, "
          "a joint may be outside its limits or OpenCR failed to activate (opencr_arm_tool.py monitor)");
      }
      return;
    }
    bool seen = false;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      seen = diag_rx_.nanoseconds() != 0;
    }
    if (seen) {
      if (armHeld()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
          "startup: waiting for arm HOLD / E-stop release (reset button = REARM) before moving to '%s'",
          startup_pose_.c_str());
        return;
      }
    } else {
      if (startup_diag_since_.nanoseconds() == 0) {startup_diag_since_ = t;}
      if ((t - startup_diag_since_).seconds() < startup_diag_wait_) {return;}
      RCLCPP_WARN(get_logger(), "startup: no opencr diagnostics for %.1f s -> moving without HOLD check (simulation?)",
        startup_diag_wait_);
    }
    job_ = Job::STARTUP;
    startup_stage_ = 0;
    RCLCPP_INFO(get_logger(), "startup: moving arm to '%s' (%zu stages, %.2f rad/s)",
      startup_pose_.c_str(), startup_stages_.size(), startup_vel_);
    runStartupStage();
  }

  void runStartupStage()
  {
    const auto current = currentJoints();
    if (!current) {
      failStartup("no recent /joint_states for arm joints");
      return;
    }
    const auto & pose = poses_.at(startup_pose_);
    while (startup_stage_ < startup_stages_.size() &&
      !startupStageNeeded(*current, pose, startup_stages_[startup_stage_], startup_skip_tol_))
    {
      ++startup_stage_;
    }
    if (startup_stage_ >= startup_stages_.size()) {
      RCLCPP_INFO(get_logger(), "startup: arm at '%s'", startup_pose_.c_str());
      startup_done_ = true;
      endJob();
      return;
    }
    const auto & stage = startup_stages_[startup_stage_];
    const auto target = startupStageTarget(*current, pose, stage, lower_, upper_);
    if (gripper_mode_ == GripperMode::ARM_JOINT && std::count(stage.begin(), stage.end(), size_t{5})) {
      gripper_cmd_ = target[5];   // 이후 팔 궤적이 이 그리퍼 값을 이어 감
    }
    const double duration = startupStageDuration(*current, target, startup_vel_, min_duration_);
    RCLCPP_INFO(get_logger(), "startup: stage %zu/%zu (%.1f s)", startup_stage_ + 1, startup_stages_.size(), duration);
    sendArm(target, duration, [this](bool ok, const std::string & msg) {
        if (!ok) {
          failStartup(msg);
          return;
        }
        ++startup_stage_;
        runStartupStage();
      });
  }

  void failStartup(const std::string & msg)
  {
    RCLCPP_WARN(get_logger(), "startup: failed (%s) -> retry in 5 s", msg.c_str());
    endJob();
    startup_retry_at_ = now() + rclcpp::Duration::from_seconds(5.0);
  }

  // ① 정밀 정렬 결과 · 미세 보정 (집게 축 기준 측정) 을 쓰는 집기인지: /manipulator/pick 은 늘, toppick_tilt 는 설정 (#82)
  bool alignmentAllowed() const
  {
    return job_ == Job::PICK || toppick_use_alignment_;
  }

  // /manipulator/pick 또는 toppick_tilt skill 이 집는 중
  bool pickActive() const
  {
    return job_ == Job::PICK || (job_ == Job::SKILL && task_ == Task::TOPPICK);
  }

  // ------------------------------------------------------------------ /manipulator/pose
  void startPose(std::shared_ptr<PoseGoalHandle> gh)
  {
    preemptCurrent();
    job_ = Job::POSE;
    pose_gh_ = gh;
    pose_name_ = gh->get_goal()->name;

    const auto current = currentJoints();
    if (!current) {
      finishPose(false, "no recent /joint_states for arm joints");
      return;
    }
    auto target = poses_.at(pose_name_);
    if (gripper_mode_ == GripperMode::ARM_JOINT) {
      target[5] = sixthValue(*current);   // 자세를 바꿔도 그리퍼는 그대로 (쥔 퍽을 놓치지 않게)
    }
    if (!withinLimits(target)) {
      finishPose(false, "pose '" + pose_name_ + "' exceeds joint limits");
      return;
    }
    RCLCPP_INFO(get_logger(), "pose '%s' start", pose_name_.c_str());
    sendArm(
      target, trajDuration(*current, target, min_duration_),
      [this](bool ok, const std::string & msg) {finishPose(ok, msg);},
      [this](double progress) {
        auto fb = std::make_shared<ArmPose::Feedback>();
        fb->progress = static_cast<float>(progress);
        pose_gh_->publish_feedback(fb);
      });
  }

  void finishPose(bool ok, const std::string & msg)
  {
    auto result = std::make_shared<ArmPose::Result>();
    result->success = ok;
    result->message = msg;
    if (ok) {
      RCLCPP_INFO(get_logger(), "pose '%s' done", pose_name_.c_str());
      pose_gh_->succeed(result);
    } else {
      RCLCPP_WARN(get_logger(), "pose '%s' failed: %s", pose_name_.c_str(), msg.c_str());
      pose_gh_->abort(result);
    }
    endJob();
  }

  // ------------------------------------------------------------------ /manipulator/pick
  // PLAN → PREGRASP → (CORRECT_WAIT ⇄ CORRECT_MOVE) → OPEN → FINAL → CLOSE → LIFT
  void startPick(std::shared_ptr<PickGoalHandle> gh)
  {
    preemptCurrent();
    job_ = Job::PICK;
    pick_gh_ = gh;
    beginPick(gh->get_goal()->target, approach_pitches_, approach_distance_);
  }

  // 집기 과정 (pick · toppick_tilt 공용). target 이 비어 있으면 ① 정밀 정렬 결과 → ② 추적 위치
  void beginPick(
    const geometry_msgs::msg::PoseStamped & target, const std::vector<double> & pitches, double approach)
  {
    // 보정 중 /arm_range 목표값 = 접근 위치에서 나와야 할 측정값
    target_range_ = std::max(0.0, approach - range_offset_);
    step_ = PickStep::PLAN;
    correction_iter_ = 0;
    final_grasp_dist_.reset();
    grasped_ = false;
    gripper_stalled_ = false;
    publishStage(STAGE_PLAN);

    Eigen::Vector3d grasp;
    std::string err;
    if (!resolveTarget(target, grasp, err)) {
      finishPick(false, err);
      return;
    }
    // 집게 끝(tool_link) = 퍽 중심 + 위로 grasp_z_offset (팔레트 궤적 TCP 와 같은 잡는 높이)
    grasp += Eigen::Vector3d::UnitZ() * grasp_z_offset_;
    const auto current = currentJoints();
    if (!current) {
      finishPick(false, "no recent /joint_states for arm joints");
      return;
    }

    // toppick_tilt 의 top 접근 (pick 은 그대로 axis): 잡는 점을 J1 에서 먼 쪽으로 radial_offset,
    // 접근 위치 = 잡는 점 위 approach_height · J1 쪽 approach_inward (Isaac 고정 동작 집기와 같은 경로)
    const bool top = (job_ == Job::SKILL && toppick_top_);
    pick_top_ = top;
    const double yaw = std::atan2(grasp.y(), grasp.x());
    const Eigen::Vector3d radial(std::cos(yaw), std::sin(yaw), 0.0);   // J1 에서 먼 쪽 (수평)
    if (top) {
      grasp += radial * toppick_radial_offset_;
    }

    // pitch 후보를 순서대로: 접근 위치 · 파지 위치 · 들어올린 위치가 모두 풀리는 첫 pitch
    std::optional<std::vector<double>> q_pre;
    for (double pitch : pitches) {
      const Eigen::Vector3d pre = top ?
        Eigen::Vector3d(grasp + Eigen::Vector3d::UnitZ() * toppick_approach_height_ - radial * toppick_approach_inward_) :
        Eigen::Vector3d(grasp - ArmIK::approachDirection(yaw, pitch) * approach);
      auto q = solveArm(pre, pitch, *current);
      if (q && solveArm(grasp, pitch, *q) &&
        solveArm(grasp + Eigen::Vector3d::UnitZ() * lift_height_, pitch, *q))
      {
        q_pre = q;
        pitch_ = pitch;
        tool_pos_ = pre;
        grasp_pos_ = grasp;
        break;
      }
    }
    if (!q_pre) {
      finishPick(false, "IK: no solution within joint limits");
      return;
    }
    RCLCPP_INFO(get_logger(), "pick: target (%.3f, %.3f, %.3f) in %s, pitch %.2f rad",
      grasp.x(), grasp.y(), grasp.z(), base_frame_.c_str(), pitch_);

    step_ = PickStep::PREGRASP;
    publishStage(STAGE_APPROACH);
    if (gripper_mode_ == GripperMode::ARM_JOINT) {
      gripper_cmd_ = gripper_open_;   // 접근하면서 연다
      (*q_pre)[5] = gripper_open_;
    }
    sendArm(*q_pre, trajDuration(*current, *q_pre, min_duration_),
      [this](bool ok, const std::string & msg) {
        if (!ok) {
          finishPick(false, msg);
        } else if (correction_enabled_ && alignmentAllowed()) {
          beginCorrectionWait();
        } else {
          openGripper();
        }
      });
  }

  // 300ms 규칙: 측정 시각이 stale_timeout 안인 최신 결과 · 거리 (없으면 nullptr)
  void freshSensors(
    PerceptionResult::ConstSharedPtr & result, sensor_msgs::msg::Range::ConstSharedPtr & range)
  {
    const rclcpp::Time stale_before = now() - rclcpp::Duration::from_seconds(stale_timeout_);
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    result = (last_result_ && measuredAfter(last_result_->header.stamp, stale_before)) ? last_result_ : nullptr;
    range = (last_range_ && measuredAfter(last_range_->header.stamp, stale_before)) ? last_range_ : nullptr;
  }

  static bool rangeValid(const sensor_msgs::msg::Range & r)
  {
    return std::isfinite(r.range) && r.range >= r.min_range && r.range <= r.max_range;
  }

  // pick 목표 (arm_base_link 기준 파지점)
  //   goal.target 지정 → 그 위치
  //   비어 있으면 ① 정밀 정렬 결과 → ② perception 추적 위치 순서
  bool resolveTarget(
    const geometry_msgs::msg::PoseStamped & goal_target, Eigen::Vector3d & out, std::string & err)
  {
    geometry_msgs::msg::PoseStamped src = goal_target;
    if (src.header.frame_id.empty()) {
      PerceptionResult::ConstSharedPtr result;
      sensor_msgs::msg::Range::ConstSharedPtr range;
      freshSensors(result, range);
      const bool offset_ok = result && (result->mode & Mode::ARM_ALIGN) && result->offset_valid &&
        std::hypot(result->arm_offset.x, result->arm_offset.y, result->arm_offset.z) <= offset_tolerance_;
      const bool range_ok = range && rangeValid(*range);
      if (alignmentAllowed() && offset_ok && range_ok) {
        // ① 팔 끝 축(x) 위 퍽 표면 + 물체 반지름 = 파지점 (range_frame 기준, /arm_range stamp 의 팔 자세)
        src.header = range->header;
        src.header.frame_id = range_frame_;
        src.pose = geometry_msgs::msg::Pose();
        src.pose.position.x = range->range + object_radius_;
        RCLCPP_INFO(get_logger(), "pick target: ① precision alignment (range %.3f m)", range->range);
      } else if (result && result->object_found) {
        // ② perception 물체 위치 (base_link 기준, 300ms 안의 result)
        src = result->object_pose;
        if (src.header.frame_id.empty()) {src.header = result->header;}
        RCLCPP_INFO(get_logger(), "pick target: ② perception object_pose");
      } else {
        err = "no pick target: precision alignment not ready (offset_valid · |arm_offset| <= " +
          std::to_string(offset_tolerance_) + " · /arm_range) and no object (fresh result with object_found)";
        return false;
      }
    }
    // object_pose 방향이 비어 있으면(quaternion 크기 0) yaw를 쓰지 않는다
    // TODO(Perception 협의): object_pose.orientation을 채울지 (물체 방향 추정). 안 채우면 use_object_yaw는 무의미
    auto & o = src.pose.orientation;
    const double qn = std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w);
    object_yaw_valid_ = std::isfinite(qn) && qn > 1e-6 && src.header.frame_id != range_frame_;
    if (object_yaw_valid_) {
      // 단위 quaternion이 아니면 getYaw 결과가 틀어지므로 정규화
      o.x /= qn; o.y /= qn; o.z /= qn; o.w /= qn;
    } else {
      o.x = o.y = o.z = 0.0;
      o.w = 1.0;  // 위치만 변환 (방향 무시)
      if (use_object_yaw_) {
        RCLCPP_WARN(get_logger(), "pick: no object orientation -> wrist roll = ik.wrist_roll_default");
      }
    }
    try {
      // 측정 시각의 src → fixed_frame → 현재 arm_base_link
      // stamp는 ROS 시계 기준 촬영 시각 (계약). TF는 측정값보다 몇 ms 늦게 들어오므로
      // 측정 시각의 TF가 아직 없으면 최신 TF로 변환한다 (측정값은 300ms 안이라 차이는 작다)
      geometry_msgs::msg::TransformStamped tf;
      try {
        tf = tf_buffer_->lookupTransform(
          base_frame_, tf2::TimePointZero, src.header.frame_id,
          tf2_ros::fromMsg(src.header.stamp), fixed_frame_, tf2::durationFromSec(0.0));
      } catch (const tf2::TransformException &) {
        tf = tf_buffer_->lookupTransform(base_frame_, src.header.frame_id, tf2::TimePointZero);
      }
      geometry_msgs::msg::PoseStamped dst;
      tf2::doTransform(src, dst, tf);
      out = {dst.pose.position.x, dst.pose.position.y, dst.pose.position.z};
      object_yaw_ = object_yaw_valid_ ? tf2::getYaw(dst.pose.orientation) : 0.0;
    } catch (const tf2::TransformException & e) {
      err = std::string("TF ") + src.header.frame_id + " -> " + base_frame_ + ": " + e.what();
      return false;
    }
    return true;
  }

  void beginCorrectionWait()
  {
    step_ = PickStep::CORRECT_WAIT;
    wait_since_ = now();
  }

  // 측정 시각 header.stamp (ROS 시계 기준 촬영 · 측정 시각, 계약)가 since 이후인지.
  // 300ms 규칙은 예외 없음 — stamp가 비어 있으면 측정 시각을 알 수 없으므로 버린다
  static bool measuredAfter(const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & since)
  {
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      return false;
    }
    return rclcpp::Time(stamp, RCL_ROS_TIME) >= since;
  }

  // 미세 보정: arm_offset(좌우 · 상하) · /arm_range(전후)로 끝단을 작은 궤적으로 반복 이동
  void correctionTick()
  {
    const rclcpp::Time t = now();
    // 팔이 멈춘 뒤 settle_time 이후에 촬영 · 측정된 데이터만 사용 (인식 지연 중 움직이던 영상 제외)
    const rclcpp::Time fresh_after = wait_since_ + rclcpp::Duration::from_seconds(settle_time_);

    PerceptionResult::ConstSharedPtr result;
    sensor_msgs::msg::Range::ConstSharedPtr range;
    // + 300ms 규칙: 측정 시각이 stale_timeout보다 오래되면 버린다
    const rclcpp::Time stale_before = t - rclcpp::Duration::from_seconds(stale_timeout_);
    const rclcpp::Time since = std::max(fresh_after, stale_before);
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      if (last_result_ && measuredAfter(last_result_->header.stamp, since)) {result = last_result_;}
      if (last_range_ && measuredAfter(last_range_->header.stamp, since)) {range = last_range_;}
    }
    const bool offset_ok = result && (result->mode & Mode::ARM_ALIGN) && result->offset_valid;
    const Eigen::Vector3d dir =
      ArmIK::approachDirection(std::atan2(tool_pos_.y(), tool_pos_.x()), pitch_);

    // /arm_range → 파지점 거리. 계획 거리와 크게 다르면(이웃 퍽을 잼) 쓰지 않는다.
    // 차이(= SEARCH 깊이 오차)는 보정으로 끝단이 움직여도 그대로 유지되므로 매번 같은 기준으로 판단
    bool range_ok = range && rangeValid(*range);
    if (range_ok) {
      const double measured = range->range + range_offset_;
      const double planned = (grasp_pos_ - tool_pos_).dot(dir);
      if (std::abs(measured - planned) > range_mismatch_tol_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "correction: range-based %.3fm vs planned %.3fm (> %.3fm) -> ignore /arm_range",
          measured, planned, range_mismatch_tol_);
        range_ok = false;
      }
    }
    // 보정을 끝내는 시점(현재 위치)의 측정 기반 파지점 거리 → 최종 접근에 사용
    final_grasp_dist_ =
      range_ok ? std::optional<double>(range->range + range_offset_) : std::nullopt;

    if (!offset_ok) {
      if ((t - wait_since_).seconds() > wait_timeout_) {
        correctionFailed("no valid arm_offset for " + std::to_string(wait_timeout_) + "s");
      }
      return;
    }

    // arm_offset (header.frame_id 축) → arm_base_link, 접근 방향 성분은 제거
    Eigen::Vector3d lateral;
    try {
      geometry_msgs::msg::Vector3Stamped v_in, v_out;
      v_in.header.frame_id = result->header.frame_id;
      v_in.vector = result->arm_offset;
      const auto tf = tf_buffer_->lookupTransform(
        base_frame_, result->header.frame_id, tf2::TimePointZero);
      tf2::doTransform(v_in, v_out, tf);
      const Eigen::Vector3d v =
        offset_gain_ * Eigen::Vector3d(v_out.vector.x, v_out.vector.y, v_out.vector.z);
      lateral = v - v.dot(dir) * dir;
    } catch (const tf2::TransformException & e) {
      finishPick(false, std::string("correction TF failed: ") + e.what());
      return;
    }
    const double axial = range_ok ? range->range - target_range_ : 0.0;

    if (lateral.norm() < lateral_tol_ && (!range_ok || std::abs(axial) < range_tol_)) {
      RCLCPP_INFO(get_logger(), "correction: aligned after %d step(s)", correction_iter_);
      openGripper();
      return;
    }
    if (correction_iter_ >= correction_max_iter_) {
      correctionFailed("not aligned after max iterations (" +
        std::to_string(correction_max_iter_) + ")");
      return;
    }

    Eigen::Vector3d delta = lateral + axial * dir;
    if (delta.norm() > max_step_) {
      delta *= max_step_ / delta.norm();
    }
    const Eigen::Vector3d next = tool_pos_ + delta;
    const auto current = currentJoints();
    if (!current) {
      finishPick(false, "no recent /joint_states for arm joints");
      return;
    }
    const auto q = solveArm(next, pitch_, *current);
    if (!q) {
      finishPick(false, "IK: correction target out of reach");
      return;
    }

    step_ = PickStep::CORRECT_MOVE;
    ++correction_iter_;
    sendArm(*q, trajDuration(*current, *q, correction_min_duration_),
      [this, next](bool ok, const std::string & msg) {
        if (!ok) {
          finishPick(false, msg);
          return;
        }
        tool_pos_ = next;
        beginCorrectionWait();
      });
  }

  // 보정 실패: correction.required면 abort (재시도는 mission), 아니면 경고 후 최종 접근 진행
  void correctionFailed(const std::string & reason)
  {
    if (correction_required_) {
      finishPick(false, "correction failed: " + reason);
      return;
    }
    RCLCPP_WARN(get_logger(), "correction: %s -> continue (correction.required: false)",
      reason.c_str());
    openGripper();
  }

  void openGripper()
  {
    step_ = PickStep::OPEN;
    if (gripper_mode_ == GripperMode::NONE ||
      (gripper_mode_ == GripperMode::ARM_JOINT && gripper_cmd_ && *gripper_cmd_ == gripper_open_))
    {
      finalApproach();   // 이미 열림 (접근하면서 열었음)
      return;
    }
    moveGripper(gripper_open_, [this](bool ok, const std::string & msg) {
        if (ok) {
          finalApproach();
        } else {
          finishPick(false, msg);
        }
      });
  }

  // 최종 전진 거리: 보정 종료 시점의 측정 기반 파지점 거리 (range + range_offset).
  // 측정값이 없거나 계획과 어긋나 버렸으면 계획한 파지점(object_pose)까지의 거리.
  // 전진 중에는 집게가 퍽을 가려 측위가 흔들리므로 다시 재지 않고 IK 직선 경로로 정해진 거리만 간다.
  // 접촉 · 파지는 그리퍼 결과(stalled)로 확인한다 (closeGripper)
  void finalApproach()
  {
    step_ = PickStep::FINAL;
    if (pick_top_) {   // top: 접근 위치 -> 잡는 점 직선 (집게 축 방향 거리 계산 없이)
      RCLCPP_INFO(get_logger(), "final approach: top -> grasp point");
      moveTool(grasp_pos_, [this]() {closeGripper();});
      return;
    }
    const Eigen::Vector3d dir =
      ArmIK::approachDirection(std::atan2(tool_pos_.y(), tool_pos_.x()), pitch_);
    double distance;
    if (final_grasp_dist_) {
      distance = *final_grasp_dist_;
      RCLCPP_INFO(get_logger(), "final approach: %.3fm (from /arm_range)", distance);
    } else {
      distance = (grasp_pos_ - tool_pos_).dot(dir);
      RCLCPP_INFO(get_logger(), "final approach: %.3fm (to planned grasp point)", distance);
    }
    if (distance < 0.0 || distance > max_final_approach_) {
      finishPick(false, "final approach distance " + std::to_string(distance) +
        "m out of range [0, " + std::to_string(max_final_approach_) + "]");
      return;
    }
    moveTool(tool_pos_ + dir * distance, [this]() {closeGripper();});
  }

  void closeGripper()
  {
    step_ = PickStep::CLOSE;
    publishStage(STAGE_GRASP);
    if (gripper_mode_ == GripperMode::NONE) {
      // 그리퍼 없이 팔만 테스트: 파지 확인 불가 → grasped = true로 보고 (mission require_grasped 통과)
      RCLCPP_WARN(get_logger(), "gripper.mode none -> grasp not verified");
      grasped_ = true;
      lift();
      return;
    }
    moveGripper(gripper_close_, [this](bool ok, const std::string & msg) {
        if (!ok) {
          finishPick(false, "grasp failed: " + msg);
          return;
        }
        if (gripper_mode_ == GripperMode::ACTION) {
          afterGrasp(gripper_stalled_);
          return;
        }
        step_ = PickStep::CLOSE_WAIT;
        waitGripper([this]() {
            const auto holding = gripperHolding();
            if (!holding) {
              finishPick(false, "no recent /joint_states for gripper");
              return;
            }
            afterGrasp(*holding);
          });
      });
  }

  // 닫은 뒤 gripper.settle_time 지나서 then (onTick)
  void waitGripper(std::function<void()> then)
  {
    wait_since_ = now();
    gripper_wait_ = std::move(then);
  }

  void afterGrasp(bool holding)
  {
    grasped_ = holding;
    if (!grasped_ && gripper_fail_on_empty_) {
      finishPick(false, "empty grasp: gripper closed fully without contact");
      return;
    }
    lift();
  }

  void lift()
  {
    step_ = PickStep::LIFT;
    publishStage(STAGE_LIFT);
    moveTool(tool_pos_ + Eigen::Vector3d::UnitZ() * lift_height_,
      [this]() {finishPick(true, "");});
  }

  // 같은 pitch로 끝단을 target까지 "직선"으로 이동 (최종 접근 · 들어올리기).
  // 관절 목표 하나만 보내면 관절 공간 보간이라 끝단 경로가 휘므로,
  // cartesian_step 간격 경유점마다 IK를 풀어 여러 점 궤적으로 보낸다.
  // 성공하면 tool_pos_ 갱신 후 on_success
  void moveTool(const Eigen::Vector3d & target, std::function<void()> on_success)
  {
    const auto current = currentJoints();
    if (!current) {
      failJob("no recent /joint_states for arm joints");
      return;
    }
    const Eigen::Vector3d start = tool_pos_;
    const double length = (target - start).norm();
    const int n = std::max(1, static_cast<int>(std::ceil(length / cartesian_step_)));
    const double segment_min_time = (length / n) / cartesian_speed_;

    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
    std::vector<double> seed = *current;
    double t = 0.0;
    for (int i = 1; i <= n; ++i) {
      const Eigen::Vector3d p = start + (target - start) * (static_cast<double>(i) / n);
      const auto q = solveArm(p, pitch_, seed);
      if (!q) {
        failJob("IK: straight path out of reach at waypoint " +
          std::to_string(i) + "/" + std::to_string(n));
        return;
      }
      double max_delta = 0.0;
      for (size_t j = 0; j < 5; ++j) {   // 팔 관절만 (6 번째 = 그리퍼, 쥐면 명령과 측정이 다름)
        max_delta = std::max(max_delta, std::abs((*q)[j] - seed[j]));
      }
      // 경유점 사이 관절이 크게 튀면 특이점 · 해 전환 → 직선 경로가 아님
      if (max_delta > max_waypoint_joint_step_) {
        failJob("IK: joint jump " + std::to_string(max_delta) +
          " rad on straight path at waypoint " + std::to_string(i) + "/" + std::to_string(n));
        return;
      }
      t += std::max(segment_min_time, max_delta / max_joint_vel_);
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = *q;
      point.time_from_start = rclcpp::Duration::from_seconds(t);
      points.push_back(point);
      seed = *q;
    }
    if (points.size() == 1) {
      // 한 점이면 끝 속도 0을 줘서 부드럽게 (여러 점은 위치만 → 경유점 사이 선형 보간)
      t = std::max(t, correction_min_duration_);
      points.front().velocities.assign(points.front().positions.size(), 0.0);
      points.front().time_from_start = rclcpp::Duration::from_seconds(t);
    }
    sendTrajectory(points, t,
      [this, target, on_success](bool ok, const std::string & msg) {
        if (!ok) {
          failJob(msg);
          return;
        }
        tool_pos_ = target;
        on_success();
      });
  }

  void publishStage(uint8_t stage)
  {
    if (job_ == Job::SKILL) {
      auto fb = std::make_shared<ExecuteSkill::Feedback>();
      fb->progress = static_cast<float>(stage) / 4.0f;
      skill_gh_->publish_feedback(fb);
      return;
    }
    auto fb = std::make_shared<Pick::Feedback>();
    fb->stage = stage;
    pick_gh_->publish_feedback(fb);
  }

  // 진행 중인 작업을 실패로 끝냄 (pick · 미션 skill 공용, moveTool · moveGripper 실패)
  void failJob(const std::string & msg)
  {
    if (job_ == Job::SKILL) {
      finishSkill(false, msg);
    } else if (job_ == Job::PICK) {
      finishPick(false, msg);
    }
  }

  void finishPick(bool ok, const std::string & msg)
  {
    if (job_ == Job::SKILL) {   // toppick_tilt
      finishSkill(ok, ok ? "grasped" : msg);
      return;
    }
    auto result = std::make_shared<Pick::Result>();
    result->success = ok;
    result->grasped = ok && grasped_;
    result->message = msg;
    if (ok) {
      RCLCPP_INFO(get_logger(), "pick done (grasped: %s)", grasped_ ? "true" : "false");
      pick_gh_->succeed(result);
    } else {
      // 재시도는 mission_node가 판단
      RCLCPP_WARN(get_logger(), "pick failed: %s", msg.c_str());
      pick_gh_->abort(result);
    }
    endJob();
  }

  // ------------------------------------------------------------------ /manipulator/execute_skill
  // 강화학습 정책을 policies.yaml의 rate_hz로 실행: 관측 → 정책 → 관절 목표 6개 → /arm_controller/joint_trajectory (1점)
  // 끝: goal.timeout 도달 — policies.yaml finish_on_timeout true면 성공, 아니면 실패 · 추론 실패 · required 관측 무효
  // TODO(Controller · 학습 담당): 팔 정책의 성공 판정 방법 미정 (지금은 finish_on_timeout만)
  void startSkill(std::shared_ptr<SkillGoalHandle> gh)
  {
    preemptCurrent();
    job_ = Job::SKILL;
    skill_gh_ = gh;
    skill_start_ = now();
    const double t = gh->get_goal()->timeout;
    skill_timeout_ = (std::isfinite(t) && t > 0.0) ? t : 30.0;
    RCLCPP_INFO(get_logger(), "skill '%s' start (timeout %.1fs)", gh->get_goal()->policy.c_str(),
      skill_timeout_);
    // 같은 이름 정책이 로드돼 있으면 정책, 아니면 미션 skill 고정 동작 (IK · 팔레트 궤적)
    const std::string pname = skillPolicyName(gh->get_goal()->policy);
    if (skill_ready_.count(pname) == 0) {
      startTaskSkill(*parseTaskSkill(gh->get_goal()->policy));   // accept 때 parseTaskSkill 로 확인함
      return;
    }
    active_policy_ = policies_.find(pname);   // accept 때 skill_ready_로 확인함
    skill_rate_ = active_policy_->spec.rate_hz;
    skill_last_cmd_.reset();   // 첫 tick 에 현재 관절각으로 초기화
    if (tcp_skills_.count(pname)) {
      tcp_active_ = tcp_cfg_.at(pname);
      if (pname != gh->get_goal()->policy) {
        RCLCPP_INFO(get_logger(), "skill '%s' -> policy '%s' (skill.toppick_tilt_policy)", gh->get_goal()->policy.c_str(), pname.c_str());
      }
      if (!startTopPickRl()) {return;}
      moveToRlStart([this]() {startSkillTimer();});   // 학습 시작 자세 · 그리퍼 벌림 뒤 정책 시작
      return;
    }
    startSkillTimer();
  }

  // 미션 skill 이름 → 돌릴 정책 이름 (skill.toppick_tilt_policy)
  std::string skillPolicyName(const std::string & skill) const
  {
    if (skill == "toppick_tilt" && !toppick_tilt_policy_.empty() && skill_ready_.count(toppick_tilt_policy_)) {
      return toppick_tilt_policy_;
    }
    return skill;
  }

  void startSkillTimer()
  {
    const uint64_t id = job_id_;
    if (rl_) {rl_->t_start = now();}
    skill_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / skill_rate_),
      [this, id]() {if (id == job_id_) {skillTick();}}, job_cb_group_);
  }

  // 강화학습 집기 시작 자세 (toppick_rl.start_pose, 모델 관절값 → 명령값) 로, 그리퍼는 학습 시작 벌림 (toppick_rl.open_m)
  void moveToRlStart(std::function<void()> then)
  {
    const auto current = currentJoints();
    if (!current) {
      finishSkill(false, "no recent /joint_states for arm joints");
      return;
    }
    const double grip = std::clamp(rl_grip_.toRad(rl_open_m_), lower_[5], upper_[5]);
    gripper_cmd_ = grip;
    const auto & start = tcp_active_.start_pose;   // <정책>.start_pose (기본 toppick_rl.start_pose)
    if (start.empty()) {
      then();
      return;
    }
    std::vector<double> q(joint_names_.size());
    for (size_t k = 0; k < 5; ++k) {q[k] = ik_signs_[k] * start[k] + ik_offsets_[k];}
    q[5] = grip;
    if (!withinLimits(q)) {
      finishSkill(false, "toppick_rl: start_pose outside joint limits");
      return;
    }
    RCLCPP_INFO(get_logger(), "toppick_rl: move to start pose [%.3f %.3f %.3f %.3f %.3f] before policy", start[0], start[1],
      start[2], start[3], start[4]);
    sendArm(q, trajDuration(*current, q, min_duration_), [this, then](bool ok, const std::string & msg) {
        if (!ok) {
          finishSkill(false, "toppick_rl start pose: " + msg);
          return;
        }
        then();
      });
  }

  void skillTick()
  {
    if (job_ != Job::SKILL || skill_gh_->is_canceling()) {return;}  // 취소 마무리는 onTick
    const double elapsed = (now() - skill_start_).seconds();
    if (elapsed > skill_timeout_) {
      if (active_policy_->spec.finish_on_timeout) {
        stream_active_ = false;   // 정상 끝: 마지막 정책 점 유지
        finishSkill(true, "finished at timeout (" + std::to_string(skill_timeout_) + "s)");
      } else {
        finishSkill(false, "timeout (" + std::to_string(skill_timeout_) + "s)");
      }
      return;
    }
    const auto q = currentJoints();
    if (!q) {
      finishSkill(false, "no recent /joint_states for arm joints");
      return;
    }
    if (armHeld()) {   // HOLD · E-stop 동안 펌웨어는 명령을 무시 → 정책을 돌리지 않음 (풀리면 측정 자세에서 이어감)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "skill: arm HOLD / E-stop -> policy paused");
      return;
    }
    if (rl_) {
      topPickRlTick(*q, elapsed);
      return;
    }
    if (lie_ && lie_->step == LieStep::RUN) {
      lieTick(*q, elapsed);
      return;
    }
    // 관측: policies.yaml 순서 · 배치 (required 무효 → abort, 선택 무효 → fill + valid 0)
    PerceptionResult::ConstSharedPtr result;
    sensor_msgs::msg::Range::ConstSharedPtr range;
    freshSensors(result, range);
    robot_policy::ObsValues values;   // 없는 항목 = 무효 (선택 항목은 fill + valid 0)
    values["joint_pos"] = std::vector<float>(q->begin(), q->end());
    if (result && (result->mode & Mode::ARM_ALIGN) && result->offset_valid) {
      values["arm_offset"] = std::vector<float>{static_cast<float>(result->arm_offset.x),
        static_cast<float>(result->arm_offset.y), static_cast<float>(result->arm_offset.z)};
    }
    if (range && rangeValid(*range)) {values["arm_range"] = std::vector<float>{range->range};}
    const auto & spec = active_policy_->spec;
    std::vector<float> obs, raw;
    std::string err;
    if (!robot_policy::ObservationBuilder(spec).build(values, obs, err)) {
      finishSkill(false, "required observation invalid: " + err);
      return;
    }
    if (!active_policy_->runner->run(obs, raw)) {finishSkill(false, "policy inference failed"); return;}
    // action_low · action_high clamp (ActionMapper) 뒤 관절 한계로 다시 clamp (계약)
    const auto mapped = robot_policy::ActionMapper(spec).map(raw);
    // 기준 = 직전 목표 (측정값 기준이면 추종이 늦을 때 목표가 계속 뒤처짐). 정책 출력이 튀어도 급동작 없음
    if (!skill_last_cmd_) {skill_last_cmd_ = *q;}
    const double max_step = skill_max_joint_vel_ / skill_rate_;
    std::vector<double> action(joint_names_.size());
    for (size_t i = 0; i < action.size(); ++i) {
      const double prev = (*skill_last_cmd_)[i];
      const double target = std::clamp(static_cast<double>(mapped[i]), lower_[i], upper_[i]);
      action[i] = std::clamp(target, prev - max_step, prev + max_step);
    }
    skill_last_cmd_ = action;
    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = action;
    point.time_from_start = rclcpp::Duration::from_seconds(skill_horizon_ / skill_rate_);
    jt.points.push_back(point);
    jt_pub_->publish(jt);
    stream_active_ = true;

    auto fb = std::make_shared<ExecuteSkill::Feedback>();
    fb->progress = static_cast<float>(std::clamp(elapsed / skill_timeout_, 0.0, 1.0));
    skill_gh_->publish_feedback(fb);
  }

  // ------------------------------------------------------------------ 강화학습 집기 (tcp_delta)
  // 시작: 퍽 목표 · 옆 퍽 · 모양을 정해 fixed_frame_ 에 고정 (학습: 인식 좌표를 시작 값으로 고정 70 %, #82)
  bool startTopPickRl()
  {
    PuckArray::ConstSharedPtr pucks;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      if (last_pucks_ && measuredAfter(last_pucks_->header.stamp, now() - rclcpp::Duration::from_seconds(stale_timeout_))) {
        pucks = last_pucks_;
      }
    }
    std::vector<std::pair<Eigen::Vector3d, uint8_t>> seen;   // fixed_frame_ 의 퍽 중심, 모양
    if (pucks) {
      try {
        const auto tf = tf_buffer_->lookupTransform(fixed_frame_, pucks->header.frame_id, tf2::TimePointZero);
        for (const auto & pk : pucks->pucks) {
          geometry_msgs::msg::Pose a;
          tf2::doTransform(pk.pose, a, tf);
          seen.emplace_back(Eigen::Vector3d(a.position.x, a.position.y, a.position.z), pk.shape);
        }
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN(get_logger(), "toppick_rl: TF %s -> %s: %s", pucks->header.frame_id.c_str(), fixed_frame_.c_str(), e.what());
      }
    }
    std::optional<Eigen::Vector3d> target;
    std::string from;
    if (target_look_latch_ && target_look_latch_->header.frame_id == fixed_frame_ &&
      (now() - target_look_latch_time_).seconds() <= toppick_latch_max_age_)
    {
      const auto & p = target_look_latch_->pose.position;
      target = Eigen::Vector3d(p.x, p.y, p.z);
      from = "target_look";
    } else if (pucks && pucks->target_index >= 0 && static_cast<size_t>(pucks->target_index) < seen.size()) {
      target = seen[pucks->target_index].first;
      from = "/perception/pucks target";
    }
    target_look_latch_.reset();   // 한 번만 씀
    if (!target) {
      finishSkill(false, "toppick_rl: no puck target (target_look or /perception/pucks)");
      return false;
    }
    TopPickRun r;
    r.puck = *target;
    r.is_box = rl_default_box_;
    double self_d = rl_self_match_, nb_d = rl_nb_match_;
    for (const auto & [p, shape] : seen) {
      const double d = std::hypot(p.x() - target->x(), p.y() - target->y());
      if (d <= self_d && shape != Puck::SHAPE_UNKNOWN) {
        self_d = d;
        r.is_box = shape == Puck::SHAPE_CUBOID;
      } else if (d > rl_self_match_ && d < nb_d) {
        nb_d = d;
        r.neighbor = p;
      }
    }
    rl_ = r;
    RCLCPP_INFO(get_logger(), "toppick_rl: puck (%.3f, %.3f, %.3f) %s from %s, %s, neighbor %s", target->x(), target->y(),
      target->z(), fixed_frame_.c_str(), from.c_str(), r.is_box ? "box" : "cylinder", r.neighbor ? "yes" : "none");
    return true;
  }

  // 한 tick: 관측 36 → 정책 → 잡는 점 변위 → 감쇠 최소제곱 IK → 6 관절 목표 (6 번째 = 그리퍼) 스트리밍
  void topPickRlTick(const std::vector<double> & q, double elapsed)
  {
    auto & r = *rl_;
    if ((now() - r.t_start).seconds() > rl_max_duration_) {
      finishSkill(false, "toppick_rl: not grasped within toppick_rl.max_duration " + std::to_string(rl_max_duration_) + " s");
      return;
    }
    Eigen::Vector3d puck_b, nb_b;
    try {
      const auto tf = tf_buffer_->lookupTransform(rl_frame_, fixed_frame_, tf2::TimePointZero);
      auto to_b = [&tf](const Eigen::Vector3d & p) {
          geometry_msgs::msg::Point a, b;
          a.x = p.x();
          a.y = p.y();
          a.z = p.z();
          tf2::doTransform(a, b, tf);
          return Eigen::Vector3d(b.x, b.y, b.z);
        };
      puck_b = to_b(r.puck);
      if (r.neighbor) {nb_b = to_b(*r.neighbor);}
    } catch (const tf2::TransformException & e) {
      finishSkill(false, std::string("toppick_rl: TF ") + fixed_frame_ + " -> " + rl_frame_ + ": " + e.what());
      return;
    }
    TopPickState s;
    std::optional<Joints5> qd_meas = Joints5{};
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      for (size_t k = 0; k < 5; ++k) {
        const auto it = joint_vel_.find(joint_names_[k]);
        if (it == joint_vel_.end()) {qd_meas.reset(); break;}
        (*qd_meas)[k] = it->second / ik_signs_[k];
      }
    }
    for (size_t k = 0; k < 5; ++k) {s.q[k] = (q[k] - ik_offsets_[k]) / ik_signs_[k];}
    const rclcpp::Time t = now();
    if (qd_meas) {
      s.qd = *qd_meas;
    } else if (r.q_prev) {   // /joint_states 에 속도가 없으면 차분
      const double dt = std::max((t - r.t_prev).seconds(), 1e-3);
      for (size_t k = 0; k < 5; ++k) {s.qd[k] = (s.q[k] - (*r.q_prev)[k]) / dt;}
    }
    r.q_prev = s.q;
    r.t_prev = t;
    s.finger_m = rl_grip_.toMeters(q[5]);
    s.puck = puck_b;
    if (r.neighbor) {s.neighbor = nb_b;}
    s.is_box = r.is_box;
    s.prev_action = r.prev_action;
    s.tilted = true;

    robot_policy::ObsValues values;
    for (auto & kv : topPickObservation(diff_ik_, s, tcp_active_.layout)) {values[kv.first] = kv.second;}
    const auto & spec = active_policy_->spec;
    std::vector<float> obs, raw;
    std::string err;
    if (!robot_policy::ObservationBuilder(spec).build(values, obs, err)) {
      finishSkill(false, "toppick_rl: observation invalid: " + err);
      return;
    }
    if (!active_policy_->runner->run(obs, raw)) {finishSkill(false, "policy inference failed"); return;}
    auto a = robot_policy::ActionMapper(spec).map(raw);
    for (auto & v : a) {v = std::clamp(v, -1.f, 1.f);}   // 학습: actions.clamp(-1, 1)
    const Joints5 tgt = diff_ik_.applyAction(r.target ? *r.target : s.q, s.q, Eigen::Vector3d(a[0], a[1], a[2]), s.tilted);
    r.target = tgt;
    r.prev_action = {a[0], a[1], a[2], a[3]};

    if (!skill_last_cmd_) {skill_last_cmd_ = q;}
    const double max_step = rl_max_joint_vel_ / skill_rate_;
    std::vector<double> cmd(joint_names_.size());
    for (size_t k = 0; k < 5; ++k) {
      const double c = std::clamp(ik_signs_[k] * tgt[k] + ik_offsets_[k], lower_[k], upper_[k]);
      cmd[k] = std::clamp(c, (*skill_last_cmd_)[k] - max_step, (*skill_last_cmd_)[k] + max_step);
    }
    const double grip = std::clamp(rl_grip_.toRad(a[3] > 0.f ? rl_open_m_ : tcp_active_.closed_m),
      std::min(gripper_open_, gripper_close_), std::max(gripper_open_, gripper_close_));
    cmd[5] = std::clamp(grip, lower_[5], upper_[5]);
    gripper_cmd_ = cmd[5];
    skill_last_cmd_ = cmd;
    last_sent_ = cmd;
    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = cmd;
    point.time_from_start = rclcpp::Duration::from_seconds(skill_horizon_ / skill_rate_);
    jt.points.push_back(point);
    jt_pub_->publish(jt);
    stream_active_ = true;

    // 성공 판정 (학습: 퍽을 success_lift 위로 success_hold 동안). 실물은 쥔 퍽 높이를 못 재므로 잡는 점 높이 · 손가락으로
    const double lift = diff_ik_.forward(s.q).tcp.z() - (puck_b.z() + rl_grasp_dz_);
    const bool held = s.finger_m > rl_grasp_lo_ && s.finger_m < rl_grasp_hi_ && lift > rl_success_lift_ && a[3] <= 0.f;
    r.hold = held ? r.hold + 1.0 / skill_rate_ : 0.0;
    if (r.hold >= rl_success_hold_) {
      grasped_ = true;
      char buf[160];
      std::snprintf(buf, sizeof(buf), "grasped by policy (lift %.3f m, finger %.4f m = %.3f rad, %.1f s)", lift, s.finger_m,
        q[5], elapsed);
      stream_active_ = false;   // 성공: 마지막 점 (쥔 채 든 자세) 유지
      finishSkill(true, buf);
      return;
    }
    auto fb = std::make_shared<ExecuteSkill::Feedback>();
    fb->progress = static_cast<float>(std::clamp(elapsed / skill_timeout_, 0.0, 1.0));
    skill_gh_->publish_feedback(fb);
  }

  void finishSkill(bool ok, const std::string & msg)
  {
    auto result = std::make_shared<ExecuteSkill::Result>();
    result->success = ok;
    result->message = msg;
    if (ok) {
      RCLCPP_INFO(get_logger(), "skill done");
      skill_gh_->succeed(result);
    } else {
      RCLCPP_WARN(get_logger(), "skill failed: %s", msg.c_str());
      skill_gh_->abort(result);
    }
    endJob();
  }

  // ------------------------------------------------------------------ 누운 직육면체 세워 놓기 (lie_upright, #192 · #193)
  // 미션 skill "lie_upright:<frame>:<상자 x>:<상자 y>:<상자 요 deg>" (계획기 상자 자리 = 사전값). 바로 앞 target_look 이 누운 퍽을 잡아 둠.
  //   ① 상자 보기 자세 (lie_upright.view_*, 경우는 사전 상자 옆 위치로 고름) → 인식 /perception/place_box/prior 에 사전값을 내고
  //      /perception/place_box/measure → /perception/place_box (상자 윗면 중심 · 요). 못 재면 lie_upright.box_required 면 실패, 아니면 사전값
  //   ② 학습 시작 자세 READY_POSE_45 + 집게 벌림 → ③ 정책 (rate_hz, lieupright.hpp: 관측 45 → 행동 6 → β · ρ · 감쇠 최소제곱 IK)
  //   ④ 들었다가 (쥠 + 잡는 점 lift_ok 위) 퍽 중심 추정이 D 수평 release_xy 안 · 잡는 점이 D 위 release_z 아래 · β ≥ release_beta 에서 벌리면 놓음,
  //      그 뒤 집게는 늘 벌림. 놓은 뒤 clear_hold 지났고 잡는 점이 D 에서 clear_distance 넘게 떨어져 있으면 끝
  //   ⑤ 잡는 점을 retreat_up 위로 → D 에서 target_look (세운 퍽을 다시 재서 다음 toppick_tilt 가 씀)
  //   실패: 시간 (lie_upright.max_duration), 든 뒤 놓기 전에 쥠이 풀림 · D 밖에서 벌림 (떨어뜨림), 관측 · 추론 오류.
  //   퍽 상태는 실물에서 못 보므로 판정하지 않음 (세운 뒤 D 의 target_look 이 퍽을 찾는지가 확인)
  enum class LieStep { VIEW, BOX_WAIT, START, RUN };
  struct LieRun
  {
    LieStep step{LieStep::VIEW};
    Eigen::Vector3d puck{Eigen::Vector3d::Zero()};    // fixed_frame_ 인식 누운 퍽 중심 (target_look)
    Eigen::Vector3d axis{Eigen::Vector3d::UnitX()};   // fixed_frame_ 긴 축
    std::optional<Eigen::Vector3d> neighbor;          // fixed_frame_ 옆 퍽
    geometry_msgs::msg::PoseStamped prior;            // 계획기 상자 자리 (skill 이름의 frame)
    std::optional<geometry_msgs::msg::PoseStamped> box;   // 잰 상자 (인식 frame), 없으면 사전값
    rclcpp::Time asked{0, 0, RCL_ROS_TIME};           // 상자 측정 요청 시각
    bool asked_ok{false};
    // 정책 (base_link, 시작 때 정함)
    LieUprightState s;
    LieHoldFilter hold;
    std::optional<Joints5> target;
    std::optional<Joints5> q_prev;
    rclcpp::Time t_prev{0, 0, RCL_ROS_TIME};
    rclcpp::Time t_start{0, 0, RCL_ROS_TIME};
    double grasp_z{0.0};                              // 쥔 순간 잡는 점 높이
    bool lifted{false}, released{false};
    double clear{0.0};                                // 놓은 뒤 지난 시간 (s)
  };

  void loadLiePolicy()
  {
    if (!lie_enabled_) {return;}
    std::string file = lie_file_;
    if (file.empty()) {
      file = (ament_index_cpp::get_package_share_path("robot_policy") / "config" / "policies.yaml").string();
    }
    std::map<std::string, robot_policy::PolicySpec> specs;
    std::string err;
    if (!robot_policy::load_specs(file, specs, err) || !specs.count(lie_policy_name_)) {
      RCLCPP_ERROR(get_logger(), "lie_upright: policy '%s' not in %s (%s) -> lie_upright skill will fail", lie_policy_name_.c_str(),
        file.c_str(), err.c_str());
      return;
    }
    const auto & spec = specs.at(lie_policy_name_);
    bool ok = spec.action_kind == "tcp_delta" && spec.action_dim == 6;
    for (const auto & o : spec.observation) {
      const auto it = lieUprightObservationSizes().find(o.name);
      ok &= it != lieUprightObservationSizes().end() && it->second == o.size && o.required;
    }
    if (!ok || gripper_mode_ != GripperMode::ARM_JOINT) {
      RCLCPP_ERROR(get_logger(), "lie_upright: '%s' needs action tcp_delta(6), lieupright.hpp observations (all required) and "
        "gripper.mode arm_joint", lie_policy_name_.c_str());
      return;
    }
    auto runner = robot_policy::make_onnx_runner(spec.model, err);
    if (!runner) {
      RCLCPP_ERROR(get_logger(), "lie_upright: %s (%s) -> lie_upright skill will fail", err.c_str(), spec.model.c_str());
      return;
    }
    lie_policy_ = std::make_unique<robot_policy::LoadedPolicy>();
    lie_policy_->spec = spec;
    lie_policy_->runner = std::move(runner);
    RCLCPP_INFO(get_logger(), "lie_upright: policy '%s' loaded (%s, %.0f Hz)", lie_policy_name_.c_str(), spec.model.c_str(),
      spec.rate_hz);
  }

  // 점 · 방향을 frame 사이로 (지금 TF)
  std::optional<Eigen::Vector3d> lieToFrame(const Eigen::Vector3d & p, const std::string & from, const std::string & to, bool vec)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(to, from, tf2::TimePointZero);
      if (vec) {
        geometry_msgs::msg::Vector3Stamped a, b;
        a.vector.x = p.x();
        a.vector.y = p.y();
        a.vector.z = p.z();
        tf2::doTransform(a, b, tf);
        return Eigen::Vector3d(b.vector.x, b.vector.y, b.vector.z);
      }
      geometry_msgs::msg::PointStamped a, b;
      a.point.x = p.x();
      a.point.y = p.y();
      a.point.z = p.z();
      tf2::doTransform(a, b, tf);
      return Eigen::Vector3d(b.point.x, b.point.y, b.point.z);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "lie_upright: TF %s -> %s: %s", from.c_str(), to.c_str(), e.what());
      return std::nullopt;
    }
  }

  void startLieUpright(const TaskSkill & t)
  {
    step_ = PickStep::TASK;
    if (!lie_policy_) {
      finishSkill(false, "lie_upright: policy not loaded (lie_upright.enabled · policies.yaml · ONNX Runtime)");
      return;
    }
    LieRun r;
    r.prior.header.frame_id = t.frame;
    r.prior.pose.position.x = t.x;
    r.prior.pose.position.y = t.y;
    tf2::Quaternion qy;
    qy.setRPY(0.0, 0.0, t.z * M_PI / 180.0);
    r.prior.pose.orientation = tf2::toMsg(qy);
    // 누운 퍽 = 바로 앞 target_look 값 (fixed_frame_). 긴 축 · 옆 퍽은 그 자리의 /perception/pucks
    if (!target_look_latch_ || target_look_latch_->header.frame_id != fixed_frame_ ||
      (now() - target_look_latch_time_).seconds() > toppick_latch_max_age_)
    {
      finishSkill(false, "lie_upright: no target_look value for the lying puck (run target_look first)");
      return;
    }
    const auto & lp = target_look_latch_->pose.position;
    r.puck = Eigen::Vector3d(lp.x, lp.y, lp.z);
    target_look_latch_.reset();
    PuckArray::ConstSharedPtr pucks;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      if (last_pucks_ && measuredAfter(last_pucks_->header.stamp, now() - rclcpp::Duration::from_seconds(lie_pucks_age_))) {
        pucks = last_pucks_;
      }
    }
    bool axis_ok = false;
    if (pucks) {
      try {
        const auto tf = tf_buffer_->lookupTransform(fixed_frame_, pucks->header.frame_id, tf2::TimePointZero);
        double self_d = rl_self_match_ * 2.0, nb_d = rl_nb_match_;
        for (const auto & pk : pucks->pucks) {
          geometry_msgs::msg::Pose a;
          tf2::doTransform(pk.pose, a, tf);
          const Eigen::Vector3d p(a.position.x, a.position.y, a.position.z);
          const double d = std::hypot(p.x() - r.puck.x(), p.y() - r.puck.y());
          if (d <= self_d) {
            self_d = d;
            if (pk.orientation_valid) {
              tf2::Quaternion q;
              tf2::fromMsg(a.orientation, q);
              const tf2::Vector3 z = tf2::quatRotate(q, tf2::Vector3(0, 0, 1));
              r.axis = Eigen::Vector3d(z.x(), z.y(), z.z());
              axis_ok = true;
            }
          } else if (d > rl_self_match_ * 2.0 && d < nb_d) {
            nb_d = d;
            r.neighbor = p;
          }
        }
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN(get_logger(), "lie_upright: TF %s -> %s: %s", pucks->header.frame_id.c_str(), fixed_frame_.c_str(), e.what());
      }
    }
    if (!axis_ok) {
      finishSkill(false, "lie_upright: long axis of the lying puck not measured (/perception/pucks orientation_valid)");
      return;
    }
    lie_ = r;
    // ① 상자 보기 자세: 사전 상자 중심의 옆 위치 (base_link y) 로 왼쪽 회전 · 오른쪽 회전 · inline 고름
    const auto bc = lieToFrame(Eigen::Vector3d(t.x, t.y, 0.0), t.frame, rl_frame_, false);
    const auto current = currentJoints();
    if (!bc || !current) {
      finishSkill(false, "lie_upright: no TF for the box prior or no /joint_states");
      return;
    }
    const std::vector<double> & view = bc->y() < -lie_side_y_ ? lie_view_left_ : bc->y() > lie_side_y_ ? lie_view_right_ : lie_view_inline_;
    std::vector<double> q(joint_names_.size());
    for (size_t k = 0; k < 5; ++k) {q[k] = ik_signs_[k] * view[k] + ik_offsets_[k];}
    q[5] = sixthValue(*current);   // 집게는 지금 그대로
    if (!withinLimits(q)) {
      finishSkill(false, "lie_upright.view_* outside joint limits");
      return;
    }
    RCLCPP_INFO(get_logger(), "lie_upright: puck (%.3f, %.3f) %s, neighbor %s, box prior (%.3f, %.3f, %.1f deg) in %s -> box view",
      r.puck.x(), r.puck.y(), fixed_frame_.c_str(), r.neighbor ? "yes" : "none", t.x, t.y, t.z, t.frame.c_str());
    sendArm(q, trajDuration(*current, q, min_duration_), [this](bool ok, const std::string & msg) {
        if (!ok) {
          finishSkill(false, "lie_upright box view: " + msg);
          return;
        }
        lie_->step = LieStep::BOX_WAIT;
        wait_since_ = now();
      });
  }

  // onTick (20 Hz): settle 뒤 사전값을 내고 측정 요청 → 요청 뒤 받은 상태 · 상자 → 시작 자세
  void lieBoxTick()
  {
    auto & r = *lie_;
    const double waited = (now() - wait_since_).seconds();
    if (waited < lie_box_settle_) {return;}
    if (!r.asked_ok && r.asked.nanoseconds() == 0) {
      r.asked = now();
      geometry_msgs::msg::PoseStamped prior = r.prior;
      prior.header.stamp = now();
      lie_prior_pub_->publish(prior);
      if (!lie_box_client_->service_is_ready()) {
        lieBoxDone("place_box service not available");
        return;
      }
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      const uint64_t id = job_id_;
      {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        lie_req_.reset();
      }
      // 응답은 sensor_cb_group_ 에서 옴 → 결과만 남기고 (sensor_mutex_) 판단은 이 tick 에서
      lie_box_client_->async_send_request(req, [this, id](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
          const auto res = f.get();
          std::lock_guard<std::mutex> lock(sensor_mutex_);
          lie_req_ = std::make_pair(id, res->success ? std::string() : res->message);
        });
      return;
    }
    std::optional<std::string> status, refused;
    geometry_msgs::msg::PoseStamped::ConstSharedPtr box;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      if (lie_req_ && lie_req_->first == job_id_) {
        if (lie_req_->second.empty()) {
          r.asked_ok = true;
        } else {
          refused = lie_req_->second;
        }
        lie_req_.reset();
      }
      if (lie_status_ && lie_status_rx_ > r.asked) {status = lie_status_->data;}
      box = lie_box_;
      if (lie_box_ && lie_box_rx_ <= r.asked) {box = nullptr;}
    }
    if (refused) {
      lieBoxDone("place_box measure refused: " + *refused);
      return;
    }
    if (status) {
      if (status->find("\"ok\": true") != std::string::npos && box) {
        r.box = *box;
        lieBoxDone("");
      } else if (status->find("\"ok\": false") != std::string::npos) {
        lieBoxDone("place_box: " + *status);
      }
      return;   // ok 인데 상자가 아직이면 다음 tick
    }
    if (waited > lie_box_settle_ + lie_box_timeout_) {lieBoxDone("place_box: no result within lie_upright.box_timeout");}
  }

  void lieBoxDone(const std::string & why)
  {
    auto & r = *lie_;
    if (!why.empty()) {
      if (lie_box_required_) {
        finishSkill(false, "lie_upright: " + why);
        return;
      }
      RCLCPP_WARN(get_logger(), "lie_upright: %s -> planner box pose", why.c_str());
    }
    r.step = LieStep::START;
    // 상자 · 퍽 · 옆 퍽 · 세울 자리 → base_link (차체는 이 스킬 동안 서 있음)
    const auto & B = r.box ? *r.box : r.prior;
    tf2::Quaternion bq;
    tf2::fromMsg(B.pose.orientation, bq);
    const tf2::Vector3 bx = tf2::quatRotate(bq, tf2::Vector3(1, 0, 0));
    const auto c = lieToFrame(Eigen::Vector3d(B.pose.position.x, B.pose.position.y, B.pose.position.z), B.header.frame_id, rl_frame_, false);
    const auto ax = lieToFrame(Eigen::Vector3d(bx.x(), bx.y(), bx.z()), B.header.frame_id, rl_frame_, true);
    const auto puck = lieToFrame(r.puck, fixed_frame_, rl_frame_, false);
    const auto axis = lieToFrame(r.axis, fixed_frame_, rl_frame_, true);
    // 상자 윗면 높이 (base_link): 바닥 frame 원점 위 box_top 점의 높이 (바닥 frame 과 base_link 는 평행)
    const auto top = lieToFrame(Eigen::Vector3d(0.0, 0.0, lie_box_top_), lie_floor_frame_, rl_frame_, false);
    std::optional<Eigen::Vector3d> nb;
    if (r.neighbor) {nb = lieToFrame(*r.neighbor, fixed_frame_, rl_frame_, false);}
    if (!c || !ax || !puck || !axis || !top || (r.neighbor && !nb)) {
      finishSkill(false, "lie_upright: TF to " + rl_frame_ + " failed");
      return;
    }
    // 상자 요는 90° 마다 같음: 사전 요에 가까운 것 (학습 상자 요 = 정지 회전 ± 3°)
    double yaw = std::atan2(ax->y(), ax->x());
    const auto pax = lieToFrame([&] {
        tf2::Quaternion pq;
        tf2::fromMsg(r.prior.pose.orientation, pq);
        const tf2::Vector3 v = tf2::quatRotate(pq, tf2::Vector3(1, 0, 0));
        return Eigen::Vector3d(v.x(), v.y(), v.z());
      }(), r.prior.header.frame_id, rl_frame_, true);
    if (pax) {
      const double py = std::atan2(pax->y(), pax->x());
      yaw = py + std::remainder(yaw - py, M_PI / 2);
    }
    auto & s = r.s;
    s.box_c = Eigen::Vector2d(c->x(), c->y());
    s.box_yaw = yaw;
    const double top_z = top->z();
    s.puck = Eigen::Vector3d(puck->x(), puck->y(), top_z + lie_puck_dz_);
    s.long_yaw = std::atan2(axis->y(), axis->x());
    if (nb) {s.neighbor = Eigen::Vector2d(nb->x(), nb->y());}
    // 세울 자리: 옆 퍽이 없으면 '있었다면' 자리 = 상자 중심에 대해 누운 퍽의 반대편 (같은 상자 두 칸)
    const Eigen::Vector2d nb_for_spot = s.neighbor ? *s.neighbor : Eigen::Vector2d(2.0 * s.box_c - s.puck.head<2>());
    const Eigen::Vector2d d = lieStandSpot(lie_params_, s.box_c, s.box_yaw, nb_for_spot, diff_ik_.params().joint1_origin.x());
    s.spot = Eigen::Vector3d(d.x(), d.y(), top_z + lie_spot_dz_);
    RCLCPP_INFO(get_logger(), "lie_upright: box (%.3f, %.3f, %.1f deg) %s, puck (%.3f, %.3f) long axis %.1f deg, stand spot D (%.3f, %.3f, %.3f) "
      "in %s -> start pose", s.box_c.x(), s.box_c.y(), yaw * 180.0 / M_PI, r.box ? "measured" : "planner", s.puck.x(), s.puck.y(),
      s.long_yaw * 180.0 / M_PI, d.x(), d.y(), s.spot.z(), rl_frame_.c_str());
    // ② 학습 시작 자세 + 벌림
    const auto current = currentJoints();
    if (!current) {
      finishSkill(false, "no recent /joint_states for arm joints");
      return;
    }
    std::vector<double> q(joint_names_.size());
    for (size_t k = 0; k < 5; ++k) {q[k] = ik_signs_[k] * kLieReadyPose45[k] + ik_offsets_[k];}
    q[5] = std::clamp(rl_grip_.toRad(lie_params_.finger_open), lower_[5], upper_[5]);
    gripper_cmd_ = q[5];
    if (!withinLimits(q)) {
      finishSkill(false, "lie_upright: start pose outside joint limits");
      return;
    }
    sendArm(q, trajDuration(*current, q, min_duration_), [this](bool ok, const std::string & msg) {
        if (!ok) {
          finishSkill(false, "lie_upright start pose: " + msg);
          return;
        }
        lie_->step = LieStep::RUN;
        lie_->t_start = now();
        active_policy_ = lie_policy_.get();
        skill_rate_ = lie_policy_->spec.rate_hz;
        skill_last_cmd_.reset();
        startSkillTimer();
      });
  }

  // ③ 한 tick: 관측 45 → 정책 → 행동 6 → β · ρ · IK → 6 관절 목표 스트리밍
  void lieTick(const std::vector<double> & q, double elapsed)
  {
    auto & r = *lie_;
    auto & s = r.s;
    if ((now() - r.t_start).seconds() > lie_max_duration_) {
      finishSkill(false, "lie_upright: not finished within lie_upright.max_duration " + std::to_string(lie_max_duration_) + " s");
      return;
    }
    std::optional<Joints5> qd_meas = Joints5{};
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      for (size_t k = 0; k < 5; ++k) {
        const auto it = joint_vel_.find(joint_names_[k]);
        if (it == joint_vel_.end()) {qd_meas.reset(); break;}
        (*qd_meas)[k] = it->second / ik_signs_[k];
      }
    }
    for (size_t k = 0; k < 5; ++k) {s.q[k] = (q[k] - ik_offsets_[k]) / ik_signs_[k];}
    const rclcpp::Time t = now();
    s.qd = Joints5{};
    if (qd_meas) {
      s.qd = *qd_meas;
    } else if (r.q_prev) {
      const double dt = std::max((t - r.t_prev).seconds(), 1e-3);
      for (size_t k = 0; k < 5; ++k) {s.qd[k] = (s.q[k] - (*r.q_prev)[k]) / dt;}
    }
    r.q_prev = s.q;
    r.t_prev = t;
    s.finger_m = rl_grip_.toMeters(q[5]);
    s.held = r.hold.update(s.finger_m, lie_params_);
    const ArmFrame f = diff_ik_.forward(s.q);
    // 단계 (실물에서 볼 수 있는 것만): 쥔 순간 높이 → 든 뒤 → 벌리면 놓음
    if (s.held && !r.lifted && r.grasp_z == 0.0) {r.grasp_z = f.tcp.z();}
    if (!s.held && !r.lifted) {r.grasp_z = 0.0;}   // 들기 전에 풀리면 다시 (학습: 들기 전 놓침은 다시 쥘 수 있음)
    if (s.held && r.grasp_z != 0.0 && f.tcp.z() > r.grasp_z + lie_lift_ok_) {r.lifted = true;}
    if (r.lifted && !r.released && !s.held) {
      finishSkill(false, "lie_upright: grip lost after lifting before release (dropped)");
      return;
    }

    robot_policy::ObsValues values;
    for (auto & kv : lieUprightObservation(diff_ik_, s)) {values[kv.first] = kv.second;}
    const auto & spec = lie_policy_->spec;
    std::vector<float> obs, raw;
    std::string err;
    if (!robot_policy::ObservationBuilder(spec).build(values, obs, err)) {
      finishSkill(false, "lie_upright: observation invalid: " + err);
      return;
    }
    if (!lie_policy_->runner->run(obs, raw)) {finishSkill(false, "lie_upright: policy inference failed"); return;}
    auto a = robot_policy::ActionMapper(spec).map(raw);
    std::array<float, 6> act{};
    for (size_t i = 0; i < 6; ++i) {act[i] = std::clamp(a[i], -1.f, 1.f);}
    if (r.lifted && !r.released && act[5] > 0.f) {
      // 학습: 놓음 = 퍽이 상자 위에 자리 잡았을 때만 (공중에서 벌리면 떨어뜨림). 실물은 퍽을 못 보므로 잡는 점이 세울 자리 위
      //   (수평 release_xy · 높이 release_z 안) 이고 집게가 퍽을 세운 자세 (β ≥ release_beta) 일 때만 놓음으로 봄.
      //   수평은 퍽 중심 추정 (잡는 점 + 집게 방향 × release_center_offset, 학습 잡는 점 = 퍽 중심 − a × 11 mm) 으로 D 와 비교
      //   (학습 '세울 자리 3 cm 안' 은 퍽 중심 기준). Isaac r2e 기록 세 에피소드: 추정 중심-D 3 ~ 12 mm, D 위 20 ~ 39 mm, β 1.55 ~ 1.75.
      //   든 직후 (아직 누운 채) 는 D 위 51 ~ 61 mm · β 0.8 ~ 1.0
      const Eigen::Vector3d pc = f.tcp + f.approach() * lie_release_center_offset_;
      const double dh = std::hypot(pc.x() - s.spot.x(), pc.y() - s.spot.y());
      const double dz = f.tcp.z() - s.spot.z();
      char buf[200];
      std::snprintf(buf, sizeof(buf), "puck center (est.) %.1f mm beside, grasp point %.1f mm above the stand spot, beta %.2f rad", dh * 1000.0,
        dz * 1000.0, s.beta);
      if (dh > lie_release_xy_ || dz > lie_release_z_ || s.beta < lie_release_beta_) {
        finishSkill(false, std::string("lie_upright: gripper opened away from the stand spot (dropped): ") + buf);
        return;
      }
      r.released = true;
      RCLCPP_INFO(get_logger(), "lie_upright: released, %s", buf);
    }
    if (r.released) {act[5] = 1.f;}   // 학습: 놓은 뒤 집게는 늘 벌림
    // 관절 속도 제한: scale = 행동 전체를 같은 비율로 줄임 (β · ρ 관측이 실제 집게와 맞음), clip = 관절마다 자름 (아래 cmd)
    const Joints5 tgt = lie_vel_scale_ ?
      lieApplyActionLimited(diff_ik_, lie_params_, r.target ? *r.target : s.q, s.q, act, s.beta, s.rho, lie_max_joint_vel_ / skill_rate_) :
      lieApplyAction(diff_ik_, lie_params_, r.target ? *r.target : s.q, s.q, act, s.beta, s.rho);
    s.action = act;

    if (!skill_last_cmd_) {skill_last_cmd_ = q;}
    const double max_step = lie_max_joint_vel_ / skill_rate_;
    std::vector<double> cmd(joint_names_.size());
    Joints5 sent{};
    for (size_t k = 0; k < 5; ++k) {
      const double c = std::clamp(ik_signs_[k] * tgt[k] + ik_offsets_[k], lower_[k], upper_[k]);
      cmd[k] = std::clamp(c, (*skill_last_cmd_)[k] - max_step, (*skill_last_cmd_)[k] + max_step);
      sent[k] = (cmd[k] - ik_offsets_[k]) / ik_signs_[k];
    }
    r.target = sent;   // 학습 arm_target 처럼 다음 스텝 기준 = 실제로 보낸 목표 (속도 제한으로 자르면 자른 값)
    const double grip = std::clamp(rl_grip_.toRad(act[5] > 0.f ? lie_params_.finger_open : lie_params_.finger_close),
      std::min(gripper_open_, gripper_close_), std::max(gripper_open_, gripper_close_));
    cmd[5] = std::clamp(grip, lower_[5], upper_[5]);
    gripper_cmd_ = cmd[5];
    skill_last_cmd_ = cmd;
    last_sent_ = cmd;
    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = cmd;
    point.time_from_start = rclcpp::Duration::from_seconds(skill_horizon_ / skill_rate_);
    jt.points.push_back(point);
    jt_pub_->publish(jt);
    stream_active_ = true;

    // ④ 끝: 놓은 뒤 clear_hold 지났고 잡는 점이 D 에서 clear_distance 넘게 (학습 성공 = 놓은 뒤 퍽이 서고 손이 퍽에서 3 cm 넘게 1 s.
    //   실물은 퍽을 못 보므로 놓은 시각부터 셈 → 학습 성공 시점보다 앞서 끝남, Isaac r2e 기록 세 에피소드에서 3 ~ 5 스텝 앞)
    if (r.released) {r.clear += 1.0 / skill_rate_;}
    if (r.released && r.clear >= lie_clear_hold_ && (f.tcp - s.spot).norm() > lie_clear_dist_) {
      if (skill_timer_) {skill_timer_->cancel();}
      holdAtMeasured();
      char buf[200];
      std::snprintf(buf, sizeof(buf), "lie_upright: placed and released (%.1f s), hand %.3f m from stand spot", (now() - r.t_start).seconds(),
        (f.tcp - s.spot).norm());
      RCLCPP_INFO(get_logger(), "%s", buf);
      // ⑤ 잡는 점을 retreat_up 만큼 위로 (지금 집게 자세에서 β · ρ 를 다시 정하고 같은 IK 를 측정 없이 이어 계산, 세운 퍽 · 상자를 치지 않게)
      //   → D 에서 target_look
      Joints5 qu = s.q, tu = s.q;
      double bu = s.beta, ru = s.rho;
      lieBetaRhoFrom(diff_ik_, lie_params_, s.q, bu, ru);   // 지금 집게 자세 그대로 (정책 β · ρ 목표와 조금 다를 수 있음)
      const int n_up = static_cast<int>(std::ceil(lie_retreat_up_ / lie_params_.cart_scale - 1e-9));
      for (int i = 0; i < n_up; ++i) {
        tu = lieApplyAction(diff_ik_, lie_params_, tu, qu, {0.f, 0.f, 1.f, 0.f, 0.f, 1.f}, bu, ru);
        qu = tu;
      }
      const double rise = diff_ik_.forward(qu).tcp.z() - f.tcp.z();
      std::vector<double> qcmd(joint_names_.size());
      for (size_t k = 0; k < 5; ++k) {qcmd[k] = ik_signs_[k] * qu[k] + ik_offsets_[k];}
      qcmd[5] = cmd[5];
      if (!withinLimits(qcmd) || rise < 0.8 * lie_retreat_up_) {
        std::snprintf(buf, sizeof(buf), "lie_upright: cannot retreat up %.0f mm after release (rise %.0f mm or joint limits)",
          lie_retreat_up_ * 1000.0, rise * 1000.0);
        finishSkill(false, buf);
        return;
      }
      const std::string done = buf;
      const Eigen::Vector3d spot = s.spot;
      const auto current = currentJoints();
      sendArm(qcmd, trajDuration(current ? *current : q, qcmd, lie_retreat_time_), [this, done, spot](bool ok, const std::string & msg) {
          if (!ok) {
            finishSkill(false, "lie_upright retreat up: " + msg);
            return;
          }
          if (!lie_look_after_) {
            finishSkill(true, done);
            return;
          }
          TaskSkill look;
          look.kind = Task::TARGET_LOOK;
          look.frame = rl_frame_;
          look.x = spot.x();
          look.y = spot.y();
          look.z = spot.z();
          lie_.reset();
          task_ = Task::TARGET_LOOK;
          startTargetLook(look);
        });
      return;
    }
    auto fb = std::make_shared<ExecuteSkill::Feedback>();
    fb->progress = static_cast<float>(std::clamp(elapsed / skill_timeout_, 0.0, 1.0));
    skill_gh_->publish_feedback(fb);
  }

  // ------------------------------------------------------------------ 미션 skill (execute_skill 이름)
  // "toppick_tilt" · "pallet_load_3_1_tilt" · "pallet_unload_2_0_tilt" · "place_L1_m1_p1_tilt" · "place_at:odom:x:y:z" ·
  // "target_look:base_footprint:x:y:z" · "box_look:map:x:y:z" · "barcode_look:map:x:y:z" · "lie_upright:base_footprint:x:y:yaw_deg"
  static std::optional<TaskSkill> parseTaskSkill(const std::string & name)
  {
    TaskSkill t;
    if (name == "toppick_tilt") {
      t.kind = Task::TOPPICK;
      return t;
    }
    auto parts = [](const std::string & text, char sep) {
        std::vector<std::string> out;
        std::string tok;
        std::istringstream in(text);
        while (std::getline(in, tok, sep)) {out.push_back(tok);}
        return out;
      };
    if (name.rfind("place_at:", 0) == 0 || name.rfind("target_look:", 0) == 0 || name.rfind("box_look:", 0) == 0 ||
      name.rfind("barcode_look:", 0) == 0 || name.rfind("lie_upright:", 0) == 0)
    {
      const auto f = parts(name, ':');   // place_at | target_look | box_look | barcode_look : frame : x : y : z
      if (f.size() != 5 || f[1].empty()) {return std::nullopt;}
      try {
        t.kind = f[0] == "target_look" ? Task::TARGET_LOOK : f[0] == "place_at" ? Task::PLACE :
          f[0] == "lie_upright" ? Task::LIE_UPRIGHT : Task::LOOK;
        t.look = f[0];
        t.frame = f[1];
        t.x = std::stod(f[2]);
        t.y = std::stod(f[3]);
        t.z = std::stod(f[4]);
      } catch (const std::exception &) {
        return std::nullopt;
      }
      return t;
    }
    const auto f = parts(name, '_');
    if (f.size() == 5 && f[0] == "pallet" && (f[1] == "load" || f[1] == "unload") && f[4] == "tilt" &&
      f[2].size() == 1 && f[3].size() == 1 && f[2][0] >= '1' && f[2][0] <= '3' && f[3][0] >= '0' && f[3][0] <= '3')
    {
      t.kind = f[1] == "load" ? Task::PALLET_LOAD : Task::PALLET_UNLOAD;
      t.row = f[2][0] - '0';
      t.index = f[3][0] - '0';
      return t;
    }
    // place_<L>_<dx>_<dy>_tilt, dx · dy = m1 · 0 · p1 (지도 +Y 가 위)
    static const std::map<std::string, std::string> idx = {{"m1", "-1"}, {"0", "0"}, {"p1", "1"}};
    if (f.size() == 5 && f[0] == "place" && (f[1] == "L1" || f[1] == "L2" || f[1] == "L3") && f[4] == "tilt" &&
      idx.count(f[2]) && idx.count(f[3]))
    {
      t.kind = Task::PLACE;
      t.place = f[1];
      t.cell = idx.at(f[2]) + "," + idx.at(f[3]);
      return t;
    }
    return std::nullopt;
  }

  void startTaskSkill(const TaskSkill & t)
  {
    task_ = t.kind;
    switch (t.kind) {
      case Task::TARGET_LOOK:
        startTargetLook(t);
        return;
      case Task::LOOK:
        startLook(t);
        return;
      case Task::LIE_UPRIGHT:
        startLieUpright(t);
        return;
      case Task::TOPPICK: {
          geometry_msgs::msg::PoseStamped target;   // 비면 ② 추적 위치 (use_alignment 면 ① 먼저)
          if (target_look_latch_ && (now() - target_look_latch_time_).seconds() <= toppick_latch_max_age_) {
            target = *target_look_latch_;
            RCLCPP_INFO(get_logger(), "pick target: target_look (%.3f, %.3f, %.3f) in %s", target.pose.position.x,
              target.pose.position.y, target.pose.position.z, target.header.frame_id.c_str());
          } else if (target_look_latch_) {
            RCLCPP_WARN(get_logger(), "target_look value older than toppick.latch_max_age -> perception target");
          }
          target_look_latch_.reset();   // 한 번만 씀 (실패 후 재시도는 새 추적 위치)
          beginPick(target, {toppick_pitch_}, toppick_approach_);
          return;
        }
      case Task::PALLET_LOAD:
      case Task::PALLET_UNLOAD:
        runPalletSlot(t.row, t.index, t.kind == Task::PALLET_LOAD);
        return;
      case Task::PLACE: {
          geometry_msgs::msg::PointStamped cell;
          if (t.place.empty()) {   // place_at
            cell.header.frame_id = t.frame;
            cell.point.x = t.x;
            cell.point.y = t.y;
            cell.point.z = t.z;
            placeAt(cell, true);
            return;
          }
          const auto it = place_cells_.find(t.place + "/" + t.cell);
          if (it == place_cells_.end()) {
            finishSkill(false, "place: no cell " + t.place + " " + t.cell + " in place.cells_file");
            return;
          }
          cell.header.frame_id = place_cells_frame_;
          cell.point.x = it->second.first;
          cell.point.y = it->second.second;
          placeAt(cell, false);
          return;
        }
      default:
        finishSkill(false, "unknown mission skill");
        return;
    }
  }

  // 팔레트 칸 궤적 재생. load: 쥔 채 칸으로 → 놓기 → 빠져나옴 / unload: 벌린 채 칸으로 → 집기 → 쥔 채 빠져나옴
  //   궤적의 rotate · carry · place (칸까지) 와 retract · return · rotate_back (돌아오기) 사이에서 그리퍼를 바꾼다
  void runPalletSlot(int row, int index, bool load)
  {
    step_ = PickStep::TASK;
    const auto it = pallet_.find({row, index});
    if (it == pallet_.end()) {
      finishSkill(false, "pallet: no trajectory for row " + std::to_string(row) + " slot " + std::to_string(index) +
        " (pallet.traj_file)");
      return;
    }
    if (load && pallet_require_held_ && gripper_mode_ == GripperMode::ARM_JOINT) {
      const auto holding = gripperHolding();
      if (!holding || !*holding) {
        finishSkill(false, "pallet_load: not holding a puck (gripper closed fully or no /joint_states)");
        return;
      }
    }
    // 구간 경유점 (Isaac pallet_place_check.py timeline · timeline_unload 와 같은 순서, #75)
    //   놓기: rotate → carry → place (쥠) → 머묾 → 벌림 → retract → return → rotate_back
    //   꺼내기: rotate_back · return · retract 거꾸로 → release (벌린 채) → 머묾 → 쥠 → place · carry · rotate 거꾸로 → q_front
    const auto & segs = it->second;
    auto find = [&segs](const std::string & n) -> const PalletSegment * {
        for (const auto & g : segs) {if (g.name == n) {return &g;}}
        return nullptr;
      };
    std::vector<std::vector<double>> in, out;
    std::string missing;
    auto add = [&](std::vector<std::vector<double>> & dst, const std::string & n, bool reversed) {
        const auto * g = find(n);
        if (!g) {missing += (missing.empty() ? "" : ", ") + n; return;}
        if (reversed) {dst.insert(dst.end(), g->q.rbegin(), g->q.rend());} else {dst.insert(dst.end(), g->q.begin(), g->q.end());}
      };
    if (load) {
      add(in, "rotate", false); add(in, "carry", false); add(in, "place", false);
      add(out, "retract", false); add(out, "return", false); add(out, "rotate_back", false);
    } else {
      add(in, "rotate_back", true); add(in, "return", true); add(in, "retract", true); add(in, "release", false);
      add(out, "place", true); add(out, "carry", true); add(out, "rotate", true);
      const auto f = pallet_front_.find({row, index});
      if (f != pallet_front_.end()) {out.push_back(f->second);} else if (!transit_tilt_.empty()) {out.push_back(transit_tilt_);}
    }
    if (!missing.empty()) {
      finishSkill(false, "pallet: segment(s) missing in traj_file: " + missing);
      return;
    }
    // 속도: 놓기의 3 열 쥔 구간만 0.4 rad/s, 나머지 (빈 집게 · 꺼내기 전부) 0.8 (#75)
    const double in_speed = (load && row == 3) ? pallet_row3_hold_speed_ : pallet_speed_;
    const double out_speed = pallet_speed_;
    const double in_grip = load ? gripper_close_ : pallet_release_;
    const double out_grip = load ? pallet_release_ : gripper_close_;
    if (sag_enabled_ && gripper_mode_ != GripperMode::ACTION) {   // 처짐 보정 스트리밍 (sag.*)
      runPalletStream(in, out, in_speed, out_speed, in_grip, out_grip, load);
      return;
    }
    if (gripper_mode_ == GripperMode::ARM_JOINT) {gripper_cmd_ = in_grip;}
    sendWaypoints(in, in_speed, in_grip, pallet_dwell_, [this, out, out_speed, out_grip, load](bool ok, const std::string & msg) {
        if (!ok) {
          finishSkill(false, "pallet approach: " + msg);
          return;
        }
        moveGripper(out_grip, [this, out, out_speed, out_grip, load](bool gok, const std::string & gmsg) {
            if (!gok) {
              finishSkill(false, "pallet gripper: " + gmsg);
              return;
            }
            auto leave = [this, out, out_speed, out_grip, load]() {
                sendWaypoints(out, out_speed, out_grip, 0.0, [this, load](bool lok, const std::string & lmsg) {
                    finishSkill(lok, lok ? (load ? "loaded" : "unloaded") : "pallet leave: " + lmsg);
                  });
              };
            if (load || gripper_mode_ != GripperMode::ARM_JOINT) {
              leave();
              return;
            }
            waitGripper([this, leave]() {   // unload: 집었는지 확인 후 빠져나옴
                const auto holding = gripperHolding();
                if (gripper_fail_on_empty_ && (!holding || !*holding)) {
                  finishSkill(false, "pallet_unload: empty grasp (no puck in slot?)");
                  return;
                }
                leave();
              });
          });
      });
  }

  // 팔레트 칸 궤적 + 처짐 보정 (sag_comp.hpp, Isaac pallet_place_check --ki 와 같은 식).
  //   시간표: 시작 자세 settle → in (in_grip) → 머묾 → 그리퍼 out_grip (gripper.move_time) → [꺼내기: settle_time 뒤 쥠 확인] → out
  //   sag.rate_hz 로 /arm_controller/joint_trajectory 한 점 (계획 + bias, 6 번째 = 그리퍼). 측정이 계획에서 max_error 넘게 벗어나면 중지
  void runPalletStream(
    const std::vector<std::vector<double>> & in, const std::vector<std::vector<double>> & out,
    double in_speed, double out_speed, double in_grip, double out_grip, bool load)
  {
    const auto current = currentJoints();
    if (!current) {
      finishSkill(false, "no recent /joint_states for arm joints");
      return;
    }
    auto arm5 = [](const std::vector<double> & q) {Arm5 a{}; for (size_t j = 0; j < 5; ++j) {a[j] = q[j];} return a;};
    std::vector<Arm5> qin, qout;
    for (const auto * src : {&in, &out}) {
      for (const auto & q : *src) {
        std::vector<double> q6(q.begin(), q.end());
        q6.push_back(0.0);
        if (q.size() != 5 || !withinLimits(q6)) {
          finishSkill(false, "pallet: waypoint outside joint limits or not 5 joints");
          return;
        }
        (src == &in ? qin : qout).push_back(arm5(q));
      }
    }
    const double grip0 = sixthValue(*current);
    JointTimeline tl(arm5(*current), grip0);
    tl.hold(sag_settle_, grip0);                          // 지금 자세에서 bias 먼저 맞춤
    tl.moveThrough(qin, in_speed, in_grip, min_duration_);
    tl.hold(pallet_dwell_, in_grip);                      // 놓는 · 집는 자세에서 머묾
    tl.hold(gripper_move_time_, out_grip);                // 그리퍼 바꿈
    sag_check_t_.reset();
    if (!load && gripper_mode_ == GripperMode::ARM_JOINT) {
      tl.hold(gripper_settle_, out_grip);                 // 쥔 뒤 GripGuard 판정 시간
      sag_check_t_ = tl.duration();
    }
    tl.moveThrough(qout, out_speed, out_grip, 0.02);
    sag_tl_ = tl;
    sag_int_ = SagIntegrator(sag_ki_, sag_bias_max_);
    sag_load_ = load;
    sag_done_msg_ = load ? "loaded" : "unloaded";
    sag_end_plan_.reset();
    sag_start_ = sag_last_ = now();
    RCLCPP_INFO(get_logger(), "pallet %s: stream %.1f s (settle %.1f s, ki %.1f, %.0f Hz)", load ? "load" : "unload",
      tl.duration(), sag_settle_, sag_ki_, sag_rate_);
    const uint64_t id = job_id_;
    skill_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / sag_rate_),
      [this, id]() {if (id == job_id_) {palletStreamTick();}}, job_cb_group_);
  }

  void palletStreamTick()
  {
    if (job_ != Job::SKILL || skill_gh_->is_canceling() || !sag_tl_) {return;}   // 취소 마무리는 onTick
    const auto cur = currentJoints();
    if (!cur) {
      finishSkill(false, "pallet: no recent /joint_states during stream");
      return;
    }
    const rclcpp::Time t_now = now();
    const double dt = std::clamp((t_now - sag_last_).seconds(), 0.0, 0.1);
    sag_last_ = t_now;
    if (armHeld()) {
      // HOLD · E-stop 동안 팔은 멈춰 있는데 계획 시간 · 적분이 앞서 가면 bias 가 포화되고 마지막 점이 실제보다 앞서 남음
      //   (REARM 순간 그 점으로 튐, #144 · #107) → 계획 시각을 멈추고 (시작 시각을 미룸) 적분 · 발행 · 오차 판정을 쉼
      sag_start_ = sag_start_ + rclcpp::Duration::from_seconds(dt);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "pallet: arm HOLD / E-stop -> stream paused");
      return;
    }
    const double t = (t_now - sag_start_).seconds();
    const auto p = sag_tl_->at(t);
    Arm5 meas{};
    double err = 0.0;
    size_t err_j = 0;
    for (size_t j = 0; j < 5; ++j) {
      meas[j] = (*cur)[j];
      if (std::abs(p.q[j] - meas[j]) > err) {err = std::abs(p.q[j] - meas[j]); err_j = j;}
    }
    if (err > sag_max_error_) {
      char buf[160];
      std::snprintf(buf, sizeof(buf), "pallet: joint%zu %.3f rad off plan at %.1f s (blocked?)", err_j + 1, err, t);
      finishSkill(false, buf);
      return;
    }
    sag_int_.update(p.q, meas, dt);
    // 컨트롤러는 한 점을 horizon 에 걸쳐 따라가므로 horizon 앞의 계획을 낸다 (안 그러면 움직이는 동안 속도 × horizon 만큼
    // 늘 뒤처지고, 적분이 그 뒤처짐까지 처짐으로 쌓아 칸 앞에서 지나침. mock 팔: 0.8 rad/s 에서 bias 0.055 rad)
    const auto ahead = sag_tl_->at(t + sag_horizon_);
    const Arm5 c = sag_int_.command(ahead.q);
    std::vector<double> q(c.begin(), c.end());
    q.push_back(gripper_mode_ == GripperMode::ARM_JOINT ? ahead.grip : gripper_roll_default_);
    for (size_t i = 0; i < q.size(); ++i) {q[i] = std::clamp(q[i], lower_[i], upper_[i]);}
    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = q;
    point.time_from_start = rclcpp::Duration::from_seconds(sag_horizon_);
    jt.points.push_back(point);
    jt_pub_->publish(jt);
    stream_active_ = true;
    last_sent_ = q;
    if (gripper_mode_ == GripperMode::ARM_JOINT) {gripper_cmd_ = ahead.grip;}

    if (sag_check_t_ && t >= *sag_check_t_) {   // 꺼내기: 집었는지
      sag_check_t_.reset();
      const auto holding = gripperHolding();
      if (gripper_fail_on_empty_ && (!holding || !*holding)) {
        finishSkill(false, "pallet_unload: empty grasp (no puck in slot?)");
        return;
      }
    }
    if (t >= sag_tl_->duration()) {
      const auto & b = sag_int_.bias();
      RCLCPP_INFO(get_logger(), "pallet: sag bias [%.3f %.3f %.3f %.3f %.3f] rad", b[0], b[1], b[2], b[3], b[4]);
      sag_end_plan_ = sag_tl_->points().back().q;   // 다음 놓기 스트림이 이 bias 를 이어 씀
      sag_end_bias_ = b;
      sag_tl_.reset();
      stream_active_ = false;   // 성공: 마지막 점 (계획 끝 + 처짐 bias) 유지 — 측정 자세로 내리면 bias 만큼 처짐
      finishSkill(true, sag_done_msg_);
    }
  }

  // 경유점을 한 궤적으로. 경유점 사이 시간 = 관절 변화 최대값 / speed, 첫 점은 지금 자세에서 출발, 끝에서 dwell 초 머묾
  void sendWaypoints(
    const std::vector<std::vector<double>> & qs, double speed, double grip, double dwell, Done done)
  {
    const auto current = currentJoints();
    if (!current) {
      done(false, "no recent /joint_states for arm joints");
      return;
    }
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
    std::vector<double> prev = *current;
    double t = 0.0;
    for (const auto & q5 : qs) {
      if (q5.size() != 5) {
        done(false, "pallet: waypoint must have 5 joints");
        return;
      }
      std::vector<double> q(q5.begin(), q5.end());
      q.push_back(gripper_mode_ == GripperMode::ARM_JOINT ? grip : sixthValue(*current));
      if (!withinLimits(q)) {
        done(false, "pallet: waypoint outside joint limits");
        return;
      }
      double max_delta = 0.0;
      for (size_t j = 0; j < 5; ++j) {max_delta = std::max(max_delta, std::abs(q[j] - prev[j]));}
      // 첫 점은 지금 자세에서 오는 거리라 arm.min_duration 이상
      t += points.empty() ? std::max(min_duration_, max_delta / speed) : std::max(0.02, max_delta / speed);
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = q;
      point.time_from_start = rclcpp::Duration::from_seconds(t);
      points.push_back(point);
      prev = q;
    }
    if (points.empty()) {
      done(false, "pallet: empty trajectory");
      return;
    }
    points.back().velocities.assign(points.back().positions.size(), 0.0);
    if (dwell > 0.0) {   // 같은 자리 한 점 더 = 머묾
      auto hold = points.back();
      t += dwell;
      hold.time_from_start = rclcpp::Duration::from_seconds(t);
      points.push_back(hold);
    }
    sendTrajectory(points, t, done);
  }

  // 쥔 퍽을 칸 (frame 기준 x, y, 윗면 높이 z) 에 놓기. cell.z 를 안 쓰면 (has_z false) place.surface_height (바닥 기준)
  //   집게 place.pitch (45°) · 집게 끝 = 퍽 중심 + grasp_z_offset → APPROACH (위 · 뒤) → LOWER (직선) → 벌림 → RETREAT → 이동 자세
  void placeAt(const geometry_msgs::msg::PointStamped & cell, bool has_z)
  {
    step_ = PickStep::TASK;
    if (gripper_mode_ == GripperMode::ARM_JOINT) {
      const auto holding = gripperHolding();
      if (!holding || !*holding) {
        finishSkill(false, "place: not holding a puck (gripper closed fully or no /joint_states)");
        return;
      }
    }
    // 칸 → arm_base_link (최신 TF)
    geometry_msgs::msg::PointStamped arm;
    double floor_z = 0.0;   // arm_base_link 기준 바닥 높이
    try {
      geometry_msgs::msg::PointStamped src = cell;
      src.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
      arm = tf_buffer_->transform(src, base_frame_);
    } catch (const tf2::TransformException & e) {
      finishSkill(false, std::string("place: TF ") + cell.header.frame_id + " -> " + base_frame_ + ": " + e.what());
      return;
    }
    try {
      floor_z = -tf_buffer_->lookupTransform(floor_frame_, base_frame_, tf2::TimePointZero).transform.translation.z;
    } catch (const tf2::TransformException &) {
      floor_z = -arm_base_height_;
      RCLCPP_WARN(get_logger(), "place: no TF %s -> %s, arm_base_height %.3f m", floor_frame_.c_str(),
        base_frame_.c_str(), arm_base_height_);
    }
    const double surface = has_z ? arm.point.z : floor_z + place_surface_height_;
    {   // 2026-11-11 처짐 보정: 칸을 J1 축 바깥으로 (a + b x (J1 거리 − 0.2665) / 0.06) mm
      const double r = std::hypot(arm.point.x, arm.point.y);
      if (r > 1e-6) {
        const double comp = (place_radial_a_ + place_radial_b_ * (r - 0.2665) / 0.06) / 1000.0;
        arm.point.x += arm.point.x / r * comp;
        arm.point.y += arm.point.y / r * comp;
      }
    }
    const Eigen::Vector3d tip(arm.point.x, arm.point.y, surface + puck_half_height_ + grasp_z_offset_ + place_clearance_);
    const auto current = currentJoints();
    if (!current) {
      finishSkill(false, "no recent /joint_states for arm joints");
      return;
    }
    pitch_ = place_pitch_;
    const Eigen::Vector3d dir = ArmIK::approachDirection(std::atan2(tip.y(), tip.x()), pitch_);
    const Eigen::Vector3d pre = tip - dir * place_approach_;
    const Eigen::Vector3d back = tip - dir * place_retreat_;
    const auto q_pre = solveArm(pre, pitch_, *current);
    if (!q_pre || !solveArm(tip, pitch_, *q_pre) || !solveArm(back, pitch_, *q_pre)) {
      char buf[160];
      std::snprintf(buf, sizeof(buf), "place: IK no solution for cell (%.3f, %.3f, %.3f) in %s, J1 distance %.3f m",
        tip.x(), tip.y(), tip.z(), base_frame_.c_str(), std::hypot(tip.x(), tip.y()));
      finishSkill(false, buf);
      return;
    }
    RCLCPP_INFO(get_logger(), "place: tip (%.3f, %.3f, %.3f) in %s", tip.x(), tip.y(), tip.z(), base_frame_.c_str());
    if (sag_enabled_ && place_sag_ && gripper_mode_ != GripperMode::ACTION) {
      runPlaceStream(*current, *q_pre, pre, tip, back);
      return;
    }
    sendArm(*q_pre, trajDuration(*current, *q_pre, min_duration_),
      [this, pre, tip, back](bool ok, const std::string & msg) {
        if (!ok) {
          finishSkill(false, "place approach: " + msg);
          return;
        }
        tool_pos_ = pre;
        moveTool(tip, [this, back]() {
            moveGripper(place_release_, [this, back](bool gok, const std::string & gmsg) {
                if (!gok) {
                  finishSkill(false, "place release: " + gmsg);
                  return;
                }
                moveTool(back, [this]() {toTransit("placed");});
              });
          });
      });
  }

  // 2026-11-11 놓기 처짐 보정 스트리밍 (palletStreamTick 공용). 처짐 보정 없이 놓으면 꺼내기 bias 가 빠지며 팔이 약 10 mm 처져
  //   퍽이 윗면을 누르고 넘어짐 (Isaac dice_place_check.py --unload: 꺼내기 → 놓기 판 전체 2.8 %, 이걸로 --place_sag ki).
  //   꺼내기 스트림이 끝 자세 (계획 + bias) 를 그대로 유지 중이면 그 bias 를 이어 쓰고, 아니면 sag.settle_time 먼저 맞춤.
  //   시간표: pre (max_joint_vel) → tip 직선 (moveTool 규칙) → 벌림 (gripper.move_time) → 머묾 (pallet.place_dwell) → back 직선 → transit
  void runPlaceStream(
    const std::vector<double> & current, const std::vector<double> & q_pre,
    const Eigen::Vector3d & pre, const Eigen::Vector3d & tip, const Eigen::Vector3d & back)
  {
    auto arm5 = [](const std::vector<double> & q) {Arm5 a{}; for (size_t j = 0; j < 5; ++j) {a[j] = q[j];} return a;};
    const double hold = gripper_close_;
    SagIntegrator integ(sag_ki_, sag_bias_max_);
    std::optional<JointTimeline> tlo;
    if (sag_end_plan_) {
      double d = 0.0;
      for (size_t j = 0; j < 5; ++j) {d = std::max(d, std::abs(current[j] - (*sag_end_plan_)[j]));}
      if (d < 0.02) {
        tlo.emplace(*sag_end_plan_, hold);
        integ.setBias(sag_end_bias_);
      }
    }
    sag_end_plan_.reset();
    const bool carried = tlo.has_value();
    if (!carried) {
      tlo.emplace(arm5(current), hold);
      tlo->hold(sag_settle_, hold);
    }
    JointTimeline & tl = *tlo;
    tl.moveThrough({arm5(q_pre)}, max_joint_vel_, hold, min_duration_);
    std::vector<double> seed = q_pre;
    auto line = [&](const Eigen::Vector3d & a, const Eigen::Vector3d & b, double grip) -> bool {
        const double length = (b - a).norm();
        const int n = std::max(1, static_cast<int>(std::ceil(length / cartesian_step_)));
        const double segment_min_time = (length / n) / cartesian_speed_;
        for (int i = 1; i <= n; ++i) {
          const auto q = solveArm(a + (b - a) * (static_cast<double>(i) / n), pitch_, seed);
          if (!q) {return false;}
          double max_delta = 0.0;
          for (size_t j = 0; j < 5; ++j) {max_delta = std::max(max_delta, std::abs((*q)[j] - seed[j]));}
          if (max_delta > max_waypoint_joint_step_) {return false;}
          tl.moveTo(arm5(*q), std::max(segment_min_time, max_delta / max_joint_vel_), grip);
          seed = *q;
        }
        return true;
      };
    if (!line(pre, tip, hold)) {
      finishSkill(false, "place: IK on straight path to tip");
      return;
    }
    tl.hold(gripper_move_time_, place_release_);   // 벌림
    tl.hold(pallet_dwell_, place_release_);        // 머묾
    if (!line(tip, back, place_release_)) {
      finishSkill(false, "place: IK on straight path back");
      return;
    }
    if (transit_tilt_.size() == 5) {tl.moveThrough({arm5(transit_tilt_)}, max_joint_vel_, place_release_, min_duration_);}
    sag_tl_ = tl;
    sag_int_ = integ;
    sag_check_t_.reset();
    sag_done_msg_ = "placed";
    sag_start_ = sag_last_ = now();
    RCLCPP_INFO(get_logger(), "place: stream %.1f s (%s, ki %.1f, %.0f Hz)", tl.duration(),
      carried ? "bias carried" : "settle", sag_ki_, sag_rate_);
    const uint64_t id = job_id_;
    skill_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / sag_rate_),
      [this, id]() {if (id == job_id_) {palletStreamTick();}}, job_cb_group_);
  }

  // ------------------------------------------------------------------ box_look · barcode_look (#65)
  // <name>.view_angles_deg · distances · min_clearance (카메라 장착은 target_look.*)
  TargetLookParams lookParams(
    const std::string & name, const std::vector<double> & angles_deg, const std::vector<double> & distances,
    double min_clearance)
  {
    TargetLookParams lp = target_look_;
    lp.view_angles.clear();
    for (double d : declare_parameter<std::vector<double>>(name + ".view_angles_deg", angles_deg)) {
      lp.view_angles.push_back(d * M_PI / 180.0);
    }
    lp.distances = declare_parameter<std::vector<double>>(name + ".distances", distances);
    lp.min_clearance = declare_parameter<double>(name + ".min_clearance", min_clearance);
    return lp;
  }

  // 보는 점 (t.frame 의 x, y, z) 을 카메라 광축이 지나는 자세 중 관절 한계 안의 첫 후보로 이동하고 끝 (재는 것은 perception)
  void startLook(const TaskSkill & t)
  {
    step_ = PickStep::TASK;
    object_yaw_valid_ = false;   // solveArm 손목 roll = 기본값
    geometry_msgs::msg::PointStamped src, arm;
    src.header.frame_id = t.frame;
    src.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    src.point.x = t.x;
    src.point.y = t.y;
    src.point.z = t.z;
    try {
      arm = tf_buffer_->transform(src, base_frame_);
    } catch (const tf2::TransformException & e) {
      finishSkill(false, t.look + ": TF " + t.frame + " -> " + base_frame_ + ": " + e.what());
      return;
    }
    const Eigen::Vector3d aim(arm.point.x, arm.point.y, arm.point.z);
    const auto current = currentJoints();
    if (!current) {
      finishSkill(false, "no recent /joint_states for arm joints");
      return;
    }
    std::optional<std::vector<double>> q;
    TargetLookCandidate used{};
    for (const auto & c : targetLookCandidates(aim, look_params_.at(t.look))) {
      q = solveArm(c.tool, c.pitch, *current);
      if (q) {
        used = c;
        break;
      }
    }
    char buf[200];
    if (!q) {
      std::snprintf(buf, sizeof(buf), "%s: no pose within joint limits for point (%.3f, %.3f, %.3f) in %s, J1 distance %.3f m",
        t.look.c_str(), aim.x(), aim.y(), aim.z(), base_frame_.c_str(), std::hypot(aim.x(), aim.y()));
      finishSkill(false, buf);
      return;
    }
    std::snprintf(buf, sizeof(buf), "%s: camera %.0f deg down at %.2f m from (%.3f, %.3f, %.3f) in %s", t.look.c_str(),
      used.view_angle * 180.0 / M_PI, used.distance, aim.x(), aim.y(), aim.z(), base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "%s", buf);
    const std::string done = buf;
    sendArm(*q, trajDuration(*current, *q, min_duration_), [this, done](bool ok, const std::string & msg) {
        finishSkill(ok, ok ? done : "look move: " + msg);
      });
  }

  // ------------------------------------------------------------------ target_look (#82)
  // 퍽 예상 중심 → 팔 카메라가 퍽 윗면을 내려다보는 자세 (target_look_pose.hpp) → TARGET_LOOK_WAIT (targetLookTick 이 /perception/pucks 로 잼)
  void startTargetLook(const TaskSkill & t)
  {
    step_ = PickStep::TASK;
    target_look_latch_.reset();
    object_yaw_valid_ = false;   // solveArm 손목 roll = 기본값
    geometry_msgs::msg::PointStamped src, arm;
    src.header.frame_id = t.frame;
    src.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    src.point.x = t.x;
    src.point.y = t.y;
    src.point.z = t.z;
    try {
      arm = tf_buffer_->transform(src, base_frame_);
    } catch (const tf2::TransformException & e) {
      finishSkill(false, std::string("target_look: TF ") + t.frame + " -> " + base_frame_ + ": " + e.what());
      return;
    }
    target_look_expected_ = {arm.point.x, arm.point.y, arm.point.z};
    const Eigen::Vector3d aim = target_look_expected_ + Eigen::Vector3d::UnitZ() * puck_half_height_;   // 퍽 윗면 (태그)
    const auto current = currentJoints();
    if (!current) {
      finishSkill(false, "no recent /joint_states for arm joints");
      return;
    }
    std::optional<std::vector<double>> q;
    TargetLookCandidate used{};
    for (const auto & c : targetLookCandidates(aim, target_look_)) {
      q = solveArm(c.tool, c.pitch, *current);
      if (q) {
        used = c;
        break;
      }
    }
    if (!q) {
      char buf[160];
      std::snprintf(buf, sizeof(buf), "target_look: no target check pose within joint limits for puck (%.3f, %.3f, %.3f) in %s, "
        "J1 distance %.3f m", aim.x(), aim.y(), target_look_expected_.z(), base_frame_.c_str(), std::hypot(aim.x(), aim.y()));
      finishSkill(false, buf);
      return;
    }
    RCLCPP_INFO(get_logger(), "target_look: puck (%.3f, %.3f, %.3f) in %s, camera %.0f deg down at %.2f m",
      target_look_expected_.x(), target_look_expected_.y(), target_look_expected_.z(), base_frame_.c_str(), used.view_angle * 180.0 / M_PI,
      used.distance);
    sendArm(*q, trajDuration(*current, *q, min_duration_), [this](bool ok, const std::string & msg) {
        if (!ok) {
          finishSkill(false, "target_look move: " + msg);
          return;
        }
        step_ = PickStep::TARGET_LOOK_WAIT;
        wait_since_ = now();
      });
  }

  // 타겟체크자세에서 settle_time 뒤 촬영된 /perception/pucks 중 예상 자리 (수평 match_distance 안) 퍽.
  // 팔 카메라 퍽 우선, arm_wait 지나면 D435 퍽도. 찾으면 fixed_frame 에 붙잡고 성공, timeout 지나면 실패
  void targetLookTick()
  {
    const rclcpp::Time t = now();
    const double waited = (t - wait_since_).seconds();
    if (waited < target_look_settle_) {return;}
    PuckArray::ConstSharedPtr pucks;
    {
      std::lock_guard<std::mutex> lock(sensor_mutex_);
      const rclcpp::Time since = std::max(wait_since_ + rclcpp::Duration::from_seconds(target_look_settle_),
        t - rclcpp::Duration::from_seconds(stale_timeout_));
      if (last_pucks_ && measuredAfter(last_pucks_->header.stamp, since)) {pucks = last_pucks_;}
    }
    if (pucks && !pucks->pucks.empty()) {
      // 촬영 시각의 퍽 frame → 지금 팔 기준 · fixed_frame (측정 시각 TF 가 아직 없으면 최신)
      auto lookup = [this, &pucks](const std::string & target, bool via_fixed) {
          try {
            return via_fixed ?
                   tf_buffer_->lookupTransform(target, tf2::TimePointZero, pucks->header.frame_id,
                     tf2_ros::fromMsg(pucks->header.stamp), fixed_frame_, tf2::durationFromSec(0.0)) :
                   tf_buffer_->lookupTransform(target, pucks->header.frame_id, tf2_ros::fromMsg(pucks->header.stamp));
          } catch (const tf2::TransformException &) {
            return tf_buffer_->lookupTransform(target, pucks->header.frame_id, tf2::TimePointZero);
          }
        };
      try {
        const auto to_arm = lookup(base_frame_, true);
        const auto to_fixed = lookup(fixed_frame_, false);
        const bool any_camera = waited >= target_look_settle_ + target_look_arm_wait_;
        const Puck * best = nullptr;
        double best_d = target_look_match_;
        for (const auto & pk : pucks->pucks) {
          if (!any_camera && pk.camera != Puck::CAMERA_ARM) {continue;}
          geometry_msgs::msg::Pose a;
          tf2::doTransform(pk.pose, a, to_arm);
          const double d = std::hypot(a.position.x - target_look_expected_.x(), a.position.y - target_look_expected_.y());
          const bool better = !best || (pk.camera == Puck::CAMERA_ARM && best->camera != Puck::CAMERA_ARM) ||
            ((pk.camera == Puck::CAMERA_ARM) == (best->camera == Puck::CAMERA_ARM) && d < best_d);
          if (d <= target_look_match_ && better) {
            best = &pk;
            best_d = d;
          }
        }
        if (best) {
          geometry_msgs::msg::PoseStamped latch;
          latch.header.frame_id = fixed_frame_;
          latch.header.stamp = pucks->header.stamp;
          tf2::doTransform(best->pose, latch.pose, to_fixed);
          latch.pose.orientation = geometry_msgs::msg::Quaternion();
          latch.pose.orientation.w = 0.0;   // 방향 없음 (resolveTarget 이 yaw 를 쓰지 않음)
          target_look_latch_ = latch;
          target_look_latch_time_ = t;
          char buf[200];
          std::snprintf(buf, sizeof(buf), "puck seen by %s (tag %d) at (%.3f, %.3f, %.3f) %s, %.3f m from expected",
            best->camera == Puck::CAMERA_ARM ? "arm camera" : "D435", best->tag_id, latch.pose.position.x,
            latch.pose.position.y, latch.pose.position.z, fixed_frame_.c_str(), best_d);
          RCLCPP_INFO(get_logger(), "target_look: %s", buf);
          finishSkill(true, buf);
          return;
        }
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "target_look: TF %s", e.what());
      }
    }
    if (waited > target_look_settle_ + target_look_timeout_) {
      char buf[200];
      std::snprintf(buf, sizeof(buf), "target_look: no puck within %.3f m of expected (%.3f, %.3f) in %s for %.1f s",
        target_look_match_, target_look_expected_.x(), target_look_expected_.y(), base_frame_.c_str(), target_look_timeout_);
      finishSkill(false, buf);
    }
  }

  // 다음 팔레트 꺼내기가 같은 자리에서 시작하게 이동 자세 (팔레트 궤적 transit q_front_tilt) 로
  void toTransit(const std::string & message)
  {
    const auto current = currentJoints();
    if (transit_tilt_.size() != 5 || !current) {
      finishSkill(true, message);
      return;
    }
    std::vector<double> q(transit_tilt_.begin(), transit_tilt_.end());
    q.push_back(sixthValue(*current));
    if (!withinLimits(q)) {
      finishSkill(true, message);
      return;
    }
    sendArm(q, trajDuration(*current, q, min_duration_), [this, message](bool ok, const std::string & msg) {
        finishSkill(ok, ok ? message : "transit: " + msg);
      });
  }

  // ------------------------------------------------------------------ 멤버
  // 파라미터
  std::vector<std::string> joint_names_;
  std::vector<double> lower_, upper_;
  double max_joint_vel_{0.5}, min_duration_{1.0}, joint_state_timeout_{1.0};
  std::unique_ptr<ArmIK> ik_;
  std::vector<double> approach_pitches_;
  bool allow_elbow_down_{false}, use_object_yaw_{false};
  double wrist_roll_default_{0.0}, gripper_roll_default_{0.0};
  std::map<std::string, std::vector<double>> poses_;
  GripperMode gripper_mode_{GripperMode::ARM_JOINT};
  std::string gripper_mode_name_{"arm_joint"};
  std::string gripper_joint_;
  double gripper_open_{-0.764}, gripper_close_{0.923}, gripper_effort_{0.0};
  double gripper_move_time_{1.0}, gripper_settle_{0.4}, grasp_margin_{0.15};
  bool gripper_fail_on_empty_{true};
  double toppick_pitch_{-M_PI_4}, toppick_approach_{0.06}, grasp_z_offset_{0.015};
  bool toppick_use_alignment_{false};
  double toppick_latch_max_age_{30.0};
  bool toppick_top_{true}, pick_top_{false};   // toppick.approach_mode top · 지금 집기가 top 경로인지
  double toppick_approach_height_{0.05}, toppick_approach_inward_{0.03}, toppick_radial_offset_{0.002};
  TargetLookParams target_look_;
  std::map<std::string, TargetLookParams> look_params_;   // box_look · barcode_look (#65)
  double target_look_settle_{0.3}, target_look_arm_wait_{1.0}, target_look_timeout_{3.0}, target_look_match_{0.04};
  std::string pallet_file_;
  double pallet_speed_{0.8}, pallet_row3_hold_speed_{0.4};
  bool pallet_require_held_{true};
  std::map<std::pair<int, int>, std::vector<PalletSegment>> pallet_;   // (열, 칸) → 구간
  std::map<std::pair<int, int>, std::vector<double>> pallet_front_;    // (열, 칸) → 앞 운반 자세 q_front (5 관절)
  double pallet_release_{-0.764}, pallet_dwell_{0.3};
  bool sag_enabled_{true};
  double sag_ki_{4.0}, sag_bias_max_{0.15}, sag_settle_{1.5}, sag_rate_{50.0}, sag_horizon_{0.06}, sag_max_error_{0.3};
  std::optional<JointTimeline> sag_tl_;            // 팔레트 스트리밍 시간표 (job_cb_group_ 전용)
  SagIntegrator sag_int_{4.0, 0.15};
  std::optional<double> sag_check_t_;              // 꺼내기 쥠 확인 시각
  bool sag_load_{true};
  std::string sag_done_msg_{"unloaded"};           // 스트림 성공 메시지 (loaded · unloaded · placed)
  std::optional<Arm5> sag_end_plan_;               // 마지막 스트림 끝 계획 자세 (그 bias 로 유지 중일 때만)
  Arm5 sag_end_bias_{};
  bool place_sag_{true};
  rclcpp::Time sag_start_{0, 0, RCL_ROS_TIME}, sag_last_{0, 0, RCL_ROS_TIME};
  std::vector<double> transit_tilt_;                                   // 팔레트 궤적 이동 자세 (5 관절)
  std::string place_cells_file_, place_cells_frame_, floor_frame_;
  std::map<std::string, std::pair<double, double>> place_cells_;      // "L1/-1,0" → (x, y)
  double place_pitch_{-M_PI_4}, place_surface_height_{0.070}, puck_half_height_{0.030}, place_clearance_{0.002};
  double place_approach_{0.04}, place_retreat_{0.04}, arm_base_height_{0.120};
  double place_release_{0.0}, place_radial_a_{0.0}, place_radial_b_{0.0};   // 2026-11-11 놓기 벌림 폭 · J1 바깥 보정
  std::string base_frame_, fixed_frame_, range_frame_;
  double approach_distance_{0.15}, range_offset_{0.015}, max_final_approach_{0.25};
  double range_mismatch_tol_{0.04};
  double lift_height_{0.05};
  double cartesian_step_{0.01}, cartesian_speed_{0.05}, max_waypoint_joint_step_{0.2};
  double stale_timeout_{0.3};
  double offset_tolerance_{0.01}, object_radius_{0.015};
  bool skill_enabled_{false};
  std::vector<std::string> skill_names_;
  std::string skill_file_;
  double skill_rate_{20.0}, skill_horizon_{1.5}, skill_max_joint_vel_{0.5};
  std::optional<std::vector<double>> skill_last_cmd_;   // 직전 정책 목표 (변화량 제한 기준)
  robot_policy::PolicyLibrary policies_;
  std::set<std::string> skill_ready_;   // 로드 · 형식 확인을 통과한 정책만 (없으면 execute_skill 거절)
  robot_policy::LoadedPolicy * active_policy_{nullptr};
  // 강화학습 집기 (tcp_delta)
  struct TopPickRun
  {
    Eigen::Vector3d puck{Eigen::Vector3d::Zero()};   // fixed_frame_ (시작 때 고정)
    std::optional<Eigen::Vector3d> neighbor;         // fixed_frame_
    bool is_box{false};
    std::array<float, 4> prev_action{0.f, 0.f, 0.f, 1.f};   // 학습 리셋 값
    std::optional<Joints5> target;                   // 직전 IK 관절 목표 (모델 값)
    std::optional<Joints5> q_prev;                   // 속도 차분용
    rclcpp::Time t_prev{0, 0, RCL_ROS_TIME};
    double hold{0.0};                                // 성공 조건 유지 시간 (s)
    rclcpp::Time t_start{0, 0, RCL_ROS_TIME};        // 정책 시작 (시작 자세 도착) 시각
  };
  std::set<std::string> tcp_skills_;
  struct TcpPolicyCfg   // 정책마다 다른 값 (<이름>.obs_ref_pose · neighbor_absent · start_pose · closed_m)
  {
    TopPickLayout layout;
    std::vector<double> start_pose;
    double closed_m{-0.0075};
  };
  std::map<std::string, TcpPolicyCfg> tcp_cfg_;
  TcpPolicyCfg tcp_active_;
  std::string toppick_tilt_policy_;
  std::optional<TopPickRun> rl_;
  // 누운 직육면체 세워 놓기
  std::optional<LieRun> lie_;
  std::unique_ptr<robot_policy::LoadedPolicy> lie_policy_;
  LieUprightParams lie_params_;
  bool lie_enabled_{true}, lie_box_required_{true}, lie_look_after_{true}, lie_vel_scale_{true};
  std::string lie_policy_name_{"lie_upright"}, lie_file_, lie_floor_frame_{"base_footprint"};
  std::vector<double> lie_view_left_, lie_view_right_, lie_view_inline_;
  double lie_box_top_{0.07}, lie_puck_dz_{0.015}, lie_spot_dz_{0.029}, lie_side_y_{0.05}, lie_box_settle_{0.5}, lie_box_timeout_{3.0};
  double lie_pucks_age_{2.0}, lie_max_duration_{15.0}, lie_lift_ok_{0.03}, lie_clear_dist_{0.03}, lie_clear_hold_{1.0};
  double lie_max_joint_vel_{3.0}, lie_release_xy_{0.03}, lie_release_z_{0.06}, lie_release_beta_{1.4}, lie_release_center_offset_{0.011}, lie_retreat_up_{0.05}, lie_retreat_time_{1.0};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lie_prior_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr lie_box_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr lie_box_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lie_status_sub_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr lie_box_;   // sensor_mutex_
  std_msgs::msg::String::ConstSharedPtr lie_status_;
  rclcpp::Time lie_box_rx_{0, 0, RCL_ROS_TIME}, lie_status_rx_{0, 0, RCL_ROS_TIME};
  std::optional<std::pair<uint64_t, std::string>> lie_req_;   // 측정 요청 응답 (작업 id, 거절 이유 · 빈 값 = 받음)
  ArmDiffIK diff_ik_;
  GripperMap rl_grip_;
  std::array<double, 5> ik_signs_{1.0, 1.0, 1.0, 1.0, 1.0}, ik_offsets_{};
  std::string rl_frame_{"base_link"};
  double rl_open_m_{0.010}, rl_closed_m_{-0.0075}, rl_grasp_lo_{-0.010}, rl_grasp_hi_{-0.003}, rl_grasp_dz_{0.015};
  double rl_success_lift_{0.03}, rl_success_hold_{1.0}, rl_nb_match_{0.12}, rl_self_match_{0.03}, rl_max_joint_vel_{2.0};
  bool rl_default_box_{false};
  std::vector<double> rl_start_pose_;
  double rl_max_duration_{12.0};
  bool correction_enabled_{true}, correction_required_{true};
  int correction_max_iter_{5};
  double offset_gain_{1.0}, lateral_tol_{0.005}, range_tol_{0.01}, target_range_{0.13};
  double max_step_{0.02}, correction_min_duration_{0.3}, settle_time_{0.2}, wait_timeout_{1.0};
  std::string startup_pose_{"DRIVE"};                 // 빈 문자열 = 켜질 때 정리 끔
  std::vector<std::vector<size_t>> startup_stages_;   // 관절 칸 묶음 순서
  double startup_vel_{0.1}, startup_skip_tol_{0.05}, startup_diag_wait_{5.0}, startup_warn_after_{15.0};

  // ROS 인터페이스
  rclcpp::CallbackGroup::SharedPtr job_cb_group_, sensor_cb_group_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<PerceptionResult>::SharedPtr result_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<PuckArray>::SharedPtr pucks_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp_action::Client<FollowJT>::SharedPtr arm_client_;
  rclcpp_action::Client<GripperCmd>::SharedPtr gripper_client_;
  rclcpp_action::Server<ArmPose>::SharedPtr pose_server_;
  rclcpp_action::Server<Pick>::SharedPtr pick_server_;
  rclcpp_action::Server<ExecuteSkill>::SharedPtr skill_server_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr jt_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  // 센서 캐시 (sensor_cb_group_ 쓰기, job_cb_group_ 읽기)
  std::mutex sensor_mutex_;
  PerceptionResult::ConstSharedPtr last_result_;
  sensor_msgs::msg::Range::ConstSharedPtr last_range_;
  PuckArray::ConstSharedPtr last_pucks_;
  std::map<std::string, double> joint_pos_;
  std::map<std::string, double> joint_vel_;   // /joint_states velocity (있을 때, 강화학습 집기 관측)
  rclcpp::Time joint_rx_{0, 0, RCL_ROS_TIME};
  bool diag_arm_hold_{false}, diag_estop_{false};   // opencr 진단 (onDiagnostics)
  rclcpp::Time diag_rx_{0, 0, RCL_ROS_TIME};        // 마지막 opencr/motors 수신 (0 = 아직 없음)

  // 작업 상태 (job_cb_group_ 전용)
  Job job_{Job::NONE};
  Task task_{Task::NONE};
  uint64_t job_id_{0};
  std::shared_ptr<PoseGoalHandle> pose_gh_;
  std::shared_ptr<PickGoalHandle> pick_gh_;
  std::shared_ptr<SkillGoalHandle> skill_gh_;
  rclcpp::TimerBase::SharedPtr skill_timer_;
  rclcpp::Time skill_start_{0, 0, RCL_ROS_TIME};
  double skill_timeout_{30.0};
  bool grasped_{false}, gripper_stalled_{false};
  std::optional<double> gripper_cmd_;                 // arm_joint 모드의 지금 그리퍼 명령 (6 번째 값)
  std::vector<double> last_sent_;                     // 마지막으로 보낸 6관절 목표
  std::function<void()> gripper_wait_;                // 그리퍼 판정 대기 (onTick 이 settle_time 뒤 부름)
  std::string pose_name_;
  Eigen::Vector3d target_look_expected_{Eigen::Vector3d::Zero()};   // target_look 퍽 예상 중심 (base_frame_)
  std::optional<geometry_msgs::msg::PoseStamped> target_look_latch_;  // target_look 이 잰 퍽 중심 (fixed_frame_)
  rclcpp::Time target_look_latch_time_{0, 0, RCL_ROS_TIME};
  rclcpp_action::ClientGoalHandle<FollowJT>::SharedPtr arm_goal_;
  rclcpp_action::ClientGoalHandle<GripperCmd>::SharedPtr gripper_goal_;
  PickStep step_{PickStep::PLAN};
  Eigen::Vector3d tool_pos_{Eigen::Vector3d::Zero()};   // 현재 끝단 목표 위치
  Eigen::Vector3d grasp_pos_{Eigen::Vector3d::Zero()};  // 계획한 파지점
  std::optional<double> final_grasp_dist_;              // 보정 종료 시점 측정 기반 파지점 거리
  double pitch_{0.0};
  double object_yaw_{0.0};
  bool object_yaw_valid_{false};
  int correction_iter_{0};
  rclcpp::Time wait_since_{0, 0, RCL_ROS_TIME};
  bool stream_active_{false};                         // 토픽 스트리밍 중 (끊기면 endJob 이 측정 자세로 다시 잡음)
  bool startup_done_{false};                          // 켜질 때 정리를 마쳤거나 goal 을 받아 건너뜀
  size_t startup_stage_{0};
  rclcpp::Time startup_wait_since_{0, 0, RCL_ROS_TIME}, startup_diag_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time startup_retry_at_{0, 0, RCL_ROS_TIME};
};

}  // namespace control

RCLCPP_COMPONENTS_REGISTER_NODE(control::ManipulatorNode)
