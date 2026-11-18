// LocMonitorNode — 위치 잃음 감지 · 다시 찾기 (localization/loc_monitor.py 를 C++ 로 옮김, 동작 같음). 계산은 loc_core.
// 왜 옮겼나: Python 판이 Jetson 실물 정지 상태에서 CPU 코어 하나의 약 20~25 % (11-13 #265 · 11-14 P7).
// 이 판은 backend amcl 만 한다(다시 넣기 = /initialpose). carto 는 Python 판(localization loc_monitor)을 쓴다
//   — Jetson 에는 cartographer_ros_msgs 가 없음.
// 상태(/localization/status, String, Reliable · TRANSIENT_LOCAL depth 1, 10 Hz · 바뀔 때): OK · SUSPECT · LOST · RECOVERING · CONFIRM · FAILED.
// /localization/lock(Bool, 같은 QoS): OK · SUSPECT 가 아니면 true.
// 감지: ① 스캔–지도 잔차 > res_thr 가 suspect_s 이어지면 SUSPECT, lost_s 이어지면 LOST
//       ② /amcl_pose 증분과 odom 증분 차 > jump_xy 또는 jump_yaw → LOST ③ /amcl_pose 가 gap_s 넘게 끊김 → LOST
//       (map → odom TF 시각이 계속 바뀌는 동안은 끊김으로 보지 않음 — amcl 은 odom 이 안 바뀌면 pose 를 안 냄).
// 대응: OK · SUSPECT 가 아니면 명령 게이트(gate_in → gate_out, none 이면 끔)가 속도 0. LOST → RECOVERING(작업 스레드):
//       마지막 정상 자세 둘레 탐색 → 안 되면 지도 전체 탐색 → 받아들이면 /initialpose → CONFIRM → 잔차가 ok_confirm_s 동안 좋으면 OK.
//       max_recover_s 넘으면 FAILED. 사건은 event_log(jsonl)에도 씀(Python 판과 같은 이름 · 같은 키).
// FAILED 에서 나오기 (2026-11-17, 11-16 실물: FAILED 뒤 bringup 을 다시 띄우는 수밖에 없었음):
//   ① failed_retry_s > 0 이면 FAILED 뒤에도 그 간격으로 탐색을 이어 가고, 받아들이면 CONFIRM (OK 는 예전처럼 잔차로만)
//   ② 운용자가 RViz '2D Pose Estimate' 등으로 /initialpose 를 내면 (이 노드가 낸 것 말고) 어느 상태든 CONFIRM 에서 다시 확인
//   failed_retry_s ≤ 0 · manual_initialpose false 면 예전과 같음.
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "perception/loc_core.hpp"

namespace perception
{
namespace L = perception::loc;
using LaserScan = sensor_msgs::msg::LaserScan;
using PoseCov = geometry_msgs::msg::PoseWithCovarianceStamped;
using TwistStamped = geometry_msgs::msg::TwistStamped;

namespace
{
// 사건 기록용 작은 JSON (Python json.dumps 와 같은 키 순서 · 짧은 실수 표기)
std::string num(double v)
{
  if (!std::isfinite(v)) {return std::isnan(v) ? "NaN" : (v > 0 ? "Infinity" : "-Infinity");}
  char b[32];
  auto r = std::to_chars(b, b + sizeof(b), v);
  std::string s(b, r.ptr);
  if (s.find_first_of(".en") == std::string::npos) {s += ".0";}   // 파이썬 실수는 1.0 처럼 씀
  return s;
}
double rnd(double v, int k) {const double m = std::pow(10.0, k); return std::round(v * m) / m;}
std::string str(const std::string & s) {return "\"" + s + "\"";}
std::string arr(const L::Pose & p, int k)
{
  return "[" + num(rnd(p[0], k)) + ", " + num(rnd(p[1], k)) + ", " + num(rnd(p[2], k)) + "]";
}
using KV = std::vector<std::pair<std::string, std::string>>;   // 값은 이미 JSON 표기

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
L::Pose tf_pose(const geometry_msgs::msg::TransformStamped & t)
{
  return {t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)};
}
bool ok_state(const std::string & s) {return s == "OK" || s == "SUSPECT";}
}  // namespace

class LocMonitorNode : public rclcpp::Node
{
public:
  explicit LocMonitorNode(const rclcpp::NodeOptions & options)
  : Node("loc_monitor", options)
  {
    backend_ = declare_parameter("backend", std::string("amcl"));
    auto pose_topic = declare_parameter("pose_topic", std::string(""));
    const auto map = declare_parameter("map", std::string(""));
    mission_ = declare_parameter("mission_frame", std::string("map"));
    odom_f_ = declare_parameter("odom_frame", std::string("odom"));
    base_ = declare_parameter("base_frame", std::string("base_link"));
    topics_ = declare_parameter("scan_topics", std::vector<std::string>{"/scan_front_filtered", "/scan_rear_filtered"});
    r_min_ = declare_parameter("r_min", 0.15);
    r_max_ = declare_parameter("r_max", 3.5);
    n_points_ = static_cast<int>(declare_parameter("n_points", 90));
    res_r_max_ = declare_parameter("res_r_max", 2.0);
    grace_ = declare_parameter("startup_grace_s", 15.0);
    tf_wait_ = declare_parameter("tf_wait_s", 0.3);
    res_mode_ = declare_parameter("res_mode", std::string("cell_median"));
    check_mode_ = declare_parameter("check_mode", std::string("front"));
    res_thr_ = declare_parameter("res_thr", 0.03);
    suspect_s_ = declare_parameter("suspect_s", 0.5);
    lost_s_ = declare_parameter("lost_s", 2.0);
    jump_xy_ = declare_parameter("jump_xy", 0.10);
    jump_yaw_deg_ = declare_parameter("jump_yaw_deg", 5.0);
    gap_s_ = declare_parameter("gap_s", 0.5);
    const auto gate_in = declare_parameter("gate_in", std::string("/cmd_vel_gate_in"));
    const auto gate_out = declare_parameter("gate_out", std::string("/cmd_vel_nav"));
    xy_sd_ = declare_parameter("initialpose_xy_sd", 0.05);
    yaw_sd_deg_ = declare_parameter("initialpose_yaw_sd_deg", 5.0);
    // carto 용 인자(같은 launch 에서 받으므로 선언만): 이 판은 쓰지 않음
    declare_parameter("carto_config_dir", std::string(""));
    declare_parameter("carto_config", std::string("carto_localization.lua"));
    declare_parameter("frozen_trajectory_id", 0);
    traj_ = declare_parameter("trajectory_id", 1);
    declare_parameter("align_x", 0.0);
    declare_parameter("align_y", 0.0);
    declare_parameter("align_yaw", 0.0);
    accept_score_ = declare_parameter("accept_score", 0.02);
    amb_margin_ = declare_parameter("ambiguity_margin", 0.005);
    local_lin_ = declare_parameter("local_lin", 0.5);
    local_ang_deg_ = declare_parameter("local_ang_deg", 30.0);
    retry_s_ = declare_parameter("retry_s", 2.0);
    max_recover_s_ = declare_parameter("max_recover_s", 60.0);
    ok_confirm_s_ = declare_parameter("ok_confirm_s", 1.0);
    confirm_timeout_s_ = declare_parameter("confirm_timeout_s", 10.0);
    failed_retry_s_ = declare_parameter("failed_retry_s", 5.0);
    const bool manual_init = declare_parameter("manual_initialpose", true);
    const auto event_log = declare_parameter("event_log", std::string(""));
    if (backend_ != "amcl") {
      throw std::invalid_argument("C++ loc_monitor 는 backend amcl 만 — carto 는 localization 패키지 Python 판(loc_cpp:=false)");
    }
    if (topics_.size() < 2) {throw std::invalid_argument("scan_topics 는 앞 · 뒤 두 개");}
    field_ = std::make_unique<L::DistField>(map);
    buf_ = std::make_shared<tf2_ros::Buffer>(get_clock(), tf2::durationFromSec(30.0));
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buf_, this, true);   // 따로 스레드에서 받음(Python spin_thread=True)
    state_ = "OK";
    state_t_ = now_s();
    if (!event_log.empty() && event_log != "none") {log_f_.open(event_log, std::ios::app);}
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    pub_status_ = create_publisher<std_msgs::msg::String>("/localization/status", latched);
    pub_lock_ = create_publisher<std_msgs::msg::Bool>("/localization/lock", latched);
    gate_ = !gate_in.empty() && gate_in != "none";
    if (gate_) {
      pub_cmd_ = create_publisher<TwistStamped>(gate_out, 10);
      sub_cmd_ = create_subscription<TwistStamped>(gate_in, 10, [this](TwistStamped::UniquePtr m) {on_cmd(std::move(m));});
      timers_.push_back(rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(50), [this] {gate_zero();}));
    }
    for (const auto & tp : topics_) {
      scan_subs_.push_back(create_subscription<LaserScan>(tp, rclcpp::SensorDataQoS(),
          [this, tp](LaserScan::ConstSharedPtr m) {on_scan(tp, *m);}));
    }
    if (pose_topic.empty()) {pose_topic = "/amcl_pose";}
    sub_pose_ = create_subscription<PoseCov>(pose_topic, 10, [this](PoseCov::ConstSharedPtr m) {on_tracked(*m);});
    pub_init_ = create_publisher<PoseCov>("/initialpose", 10);
    if (manual_init) {
      sub_init_ = create_subscription<PoseCov>("/initialpose", 10, [this](PoseCov::ConstSharedPtr m) {on_initialpose(*m);});
    }
    timers_.push_back(rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100), [this] {tick();}));
    timers_.push_back(rclcpp::create_timer(this, get_clock(), std::chrono::seconds(10), [this] {report();}));
    event("start", {{"backend", str(backend_)}, {"gate", gate_ ? "true" : "false"}, {"traj", std::to_string(traj_)},
      {"res_thr", num(res_thr_)}, {"res_mode", str(res_mode_)}, {"check_mode", str(check_mode_)}});
  }

  ~LocMonitorNode() override
  {
    {
      std::lock_guard<std::mutex> g(stop_mu_);
      stop_ = true;
    }
    stop_cv_.notify_all();
    if (worker_.joinable()) {worker_.join();}
  }

private:
  struct Scan {rclcpp::Time st; std::vector<L::P2> pts, near;};
  using Pair = std::vector<std::pair<rclcpp::Time, std::vector<L::P2>>>;

  // ---- 공통 ----
  double now_s() {return get_clock()->now().nanoseconds() * 1e-9;}

  void event(const std::string & kind, const KV & kw = {})
  {
    std::string st;
    {
      std::lock_guard<std::mutex> g(mu_);
      st = state_;
    }
    std::string s = "{\"t\": " + num(rnd(now_s(), 3)) + ", \"event\": " + str(kind) + ", \"state\": " + str(st);
    for (const auto & [k, v] : kw) {s += ", \"" + k + "\": " + v;}
    s += "}";
    RCLCPP_INFO(get_logger(), "%s", s.c_str());
    if (log_f_.is_open()) {
      std::lock_guard<std::mutex> g(log_mu_);
      log_f_ << s << "\n";
      log_f_.flush();
    }
  }

  void set_state(const std::string & s, const KV & kw = {})
  {
    std::string old;
    {
      std::lock_guard<std::mutex> g(mu_);
      if (s == state_) {return;}
      old = state_;
      state_ = s;
      state_t_ = now_s();
    }
    KV k2{{"old", str(old)}, {"new", str(s)}};
    k2.insert(k2.end(), kw.begin(), kw.end());
    event("state", k2);
    publish_state(s);
  }

  void publish_state(const std::string & s)
  {
    std_msgs::msg::String m;
    m.data = s;
    pub_status_->publish(m);
    std_msgs::msg::Bool b;
    b.data = !ok_state(s);
    pub_lock_->publish(b);
  }

  std::string state()
  {
    std::lock_guard<std::mutex> g(mu_);
    return state_;
  }

  // 콜백 안에서는 기다리지 않는다(timeout 0). 다시 찾기 작업 스레드만 timeout 을 준다
  std::optional<L::Pose> lookup(const std::string & parent, const std::string & child, const rclcpp::Time & st, double timeout = 0.0)
  {
    try {
      if (timeout > 0) {
        return tf_pose(buf_->lookupTransform(parent, child, st, rclcpp::Duration::from_seconds(timeout)));
      }
      return tf_pose(buf_->lookupTransform(parent, child, st));
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  // ---- 명령 게이트 ----
  void on_cmd(TwistStamped::UniquePtr m)
  {
    if (!ok_state(state())) {m->twist.linear.x = m->twist.linear.y = m->twist.angular.z = 0.0;}
    pub_cmd_->publish(std::move(m));
  }

  void gate_zero()
  {
    if (ok_state(state())) {return;}
    TwistStamped z;
    z.header.stamp = get_clock()->now();
    z.header.frame_id = base_;
    pub_cmd_->publish(z);
  }

  // ---- 감지 ----
  void on_scan(const std::string & topic, const LaserScan & m)
  {
    if (!T_laser_.count(topic)) {   // 정적 · 3차원(라이다 roll π 장착) → 2×3. 아직 없으면 다음 스캔 때 다시
      try {
        const auto t = buf_->lookupTransform(base_, m.header.frame_id, tf2::TimePointZero).transform;
        T_laser_[topic] = L::T2d_from_tf(t.translation.x, t.translation.y, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w);
      } catch (const tf2::TransformException &) {
        return;
      }
    }
    const rclcpp::Time st(m.header.stamp, get_clock()->get_clock_type());
    const auto & T = T_laser_[topic];
    auto pts = L::scan_points(m.ranges, m.angle_min, m.angle_increment, T, r_min_, r_max_, n_points_);
    auto near = L::scan_points(m.ranges, m.angle_min, m.angle_increment, T, r_min_, res_r_max_, n_points_);
    const bool front = topic == topics_[0];
    const bool pair = check_mode_ == "pair";
    std::optional<Scan> other;
    {
      std::lock_guard<std::mutex> g(mu_);
      scans_[topic] = Scan{st, std::move(pts), near};
      for (const auto & [k, v] : scans_) {if (k != topic) {other = v; break;}}
      if (pair && other && std::abs((st - other->st).nanoseconds()) * 1e-9 <= 0.06) {
        const int64_t key = (front ? st : other->st).nanoseconds();   // 같은 짝(앞 stamp)을 두 번 검사하지 않음
        if (have_last_pair_ && key == last_pair_) {return;}
        last_pair_ = key;
        have_last_pair_ = true;
      }
    }
    if (pair) {   // 앞 · 뒤 어느 쪽이 나중에 와도 짝이 맞으면 검사
      if (!other || std::abs((st - other->st).nanoseconds()) * 1e-9 > 0.06) {return;}
      if (front) {queue_check({{st, near}, {other->st, other->near}}, st);} else {queue_check({{other->st, other->near}, {st, near}}, st);}
      return;
    }
    if (!front || !other) {return;}   // 앞 스캔이 올 때 둘을 합쳐 판단
    if (std::abs((st - other->st).nanoseconds()) * 1e-9 > 0.06) {return;}
    queue_check({{st, near}, {other->st, other->near}}, st);
  }

  // 짝을 줄에 넣고, 스캔 시각 TF 가 들어온 짝부터 검사. tf_wait_s 넘게 TF 가 안 오면 건너뜀(n_skip)
  void queue_check(Pair scans, const rclcpp::Time & newest)
  {
    pending_.push_back(std::move(scans));
    std::vector<Pair> keep;
    for (auto & sc : pending_) {
      if (!keep.empty() || !check(sc)) {   // 앞 짝이 아직이면 뒤 짝도 기다림(시각 순서 유지)
        if ((newest - sc[0].first).nanoseconds() * 1e-9 > tf_wait_) {
          ++n_skip_;
        } else {
          keep.push_back(std::move(sc));
        }
      }
    }
    pending_ = std::move(keep);
  }

  bool check(const Pair & scans)
  {
    const rclcpp::Time & st = scans[0].first;
    std::vector<std::pair<L::Pose, std::vector<L::P2>>> pp;
    for (const auto & [t, q] : scans) {
      const auto p = lookup(mission_, base_, t);
      if (!p) {return false;}   // TF 가 아직 — queue_check 가 다음 스캔 때 다시
      pp.emplace_back(*p, q);
    }
    const auto odom = lookup(odom_f_, base_, st);
    if (!odom) {return false;}
    size_t n = 0;
    for (const auto & s : scans) {n += s.second.size();}
    if (n < 20) {++n_skip_; return true;}
    const L::Pose pose = pp[0].first;
    const double r = L::residual_multi(*field_, pp, res_mode_);
    res_win_.push_back(r);
    const double t = st.nanoseconds() * 1e-9;
    const bool bad = r > res_thr_;
    const std::string s = state();
    if (ok_state(s)) {
      if (!bad) {
        bad_since_.reset();
        set_last_good(t, pose, *odom);
        set_state("OK", {{"residual", num(rnd(r, 4))}});
      } else {
        if (!bad_since_) {bad_since_ = t;}
        const double d = t - *bad_since_;
        if (d >= lost_s_) {
          lost("residual", {{"residual", num(rnd(r, 4))}, {"bad_s", num(rnd(d, 2))}});
        } else if (d >= suspect_s_) {
          set_state("SUSPECT", {{"residual", num(rnd(r, 4))}, {"bad_s", num(rnd(d, 2))}});
        }
      }
    } else if (s == "CONFIRM") {
      bool done = false;
      {
        std::lock_guard<std::mutex> g(mu_);   // 작업 스레드가 CONFIRM 으로 바꿀 때 지움
        if (bad) {
          confirm_ok_since_.reset();
        } else {
          if (!confirm_ok_since_) {confirm_ok_since_ = t;}
          done = t - *confirm_ok_since_ >= ok_confirm_s_;
        }
      }
      if (done) {
        {
          bad_since_.reset();
          set_last_good(t, pose, *odom);
          double rs;
          {
            std::lock_guard<std::mutex> g(mu_);
            rs = recover_since_.value_or(0.0);
          }
          set_state("OK", {{"residual", num(rnd(r, 4))}, {"recover_s", num(rnd(now_s() - rs, 2))}});
        }
      }
    }
    return true;
  }

  void set_last_good(double t, const L::Pose & p, const L::Pose & o)
  {
    std::lock_guard<std::mutex> g(mu_);
    last_good_ = LastGood{t, p, o};
  }

  void on_tracked(const PoseCov & m)
  {
    last_tracked_rx_ = now_s();
    gap_logged_ = false;
    const rclcpp::Time st(m.header.stamp, get_clock()->get_clock_type());
    const auto odom = lookup(odom_f_, base_, st);
    const auto & q = m.pose.pose;
    const L::Pose p{q.position.x, q.position.y, yaw_of(q.orientation)};
    std::optional<Tracked> prev;
    {
      std::lock_guard<std::mutex> g(mu_);
      prev = last_tracked_;
      last_tracked_ = Tracked{p, odom};
    }
    if (!prev || !odom || !prev->odom || !ok_state(state())) {return;}
    const auto dc = L::se2_compose(L::se2_inverse(prev->pose), p);   // 위치 추정 증분(차체 기준)
    const auto dodo = L::se2_compose(L::se2_inverse(*prev->odom), *odom);   // odom 증분
    const double ex = dc[0] - dodo[0], ey = dc[1] - dodo[1], eyaw = L::wrap(dc[2] - dodo[2]);
    if (std::hypot(ex, ey) > jump_xy_ || std::abs(eyaw * 180.0 / M_PI) > jump_yaw_deg_) {
      lost("jump", {{"dxy", num(rnd(std::hypot(ex, ey), 3))}, {"dyaw_deg", num(rnd(eyaw * 180.0 / M_PI, 2))}});
    }
  }

  void tick()
  {
    const std::string s = state();
    publish_state(s);
    const double now = now_s();
    if (now <= 0.0) {return;}   // 모의 시계 전
    if (!t_start_) {t_start_ = now;}
    const bool alive = map_odom_alive(now);
    if (ok_state(s) && last_tracked_rx_ && now - *last_tracked_rx_ > gap_s_) {
      if (!alive) {
        lost("gap", {{"gap_s", num(rnd(now - *last_tracked_rx_, 2))}});
      } else if (!gap_logged_) {
        gap_logged_ = true;
        event("gap_alive", {{"gap_s", num(rnd(now - *last_tracked_rx_, 2))}});
      }
    }
    if (ok_state(state()) && !last_tracked_rx_ && now - *t_start_ > grace_) {
      lost("no_tracked_pose", {{"after_s", num(rnd(now - *t_start_, 2))}});   // 시작 뒤 한 번도 안 옴
    }
    double cs;
    {
      std::lock_guard<std::mutex> g(mu_);
      cs = confirm_since_;
    }
    if (state() == "CONFIRM" && now - cs > confirm_timeout_s_) {
      event("confirm_timeout");
      lost("confirm_timeout");
    }
  }

  // amcl 은 pose 를 안 낼 때도 스캔마다 map → odom 을 새 시각으로 다시 내므로, 그 시각이 gap_s 안에 바뀌었으면 살아 있다. 기다리지 않는다
  bool map_odom_alive(double now)
  {
    builtin_interfaces::msg::Time st;
    try {
      st = buf_->lookupTransform(mission_, odom_f_, tf2::TimePointZero).header.stamp;
    } catch (const tf2::TransformException &) {
      return false;
    }
    if (!mo_stamp_ || mo_stamp_->sec != st.sec || mo_stamp_->nanosec != st.nanosec) {
      mo_stamp_ = st;
      mo_change_t_ = now;
    }
    return now - mo_change_t_ <= gap_s_;
  }

  void report()
  {
    std::vector<double> w;
    w.swap(res_win_);
    auto q = [&](double p) {return w.empty() ? std::string("null") : num(rnd(L::percentile(w, p), 4));};
    event("report", {{"n_check", std::to_string(w.size())}, {"n_skip", std::to_string(n_skip_)}, {"res_p50", q(50)},
      {"res_p95", q(95)}, {"res_max", q(100)}, {"traj", std::to_string(traj_)}});
    n_skip_ = 0;
  }

  // ---- 다시 찾기 ----
  void lost(const std::string & why, const KV & kw = {})
  {
    std::lock_guard<std::mutex> g(lost_mu_);   // 여러 곳에서 동시에 불려 작업 스레드를 두 번 만들지 않게
    const std::string s = state();
    if (s == "LOST" || s == "RECOVERING" || s == "FAILED") {return;}
    {
      std::lock_guard<std::mutex> g2(mu_);
      if (!recover_since_ || ok_state(s)) {recover_since_ = now_s();}
    }
    KV k2{{"why", str(why)}};
    k2.insert(k2.end(), kw.begin(), kw.end());
    set_state("LOST", k2);
    if (!worker_alive_) {
      if (worker_.joinable()) {worker_.join();}
      worker_alive_ = true;
      worker_ = std::thread([this] {recover(); worker_alive_ = false;});
    }
  }

  // 앞 · 뒤 스캔을 앞 스캔 시각 base_link 로 모음. 뒤 스캔은 두 시각 사이 odom 이동만큼 옮김(#151)
  bool latest_pts(rclcpp::Time & sf_out, L::Pose & of_out, std::vector<L::P2> & pts)
  {
    std::map<std::string, Scan> v;
    {
      std::lock_guard<std::mutex> g(mu_);
      v = scans_;
    }
    const auto & tf_ = topics_[0], & tr_ = topics_[1];
    if (!v.count(tf_) || !v.count(tr_) || std::abs((v[tf_].st - v[tr_].st).nanoseconds()) * 1e-9 > 0.06) {return false;}
    const auto of = lookup(odom_f_, base_, v[tf_].st, 0.1);   // 작업 스레드라 기다려도 됨
    const auto orr = lookup(odom_f_, base_, v[tr_].st, 0.1);
    if (!of || !orr) {return false;}
    pts = v[tf_].pts;
    const auto moved = L::move_points(v[tr_].pts, L::se2_compose(L::se2_inverse(*of), *orr));
    pts.insert(pts.end(), moved.begin(), moved.end());
    sf_out = v[tf_].st;
    of_out = *of;
    return true;
  }

  // 스캔 시각 자세 ∘ (스캔 시각 → 가장 최근 odom TF 이동). 탐색하는 동안 움직인 만큼 메움(#151)
  bool to_now(const L::Pose & pose_st, const L::Pose & odom_st, L::Pose & out, builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = buf_->lookupTransform(odom_f_, base_, tf2::TimePointZero, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException &) {
      return false;
    }
    const auto d = L::se2_compose(L::se2_inverse(odom_st), tf_pose(t));
    event("motion_comp", {{"dxy", num(rnd(std::hypot(d[0], d[1]), 3))}, {"dyaw_deg", num(rnd(d[2] * 180.0 / M_PI, 2))}});
    out = L::se2_compose(pose_st, d);
    stamp = t.header.stamp;
    return true;
  }

  bool accept(const L::SearchResult & r) const
  {
    if (r.score > accept_score_) {return false;}
    return !r.second_score || *r.second_score - r.score >= amb_margin_;
  }

  static double mono() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}

  void recover()
  {
    set_state("RECOVERING");
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> g(stop_mu_);
        if (stop_) {return;}
      }
      double rs;
      std::optional<LastGood> lg;
      {
        std::lock_guard<std::mutex> g(mu_);
        rs = recover_since_.value_or(now_s());
        lg = last_good_;
      }
      const std::string cur = state();
      if (cur == "CONFIRM" || ok_state(cur)) {return;}   // 운용자 /initialpose 로 이미 다시 확인 중
      if (now_s() - rs > max_recover_s_ && cur != "FAILED") {
        set_state("FAILED", {{"recover_s", num(rnd(now_s() - rs, 2))}});
        if (failed_retry_s_ <= 0) {return;}   // 예전 동작: 멈춤
        event("failed_retry", {{"every_s", num(failed_retry_s_)}});
      }
      rclcpp::Time st;
      L::Pose odom{};
      std::vector<L::P2> pts;
      std::optional<L::SearchResult> cand;
      if (latest_pts(st, odom, pts)) {
        if (lg) {
          const auto center = L::se2_compose(lg->pose, L::se2_compose(L::se2_inverse(lg->odom), odom));
          const double t0 = mono();
          const auto Lr = L::search(*field_, pts, center, local_lin_, local_ang_deg_ * M_PI / 180.0);
          event("search_local", {{"center", arr(center, 3)}, {"best", arr(Lr.best, 3)}, {"score", num(rnd(Lr.score, 4))},
            {"second_score", Lr.second_score ? num(*Lr.second_score) : "null"}, {"wall_s", num(rnd(mono() - t0, 2))}});
          if (accept(Lr)) {cand = Lr;}
        }
        if (!cand) {
          const double t0 = mono();
          const auto G = L::search(*field_, pts, std::nullopt);
          event("search_global", {{"best", arr(G.best, 3)}, {"score", num(rnd(G.score, 4))},
            {"second", G.second ? "[" + num((*G.second)[0]) + ", " + num((*G.second)[1]) + ", " + num((*G.second)[2]) + "]" : "null"},
            {"second_score", G.second_score ? num(*G.second_score) : "null"}, {"n_cand", std::to_string(G.n_cand)},
            {"wall_s", num(rnd(mono() - t0, 2))}});
          if (accept(G)) {cand = G;}
        }
      }
      L::Pose now_pose{};
      builtin_interfaces::msg::Time stamp;
      const std::string cur2 = state();
      if (cur2 == "CONFIRM" || ok_state(cur2)) {return;}   // 탐색하는 동안 운용자가 자세를 넣음 — 덮어쓰지 않음
      if (cand && to_now(cand->best, odom, now_pose, stamp) && restart_amcl(now_pose, stamp)) {
        {
          std::lock_guard<std::mutex> g(mu_);
          confirm_since_ = now_s();
          confirm_ok_since_.reset();
        }
        set_state("CONFIRM", {{"pose", arr(now_pose, 3)}, {"score", num(rnd(cand->score, 4))}});
        return;
      }
      std::unique_lock<std::mutex> lk(stop_mu_);   // 벽시계 — 모의 시계가 멈춰도 다시 시도
      const double wait_s = state() == "FAILED" ? failed_retry_s_ : retry_s_;
      stop_cv_.wait_for(lk, std::chrono::duration<double>(wait_s), [this] {return stop_;});
    }
  }

  // AMCL 에 /initialpose(map = 발제 좌표)로 다시 넣음. 입자는 이 자세 둘레(initialpose_*_sd)로 다시 뿌려짐
  bool restart_amcl(const L::Pose & p, const builtin_interfaces::msg::Time & stamp)
  {
    PoseCov m;
    m.header.stamp = stamp;   // 자세가 맞는 TF 시각 → 남은 이동은 AMCL 이 odom 으로 메움
    m.header.frame_id = mission_;
    m.pose.pose.position.x = p[0];
    m.pose.pose.position.y = p[1];
    m.pose.pose.orientation.z = std::sin(p[2] / 2);
    m.pose.pose.orientation.w = std::cos(p[2] / 2);
    const double syaw = yaw_sd_deg_ * M_PI / 180.0;
    m.pose.covariance[0] = m.pose.covariance[7] = xy_sd_ * xy_sd_;
    m.pose.covariance[35] = syaw * syaw;
    {
      std::lock_guard<std::mutex> g(mu_);
      own_init_stamp_ = m.header.stamp;
    }
    pub_init_->publish(m);
    event("initialpose", {{"mission_pose", arr(p, 4)}});
    std::lock_guard<std::mutex> g(mu_);
    last_tracked_.reset();
    return true;
  }

  // 운용자가 낸 /initialpose (이 노드가 낸 것은 stamp 로 거름). AMCL 이 그 자세로 다시 뿌리므로 여기서는 다시 확인만 시작
  void on_initialpose(const PoseCov & m)
  {
    {
      std::lock_guard<std::mutex> g(mu_);
      if (own_init_stamp_ && own_init_stamp_->sec == m.header.stamp.sec && own_init_stamp_->nanosec == m.header.stamp.nanosec) {return;}
      last_tracked_.reset();   // 자세가 크게 바뀌므로 다음 /amcl_pose 를 튐으로 보지 않게
      confirm_since_ = now_s();
      confirm_ok_since_.reset();
      recover_since_ = now_s();   // 다시 찾기 시간을 새로 셈
    }
    bad_since_.reset();
    const auto & q = m.pose.pose.orientation;
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    set_state("CONFIRM", {{"why", str("manual_initialpose")},
      {"pose", "[" + num(rnd(m.pose.pose.position.x, 3)) + ", " + num(rnd(m.pose.pose.position.y, 3)) + ", " + num(rnd(yaw, 3)) + "]"}});
  }

  struct LastGood {double t; L::Pose pose, odom;};
  struct Tracked {L::Pose pose; std::optional<L::Pose> odom;};

  std::string backend_, mission_, odom_f_, base_, res_mode_, check_mode_;
  std::vector<std::string> topics_;
  double r_min_, r_max_, res_r_max_, grace_, tf_wait_, res_thr_, suspect_s_, lost_s_, jump_xy_, jump_yaw_deg_, gap_s_;
  double xy_sd_, yaw_sd_deg_, accept_score_, amb_margin_, local_lin_, local_ang_deg_, retry_s_, max_recover_s_, ok_confirm_s_, confirm_timeout_s_;
  double failed_retry_s_ = 5.0;
  std::optional<builtin_interfaces::msg::Time> own_init_stamp_;   // 이 노드가 마지막으로 낸 /initialpose stamp (mu_)
  int n_points_;
  int64_t traj_;
  bool gate_ = false;
  std::unique_ptr<L::DistField> field_;
  std::shared_ptr<tf2_ros::Buffer> buf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;

  std::mutex mu_;                 // 상태 · 스캔 · 마지막 정상 자세 등 (작업 스레드와 함께 씀)
  std::mutex lost_mu_, log_mu_;
  std::string state_;
  double state_t_ = 0;
  std::map<std::string, Scan> scans_;
  std::optional<LastGood> last_good_;
  std::optional<Tracked> last_tracked_;
  std::optional<double> recover_since_, confirm_ok_since_;
  double confirm_since_ = 0;
  int64_t last_pair_ = 0;
  bool have_last_pair_ = false;

  // 아래는 실행기 스레드에서만
  std::optional<double> bad_since_, last_tracked_rx_, t_start_;
  std::vector<Pair> pending_;
  std::map<std::string, L::T2> T_laser_;
  std::vector<double> res_win_;
  long n_skip_ = 0;
  std::optional<builtin_interfaces::msg::Time> mo_stamp_;
  double mo_change_t_ = 0;
  bool gap_logged_ = false;

  std::thread worker_;
  std::atomic<bool> worker_alive_{false};
  std::mutex stop_mu_;
  std::condition_variable stop_cv_;
  bool stop_ = false;
  std::ofstream log_f_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_lock_;
  rclcpp::Publisher<TwistStamped>::SharedPtr pub_cmd_;
  rclcpp::Publisher<PoseCov>::SharedPtr pub_init_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_cmd_;
  rclcpp::Subscription<PoseCov>::SharedPtr sub_pose_, sub_init_;
  std::vector<rclcpp::Subscription<LaserScan>::SharedPtr> scan_subs_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
};

}  // namespace perception

RCLCPP_COMPONENTS_REGISTER_NODE(perception::LocMonitorNode)
