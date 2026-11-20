// LocScanMergeNode — AMCL 입력용 스캔 합치기 (localization/scan_merge.py 를 C++ 로 옮김, 동작 같음).
// 왜 옮겼나: Python 판이 Jetson 실물 정지 상태에서 CPU 코어 하나의 약 22 % (11-13 #265).
// (perception_container 의 ScanMergerNode(/scan, 장애물용)와 다름: 이쪽은 AMCL 용, 움직임 보정 · 720 칸 · base_link 원점 기준)
// 앞 · 뒤 가림 제거 스캔(/scan_front_filtered · /scan_rear_filtered)을 base_link 원점 기준 한 스캔(/scan_merged, frame base_link,
// 720 칸 0.5°)으로 낸다. 각 점의 끝 위치는 그대로 두고 base_link 원점에서 본 각 · 거리로만 바꾼다.
// 앞 스캔마다 낸다(stamp = 앞 스캔). 같은 stamp 의 뒤 스캔이 pair_wait 초 안에 오면 그대로 붙이고, 안 오면 가장 최근 뒤 스캔(max_rear_age 안)을
// odom TF 로 앞 스캔 시각의 차체 자세에 옮겨 넣는다(움직임 보정).
// 앞 라이다가 끊기면 (front_timeout 넘게 앞 스캔 없음) 뒤 스캔마다 뒤 스캔만으로 낸다 (2026-11-17, 데모 점검 2 번:
//   예전에는 앞 스캔이 와야만 내서 앞 라이다 하나가 빠지면 AMCL · costmap 입력이 통째로 멈췄음). front_timeout ≤ 0 이면 예전과 같음.
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "perception/tf_helper.hpp"

namespace perception
{
using LaserScan = sensor_msgs::msg::LaserScan;

class LocScanMergeNode : public rclcpp::Node
{
public:
  explicit LocScanMergeNode(const rclcpp::NodeOptions & options)
  : Node("scan_merge", options)
  {
    const auto front = declare_parameter("front", std::string("/scan_front_filtered"));
    const auto rear = declare_parameter("rear", std::string("/scan_rear_filtered"));
    out_ = declare_parameter("out", std::string("/scan_merged"));
    base_ = declare_parameter("base_frame", std::string("base_link"));
    odom_ = declare_parameter("odom_frame", std::string("odom"));
    bins_ = static_cast<int>(declare_parameter("bins", 720));
    rmax_ = declare_parameter("range_max", 12.0);
    max_age_ = declare_parameter("max_rear_age", 0.15);
    pair_wait_ = declare_parameter("pair_wait", 0.06);
    front_timeout_ = declare_parameter("front_timeout", 0.5);
    if (bins_ < 1) {throw std::invalid_argument("bins < 1");}
    tf_ = std::make_unique<TfHelper>(*this);
    pub_ = create_publisher<LaserScan>(out_, 10);
    last_front_rx_ = now().seconds();   // 시작 직후는 앞 스캔을 기다림
    rear_sub_ = create_subscription<LaserScan>(rear, rclcpp::SensorDataQoS(),
        [this](LaserScan::ConstSharedPtr m) {on_rear(m);});
    front_sub_ = create_subscription<LaserScan>(front, rclcpp::SensorDataQoS(),
        [this](LaserScan::ConstSharedPtr m) {on_front_in(m);});
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(20), [this] {flush();});   // Python 판처럼 노드 시계
  }

private:
  struct P2 {double x, y;};

  static bool same_stamp(const LaserScan & a, const LaserScan & b)
  {
    return a.header.stamp.sec == b.header.stamp.sec && a.header.stamp.nanosec == b.header.stamp.nanosec;
  }
  static double sec(const builtin_interfaces::msg::Time & t) {return t.sec + t.nanosec * 1e-9;}

  // 스캔 → base_link 기준 점 (유한 · range_min ~ range_max 안만). 장착은 정적이라 처음 한 번 찾음(TfHelper)
  bool points(const LaserScan & m, std::vector<P2> & out)
  {
    const auto T = tf_->fixed(base_, m.header.frame_id);
    if (!T) {return false;}
    const auto & R = T->getBasis();
    const double px = T->getOrigin().x(), py = T->getOrigin().y();
    for (size_t i = 0; i < m.ranges.size(); ++i) {
      const double r = m.ranges[i];
      if (!std::isfinite(r) || r < m.range_min || r > m.range_max) {continue;}
      const double a = m.angle_min + static_cast<double>(m.angle_increment) * static_cast<double>(i);
      const double ca = std::cos(a), sa = std::sin(a);
      out.push_back({px + r * (R[0][0] * ca + R[0][1] * sa), py + r * (R[1][0] * ca + R[1][1] * sa)});
    }
    return true;
  }

  // odom ← base 의 평면 변환 (요만) — Python 판 tf_matrix 와 같음
  bool odom_base(const builtin_interfaces::msg::Time & st, double & c, double & s, double & tx, double & ty)
  {
    try {
      const auto t = tf_->buffer().lookupTransform(odom_, base_, rclcpp::Time(st), rclcpp::Duration::from_seconds(0.05));
      const auto & q = t.transform.rotation;
      const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      c = std::cos(yaw); s = std::sin(yaw); tx = t.transform.translation.x; ty = t.transform.translation.y;
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void on_front_in(const LaserScan::ConstSharedPtr & m)
  {
    last_front_rx_ = now().seconds();
    if (rear_only_) {
      rear_only_ = false;
      RCLCPP_WARN(get_logger(), "앞 스캔 다시 들어옴 — 앞 · 뒤 합치기로 돌아감 (뒤 스캔만 %ld 장 냈음)", n_rear_only_);
    }
    if (pending_) {auto p = pending_; pending_.reset(); on_front(*p);}
    if (rear_ && same_stamp(*rear_, *m)) {
      on_front(*m);
    } else {
      pending_ = m;
    }
  }

  void on_rear(const LaserScan::ConstSharedPtr & m)
  {
    rear_ = m;
    const double t = now().seconds();
    if (front_timeout_ > 0 && t - last_front_rx_ > front_timeout_) {
      if (!rear_only_) {
        rear_only_ = true;
        RCLCPP_WARN(get_logger(), "앞 스캔이 %.2f s 넘게 없음 — 뒤 스캔만으로 %s 를 냄", front_timeout_, out_.c_str());
      }
      std::vector<P2> pts;
      if (points(*m, pts)) {
        publish(pts, *m);
        ++n_rear_only_;
      }
      return;
    }
    if (pending_ && same_stamp(*pending_, *m)) {auto p = pending_; pending_.reset(); on_front(*p);}
  }

  void flush()
  {
    if (pending_ && (now().seconds() - sec(pending_->header.stamp)) > pair_wait_) {
      auto p = pending_; pending_.reset(); on_front(*p);
    }
  }

  void on_front(const LaserScan & msg)
  {
    std::vector<P2> pts;
    if (!points(msg, pts)) {return;}
    const double tf_s = sec(msg.header.stamp);
    if (rear_ && std::abs(tf_s - sec(rear_->header.stamp)) <= max_age_) {
      if (same_stamp(*rear_, msg)) {   // 같은 시각이면 차체 움직임 보정이 필요 없음
        if (points(*rear_, pts)) {++n_paired_;} else {++n_no_rear_;}
      } else {
        double c0, s0, x0, y0, c1, s1, x1, y1;
        std::vector<P2> rp;
        if (odom_base(msg.header.stamp, c0, s0, x0, y0) && odom_base(rear_->header.stamp, c1, s1, x1, y1) && points(*rear_, rp)) {
          for (const auto & p : rp) {   // inv(to) · tr · p
            const double ox = c1 * p.x - s1 * p.y + x1, oy = s1 * p.x + c1 * p.y + y1;
            const double dx = ox - x0, dy = oy - y0;
            pts.push_back({c0 * dx + s0 * dy, -s0 * dx + c0 * dy});
          }
        } else {
          ++n_no_rear_;
        }
      }
    } else {
      ++n_no_rear_;
    }
    publish(pts, msg);
  }

  // base_link 점 → 720 칸 스캔 (stamp · scan_time = src)
  void publish(const std::vector<P2> & pts, const LaserScan & src)
  {
    const double inc = 2 * M_PI / bins_;
    auto m = std::make_unique<LaserScan>();
    m->ranges.assign(bins_, std::numeric_limits<float>::infinity());
    std::vector<double> best(bins_, std::numeric_limits<double>::infinity());
    for (const auto & p : pts) {
      const double ang = std::atan2(p.y, p.x), rng = std::hypot(p.x, p.y);
      int idx = static_cast<int>((ang + M_PI) / inc);   // Python astype(int) 와 같이 0 쪽으로 자름
      idx = std::min(std::max(idx, 0), bins_ - 1);
      if (rng < best[idx]) {best[idx] = rng;}
    }
    for (int i = 0; i < bins_; ++i) {m->ranges[i] = static_cast<float>(best[i]);}
    m->header.stamp = src.header.stamp;
    m->header.frame_id = base_;
    m->angle_min = static_cast<float>(-M_PI + inc / 2);
    m->angle_increment = static_cast<float>(inc);
    m->angle_max = static_cast<float>(-M_PI + inc / 2 + inc * (bins_ - 1));
    m->time_increment = 0.0f;
    m->scan_time = src.scan_time;
    m->range_min = 0.0f;
    m->range_max = static_cast<float>(rmax_);
    pub_->publish(std::move(m));
    if (++n_out_ % 300 == 0) {
      RCLCPP_INFO(get_logger(), "합친 스캔 %ld (같은 시각 뒤 스캔 %ld, 뒤 스캔 못 붙임 %ld)", n_out_, n_paired_, n_no_rear_);
    }
  }

  std::string base_, odom_, out_;
  double front_timeout_ = 0.5, last_front_rx_ = 0.0;
  bool rear_only_ = false;
  long n_rear_only_ = 0;
  int bins_ = 720;
  double rmax_ = 12.0, max_age_ = 0.15, pair_wait_ = 0.06;
  std::unique_ptr<TfHelper> tf_;
  LaserScan::ConstSharedPtr rear_, pending_;
  long n_out_ = 0, n_paired_ = 0, n_no_rear_ = 0;
  rclcpp::Publisher<LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<LaserScan>::SharedPtr rear_sub_, front_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace perception

RCLCPP_COMPONENTS_REGISTER_NODE(perception::LocScanMergeNode)
