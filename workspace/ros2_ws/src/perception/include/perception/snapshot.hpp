// 팔 카메라 사진 저장 (/perception/snapshot): 요청 하나 = 일정한 간격으로 몇 장 (ROS 의존 없음, 시험용으로 분리)
//   SnapshotSchedule : 언제 몇 번째 장을 찍을지 · 새 영상이 안 오면 언제 포기할지 (시계 · 영상 번호만 받음)
//   unique_jpg_path  : <접두어>_<k>.jpg, 이미 있으면 _<k>_2.jpg ... (덮어쓰지 않음)
//   JobQueue         : 영상 변환 · JPEG 인코딩 · 쓰기를 처리 주기 밖의 스레드 하나에서 차례로
#pragma once

#include <condition_variable>
#include <deque>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace perception
{

struct SnapshotConfig
{
  int count = 3;             // 요청 하나에 찍는 장 수 (0 이면 끔)
  double delay_s = 0.5;      // 요청 → 첫 장 [s]
  double interval_s = 0.5;   // 장 사이 [s]
  double max_wait_s = 3.0;   // 그 장 시각 뒤 새 영상을 기다리는 한도 [s]. 넘으면 남은 장을 포기
  int jpeg_quality = 90;
};

class SnapshotSchedule
{
public:
  enum class Action { Idle, Wait, Take, GiveUp };

  explicit SnapshotSchedule(const SnapshotConfig & cfg = {}) : cfg_(cfg) {}

  // 새 요청. 이전 요청이 끝나기 전이면 true (남은 장은 버림). seq = 요청 때 마지막으로 받은 영상 번호
  bool start(const std::string & prefix, double t, uint64_t seq)
  {
    const bool cut = active();
    prefix_ = prefix;
    t0_ = t;
    last_seq_ = seq;
    next_ = cfg_.count > 0 ? 1 : 0;
    return cut;
  }

  // 지금 할 일. latest_seq = 지금까지 받은 영상 번호 (받을 때마다 1 씩 늘어남)
  //   Take: 지금 영상을 저장하고 taken(latest_seq) 를 부를 것. GiveUp: 남은 장 포기 (이미 비활성)
  Action poll(double t, uint64_t latest_seq)
  {
    if (!active()) {return Action::Idle;}
    const double due = dueTime(next_);
    if (t < due) {return Action::Wait;}
    if (latest_seq > last_seq_) {return Action::Take;}   // 요청 뒤 · 직전 장 뒤에 새로 받은 영상만
    if (t > due + cfg_.max_wait_s) {
      given_up_at_ = next_;
      next_ = 0;
      return Action::GiveUp;
    }
    return Action::Wait;
  }

  // Take 뒤: 찍은 장 번호 (1 부터) 를 돌려줌
  int taken(uint64_t seq)
  {
    const int k = next_;
    last_seq_ = seq;
    next_ = next_ < cfg_.count ? next_ + 1 : 0;
    return k;
  }

  void cancel() {next_ = 0;}
  bool active() const {return next_ > 0;}
  int next() const {return next_;}
  int givenUpAt() const {return given_up_at_;}
  const std::string & prefix() const {return prefix_;}
  const SnapshotConfig & config() const {return cfg_;}
  double dueTime(int k) const {return t0_ + cfg_.delay_s + (k - 1) * cfg_.interval_s;}

private:
  SnapshotConfig cfg_;
  std::string prefix_;
  double t0_ = 0.0;
  uint64_t last_seq_ = 0;
  int next_ = 0;          // 다음에 찍을 장 (1..count), 0 = 비활성
  int given_up_at_ = 0;
};

// <prefix>_<k>.jpg. 이미 있으면 <prefix>_<k>_2.jpg, _3 ... (PAUSED 재개 · 재시도로 같은 이름이 와도 덮어쓰지 않음)
inline std::string unique_jpg_path(const std::string & prefix, int k)
{
  const std::string base = prefix + "_" + std::to_string(k);
  std::error_code ec;
  std::string p = base + ".jpg";
  for (int n = 2; std::filesystem::exists(p, ec); ++n) {
    p = base + "_" + std::to_string(n) + ".jpg";
  }
  return p;
}

// 작업을 차례로 처리하는 스레드 하나. 소멸 때 남은 작업을 다 하고 끝난다
class JobQueue
{
public:
  JobQueue() : th_([this]() {run();}) {}
  ~JobQueue()
  {
    {
      std::lock_guard<std::mutex> lk(m_);
      stop_ = true;
    }
    cv_.notify_all();
    th_.join();
  }
  JobQueue(const JobQueue &) = delete;
  JobQueue & operator=(const JobQueue &) = delete;

  void push(std::function<void()> job)
  {
    {
      std::lock_guard<std::mutex> lk(m_);
      q_.push_back(std::move(job));
    }
    cv_.notify_one();
  }

  size_t pending() const
  {
    std::lock_guard<std::mutex> lk(m_);
    return q_.size() + (busy_ ? 1 : 0);
  }

private:
  void run()
  {
    for (;;) {
      std::function<void()> job;
      {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [this]() {return stop_ || !q_.empty();});
        if (q_.empty()) {return;}   // stop_ 이고 남은 일 없음
        job = std::move(q_.front());
        q_.pop_front();
        busy_ = true;
      }
      job();
      std::lock_guard<std::mutex> lk(m_);
      busy_ = false;
    }
  }

  mutable std::mutex m_;
  std::condition_variable cv_;
  std::deque<std::function<void()>> q_;
  bool stop_ = false, busy_ = false;
  std::thread th_;   // 맨 뒤: 위 멤버가 만들어진 뒤 시작
};

}  // namespace perception
