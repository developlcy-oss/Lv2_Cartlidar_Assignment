// 신호등 점멸 순서 세기 (ROS 의존 없음 → 단위 시험)
//   입력: 프레임마다 신호등 상태(꺼짐 · 빨강 · 노랑 · 초록)와 촬영 시각. 신호등을 못 찾은 프레임은 넣지 않는다.
//   1) 최근 3프레임 다수결로 한 프레임짜리 잘못된 상태를 거른다 (다수가 없으면 직전 상태 유지)
//   2) 꺼짐 → 켜짐, 또는 켜진 채 색이 바뀌면 점멸 1회
//   3) 잡음 정리: 켜짐 길이 중앙값 × on_ratio 보다 짧은 점멸은 버리고,
//      같은 색 두 점멸 사이 꺼짐이 꺼짐 중앙값 × merge_ratio 보다 짧으면 한 점멸로 합친다
//      (켜짐 · 꺼짐 길이는 출제자가 정하므로 절대 시간 대신 그 신호등에서 잰 길이에 비례해 거른다)
//   4) 꺼짐 중앙값 × term_ratio 와 꺼짐 중앙값 + 프레임 간격 × term_margin_frames 중 긴 것보다 오래 꺼져 있으면 묶음 사이 텀
//      (꺼짐이 몇 프레임뿐이면 잡음 1~2프레임으로도 비율을 넘으므로 프레임 여유를 둔다)
//      (한 묶음에 꺼짐이 blinks_per_cycle - 1 번, 텀은 1 번이라 중앙값은 점멸 사이 꺼짐 길이를 따른다)
//   5) 텀으로 끝난 묶음을 기록한다. 맨 앞 묶음은 중간부터 봤을 수 있지만 점멸 수가 맞을 때만 후보가 되므로 같이 기록
//      묶음은 꺼짐이 텀을 넘는 순간 먼저 기록되고, 텀 길이는 다음 점멸이 끝난 뒤 그 묶음에 채운다
//   6) blinks_per_cycle 개짜리 묶음 중 같은 순서가 confirm_cycles 번 나오면 확정. reset() 전까지 유지
//      (확정 뒤에도 잰 켜짐 · 꺼짐 · 텀 길이는 계속 갱신한다. 텀은 점멸 수가 맞는 두 묶음 사이만 쓰므로
//       맞는 묶음을 연달아 두 번 본 뒤에야 0 이 아니다)
//   max_gap 보다 오래 프레임이 끊기면(신호등을 놓침) 세던 점멸은 버리고 기록한 묶음만 남긴다.
//   height_color: 빨강 · 노랑을 켜진 등 높이(z)로 가른다 (신호등은 늘 위부터 빨강 · 노랑 · 초록).
//     빨강 · 노랑 프레임은 한 '따뜻한 색' 점멸로 세고 점멸마다 높이 평균을 저장한다. 묶음 순서는 평가할 때마다
//     지금까지 본 초록 점멸 높이(중앙값)를 기준으로 다시 읽는다 → 초록을 보기 전에 기록한 점멸도 고쳐진다.
//     초록이 없으면 따뜻한 색 점멸 높이가 반 칸 넘게 갈렸을 때 가운데보다 위 = 빨강, 그것도 없으면 색상 다수결.
//     경계(1.5 칸)에서 height_margin 칸 안이거나 기준이 없는 점멸은 '애매'. 애매한 점멸이 없는 묶음이
//     confirm_clear_cycles 번 나오면 바로 확정(0 = 끔), 아니면 위 6) 처럼 같은 순서 confirm_cycles 번.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace perception
{

enum class Light : uint8_t {Off = 0, Red = 1, Yellow = 2, Green = 3};

inline char light_char(Light l)
{
  switch (l) {
    case Light::Red: return 'R';
    case Light::Yellow: return 'Y';
    case Light::Green: return 'G';
    default: return '-';
  }
}

struct SignalConfig
{
  int blinks_per_cycle = 12;     // 과제: 한 묶음 12회
  double term_ratio = 2.0;       // 텀 = 꺼짐 중앙값의 이 배수보다 긴 꺼짐 ★
  double term_margin_frames = 4; // 그리고 꺼짐 중앙값보다 이 프레임 수 이상 긴 꺼짐 ★
  double on_ratio = 0.5;         // 켜짐 중앙값의 이 배수보다 짧은 점멸은 잡음 ★
  double merge_ratio = 0.5;      // 같은 색 사이 꺼짐이 꺼짐 중앙값의 이 배수보다 짧으면 한 점멸 ★
  int min_gaps = 4;              // 중앙값을 믿기 전에 필요한 꺼짐 · 켜짐 개수
  double max_gap = 0.5;          // 이보다 오래 프레임이 없으면 끊김 [s]
  int confirm_cycles = 2;        // 같은 묶음이 이만큼 모이면 확정
  bool height_color = false;     // 빨강 · 노랑을 켜진 등 높이로 가름 (위 머리말). false = 색상 그대로
  double segment_pitch = 0.0349; // [m] 칸 중심 위아래 간격 (height_color 일 때)
  double height_margin = 0.25;   // [칸] 빨강 · 노랑 경계에서 이만큼 안이면 애매
  int confirm_clear_cycles = 1;  // height_color 일 때 애매한 점멸 없는 같은 묶음이 이만큼이면 확정 (0 = 끔)
  std::size_t max_events = 120;  // 기억할 점멸 수 상한
};

struct SignalResult
{
  enum Status : uint8_t {SEARCHING = 0, COUNTING = 1, CANDIDATE = 2, CONFIRMED = 3, MISMATCH = 4};
  Status status = SEARCHING;
  std::string sequence;          // 확정(또는 마지막 후보) 순서, 예: "RGRRGRGGYGYG"
  int red = 0, yellow = 0, green = 0;
  int cycles = 0;                // 본 묶음 중 점멸 수가 맞는 것
  int ambiguous = 0;             // sequence 안 애매한 점멸 수 (height_color 일 때만 0 이 아닐 수 있음)
  double t_on = 0, t_off = 0, t_term = 0;   // 잰 켜짐 중앙값 · 점멸 사이 꺼짐 중앙값 · 텀 중앙값(점멸 수가 맞는 두 묶음 사이) [s]
};

class SignalSequenceTracker
{
public:
  explicit SignalSequenceTracker(SignalConfig c = {})
  : c_(c) {}

  void reset()
  {
    raw_.clear();
    events_.clear();
    cycles_.clear();
    stable_ = Light::Off;
    started_ = false;
    green_z_.clear();
    result_ = {};
  }

  const SignalResult & result() const {return result_;}

  // t: 촬영 시각 [s], 시각 순서대로. z: 켜진 등 높이 [m] (height_color 일 때만 씀, 모르면 NaN)
  const SignalResult & update(double t, Light raw, double z = std::numeric_limits<double>::quiet_NaN())
  {
    const bool warm = raw == Light::Red || raw == Light::Yellow;
    const Light hue = raw;
    if (c_.height_color && warm) {raw = Light::Yellow;}   // 따뜻한 색은 한 가지로 센다 (빨강 · 노랑은 높이로 나중에 가름)
    if (!started_ || t - last_t_ > c_.max_gap) {
      // 처음이거나 끊긴 뒤: 그 전 일은 모르므로 꺼짐에서 새로 시작
      started_ = true;
      raw_.clear();
      events_.clear();
      stable_ = Light::Off;
      dt_ = -1;
      last_t_ = t;
    }
    if (dt_ <= 0) {
      dt_ = t - last_t_;
    } else if (t > last_t_) {
      dt_ += 0.05 * (std::min(t - last_t_, 3 * dt_) - dt_);   // 빠진 프레임이 섞여도 크게 흔들리지 않게
    }
    last_t_ = t;
    raw_.push_back(raw);
    if (raw_.size() > 3) {raw_.pop_front();}
    const Light s = filtered();
    if (s != stable_) {transition(t, s);}
    if (stable_ != Light::Off && raw == stable_ && !events_.empty() && events_.back().off < 0) {
      Event & e = events_.back();
      if (std::isfinite(z)) {e.zsum += z; ++e.zn;}
      e.hue_r += hue == Light::Red;
      e.hue_y += hue == Light::Yellow;
    }
    evaluate(t);
    return result_;
  }

private:
  struct Event
  {
    Light color;
    double on = 0, off = -1;   // off < 0: 아직 켜져 있음
    double zsum = 0;           // 켜진 프레임 높이 합 · 수 (height_color)
    int zn = 0;
    int hue_r = 0, hue_y = 0;  // 색상으로 본 빨강 · 노랑 프레임 수 (높이 기준이 없을 때 씀)
    double z() const {return zn > 0 ? zsum / zn : std::numeric_limits<double>::quiet_NaN();}
    void absorb(const Event & b) {off = b.off; zsum += b.zsum; zn += b.zn; hue_r += b.hue_r; hue_y += b.hue_y;}
  };

  struct Cycle
  {
    double t0, t1;             // 첫 점멸 켜짐 · 마지막 점멸 꺼짐 시각
    std::string seq;           // 기록할 때 색 (height_color 면 따뜻한 색은 'Y', 평가 때 label 로 다시 읽음)
    double term = -1;          // 이 묶음 뒤 텀 [s] (다음 점멸을 아직 못 봤으면 < 0)
    std::vector<Event> ev;     // 묶음의 점멸 (높이 포함)
  };

  // 높이 기준: 본 초록 점멸 높이 중앙값, 없으면 따뜻한 색 점멸 높이의 가운데(반 칸 넘게 갈렸을 때)
  struct HeightRef
  {
    enum Kind {NONE, GREEN, SPREAD} kind = NONE;
    double z = 0;
  };

  HeightRef height_ref() const
  {
    HeightRef r;
    if (!green_z_.empty()) {r.kind = HeightRef::GREEN; r.z = median(green_z_); return r;}
    double lo = 0, hi = 0;
    int n = 0;
    auto add = [&](const Event & e) {
        if (e.color != Light::Yellow || e.zn == 0) {return;}
        const double z = e.z();
        lo = n ? std::min(lo, z) : z;
        hi = n ? std::max(hi, z) : z;
        ++n;
      };
    for (const auto & c : cycles_) {for (const auto & e : c.ev) {add(e);}}
    for (const auto & e : events_) {add(e);}
    if (n >= 2 && hi - lo >= 0.5 * c_.segment_pitch) {r.kind = HeightRef::SPREAD; r.z = 0.5 * (lo + hi);}
    return r;
  }

  // 점멸 하나의 색과 애매 여부
  char label(const Event & e, const HeightRef & ref, bool & amb) const
  {
    amb = false;
    if (!c_.height_color || e.color != Light::Yellow) {return light_char(e.color);}
    if (e.zn > 0 && ref.kind == HeightRef::GREEN) {
      const double x = (e.z() - ref.z) / c_.segment_pitch;   // 초록 칸에서 몇 칸 위 (노랑 1 · 빨강 2)
      amb = std::abs(x - 1.5) < c_.height_margin || x < 0.5 || x > 2.5;
      return x >= 1.5 ? 'R' : 'Y';
    }
    if (e.zn > 0 && ref.kind == HeightRef::SPREAD) {
      amb = std::abs(e.z() - ref.z) < c_.height_margin * c_.segment_pitch;
      return e.z() >= ref.z ? 'R' : 'Y';
    }
    amb = true;
    return e.hue_r > e.hue_y ? 'R' : 'Y';
  }

  Light filtered() const
  {
    if (raw_.size() < 3) {return raw_.back();}
    if (raw_[0] == raw_[1] || raw_[0] == raw_[2]) {return raw_[0];}
    if (raw_[1] == raw_[2]) {return raw_[1];}
    return stable_;
  }

  void transition(double t, Light s)
  {
    if (!events_.empty() && events_.back().off < 0) {
      events_.back().off = t;
      if (events_.back().color == Light::Green && events_.back().zn > 0) {
        green_z_.push_back(events_.back().z());
        if (green_z_.size() > 400) {green_z_.erase(green_z_.begin());}
      }
    }
    if (s != Light::Off) {
      events_.push_back({s, t, -1});
      if (events_.size() > c_.max_events) {events_.pop_front();}
    }
    stable_ = s;
  }

  static double median(std::vector<double> v)
  {
    if (v.empty()) {return 0;}
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
  }

  static double gap(const Event & a, const Event & b) {return a.off < 0 ? 0 : b.on - a.off;}

  // 3) 잡음 정리
  std::vector<Event> cleaned(double & med_on, double & med_off) const
  {
    std::vector<double> ons, offs;
    for (std::size_t i = 0; i < events_.size(); ++i) {
      if (events_[i].off >= 0) {ons.push_back(events_[i].off - events_[i].on);}
      if (i + 1 < events_.size()) {offs.push_back(gap(events_[i], events_[i + 1]));}
    }
    med_on = median(ons);
    med_off = median(offs);
    const bool trust = static_cast<int>(ons.size()) >= c_.min_gaps && static_cast<int>(offs.size()) >= c_.min_gaps;
    std::vector<Event> v(events_.begin(), events_.end());
    if (!trust) {return v;}
    const double merge = c_.merge_ratio * med_off;
    // 같은 색 사이 짧은 꺼짐 합치기 (켜진 중간에 끼어든 꺼짐 잡음)
    std::vector<Event> m;
    for (const auto & e : v) {
      if (!m.empty() && m.back().color == e.color && gap(m.back(), e) < merge) {
        m.back().absorb(e);
      } else {
        m.push_back(e);
      }
    }
    // 가장 짧은 점멸부터 지우고, 지울 때마다 양옆이 같은 색이면 다시 합친다
    // (켜진 중간에 다른 색이 끼면 앞뒤 조각이 각각 짧아지므로 끼어든 것부터 지워야 한다)
    for (;;) {
      std::size_t k = m.size();
      double shortest = c_.on_ratio * med_on;
      for (std::size_t i = 0; i < m.size(); ++i) {
        if (m[i].off >= 0 && m[i].off - m[i].on < shortest) {shortest = m[i].off - m[i].on; k = i;}
      }
      if (k == m.size()) {break;}
      m.erase(m.begin() + k);
      if (k > 0 && k < m.size() && m[k - 1].color == m[k].color && gap(m[k - 1], m[k]) < merge) {
        m[k - 1].absorb(m[k]);
        m.erase(m.begin() + k);
      }
    }
    return m;
  }

  void evaluate(double t)
  {
    double med_on = 0, med_off = 0;
    const std::vector<Event> ev = cleaned(med_on, med_off);
    std::vector<double> gaps;
    for (std::size_t i = 0; i + 1 < ev.size(); ++i) {gaps.push_back(gap(ev[i], ev[i + 1]));}
    const double med = median(gaps);
    if (static_cast<int>(gaps.size()) >= c_.min_gaps && med > 0) {
      const double term = std::max(med * c_.term_ratio, med + c_.term_margin_frames * std::max(dt_, 0.0));
      std::size_t begin = 0;
      for (std::size_t i = 0; i < gaps.size(); ++i) {
        if (gaps[i] <= term) {continue;}
        if (ev[i + 1].off < 0) {break;}   // 텀 뒤 점멸이 아직 켜져 있으면 잡음 정리가 바뀔 수 있어 기다린다
        record(ev, begin, i + 1, gaps[i]);
        begin = i + 1;
      }
      const Event & last = ev.back();
      if (stable_ == Light::Off && last.off >= 0 && t - last.off > term) {record(ev, begin, ev.size(), -1);}
    }

    const double t_term = measured_term();
    if (result_.status == SignalResult::CONFIRMED) {
      // 순서는 그대로 두고 잰 길이만 갱신 (신호등을 놓쳐 점멸 기록이 비면 마지막 값 유지)
      if (med_on > 0) {result_.t_on = med_on;}
      if (med > 0) {result_.t_off = med;}
      if (t_term > 0) {result_.t_term = t_term;}
      return;
    }
    std::vector<std::string> valid;
    std::vector<int> amb_n;   // valid 와 같은 순서: 묶음의 애매한 점멸 수
    const HeightRef ref = height_ref();
    for (const auto & c : cycles_) {
      if (static_cast<int>(c.seq.size()) != c_.blinks_per_cycle) {continue;}
      std::string s;
      int n = 0;
      for (const auto & e : c.ev) {bool a; s += label(e, ref, a); n += a;}
      valid.push_back(s);
      amb_n.push_back(n);
    }
    SignalResult r;
    r.cycles = static_cast<int>(valid.size());
    r.t_on = med_on;
    r.t_off = med;
    r.t_term = t_term;
    r.status = events_.empty() && cycles_.empty() ? SignalResult::SEARCHING : SignalResult::COUNTING;
    for (std::size_t i = 0; i < valid.size(); ++i) {
      const std::string & s = valid[i];
      int clear = 0;
      for (std::size_t j = 0; j < valid.size(); ++j) {clear += valid[j] == s && amb_n[j] == 0;}
      const bool by_clear = c_.height_color && c_.confirm_clear_cycles > 0 && clear >= c_.confirm_clear_cycles;
      if (by_clear || std::count(valid.begin(), valid.end(), s) >= c_.confirm_cycles) {
        r.status = SignalResult::CONFIRMED;
        r.sequence = s;
        r.ambiguous = by_clear ? 0 : amb_n[i];
        break;
      }
    }
    if (r.status != SignalResult::CONFIRMED && !valid.empty()) {
      r.sequence = valid.back();
      r.ambiguous = amb_n.back();
      r.status = valid.size() >= 2 ? SignalResult::MISMATCH : SignalResult::CANDIDATE;
    }
    for (char ch : r.sequence) {
      r.red += ch == 'R';
      r.yellow += ch == 'Y';
      r.green += ch == 'G';
    }
    result_ = r;
  }

  // 텀 길이 = 점멸 수가 맞는 두 묶음 사이 꺼짐의 중앙값 (모르면 0).
  // 텀 중간에 잡음 점멸이 끼면 텀이 둘로 나뉘고 뒤 묶음의 점멸 수가 틀려지므로, 앞뒤 묶음이 모두 맞을 때만 쓴다
  double measured_term() const
  {
    std::vector<Cycle> cs(cycles_.begin(), cycles_.end());
    std::sort(cs.begin(), cs.end(), [](const Cycle & a, const Cycle & b) {return a.t0 < b.t0;});
    const auto full = [this](const Cycle & c) {return static_cast<int>(c.seq.size()) == c_.blinks_per_cycle;};
    std::vector<double> terms;
    for (std::size_t i = 0; i + 1 < cs.size(); ++i) {
      if (cs[i].term > 0 && full(cs[i]) && full(cs[i + 1])) {terms.push_back(cs[i].term);}
    }
    return median(terms);
  }

  // 같은 시간대 묶음은 한 번만 기록 (잡음 정리 결과가 나중에 조금 달라져도 같은 묶음을 두 번 세지 않게).
  // 이미 기록한 묶음이면 텀 길이만 아직 없을 때 채운다 (term_after < 0: 다음 점멸을 아직 못 봄)
  void record(const std::vector<Event> & ev, std::size_t begin, std::size_t end, double term_after)
  {
    if (begin >= end) {return;}
    const double t0 = ev[begin].on, t1 = ev[end - 1].off;
    for (auto & c : cycles_) {
      if (t0 <= c.t1 && c.t0 <= t1) {
        if (term_after > 0 && c.term < 0) {c.term = term_after;}
        return;
      }
    }
    std::string s;
    for (std::size_t i = begin; i < end; ++i) {s += light_char(ev[i].color);}
    cycles_.push_back({t0, t1, s, term_after, std::vector<Event>(ev.begin() + begin, ev.begin() + end)});
  }

  SignalConfig c_;
  std::deque<Light> raw_;
  std::deque<Event> events_;
  std::vector<Cycle> cycles_;
  Light stable_ = Light::Off;
  bool started_ = false;
  double last_t_ = 0, dt_ = -1;   // 프레임 간격 추정 [s]
  std::vector<double> green_z_;   // 본 초록 점멸 높이 (height_color 기준, reset() 까지 유지)
  SignalResult result_;
};

}  // namespace perception
