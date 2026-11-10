// 신호등 점멸 순서 세기 시험 (판정 기준안 v1 U1 · U2, 영상 없음)
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "perception/signal_sequence.hpp"

using namespace perception;

namespace
{
constexpr double FPS = 30.0;

Light from_char(char c)
{
  return c == 'R' ? Light::Red : c == 'Y' ? Light::Yellow : c == 'G' ? Light::Green : Light::Off;
}

// 반복하는 신호등: 한 주기 = 12회 점멸(켜짐 t_on, 사이 꺼짐 t_off) + 텀
struct Blinker
{
  std::string seq;
  double t_on, t_off, t_term;
  double period() const {return seq.size() * t_on + (seq.size() - 1) * t_off + t_term;}
  Light at(double t) const
  {
    double u = std::fmod(t, period());
    for (char c : seq) {
      if (u < t_on) {return from_char(c);}
      u -= t_on;
      if (u < t_off) {return Light::Off;}
      u -= t_off;
    }
    return Light::Off;
  }
};

struct Watch
{
  bool confirmed = false;
  std::string sequence;
  double elapsed = 0;     // 보기 시작부터 확정까지 [s]
};

// start: 신호등 시각 기준 보기 시작, limit: 볼 시간. drop · glitch: 프레임 빠짐 · 잘못된 상태 확률
Watch watch(const Blinker & b, double start, double limit, std::mt19937 & rng, double drop = 0, double glitch = 0)
{
  SignalSequenceTracker tr;
  std::uniform_real_distribution<double> u(0, 1);
  std::uniform_int_distribution<int> other(1, 3);
  Watch r;
  for (int n = 0; n / FPS <= limit; ++n) {
    const double t = start + n / FPS;
    if (u(rng) < drop) {continue;}
    Light s = b.at(t);
    if (u(rng) < glitch) {
      s = static_cast<Light>((static_cast<int>(s) + other(rng)) % 4);
    }
    const auto & res = tr.update(t, s);
    if (res.status == SignalResult::CONFIRMED) {
      r.confirmed = true;
      r.sequence = res.sequence;
      r.elapsed = n / FPS;
      return r;
    }
  }
  return r;
}

std::string random_seq(std::mt19937 & rng)
{
  std::uniform_int_distribution<int> c(0, 2);
  std::string s;
  for (int i = 0; i < 12; ++i) {s += "RYG"[c(rng)];}
  return s;
}
}  // namespace

// U1: 무작위 1000회. 켜짐 · 꺼짐 0.1~1.0 s, 텀 max(3 × 꺼짐, 1.0)~5.0 s, 아무 때나 보기 시작,
//     잘못된 프레임 2 %, 빠진 프레임 5 %. 통과: 1000/1000 일치, 틀린 확정 0, 5주기 안에 확정
//     (켜짐 · 꺼짐이 몇 프레임뿐이면 잡음으로 점멸 정보 자체가 사라지는 묶음이 생겨 다음 묶음을 기다린다)
TEST(SignalSequence, U1_Random1000)
{
  std::mt19937 rng(20261023);
  std::uniform_real_distribution<double> on_off(0.1, 1.0), unit(0, 1);
  int ok = 0, wrong = 0, late = 0;
  for (int k = 0; k < 1000; ++k) {
    Blinker b{random_seq(rng), on_off(rng), on_off(rng), 0};
    const double lo = std::max(3 * b.t_off, 1.0);
    b.t_term = lo + unit(rng) * (5.0 - lo);
    const double P = b.period();
    const Watch r = watch(b, unit(rng) * P, 5 * P, rng, 0.05, 0.02);
    if (!r.confirmed) {
      ++late;
      ADD_FAILURE() << k << ": 5주기 안에 확정 못 함 " << b.seq << " on " << b.t_on << " off " << b.t_off
                    << " term " << b.t_term;
    } else if (r.sequence != b.seq) {
      ++wrong;
      ADD_FAILURE() << k << ": 틀린 확정 " << r.sequence << " (정답 " << b.seq << ")";
    } else {
      ++ok;
    }
  }
  RecordProperty("ok", ok);
  RecordProperty("wrong", wrong);
  RecordProperty("late", late);
  EXPECT_EQ(ok, 1000) << "틀린 확정 " << wrong << ", 확정 못 함 " << late;
}

// U2: 경계 상황
TEST(SignalSequence, U2_SameColorRuns)
{
  std::mt19937 rng(1);
  for (const std::string seq : {"RRRRRRRRRRRR", "RRGGYYRRGGYY", "GGGGGGRRRRYY", "YRRRRRRRRRRR"}) {
    Blinker b{seq, 0.3, 0.2, 1.5};
    const Watch r = watch(b, 0.05, 3 * b.period(), rng);
    EXPECT_TRUE(r.confirmed) << seq;
    EXPECT_EQ(r.sequence, seq);
  }
}

TEST(SignalSequence, U2_StartInsideOnAndInsideTerm)
{
  std::mt19937 rng(2);
  Blinker b{"RGRRGRGGYGYG", 0.4, 0.3, 2.0};
  const double P = b.period();
  // 확정 = 두 번째 묶음의 마지막 점멸 뒤 텀 판정(꺼짐 중앙값 × 2)까지
  // 첫 점멸이 켜진 중간에 보기 시작 → 첫 점멸도 색은 보이므로 첫 묶음부터 셈
  Watch r = watch(b, 0.2, 3 * P, rng);
  EXPECT_TRUE(r.confirmed);
  EXPECT_EQ(r.sequence, b.seq);
  EXPECT_LT(r.elapsed, 2 * P - b.t_term + 2 * b.t_off + 0.1);
  // 텀 중간에 보기 시작 → 첫 묶음부터 점멸 수가 맞아 두 묶음째 끝(텀 판정 시간 포함)에 확정
  r = watch(b, P - b.t_term + 0.1, 3 * P, rng);
  EXPECT_TRUE(r.confirmed);
  EXPECT_EQ(r.sequence, b.seq);
  EXPECT_LT(r.elapsed, 2 * P + 2 * b.t_off + 0.1);
}

TEST(SignalSequence, U2_DifferentCyclesDoNotConfirm)
{
  // 묶음 A, B, A 순서로 바뀌는 신호등: A · B 만 봤을 때는 확정하지 않고(서로 다름), A 가 두 번 나오면 확정
  const std::string A = "RGRRGRGGYGYG", B = "GGGGRRRRYYYY";
  Blinker a{A, 0.3, 0.2, 1.5}, bb{B, 0.3, 0.2, 1.5};
  const double P = a.period();
  SignalSequenceTracker tr;
  bool saw_mismatch = false;
  for (int n = 0; n / FPS < 4 * P; ++n) {
    const double t = n / FPS;
    const int cycle = static_cast<int>(t / P);
    const Light s = (cycle == 1 ? bb : a).at(t);
    const auto & res = tr.update(t, s);
    if (t < 3 * P - a.t_term) {
      ASSERT_NE(res.status, SignalResult::CONFIRMED) << t;     // 세 번째 묶음(A)이 끝나기 전
    }
    saw_mismatch = saw_mismatch || res.status == SignalResult::MISMATCH;
  }
  EXPECT_TRUE(saw_mismatch);
  EXPECT_EQ(tr.result().status, SignalResult::CONFIRMED);
  EXPECT_EQ(tr.result().sequence, A);
}

TEST(SignalSequence, U2_LostFramesKeepsCompletedCycles)
{
  // 첫 묶음을 다 본 뒤 1 s 동안 신호등을 놓침 → 놓친 묶음은 버리고 그다음 묶음과 맞춰 확정
  Blinker b{"RGRRGRGGYGYG", 0.3, 0.3, 2.0};
  const double P = b.period();
  SignalSequenceTracker tr;
  const double lost_from = P + 1.0;   // 두 번째 묶음 중간
  for (int n = 0; n / FPS < 4 * P; ++n) {
    const double t = (P - b.t_term + 0.1) + n / FPS;     // 텀 중간부터 봄
    if (t > lost_from && t < lost_from + 1.0) {continue;}
    tr.update(t, b.at(t));
    if (tr.result().status == SignalResult::CONFIRMED) {break;}
  }
  EXPECT_EQ(tr.result().status, SignalResult::CONFIRMED);
  EXPECT_EQ(tr.result().sequence, b.seq);
}

TEST(SignalSequence, U2_NothingSeen)
{
  SignalSequenceTracker tr;
  EXPECT_EQ(tr.result().status, SignalResult::SEARCHING);
  for (int n = 0; n < 300; ++n) {tr.update(n / FPS, Light::Off);}
  EXPECT_EQ(tr.result().status, SignalResult::SEARCHING);
  EXPECT_TRUE(tr.result().sequence.empty());
}

TEST(SignalSequence, U2_MeasuredTimesAndCounts)
{
  std::mt19937 rng(3);
  Blinker b{"RGRRGRGGYGYG", 0.5, 0.25, 2.0};
  SignalSequenceTracker tr;
  for (int n = 0; n / FPS < 3 * b.period(); ++n) {
    tr.update(n / FPS, b.at(n / FPS));
  }
  const auto & r = tr.result();
  ASSERT_EQ(r.status, SignalResult::CONFIRMED);
  EXPECT_EQ(r.red, 4);
  EXPECT_EQ(r.yellow, 2);
  EXPECT_EQ(r.green, 6);
  // 잰 길이: 프레임 경계만큼 어긋날 수 있음. 텀은 확정 뒤 다음 점멸이 끝나야 알 수 있다
  EXPECT_NEAR(r.t_on, b.t_on, 1.5 / FPS);
  EXPECT_NEAR(r.t_off, b.t_off, 1.5 / FPS);
  EXPECT_NEAR(r.t_term, b.t_term, 1.5 / FPS);
}

// 텀 길이: 확정 순간에는 아직 모를 수 있지만, 그 뒤 다음 점멸을 보면 채워져야 한다(확정 뒤에도 갱신).
// U1 과 같은 범위 · 잡음으로 200회, 5주기 본 뒤 잰 텀. 통과(사용자 2026-10-24): 99 % 이상이 정답과 3프레임 안,
// 나머지도 0.5 s 안. 100 % 가 아닌 이유: 텀 끝에 같은 색 잘못된 프레임이 2장 겹치면(3프레임 다수결을 넘음) 다음 점멸에
// 합쳐져 텀이 짧게 재지는 경우가 있다(첫 실행 199/200, 0.46 s). 텀은 순서 확정에 쓰지 않는 참고 값이다
TEST(SignalSequence, U2_TermMeasuredAfterConfirm)
{
  std::mt19937 rng(20261024);
  std::uniform_real_distribution<double> on_off(0.1, 1.0), unit(0, 1);
  std::uniform_int_distribution<int> other(1, 3);
  int ok = 0, near = 0;
  for (int k = 0; k < 200; ++k) {
    Blinker b{random_seq(rng), on_off(rng), on_off(rng), 0};
    const double lo = std::max(3 * b.t_off, 1.0);
    b.t_term = lo + unit(rng) * (5.0 - lo);
    const double P = b.period(), start = unit(rng) * P;
    SignalSequenceTracker tr;
    for (int n = 0; n / FPS <= 5 * P; ++n) {
      if (unit(rng) < 0.05) {continue;}
      Light s = b.at(start + n / FPS);
      if (unit(rng) < 0.02) {s = static_cast<Light>((static_cast<int>(s) + other(rng)) % 4);}
      tr.update(start + n / FPS, s);
    }
    const auto & r = tr.result();
    const double err = std::abs(r.t_term - b.t_term);
    if (r.status == SignalResult::CONFIRMED && err <= 3.0 / FPS) {
      ++ok;
    } else if (r.status == SignalResult::CONFIRMED && err <= 0.5) {
      ++near;
      std::cout << k << ": 텀 " << r.t_term << " (정답 " << b.t_term << ") on " << b.t_on << " off " << b.t_off << "\n";
    } else {
      ADD_FAILURE() << k << ": 상태 " << int(r.status) << " 텀 " << r.t_term << " (정답 " << b.t_term
                    << ") on " << b.t_on << " off " << b.t_off;
    }
  }
  RecordProperty("within_3_frames", ok);
  RecordProperty("within_0_5s", near);
  EXPECT_GE(ok, 198) << "3프레임 안 " << ok << "/200";
}

// ---------------- 높이 판별 (height_color, 실물 2026-11-10 P5c) ----------------
//   실물 노출 80 에서 빨간 등이 노랑 색상으로 찍힘. 12 회가 빨강 둘로 시작하고 초록은 맨 끝인 순서에서
//   첫 묶음 앞 빨강 2 개를 노랑으로 읽었음 → 초록을 본 뒤 지난 점멸도 높이로 다시 읽어야 함
namespace
{
constexpr double PITCH = 0.0349, Z_GREEN = 0.056;

SignalConfig height_cfg()
{
  SignalConfig c;
  c.height_color = true;
  c.segment_pitch = PITCH;
  return c;
}

struct HeightRun
{
  bool confirmed = false;
  std::string sequence;
  double elapsed = 0;       // 확정까지 [s]
  int cycles = 0;           // 확정 때 본 묶음 수
  std::vector<std::pair<int, std::string>> seen;   // (묶음 수, 상태별 순서) 바뀔 때마다
};

// 빨강 · 노랑을 모두 노랑 색상으로 넣고(포화) 높이로만 가를 수 있게 한다.
//   z_of(묶음 번호, 점멸 번호, 색) 로 켜진 등 높이, NaN 이면 높이 없음
template<class ZOf>
HeightRun watch_height(const Blinker & b, double limit, SignalConfig cfg, ZOf z_of)
{
  SignalSequenceTracker tr(cfg);
  HeightRun r;
  int last_status = -1;
  for (int n = 0; n / FPS <= limit; ++n) {
    const double t = n / FPS;
    const Light l = b.at(t);
    double z = std::numeric_limits<double>::quiet_NaN();
    Light in = l;
    if (l != Light::Off) {
      double u = std::fmod(t, b.period());
      const int k = std::min(static_cast<int>(u / (b.t_on + b.t_off)), static_cast<int>(b.seq.size()) - 1);
      z = z_of(static_cast<int>(t / b.period()), k, l);
      if (l == Light::Red) {in = Light::Yellow;}
    }
    const auto & res = tr.update(t, in, z);
    if (static_cast<int>(res.status) != last_status) {
      last_status = static_cast<int>(res.status);
      r.seen.push_back({res.cycles, std::to_string(last_status) + " " + res.sequence});
    }
    if (res.status == SignalResult::CONFIRMED) {
      r.confirmed = true;
      r.sequence = res.sequence;
      r.elapsed = t;
      r.cycles = res.cycles;
      break;
    }
  }
  return r;
}

double seg_z(Light l) {return Z_GREEN + (l == Light::Green ? 0 : l == Light::Yellow ? 1 : 2) * PITCH;}
}  // namespace

// 실물에서 본 순서 그대로: 첫 묶음에서 바로 맞는 순서로 확정 (빨강 둘을 노랑으로 남기지 않음)
TEST(SignalHeight, RelabelsEarlyRedOnceGreenSeen)
{
  const Blinker b{"RRYYYRYYRRYG", 1.0, 1.0, 3.0};
  const auto r = watch_height(b, 4 * b.period(), height_cfg(), [](int, int, Light l) {return seg_z(l) + 0.002;});
  ASSERT_TRUE(r.confirmed);
  EXPECT_EQ(r.sequence, "RRYYYRYYRRYG");
  EXPECT_EQ(r.cycles, 1);
  EXPECT_LT(r.elapsed, b.period() + 1.0) << "첫 묶음 텀이 끝나면 확정";
}

// 첫 묶음에 경계 근처(애매) 점멸이 하나 있으면 한 묶음 더 보고, 둘째 묶음이 분명하면 그것으로 확정 (최대 두 번)
TEST(SignalHeight, AmbiguousFirstCycleLooksOnceMore)
{
  const Blinker b{"RGYRYYGRRYGY", 1.0, 1.0, 3.0};
  const auto r = watch_height(b, 5 * b.period(), height_cfg(), [](int cyc, int k, Light l) {
      return cyc == 0 && k == 3 ? Z_GREEN + 1.45 * PITCH : seg_z(l);   // 첫 묶음 4 번째 빨강이 경계 바로 아래
    });
  ASSERT_TRUE(r.confirmed);
  EXPECT_EQ(r.sequence, "RGYRYYGRRYGY");
  EXPECT_EQ(r.cycles, 2) << "애매한 첫 묶음 뒤 둘째 묶음에서 확정";
  EXPECT_LT(r.elapsed, 2 * b.period() + 1.0);
}

// 초록이 없는 묶음: 따뜻한 색 높이가 두 층으로 갈리면 위층 = 빨강, 한 번 보고 확정
TEST(SignalHeight, NoGreenUsesWarmSpread)
{
  const Blinker b{"RYRYRRYYRYRY", 1.0, 1.0, 3.0};
  const auto r = watch_height(b, 4 * b.period(), height_cfg(), [](int, int, Light l) {return seg_z(l);});
  ASSERT_TRUE(r.confirmed);
  EXPECT_EQ(r.sequence, "RYRYRRYYRYRY");
  EXPECT_EQ(r.cycles, 1);
}

// 높이를 모르면(z 없음) 애매 → 색상 그대로, 같은 순서 두 번이어야 확정 (예전 규칙)
TEST(SignalHeight, NoHeightFallsBackToTwoSameCycles)
{
  const Blinker b{"RGYRYYGRRYGY", 1.0, 1.0, 3.0};
  const auto r = watch_height(b, 5 * b.period(), height_cfg(), [](int, int, Light) {return std::numeric_limits<double>::quiet_NaN();});
  ASSERT_TRUE(r.confirmed);
  EXPECT_EQ(r.cycles, 2);
  EXPECT_EQ(r.sequence, "YGYYYYGYYYGY") << "빨강 색상이 노랑으로 찍히면 높이 없이는 고칠 수 없음 (알려진 한계)";
}

// 한 번 확정을 끄면(confirm_clear_cycles 0) 분명해도 두 번 봐야 확정
TEST(SignalHeight, ClearConfirmCanBeTurnedOff)
{
  const Blinker b{"RRYYYRYYRRYG", 1.0, 1.0, 3.0};
  auto cfg = height_cfg();
  cfg.confirm_clear_cycles = 0;
  const auto r = watch_height(b, 4 * b.period(), cfg, [](int, int, Light l) {return seg_z(l);});
  ASSERT_TRUE(r.confirmed);
  EXPECT_EQ(r.sequence, "RRYYYRYYRRYG");
  EXPECT_EQ(r.cycles, 2);
}

// 점멸 사이 꺼짐이 짧아 켜진 채 빨강 → 노랑 색상이 바뀌어도(높이 판별 중) 따뜻한 색 한 점멸로 세지 않고 높이로 가름
TEST(SignalHeight, HueFlickerInsideOneBlinkIsOneBlink)
{
  const Blinker b{"RRYYYRYYRRYG", 1.0, 1.0, 3.0};
  SignalSequenceTracker tr(height_cfg());
  SignalResult res;
  for (int n = 0; n / FPS <= 2 * b.period(); ++n) {
    const double t = n / FPS;
    const Light l = b.at(t);
    Light in = l;
    if (l == Light::Red) {in = (n / 4) % 2 ? Light::Red : Light::Yellow;}   // 4 프레임마다 색상이 빨강 · 노랑 오감
    res = tr.update(t, in, l == Light::Off ? std::numeric_limits<double>::quiet_NaN() : seg_z(l));
    if (res.status == SignalResult::CONFIRMED) {break;}
  }
  EXPECT_EQ(res.status, SignalResult::CONFIRMED);
  EXPECT_EQ(res.sequence, "RRYYYRYYRRYG");
}
