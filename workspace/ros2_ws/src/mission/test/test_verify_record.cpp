// 장소 재검증 기록 · 전체 중복/누락 판정 시험 (verify_record.hpp, 영상 없음)
//   장소 3 곳에 주사위 눈 4 · 6 · 2 (공개 연습 문제 Q1 의 R · G · Y 횟수) 로 퍽 12 개를 놓은 상태를 만든다
#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "mission/verify_record.hpp"

using namespace mission;

namespace
{
constexpr double GRID = 0.06, TOP = 0.07, PUCK_Z = 0.10;   // 칸 간격 · 상자 윗면 (verify z) · 선 퍽 중심 높이 [m]
const std::vector<std::vector<int>> PIPS = {
  {}, {4}, {0, 8}, {0, 4, 8}, {0, 2, 6, 8}, {0, 2, 4, 6, 8}, {0, 3, 6, 2, 5, 8}};

VerifyRecorder::Cells cells_at(double cx, double cy)
{
  VerifyRecorder::Cells c{};
  for (int k = 0; k < 9; ++k) {
    c[k] = {cx + (k % 3 - 1) * GRID, cy + (1 - k / 3) * GRID, TOP};   // mission_node 와 같은 번호 (위 줄 +Y, 줄 안은 -X 부터)
  }
  return c;
}

struct Placed
{
  int cell, id;
};

// 한 장소를 n 장 본다. 퍽마다 read_every 장에 한 번만 태그를 읽음 (나머지는 ID −1), 위치는 ±jitter 흔들림
void observe(VerifyRecorder & rec, const VerifyRecorder::Cells & c, const std::vector<Placed> & placed, int frames,
  int read_every, std::mt19937 & rng, double jitter = 0.004)
{
  std::uniform_real_distribution<double> u(-jitter, jitter);
  for (int f = 0; f < frames; ++f) {
    std::vector<VerifyObs> obs;
    for (const auto & p : placed) {
      VerifyObs o;
      o.x = c[p.cell][0] + u(rng);
      o.y = c[p.cell][1] + u(rng);
      o.z = PUCK_Z;
      o.mx = o.x;
      o.my = o.y;
      o.tag_id = (f % read_every == 0) ? p.id : -1;
      obs.push_back(o);
    }
    rec.add_frame(obs);
  }
}

std::vector<int> all_ids() {return {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};}

// 세 장소 (L1 · L2 · L3) 에 ID 0 ~ 11 을 눈 4 · 6 · 2 로 놓은 배치
std::map<std::string, std::vector<Placed>> q1_layout()
{
  std::map<std::string, std::vector<Placed>> m;
  int id = 0;
  const std::map<std::string, int> pips = {{"L1", 4}, {"L2", 6}, {"L3", 2}};
  for (const auto & [place, n] : pips) {
    for (int cell : PIPS[n]) {m[place].push_back({cell, id++});}
  }
  return m;
}
const std::map<std::string, std::pair<double, double>> CENTER = {{"L1", {0.8, 3.7}}, {"L2", {2.202, 3.7}}, {"L3", {3.6, 1.5}}};
}  // namespace

TEST(VerifyRecord, AllTwelveRecordedAndJudgedOk)
{
  std::mt19937 rng(7);
  std::map<std::string, std::vector<VerifyPuck>> by_place;
  for (const auto & [place, placed] : q1_layout()) {
    VerifyRecorder rec;
    const auto c = cells_at(CENTER.at(place).first, CENTER.at(place).second);
    rec.start(c, 0.04);
    observe(rec, c, placed, 20, 3, rng);   // 20 장 중 7 장만 태그를 읽음
    const auto r = rec.records(2);
    auto want = placed;   // 기록은 칸 번호 순
    std::sort(want.begin(), want.end(), [](const Placed & a, const Placed & b) {return a.cell < b.cell;});
    ASSERT_EQ(r.size(), want.size()) << place;
    for (size_t i = 0; i < r.size(); ++i) {
      EXPECT_EQ(r[i].cell, want[i].cell) << place;
      EXPECT_EQ(r[i].tag_id, want[i].id) << place;
      EXPECT_EQ(r[i].seen, 7) << place;
      EXPECT_LT(r[i].offset, 0.006) << place;   // 흔들림 ±4 mm 의 평균
      EXPECT_NEAR(r[i].mx, c[r[i].cell][0], 0.004);
    }
    EXPECT_EQ(rec.frames(), 20);
    by_place[place] = r;
  }
  const auto j = judge_ids(by_place, all_ids());
  EXPECT_TRUE(j.passed);
  EXPECT_EQ(j.total, 12);
  EXPECT_EQ(j.unread, 0);
  EXPECT_TRUE(j.duplicate.empty() && j.missing.empty() && j.unexpected.empty());
}

TEST(VerifyRecord, DuplicateAcrossPlaces)
{
  std::mt19937 rng(1);
  auto layout = q1_layout();
  layout["L3"][0].id = 0;   // L1 에도 있는 ID 0 을 L3 에서도 봄 (원래 ID 10 은 못 봄)
  std::map<std::string, std::vector<VerifyPuck>> by_place;
  for (const auto & [place, placed] : layout) {
    VerifyRecorder rec;
    const auto c = cells_at(CENTER.at(place).first, CENTER.at(place).second);
    rec.start(c, 0.04);
    observe(rec, c, placed, 10, 1, rng);
    by_place[place] = rec.records(2);
  }
  const auto j = judge_ids(by_place, all_ids());
  EXPECT_FALSE(j.passed);
  EXPECT_EQ(j.duplicate, std::vector<int>({0}));
  EXPECT_EQ(j.missing, std::vector<int>({10}));
}

TEST(VerifyRecord, UnreadTagIsKeptAsMinusOneAndCountsAsMissing)
{
  std::mt19937 rng(2);
  auto layout = q1_layout();
  std::map<std::string, std::vector<VerifyPuck>> by_place;
  for (const auto & [place, placed] : layout) {
    VerifyRecorder rec;
    const auto c = cells_at(CENTER.at(place).first, CENTER.at(place).second);
    rec.start(c, 0.04);
    std::vector<Placed> p = placed;
    if (place == "L2") {p[5].id = -1;}   // L2 마지막 퍽 (ID 9) 은 태그를 한 번도 못 읽음 (화면 끝에 잘림 등)
    observe(rec, c, p, 10, 1, rng);
    by_place[place] = rec.records(2);
  }
  ASSERT_EQ(by_place["L2"].size(), 6u);
  EXPECT_EQ(by_place["L2"][5].tag_id, -1);
  EXPECT_EQ(by_place["L2"][5].cell, PIPS[6][5]);
  const auto j = judge_ids(by_place, all_ids());
  EXPECT_FALSE(j.passed);
  EXPECT_EQ(j.unread, 1);
  EXPECT_EQ(j.missing, std::vector<int>({9}));
  EXPECT_TRUE(j.duplicate.empty());
}

TEST(VerifyRecord, RareMisreadBelowMinSeenIgnored)
{
  std::mt19937 rng(3);
  VerifyRecorder rec;
  const auto c = cells_at(1.0, 2.0);
  rec.start(c, 0.04);
  observe(rec, c, {{4, 5}}, 10, 1, rng);
  rec.add_frame({{c[4][0], c[4][1], PUCK_Z, 0, 0, 7}});   // 한 장만 다른 번호로 읽힘
  const auto r = rec.records(2);
  ASSERT_EQ(r.size(), 1u);
  EXPECT_EQ(r[0].tag_id, 5);
  EXPECT_EQ(rec.records(1).size(), 2u);   // min_seen 1 이면 한 장 오독도 남음
}

TEST(VerifyRecord, OutsideRegionAndHeightFiltered)
{
  VerifyRecorder rec;
  const auto c = cells_at(1.0, 2.0);
  rec.start(c, 0.04);
  for (int f = 0; f < 5; ++f) {
    rec.add_frame({
      {c[0][0] - 0.11, c[0][1], PUCK_Z, 0, 0, 1},   // 칸 범위 + 0.04 밖 (0.11 > 0.06 + 0.04)
      {c[4][0], c[4][1], 0.03, 0, 0, 2},            // 바닥 퍽 (상자 윗면 0.07 − 0.02 아래)
      {c[4][0], c[4][1], 0.25, 0, 0, 3},            // 집게 속 퍽 (윗면 + 0.10 위)
      {c[8][0] + 0.03, c[8][1], PUCK_Z, 0, 0, 4}}); // 범위 안 (칸 8 에 넣음)
  }
  const auto r = rec.records(2);
  ASSERT_EQ(r.size(), 1u);
  EXPECT_EQ(r[0].tag_id, 4);
  EXPECT_EQ(r[0].cell, 8);
  EXPECT_NEAR(r[0].offset, 0.03, 1e-9);
}

TEST(VerifyRecord, UnexpectedIdReported)
{
  std::map<std::string, std::vector<VerifyPuck>> by_place;
  VerifyPuck p;
  for (int id : all_ids()) {p.tag_id = id; by_place["L1"].push_back(p);}
  p.tag_id = 13;   // 장소 태그 번호가 퍽 목록에 섞인 경우
  by_place["L2"].push_back(p);
  const auto j = judge_ids(by_place, all_ids());
  EXPECT_FALSE(j.passed);
  EXPECT_EQ(j.unexpected, std::vector<int>({13}));
  EXPECT_TRUE(j.missing.empty());
  EXPECT_TRUE(j.duplicate.empty());
}

TEST(VerifyRecord, InactiveOrRestartedRecorderKeepsNoOldFrames)
{
  VerifyRecorder rec;
  const auto c = cells_at(1.0, 2.0);
  EXPECT_EQ(rec.add_frame({{c[4][0], c[4][1], PUCK_Z, 0, 0, 1}}), 0);   // start 전
  rec.start(c, 0.04);
  rec.add_frame({{c[4][0], c[4][1], PUCK_Z, 0, 0, 1}});
  rec.add_frame({{c[4][0], c[4][1], PUCK_Z, 0, 0, 1}});
  rec.stop();
  EXPECT_EQ(rec.add_frame({{c[4][0], c[4][1], PUCK_Z, 0, 0, 1}}), 0);   // stop 뒤
  EXPECT_EQ(rec.records(2).size(), 1u);
  rec.start(c, 0.04);   // 같은 장소 재시도: 이전 관측을 지움
  EXPECT_TRUE(rec.records(1).empty());
  EXPECT_EQ(rec.frames(), 0);
}
