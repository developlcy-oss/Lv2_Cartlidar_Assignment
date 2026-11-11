// 장소 재검증 기록 (ROS 의존 없음 → 단위 시험). 발제 06 쪽 "이동 후 검증": 새 관측 기반 장소별 퍽 ID · 위치 + 전체 중복/누락 판정
//   VerifyRecorder: verify 한 번 동안 /perception/pucks 를 받을 때마다 add_frame. 퍽마다 가장 가까운 칸에 넣고
//     (칸, 태그 ID) 별로 본 횟수 · 위치 합을 쌓는다. 같은 퍽이 어떤 장은 태그를 읽고 어떤 장은 못 읽어도 칸으로 묶인다
//   records(min_seen): (칸, ID) 가 min_seen 번 이상 보인 것만. 태그를 한 번도 못 읽은 퍽이 있는 칸은 ID −1 로 남긴다
//   judge_ids: 모든 장소 기록을 모아 ID 중복 (두 장소 이상, 한 장소 안 여러 칸은 records 가 합침) · 누락 (기대 ID 를 못 봄) · 기대 밖 ID 를 낸다
//   칸 범위 · 높이 거르기는 perception check_dice 와 같다 (칸 범위 + margin, 칸 높이 −0.02 ~ +0.10 m)
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace mission
{

// 한 장의 퍽 하나: 칸 좌표계 (x, y, z) 와 지도 좌표 (mx, my). tag_id −1 = 태그 못 읽음
struct VerifyObs
{
  double x = 0, y = 0, z = 0;
  double mx = 0, my = 0;
  int tag_id = -1;
};

// 칸 하나에서 본 퍽 하나의 기록
struct VerifyPuck
{
  int cell = 0;          // 0 ~ 8 (CheckDice 와 같은 행 우선 번호)
  int tag_id = -1;       // −1 = 이 칸에 퍽은 있었지만 태그를 한 번도 못 읽음
  int seen = 0;          // 이 (칸, ID) 로 본 횟수
  double x = 0, y = 0;   // 칸 좌표계 평균
  double mx = 0, my = 0; // 지도 평균
  double offset = 0;     // 칸 중심과의 수평 거리 [m]
  std::vector<int> split_cells;   // 같은 ID 를 이 칸 말고도 본 칸 (한 퍽으로 합침, 경계 근처 퍽)
};

class VerifyRecorder
{
public:
  using Cells = std::array<std::array<double, 3>, 9>;   // 칸 중심 (x, y, z), 칸 좌표계

  void start(const Cells & cells, double margin)
  {
    cells_ = cells;
    margin_ = margin;
    acc_.clear();
    frames_ = 0;
    active_ = true;
    x0_ = y0_ = z0_ = std::numeric_limits<double>::infinity();
    x1_ = y1_ = z1_ = -std::numeric_limits<double>::infinity();
    for (const auto & c : cells_) {
      x0_ = std::min(x0_, c[0]); x1_ = std::max(x1_, c[0]);
      y0_ = std::min(y0_, c[1]); y1_ = std::max(y1_, c[1]);
      z0_ = std::min(z0_, c[2]); z1_ = std::max(z1_, c[2]);
    }
  }
  void stop() {active_ = false;}
  bool active() const {return active_;}
  int frames() const {return frames_;}

  // 퍽 목록 한 장. 칸 범위 밖 퍽은 버린다. 돌려주는 값 = 이 장에서 칸에 넣은 퍽 수
  int add_frame(const std::vector<VerifyObs> & obs)
  {
    if (!active_) {return 0;}
    ++frames_;
    int kept = 0;
    for (const auto & o : obs) {
      if (o.x < x0_ - margin_ || o.x > x1_ + margin_ || o.y < y0_ - margin_ || o.y > y1_ + margin_) {continue;}
      if (o.z < z0_ - 0.02 || o.z > z1_ + 0.10) {continue;}
      int k = 0;
      double dk = std::numeric_limits<double>::infinity();
      for (int i = 0; i < 9; ++i) {
        const double d = std::hypot(o.x - cells_[i][0], o.y - cells_[i][1]);
        if (d < dk) {dk = d; k = i;}
      }
      auto & a = acc_[{k, o.tag_id < 0 ? -1 : o.tag_id}];
      ++a.n;
      a.x += o.x; a.y += o.y; a.mx += o.mx; a.my += o.my;
      ++kept;
    }
    return kept;
  }

  // 칸 번호 · ID 순. 태그를 읽은 기록이 있는 칸의 ID −1 관측은 같은 퍽으로 보고 뺀다
  //   같은 ID 가 한 장소의 여러 칸에 나뉘면 (퍽이 칸 경계 근처라 장마다 가까운 칸이 바뀜, #172) 한 퍽으로 합친다:
  //   가장 많이 본 칸에 두고 본 횟수 · 위치는 합쳐 평균, 나머지 칸은 split_cells 로 남긴다. min_seen 은 합친 횟수로 본다
  std::vector<VerifyPuck> records(int min_seen) const
  {
    std::vector<VerifyPuck> out;
    std::array<bool, 9> tagged{};
    std::map<int, std::vector<std::pair<int, const Acc *>>> by_id;   // ID -> [(칸, 누적)]
    for (const auto & [key, a] : acc_) {
      if (key.second < 0) {continue;}
      if (a.n >= min_seen) {tagged[key.first] = true;}
      by_id[key.second].push_back({key.first, &a});
    }
    for (const auto & [key, a] : acc_) {   // ID −1: 칸마다 따로
      if (key.second >= 0 || a.n < min_seen || tagged[key.first]) {continue;}
      out.push_back(make(key.first, -1, a, {}));
    }
    for (const auto & [id, parts] : by_id) {
      Acc sum;
      int best = parts.front().first, best_n = 0;
      for (const auto & [cell, a] : parts) {
        sum.n += a->n;
        sum.x += a->x; sum.y += a->y; sum.mx += a->mx; sum.my += a->my;
        if (a->n > best_n) {best_n = a->n; best = cell;}   // 같으면 칸 번호 작은 쪽 (map 순서)
      }
      if (sum.n < min_seen) {continue;}
      std::vector<int> split;
      for (const auto & [cell, a] : parts) {
        if (cell != best) {split.push_back(cell);}
      }
      out.push_back(make(best, id, sum, split));
    }
    std::sort(out.begin(), out.end(), [](const VerifyPuck & a, const VerifyPuck & b) {
        return a.cell != b.cell ? a.cell < b.cell : a.tag_id < b.tag_id;
      });
    return out;
  }

private:
  struct Acc
  {
    int n = 0;
    double x = 0, y = 0, mx = 0, my = 0;
  };
  VerifyPuck make(int cell, int id, const Acc & a, std::vector<int> split) const
  {
    VerifyPuck p;
    p.cell = cell;
    p.tag_id = id;
    p.seen = a.n;
    p.x = a.x / a.n; p.y = a.y / a.n;
    p.mx = a.mx / a.n; p.my = a.my / a.n;
    p.offset = std::hypot(p.x - cells_[cell][0], p.y - cells_[cell][1]);
    p.split_cells = std::move(split);
    return p;
  }
  Cells cells_{};
  double margin_ = 0.04;
  double x0_ = 0, x1_ = 0, y0_ = 0, y1_ = 0, z0_ = 0, z1_ = 0;
  std::map<std::pair<int, int>, Acc> acc_;
  int frames_ = 0;
  bool active_ = false;
};

// 전체 판정 (장소 기록을 모두 모아서)
struct IdJudgement
{
  bool passed = false;               // 중복 · 누락 · 기대 밖 ID 가 모두 없음
  std::vector<int> duplicate;        // 두 칸 이상에서 본 ID
  std::vector<int> missing;          // 기대 ID 중 어느 칸에서도 못 본 것
  std::vector<int> unexpected;       // 기대 목록에 없는 ID
  int unread = 0;                    // 퍽은 있었지만 태그를 못 읽은 칸 수 (누락의 원인일 수 있음)
  int total = 0;                     // ID 를 읽은 기록 수
};

inline IdJudgement judge_ids(
  const std::map<std::string, std::vector<VerifyPuck>> & by_place, const std::vector<int> & expected)
{
  IdJudgement j;
  std::map<int, int> count;
  for (const auto & [place, recs] : by_place) {
    (void)place;
    for (const auto & r : recs) {
      if (r.tag_id < 0) {++j.unread; continue;}
      ++count[r.tag_id];
      ++j.total;
    }
  }
  for (const auto & [id, n] : count) {
    if (n > 1) {j.duplicate.push_back(id);}
    if (std::find(expected.begin(), expected.end(), id) == expected.end()) {j.unexpected.push_back(id);}
  }
  std::vector<int> want(expected);
  std::sort(want.begin(), want.end());
  want.erase(std::unique(want.begin(), want.end()), want.end());
  for (int id : want) {
    if (!count.count(id)) {j.missing.push_back(id);}
  }
  j.passed = j.duplicate.empty() && j.missing.empty() && j.unexpected.empty();
  return j;
}

}  // namespace mission
