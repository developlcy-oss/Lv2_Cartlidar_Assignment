// ARM_ALIGN 목표 퍽 유지 (팔 카메라 검출 중 따라갈 박스 하나, ROS 의존 없음 → 단위 시험)
//
//   좌표는 image_offset(geometry.hpp)과 같은 정규화 값: x · y 화면 중심 대비 −1~1, w = 박스 폭 / 화면 폭
//   거리는 화소 비율이 맞게 x 단위(화면 반폭)로 잰다: hypot(dx, dy × aspect), aspect = 높이 / 폭. 박스 폭은 같은 단위로 2w
//   1) 잠그기: 잠기지 않았으면 seed(D435 추적 목표를 팔 영상에 투영한 점, 화면 밖이어도 됨)에 가장 가까운 박스(거리 ≤ seed_gate).
//      seed 가 있는데 seed_gate 안 박스가 없으면 이번에는 잠그지 않는다(목표가 안 잡힌 장에 옆 퍽을 잠그지 않게).
//      seed 가 없으면 점수가 가장 높은 박스
//   2) 따라가기(직전 장에 보였음): 점수를 보지 않고, 같은 클래스 · 폭이 직전의 1/size_ratio ~ size_ratio 배
//      (한 장 사이라 좁게: 라벨이 화면 끝에 잘려 이웃과 한 덩어리가 된 상자를 따라가지 않게) ·
//      직전 중심 또는 한 장 앞 예측(직전 + 직전 이동)에서 min(gate, follow_k × 직전 폭) 안인 박스 중 가장 가까운 것.
//      문턱을 박스 폭에 비례시켜 맞닿은 이웃(중심 사이 약 퍽 폭 하나 이상)으로 넘어가지 않고, 예측으로 빠른 이동을 따라간다
//   3) 놓침: 맞는 박스가 없으면 -1(정렬 값 없음). 다시 잡을 때는 폭 reacquire_size_ratio 배 안, seed 가 있으면 1) 과 같이 seed_gate 안만
//      (목표가 화면 밖으로 나간 동안 이웃이 그 자리로 들어와도 잡지 않게), 없으면 직전 자리 근처(2 의 문턱)
//   4) 풀기: reset() — perception_node 는 ARM_ALIGN 을 켜거나 끌 때 · OFF · 팔 영상 시각 재시작에서 부른다
#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace perception
{

struct ArmBox
{
  double x = 0, y = 0, w = 0;   // 정규화 중심 · 폭
  int cls = -1;
  double score = 0;
  double h = 0;              // 박스 높이 / 화면 높이(0이면 기존 호출)
  double size = 0;           // 회전영향을줄인검출크기/화면폭 (0이면 w)
};

struct ArmLockConfig
{
  double gate = 0.25;        // ★ 따라가기 문턱 상한 (x 단위 거리)
  double seed_gate = 0.10;   // ★ 처음 잠글 때 투영점에서 이 안만 (D435 1° · 깊이 면 오차가 팔 0.25 m 에서 약 0.07)
  double follow_k = 0.6;     // ★ 따라가기 문턱 = follow_k × 직전 박스 폭 (gate 를 넘지 않음)
  double size_ratio = 1.3;   // ★ 따라가는 동안 한 장 사이 박스 폭 변화 허용 배율 (다가갈 때 한 장 사이 약 1.06 배)
  double reacquire_size_ratio = 2.0;   // ★ 놓친 뒤 다시 잡을 때 직전 폭 대비 허용 배율
};

struct ArmPoint
{
  double x = 0, y = 0;
  double depth = 0;          // 목표 카메라 z(m), 0이면 크기예측 안 함
};

class ArmTargetLock
{
public:
  explicit ArmTargetLock(ArmLockConfig c = {})
  : c_(c) {}

  void reset() {locked_ = false; moving_ = false; seen_last_ = false; reference_w_ = 0; reference_depth_ = 0; last_depth_ = 0; last_seed_.reset();}
  bool locked() const {return locked_;}

  // 따라갈 박스 번호, 없으면 -1
  int update(const std::vector<ArmBox> & boxes, const std::optional<ArmPoint> & seed, double aspect = 0.75)
  {
    auto dist = [aspect](double dx, double dy) {return std::hypot(dx, dy * aspect);};
    int seed_candidates = 0;
    double seed_best = 1e9, seed_second = 1e9;
    int best = -1;
    double best_d = 0;
    const double follow = std::min(c_.gate, c_.follow_k * 2.0 * last_.w);
    for (size_t i = 0; i < boxes.size(); ++i) {
      const ArmBox & b = boxes[i];
      double d;
      if (locked_) {
        const double r = seen_last_ ? c_.size_ratio : c_.reacquire_size_ratio;
        const double depth = seed ? seed->depth : 0;
        const double last_size = last_.size > 0 ? last_.size : last_.w;
        double size_base = last_size;
        if (depth > 0 && last_depth_ > 0) {size_base *= last_depth_ / depth;}
        if (!seen_last_ && reference_w_ > 0) {
          double full_size = reference_w_;
          if (depth > 0 && reference_depth_ > 0) {full_size *= reference_depth_ / depth;}
          size_base = std::max(size_base, full_size);
        }
        const double observed_size = b.size > 0 ? b.size : b.w;
        if (b.cls != last_.cls || observed_size < size_base / r || observed_size > size_base * r) {continue;}
        if (!seen_last_ && seed) {   // 놓친 뒤 다시 잡기: 투영점 근처만
          d = dist(b.x - seed->x, b.y - seed->y);
          if (d > c_.seed_gate) {continue;}
          ++seed_candidates;
          if (d < seed_best) {seed_second = seed_best; seed_best = d;}
          else {seed_second = std::min(seed_second, d);}
        } else {
          if (seen_last_ && seed && last_seed_) {
            // 팔 카메라가 움직이면 같은 정지 퍽의 영상 위치도 움직인다.
            // 실제 TF로 투영한 목표의 이동량을 사용해 이전 자리에 들어온 이웃을 피한다.
            d = dist(b.x - last_.x - (seed->x - last_seed_->x),
                     b.y - last_.y - (seed->y - last_seed_->y));
          } else {
            d = dist(b.x - last_.x, b.y - last_.y);
            if (moving_) {d = std::min(d, dist(b.x - last_.x - vx_, b.y - last_.y - vy_));}
          }
          if (d > follow) {continue;}
        }
      } else if (seed) {
        d = dist(b.x - seed->x, b.y - seed->y);
        if (d > c_.seed_gate) {continue;}
        ++seed_candidates;
        if (d < seed_best) {seed_second = seed_best; seed_best = d;}
        else {seed_second = std::min(seed_second, d);}
      } else {
        d = -b.score;   // 점수가 높을수록 앞
      }
      if (best < 0 || d < best_d) {best = static_cast<int>(i); best_d = d;}
    }
    if (seed_candidates > 1 && seed_second - seed_best <= 0.01) {best = -1;}  // 목표 영역이 겹치면 점수로 임의 선택하지 않음
    if (best >= 0) {
      if (locked_ && seen_last_) {   // 바로 앞 장에도 보였으면 그 이동을 다음 예측에 쓴다
        vx_ = boxes[best].x - last_.x;
        vy_ = boxes[best].y - last_.y;
        moving_ = true;
      } else {
        moving_ = false;
      }
      const bool continuous = !locked_ || seen_last_;
      locked_ = true;
      last_ = boxes[best];
      last_depth_ = seed ? seed->depth : 0;
      if (continuous && std::abs(last_.x) + last_.w < 0.98 &&
          (last_.h <= 0 || std::abs(last_.y) + last_.h < 0.98)) {
        reference_depth_ = last_depth_;
        reference_w_ = last_.size > 0 ? last_.size : last_.w;  // 연속 크기검사를 통과하고 FOV에서 안 잘린 폭만 기억
      }
    } else {
      moving_ = false;   // 놓친 뒤에는 직전 자리 근처에서만 다시 잡는다
    }
    last_seed_ = seed;
    seen_last_ = best >= 0;
    return best;
  }

private:
  ArmLockConfig c_;
  bool locked_ = false, moving_ = false, seen_last_ = false;
  double vx_ = 0, vy_ = 0;
  ArmBox last_;
  double reference_w_ = 0, reference_depth_ = 0, last_depth_ = 0;
  std::optional<ArmPoint> last_seed_;
};

}  // namespace perception
