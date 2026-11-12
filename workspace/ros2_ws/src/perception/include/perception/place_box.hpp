// 장소 상자 윗면 찾기 (팔 Arducam 한 장, 깊이 없음. ROS 의존 없음 → 단위 시험)
//   놓기 칸을 발제 지도 좌표가 아니라 눈앞 상자 기준으로 정하려는 것 (상자가 돌아가 있거나 밀려 있을 수 있음).
//   가정: 상자는 바닥에 놓인 box × box (기본 205 mm) 정사각 윗면, 높이 height (기본 0.07 m). 태그 위치에는 기대지 않는다.
//   1) 윗면 높이 평면을 정사영 영상(res, 기본 1 mm / 화소)으로 펼침 (평면 → 영상은 호모그래피) → 윗면은 실제 크기 정사각형
//   2) 크기를 아는 정사각형을 (중심, 요 ±45°) 거친 격자 → 고운 격자로 찾음. 점수: 변마다 안 · 밖 띠 Lab 차이 중앙값의 크기를 더하고,
//      네 변 안쪽 색이 서로 다르면 깎고(윗면은 한 색), 가장 약한 변의 대비를 더함(네 변 모두 실제 경계).
//      퍽 색(빨강 · 초록, 검출기 HSV 범위) 화소는 표본에서 뺌 — 놓인 퍽은 원근으로 바깥에 기울어 보여 테두리를 가림
//   3) 테두리 밝기 기울기 중앙값으로 다시 다듬고, 변마다 경계점(화소 이하)을 정사각형에 Huber 가중 최소제곱으로 맞춤
//   4) 정사각형은 90° 마다 같음 → 사전 요 ±45° 안에서 고름 (상자가 그보다 많이 돌지 않는다고 가정)
//   5) 화면 밖 (카메라 영상 밖 · 정사영 범위 밖) 으로 나가는 후보는 뺌 (max_offview). 퍽에 가려 표본이 적은 변과 화면 밖 변을
//      같은 '대비 0' 으로 두면, 윗면 변이 퍽에 많이 가릴 때 화면 밖으로 나간 엉뚱한 사각형 (앞 옆면 · 바닥 경계) 이 이길 수 있어서.
//      box_look 자세는 윗면 전체가 화면 안 (가장자리 30 px 넘게) 이 되게 고르므로 참 윗면은 빠지지 않는다.
//   6) 크기 배율 (scales, 기본 0.94 ~ 1.06) 도 같이 찾고 마지막 경계 맞춤에서 배율을 품. 카메라 높이 (정기구학) 가 실제와 다르면
//      윗면 높이 평면에 펼친 윗면이 몇 % 크거나 작게 보인다. 고른 배율로 중심 시차를 고침 (카메라 바로 아래를 중심으로 1 / 배율)
//   7) 옆면 띠 검사: 카메라 쪽 변 밖, 바닥 닿는 선 투영 자리에 옆면 → 바닥 경계가 없으면 아래 테두리를 잡은 것 → 카메라에서 먼 쪽을
//      다시 찾고, 안 되면 못 찾음. 8) 먼 변 검사: 카메라 반대쪽에 보이는 변의 대비가 작으면 (바닥 위 엉뚱한 변) 못 찾음
//   근거 (모의 261112, 비스듬히 서기 0.44 m · 50°): 카메라 높이 ±0.01 ~ ±0.02 m 224 장 모두 1 mm 안 (고치기 전 ±0.01 에서 4 장 60 ~ 95 mm),
//      카메라 쪽 변을 퍽으로 다 가린 42 장에서 틀린 값 0 (고치기 전 35 장) — lv2_next_module 261112_1221_front_d435_cal/pb_bottom
//   근거(모의, Isaac Sim 렌더 2026-11-03, 상자 요 ±30° · 옆 ±5 cm · 놓인 퍽 0 ~ 9 개, 카메라 윗면 위 0.20 m):
//      Python 시제품 판정용 120/120 — 중심 최대 0.68 mm · 요 0.37° · 칸 0.78 mm (lv2_next_module 261103_1114_place_box)
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>

namespace perception
{

struct PlaceBoxConfig
{
  double box = 0.205;          // [m] 윗면 한 변
  double height = 0.07;        // [m] 윗면 높이 (평면 기준 프레임 z)
  double res = 0.001;          // [m / 화소] 정사영 영상
  double span = 0.44;          // [m] 정사영 영상 한 변 (사전 중심 둘레)
  double search = 0.08;        // [m] 사전 중심에서 찾는 범위 (±)
  double coarse_xy = 0.004;    // [m] 거친 격자
  double coarse_deg = 1.0;     // [°]
  double band_px = 4.0;        // 안 · 밖 띠 거리 [화소]
  double lam = 2.0;            // 안쪽 색 불일치 벌점
  double beta = 2.0;           // 가장 약한 변 대비 가중
  int mask_grow = 3;           // 퍽 색 화소 부풀림 [화소]
  double max_offview = 0.02;   // 변마다 화면 밖 표본 비율이 이것보다 크면 그 후보를 뺌 (안 · 밖 띠 포함)
  // 윗면 크기 배율 후보: 카메라 높이(정기구학)가 실제와 ±1 ~ 2 cm 다르면 윗면 높이 평면에 펼친 상자가 몇 % 크거나 작게 보여
  //   한 크기로만 찾으면 참 테두리 대비가 떨어져 엉뚱한 사각형 (아래 테두리 등) 이 이김 (모의 261112 FK 높이 ±0.01 m)
  std::vector<double> scales{0.94, 0.97, 1.0, 1.03, 1.06};
  double scale_pen = 5.0;      // 거친 격자에서 배율이 1 에서 0.03 멀어질 때마다 깎는 점수 (비슷하면 배율 1 을 고름)
  double scale_reg = 20.0;     // 배율 풀 때 찾은 배율 쪽으로 묶는 세기 (경계점 몇 개 몫)
  bool scale_correct = true;   // 고른 배율로 중심 시차를 고침: 참 xy = 카메라 xy + (찾은 xy − 카메라 xy) / 배율
  // 옆면 띠 검사: 윗면이 맞으면 카메라 쪽 변 밖에 옆면이 보이고, 그 바깥 (바닥 닿는 선의 투영) 에 옆면 → 바닥 경계가 있다.
  //   아래 (바닥 닿는) 테두리를 윗면으로 잘못 잡으면 그 자리는 바닥 → 바닥이라 대비가 없다. 대비 중앙값이 side_min 보다 작으면
  //   카메라에서 먼 쪽 (윗면 후보) 을 다시 찾고, 그것도 안 되면 못 찾음 (ok false). 옆면 띠가 band_px * 2 + 2 화소보다 좁으면 (위에서 내려다봄) 검사 안 함
  double side_min = 2.0;
  // 먼 변 검사: 카메라 반대쪽 변은 바깥이 바닥이라 맞는 윗면이면 대비가 크다 (모의 맞은 장 17 이상). 보이는데 (표본 50 % 이상)
  //   대비가 이것보다 작으면 엉뚱한 사각형 (카메라 쪽 변이 퍽에 다 가린 장면에서 먼 쪽으로 밀린 것 등) → 못 찾음
  double far_min = 8.0;
  // 퍽 색 HSV (OpenCV H 0~179). perception.yaml 검출기 범위와 같게
  std::vector<std::array<int, 6>> puck_hsv{{0, 80, 35, 10, 255, 255}, {170, 80, 35, 179, 255, 255}, {40, 80, 35, 85, 255, 255}};
};

struct PlaceBoxResult
{
  bool ok = false;
  double x = 0.0, y = 0.0, yaw = 0.0;   // 평면 기준 프레임 [m, rad]
  double score = 0.0;
  double scale = 1.0;   // 고른 윗면 크기 배율 (1 = box)
  double side_e = -1.0; // 옆면 띠 대비 (−1 = 검사 안 함)
  double weak = -1.0;   // 카메라 반대쪽 보이는 변 중 가장 약한 대비 (−1 = 그런 변 없음)
};

namespace place_box_detail
{
struct View
{
  cv::Mat lab;       // CV_32FC3
  cv::Mat valid;     // CV_8U
  cv::Mat mask;      // CV_8U 퍽 색
  cv::Mat mag;       // CV_32F 기울기 크기
  cv::Matx33d S;     // (u, v, 1) → (X, Y, 1)
  int n = 0;
};

// 정사각형 (중심 0, 각 th, 한 변 side) 변 4 개 위 k 점 (모서리 근처 10 % 뺌) 과 바깥 법선
inline void perimeter(double th, double side, int k, std::vector<cv::Point2d> & off, std::array<cv::Point2d, 4> & nrm)
{
  off.clear();
  const double c = std::cos(th), s = std::sin(th);
  const cv::Point2d ax(c, s), ay(-s, c);
  const std::array<std::pair<double, int>, 4> sides{{{1, 0}, {-1, 0}, {1, 1}, {-1, 1}}};
  for (int si = 0; si < 4; ++si) {
    const double sign = sides[si].first;
    const cv::Point2d a = sides[si].second == 0 ? ax : ay, b = sides[si].second == 0 ? ay : ax;
    nrm[si] = a * sign;
    const cv::Point2d mid = a * (sign * side / 2);
    for (int i = 0; i < k; ++i) {
      const double t = (-0.45 + 0.9 * i / (k - 1)) * side;
      off.push_back(mid + b * t);
    }
  }
}

inline double median(std::vector<double> & v)
{
  if (v.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  const size_t m = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + m, v.end());
  if (v.size() % 2) {return v[m];}
  const double hi = v[m];
  return 0.5 * (hi + *std::max_element(v.begin(), v.begin() + m));
}

inline bool at(const View & V, const cv::Point2d & p, int & u, int & v)
{
  u = static_cast<int>(std::lround(p.x));
  v = static_cast<int>(std::lround(p.y));
  const bool in_span = u >= 0 && v >= 0 && u < V.n && v < V.n;   // 정사영 범위 밖은 화면 밖과 같이 봄
  u = std::clamp(u, 0, V.n - 1);
  v = std::clamp(v, 0, V.n - 1);
  return in_span && V.valid.at<uchar>(v, u) != 0;
}

// 자리 잡기 점수 (중심 c, 각 th)
inline double side_score(const View & V, const PlaceBoxConfig & cfg, cv::Point2d c, double th, int k, double scale = 1.0)
{
  std::vector<cv::Point2d> off;
  std::array<cv::Point2d, 4> nrm;
  const double side = scale * cfg.box / cfg.res;
  perimeter(th, side, k, off, nrm);
  std::array<double, 4> contrast{}, seen{};
  std::array<cv::Vec3d, 4> med_in{};
  std::array<bool, 4> has{};
  for (int si = 0; si < 4; ++si) {
    std::array<std::vector<double>, 3> din, ddf;
    int cnt = 0, offv = 0;
    for (int i = 0; i < k; ++i) {
      const cv::Point2d p = c + off[si * k + i];
      int ui, vi, uo, vo;
      const bool okin = at(V, p - nrm[si] * cfg.band_px, ui, vi), okout = at(V, p + nrm[si] * cfg.band_px, uo, vo);
      if (!okin || !okout) {++offv; continue;}   // 화면 밖 (가린 것과 따로 셈)
      if (V.mask.at<uchar>(vi, ui) || V.mask.at<uchar>(vo, uo)) {continue;}   // 퍽에 가림
      const cv::Vec3f a = V.lab.at<cv::Vec3f>(vi, ui), b = V.lab.at<cv::Vec3f>(vo, uo);
      for (int ch = 0; ch < 3; ++ch) {din[ch].push_back(a[ch]); ddf[ch].push_back(a[ch] - b[ch]);}
      ++cnt;
    }
    if (offv > cfg.max_offview * k) {return -std::numeric_limits<double>::infinity();}   // 화면 밖으로 나가는 후보는 뺌
    seen[si] = static_cast<double>(cnt) / k;
    has[si] = cnt > 0;
    cv::Vec3d md;
    for (int ch = 0; ch < 3; ++ch) {med_in[si][ch] = median(din[ch]); md[ch] = median(ddf[ch]);}
    contrast[si] = (seen[si] >= 0.15 && cnt > 0) ? cv::norm(md) : 0.0;
  }
  cv::Vec3d ref;
  for (int ch = 0; ch < 3; ++ch) {
    std::vector<double> v;
    for (int si = 0; si < 4; ++si) {if (has[si]) {v.push_back(med_in[si][ch]);}}
    ref[ch] = v.empty() ? 0.0 : median(v);
  }
  double sc = 0.0, cmin = std::numeric_limits<double>::infinity();
  for (int si = 0; si < 4; ++si) {
    const double spread = (seen[si] >= 0.15 && has[si]) ? cv::norm(med_in[si] - ref) : 0.0;
    sc += contrast[si] - cfg.lam * spread - 50.0 * std::max(0.0, 0.3 - seen[si]);
    cmin = std::min(cmin, contrast[si]);
  }
  return sc + cfg.beta * cmin;
}

// 다듬기 점수: 테두리 위 기울기 크기 (퍽 색 · 화면 밖 뺌) 변 중앙값의 평균
inline double grad_score(const View & V, const PlaceBoxConfig & cfg, cv::Point2d c, double th, int k, double scale = 1.0)
{
  std::vector<cv::Point2d> off;
  std::array<cv::Point2d, 4> nrm;
  perimeter(th, scale * cfg.box / cfg.res, k, off, nrm);
  double sum = 0.0;
  for (int si = 0; si < 4; ++si) {
    std::vector<double> v;
    for (int i = 0; i < k; ++i) {
      int u, w;
      if (at(V, c + off[si * k + i], u, w) && !V.mask.at<uchar>(w, u)) {v.push_back(V.mag.at<float>(w, u));}
    }
    const double m = median(v);
    sum += std::isfinite(m) ? m : 0.0;
  }
  return sum / 4.0;
}

template<typename F>
inline void local_search(F && f, cv::Point2d & c, double & th, double & best, double step_px, double step_rad, int r)
{
  cv::Point2d bc = c;
  double bt = th;
  for (int it = -r; it <= r; ++it) {
    const double t = th + it * step_rad;
    for (int iu = -r; iu <= r; ++iu) {
      for (int iv = -r; iv <= r; ++iv) {
        const cv::Point2d q(c.x + iu * step_px, c.y + iv * step_px);
        const double s = f(q, t);
        if (s > best) {best = s; bc = q; bt = t;}
      }
    }
  }
  c = bc;
  th = bt;
}

// 옆면 띠 대비: 윗면 후보 (중심 c, 각 th, 한 변 side 화소) 의 카메라 쪽 변마다, 바닥 닿는 선을 윗면 높이 평면에 투영한 자리 q 에서
//   안 (옆면) · 밖 (바닥) Lab 차이 중앙값. N = 카메라 바로 아래 (정사영 화소), sb = 바닥 → 윗면 높이 평면 투영 배율, sg = 고른 배율
inline double side_band(const View & V, const PlaceBoxConfig & cfg, cv::Point2d c, double th, double side, cv::Point2d N, double sb, double sg)
{
  std::vector<cv::Point2d> off;
  std::array<cv::Point2d, 4> nrm;
  const int k = 40;
  perimeter(th, side, k, off, nrm);
  std::vector<double> d;
  int tried = 0, offv = 0;
  for (int si = 0; si < 4; ++si) {
    const cv::Point2d mid = c + nrm[si] * (side / 2);
    if (nrm[si].dot(N - mid) <= 0) {continue;}   // 카메라 쪽 변만
    for (int i = 0; i < k; ++i) {
      const cv::Point2d p = c + off[si * k + i];
      const cv::Point2d P = N + (p - N) * (1.0 / sg);   // 참 xy (화소)
      const cv::Point2d q = N + (P - N) * sb;           // 바닥 닿는 점의 투영
      const double w = cv::norm(q - p);
      if (w < 2 * cfg.band_px + 2) {continue;}
      ++tried;
      const cv::Point2d u = (q - p) * (1.0 / w);
      int ui, vi, uo, vo;
      if (!at(V, q - u * cfg.band_px, ui, vi) || !at(V, q + u * cfg.band_px, uo, vo)) {++offv; continue;}
      if (V.mask.at<uchar>(vi, ui) || V.mask.at<uchar>(vo, uo)) {continue;}
      d.push_back(cv::norm(cv::Vec3f(V.lab.at<cv::Vec3f>(vi, ui) - V.lab.at<cv::Vec3f>(vo, uo))));
    }
  }
  if (tried == 0) {return -1.0;}                        // 위에서 내려다봐 옆면이 안 보임 → 검사 안 함
  if (d.size() < 10) {return 2 * offv >= tried ? -1.0 : 0.0;}   // 바닥 닿는 선이 거의 화면 밖이면 검사 안 함, 퍽에 가렸으면 증거 없음
  return median(d);
}

// 카메라 반대쪽 변 (바깥이 바닥) 중 보이는 변 (화면 안 · 퍽에 안 가린 표본 seen_min 이상) 의 가장 약한 안 · 밖 대비 (Lab). 없으면 −1
//   카메라 쪽 변은 바깥이 같은 재질의 옆면이라 대비가 약할 수 있어 뺌. N = 카메라 바로 아래 (정사영 화소)
inline double weakest_far_side(const View & V, const PlaceBoxConfig & cfg, cv::Point2d c, double th, double scale, double seen_min, cv::Point2d N)
{
  std::vector<cv::Point2d> off;
  std::array<cv::Point2d, 4> nrm;
  const int k = 60;
  perimeter(th, scale * cfg.box / cfg.res, k, off, nrm);
  double w = std::numeric_limits<double>::infinity();
  for (int si = 0; si < 4; ++si) {
    if (nrm[si].dot(N - (c + nrm[si] * (scale * cfg.box / cfg.res / 2))) > 0) {continue;}   // 카메라 쪽 변
    std::array<std::vector<double>, 3> ddf;
    int cnt = 0;
    for (int i = 0; i < k; ++i) {
      const cv::Point2d p = c + off[si * k + i];
      int ui, vi, uo, vo;
      if (!at(V, p - nrm[si] * cfg.band_px, ui, vi) || !at(V, p + nrm[si] * cfg.band_px, uo, vo)) {continue;}
      if (V.mask.at<uchar>(vi, ui) || V.mask.at<uchar>(vo, uo)) {continue;}
      const cv::Vec3f a = V.lab.at<cv::Vec3f>(vi, ui), b = V.lab.at<cv::Vec3f>(vo, uo);
      for (int ch = 0; ch < 3; ++ch) {ddf[ch].push_back(a[ch] - b[ch]);}
      ++cnt;
    }
    if (cnt < seen_min * k) {continue;}
    cv::Vec3d md;
    for (int ch = 0; ch < 3; ++ch) {md[ch] = median(ddf[ch]);}
    w = std::min(w, cv::norm(md));
  }
  return std::isfinite(w) ? w : -1.0;
}
}  // namespace place_box_detail

// bgr: 왜곡 보정한 팔 영상, K: 그 내부값, R_pc · t_pc: 카메라 광학 좌표 → 평면 기준 프레임 (z 위, 예 base_footprint),
// prior: 사전 상자 중심 (평면 기준 x, y), prior_yaw: 사전 요 (이것 ±45° 안에서 고름)
inline PlaceBoxResult find_place_box(const cv::Mat & bgr, const cv::Matx33d & K, const cv::Matx33d & R_pc, const cv::Vec3d & t_pc,
  cv::Point2d prior, double prior_yaw, const PlaceBoxConfig & cfg = PlaceBoxConfig())
{
  using namespace place_box_detail;
  PlaceBoxResult out;
  if (bgr.empty() || bgr.type() != CV_8UC3) {return out;}
  View V;
  V.n = static_cast<int>(std::lround(cfg.span / cfg.res));
  // 평면 z = h 의 (X, Y) → 화소: K [r1 r2 r3·h + t], 여기서 [r | t] = 평면 → 카메라
  const cv::Matx33d Rt = R_pc.t();
  const cv::Vec3d tt = -(Rt * t_pc);
  cv::Matx33d M;
  for (int i = 0; i < 3; ++i) {M(i, 0) = Rt(i, 0); M(i, 1) = Rt(i, 1); M(i, 2) = Rt(i, 2) * cfg.height + tt[i];}
  // 정사영 화소 (u, v): 위쪽 = +X (앞), 왼쪽 = +Y
  V.S = cv::Matx33d(0, -cfg.res, prior.x + cfg.span / 2, -cfg.res, 0, prior.y + cfg.span / 2, 0, 0, 1);
  const cv::Matx33d Hm = K * M * V.S;
  cv::Mat tv;
  cv::warpPerspective(bgr, tv, cv::Mat(Hm), cv::Size(V.n, V.n), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT);
  cv::warpPerspective(cv::Mat(bgr.size(), CV_8U, cv::Scalar(255)), V.valid, cv::Mat(Hm), cv::Size(V.n, V.n),
    cv::INTER_NEAREST | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT);
  cv::Mat lab8, labf, hsv;
  cv::cvtColor(tv, lab8, cv::COLOR_BGR2Lab);
  lab8.convertTo(labf, CV_32FC3);
  cv::GaussianBlur(labf, V.lab, cv::Size(0, 0), 1.5);
  cv::cvtColor(tv, hsv, cv::COLOR_BGR2HSV);
  V.mask = cv::Mat::zeros(tv.size(), CV_8U);
  for (const auto & r : cfg.puck_hsv) {
    cv::Mat m;
    cv::inRange(hsv, cv::Scalar(r[0], r[1], r[2]), cv::Scalar(r[3], r[4], r[5]), m);
    V.mask |= m;
  }
  if (cfg.mask_grow > 0) {
    cv::dilate(V.mask, V.mask, cv::Mat::ones(2 * cfg.mask_grow + 1, 2 * cfg.mask_grow + 1, CV_8U));
  }
  {
    std::vector<cv::Mat> ch;
    cv::split(V.lab, ch);
    V.mag = cv::Mat::zeros(tv.size(), CV_32F);
    for (const auto & c : ch) {
      cv::Mat gx, gy;
      cv::Sobel(c, gx, CV_32F, 1, 0, 3);
      cv::Sobel(c, gy, CV_32F, 0, 1, 3);
      V.mag += gx.mul(gx) + gy.mul(gy);
    }
    cv::sqrt(V.mag, V.mag);
  }
  const double c0 = V.n / 2.0;
  // 거친 격자: (배율, 각) 마다 따로 (여러 스레드), 끝나면 순서대로 가장 좋은 것 — 한 스레드로 돈 것과 같은 답
  const int ng = static_cast<int>(std::floor(cfg.search / cfg.coarse_xy + 1e-9));
  const int na = static_cast<int>(std::ceil(90.0 / cfg.coarse_deg - 1e-9));
  const std::vector<double> scs = cfg.scales.empty() ? std::vector<double>{1.0} : cfg.scales;
  const int ns = static_cast<int>(scs.size());
  std::vector<double> abest(na * ns, -std::numeric_limits<double>::infinity());
  std::vector<cv::Point2d> apos(na * ns, cv::Point2d(c0, c0));
  cv::parallel_for_(cv::Range(0, na * ns), [&](const cv::Range & rg) {
    for (int j = rg.start; j < rg.end; ++j) {
      const int is = j / na, ia = j % na;
      const double th = (-45.0 + ia * cfg.coarse_deg) * CV_PI / 180.0 + prior_yaw;
      for (int iu = -ng; iu <= ng; ++iu) {
        for (int iv = -ng; iv <= ng; ++iv) {
          const cv::Point2d q(c0 + iu * cfg.coarse_xy / cfg.res, c0 + iv * cfg.coarse_xy / cfg.res);
          const double s = side_score(V, cfg, q, th, 24, scs[is]);
          if (s > abest[j]) {abest[j] = s; apos[j] = q;}
        }
      }
    }
  });
  double best = -std::numeric_limits<double>::infinity(), bt = 0.0, bs = 1.0;
  cv::Point2d bc(c0, c0);
  double bpen = -std::numeric_limits<double>::infinity();
  for (int j = 0; j < na * ns; ++j) {
    const double pen = abest[j] - cfg.scale_pen * std::abs(scs[j / na] - 1.0) / 0.03;
    if (pen > bpen) {bpen = pen; best = abest[j]; bc = apos[j]; bt = (-45.0 + (j % na) * cfg.coarse_deg) * CV_PI / 180.0 + prior_yaw; bs = scs[j / na];}
  }
  const double d2r = CV_PI / 180.0;
  auto fs = [&](cv::Point2d q, double t) {return side_score(V, cfg, q, t, 60, bs);};
  local_search(fs, bc, bt, best, 0.001 / cfg.res, 0.25 * d2r, 4);
  local_search(fs, bc, bt, best, 0.0005 / cfg.res, 0.1 * d2r, 3);
  local_search(fs, bc, bt, best, 0.00025 / cfg.res, 0.05 * d2r, 2);
  // 카메라 바로 아래 (정사영 화소) 와 바닥 → 윗면 높이 평면 투영 배율
  const cv::Matx33d Sinv = V.S.inv();
  const cv::Vec3d Nh = Sinv * cv::Vec3d(t_pc[0], t_pc[1], 1.0);
  const cv::Point2d N(Nh[0] / Nh[2], Nh[1] / Nh[2]);
  const double sb = t_pc[2] > cfg.height + 1e-3 ? (t_pc[2] - cfg.height) / t_pc[2] : 1.0;
  // 옆면 띠 검사: 실패하면 '이 후보가 아래 테두리라면' 의 윗면 자리 (카메라에서 먼 쪽) 를 다시 찾음
  double se = side_band(V, cfg, bc, bt, bs * cfg.box / cfg.res, N, sb, bs);
  if (se >= 0.0 && se < cfg.side_min) {
    cv::Point2d ca = N + (bc - N) * (1.0 / sb);
    double ta = bt, ba = -std::numeric_limits<double>::infinity(), sa = bs;
    for (const double sc : scs) {
      cv::Point2d cq = ca;
      double tq = bt, bq = -std::numeric_limits<double>::infinity();
      auto fq = [&](cv::Point2d q, double t) {return side_score(V, cfg, q, t, 60, sc);};
      local_search(fq, cq, tq, bq, 0.004 / cfg.res, 1.0 * d2r, 5);
      local_search(fq, cq, tq, bq, 0.001 / cfg.res, 0.25 * d2r, 4);
      if (bq > ba) {ba = bq; ca = cq; ta = tq; sa = sc;}
    }
    const double sea = side_band(V, cfg, ca, ta, sa * cfg.box / cfg.res, N, sb, sa);
    if (std::isfinite(ba) && ba > 0.0 && (sea < 0.0 || sea >= cfg.side_min)) {
      bc = ca; bt = ta; bs = sa; best = ba; se = sea;
    } else {
      out.score = best;
      out.scale = bs;
      out.side_e = se;
      out.ok = false;   // 윗면이라는 증거가 없음 → 틀린 값 대신 못 찾음
      const cv::Vec3d XY0 = V.S * cv::Vec3d(bc.x, bc.y, 1.0);
      out.x = XY0[0];
      out.y = XY0[1];
      out.yaw = prior_yaw;
      return out;
    }
  }
  out.score = best;
  out.scale = bs;
  out.side_e = se;
  double side = bs * cfg.box / cfg.res;
  auto gs = [&](cv::Point2d q, double t) {return grad_score(V, cfg, q, t, 80, bs);};
  double gbest = -std::numeric_limits<double>::infinity();
  local_search(gs, bc, bt, gbest, 0.001 / cfg.res, 0.25 * d2r, 3);
  local_search(gs, bc, bt, gbest, 0.0005 / cfg.res, 0.1 * d2r, 3);
  local_search(gs, bc, bt, gbest, 0.00025 / cfg.res, 0.05 * d2r, 2);
  // 변마다 경계점 → 정사각형 가중 최소제곱 (Huber 1 화소). 정사각형을 dθ 돌리면 변 위 점의 법선 성분은 −t·dθ
  //   배율 후보가 둘 이상이면 배율 (상대 e: 모든 변이 바깥으로 side/2·e) 도 같이 푼다. 마주 보는 변이 안 보이면 배율과 자리가 섞이므로
  //   찾은 배율 쪽으로 약하게 묶음 (scale_reg 점 몫)
  for (int it = 0; it < 3; ++it) {
    std::vector<cv::Point2d> off;
    std::array<cv::Point2d, 4> nrm;
    const int k = 80;
    perimeter(bt, side, k, off, nrm);
    std::vector<std::array<double, 4>> A;
    const bool fit_scale = cfg.scales.size() > 1;
    std::vector<double> y;
    for (int si = 0; si < 4; ++si) {
      const cv::Point2d tang(-nrm[si].y, nrm[si].x);
      for (int i = 0; i < k; ++i) {
        const cv::Point2d p0 = bc + off[si * k + i];
        const double ts = off[si * k + i].dot(tang);
        std::array<double, 11> prof;
        bool bad = false;
        double mx = -1.0;
        int j = 0;
        for (int dl = -5; dl <= 5; ++dl) {
          const cv::Point2d q = p0 + nrm[si] * dl;
          const int u = static_cast<int>(std::lround(q.x)), v = static_cast<int>(std::lround(q.y));
          if (u < 0 || v < 0 || u >= V.n || v >= V.n || !V.valid.at<uchar>(v, u) || V.mask.at<uchar>(v, u)) {bad = true; break;}
          prof[dl + 5] = V.mag.at<float>(v, u);
          if (prof[dl + 5] > mx) {mx = prof[dl + 5]; j = dl + 5;}
        }
        if (bad || mx < 5.0 || j == 0 || j == 10) {continue;}
        const double a = prof[j - 1], b = prof[j], c = prof[j + 1], den = a - 2 * b + c;
        const double dd = (j - 5) + (den != 0.0 ? 0.5 * (a - c) / den : 0.0);
        A.push_back({nrm[si].x, nrm[si].y, -ts, fit_scale ? side / 2 : 0.0});
        y.push_back(dd);
      }
    }
    if (A.size() < 40) {break;}
    std::vector<double> w(A.size(), 1.0);
    cv::Vec4d sol;
    for (int h = 0; h < 5; ++h) {
      cv::Matx44d AtA = cv::Matx44d::zeros();
      cv::Vec4d Aty(0, 0, 0, 0);
      for (size_t i = 0; i < A.size(); ++i) {
        const double ww = w[i] * w[i];
        for (int r = 0; r < 4; ++r) {
          Aty[r] += ww * A[i][r] * y[i];
          for (int c = 0; c < 4; ++c) {AtA(r, c) += ww * A[i][r] * A[i][c];}
        }
      }
      AtA(3, 3) += fit_scale ? cfg.scale_reg * (side / 2) * (side / 2) : 1.0;
      if (!cv::solve(cv::Mat(AtA), cv::Mat(Aty), cv::Mat(sol, false), cv::DECOMP_SVD)) {break;}
      for (size_t i = 0; i < A.size(); ++i) {
        const double res = std::abs(y[i] - (A[i][0] * sol[0] + A[i][1] * sol[1] + A[i][2] * sol[2] + A[i][3] * sol[3]));
        w[i] = res <= 1.0 ? 1.0 : std::sqrt(1.0 / std::max(res, 1e-6));
      }
    }
    if (std::max(std::abs(sol[0]), std::abs(sol[1])) > 3.0 || std::abs(sol[2]) > 2.0 * d2r || std::abs(sol[3]) > 0.03) {break;}
    bc += cv::Point2d(sol[0], sol[1]);
    bt += sol[2];
    bs *= 1.0 + sol[3];
    side = bs * cfg.box / cfg.res;
  }
  out.scale = bs;
  out.weak = weakest_far_side(V, cfg, bc, bt, bs, 0.5, N);
  const bool far_ok = out.weak < 0.0 || out.weak >= cfg.far_min;
  const cv::Vec3d XY = V.S * cv::Vec3d(bc.x, bc.y, 1.0);
  out.x = XY[0];
  out.y = XY[1];
  if (cfg.scale_correct && std::abs(bs - 1.0) > 1e-9) {   // 배율 = 윗면 실제 높이 차이 → 카메라 바로 아래를 중심으로 줄이고 늘림
    out.x = t_pc[0] + (out.x - t_pc[0]) / bs;
    out.y = t_pc[1] + (out.y - t_pc[1]) / bs;
  }
  // 영상 (u, v) 방향 (cos, sin) → 평면 (X, Y) = (−sin, −cos)  (u = −Y, v = −X)
  double yaw = std::atan2(-std::cos(bt), -std::sin(bt)) - prior_yaw;
  yaw = std::fmod(yaw + CV_PI / 4, CV_PI / 2);
  if (yaw < 0) {yaw += CV_PI / 2;}
  out.yaw = yaw - CV_PI / 4 + prior_yaw;
  out.ok = std::isfinite(out.score) && out.score > 0.0 && far_ok;
  return out;
}

// 칸 9 개 중심 (평면 기준, 행 우선: dx −1 → 1 바깥 반복, dy −1 → 1 안쪽 반복). pitch 기본 0.06 m
inline std::array<cv::Point2d, 9> place_cells(const PlaceBoxResult & r, double pitch = 0.06)
{
  std::array<cv::Point2d, 9> c;
  const double cy = std::cos(r.yaw), sy = std::sin(r.yaw);
  int i = 0;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      c[i++] = cv::Point2d(r.x + (cy * dx - sy * dy) * pitch, r.y + (sy * dx + cy * dy) * pitch);
    }
  }
  return c;
}

}  // namespace perception
