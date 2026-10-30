#include <algorithm>
#include <climits>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

#include "perception/detector.hpp"
#include "perception/tag_detector.hpp"

namespace perception
{

std::vector<Detection> ColorDetector::detect(const cv::Mat & bgr)
{
  return detect_with_reference(bgr, bgr);
}

std::vector<Detection> ColorDetector::detect_with_reference(const cv::Mat & bgr, const cv::Mat & reference)
{
  std::vector<Detection> out;
  if (bgr.empty()) {return out;}
  // 블러는 새 버퍼에 한다. 예전에는 src 가 bgr 와 버퍼를 같이 써서 입력 영상을 제자리에서 흐리게 했다
  // → 노드가 같은 영상으로 이어서 찾는 태그 · 장소 태그 · 깊이 나누기가 흐린 영상을 받았다(2026-10-25 발견)
  cv::Mat src = bgr, hsv;
  if (cfg_.blur_ksize > 1) {
    cv::Mat blurred;
    cv::GaussianBlur(bgr, blurred, cv::Size(cfg_.blur_ksize | 1, cfg_.blur_ksize | 1), 0);
    src = blurred;
  }
  cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
  const double img_area = static_cast<double>(bgr.cols) * bgr.rows;
  const int k = std::max(1, cfg_.morph_kernel);
  const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {k, k});
  if (cfg_.label_aware) {return detect_labeled(hsv, kernel, reference);}
  for (int cls = 0; cls < 3; ++cls) {
    cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8U), m;
    bool any = false;
    for (const auto & h : cfg_.hsv) {   // 같은 색의 범위는 합침 (빨강 두 범위)
      if (h.cls != cls || (cfg_.target_color != "any" && cfg_.target_color != h.color)) {continue;}
      cv::inRange(hsv, h.lower, h.upper, m);
      mask |= m;
      any = true;
    }
    if (!any) {continue;}
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto & c : contours) {
      const double area = cv::contourArea(c);
      if (area < cfg_.min_area_ratio * img_area) {continue;}
      const cv::Rect r = cv::boundingRect(c);
      // 채움률은 실제 마스크 픽셀로 셈 (외곽 컨투어 넓이는 속 빈 테두리도 1 로 나옴). 사각 · 원 퍽은 0.75~1
      const double fill = cv::countNonZero(mask(r)) / std::max(1.0, static_cast<double>(r.area()));
      if (fill < cfg_.min_fill_ratio) {continue;}
      const double aspect = static_cast<double>(std::min(r.width, r.height)) / std::max(r.width, r.height);
      Detection d;
      d.box = cv::Rect2f(r);
      d.class_id = cls;
      d.score = static_cast<float>(std::clamp(0.5 * fill + 0.5 * aspect, 0.0, 1.0));
      if (d.score >= cfg_.score_threshold) {out.push_back(d);}
    }
  }
  return out;
}

namespace
{
struct Label
{
  cv::Point2f c;
  std::vector<cv::Point> poly;
  double side = 0;
  int cls = -1;
  int tag_id = -1;
  cv::Mat mask;
};
}  // namespace

// 라벨 인식 검출 (detector.hpp DetectorConfig::label_aware 설명)
std::vector<Detection> ColorDetector::detect_labeled(const cv::Mat & hsv, const cv::Mat & kernel, const cv::Mat & reference)
{
  std::vector<Detection> out;
  const cv::Size sz = hsv.size();
  const double img_area = static_cast<double>(sz.area());
  // 색마다 마스크 (detect 와 같음)
  std::vector<cv::Mat> masks(3);
  std::vector<bool> used(3, false);
  for (int cls = 0; cls < 3; ++cls) {
    masks[cls] = cv::Mat::zeros(sz, CV_8U);
    cv::Mat m;
    for (const auto & h : cfg_.hsv) {
      if (h.cls != cls || (cfg_.target_color != "any" && cfg_.target_color != h.color)) {continue;}
      cv::Scalar lower = h.lower;
      lower[2] = std::clamp(cfg_.label_body_v_min, 0, 255);
      cv::inRange(hsv, lower, h.upper, m);
      masks[cls] |= m;
      used[cls] = true;
    }
    if (!used[cls]) {continue;}
    cv::morphologyEx(masks[cls], masks[cls], cv::MORPH_OPEN, kernel);
    cv::morphologyEx(masks[cls], masks[cls], cv::MORPH_CLOSE, kernel);
  }
  // 태그: 어둡고 채도 낮은 화소를 닫아 한 덩어리로, 정사각형에 가까운 것만 → 라벨 = 태그 회전 사각형 × label_scale
  std::vector<cv::Mat> ch;
  cv::split(hsv, ch);
  cv::Mat dark = (ch[2] <= cfg_.label_dark_v_max) & (ch[1] <= cfg_.label_dark_s_max);
  if (cfg_.label_dark_window > 0) {
    cv::Mat value, mean;
    ch[2].convertTo(value, CV_32F);
    const int win = cfg_.label_dark_window | 1;
    cv::blur(value, mean, {win, win});
    dark &= value < mean - cfg_.label_dark_c;
  }
  const int tk = std::max(1, cfg_.label_tag_close);
  cv::morphologyEx(dark, dark, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, {tk, tk}));
  std::vector<std::vector<cv::Point>> tags;
  cv::findContours(dark, tags, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  std::vector<Label> labels;
  for (const auto & t : tags) {
    const double a = cv::contourArea(t);
    if (a < cfg_.label_tag_min_px) {continue;}
    const cv::RotatedRect r = cv::minAreaRect(t);
    const double w = r.size.width, h = r.size.height;
    if (std::min(w, h) / std::max({w, h, 1e-6}) < cfg_.label_square_min || a / std::max(w * h, 1e-6) < cfg_.label_square_min) {
      continue;
    }
    Label L;
    L.c = r.center;
    L.side = std::sqrt(w * h) * cfg_.label_scale;
    cv::Point2f p[4];
    cv::RotatedRect(r.center, cv::Size2f(w * cfg_.label_scale, h * cfg_.label_scale), r.angle).points(p);
    for (const auto & q : p) {L.poly.emplace_back(cvRound(q.x), cvRound(q.y));}
    L.mask = cv::Mat::zeros(sz, CV_8U);
    cv::fillConvexPoly(L.mask, L.poly, 255);
    // 라벨 색: 바로 바깥 띠에서 그 색 화소 비율이 가장 큰 색
    const int rr = std::max(2, cvRound(L.side * cfg_.label_ring_frac));
    cv::Mat ring;
    cv::dilate(L.mask, ring, cv::getStructuringElement(cv::MORPH_ELLIPSE, {2 * rr + 1, 2 * rr + 1}));
    ring &= ~L.mask;
    const double n = std::max(1, cv::countNonZero(ring));
    double best = 0;
    for (int cls = 0; cls < 3; ++cls) {
      if (!used[cls]) {continue;}
      const double f = cv::countNonZero(ring & masks[cls]) / n;
      if (f > best) {best = f; L.cls = cls;}
    }
    if (best < cfg_.label_ring_min) {L.cls = -1;}
    labels.push_back(std::move(L));
  }
  // 원본 해상도에서 실제로 해독된 동일 ID만 여러 면으로 묶는다. 미해독/다른 ID는 합치지 않음.
  if (cfg_.label_merge_same_id && labels.size() > 1 && !reference.empty()) {
    const auto decoded = TagDetector("tag36h11", .016).detect(reference, cv::Mat(), cv::Mat());
    std::vector<int> ids; std::vector<std::vector<cv::Point2f>> corners;
    for(const auto & t : decoded) {ids.push_back(t.id);corners.push_back(t.corners);}
    const float sx = float(sz.width) / reference.cols, sy = float(sz.height) / reference.rows;
    for (auto & cs : corners) {for (auto & p : cs) {p.x=(p.x+.5f)*sx-.5f;p.y=(p.y+.5f)*sy-.5f;}}
    for (auto & L : labels) {
      int id=-1;
      for (size_t k=0;k<ids.size();++k) {
        if (ids[k]<0 || ids[k]>11 || cv::pointPolygonTest(corners[k], L.c, false)<0) {continue;}
        if(id>=0 && id!=ids[k]) {id=-2;break;}
        id=ids[k];
      }
      L.tag_id=id;
    }
  }
  // 색마다: 라벨을 더해 조각을 묶고, 덩어리 구멍을 채우고, 라벨이 여럿이면 가까운 라벨로 나눔
  for (int cls = 0; cls < 3; ++cls) {
    if (!used[cls]) {continue;}
    cv::Mat comb = masks[cls].clone();
    std::vector<const Label *> own;
    for (const auto & L : labels) {
      if (L.cls == cls) {comb |= L.mask; own.push_back(&L);}
    }
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(comb, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (size_t ci = 0; ci < contours.size(); ++ci) {
      const cv::Rect bb = cv::boundingRect(contours[ci]);
      cv::Mat reg = cv::Mat::zeros(bb.size(), CV_8U);
      cv::drawContours(reg, contours, static_cast<int>(ci), 255, cv::FILLED, cv::LINE_8, cv::noArray(), INT_MAX, -bb.tl());
      std::vector<cv::Point2f> cen;
      std::vector<int> group_ids, group_counts;
      for (const auto * L : own) {
        const cv::Point q(cvFloor(L->c.x) - bb.x, cvFloor(L->c.y) - bb.y);
        if (q.inside(cv::Rect(0, 0, bb.width, bb.height)) && reg.at<uint8_t>(q)) {
          auto found=std::find(group_ids.begin(),group_ids.end(),L->tag_id);
          if(L->tag_id>=0 && found!=group_ids.end()) {
            const auto k=static_cast<size_t>(found-group_ids.begin());
            cen[k]+=L->c-cv::Point2f(bb.tl()); ++group_counts[k];
          } else {cen.push_back(L->c-cv::Point2f(bb.tl()));group_ids.push_back(L->tag_id);group_counts.push_back(1);}
        }
      }
      for(size_t k=0;k<cen.size();++k) {cen[k]*=1.f/group_counts[k];}
      std::vector<std::vector<cv::Point>> parts(std::max<size_t>(1, cen.size()));
      for (int y = 0; y < reg.rows; ++y) {
        const uint8_t * row = reg.ptr<uint8_t>(y);
        for (int x = 0; x < reg.cols; ++x) {
          if (!row[x]) {continue;}
          size_t k = 0;
          if (cen.size() >= 2) {
            float bd = std::numeric_limits<float>::max();
            for (size_t i = 0; i < cen.size(); ++i) {
              const float dx = x - cen[i].x, dy = y - cen[i].y, d = dx * dx + dy * dy;
              if (d < bd) {bd = d; k = i;}
            }
          }
          parts[k].emplace_back(x + bb.x, y + bb.y);
        }
      }
      for (const auto & pts : parts) {
        if (pts.empty()) {continue;}
        const double area = static_cast<double>(pts.size());
        if (area < cfg_.min_area_ratio * img_area) {continue;}
        const cv::RotatedRect r = cv::minAreaRect(pts);
        const double w = std::max(1.0f, r.size.width), h = std::max(1.0f, r.size.height);
        const double fill = area / (w * h);
        if (fill < cfg_.min_fill_ratio) {continue;}
        Detection d;
        d.box = cv::Rect2f(cv::boundingRect(pts));
        d.tracking_size = std::sqrt(w * h);
        d.class_id = cls;
        d.score = static_cast<float>(std::clamp(0.5 * fill + 0.5 * std::min(w, h) / std::max(w, h), 0.0, 1.0));
        if (d.score >= cfg_.score_threshold) {out.push_back(d);}
      }
    }
  }
  return out;
}

int best_detection(const std::vector<Detection> & d, int target_class)
{
  int best = -1;
  for (int i = 0; i < static_cast<int>(d.size()); ++i) {
    if (target_class >= 0 && d[i].class_id != target_class) {continue;}
    if (best < 0 || d[i].score > d[best].score) {best = i;}
  }
  return best;
}

#ifdef USE_TENSORRT
std::unique_ptr<Detector> make_tensorrt_detector(const DetectorConfig & c, std::string & info);
#endif

std::unique_ptr<Detector> make_detector(const DetectorConfig & c, std::string & info)
{
  if (c.type == "tensorrt" || c.type == "auto") {
#ifdef USE_TENSORRT
    if (!c.engine_path.empty()) {
      auto d = make_tensorrt_detector(c, info);
      if (d) {return d;}
    } else {
      info = "engine_path 없음";
    }
#else
    info = "TensorRT 없이 빌드됨";
#endif
    if (c.type == "tensorrt") {return nullptr;}
    info += " → ColorDetector 사용";
  }
  return std::make_unique<ColorDetector>(c);
}

}  // namespace perception
