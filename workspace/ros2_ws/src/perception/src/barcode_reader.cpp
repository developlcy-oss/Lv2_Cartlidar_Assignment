// 바코드 문자열 읽기 (barcode_reader.hpp). zbar C API
#include "perception/barcode_reader.hpp"

#include <algorithm>
#include <cmath>
#include <set>

#include <opencv2/imgproc.hpp>
#include <zbar.h>

namespace perception
{
using namespace zbar;   // C++ 로 읽으면 zbar.h 의 C 함수가 zbar 이름공간 안에 있다

BarcodeRead decide_barcode(const std::vector<std::string> & symbols)
{
  BarcodeRead r;
  r.symbols = symbols;
  const std::set<std::string> u(symbols.begin(), symbols.end());
  if (u.size() == 1) {
    r.ok = true;
    r.text = *u.begin();
  }
  return r;
}

BarcodeReader::BarcodeReader(double scale, double gamma, int min_quality, int x_density, int y_density)
: scale_(scale > 0 ? scale : 1.0), min_quality_(min_quality)
{
  if (gamma > 0 && gamma != 1.0) {   // Py: (255 * (np.arange(256) / 255.0) ** GAMMA).round()
    lut_.create(1, 256, CV_8U);
    for (int i = 0; i < 256; ++i) {lut_.at<uchar>(i) = static_cast<uchar>(std::nearbyint(255 * std::pow(i / 255.0, gamma)));}
  }
  auto * sc = zbar_image_scanner_create();
  zbar_image_scanner_set_config(sc, ZBAR_NONE, ZBAR_CFG_ENABLE, 0);      // 모두 끔
  zbar_image_scanner_set_config(sc, ZBAR_CODE128, ZBAR_CFG_ENABLE, 1);   // CODE128 켬
  zbar_image_scanner_set_config(sc, ZBAR_NONE, ZBAR_CFG_X_DENSITY, x_density);
  zbar_image_scanner_set_config(sc, ZBAR_NONE, ZBAR_CFG_Y_DENSITY, y_density);
  scanner_ = sc;
}

BarcodeReader::~BarcodeReader()
{
  if (scanner_) {zbar_image_scanner_destroy(static_cast<zbar_image_scanner_t *>(scanner_));}
}

void BarcodeReader::set_label(const BarcodeLabelConfig & c)
{
  label_ = c;
  label_lut_.release();
  if (c.gamma > 0 && c.gamma != 1.0) {
    label_lut_.create(1, 256, CV_8U);
    for (int i = 0; i < 256; ++i) {label_lut_.at<uchar>(i) = static_cast<uchar>(std::nearbyint(255 * std::pow(i / 255.0, c.gamma)));}
  }
}

void BarcodeReader::scan(const cv::Mat & g, std::vector<std::string> & syms, int & dropped)
{
  zbar_image_t * zi = zbar_image_create();
  zbar_image_set_format(zi, zbar_fourcc('Y', '8', '0', '0'));
  zbar_image_set_size(zi, static_cast<unsigned>(g.cols), static_cast<unsigned>(g.rows));
  zbar_image_set_data(zi, g.data, g.total(), nullptr);
  zbar_scan_image(static_cast<zbar_image_scanner_t *>(scanner_), zi);
  for (const zbar_symbol_t * s = zbar_image_first_symbol(zi); s; s = zbar_symbol_next(s)) {
    if (zbar_symbol_get_quality(s) >= min_quality_) {
      syms.emplace_back(zbar_symbol_get_data(s), zbar_symbol_get_data_length(s));
    } else {
      ++dropped;
    }
  }
  zbar_image_destroy(zi);
}

namespace
{
// 장 g 를 가운데 기준 angle 도 돌림 (잘리지 않게 키운 캔버스, 바깥은 흰색)
cv::Mat rotate_white(const cv::Mat & g, double angle)
{
  cv::Mat matrix = cv::getRotationMatrix2D(cv::Point2f(g.cols / 2.f, g.rows / 2.f), angle, 1.0);
  const double c = std::abs(matrix.at<double>(0, 0)), sn = std::abs(matrix.at<double>(0, 1));
  const int width = static_cast<int>(std::ceil(g.cols * c + g.rows * sn));
  const int height = static_cast<int>(std::ceil(g.cols * sn + g.rows * c));
  matrix.at<double>(0, 2) += (width - g.cols) / 2.0;
  matrix.at<double>(1, 2) += (height - g.rows) / 2.0;
  cv::Mat out;
  cv::warpAffine(g, out, matrix, cv::Size(width, height), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));
  return out;
}
}  // namespace

int BarcodeReader::read_labels(const cv::Mat & gray, std::vector<std::string> & syms, int & dropped)
{
  cv::Mat b, bw;
  cv::GaussianBlur(gray, b, cv::Size(5, 5), 0);
  // 99.5 백분위 밝기 (라벨 흰 종이 쪽)
  int hist[256] = {0};
  for (int y = 0; y < b.rows; ++y) {
    const uchar * r = b.ptr<uchar>(y);
    for (int x = 0; x < b.cols; ++x) {++hist[r[x]];}
  }
  const double target = 0.995 * static_cast<double>(b.total());
  double acc = 0;
  int p995 = 255;
  for (int v = 0; v < 256; ++v) {acc += hist[v]; if (acc >= target) {p995 = v; break;}}
  cv::threshold(b, bw, label_.rel * p995, 255, cv::THRESH_BINARY);
  std::vector<std::vector<cv::Point>> cnts;
  cv::findContours(bw, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  const double A = static_cast<double>(gray.total());
  std::vector<std::pair<double, size_t>> cand;
  for (size_t k = 0; k < cnts.size(); ++k) {
    const double a = cv::contourArea(cnts[k]);
    if (a < label_.min_area * A || a > label_.max_area * A) {continue;}
    const cv::RotatedRect rr = cv::minAreaRect(cnts[k]);
    const double L = std::max(rr.size.width, rr.size.height), S = std::min(rr.size.width, rr.size.height);
    if (S < 8 || L / S < label_.ar_min || L / S > label_.ar_max || a / (rr.size.width * rr.size.height) < label_.fill_min) {continue;}
    cand.emplace_back(a, k);
  }
  std::sort(cand.begin(), cand.end(), [](const auto & x, const auto & y) {return x.first > y.first;});
  if (static_cast<int>(cand.size()) > label_.max_labels) {cand.resize(static_cast<size_t>(std::max(0, label_.max_labels)));}
  for (const auto & [area, k] : cand) {
    (void)area;
    cv::Mat crop;
    std::vector<cv::Point> hull, ap;
    cv::convexHull(cnts[k], hull);
    cv::approxPolyDP(hull, ap, 0.02 * cv::arcLength(hull, true), true);
    if (ap.size() == 4) {
      // 네 꼭짓점 → 직사각형 (원근 보정). 가운데 기준 각도 순으로 놓고 긴 변이 첫 변이 되게
      cv::Point2f c(0.f, 0.f);
      for (const auto & q : ap) {c += cv::Point2f(q) * 0.25f;}
      std::vector<cv::Point2f> q(ap.begin(), ap.end());
      std::sort(q.begin(), q.end(), [&](const cv::Point2f & u, const cv::Point2f & v) {
          return std::atan2(u.y - c.y, u.x - c.x) < std::atan2(v.y - c.y, v.x - c.x);});
      auto edge = [&](int i) {return cv::norm(q[(i + 1) % 4] - q[i]);};
      if (edge(0) + edge(2) < edge(1) + edge(3)) {std::rotate(q.begin(), q.begin() + 1, q.end());}
      const int W = static_cast<int>(std::lround((edge(0) + edge(2)) / 2)), H = static_cast<int>(std::lround((edge(1) + edge(3)) / 2));
      if (W < 8 || H < 4) {continue;}
      const std::vector<cv::Point2f> dst = {{0.f, 0.f}, {float(W), 0.f}, {float(W), float(H)}, {0.f, float(H)}};
      cv::warpPerspective(gray, crop, cv::getPerspectiveTransform(q, dst), cv::Size(W, H), cv::INTER_LINEAR,
        cv::BORDER_CONSTANT, cv::Scalar(255));
    } else {
      // 꼭짓점이 넷이 아니면 최소 외접 사각형으로 돌려 자름
      cv::RotatedRect rr = cv::minAreaRect(cnts[k]);
      if (rr.size.width < rr.size.height) {std::swap(rr.size.width, rr.size.height); rr.angle += 90.f;}
      cv::Mat r;
      cv::warpAffine(gray, r, cv::getRotationMatrix2D(rr.center, rr.angle, 1.0), gray.size(), cv::INTER_LINEAR,
        cv::BORDER_CONSTANT, cv::Scalar(255));
      const cv::Rect box = cv::Rect(cv::Point(cvRound(rr.center.x - rr.size.width / 2), cvRound(rr.center.y - rr.size.height / 2)),
        cv::Size(cvRound(rr.size.width), cvRound(rr.size.height))) & cv::Rect(0, 0, r.cols, r.rows);
      if (box.area() <= 0) {continue;}
      crop = r(box).clone();
    }
    if (!label_lut_.empty()) {cv::Mat t; cv::LUT(crop, label_lut_, t); crop = t;}
    const int py = static_cast<int>(label_.pad * crop.rows) + 4, px = static_cast<int>(label_.pad * crop.cols) + 4;
    cv::Mat lab;
    cv::copyMakeBorder(crop, lab, py, py, px, px, cv::BORDER_CONSTANT, cv::Scalar(255));
    // 바로 세운 뒤라 남은 기울기만: 작은 고정 후보각
    constexpr double angles[] = {0., -4., 4., -8., 8.};
    const size_t before = syms.size();
    for (const double angle : angles) {scan(angle == 0.0 ? lab : rotate_white(lab, angle), syms, dropped);}
    if (syms.size() == before && label_.bar_blur > 0) {
      // 기호가 없을 때만: 돌린 뒤 세로로 흐려 막대 속 잡음을 줄이고 다시 읽음 (막대 방향 평균)
      const int k = static_cast<int>(std::lround(label_.bar_blur * crop.rows)) | 1;
      if (k >= 3) {
        for (const double angle : angles) {
          cv::Mat blurred;
          cv::blur(angle == 0.0 ? lab : rotate_white(lab, angle), blurred, cv::Size(1, k));
          scan(blurred, syms, dropped);
        }
      }
    }
  }
  return static_cast<int>(cand.size());
}

BarcodeRead BarcodeReader::read(const cv::Mat & img)
{
  std::vector<std::string> syms;
  if (img.empty()) {return decide_barcode(syms);}
  cv::Mat g;
  if (img.channels() == 3) {
    cv::cvtColor(img, g, cv::COLOR_BGR2GRAY);
  } else if (img.type() == CV_8U) {
    g = img.isContinuous() ? img : img.clone();
  } else {
    return decide_barcode(syms);
  }
  int dropped = 0, labels = 0;
  if (label_.enabled) {
    labels = read_labels(g, syms, dropped);
    if (!syms.empty()) {
      BarcodeRead r = decide_barcode(syms);
      r.dropped = dropped;
      r.label = true;
      r.labels = labels;
      return r;
    }
  }
  if (!lut_.empty()) {
    cv::Mat t;
    cv::LUT(g, lut_, t);
    g = t;
  }
  if (scale_ != 1.0) {
    cv::Mat big;
    cv::resize(g, big, cv::Size(), scale_, scale_, cv::INTER_CUBIC);
    g = big;
  }
  // 후보각은 영상 내용/정답과 무관하게 고정. 모든 각도의 문자열을 합쳐 충돌을 거부한다.
  for (const double angle : {0., -5., 5., -10., 10., -15., 15., -20., 20., -25., 25., -30., 30.}) {
    scan(angle == 0.0 ? g : rotate_white(g, angle), syms, dropped);
  }
  BarcodeRead r = decide_barcode(syms);
  r.dropped = dropped;
  r.labels = labels;
  return r;
}

}  // namespace perception
