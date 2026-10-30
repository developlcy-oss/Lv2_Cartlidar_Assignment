// 팔 카메라 라벨 인식 색 검출 시험 (ColorDetector label_aware). 위에서 본 퍽 합성 영상, 퍽 30 mm = 60 px
#include <gtest/gtest.h>

#include <cmath>

#include <opencv2/imgproc.hpp>
#include <opencv2/core/version.hpp>
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

#include "perception/detector.hpp"

using namespace perception;

namespace
{
constexpr int RED = 0, GREEN = 1;
const cv::Scalar BG(200, 200, 200), RED_BGR(60, 70, 220), GREEN_BGR(120, 210, 110);

DetectorConfig config(bool label)
{
  DetectorConfig c;   // perception.yaml 과 같은 값
  c.score_threshold = 0.3f;
  c.min_area_ratio = 0.0004;
  c.blur_ksize = 3;
  c.morph_kernel = 3;
  c.min_fill_ratio = 0.35;
  c.label_aware = label;
  return c;
}

// 라벨(흰 40 px) + 태그(검은 32 px, 무늬 흰 칸 몇 개)를 (c, 각도) 에 그린다
void draw_label(cv::Mat & img, cv::Point2f c, float deg)
{
  auto square = [&](float side, const cv::Scalar & col) {
      cv::Point2f p[4];
      cv::RotatedRect(c, {side, side}, deg).points(p);
      std::vector<cv::Point> q;
      for (auto & v : p) {q.emplace_back(cvRound(v.x), cvRound(v.y));}
      cv::fillConvexPoly(img, q, col);
    };
  square(40, cv::Scalar(255, 255, 255));
  square(32, cv::Scalar(20, 20, 20));
  for (int i = -1; i <= 1; i += 2) {
    cv::circle(img, c + cv::Point2f(6.f * i, 4.f), 2, cv::Scalar(255, 255, 255), cv::FILLED);
  }
}

void cylinder_top(cv::Mat & img, cv::Point2f c, const cv::Scalar & col)
{
  cv::circle(img, c, 30, col, cv::FILLED, cv::LINE_AA);
  draw_label(img, c, 0);
}

void cuboid_top(cv::Mat & img, cv::Point2f c, float deg, const cv::Scalar & col)
{
  cv::Point2f p[4];
  cv::RotatedRect(c, {60, 60}, deg).points(p);
  std::vector<cv::Point> q;
  for (auto & v : p) {q.emplace_back(cvRound(v.x), cvRound(v.y));}
  cv::fillConvexPoly(img, q, col, cv::LINE_AA);
  draw_label(img, c, deg);
}

int count_near(const std::vector<Detection> & d, int cls, cv::Point2f c, float tol = 4.f)
{
  int n = 0;
  for (const auto & x : d) {
    const cv::Point2f m(x.box.x + x.box.width / 2, x.box.y + x.box.height / 2);
    if (x.class_id == cls && cv::norm(m - c) <= tol) {++n;}
  }
  return n;
}
}  // namespace

// 원기둥 윗면: 색이 라벨 네 귀퉁이 틈에만 남아 지금 검출기는 못 찾고, 라벨 인식은 퍽 하나(폭 60 px)로 찾는다
TEST(ArmDetect, CylinderTopFound)
{
  cv::Mat img(480, 640, CV_8UC3, BG);
  cylinder_top(img, {320, 240}, RED_BGR);
  EXPECT_EQ(count_near(ColorDetector(config(false)).detect(img), RED, {320, 240}), 0);
  const auto d = ColorDetector(config(true)).detect(img);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_EQ(count_near(d, RED, {320, 240}), 1);
  EXPECT_NEAR(d[0].box.width, 60, 4);
}

// 화면에서 45° 돈 직육면체 윗면: 축 정렬 상자로는 채움 0.28, 회전 외접 사각형 · 구멍 채움이면 1 에 가깝다
TEST(ArmDetect, RotatedCuboidFound)
{
  cv::Mat img(480, 640, CV_8UC3, BG);
  cuboid_top(img, {300, 260}, 45, GREEN_BGR);
  EXPECT_EQ(count_near(ColorDetector(config(false)).detect(img), GREEN, {300, 260}), 0);
  const auto d = ColorDetector(config(true)).detect(img);
  ASSERT_EQ(count_near(d, GREEN, {300, 260}), 1);
  EXPECT_GT(d[0].score, 0.9f);
}

// 맞닿은 같은 색 직육면체 둘: 한 덩어리지만 라벨이 둘이라 둘로 나눈다
TEST(ArmDetect, TouchingSameColorSplit)
{
  cv::Mat img(480, 640, CV_8UC3, BG);
  cuboid_top(img, {290, 240}, 0, GREEN_BGR);
  cuboid_top(img, {350, 240}, 0, GREEN_BGR);
  const auto d = ColorDetector(config(true)).detect(img);
  EXPECT_EQ(d.size(), 2u);
  EXPECT_EQ(count_near(d, GREEN, {290, 240}), 1);
  EXPECT_EQ(count_near(d, GREEN, {350, 240}), 1);
}

// 다른 색이 맞닿으면 라벨은 둘레 색이 많은 쪽 퍽에만 붙는다 (빨강 덩어리에 초록 라벨이 끼어 헛퍽이 생기지 않음)
TEST(ArmDetect, LabelJoinsItsOwnColor)
{
  cv::Mat img(480, 640, CV_8UC3, BG);
  cuboid_top(img, {290, 240}, 0, GREEN_BGR);
  cylinder_top(img, {350, 240}, RED_BGR);
  const auto d = ColorDetector(config(true)).detect(img);
  EXPECT_EQ(d.size(), 2u);
  EXPECT_EQ(count_near(d, GREEN, {290, 240}), 1);
  EXPECT_EQ(count_near(d, RED, {350, 240}), 1);
}

// 라벨 없는 퍽(옆면)은 지금 검출과 같이 찾는다
TEST(ArmDetect, PlainSideStillFound)
{
  cv::Mat img(480, 640, CV_8UC3, BG);
  cv::rectangle(img, {280, 210}, {400, 270}, RED_BGR, cv::FILLED);
  EXPECT_EQ(count_near(ColorDetector(config(true)).detect(img), RED, {340, 240}), 1);
  EXPECT_EQ(count_near(ColorDetector(config(false)).detect(img), RED, {340, 240}), 1);
}

// 어두운 선반이 절대 어둠 문턱에 들어가도 라벨을 분리한다.
TEST(ArmDetect, RelativeThresholdOnDarkBackground)
{
  cv::Mat img(480,640,CV_8UC3,cv::Scalar(70,70,70));
  cylinder_top(img,{320,240},cv::Scalar(10,15,90));
  EXPECT_EQ(count_near(ColorDetector(config(true)).detect(img),RED,{320,240}),1);
}

// 밝은 윗면만 검출하면 40px 아래로 치우친다. 낮은 V의 몸체를 포함해 실제 중심 회복.
TEST(ArmDetect, ShadowSideRestoresBoxCenter)
{
  cv::Mat img(480,640,CV_8UC3,BG);
  cv::rectangle(img,{290,180},{350,300},cv::Scalar(2,4,25),cv::FILLED);
  cv::rectangle(img,{290,260},{350,300},RED_BGR,cv::FILLED);
  draw_label(img,{320,280},0);
  auto before=config(true);before.label_body_v_min=35;
  EXPECT_EQ(count_near(ColorDetector(before).detect(img),RED,{320,240}),0);
  auto after=config(true);after.label_body_v_min=15;
  EXPECT_EQ(count_near(ColorDetector(after).detect(img),RED,{320,240}),1);
}

// 팔 전용 V 문턱이 일반 D435 검출 경로에 적용되지 않는지 확인.
TEST(ArmDetect, ArmThresholdDoesNotChangeD435)
{
  cv::Mat img(480,640,CV_8UC3,BG);
  cv::rectangle(img,{290,210},{350,270},cv::Scalar(2,4,25),cv::FILLED);
  auto a=config(false),b=config(false);a.label_body_v_min=1;b.label_body_v_min=100;
  EXPECT_EQ(ColorDetector(a).detect(img).size(),ColorDetector(b).detect(img).size());
}

namespace {
cv::Mat two_face_labels(int second_id)
{
  cv::Mat img(720,1280,CV_8UC3,BG);
  cv::rectangle(img,{590,240},{690,480},RED_BGR,cv::FILLED);
  for (int k=0;k<2;++k) {
    const cv::Point c(640,300+120*k);
    cv::rectangle(img,c-cv::Point(40,40),c+cv::Point(40,40),cv::Scalar(255,255,255),cv::FILLED);
    cv::Mat marker,bgr;
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11),k?second_id:5,64,marker);
#else
    cv::aruco::drawMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11),k?second_id:5,64,marker);
#endif
    cv::cvtColor(marker,bgr,cv::COLOR_GRAY2BGR);bgr.copyTo(img(cv::Rect(c-cv::Point(32,32),cv::Size(64,64))));
  }
  return img;
}
}
TEST(ArmDetect, SameDecodedPuckFacesAreNotSplit)
{
  const cv::Mat full=two_face_labels(5);cv::Mat small;cv::resize(full,small,{640,360});
  auto cfg=config(true);cfg.label_merge_same_id=true;
  auto dets=ColorDetector(cfg).detect_with_reference(small,full);
  EXPECT_EQ(count_near(dets,RED,{320,180}),1);
}
TEST(ArmDetect, DifferentDecodedPucksRemainSplit)
{
  const cv::Mat full=two_face_labels(7);cv::Mat small;cv::resize(full,small,{640,360});
  auto cfg=config(true);cfg.label_merge_same_id=true;
  auto dets=ColorDetector(cfg).detect_with_reference(small,full);
  EXPECT_EQ(count_near(dets,RED,{320,180}),0);
  EXPECT_GE(dets.size(),2u);
}
