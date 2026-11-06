// AprilTag (tag36h11) 검출 · 자세 추정 — OpenCV aruco 의 AprilTag 사전 사용 (별도 apriltag 라이브러리 불필요)
//   OpenCV ≥ 4.7: objdetect 의 cv::aruco::ArucoDetector / 4.6: contrib aruco::detectMarkers
//   자세: solvePnP(IPPE_SQUARE) — 광학 좌표(x 오른쪽 · y 아래 · z 앞)
#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace perception
{

struct TagDetection
{
  int id = -1;
  double area = 0;            // 화소 넓이 (가장 크게 보이는 태그 고르기)
  std::vector<cv::Point2f> corners;   // 왼쪽 위 · 오른쪽 위 · 오른쪽 아래 · 왼쪽 아래 (화소)
  bool pose_valid = false;
  cv::Vec3d rvec, tvec;       // 광학 좌표, m
  double err = 0;             // rvec · tvec 의 재투영 오차 [px] (IPPE)
  bool alt_valid = false;     // 카메라를 향하는 IPPE 의 다른 해 (작은 태그는 두 해의 오차가 거의 같음 → tag_upright.hpp)
  cv::Vec3d rvec_alt, tvec_alt;
  double err_alt = 0;
};

class TagDetector
{
public:
  // family: "tag36h11" (기본) · "tag25h9" · "tag16h5"
  //         Aruco(과제 마커): "DICT_4X4_50" · "DICT_4X4_100" · "DICT_5X5_50" · "DICT_6X6_50"
  TagDetector(const std::string & family, double tag_size_m, double gamma = 1.0, bool upscale_all = false);
  bool ok() const {return ok_;}
  // 폭이 이보다 큰 영상은 이 폭으로 줄여 찾고, 꼭짓점은 원래 영상에서 다시 다듬는다(0 = 줄이지 않음).
  //   팔 Arducam 1280 × 720 에서 전체 장 검출이 Jetson 한 장 약 0.23 s(못 찾은 장은 2배 확대 재검출까지 약 0.8 s)라
  //   예전 팔 영상 크기(640 폭)로 줄여 같은 처리량으로 돌린다 (2026-11-05 실물 표본)
  void set_max_width(int w) {max_width_ = w;}
  // 줄인 영상에서 '무늬 거르기' 문턱(큰 창 · 큰 상수)으로 한 번 더 찾는다. 줄이지 않는 영상(D435)에는 쓰지 않는다.
  //   흰 여백이 좁으면 카펫 같은 바닥 무늬의 윤곽이 태그 테두리에 붙어 사각형을 못 찾는다. 상수를 키우면 약한 무늬가 빠진다
  //   (실물 장소 태그 2026-11-06, 261106_0917_real_judge2)
  void set_texture_pass(bool on) {texture_pass_ = on;}
  // K · D 가 비어 있으면 자세 추정 없이 id 만
  std::vector<TagDetection> detect(const cv::Mat & bgr, const cv::Mat & K, const cv::Mat & D) const;

private:
  int dict_id_ = 0;
  double size_ = 0.1;
  double gamma_ = 1.0;
  bool upscale_all_ = false;
  bool ok_ = false;
  bool aruco_ = false;
  int max_width_ = 0;
  bool texture_pass_ = false;
};

}  // namespace perception
