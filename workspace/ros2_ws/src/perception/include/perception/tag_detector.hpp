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

// 영상에서 찾은 태그 하나 (자세 계산 전)
struct TagCorners
{
  int id = -1;
  std::vector<cv::Point2f> corners;   // 왼쪽 위 · 오른쪽 위 · 오른쪽 아래 · 왼쪽 아래 (화소)
};

// 앞 영상에서 찾은 태그 (find_tracked 가 그 둘레만 다시 찾는 데 씀). 영상 흐름마다 부르는 쪽이 하나 가진다
struct TagTrack
{
  std::vector<TagCorners> tags;
  int since_full = 0;   // 마지막 전체 검색 뒤 지난 영상 수
  void reset() {tags.clear(); since_full = 0;}
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
  // find_tracked 에서 이 영상 수마다 한 번 전체 검색 (추적 중 새로 보인 태그를 찾으려고). 0 = 추적 끔, 항상 전체 검색
  void set_track_full_every(int n) {track_full_every_ = n;}
  // K · D 가 비어 있으면 자세 추정 없이 id 만
  std::vector<TagDetection> detect(const cv::Mat & bgr, const cv::Mat & K, const cv::Mat & D) const;
  // detect 를 나눈 두 단계. 찾기는 태그 크기와 상관없으므로 same_search 면 한 번 찾은 결과를 각자 크기로 풀 수 있다
  std::vector<TagCorners> find(const cv::Mat & bgr) const;
  std::vector<TagDetection> solve(const std::vector<TagCorners> & found, const cv::Mat & K, const cv::Mat & D) const;
  // find 와 같되 앞 영상의 태그가 있으면 그 둘레(태그 긴 변의 반 + 2 px 여백)만 찾는다. 원래 크기에서 못 찾으면
  //   같은 둘레를 find 와 같은 배율로 줄여 한 번 더 찾는다 (실물 팔 왕복 녹화: 원래 크기에서만 찾을 때 0.1 rad/s 에서도
  //   등속 구간 3~14 % 를 잃었고, 그 장의 대부분은 줄여 찾으면 찾아짐, 2026-11-10 P9j).
  //   2 배로 키워서도 못 찾으면 그 영상에서 바로 전체 검색 (같은 녹화 재생: 잃음 0, PC 모의 재생 2026-11-10).
  //   Jetson 태그 있는 장 find() 96 ms → 둘레 약 20 ms (2026-11-09 P9h).
  //   줄여 찾는 영상(팔 카메라)만 추적하고, 줄이지 않는 영상은 항상 find 와 같다
  std::vector<TagCorners> find_tracked(const cv::Mat & bgr, TagTrack & track) const;
  // 같은 영상에서 찾은 결과가 항상 같은 설정인가 (태그 크기만 달라도 됨)
  bool same_search(const TagDetector & o) const
  {
    return ok_ && o.ok_ && dict_id_ == o.dict_id_ && gamma_ == o.gamma_ && upscale_all_ == o.upscale_all_ &&
           max_width_ == o.max_width_ && texture_pass_ == o.texture_pass_;
  }

private:
  // roi 안에서 번호 id · 가운데가 c0 에서 gate 안인 태그를 찾아 q(원래 크기 좌표)에 넣는다 (못 찾으면 false, q 그대로).
  //   scale ≠ 1 이면 roi 를 그 배율로 바꿔서 찾는다 (find 의 줄여 찾기와 같은 배율이면 원래 크기에서 놓치는 흐린 태그를 찾음)
  bool refine_in(const cv::Mat & full_gray, const cv::Rect & roi, int id, const cv::Point2f & c0, double gate,
    std::vector<cv::Point2f> & q, double scale = 1.0) const;

  int dict_id_ = 0;
  double size_ = 0.1;
  double gamma_ = 1.0;
  bool upscale_all_ = false;
  bool ok_ = false;
  bool aruco_ = false;
  int max_width_ = 0;
  bool texture_pass_ = false;
  int track_full_every_ = 0;
};

}  // namespace perception
