// 바코드 문자열 읽기 — 팔 카메라 (zbar, ROS 의존 없음 → 단위 시험)
//   기준 Python sim_render/proto_r3/zbar_check.py(sha256 앞 16자 ba6a17bac600cb5c, R3 설계 44차)와 같은 순서 · 설정:
//   1) 회색조 → 감마 LUT out = round(255·(g/255)^gamma) (기본 2: 막대 1.5~1.8 화소에서 흐려진 좁은 흰 칸을 살림)
//   2) scale 배 확대(1 = 원본, 기본. 1 이 아니면 INTER_CUBIC). 입력 해상도는 가정하지 않는다(팔 카메라는 640 × 480)
//   3) zbar: CODE128 만 켬, X · Y 훑기 간격 1(모든 열 · 행)
//   4) quality(그 문자열을 읽은 스캔 수) ≥ min_quality 인 기호만 씀 (기본 4: 체크섬이 상쇄되는 잘못 읽기 P2-0x → PF-&x 는
//      몇 스캔에서만 나옴 — 설계 자료에서 감마 2 최대 2, 맞는 문자열 최소 34)
//   한 장에서 서로 다른 문자열이 둘 이상이면 어느 것이 맞는지 모르므로 '못 읽음'
//   (판정 1차 틀린 문자열 1건이 ['PF-&3', 'P2-03'] 두 개였음). 문자열 형식 검사는 하지 않는다.
// 라벨 찾기(BarcodeLabelConfig.enabled, 실물 Arducam 2026-11-05): 위 전체 장 읽기 앞에 먼저 한다.
//   실물 라벨은 흰 여백이 좁고 둘레가 무늬 있는 바닥 · 벽이라 Code 128 의 조용한 구역이 없어 전체 장에서는 기호를
//   하나도 못 찾았다(1038장 0). 그래서 흰 라벨(밝은 볼록 사각형)을 찾아 → 네 꼭짓점으로 원근 보정해 긴 변을 가로로 →
//   둘레에 흰 여백 → 라벨용 감마 → 작은 회전 후보로 읽는다. 여기서 기호가 하나라도 나오면 그것으로 결정하고, 없으면
//   예전처럼 전체 장을 읽는다(모의 렌더 R3 은 흰 벽이라 전체 장 길로 그대로 읽힘)
//   라벨을 그냥 읽어 기호가 없으면 같은 후보를 막대 방향(세로)으로 흐려 한 번 더 읽는다. 막대 속 잡음이 zbar 의 경계를
//   흔들던 장이 읽힌다(설계 자료 못 읽은 47장 중 34장, Python 시제품)
#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace perception
{

struct BarcodeRead
{
  bool ok = false;                  // 서로 다른 문자열이 정확히 하나
  std::string text;                 // ok 일 때
  std::vector<std::string> symbols; // quality 거르기를 통과한 기호 문자열 (zbar 가 낸 순서)
  int dropped = 0;                  // quality 가 모자라 뺀 기호 수
  bool label = false;               // 라벨 찾기 길에서 결정함 (false = 전체 장 길)
  int labels = 0;                   // 찾은 라벨 후보 수
};

struct BarcodeLabelConfig
{
  bool enabled = false;
  double rel = 0.85;                // 라벨 문턱 = rel × 밝기 99.5 백분위 (5 × 5 흐림 뒤)
  double min_area = 0.004;          // 라벨 넓이 하한 · 상한 (화면 넓이 비)
  double max_area = 0.5;
  double ar_min = 1.8;              // 긴 변 / 짧은 변 (라벨 64.8 × 17.7 mm 는 3.7, 바코드 막대 부분만 잡히면 더 김)
  double ar_max = 6.0;
  double fill_min = 0.75;           // 윤곽 넓이 / 최소 외접 사각형 넓이
  int max_labels = 3;               // 넓이 큰 것부터
  double pad = 0.35;                // 흰 여백 = 자른 라벨 폭 · 높이 × pad (조용한 구역)
  double gamma = 0.5;               // 라벨용 감마 (실물 설계 자료: 2 이면 회색 막대가 번져 좁은 흰 칸이 막힘)
  double bar_blur = 0.13;           // 막대 방향 흐림 폭 = 자른 라벨 높이 × bar_blur. 0 이면 안 함. 그냥 읽어 기호가 없을 때만 함
};

// 한 장의 기호 문자열들 → 결정 (Py: decide)
BarcodeRead decide_barcode(const std::vector<std::string> & symbols);

class BarcodeReader
{
public:
  explicit BarcodeReader(double scale = 1.0, double gamma = 2.0, int min_quality = 4, int x_density = 1, int y_density = 1);
  ~BarcodeReader();
  BarcodeReader(const BarcodeReader &) = delete;
  BarcodeReader & operator=(const BarcodeReader &) = delete;

  // img: 회색조(CV_8U) 또는 BGR(CV_8UC3, 회색조로 바꿈). 한 스레드에서만 부른다
  BarcodeRead read(const cv::Mat & img);
  void set_label(const BarcodeLabelConfig & c);

private:
  // 라벨 길: 찾은 라벨들의 기호(quality 거르기 뒤)를 syms 에. 찾은 라벨 수를 돌려줌
  int read_labels(const cv::Mat & gray, std::vector<std::string> & syms, int & dropped);
  void scan(const cv::Mat & g, std::vector<std::string> & syms, int & dropped);
  BarcodeLabelConfig label_;
  cv::Mat label_lut_;

  double scale_ = 1.0;
  int min_quality_ = 4;
  cv::Mat lut_;                     // 감마 (1 × 256, CV_8U). 감마 1 이면 비움
  void * scanner_ = nullptr;        // zbar_image_scanner_t
};

}  // namespace perception
