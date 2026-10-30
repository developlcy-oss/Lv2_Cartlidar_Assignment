// 검출 클래스 번호 → 퍽 색 · 모양 (perception_node, ROS 의존 없음 → 단위 시험)
//   class_names 가 비면 ColorDetector 번호(0 빨강 · 1 초록 · 2 파랑, 모양 모름)
//   class_names 가 있으면 이름에 든 낱말로 정한다: red · green · blue → 색, cylinder · cuboid → 모양.
//     예: [red_cylinder, green_cylinder, red_cuboid, green_cuboid] (YOLO 4 클래스)
//   번호가 목록 밖이거나 낱말이 없으면 그 값은 모름(0)
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace perception
{

struct PuckClass
{
  uint8_t color = 0;   // Puck.COLOR_* (0 모름 · 1 빨강 · 2 초록 · 3 파랑)
  uint8_t shape = 0;   // Puck.SHAPE_* (0 모름 · 1 원기둥 · 2 직육면체)
};

inline PuckClass puck_class_from_name(const std::string & name)
{
  auto has = [&](const char * w) {return name.find(w) != std::string::npos;};
  PuckClass c;
  if (has("red")) {c.color = 1;} else if (has("green")) {c.color = 2;} else if (has("blue")) {c.color = 3;}
  if (has("cylinder")) {c.shape = 1;} else if (has("cuboid")) {c.shape = 2;}
  return c;
}

inline PuckClass puck_class(int class_id, const std::vector<std::string> & class_names)
{
  PuckClass c;
  if (class_names.empty()) {
    if (class_id >= 0 && class_id < 3) {c.color = static_cast<uint8_t>(class_id + 1);}
    return c;
  }
  if (class_id < 0 || class_id >= static_cast<int>(class_names.size())) {return c;}
  return puck_class_from_name(class_names[class_id]);
}

}  // namespace perception
