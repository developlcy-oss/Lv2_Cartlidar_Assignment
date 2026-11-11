cmake_minimum_required(VERSION 3.20)
project(mission)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(action_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)   # 퍽 측량 완료 파일 · revisit 응답 (JSON) 읽기
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)   # survey 조사 자세 = mission_planner approach_poses.yaml

# --- MissionNode (공유 라이브러리 + component 등록, ⑨ mission_container) ---
add_library(mission_component SHARED src/mission_node.cpp)
target_include_directories(mission_component PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mission_component PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  ${action_msgs_TARGETS}
  ${robot_interfaces_TARGETS}
  ${nav2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  yaml-cpp::yaml-cpp
  ament_index_cpp::ament_index_cpp
)
rclcpp_components_register_node(mission_component
  PLUGIN "mission::MissionNode"
  EXECUTABLE mission_node
  EXECUTOR MultiThreadedExecutor
)

install(TARGETS mission_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 장소 재검증 기록 · 전체 중복/누락 판정 (verify_record.hpp, 헤더만)
  ament_add_gtest(test_verify_record test/test_verify_record.cpp)
  target_include_directories(test_verify_record PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  # 미션 기록 폴더 · 사진 접두어 · 사본 (episode_log.hpp, 헤더만)
  ament_add_gtest(test_episode_log test/test_episode_log.cpp)
  target_include_directories(test_episode_log PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  # 측량 완료 파일 -> 퍽 등록표 (pose 포함, survey_pucks.hpp, 헤더만)
  ament_add_gtest(test_survey_pucks test/test_survey_pucks.cpp)
  target_include_directories(test_survey_pucks PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_survey_pucks yaml-cpp::yaml-cpp)
  # 복귀 판정: 차체 네 모서리 · 출발 구역 (return_check.hpp, 헤더만)
  ament_add_gtest(test_return_check test/test_return_check.cpp)
  target_include_directories(test_return_check PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
