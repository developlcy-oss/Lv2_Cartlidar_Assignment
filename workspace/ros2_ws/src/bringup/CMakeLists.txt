cmake_minimum_required(VERSION 3.16)
project(bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# #92 회전 중심 옆 이동 -> EKF. C++ (#265: Python 판은 Jetson 에서 CPU 28 %). 컴포넌트 bringup::OdomIcr + 실행 파일 odom_icr
add_library(odom_icr_component SHARED src/odom_icr.cpp)
target_compile_features(odom_icr_component PUBLIC cxx_std_17)
target_link_libraries(odom_icr_component PUBLIC rclcpp::rclcpp rclcpp_components::component ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS})
rclcpp_components_register_node(odom_icr_component PLUGIN "bringup::OdomIcr" EXECUTABLE odom_icr)
install(TARGETS odom_icr_component ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
