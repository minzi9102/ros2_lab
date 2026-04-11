cmake_minimum_required(VERSION 3.8)
project(ur3_stage1_review_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(demo_package_build_cpp src/demo_package_build.cpp)
ament_target_dependencies(demo_package_build_cpp rclcpp)

install(TARGETS demo_package_build_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
