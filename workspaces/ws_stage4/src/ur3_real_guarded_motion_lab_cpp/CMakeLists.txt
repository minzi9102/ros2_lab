cmake_minimum_required(VERSION 3.8)
project(ur3_real_guarded_motion_lab_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(guarded_joint_motion_node src/guarded_joint_motion_node.cpp)
ament_target_dependencies(
  guarded_joint_motion_node
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
)

add_executable(planned_home_motion_node src/planned_home_motion_node.cpp)
target_include_directories(planned_home_motion_node PRIVATE include)
ament_target_dependencies(
  planned_home_motion_node
  moveit_ros_planning_interface
  rclcpp
)

install(
  TARGETS guarded_joint_motion_node planned_home_motion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_planned_home_utils test/test_planned_home_utils.cpp)
  target_include_directories(test_planned_home_utils PRIVATE include)
endif()

ament_package()
