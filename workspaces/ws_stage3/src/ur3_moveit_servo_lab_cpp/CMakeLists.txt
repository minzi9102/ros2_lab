cmake_minimum_required(VERSION 3.8)
project(ur3_moveit_servo_lab_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(servo_twist_commander_node src/servo_twist_commander_node.cpp)
ament_target_dependencies(
  servo_twist_commander_node
  rclcpp
  geometry_msgs
  moveit_msgs
)

install(
  TARGETS servo_twist_commander_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/wait_for_joint_states.py
    scripts/wait_for_servo_status.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
