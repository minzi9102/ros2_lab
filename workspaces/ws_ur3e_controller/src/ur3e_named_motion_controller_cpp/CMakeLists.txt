cmake_minimum_required(VERSION 3.8)
project(ur3e_named_motion_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)
find_package(ur3e_controller_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(named_motion_controller_node src/named_motion_controller_node.cpp)
ament_target_dependencies(
  named_motion_controller_node
  controller_manager_msgs
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
  std_msgs
  ur_dashboard_msgs
  ur3e_controller_msgs
)
target_link_libraries(named_motion_controller_node yaml-cpp)

install(
  TARGETS named_motion_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_add_pytest_test(test_target_catalog test/test_target_catalog.py)
  ament_add_pytest_test(
    test_sim_launch_service
    test/test_sim_launch_service.py
    TIMEOUT 300
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
