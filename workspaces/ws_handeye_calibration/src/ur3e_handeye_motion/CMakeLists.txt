cmake_minimum_required(VERSION 3.8)
project(ur3e_handeye_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)
find_package(ur_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanPose.srv"
  "action/ExecutePlan.action"
  DEPENDENCIES builtin_interfaces geometry_msgs moveit_msgs sensor_msgs
)

add_executable(handeye_motion_node src/handeye_motion_node.cpp)
target_include_directories(handeye_motion_node PRIVATE include)
ament_target_dependencies(handeye_motion_node
  controller_manager_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2_ros
  ur_dashboard_msgs
  ur_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(handeye_motion_node "${cpp_typesupport_target}")

install(TARGETS handeye_motion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_motion_safety test/test_motion_safety.cpp)
  target_include_directories(test_motion_safety PRIVATE include)
  ament_target_dependencies(test_motion_safety geometry_msgs ur_dashboard_msgs)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
