cmake_minimum_required(VERSION 3.8)
project(tool_point_calibration_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}
  src/tool_point_calibration.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME} Eigen3)

add_executable(console_calibration_node src/console_calibration_node.cpp)
target_link_libraries(console_calibration_node ${PROJECT_NAME})
ament_target_dependencies(
  console_calibration_node
  geometry_msgs
  rclcpp
  tf2_eigen
  tf2_ros
)

add_executable(console_paper_calibration_node src/console_paper_calibration_node.cpp)
target_link_libraries(console_paper_calibration_node ${PROJECT_NAME})
ament_target_dependencies(
  console_paper_calibration_node
  rclcpp
  tf2_ros
)

install(
  TARGETS ${PROJECT_NAME} console_calibration_node console_paper_calibration_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/watch_calibrated_tcp.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_tool_point_calibration test/test_tool_point_calibration.cpp)
  target_link_libraries(test_tool_point_calibration ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Eigen3)
ament_package()
