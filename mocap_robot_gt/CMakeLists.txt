cmake_minimum_required(VERSION 3.8)
project(mocap_robot_gt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mocap_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
    rclcpp
    rclcpp_components
    mocap_msgs
    tf2_ros
    geometry_msgs
    tf2_geometry_msgs
)

include_directories(
  include
)

add_library(gt_component SHARED src/mocap_robot_gt/gt_component.cpp)
ament_target_dependencies(gt_component ${dependencies})
rclcpp_components_register_nodes(gt_component "mocap_robot_gt::GTNode")

add_executable(gt_program src/gt_program.cpp)
ament_target_dependencies(gt_component ${dependencies})
target_link_libraries(gt_program gt_component)

install(TARGETS
  gt_component
  gt_program
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
