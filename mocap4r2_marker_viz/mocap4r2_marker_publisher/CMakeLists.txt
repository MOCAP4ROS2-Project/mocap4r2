cmake_minimum_required(VERSION 3.5)
project(mocap4r2_marker_publisher)
set(version 2.0)

set (CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(mocap_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mocap4r2_marker_publisher src/mocap4r2_marker_publisher.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp mocap_interfaces)

install(TARGETS
  mocap4r2_marker_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
