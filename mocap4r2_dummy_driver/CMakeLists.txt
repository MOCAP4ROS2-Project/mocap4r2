cmake_minimum_required(VERSION 3.5)

project(mocap4r2_dummy_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(mocap4r2_control REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  mocap4r2_msgs
  mocap4r2_control
)

set(target_dependencies
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${mocap4r2_msgs_TARGETS}
  ${mocap4r2_control_LIBRARIES}
)

include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/mocap4r2_dummy_driver/mocap4r2_dummy_driver.cpp)
target_link_libraries(${PROJECT_NAME} ${target_dependencies})

add_executable(mocap4r2_dummy_driver_main
  src/mocap4r2_dummy_driver_main.cpp
)
target_link_libraries(mocap4r2_dummy_driver_main ${target_dependencies})
target_link_libraries(mocap4r2_dummy_driver_main ${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  mocap4r2_dummy_driver_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
