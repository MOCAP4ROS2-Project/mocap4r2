cmake_minimum_required(VERSION 3.5)
project(mocap4r2_marker_viz)

set(version 2.0)
set(CMAKE_CXX_STANDARD 17)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mocap4r2_marker_viz_srvs REQUIRED)
find_package(mocap_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  mocap4r2_marker_viz_srvs
  mocap_interfaces
  visualization_msgs
  geometry_msgs
)

add_executable(mocap4r2_marker_viz src/mocap4r2_marker_viz_main.cpp)

add_library(${PROJECT_NAME}_NODE src/mocap4r2_marker_viz_node.cpp)

target_include_directories(${PROJECT_NAME}_NODE
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_NODE ${dependencies})

target_link_libraries(mocap4r2_marker_viz ${PROJECT_NAME}_NODE)

install(TARGETS
  ${PROJECT_NAME}_NODE
  mocap4r2_marker_viz
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
