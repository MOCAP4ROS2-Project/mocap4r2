cmake_minimum_required(VERSION 3.10)
project(mocap4r2_marker_viz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mocap4r2_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/viz_builder.cpp src/marker_viz_node.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${mocap4r2_interfaces_TARGETS}
  rclcpp::rclcpp
  ${visualization_msgs_TARGETS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE rclcpp_components::component)
rclcpp_components_register_nodes(${PROJECT_NAME} "mocap4r2_marker_viz::MarkerVizNode")

add_executable(marker_viz src/marker_viz_main.cpp)
target_link_libraries(marker_viz ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS marker_viz RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    # uncrustify 0.72 of Humble formats differently: style is checked on newer distros
    set(ament_cmake_uncrustify_FOUND TRUE)
  endif()
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_viz_builder test/test_viz_builder.cpp)
  target_link_libraries(test_viz_builder ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mocap4r2_interfaces rclcpp visualization_msgs)
ament_package()
