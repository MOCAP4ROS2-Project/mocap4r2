cmake_minimum_required(VERSION 3.10)
project(mocap4r2_robot_gt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap4r2_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED src/gt_math.cpp src/robot_gt.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${geometry_msgs_TARGETS}
  ${mocap4r2_interfaces_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
)
target_link_libraries(${PROJECT_NAME} PRIVATE rclcpp_components::component)
rclcpp_components_register_nodes(${PROJECT_NAME} "mocap4r2_robot_gt::RobotGT")

add_executable(robot_gt src/robot_gt_main.cpp)
target_link_libraries(robot_gt ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS robot_gt RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    # uncrustify 0.72 of Humble formats differently: style is checked on newer distros
    set(ament_cmake_uncrustify_FOUND TRUE)
  endif()
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gt_math test/test_gt_math.cpp)
  target_link_libraries(test_gt_math ${PROJECT_NAME})
  ament_add_gtest(test_robot_gt test/test_robot_gt.cpp TIMEOUT 120)
  target_link_libraries(test_robot_gt ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs mocap4r2_interfaces nav_msgs rclcpp tf2 tf2_geometry_msgs tf2_ros)
ament_package()
