cmake_minimum_required(VERSION 3.10)
project(mocap4r2_sim_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mocap4r2_control REQUIRED)
find_package(mocap4r2_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(${PROJECT_NAME} SHARED src/sim_scene.cpp src/sim_driver.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  mocap4r2_control::mocap4r2_control
  ${mocap4r2_interfaces_TARGETS}
  rclcpp::rclcpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE rclcpp_components::component)
rclcpp_components_register_nodes(${PROJECT_NAME} "mocap4r2_sim_driver::SimDriver")

add_executable(sim_driver src/sim_driver_main.cpp)
target_link_libraries(sim_driver ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS sim_driver RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    # uncrustify 0.72 of Humble formats differently: style is checked on newer distros
    set(ament_cmake_uncrustify_FOUND TRUE)
  endif()
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sim_scene test/test_sim_scene.cpp)
  target_link_libraries(test_sim_scene ${PROJECT_NAME})
  find_package(tf2_msgs REQUIRED)
  ament_add_gtest(test_sim_driver test/test_sim_driver.cpp TIMEOUT 120)
  target_link_libraries(test_sim_driver ${PROJECT_NAME} ${tf2_msgs_TARGETS})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mocap4r2_control mocap4r2_interfaces rclcpp)
ament_package()
