find_package(ament_cmake_gtest REQUIRED)
find_package(diagnostic_msgs REQUIRED)

ament_add_gtest(test_frame_stats test_frame_stats.cpp)
target_link_libraries(test_frame_stats ${PROJECT_NAME})

ament_add_gtest(test_mocap_system_node test_mocap_system_node.cpp TIMEOUT 120)
target_link_libraries(test_mocap_system_node ${PROJECT_NAME} ${diagnostic_msgs_TARGETS})
