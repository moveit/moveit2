find_package(ament_cmake_gtest REQUIRED)
find_package(pluginlib REQUIRED)
ament_add_gtest(test_controller_manager_plugin
                test_controller_manager_plugin.cpp TIMEOUT 20)
target_link_libraries(test_controller_manager_plugin
                      moveit_ros_control_interface_plugin)
target_include_directories(test_controller_manager_plugin
                           PRIVATE ${CMAKE_SOURCE_DIR}/include)
