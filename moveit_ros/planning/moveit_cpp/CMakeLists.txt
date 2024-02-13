add_library(moveit_cpp SHARED src/moveit_cpp.cpp src/planning_component.cpp)
set_target_properties(moveit_cpp PROPERTIES VERSION
                                            "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_cpp rclcpp moveit_core)
target_link_libraries(
  moveit_cpp moveit_planning_scene_monitor moveit_planning_pipeline
  moveit_planning_pipeline_interfaces moveit_trajectory_execution_manager)

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning)

# TODO: Port MoveItCpp test if (BUILD_TESTING)
# find_package(moveit_resources_panda_moveit_config REQUIRED)
# find_package(rostest REQUIRED)
#
# add_rostest_gtest(moveit_cpp_test test/moveit_cpp_test.test
# test/moveit_cpp_test.cpp) target_link_libraries(moveit_cpp_test moveit_cpp
# ${catkin_LIBRARIES}) endif()
