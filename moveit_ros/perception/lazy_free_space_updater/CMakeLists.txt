add_library(moveit_lazy_free_space_updater SHARED
            src/lazy_free_space_updater.cpp)
set_target_properties(moveit_lazy_free_space_updater
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(
  moveit_lazy_free_space_updater
  PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(moveit_lazy_free_space_updater
                      PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")
if(APPLE)
  target_link_libraries(moveit_lazy_free_space_updater OpenMP::OpenMP_CXX)
endif()
target_link_libraries(
  moveit_lazy_free_space_updater
  PUBLIC ${sensor_msgs_TARGETS}
         moveit_ros_occupancy_map_monitor::moveit_ros_occupancy_map_monitor
         rclcpp::rclcpp sensor_msgs::sensor_msgs_library)

install(DIRECTORY include/ DESTINATION include/moveit_ros_perception)
