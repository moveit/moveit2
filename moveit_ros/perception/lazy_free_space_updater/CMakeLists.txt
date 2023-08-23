add_library(moveit_lazy_free_space_updater SHARED src/lazy_free_space_updater.cpp)
set_target_properties(moveit_lazy_free_space_updater PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(moveit_lazy_free_space_updater PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(moveit_lazy_free_space_updater PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")
if(APPLE)
  target_link_libraries(moveit_lazy_free_space_updater OpenMP::OpenMP_CXX)
endif()
ament_target_dependencies(moveit_lazy_free_space_updater
  rclcpp
  moveit_ros_occupancy_map_monitor
  sensor_msgs
)

install(DIRECTORY include/ DESTINATION include/moveit_ros_perception)
