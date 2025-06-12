add_library(moveit_point_containment_filter SHARED src/shape_mask.cpp)
set_target_properties(moveit_point_containment_filter
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(
  moveit_point_containment_filter
  PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(moveit_point_containment_filter
                      PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")
target_link_libraries(
  moveit_point_containment_filter
  PUBLIC ${sensor_msgs_TARGETS} geometric_shapes::geometric_shapes
         moveit_core::moveit_collision_detection rclcpp::rclcpp
         sensor_msgs::sensor_msgs_library)

install(DIRECTORY include/ DESTINATION include/moveit_ros_perception)
