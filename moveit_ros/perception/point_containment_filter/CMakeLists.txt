add_library(moveit_point_containment_filter SHARED src/shape_mask.cpp)
set_target_properties(moveit_point_containment_filter PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(moveit_point_containment_filter PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(moveit_point_containment_filter PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")
ament_target_dependencies(moveit_point_containment_filter
  rclcpp
  sensor_msgs
  geometric_shapes
)

install(DIRECTORY include/ DESTINATION include/moveit_ros_perception)
