add_library(moveit_semantic_world SHARED src/semantic_world.cpp)
set_target_properties(moveit_semantic_world PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_semantic_world
  rclcpp
  moveit_core
  object_recognition_msgs
  visualization_msgs
  geometry_msgs
  geometric_shapes
  moveit_msgs
  tf2_eigen
  Eigen3
  Boost
)
target_link_libraries(moveit_semantic_world ${OpenCV_LIBRARIES})

install(DIRECTORY include/ DESTINATION include/moveit_ros_perception)
