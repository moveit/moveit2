add_library(moveit_semantic_world SHARED src/semantic_world.cpp)
set_target_properties(moveit_semantic_world
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_semantic_world
  PUBLIC rclcpp::rclcpp
         moveit_core::moveit_core
         ${object_recognition_msgs_TARGETS}
         ${visualization_msgs_TARGETS}
         ${geometry_msgs_TARGETS}
         geometric_shapes::geometric_shapes
         ${moveit_msgs_TARGETS}
         tf2_eigen::tf2_eigen
         Eigen3::Eigen
         Boost::headers
         ${OpenCV_LIBS})

install(DIRECTORY include/ DESTINATION include/moveit_ros_perception)
