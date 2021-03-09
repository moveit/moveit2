set(MOVEIT_LIB_NAME moveit_semantic_world)

add_library(${MOVEIT_LIB_NAME} SHARED src/semantic_world.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
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
target_link_libraries(${MOVEIT_LIB_NAME} ${OpenCV_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)
