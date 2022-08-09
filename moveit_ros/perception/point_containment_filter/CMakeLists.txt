set(MOVEIT_LIB_NAME moveit_point_containment_filter)

add_library(${MOVEIT_LIB_NAME} SHARED src/shape_mask.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  rclcpp
  sensor_msgs
  geometric_shapes
)

install(DIRECTORY include/ DESTINATION include)
