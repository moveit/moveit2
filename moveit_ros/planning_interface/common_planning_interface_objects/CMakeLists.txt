set(MOVEIT_LIB_NAME moveit_common_planning_interface_objects)

add_library(${MOVEIT_LIB_NAME} SHARED src/common_objects.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
    rclcpp
    moveit_ros_planning
    tf2_ros
)

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
