set(MOVEIT_LIB_NAME moveit_cpp)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/moveit_cpp.cpp
  src/planning_component.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  rclcpp
  moveit_core
  moveit_ros_planning
)

install(TARGETS ${MOVEIT_LIB_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
