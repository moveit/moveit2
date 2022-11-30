add_library(moveit_planning_interface SHARED
  src/planning_interface.cpp
  src/planning_response.cpp
)
set_target_properties(moveit_planning_interface PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_planning_interface
  moveit_msgs
  urdf
  urdfdom
  urdfdom_headers
)
target_link_libraries(moveit_planning_interface
  moveit_robot_trajectory
  moveit_robot_state
)

install(DIRECTORY include/ DESTINATION include)
