set(MOVEIT_LIB_NAME moveit_planning_interface)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/planning_interface.cpp
  src/planning_response.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_msgs
  urdf
  urdfdom
  urdfdom_headers
)
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_trajectory
  moveit_robot_state
)

install(DIRECTORY include/ DESTINATION include)
