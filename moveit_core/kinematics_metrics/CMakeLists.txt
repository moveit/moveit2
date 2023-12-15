add_library(moveit_kinematics_metrics SHARED src/kinematics_metrics.cpp)
target_include_directories(moveit_kinematics_metrics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/moveit_core>
)
set_target_properties(moveit_kinematics_metrics PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(moveit_kinematics_metrics
  urdf
  urdfdom_headers
  visualization_msgs)

target_link_libraries(moveit_kinematics_metrics
  moveit_robot_model
  moveit_robot_state
  moveit_utils
)

install(DIRECTORY include/ DESTINATION include/moveit_core)
