target_sources(moveit_core PRIVATE src/kinematics_metrics.cpp)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(moveit_core
  urdf
  urdfdom_headers
  visualization_msgs
)

install(DIRECTORY include/ DESTINATION include/moveit_core)
