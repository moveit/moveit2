target_sources(moveit_core PRIVATE
  src/lexical_casts.cpp
  src/message_checks.cpp
  src/rclcpp_utils.cpp
)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(moveit_core Boost moveit_msgs)

install(DIRECTORY include/ DESTINATION include/moveit_core)

target_sources(moveit_core PRIVATE src/robot_model_test_utils.cpp)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(moveit_core
  ament_index_cpp
  Boost
  geometry_msgs
  urdf
  srdfdom
  urdfdom
  urdfdom_headers
)
