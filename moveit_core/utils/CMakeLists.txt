add_library(moveit_utils SHARED
  src/lexical_casts.cpp
  src/message_checks.cpp
  src/rclcpp_utils.cpp
)
target_include_directories(moveit_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/moveit_core>
)
ament_target_dependencies(moveit_utils Boost moveit_msgs)

install(DIRECTORY include/ DESTINATION include/moveit_core)

add_library(moveit_test_utils SHARED src/robot_model_test_utils.cpp)
target_include_directories(moveit_test_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/moveit_core>
)
target_link_libraries(moveit_test_utils moveit_robot_model)
ament_target_dependencies(moveit_test_utils
  ament_index_cpp
  Boost
  geometry_msgs
  urdf
  srdfdom
  urdfdom
  urdfdom_headers
)
