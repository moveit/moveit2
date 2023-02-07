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
set_target_properties(moveit_utils PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

install(DIRECTORY include/ DESTINATION include/moveit_core)


find_package(ament_index_cpp REQUIRED)
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
set_target_properties(moveit_test_utils PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
