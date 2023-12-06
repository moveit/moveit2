add_library(moveit_robot_trajectory SHARED src/robot_trajectory.cpp)
target_include_directories(moveit_robot_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/moveit_core>
)
set_target_properties(moveit_robot_trajectory PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
ament_target_dependencies(moveit_robot_trajectory
  rclcpp
  urdfdom
  urdfdom_headers
)

target_link_libraries(moveit_robot_trajectory
  moveit_robot_model
  moveit_robot_state
  moveit_utils
)

install(DIRECTORY include/ DESTINATION include/moveit_core)

if(BUILD_TESTING)
  ament_add_gtest(test_robot_trajectory test/test_robot_trajectory.cpp)
  target_link_libraries(test_robot_trajectory moveit_test_utils moveit_robot_trajectory)
endif()
