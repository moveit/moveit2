add_library(moveit_utils SHARED src/lexical_casts.cpp src/message_checks.cpp
                                src/rclcpp_utils.cpp src/logger.cpp)
target_include_directories(
  moveit_utils PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include/moveit_core>)
ament_target_dependencies(moveit_utils Boost moveit_msgs rclcpp fmt)
target_link_libraries(moveit_utils rsl::rsl)
set_target_properties(moveit_utils PROPERTIES VERSION
                                              "${${PROJECT_NAME}_VERSION}")

install(DIRECTORY include/ DESTINATION include/moveit_core)

find_package(ament_index_cpp REQUIRED)
add_library(moveit_test_utils SHARED src/robot_model_test_utils.cpp)
target_include_directories(
  moveit_test_utils
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_core>)
target_link_libraries(moveit_test_utils moveit_robot_model
                      moveit_kinematics_base rsl::rsl)
ament_target_dependencies(
  moveit_test_utils
  ament_index_cpp
  Boost
  geometry_msgs
  urdf
  pluginlib
  srdfdom
  urdfdom
  urdfdom_headers
  rclcpp
  fmt)
set_target_properties(moveit_test_utils PROPERTIES VERSION
                                                   "${${PROJECT_NAME}_VERSION}")

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
