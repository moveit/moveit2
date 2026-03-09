add_library(moveit_utils SHARED src/lexical_casts.cpp src/message_checks.cpp
                                src/rclcpp_utils.cpp src/logger.cpp)
target_include_directories(
  moveit_utils PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include/moveit_core>)
target_link_libraries(moveit_utils rsl::rsl Boost::headers
                      ${moveit_msgs_TARGETS} rclcpp::rclcpp fmt::fmt)
set_target_properties(moveit_utils PROPERTIES VERSION
                                              "${${PROJECT_NAME}_VERSION}")

install(DIRECTORY include/ DESTINATION include/moveit_core)

find_package(ament_index_cpp REQUIRED)
add_library(moveit_test_utils SHARED src/robot_model_test_utils.cpp)
target_include_directories(
  moveit_test_utils
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_core>)
target_link_libraries(
  moveit_test_utils
  moveit_robot_model
  moveit_kinematics_base
  rsl::rsl
  ament_index_cpp::ament_index_cpp
  Boost::headers
  ${geometry_msgs_TARGETS}
  urdf::urdf
  pluginlib::pluginlib
  srdfdom::srdfdom
  urdfdom::urdf_parser
  urdfdom_headers::urdfdom_headers
  rclcpp::rclcpp
  fmt::fmt)
set_target_properties(moveit_test_utils PROPERTIES VERSION
                                                   "${${PROJECT_NAME}_VERSION}")

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
