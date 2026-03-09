find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ros_testing REQUIRED)
find_package(moveit_resources_prbt_support REQUIRED)
find_package(moveit_resources_prbt_moveit_config REQUIRED)
find_package(Boost REQUIRED thread)

find_package(${PROJECT_NAME}_testutils REQUIRED)

# Add test directory as include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# pilz_industrial_motion_testhelpers
add_library(${PROJECT_NAME}_test_utils test_utils.cpp)
target_link_libraries(
  ${PROJECT_NAME}_test_utils
  joint_limits_common
  trajectory_generation_common
  Eigen3::Eigen
  ${geometry_msgs_TARGETS}
  moveit_core::moveit_core
  ${moveit_msgs_TARGETS}
  moveit_ros_move_group::moveit_ros_move_group
  moveit_ros_planning::moveit_ros_planning
  pluginlib::pluginlib
  rclcpp::rclcpp
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_eigen_kdl::tf2_eigen_kdl
  ${tf2_geometry_msgs_TARGETS}
  tf2_kdl::tf2_kdl
  tf2_ros::tf2_ros
  ${orocos_kdl_LIBRARIES}
  Boost::headers)

# Unit tests
add_subdirectory(unit_tests)

# Integration tests
add_subdirectory(integration_tests)
# Install test data
install(DIRECTORY test_data DESTINATION share/${PROJECT_NAME})
