cmake_minimum_required(VERSION 3.22)
project(moveit_ros_move_group LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# Finds Boost Components
include(ConfigExtras.cmake)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    fmt
    ament_cmake
    moveit_core
    moveit_ros_occupancy_map_monitor
    moveit_ros_planning
    pluginlib
    rclcpp
    rclcpp_action
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros)

foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(moveit_move_group_capabilities_base SHARED
            src/move_group_context.cpp src/move_group_capability.cpp)
target_include_directories(
  moveit_move_group_capabilities_base
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_ros_move_group>)
set_target_properties(moveit_move_group_capabilities_base
                      PROPERTIES VERSION "${moveit_ros_move_group_VERSION}")
target_link_libraries(
  moveit_move_group_capabilities_base
  fmt::fmt
  moveit_core::moveit_core
  moveit_ros_occupancy_map_monitor::moveit_ros_occupancy_map_monitor
  moveit_ros_planning::moveit_ros_planning
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros)

add_library(
  moveit_move_group_default_capabilities SHARED
  src/default_capabilities/apply_planning_scene_service_capability.cpp
  src/default_capabilities/cartesian_path_service_capability.cpp
  src/default_capabilities/clear_octomap_service_capability.cpp
  src/default_capabilities/execute_trajectory_action_capability.cpp
  src/default_capabilities/get_group_urdf_capability.cpp
  src/default_capabilities/get_planning_scene_service_capability.cpp
  src/default_capabilities/kinematics_service_capability.cpp
  src/default_capabilities/load_geometry_from_file_service_capability.cpp
  src/default_capabilities/move_action_capability.cpp
  src/default_capabilities/plan_service_capability.cpp
  src/default_capabilities/query_planners_service_capability.cpp
  src/default_capabilities/save_geometry_to_file_service_capability.cpp
  src/default_capabilities/state_validation_service_capability.cpp
  src/default_capabilities/multi_state_validation_service_capability.cpp
  src/default_capabilities/tf_publisher_capability.cpp)
target_include_directories(
  moveit_move_group_default_capabilities
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_ros_move_group>)
set_target_properties(moveit_move_group_default_capabilities
                      PROPERTIES VERSION "${moveit_ros_move_group_VERSION}")
target_link_libraries(
  moveit_move_group_default_capabilities
  moveit_move_group_capabilities_base
  fmt::fmt
  moveit_core::moveit_core
  moveit_ros_occupancy_map_monitor::moveit_ros_occupancy_map_monitor
  moveit_ros_planning::moveit_ros_planning
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros)

add_executable(move_group src/move_group.cpp)
target_include_directories(move_group PUBLIC include)
target_link_libraries(
  move_group
  moveit_move_group_capabilities_base
  fmt::fmt
  moveit_core::moveit_core
  moveit_ros_occupancy_map_monitor::moveit_ros_occupancy_map_monitor
  moveit_ros_planning::moveit_ros_planning
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  Boost::headers)

add_executable(list_move_group_capabilities src/list_capabilities.cpp)
target_link_libraries(
  list_move_group_capabilities
  moveit_move_group_capabilities_base
  fmt::fmt
  moveit_core::moveit_core
  moveit_ros_occupancy_map_monitor::moveit_ros_occupancy_map_monitor
  moveit_ros_planning::moveit_ros_planning
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros)

install(TARGETS move_group list_move_group_capabilities
        RUNTIME DESTINATION lib/moveit_ros_move_group)

# Create an umbrella INTERFACE target so downstream packages can link the
# move_group capabilities API via moveit_ros_move_group::moveit_ros_move_group.
# moveit_move_group_default_capabilities is a runtime pluginlib plugin and is
# intentionally not part of this target.
add_library(moveit_ros_move_group INTERFACE)
target_link_libraries(moveit_ros_move_group
                      INTERFACE moveit_move_group_capabilities_base)

install(
  TARGETS moveit_ros_move_group moveit_move_group_capabilities_base
  EXPORT moveit_ros_move_groupTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  TARGETS moveit_move_group_default_capabilities
  EXPORT moveit_move_group_default_capabilitiesTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/moveit_ros_move_group)

install(PROGRAMS scripts/load_map scripts/save_map
        DESTINATION lib/moveit_ros_move_group)

pluginlib_export_plugin_description_file(
  moveit_ros_move_group default_capabilities_plugin_description.xml)

# Process-group cleanup uses the POSIX platforms supported by CI.
if(BUILD_TESTING AND UNIX)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(class_loader REQUIRED)
  find_package(moveit_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(shape_msgs REQUIRED)

  add_library(move_action_test_plugins SHARED test/move_action_test_plugins.cpp)
  target_link_libraries(
    move_action_test_plugins moveit_core::moveit_core ${moveit_msgs_TARGETS}
    class_loader::class_loader rclcpp::rclcpp pluginlib::pluginlib)
  set_target_properties(move_action_test_plugins
                        PROPERTIES VERSION "${moveit_ros_move_group_VERSION}")
  install(TARGETS move_action_test_plugins LIBRARY DESTINATION lib)
  pluginlib_export_plugin_description_file(moveit_core
                                           test/move_action_test_plugins.xml)

  ament_add_gtest_executable(
    test_move_action_planning_time test/test_move_action_planning_time.cpp
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(
    test_move_action_planning_time
    move_action_test_plugins
    moveit_move_group_capabilities_base
    moveit_core::moveit_core
    moveit_ros_planning::moveit_ros_planning
    ${moveit_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${shape_msgs_TARGETS}
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action)

  # One fresh executable per case: ROS/DDS are never initialized in a fork
  # child.
  foreach(
    test_case IN
    ITEMS PlanOnlyReportsPlanningTime
          PlanAndExecuteReportsPlanningTime
          AlreadySatisfiedGoalSkipsPlanningWithZeroTime
          PlanningFailureReportsAttemptPlanningTime
          ExecutionFailureReportsControlFailedAndPlanningTime
          ReplanningReportsCumulativePlanningTime
          SuccessfulFirstPlanInvalidatedThenFailedReplanExcludesExecAndDelay
          CancellationDuringExecutionReportsCanceledAndPlanningTime
          NoStalePlanningTimeAcrossRequests)
    set(result_file
        "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${test_case}.gtest.xml")
    ament_add_test(
      "test_move_action_planning_time.${test_case}"
      COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/run_move_action_test.py"
      --timeout
      30
      "$<TARGET_FILE:test_move_action_planning_time>"
      "--gtest_filter=MoveActionPlanningTimeTest.${test_case}"
      "--gtest_output=xml:${result_file}"
      RESULT_FILE
      "${result_file}"
      OUTPUT_FILE
      "${CMAKE_BINARY_DIR}/move_action_test_logs/${test_case}.log"
      TIMEOUT
      40)
  endforeach()

  ament_add_test(
    test_move_action_launcher
    COMMAND
    "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_run_move_action_test.py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT
    15)

  # TODO(henningkayser): enable rostests find_package(rostest REQUIRED) #
  # rostest under development in ROS2
  # https://github.com/moveit/moveit2/issues/23 this test is flaky
  # add_rostest(test/test_cancel_before_plan_execution.test)
  # add_rostest(test/test_check_state_validity_in_empty_scene.test)
endif()

ament_export_targets(moveit_ros_move_groupTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
