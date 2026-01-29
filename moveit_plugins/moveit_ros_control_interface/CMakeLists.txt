cmake_minimum_required(VERSION 3.22)
project(moveit_ros_control_interface LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp_action controller_manager_msgs moveit_core
    moveit_simple_controller_manager pluginlib trajectory_msgs)

foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()


include_directories(include)


add_library(moveit_ros_control_interface_plugin SHARED
            src/controller_manager_plugin.cpp)
set_target_properties(
  moveit_ros_control_interface_plugin
  PROPERTIES VERSION "${moveit_ros_control_interface_VERSION}")
target_include_directories(moveit_ros_control_interface_plugin PRIVATE include)
target_link_libraries(moveit_ros_control_interface_plugin PUBLIC
      ${controller_manager_msgs_TARGETS}
      ${trajectory_msgs_TARGETS}
      moveit_core::moveit_collision_detection
      moveit_core::moveit_collision_detection_bullet
      moveit_core::moveit_collision_detection_fcl
      moveit_core::moveit_collision_distance_field
      moveit_core::moveit_constraint_samplers
      moveit_core::moveit_distance_field
      moveit_core::moveit_dynamics_solver
      moveit_core::moveit_exceptions
      moveit_core::moveit_kinematic_constraints
      moveit_core::moveit_kinematics_base
      moveit_core::moveit_kinematics_metrics
      moveit_core::moveit_macros
      moveit_core::moveit_planning_interface
      moveit_core::moveit_planning_scene
      moveit_core::moveit_robot_model
      moveit_core::moveit_robot_state
      moveit_core::moveit_robot_trajectory
      moveit_core::moveit_smoothing_base
      moveit_core::moveit_test_utils
      moveit_core::moveit_trajectory_processing
      moveit_core::moveit_transforms
      moveit_core::moveit_utils
      moveit_simple_controller_manager::moveit_simple_controller_manager
      pluginlib::pluginlib
      rclcpp_action::rclcpp_action
    )

add_library(moveit_ros_control_interface_trajectory_plugin SHARED
            src/joint_trajectory_controller_plugin.cpp)
set_target_properties(
  moveit_ros_control_interface_trajectory_plugin
  PROPERTIES VERSION "${moveit_ros_control_interface_VERSION}")
target_include_directories(moveit_ros_control_interface_trajectory_plugin
                           PRIVATE include)
ament_target_dependencies(moveit_ros_control_interface_trajectory_plugin
                          ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

add_library(moveit_ros_control_interface_gripper_plugin SHARED
            src/gripper_command_controller_plugin.cpp)
set_target_properties(
  moveit_ros_control_interface_gripper_plugin
  PROPERTIES VERSION "${moveit_ros_control_interface_VERSION}")
target_include_directories(moveit_ros_control_interface_gripper_plugin
                           PRIVATE include)
ament_target_dependencies(moveit_ros_control_interface_gripper_plugin
                          ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

add_library(moveit_ros_control_interface_parallel_gripper_plugin SHARED
            src/parallel_gripper_command_controller_plugin.cpp)
set_target_properties(
  moveit_ros_control_interface_parallel_gripper_plugin
  PROPERTIES VERSION "${moveit_ros_control_interface_VERSION}")
target_include_directories(moveit_ros_control_interface_parallel_gripper_plugin
                           PRIVATE include)
ament_target_dependencies(moveit_ros_control_interface_parallel_gripper_plugin
                          ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

add_library(moveit_ros_control_interface_empty_plugin SHARED
            src/empty_controller_plugin.cpp)
set_target_properties(
  moveit_ros_control_interface_empty_plugin
  PROPERTIES VERSION "${moveit_ros_control_interface_VERSION}")
target_include_directories(moveit_ros_control_interface_empty_plugin
                           PRIVATE include)
ament_target_dependencies(moveit_ros_control_interface_empty_plugin
                          ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

# ##############################################################################
# Install ##
# ##############################################################################

set(TARGET_LIBRARIES
    moveit_ros_control_interface_plugin
    moveit_ros_control_interface_trajectory_plugin
    moveit_ros_control_interface_gripper_plugin
    moveit_ros_control_interface_parallel_gripper_plugin
    moveit_ros_control_interface_empty_plugin)

# Mark executables and/or libraries for installation
install(
  TARGETS ${TARGET_LIBRARIES}
  EXPORT moveit_ros_control_interfaceTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_ros_control_interface)

# Mark cpp header files for installation
install(DIRECTORY include/ DESTINATION include/moveit_ros_control_interface)

pluginlib_export_plugin_description_file(moveit_core moveit_core_plugins.xml)
pluginlib_export_plugin_description_file(
  moveit_ros_control_interface moveit_ros_control_interface_plugins.xml)

ament_export_targets(moveit_ros_control_interfaceTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package(CONFIG_EXTRAS)
