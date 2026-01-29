cmake_minimum_required(VERSION 3.22)
project(moveit_planners_stomp)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_core REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stomp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rsl REQUIRED)

generate_parameter_library(stomp_moveit_parameters res/stomp_moveit.yaml)

include_directories(include)

# Planner Plugin
add_library(stomp_moveit_plugin SHARED src/stomp_moveit_planner_plugin.cpp
                                       src/stomp_moveit_planning_context.cpp)
target_link_libraries(stomp_moveit_plugin PUBLIC
      ${std_msgs_TARGETS}
      ${visualization_msgs_TARGETS}
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
      tf2_eigen::tf2_eigen
      rsl::rsl
      stomp::stomp
      stomp_moveit_parameters
    )

pluginlib_export_plugin_description_file(moveit_core
                                         stomp_moveit_plugin_description.xml)

install(
  TARGETS stomp_moveit_plugin stomp_moveit_parameters
  EXPORT moveit_planners_stompTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_planners_stomp)

ament_export_targets(moveit_planners_stompTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core stomp generate_parameter_library)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

