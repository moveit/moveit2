generate_parameter_library(default_request_adapter_parameters
                           res/default_request_adapter_params.yaml)

add_library(
  moveit_default_planning_request_adapter_plugins SHARED
  src/check_for_stacked_constraints.cpp src/check_start_state_bounds.cpp
  src/check_start_state_collision.cpp src/validate_workspace_bounds.cpp
  src/resolve_constraint_frames.cpp)

      target_link_libraries(moveit_default_planning_request_adapter_plugins PUBLIC
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
        pluginlib::pluginlib
        rclcpp::rclcpp
        default_request_adapter_parameters)

set_target_properties(moveit_default_planning_request_adapter_plugins
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_check_start_state_bounds
                  test/test_check_start_state_bounds.cpp)
  target_link_libraries(test_check_start_state_bounds moveit_planning_pipeline
                        moveit_default_planning_request_adapter_plugins)

endif()
