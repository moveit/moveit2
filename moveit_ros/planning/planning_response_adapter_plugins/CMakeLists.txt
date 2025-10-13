generate_parameter_library(default_response_adapter_parameters
                           res/default_response_adapter_params.yaml)

add_library(
  moveit_default_planning_response_adapter_plugins SHARED
  src/add_ruckig_traj_smoothing.cpp src/add_time_optimal_parameterization.cpp
  src/display_motion_path.cpp src/validate_path.cpp)

target_link_libraries(
  moveit_default_planning_response_adapter_plugins
  PUBLIC default_response_adapter_parameters
         moveit_core::moveit_collision_detection pluginlib::pluginlib
         rclcpp::rclcpp)

set_target_properties(moveit_default_planning_response_adapter_plugins
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
