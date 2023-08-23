generate_parameter_library(default_plan_request_adapter_parameters res/default_plan_request_adapter_params.yaml)

add_library(moveit_default_planning_request_adapter_plugins SHARED
  src/empty.cpp
  src/fix_start_state_bounds.cpp
  src/fix_start_state_collision.cpp
  src/fix_start_state_path_constraints.cpp
  src/fix_workspace_bounds.cpp
  src/add_ruckig_traj_smoothing.cpp
  src/add_time_optimal_parameterization.cpp
  src/resolve_constraint_frames.cpp
)

target_link_libraries(moveit_default_planning_request_adapter_plugins default_plan_request_adapter_parameters)

set_target_properties(moveit_default_planning_request_adapter_plugins PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_default_planning_request_adapter_plugins
  Boost
  moveit_core
  rclcpp
  pluginlib
)
