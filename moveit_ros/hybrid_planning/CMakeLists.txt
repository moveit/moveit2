cmake_minimum_required(VERSION 3.22)
project(moveit_hybrid_planning LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(LIBRARIES
    # Components
    moveit_global_planner_component
    moveit_hybrid_planning_manager
    moveit_local_planner_component
    # Plugins
    forward_trajectory_plugin
    motion_planning_pipeline_plugin
    replan_invalidated_trajectory_plugin
    simple_sampler_plugin
    single_plan_execution_plugin
    # Parameters
    local_planner_parameters
    hp_manager_parameters)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    rclcpp
    rclcpp_action
    rclcpp_components
    std_msgs
    std_srvs
    tf2_ros
    trajectory_msgs)

set(THIS_PACKAGE_INCLUDE_DIRS
    global_planner/global_planner_component/include/
    global_planner/global_planner_plugins/include/
    hybrid_planning_manager/hybrid_planning_manager_component/include/
    hybrid_planning_manager/planner_logic_plugins/include/
    local_planner/local_planner_component/include/
    local_planner/trajectory_operator_plugins/include/
    local_planner/local_constraint_solver_plugins/include/)
include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_subdirectory(hybrid_planning_manager)
add_subdirectory(global_planner)
add_subdirectory(local_planner)
add_subdirectory(test)

rclcpp_components_register_nodes(
  moveit_hybrid_planning_manager
  "moveit::hybrid_planning::HybridPlanningManager")
rclcpp_components_register_nodes(
  moveit_global_planner_component
  "moveit::hybrid_planning::GlobalPlannerComponent")
rclcpp_components_register_nodes(
  moveit_local_planner_component
  "moveit::hybrid_planning::LocalPlannerComponent")

install(
  TARGETS ${LIBRARIES}
  EXPORT moveit_hybrid_planningTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_hybrid_planning)

install(
  TARGETS cancel_action
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/moveit_hybrid_planning
  INCLUDES
  DESTINATION include/moveit_hybrid_planning)

install(DIRECTORY ${THIS_PACKAGE_INCLUDE_DIRS}
        DESTINATION include/moveit_hybrid_planning)

install(DIRECTORY test/launch DESTINATION share/moveit_hybrid_planning)
install(DIRECTORY test/config DESTINATION share/moveit_hybrid_planning)

pluginlib_export_plugin_description_file(moveit_hybrid_planning
                                         single_plan_execution_plugin.xml)
pluginlib_export_plugin_description_file(moveit_hybrid_planning
                                         moveit_planning_pipeline_plugin.xml)
pluginlib_export_plugin_description_file(moveit_hybrid_planning
                                         simple_sampler_plugin.xml)
pluginlib_export_plugin_description_file(
  moveit_hybrid_planning replan_invalidated_trajectory_plugin.xml)
pluginlib_export_plugin_description_file(moveit_hybrid_planning
                                         forward_trajectory_plugin.xml)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(moveit_hybrid_planningTargets HAS_LIBRARY_TARGET)

ament_package()
