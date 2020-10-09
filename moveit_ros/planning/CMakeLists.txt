cmake_minimum_required(VERSION 3.10.2)
project(moveit_ros_planning)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(Boost REQUIRED system filesystem date_time program_options thread chrono)
find_package(ament_cmake REQUIRED)
find_package(moveit_msgs REQUIRED)
# find_package(moveit_ros_perception REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit_ros_occupancy_map_monitor REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
    rdf_loader/include
    kinematics_plugin_loader/include
    robot_model_loader/include
    constraint_sampler_manager_loader/include
    planning_pipeline/include
    planning_scene_monitor/include
    trajectory_execution_manager/include
    plan_execution/include
    collision_plugin_loader/include
)

set(libraries
  moveit_rdf_loader
  moveit_kinematics_plugin_loader
  moveit_robot_model_loader
  moveit_constraint_sampler_manager_loader
  moveit_planning_pipeline
  moveit_trajectory_execution_manager
  moveit_plan_execution
  moveit_planning_scene_monitor
  moveit_collision_plugin_loader
  moveit_default_planning_request_adapter_plugins
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  Boost
  pluginlib
  rclcpp
  message_filters
  srdfdom
  urdf
  tf2
  tf2_eigen
  tf2_ros
  Eigen3
  moveit_core
  # moveit_ros_perception
  moveit_ros_occupancy_map_monitor
  moveit_msgs
  tf2_msgs
  tf2_geometry_msgs
  Boost
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM
                    ${EIGEN3_INCLUDE_DIRS})

add_subdirectory(rdf_loader)
add_subdirectory(collision_plugin_loader)
add_subdirectory(kinematics_plugin_loader)
add_subdirectory(robot_model_loader)
add_subdirectory(constraint_sampler_manager_loader)
add_subdirectory(planning_pipeline)
add_subdirectory(planning_request_adapter_plugins)
add_subdirectory(planning_scene_monitor)
add_subdirectory(planning_components_tools)
add_subdirectory(trajectory_execution_manager)
add_subdirectory(plan_execution)

ament_export_include_directories(include)
ament_export_libraries(${libraries})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(moveit_core "planning_request_adapters_plugin_description.xml")

ament_package()
