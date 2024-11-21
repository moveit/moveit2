cmake_minimum_required(VERSION 3.22)
project(moveit_ros_planning LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(message_filters REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_occupancy_map_monitor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(srdfdom REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
# find_package(moveit_ros_perception REQUIRED)

# Finds Boost Components
include(ConfigExtras.cmake)

set(THIS_PACKAGE_INCLUDE_DIRS
    collision_plugin_loader/include
    constraint_sampler_manager_loader/include
    kinematics_plugin_loader/include
    moveit_cpp/include
    plan_execution/include
    planning_pipeline_interfaces/include
    planning_pipeline/include
    planning_scene_monitor/include
    rdf_loader/include
    robot_model_loader/include
    trajectory_execution_manager/include)

set(THIS_PACKAGE_LIBRARIES
    default_request_adapter_parameters
    default_response_adapter_parameters
    moveit_collision_plugin_loader
    moveit_constraint_sampler_manager_loader
    moveit_cpp
    moveit_kinematics_plugin_loader
    moveit_plan_execution
    moveit_planning_pipeline
    moveit_planning_pipeline_interfaces
    moveit_planning_scene_monitor
    moveit_rdf_loader
    moveit_robot_model_loader
    moveit_trajectory_execution_manager
    planning_pipeline_parameters
    srdf_publisher_node)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    Eigen3
    generate_parameter_library
    message_filters
    moveit_core
    moveit_msgs
    moveit_ros_occupancy_map_monitor
    pluginlib
    rclcpp
    rclcpp_components
    srdfdom
    std_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_msgs
    tf2_ros
    urdf
    # moveit_ros_perception
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

add_subdirectory(collision_plugin_loader)
add_subdirectory(constraint_sampler_manager_loader)
add_subdirectory(kinematics_plugin_loader)
add_subdirectory(moveit_cpp)
add_subdirectory(plan_execution)
add_subdirectory(planning_components_tools)
add_subdirectory(planning_pipeline_interfaces)
add_subdirectory(planning_pipeline)
add_subdirectory(planning_request_adapter_plugins)
add_subdirectory(planning_response_adapter_plugins)
add_subdirectory(planning_scene_monitor)
add_subdirectory(rdf_loader)
add_subdirectory(robot_model_loader)
add_subdirectory(srdf_publisher_node)
add_subdirectory(trajectory_execution_manager)

install(
  TARGETS ${THIS_PACKAGE_LIBRARIES}
  EXPORT moveit_ros_planningTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_ros_planning)

# install plugins as separate export set
install(
  TARGETS moveit_default_planning_response_adapter_plugins
          moveit_default_planning_request_adapter_plugins
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_ros_planning)

ament_export_targets(moveit_ros_planningTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

rclcpp_components_register_node(
  srdf_publisher_node PLUGIN "moveit_ros_planning::SrdfPublisher" EXECUTABLE
  srdf_publisher)

pluginlib_export_plugin_description_file(
  moveit_core "default_request_adapters_plugin_description.xml")
pluginlib_export_plugin_description_file(
  moveit_core "default_response_adapters_plugin_description.xml")
if(BUILD_TESTING)
  pluginlib_export_plugin_description_file(
    moveit_core "planning_pipeline_test_plugins_description.xml")
endif()

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
