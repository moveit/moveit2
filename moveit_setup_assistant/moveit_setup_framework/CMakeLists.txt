cmake_minimum_required(VERSION 3.22)
project(moveit_setup_framework LANGUAGES CXX)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
if(Qt${QT_VERSION_MAJOR} STREQUAL "Qt6")
  find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Core Core5Compat)
elseif()
  find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Core)
endif()

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_visualization REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt_wrap_cpp(
  MOC_FILES
  include/moveit_setup_framework/qt/helper_widgets.hpp
  include/moveit_setup_framework/qt/setup_step_widget.hpp
  include/moveit_setup_framework/qt/rviz_panel.hpp
  include/moveit_setup_framework/qt/double_list_widget.hpp)

add_library(
  moveit_setup_framework
  src/helper_widgets.cpp
  src/double_list_widget.cpp
  src/urdf_config.cpp
  src/package_settings_config.cpp
  src/srdf_config.cpp
  src/rviz_panel.cpp
  src/data_warehouse.cpp
  src/templates.cpp
  src/utilities.cpp
  src/xml_syntax_highlighter.cpp
  ${MOC_FILES})
target_include_directories(
  moveit_setup_framework
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_setup_framework>)

if(Qt${QT_VERSION_MAJOR} STREQUAL "Qt6")
  target_link_libraries(moveit_setup_framework
    PUBLIC
      ament_index_cpp::ament_index_cpp
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
      moveit_ros_planning::default_request_adapter_parameters
      moveit_ros_planning::default_response_adapter_parameters
      moveit_ros_planning::kinematics_parameters
      moveit_ros_planning::moveit_collision_plugin_loader
      moveit_ros_planning::moveit_constraint_sampler_manager_loader
      moveit_ros_planning::moveit_cpp
      moveit_ros_planning::moveit_kinematics_plugin_loader
      moveit_ros_planning::moveit_plan_execution
      moveit_ros_planning::moveit_planning_pipeline
      moveit_ros_planning::moveit_planning_pipeline_interfaces
      moveit_ros_planning::moveit_planning_scene_monitor
      moveit_ros_planning::moveit_rdf_loader
      moveit_ros_planning::moveit_robot_model_loader
      moveit_ros_planning::moveit_trajectory_execution_manager
      moveit_ros_planning::planning_pipeline_parameters
      moveit_ros_planning::srdf_publisher_node
      moveit_ros_visualization::moveit_motion_planning_rviz_plugin
      moveit_ros_visualization::moveit_motion_planning_rviz_plugin_core
      moveit_ros_visualization::moveit_planning_scene_rviz_plugin
      moveit_ros_visualization::moveit_planning_scene_rviz_plugin_core
      moveit_ros_visualization::moveit_robot_state_rviz_plugin
      moveit_ros_visualization::moveit_robot_state_rviz_plugin_core
      moveit_ros_visualization::moveit_rviz_plugin_render_tools
      moveit_ros_visualization::moveit_trajectory_rviz_plugin
      moveit_ros_visualization::moveit_trajectory_rviz_plugin_core
      pluginlib::pluginlib
      rclcpp::rclcpp
      rviz_common::rviz_common
      rviz_rendering::rviz_rendering
      srdfdom::srdfdom
      urdf::urdf
      fmt::fmt
      Qt${QT_VERSION_MAJOR}::Core
      Qt${QT_VERSION_MAJOR}::Core5Compat)
elseif()
  target_link_libraries(moveit_setup_framework
    PUBLIC
      ament_index_cpp::ament_index_cpp
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
      moveit_ros_planning::default_request_adapter_parameters
      moveit_ros_planning::default_response_adapter_parameters
      moveit_ros_planning::kinematics_parameters
      moveit_ros_planning::moveit_collision_plugin_loader
      moveit_ros_planning::moveit_constraint_sampler_manager_loader
      moveit_ros_planning::moveit_cpp
      moveit_ros_planning::moveit_kinematics_plugin_loader
      moveit_ros_planning::moveit_plan_execution
      moveit_ros_planning::moveit_planning_pipeline
      moveit_ros_planning::moveit_planning_pipeline_interfaces
      moveit_ros_planning::moveit_planning_scene_monitor
      moveit_ros_planning::moveit_rdf_loader
      moveit_ros_planning::moveit_robot_model_loader
      moveit_ros_planning::moveit_trajectory_execution_manager
      moveit_ros_planning::planning_pipeline_parameters
      moveit_ros_planning::srdf_publisher_node
      moveit_ros_visualization::moveit_motion_planning_rviz_plugin
      moveit_ros_visualization::moveit_motion_planning_rviz_plugin_core
      moveit_ros_visualization::moveit_planning_scene_rviz_plugin
      moveit_ros_visualization::moveit_planning_scene_rviz_plugin_core
      moveit_ros_visualization::moveit_robot_state_rviz_plugin
      moveit_ros_visualization::moveit_robot_state_rviz_plugin_core
      moveit_ros_visualization::moveit_rviz_plugin_render_tools
      moveit_ros_visualization::moveit_trajectory_rviz_plugin
      moveit_ros_visualization::moveit_trajectory_rviz_plugin_core
      pluginlib::pluginlib
      rclcpp::rclcpp
      rviz_common::rviz_common
      rviz_rendering::rviz_rendering
      srdfdom::srdfdom
      urdf::urdf
      fmt::fmt
      Qt${QT_VERSION_MAJOR}::Core)
endif()



install(DIRECTORY include/ DESTINATION include/moveit_setup_framework)
install(FILES moveit_setup_framework_plugins.xml
        DESTINATION share/moveit_setup_framework)

install(DIRECTORY templates DESTINATION share/moveit_setup_framework)

ament_export_targets(moveit_setup_frameworkTargets HAS_LIBRARY_TARGET)
install(
  TARGETS moveit_setup_framework
  EXPORT moveit_setup_frameworkTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_setup_framework)

ament_export_dependencies(fmt)
ament_export_dependencies(rclcpp)
ament_export_dependencies(moveit_common)
ament_export_dependencies(moveit_core)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(moveit_ros_visualization)
ament_export_dependencies(rviz_common)
ament_export_dependencies(rviz_rendering)
pluginlib_export_plugin_description_file(moveit_setup_framework
                                         moveit_setup_framework_plugins.xml)

ament_package()
