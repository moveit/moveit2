cmake_minimum_required(VERSION 3.22)
project(moveit_ros_benchmarks LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_warehouse REQUIRED)
find_package(moveit_ros_occupancy_map_monitor REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(pluginlib REQUIRED)
find_package(warehouse_ros REQUIRED)
# Finds Boost Components
include(ConfigExtras.cmake)

include_directories(include)

add_library(moveit_ros_benchmarks SHARED src/BenchmarkOptions.cpp
                                         src/BenchmarkExecutor.cpp)
set_target_properties(moveit_ros_benchmarks
                      PROPERTIES VERSION "${moveit_ros_benchmarks_VERSION}")
if(WIN32)
  set(EXTRA_LIB ws2_32.lib)
endif()
target_link_libraries(moveit_ros_benchmarks PUBLIC
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
        moveit_ros_warehouse::moveit_warehouse
        pluginlib::pluginlib
        rclcpp::rclcpp
        tf2_eigen::tf2_eigen
      )

add_executable(moveit_run_benchmark src/RunBenchmark.cpp)

target_link_libraries(moveit_run_benchmark PUBLIC
        moveit_ros_benchmarks
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
        moveit_ros_warehouse::moveit_warehouse
        pluginlib::pluginlib
        rclcpp::rclcpp
        tf2_eigen::tf2_eigen
      )

install(
  TARGETS moveit_ros_benchmarks
  EXPORT moveit_ros_benchmarksTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_ros_benchmarks)

ament_export_targets(moveit_ros_benchmarksTargets HAS_LIBRARY_TARGET)

install(TARGETS moveit_run_benchmark DESTINATION lib/moveit_ros_benchmarks)

install(DIRECTORY include/ DESTINATION include/moveit_ros_benchmarks)

install(PROGRAMS scripts/moveit_benchmark_statistics.py
        DESTINATION lib/moveit_ros_benchmarks)

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
