cmake_minimum_required(VERSION 3.22)
project(pilz_industrial_motion_planner_testutils LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(include)

# Declare a C++ library
add_library(
  pilz_industrial_motion_planner_testutils SHARED
  src/cartesianconfiguration.cpp src/jointconfiguration.cpp
  src/robotconfiguration.cpp src/sequence.cpp src/xml_testdata_loader.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(pilz_industrial_motion_planner_testutils PUBLIC
      ${moveit_msgs_TARGETS}
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
      rclcpp::rclcpp
      tf2_eigen::tf2_eigen
    )


# ##############################################################################
# Install ##
# ##############################################################################

install(
  TARGETS pilz_industrial_motion_planner_testutils
  EXPORT pilz_industrial_motion_planner_testutilsTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/pilz_industrial_motion_planner_testutils)

install(DIRECTORY include/
        DESTINATION include/pilz_industrial_motion_planner_testutils)

ament_export_targets(pilz_industrial_motion_planner_testutilsTargets
                     HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp moveit_core moveit_msgs tf2_eigen)
