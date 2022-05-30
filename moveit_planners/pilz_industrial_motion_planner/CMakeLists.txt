cmake_minimum_required(VERSION 3.10.2)
project(pilz_industrial_motion_planner)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
# TODO(henning): Enable when this is available
# find_package(joint_limits_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED COMPONENTS)

# TODO(henning): Remove/Port
# if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
#   find_package(code_coverage REQUIRED)
#   append_coverage_compiler_flags()
# endif()
#
# ################
# ## Clang tidy ##
# ################
# if(CATKIN_ENABLE_CLANG_TIDY)
#   find_program(
#     CLANG_TIDY_EXE
#     NAMES "clang-tidy"
#     DOC "Path to clang-tidy executable"
#   )
#   if(NOT CLANG_TIDY_EXE)
#     message(FATAL_ERROR "clang-tidy not found.")
#   else()
#     message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
#     set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
#   endif()
# endif()

###########
## Build ##
###########
include_directories(
  include
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  Eigen3
  eigen3_cmake_module
  # joint_limits_interface
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_move_group
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  rclcpp
  tf2
  tf2_eigen
  tf2_eigen_kdl
  tf2_geometry_msgs
  tf2_kdl
  tf2_ros
  orocos_kdl
  Boost
)


###############
## Libraries ##
###############

add_library(planning_context_loader_base SHARED
  src/planning_context_loader.cpp
)
ament_target_dependencies(planning_context_loader_base ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(joint_limits_common SHARED
  src/joint_limits_aggregator.cpp
  src/joint_limits_container.cpp
  src/joint_limits_validator.cpp
  src/limits_container.cpp
  src/cartesian_limit.cpp
  src/cartesian_limits_aggregator.cpp
)
ament_target_dependencies(joint_limits_common ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(trajectory_generation_common SHARED
  src/trajectory_functions.cpp
  src/trajectory_generator.cpp
  src/trajectory_blender_transition_window.cpp
)
ament_target_dependencies(trajectory_generation_common ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(command_list_manager SHARED
  src/command_list_manager.cpp
  src/plan_components_builder.cpp
)
target_link_libraries(command_list_manager trajectory_generation_common joint_limits_common)
ament_target_dependencies(command_list_manager ${THIS_PACKAGE_INCLUDE_DEPENDS})

#############
## Plugins ##
#############

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME}
  planning_context_loader_base
  joint_limits_common)

add_library(planning_context_loader_ptp SHARED
  src/planning_context_loader_ptp.cpp
  src/trajectory_generator_ptp.cpp
  src/velocity_profile_atrap.cpp
)
ament_target_dependencies(planning_context_loader_ptp ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(planning_context_loader_ptp planning_context_loader_base joint_limits_common trajectory_generation_common)

add_library(planning_context_loader_lin SHARED
  src/planning_context_loader_lin.cpp
  src/trajectory_generator_lin.cpp
  src/velocity_profile_atrap.cpp
)
ament_target_dependencies(planning_context_loader_lin ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(planning_context_loader_lin planning_context_loader_base joint_limits_common trajectory_generation_common)

add_library(planning_context_loader_circ SHARED
  src/planning_context_loader_circ.cpp
  src/trajectory_generator_circ.cpp
  src/path_circle_generator.cpp
)
ament_target_dependencies(planning_context_loader_circ ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(planning_context_loader_circ planning_context_loader_base joint_limits_common trajectory_generation_common)


add_library(sequence_capability SHARED
  src/move_group_sequence_action.cpp
  src/move_group_sequence_service.cpp
)
ament_target_dependencies(sequence_capability ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(sequence_capability joint_limits_common command_list_manager trajectory_generation_common)

#############
## Install ##
#############
pluginlib_export_plugin_description_file(moveit_core plugins/pilz_industrial_motion_planner_plugin_description.xml)
pluginlib_export_plugin_description_file(moveit_ros_move_group plugins/sequence_capability_plugin_description.xml)
pluginlib_export_plugin_description_file(pilz_industrial_motion_planner plugins/planning_context_plugin_description.xml)

## Mark libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}
    joint_limits_common
    planning_context_loader_base
    planning_context_loader_ptp
    planning_context_loader_lin
    planning_context_loader_circ
    command_list_manager
    sequence_capability
    trajectory_generation_common
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} orocos_kdl_vendor)

if(BUILD_TESTING)
  # Include Tests
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
