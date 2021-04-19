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


## Declare a C++ library
add_library(${PROJECT_NAME}_lib SHARED
  src/${PROJECT_NAME}.cpp
  src/planning_context_loader.cpp
  src/joint_limits_validator.cpp
  src/joint_limits_aggregator.cpp
  src/joint_limits_container.cpp
  src/cartesian_limits_aggregator.cpp
  src/cartesian_limit.cpp
  src/limits_container.cpp
  src/trajectory_functions.cpp
  src/plan_components_builder.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib ${THIS_PACKAGE_INCLUDE_DEPENDS})

#############
## Plugins ##
#############
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/planning_context_loader.cpp
  src/joint_limits_aggregator.cpp
  src/joint_limits_container.cpp
  src/limits_container.cpp
  src/cartesian_limit.cpp
  src/cartesian_limits_aggregator.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(planning_context_loader_ptp
  src/planning_context_loader_ptp.cpp
  src/planning_context_loader.cpp
  src/trajectory_functions.cpp
  src/trajectory_generator.cpp
  src/trajectory_generator_ptp.cpp
  src/velocity_profile_atrap.cpp
  src/joint_limits_container.cpp
)
ament_target_dependencies(planning_context_loader_ptp ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(planning_context_loader_lin
  src/planning_context_loader_lin.cpp
  src/planning_context_loader.cpp
  src/trajectory_functions.cpp
  src/trajectory_generator.cpp
  src/trajectory_generator_lin.cpp
  src/velocity_profile_atrap.cpp
)
ament_target_dependencies(planning_context_loader_lin ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(planning_context_loader_circ
  src/planning_context_loader_circ.cpp
  src/planning_context_loader.cpp
  src/trajectory_functions.cpp
  src/trajectory_generator.cpp
  src/trajectory_generator_circ.cpp
  src/path_circle_generator.cpp
)
ament_target_dependencies(planning_context_loader_circ ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(command_list_manager
  src/command_list_manager.cpp
  src/plan_components_builder.cpp
)
ament_target_dependencies(command_list_manager ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(sequence_capability
  src/move_group_sequence_action.cpp
  src/move_group_sequence_service.cpp
  src/plan_components_builder.cpp
  src/command_list_manager.cpp
  src/trajectory_blender_transition_window.cpp
  src/joint_limits_aggregator.cpp  # do we need joint limits and cartesian_limit here?
  src/joint_limits_container.cpp
  src/limits_container.cpp
  src/cartesian_limit.cpp
  src/cartesian_limits_aggregator.cpp
)
ament_target_dependencies(sequence_capability ${THIS_PACKAGE_INCLUDE_DEPENDS})

#############
## Install ##
#############
install(FILES
  plugins/sequence_capability_plugin_description.xml
  plugins/${PROJECT_NAME}_plugin_description.xml
  plugins/planning_context_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

## Mark libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_lib
    planning_context_loader_ptp
    planning_context_loader_lin
    planning_context_loader_circ
    command_list_manager
    sequence_capability
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

#############
## Testing ##
#############
# TODO(henning): Enable tests
# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   find_package(${PROJECT_NAME}_testutils REQUIRED)
#
#   include_directories(
#     ${${PROJECT_NAME}_testutils_INCLUDE_DIRS}
#   )
#
#   if(ENABLE_COVERAGE_TESTING)
#     include(CodeCoverage)
#     append_coverage_compiler_flags()
#   endif()
#
#   #########################
#   ####Integration Tests####
#   #########################
#   set(${PROJECT_NAME}_INTEGRATIONTEST_LIBRARIES
#     sequence_capability
#     ${PROJECT_NAME}
#     planning_context_loader_ptp
#     planning_context_loader_lin
#     planning_context_loader_circ
#   )
#
#   # Planning Integration tests
#   add_rostest_gmock(integrationtest_sequence_action_preemption
#     test/integrationtest_sequence_action_preemption.test
#     test/integrationtest_sequence_action_preemption.cpp
#   )
#   target_link_libraries(integrationtest_sequence_action_preemption
#     ${catkin_LIBRARIES}
#     ${${PROJECT_NAME}_testutils_LIBRARIES}
#     ${${PROJECT_NAME}_INTEGRATIONTEST_LIBRARIES}
#   )
#
#   add_rostest_gmock(integrationtest_sequence_action
#     test/integrationtest_sequence_action.test
#     test/integrationtest_sequence_action.cpp
#   )
#   target_link_libraries(integrationtest_sequence_action
#     ${catkin_LIBRARIES}
#     ${${PROJECT_NAME}_testutils_LIBRARIES}
#     ${${PROJECT_NAME}_INTEGRATIONTEST_LIBRARIES}
#   )
#
#   add_rostest(test/integrationtest_sequence_action_capability_with_gripper.test
#     DEPENDENCIES integrationtest_sequence_action
#   )
#
#   add_rostest(test/integrationtest_sequence_action_capability_frankaemika_panda.test
#     DEPENDENCIES integrationtest_sequence_action
#   )
#
#   add_rostest_gtest(integrationtest_sequence_service_capability
#     test/integrationtest_sequence_service_capability.test
#     test/integrationtest_sequence_service_capability.cpp
#   )
#   target_link_libraries(integrationtest_sequence_service_capability
#     ${catkin_LIBRARIES}
#     ${${PROJECT_NAME}_testutils_LIBRARIES}
#     ${${PROJECT_NAME}_INTEGRATIONTEST_LIBRARIES}
#   )
#
#   add_rostest(test/integrationtest_sequence_service_capability_frankaemika_panda.test
#     DEPENDENCIES integrationtest_sequence_service_capability
#   )
#
#   add_rostest(test/integrationtest_sequence_service_capability_with_gripper.test
#     DEPENDENCIES integrationtest_sequence_service_capability
#   )
#
#   # Planning Integration tests
#   add_rostest_gtest(integrationtest_command_planning
#     test/integrationtest_command_planning.test
#     test/integrationtest_command_planning.cpp
#     test/test_utils.cpp
#   )
#   target_link_libraries(integrationtest_command_planning
#     ${catkin_LIBRARIES}
#     ${${PROJECT_NAME}_testutils_LIBRARIES}
#     ${${PROJECT_NAME}_INTEGRATIONTEST_LIBRARIES}
#   )
#
#   add_rostest(test/integrationtest_command_planning_with_gripper.test
#     DEPENDENCIES integrationtest_command_planning
#   )
#
#   add_rostest(test/integrationtest_command_planning_frankaemika_panda.test
#     DEPENDENCIES integrationtest_command_planning
#   )
#
#   # Blending Integration tests
#   add_rostest_gtest(integrationtest_command_list_manager
#     test/integrationtest_command_list_manager.test
#     test/integrationtest_command_list_manager.cpp
#     test/test_utils.cpp
#   )
#   target_link_libraries(integrationtest_command_list_manager
#     ${catkin_LIBRARIES}
#     ${${PROJECT_NAME}_testutils_LIBRARIES}
#     ${${PROJECT_NAME}_INTEGRATIONTEST_LIBRARIES}
#   )
#
#   add_rostest_gtest(integrationtest_get_solver_tip_frame
#     test/integrationtest_get_solver_tip_frame.test
#     test/integrationtest_get_solver_tip_frame.cpp
#   )
#   target_link_libraries(integrationtest_get_solver_tip_frame
#     ${catkin_LIBRARIES}
#     ${${PROJECT_NAME}_INTEGRATIONTEST_LIBRARIES}
#   )
#
#   add_rostest_gtest(integrationtest_plan_components_builder
#     test/integrationtest_plan_components_builder.test
#     test/integrationtest_plan_components_builder.cpp
#   )
#   target_link_libraries(integrationtest_plan_components_builder
#     ${catkin_LIBRARIES}
#     ${${PROJECT_NAME}_INTEGRATIONTEST_LIBRARIES}
#   )
#
#   ##################
#   ####Unit Tests####
#   ##################
#   add_library(${PROJECT_NAME}_testhelpers
#     test/test_utils.cpp
#     src/trajectory_generator.cpp
#     src/trajectory_generator_circ.cpp
#     src/trajectory_generator_lin.cpp
#     src/trajectory_generator_ptp.cpp
#     src/path_circle_generator.cpp
#     src/velocity_profile_atrap.cpp
#   )
#
#   target_link_libraries(${PROJECT_NAME}_testhelpers ${PROJECT_NAME}_lib)
#
#   ## Add gtest based cpp test target and link libraries
#   catkin_add_gtest(unittest_${PROJECT_NAME}_direct
#     test/unittest_${PROJECT_NAME}_direct.cpp
#     src/planning_context_loader_ptp.cpp
#   )
#   target_link_libraries(unittest_${PROJECT_NAME}_direct
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
#   ## Add gtest based cpp test target and link libraries
#   catkin_add_gtest(unittest_velocity_profile_atrap
#     test/unittest_velocity_profile_atrap.cpp
#     src/velocity_profile_atrap.cpp
#   )
#   target_link_libraries(unittest_velocity_profile_atrap
#     ${catkin_LIBRARIES}
#   )
#
#   catkin_add_gtest(unittest_trajectory_generator
#     test/unittest_trajectory_generator.cpp
#     src/trajectory_generator.cpp
#   )
#   target_link_libraries(unittest_trajectory_generator
#     ${catkin_LIBRARIES}
#     ${${PROJECT_NAME}_testutils_LIBRARIES}
#     ${PROJECT_NAME}_lib
#   )
#
#   # Trajectory Generator Unit Test
#   add_rostest_gtest(unittest_trajectory_functions
#     test/unittest_trajectory_functions.test
#     test/unittest_trajectory_functions.cpp
#   )
#   target_link_libraries(unittest_trajectory_functions
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
#   # unittest for trajectory blender transition window
#   add_rostest_gtest(unittest_trajectory_blender_transition_window
#     test/unittest_trajectory_blender_transition_window.test
#     test/unittest_trajectory_blender_transition_window.cpp
#     src/trajectory_blender_transition_window.cpp
#   )
#   target_link_libraries(unittest_trajectory_blender_transition_window
#     ${catkin_LIBRARIES}
#     ${${PROJECT_NAME}_testutils_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
#   # trajectory generator Unit Test
#   add_rostest_gtest(unittest_trajectory_generator_common
#     test/unittest_trajectory_generator_common.test
#     test/unittest_trajectory_generator_common.cpp
#   )
#   target_link_libraries(unittest_trajectory_generator_common
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
#   # trajectory generator circ Unit Test
#   add_rostest_gtest(unittest_trajectory_generator_circ
#     test/unittest_trajectory_generator_circ.test
#     test/unittest_trajectory_generator_circ.cpp
#   )
#   target_link_libraries(unittest_trajectory_generator_circ
#     ${catkin_LIBRARIES}
#     ${${PROJECT_NAME}_testutils_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
#   # trajectory generator lin Unit Test
#   add_rostest_gtest(unittest_trajectory_generator_lin
#     test/unittest_trajectory_generator_lin.test
#     test/unittest_trajectory_generator_lin.cpp
#   )
#   target_link_libraries(unittest_trajectory_generator_lin
#     ${catkin_LIBRARIES}
#     ${${PROJECT_NAME}_testutils_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
#   # trajectory generator ptp Unit Test
#   add_rostest_gtest(unittest_trajectory_generator_ptp
#     test/unittest_trajectory_generator_ptp.test
#     test/unittest_trajectory_generator_ptp.cpp
#   )
#   target_link_libraries(unittest_trajectory_generator_ptp
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
#   # Command Planner Unit Test
#   add_rostest_gtest(unittest_${PROJECT_NAME}
#     test/unittest_${PROJECT_NAME}.test
#     test/unittest_${PROJECT_NAME}.cpp
#   )
#   target_link_libraries(unittest_${PROJECT_NAME}
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_lib
#   )
#
#   # JointLimits Unit Test
#   add_rostest_gtest(unittest_joint_limit
#     test/unittest_joint_limit.test
#     test/unittest_joint_limit.cpp
#   )
#   target_link_libraries(unittest_joint_limit
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_lib
#   )
#
#   # JointLimitsAggregator Unit Test
#   add_rostest_gtest(unittest_joint_limits_aggregator
#     test/unittest_joint_limits_aggregator.test
#     test/unittest_joint_limits_aggregator.cpp
#   )
#   target_link_libraries(unittest_joint_limits_aggregator
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_lib
#   )
#
#   # JointLimitsContainer Unit Test
#   catkin_add_gtest(unittest_joint_limits_container
#     test/unittest_joint_limits_container.cpp
#   )
#   target_link_libraries(unittest_joint_limits_container
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_lib
#   )
#
#   # JointLimitsValidator Unit Test
#   catkin_add_gtest(unittest_joint_limits_validator
#     test/unittest_joint_limits_validator.cpp
#   )
#   target_link_libraries(unittest_joint_limits_validator
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_lib
#   )
#
#   # Cartesian Limits Aggregator Unit Test
#   add_rostest_gtest(unittest_cartesian_limits_aggregator
#     test/unittest_cartesian_limits_aggregator.test
#     test/unittest_cartesian_limits_aggregator.cpp
#   )
#   target_link_libraries(unittest_cartesian_limits_aggregator
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_lib
#   )
#
#   # PlanningContextLoaderPTP Unit Test
#   add_rostest_gtest(unittest_planning_context_loaders
#     test/unittest_planning_context_loaders.test
#     test/unittest_planning_context_loaders.cpp
#   )
#   target_link_libraries(unittest_planning_context_loaders
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
#   # PlanningContext Unit Test (Typed test)
#   add_rostest_gtest(unittest_planning_context
#     test/unittest_planning_context.test
#     test/unittest_planning_context.cpp
#     src/planning_context_loader_circ.cpp
#     src/planning_context_loader_lin.cpp
#     src/planning_context_loader_ptp.cpp
#   )
#   target_link_libraries(unittest_planning_context
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
#   # GetTipFrames Unit Test
#   catkin_add_gmock(unittest_get_solver_tip_frame
#     test/unittest_get_solver_tip_frame.cpp
#   )
#   target_link_libraries(unittest_get_solver_tip_frame
#     ${catkin_LIBRARIES}
#   )
#
#   # to run: catkin_make -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE_TESTING=ON package_name_coverage
#   if(ENABLE_COVERAGE_TESTING)
#     set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
#     add_code_coverage(
#       NAME ${PROJECT_NAME}_coverage
#       # specifying dependencies in a reliable way is on open issue
#       # see https://github.com/mikeferguson/code_coverage/pull/14
#       #DEPENDENCIES tests
#     )
#   endif()
# endif()
