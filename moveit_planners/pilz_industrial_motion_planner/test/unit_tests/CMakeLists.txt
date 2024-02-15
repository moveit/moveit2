set(UNIT_TEST_SOURCEFILES
    unittest_pilz_industrial_motion_planner_direct
    unittest_velocity_profile_atrap
    unittest_trajectory_generator
    unittest_trajectory_functions
    unittest_trajectory_blender_transition_window
    unittest_trajectory_generator_common
    unittest_trajectory_generator_circ
    unittest_trajectory_generator_lin
    unittest_trajectory_generator_ptp
    unittest_pilz_industrial_motion_planner
    unittest_joint_limit
    unittest_joint_limits_aggregator
    unittest_joint_limits_container
    unittest_joint_limits_validator
    unittest_planning_context_loaders
    unittest_planning_context
    unittest_get_solver_tip_frame)

# Direct Command Planner Unit Test
ament_add_gtest(unittest_pilz_industrial_motion_planner_direct
                src/unittest_pilz_industrial_motion_planner_direct.cpp)
target_link_libraries(unittest_pilz_industrial_motion_planner_direct
                      ${PROJECT_NAME} planning_context_loader_ptp)

# Asymmetric Trapezoidal Velocity Profile Unit Test
ament_add_gtest(
  unittest_velocity_profile_atrap src/unittest_velocity_profile_atrap.cpp
  ${CMAKE_SOURCE_DIR}/src/velocity_profile_atrap.cpp)
target_link_libraries(unittest_velocity_profile_atrap ${PROJECT_NAME})

# Trajectory Generator Unit Test
ament_add_gtest(
  unittest_trajectory_generator src/unittest_trajectory_generator.cpp
  ${CMAKE_SOURCE_DIR}/src/trajectory_generator.cpp)
target_link_libraries(unittest_trajectory_generator ${PROJECT_NAME})

# Trajectory Functions Unit Test
ament_add_gtest_executable(unittest_trajectory_functions
                           src/unittest_trajectory_functions.cpp)
ament_target_dependencies(unittest_trajectory_functions Boost)
target_link_libraries(unittest_trajectory_functions ${PROJECT_NAME}_test_utils
                      trajectory_generation_common joint_limits_common)
add_ros_test(launch/unittest_trajectory_functions.test.py ARGS
             "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

# unittest for trajectory blender transition window
ament_add_gtest_executable(
  unittest_trajectory_blender_transition_window
  src/unittest_trajectory_blender_transition_window.cpp
  ${CMAKE_SOURCE_DIR}/src/trajectory_blender_transition_window.cpp)
ament_target_dependencies(unittest_trajectory_blender_transition_window
                          pilz_industrial_motion_planner_testutils)
target_link_libraries(
  unittest_trajectory_blender_transition_window ${PROJECT_NAME}_test_utils
  trajectory_generation_common planning_context_loader_lin)
add_ros_test(launch/unittest_trajectory_blender_transition_window.test.py ARGS
             "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

# trajectory generator Unit Test
ament_add_gtest_executable(unittest_trajectory_generator_common
                           src/unittest_trajectory_generator_common.cpp)
target_link_libraries(
  unittest_trajectory_generator_common ${PROJECT_NAME}_test_utils
  trajectory_generation_common planning_context_loader_lin
  planning_context_loader_ptp planning_context_loader_circ)
add_ros_test(launch/unittest_trajectory_generator_common.test.py ARGS
             "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

# trajectory generator circ Unit Test
ament_add_gtest_executable(unittest_trajectory_generator_circ
                           src/unittest_trajectory_generator_circ.cpp)
ament_target_dependencies(unittest_trajectory_generator_circ
                          pilz_industrial_motion_planner_testutils)
target_link_libraries(unittest_trajectory_generator_circ
                      ${PROJECT_NAME}_test_utils planning_context_loader_circ)
add_ros_test(launch/unittest_trajectory_generator_circ.test.py ARGS
             "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

# trajectory generator lin Unit Test
ament_add_gtest_executable(unittest_trajectory_generator_lin
                           src/unittest_trajectory_generator_lin.cpp)
ament_target_dependencies(unittest_trajectory_generator_lin
                          pilz_industrial_motion_planner_testutils)
target_link_libraries(unittest_trajectory_generator_lin
                      ${PROJECT_NAME}_test_utils planning_context_loader_lin)
add_ros_test(launch/unittest_trajectory_generator_lin.test.py ARGS
             "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

# trajectory generator ptp Unit Test
ament_add_gtest_executable(unittest_trajectory_generator_ptp
                           src/unittest_trajectory_generator_ptp.cpp)
target_link_libraries(
  unittest_trajectory_generator_ptp ${PROJECT_NAME}_test_utils
  trajectory_generation_common planning_context_loader_ptp)
add_ros_test(launch/unittest_trajectory_generator_ptp.test.py ARGS
             "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

# Command Planner Unit Test
ament_add_gtest_executable(unittest_pilz_industrial_motion_planner
                           src/unittest_pilz_industrial_motion_planner.cpp)
target_link_libraries(unittest_pilz_industrial_motion_planner ${PROJECT_NAME})
add_ros_test(launch/unittest_pilz_industrial_motion_planner.test.py ARGS
             "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

# JointLimits Unit Test
ament_add_gtest_executable(unittest_joint_limit src/unittest_joint_limit.cpp)
target_link_libraries(unittest_joint_limit joint_limits_common)
add_ros_test(launch/unittest_joint_limit.test.py ARGS
             "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

# JointLimitsAggregator Unit Test
ament_add_gtest_executable(unittest_joint_limits_aggregator
                           src/unittest_joint_limits_aggregator.cpp)
target_link_libraries(unittest_joint_limits_aggregator joint_limits_common)
add_ros_test(launch/unittest_joint_limits_aggregator.test.py ARGS
             "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

# JointLimitsContainer Unit Test
ament_add_gtest(unittest_joint_limits_container
                src/unittest_joint_limits_container.cpp)
target_link_libraries(unittest_joint_limits_container joint_limits_common)

# JointLimitsValidator Unit Test
ament_add_gtest(unittest_joint_limits_validator
                src/unittest_joint_limits_validator.cpp)
target_link_libraries(unittest_joint_limits_validator joint_limits_common)

# PlanningContextLoaderPTP Unit Test
ament_add_gtest_executable(unittest_planning_context_loaders
                           src/unittest_planning_context_loaders.cpp)
target_link_libraries(unittest_planning_context_loaders ${PROJECT_NAME}
                      ${PROJECT_NAME}_test_utils)
add_ros_test(launch/unittest_planning_context_loaders.test.py ARGS
             "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

# PlanningContext Unit Test (Typed test)
ament_add_gtest_executable(unittest_planning_context
                           src/unittest_planning_context.cpp)
target_link_libraries(
  unittest_planning_context ${PROJECT_NAME} ${PROJECT_NAME}_test_utils
  planning_context_loader_lin planning_context_loader_ptp
  planning_context_loader_circ)
add_ros_test(launch/unittest_planning_context.test.py ARGS
             "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

# GetTipFrames Unit Test
ament_add_gmock(unittest_get_solver_tip_frame
                src/unittest_get_solver_tip_frame.cpp)
target_link_libraries(unittest_get_solver_tip_frame ${PROJECT_NAME})

install(
  TARGETS ${UNIT_TEST_SOURCEFILES}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
